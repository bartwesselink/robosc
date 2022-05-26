#include <chrono>
#include <memory>
#include <thread>
#include <unistd.h>
#include <functional>
#include <future>
#include <memory>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
#include <cinttypes>

extern "C" {
    #include "../include/controller/controller_engine.h"
}

using namespace std::chrono_literals;

// Utility functions
void shuffle_events(controller_Event_ *x, size_t n)
{
    if (n > 1) {
        srand(time(NULL));

        for (unsigned i = 0; i < n-1; ++i)
        {
            unsigned j = rand() % (n-i) + i;
            controller_Event_ temp = x[i];
            x[i] = x[j];
            x[j] = temp;
        }
    }
}

std::string serialize_json_vector(const std::vector<std::string>& list) {
    std::stringstream output;
    output << "[";
    
    bool first = true;

    for (const auto& value : list) {
        if (!first) {
            output << ", ";
        }
        
        first = false;

        output << "\"" << value << "\"";
    }
    
    output << "]";   
    
    return output.str();
}

controllerEnum code_LidarScanner_front = _controller_unsafe_front;
controllerEnum code_LidarScanner_left = _controller_unsafe_left;
controllerEnum code_LidarScanner_right = _controller_unsafe_right;
bool code_LidarScanner_has_front = false;
int code_ObjectDetector_scanned_object_count = 0;
controllerEnum code_ObjectDetector_scanned_object = _controller_no_object;

class Controller : public rclcpp::Node {
public:	
	// Enum conversions
	controllerEnum convert_enum_DistanceFront(const sensor_msgs::msg::LaserScan::SharedPtr input) {
		if (input->ranges[0] >= 0.6) {
			return _controller_safe_front;
		}
	
		return _controller_unsafe_front;
	}
	controllerEnum convert_enum_DistanceLeft(const sensor_msgs::msg::LaserScan::SharedPtr input) {
		if (input->ranges[90] >= 0.6) {
			return _controller_safe_left;
		}
	
		return _controller_unsafe_left;
	}
	controllerEnum convert_enum_DistanceRight(const sensor_msgs::msg::LaserScan::SharedPtr input) {
		if (input->ranges[270] >= 0.6) {
			return _controller_safe_right;
		}
	
		return _controller_unsafe_right;
	}
	controllerEnum convert_enum_ScannedObject(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr input) {
		if (input->bounding_boxes[0].class_id == "sfront sign") {
			return _controller_sfront_sign;
		}
	
		return _controller_no_object;
	}

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_client_scan_front;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_client_scan_left;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_client_scan_right;
	rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_client_rotate_left;
	rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_client_rotate_right;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_rotate_done;
	rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr subscriber_client_object_count;
	rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr subscriber_client_object_scan;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_client_move;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_client_halt;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_stop;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_continue;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_information;
	std::vector<std::string> taken_transitions;

	Controller() : Node("controller") {
		subscriber_client_scan_front = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Controller::callback_message_scan_front, this, std::placeholders::_1));
		subscriber_client_scan_left = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Controller::callback_message_scan_left, this, std::placeholders::_1));
		subscriber_client_scan_right = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Controller::callback_message_scan_right, this, std::placeholders::_1));
		publisher_client_rotate_left = this->create_publisher<std_msgs::msg::Empty>("/rotate_left", 10);
		publisher_client_rotate_right = this->create_publisher<std_msgs::msg::Empty>("/rotate_right", 10);
		subscriber_client_rotate_done = this->create_subscription<std_msgs::msg::Empty>("/rotate_done", 10, std::bind(&Controller::callback_message_rotate_done, this, std::placeholders::_1));
		subscriber_client_object_count = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>("/darknet_ros/found_object", 10, std::bind(&Controller::callback_message_object_count, this, std::placeholders::_1));
		subscriber_client_object_scan = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>("/darknet_ros/bounding_boxes", 10, std::bind(&Controller::callback_message_object_scan, this, std::placeholders::_1));
		publisher_client_move = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
		publisher_client_halt = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
		subscriber_client_stop = this->create_subscription<std_msgs::msg::Empty>("/stop", 10, std::bind(&Controller::callback_message_stop, this, std::placeholders::_1));
		subscriber_client_continue = this->create_subscription<std_msgs::msg::Empty>("/continue", 10, std::bind(&Controller::callback_message_continue, this, std::placeholders::_1));

		state_information = this->create_publisher<std_msgs::msg::String>("/controller/state", 10);
		timer = this->create_wall_timer(100ms, std::bind(&Controller::tick, this));
		controller_EngineFirstStep();
	}

	void callback_message_scan_front(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
		message_scan_front_i_response_ = convert_enum_DistanceFront(msg);
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_scan_front_u_response_);
	}
	
	
	void callback_message_scan_left(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
		message_scan_left_i_response_ = convert_enum_DistanceLeft(msg);
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_scan_left_u_response_);
	}
	
	
	void callback_message_scan_right(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
		message_scan_right_i_response_ = convert_enum_DistanceRight(msg);
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_scan_right_u_response_);
	}
	
	
	
	
	void call_message_rotate_left() {
		auto value = std_msgs::msg::Empty();
		
		
		this->publisher_client_rotate_left->publish(value);
	}
	
	
	void call_message_rotate_right() {
		auto value = std_msgs::msg::Empty();
		
		
		this->publisher_client_rotate_right->publish(value);
	}
	void callback_message_rotate_done(const std_msgs::msg::Empty::SharedPtr msg) {
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_rotate_done_u_response_);
	}
	
	
	void callback_message_object_count(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg) {
		message_object_count_i_response_count_ = msg->;
		
		
		// Call engine function
		controller_EnginePerformEvent(message_object_count_u_response_);
	}
	
	
	void callback_message_object_scan(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg) {
		message_object_scan_i_response_ = convert_enum_ScannedObject(msg);
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_object_scan_u_response_);
	}
	
	
	
	
	void call_message_move() {
		auto value = geometry_msgs::msg::Twist();
		
		if (data_move_ == _controller_data_p9SPNVSKRJB1V) {
			value.linear.x = 0.5;
			value.angular.z = 0.0;
		}
		
		this->publisher_client_move->publish(value);
	}
	
	
	void call_message_halt() {
		auto value = geometry_msgs::msg::Twist();
		
		if (data_halt_ == _controller_data_pUJAC2EE91VKU) {
			value.linear.x = 0.0;
			value.angular.z = 0.0;
		}
		
		this->publisher_client_halt->publish(value);
	}
	void callback_message_stop(const std_msgs::msg::Empty::SharedPtr msg) {
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_stop_u_response_);
	}
	
	
	void callback_message_continue(const std_msgs::msg::Empty::SharedPtr msg) {
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_continue_u_response_);
	}
	
	
	
	void emit_current_state() {
		std::stringstream output;
		
		output << "{";
		
		output << "\"current\": {" << "";
		output << "\"LidarScanner\": {";
		output << "\"state\": \"" << "sensing""" << "\",";
		output << "\"variables\": {";
		
		output << "\"front\": \"" << enum_names[component_LidarScanner_v_front_] << "\",";				
		output << "\"left\": \"" << enum_names[component_LidarScanner_v_left_] << "\",";				
		output << "\"right\": \"" << enum_names[component_LidarScanner_v_right_] << "\",";				
		output << "\"has_front\": \"" << component_LidarScanner_v_has_front_ << "\"";				
		
		output << "}";
		output << "},";
		output << "\"Rotator\": {";
		output << "\"state\": \"" << enum_names[component_Rotator_] << "\",";
		output << "\"variables\": {";
		
		
		output << "}";
		output << "},";
		output << "\"ObjectDetector\": {";
		output << "\"state\": \"" << enum_names[component_ObjectDetector_] << "\",";
		output << "\"variables\": {";
		
		output << "\"scanned_object_count\": \"" << component_ObjectDetector_v_scanned_object_count_ << "\",";				
		output << "\"scanned_object\": \"" << enum_names[component_ObjectDetector_v_scanned_object_] << "\"";				
		
		output << "}";
		output << "},";
		output << "\"EmergencyStop\": {";
		output << "\"state\": \"" << enum_names[component_EmergencyStop_] << "\",";
		output << "\"variables\": {";
		
		
		output << "}";
		output << "}";
		output << "},";
		output << "\"transitions\": " << serialize_json_vector(taken_transitions) << ",";
		output << "\"definition\": " << "{\"name\":\"ObjectFinder\",\"components\":[{\"name\":\"LidarScanner\",\"messages\":[\"scan_front\",\"scan_left\",\"scan_right\"],\"services\":[],\"actions\":[],\"behaviour\":{\"variables\":[\"front\",\"left\",\"right\",\"has_front\"],\"states\":[{\"name\":\"sensing\",\"initial\":true,\"transitions\":[{\"next\":null,\"id\":\"message_scan_front_u_response_\",\"type\":\"response\",\"communication\":\"scan_front\"},{\"next\":null,\"id\":\"message_scan_left_u_response_\",\"type\":\"response\",\"communication\":\"scan_left\"},{\"next\":null,\"id\":\"message_scan_right_u_response_\",\"type\":\"response\",\"communication\":\"scan_right\"}]}]}},{\"name\":\"Rotator\",\"messages\":[\"rotate_left\",\"rotate_right\",\"rotate_done\"],\"services\":[],\"actions\":[],\"behaviour\":{\"variables\":[],\"states\":[{\"name\":\"awaiting_command\",\"initial\":true,\"transitions\":[{\"next\":\"executing\",\"id\":\"message_rotate_left_c_trigger_\",\"type\":\"request\",\"communication\":\"rotate_left\"},{\"next\":\"executing\",\"id\":\"message_rotate_right_c_trigger_\",\"type\":\"request\",\"communication\":\"rotate_right\"}]},{\"name\":\"executing\",\"initial\":false,\"transitions\":[{\"next\":\"awaiting_command\",\"id\":\"message_rotate_done_u_response_\",\"type\":\"response\",\"communication\":\"rotate_done\"}]}]}},{\"name\":\"ObjectDetector\",\"messages\":[\"object_count\",\"object_scan\"],\"services\":[],\"actions\":[],\"behaviour\":{\"variables\":[\"scanned_object_count\",\"scanned_object\"],\"states\":[{\"name\":\"no_object\",\"initial\":true,\"transitions\":[{\"next\":\"object_found\",\"id\":\"component_ObjectDetector_c_p22L9O1Y5OJS3_\",\"type\":\"tau\"},{\"next\":null,\"id\":\"message_object_count_u_response_\",\"type\":\"response\",\"communication\":\"object_count\"},{\"next\":null,\"id\":\"message_object_scan_u_response_\",\"type\":\"response\",\"communication\":\"object_scan\"}]},{\"name\":\"object_found\",\"initial\":false,\"transitions\":[{\"next\":\"no_object\",\"id\":\"component_ObjectDetector_c_p8ZQITWII26U0_\",\"type\":\"tau\"},{\"next\":null,\"id\":\"message_object_count_u_response_\",\"type\":\"response\",\"communication\":\"object_count\"},{\"next\":null,\"id\":\"message_object_scan_u_response_\",\"type\":\"response\",\"communication\":\"object_scan\"}]}]}},{\"name\":\"TurtlebotPlatfrom\",\"messages\":[\"move\",\"halt\"],\"services\":[],\"actions\":[]},{\"name\":\"EmergencyStop\",\"messages\":[\"stop\",\"continue\"],\"services\":[],\"actions\":[],\"behaviour\":{\"variables\":[],\"states\":[{\"name\":\"in_service\",\"initial\":true,\"transitions\":[{\"next\":\"stopped\",\"id\":\"message_stop_u_response_\",\"type\":\"response\",\"communication\":\"stop\"}]},{\"name\":\"stopped\",\"initial\":false,\"transitions\":[{\"next\":\"in_service\",\"id\":\"message_continue_u_response_\",\"type\":\"response\",\"communication\":\"continue\"}]}]}}]}";
		output << "}";
		
		auto msg = std_msgs::msg::String();
		msg.data = output.str();
		
		this->state_information->publish(msg);
	
		taken_transitions.clear();
	}
private:
	// Heart of the controller
	void tick() {
		int nOfDataEvents = 2;
		      controller_Event_ data_events[2] = { data_move_c_pCMXK69FK33BV_,data_halt_c_p0NJ8JZ17CN5G_ };
		
		// Always execute data transitions that are possible
		shuffle_events(data_events, nOfDataEvents);
		
		for (int i = 0; i < nOfDataEvents; i++) {
			controller_EnginePerformEvent(data_events[i]);
		}
		
		int nOfControllableEvents = 6;
		      controller_Event_ controllable_events[6] = { component_ObjectDetector_c_p22L9O1Y5OJS3_,component_ObjectDetector_c_p8ZQITWII26U0_,message_rotate_left_c_trigger_,message_rotate_right_c_trigger_,message_move_c_trigger_,message_halt_c_trigger_ };
		
		shuffle_events(controllable_events, nOfControllableEvents);
		
		for (int i = 0; i < nOfControllableEvents; i++) {
			if (controller_EnginePerformEvent(controllable_events[i])) {
				break;
			}
		}

		this->emit_current_state();
	}
	
	rclcpp::TimerBase::SharedPtr timer;
};

std::shared_ptr<Controller> node_controller = nullptr;

// Control synthesis engine
bool assigned = false;

void controller_AssignInputVariables() {
	if (assigned) return;
	
	message_scan_front_i_response_ = _controller_unsafe_front;
	message_scan_left_i_response_ = _controller_unsafe_left;
	message_scan_right_i_response_ = _controller_unsafe_right;
	message_object_count_i_response_count_ = 0;
	message_object_scan_i_response_ = _controller_no_object;
	
	assigned = true;
}
void controller_InfoEvent(controller_Event_ event, BoolType pre) {
    if (pre) {
    	node_controller->taken_transitions.push_back(controller_event_names[event]);
    	return;
    }
    
    switch (event) {
case message_rotate_left_c_trigger_:
	node_controller->call_message_rotate_left();
break;
case message_rotate_right_c_trigger_:
	node_controller->call_message_rotate_right();
break;
case message_move_c_trigger_:
	node_controller->call_message_move();
break;
case message_halt_c_trigger_:
	node_controller->call_message_halt();
break;

    default:
		return;	
   	}
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    node_controller = std::make_shared<Controller>();

    rclcpp::spin(node_controller);
    rclcpp::shutdown();

    return 0;
}
