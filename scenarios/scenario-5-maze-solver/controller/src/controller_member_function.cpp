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
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
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

controllerEnum code_Distance_right = _controller_no_wall_right;
controllerEnum code_Distance_front = _controller_no_wall_front;
controllerEnum code_Distance_diag_right = _controller_no_wall_diag_right;

class Controller : public rclcpp::Node {
public:	
	// Enum conversions
	controllerEnum convert_enum_DistanceRight(const sensor_msgs::msg::LaserScan::SharedPtr input) {
		if (input->ranges[270] < 0.7 || input->ranges[240] < 0.7) {
			return _controller_wall_right;
		}
	
		return _controller_no_wall_right;
	}
	controllerEnum convert_enum_DistanceFront(const sensor_msgs::msg::LaserScan::SharedPtr input) {
		if (input->ranges[0] < 0.6) {
			return _controller_wall_front;
		}
	
		return _controller_no_wall_front;
	}
	controllerEnum convert_enum_DistanceDiagRight(const sensor_msgs::msg::LaserScan::SharedPtr input) {
		if (input->ranges[225] < 0.9) {
			return _controller_wall_diag_right;
		}
	
		return _controller_no_wall_diag_right;
	}

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_client_scan_right;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_client_scan_front;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_client_scan_diag_right;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_client_movement;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_client_halt;
	rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_client_turn_left;
	rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_client_turn_right;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_rotate_done;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_stop;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_continue;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_information;
	std::vector<std::string> taken_transitions;

	Controller() : Node("controller") {
		subscriber_client_scan_right = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Controller::callback_message_scan_right, this, std::placeholders::_1));
		subscriber_client_scan_front = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Controller::callback_message_scan_front, this, std::placeholders::_1));
		subscriber_client_scan_diag_right = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Controller::callback_message_scan_diag_right, this, std::placeholders::_1));
		publisher_client_movement = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
		publisher_client_halt = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
		publisher_client_turn_left = this->create_publisher<std_msgs::msg::Int16>("/rotate_left", 10);
		publisher_client_turn_right = this->create_publisher<std_msgs::msg::Int16>("/rotate_right", 10);
		subscriber_client_rotate_done = this->create_subscription<std_msgs::msg::Empty>("/rotate_done", 10, std::bind(&Controller::callback_message_rotate_done, this, std::placeholders::_1));
		subscriber_client_stop = this->create_subscription<std_msgs::msg::Empty>("/stop", 10, std::bind(&Controller::callback_message_stop, this, std::placeholders::_1));
		subscriber_client_continue = this->create_subscription<std_msgs::msg::Empty>("/continue", 10, std::bind(&Controller::callback_message_continue, this, std::placeholders::_1));

		state_information = this->create_publisher<std_msgs::msg::String>("/controller/state", 10);
		timer = this->create_wall_timer(100ms, std::bind(&Controller::tick, this));
		controller_EngineFirstStep();
	}

	void callback_message_scan_right(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
		message_scan_right_i_response_ = convert_enum_DistanceRight(msg);
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_scan_right_u_response_);
	}
	
	
	void callback_message_scan_front(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
		message_scan_front_i_response_ = convert_enum_DistanceFront(msg);
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_scan_front_u_response_);
	}
	
	
	void callback_message_scan_diag_right(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
		message_scan_diag_right_i_response_ = convert_enum_DistanceDiagRight(msg);
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_scan_diag_right_u_response_);
	}
	
	
	
	
	void call_message_movement() {
		auto value = geometry_msgs::msg::Twist();
		
		if (data_movement_ == _controller_data_pXK0J4DUTHLBO) {
			value.linear.x = 0.3;
		}
		
		this->publisher_client_movement->publish(value);
	}
	
	
	void call_message_halt() {
		auto value = geometry_msgs::msg::Twist();
		
		if (data_halt_ == _controller_data_pTK0WMX514XC7) {
			value.linear.x = 0.0;
		}
		
		this->publisher_client_halt->publish(value);
	}
	
	
	void call_message_turn_left() {
		auto value = std_msgs::msg::Int16();
		
		if (data_turn_left_ == _controller_data_pYGV3RPRPHSPV) {
			value.data = 90;
		}
		
		this->publisher_client_turn_left->publish(value);
	}
	
	
	void call_message_turn_right() {
		auto value = std_msgs::msg::Int16();
		
		if (data_turn_right_ == _controller_data_p97PUYJKDV3WD) {
			value.data = 90;
		}
		
		this->publisher_client_turn_right->publish(value);
	}
	void callback_message_rotate_done(const std_msgs::msg::Empty::SharedPtr msg) {
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_rotate_done_u_response_);
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
		output << "\"Distance\": {";
		output << "\"state\": \"" << "sensing""" << "\",";
		output << "\"variables\": {";
		
		output << "\"right\": \"" << enum_names[component_Distance_v_right_] << "\",";				
		output << "\"front\": \"" << enum_names[component_Distance_v_front_] << "\",";				
		output << "\"diag_right\": \"" << enum_names[component_Distance_v_diag_right_] << "\"";				
		
		output << "}";
		output << "},";
		output << "\"Platform\": {";
		output << "\"state\": \"" << enum_names[component_Platform_] << "\",";
		output << "\"variables\": {";
		
		
		output << "}";
		output << "},";
		output << "\"EmergencyStop\": {";
		output << "\"state\": \"" << enum_names[component_EmergencyStop_] << "\",";
		output << "\"variables\": {";
		
		
		output << "}";
		output << "}";
		output << "},";
		output << "\"transitions\": " << serialize_json_vector(taken_transitions) << ",";
		output << "\"definition\": " << "{\"name\":\"MazeSolver\",\"components\":[{\"name\":\"Distance\",\"messages\":[\"scan_right\",\"scan_front\",\"scan_diag_right\"],\"services\":[],\"actions\":[],\"behaviour\":{\"variables\":[\"right\",\"front\",\"diag_right\"],\"states\":[{\"name\":\"sensing\",\"initial\":true,\"transitions\":[{\"next\":null,\"id\":\"message_scan_right_u_response_\",\"type\":\"response\",\"communication\":\"scan_right\"},{\"next\":null,\"id\":\"message_scan_front_u_response_\",\"type\":\"response\",\"communication\":\"scan_front\"},{\"next\":null,\"id\":\"message_scan_diag_right_u_response_\",\"type\":\"response\",\"communication\":\"scan_diag_right\"}]}]}},{\"name\":\"Platform\",\"messages\":[\"movement\",\"halt\",\"turn_left\",\"turn_right\",\"rotate_done\"],\"services\":[],\"actions\":[],\"behaviour\":{\"variables\":[],\"states\":[{\"name\":\"ready\",\"initial\":true,\"transitions\":[{\"next\":\"turning\",\"id\":\"message_turn_left_c_trigger_\",\"type\":\"request\",\"communication\":\"turn_left\"},{\"next\":\"turning\",\"id\":\"message_turn_right_c_trigger_\",\"type\":\"request\",\"communication\":\"turn_right\"}]},{\"name\":\"turning\",\"initial\":false,\"transitions\":[{\"next\":\"ready\",\"id\":\"message_rotate_done_u_response_\",\"type\":\"response\",\"communication\":\"rotate_done\"},{\"next\":\"ready\",\"id\":\"message_stop_u_response_\",\"type\":\"response\",\"communication\":\"stop\"}]}]}},{\"name\":\"EmergencyStop\",\"messages\":[\"stop\",\"continue\"],\"services\":[],\"actions\":[],\"behaviour\":{\"variables\":[],\"states\":[{\"name\":\"in_service\",\"initial\":true,\"transitions\":[{\"next\":\"stopped\",\"id\":\"message_stop_u_response_\",\"type\":\"response\",\"communication\":\"stop\"}]},{\"name\":\"stopped\",\"initial\":false,\"transitions\":[{\"next\":\"in_service\",\"id\":\"message_continue_u_response_\",\"type\":\"response\",\"communication\":\"continue\"}]}]}}]}";
		output << "}";
		
		auto msg = std_msgs::msg::String();
		msg.data = output.str();
		
		this->state_information->publish(msg);
	
		taken_transitions.clear();
	}
private:
	// Heart of the controller
	void tick() {
		int nOfDataEvents = 4;
		      controller_Event_ data_events[4] = { data_movement_c_pFHLDWOIYH8GF_,data_halt_c_pUUGM20RFL8GN_,data_turn_left_c_pM4I19LT4GL6W_,data_turn_right_c_pBBPJFOU0CVVQ_ };
		
		// Always execute data transitions that are possible
		shuffle_events(data_events, nOfDataEvents);
		
		for (int i = 0; i < nOfDataEvents; i++) {
			controller_EnginePerformEvent(data_events[i]);
		}
		
		int nOfControllableEvents = 4;
		      controller_Event_ controllable_events[4] = { message_movement_c_trigger_,message_halt_c_trigger_,message_turn_left_c_trigger_,message_turn_right_c_trigger_ };
		
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
	
	message_scan_right_i_response_ = _controller_no_wall_right;
	message_scan_front_i_response_ = _controller_no_wall_front;
	message_scan_diag_right_i_response_ = _controller_no_wall_diag_right;
	
	assigned = true;
}
void controller_InfoEvent(controller_Event_ event, BoolType pre) {
    if (pre) {
    	node_controller->taken_transitions.push_back(controller_event_names[event]);
    	return;
    }
    
    switch (event) {
case message_movement_c_trigger_:
	node_controller->call_message_movement();
break;
case message_halt_c_trigger_:
	node_controller->call_message_halt();
break;
case message_turn_left_c_trigger_:
	node_controller->call_message_turn_left();
break;
case message_turn_right_c_trigger_:
	node_controller->call_message_turn_right();
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
