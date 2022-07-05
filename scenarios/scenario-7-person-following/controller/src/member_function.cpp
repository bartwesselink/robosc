#include <chrono>
#include <memory>
#include <thread>
#include <unistd.h>
#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <mutex>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "bboxes_ex_msgs/msg/bounding_boxes.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cinttypes>

extern "C" {
    #include "../include/controller/controller_engine.h"
}

using namespace std::chrono_literals;
#include <iostream>
#include <fstream>

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

controllerEnum code_Scanner_distance = _controller_person;
double code_YoloxDetection_current_image_size = 0.0;
double code_YoloxDetection_current_xmax = 0.0;
double code_YoloxDetection_current_xmin = 0.0;

class Controller : public rclcpp::Node {
public:	
	// Enum conversions
	controllerEnum convert_enum_Distance(const sensor_msgs::msg::LaserScan::SharedPtr input) {
		if (input->ranges[0] > 5.0 && input->ranges[350] > 5.0 && input->ranges[10] > 5.0) {
			return _controller_free;
		}
	
		return _controller_person;
	}

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_client_scan;
	rclcpp::Subscription<bboxes_ex_msgs::msg::BoundingBoxes>::SharedPtr subscriber_client_bounding_boxes;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_stop;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_continue;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_client_move;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_client_halt;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_information;
	std::vector<std::string> taken_transitions;

	Controller() : Node("controller") {
		subscriber_client_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Controller::callback_message_scan, this, std::placeholders::_1));
		subscriber_client_bounding_boxes = this->create_subscription<bboxes_ex_msgs::msg::BoundingBoxes>("/bounding_boxes", 10, std::bind(&Controller::callback_message_bounding_boxes, this, std::placeholders::_1));
		subscriber_client_stop = this->create_subscription<std_msgs::msg::Empty>("/stop", 10, std::bind(&Controller::callback_message_stop, this, std::placeholders::_1));
		subscriber_client_continue = this->create_subscription<std_msgs::msg::Empty>("/continue", 10, std::bind(&Controller::callback_message_continue, this, std::placeholders::_1));
		publisher_client_move = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
		publisher_client_halt = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

		state_information = this->create_publisher<std_msgs::msg::String>("/state", 10);
		timer = this->create_wall_timer(100ms, std::bind(&Controller::tick, this));
		controller_EngineFirstStep();
				
	}

	void callback_message_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
		message_scan_i_response_ = convert_enum_Distance(msg);
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_scan_u_response_);
								
	}
	
	
	void callback_message_bounding_boxes(const bboxes_ex_msgs::msg::BoundingBoxes::SharedPtr msg) {
		
		code_YoloxDetection_current_xmax = msg->bounding_boxes[0].xmax;
		
		code_YoloxDetection_current_xmin = msg->bounding_boxes[0].xmin;
		
		code_YoloxDetection_current_image_size = msg->bounding_boxes[0].img_width;
		
		// Call engine function
		controller_EnginePerformEvent(message_bounding_boxes_u_response_);
								
	}
	
	
	void callback_message_stop(const std_msgs::msg::Empty::SharedPtr msg) {
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_stop_u_response_);
								
	}
	
	
	void callback_message_continue(const std_msgs::msg::Empty::SharedPtr msg) {
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_continue_u_response_);
								
	}
	
	
	
	
	void call_message_move() {
		auto value = geometry_msgs::msg::Twist();
		
		if (data_move_ == _controller_data_p8X5ZPW7S5PZD) {
			value.linear.x = 0.0;
			value.angular.z = ((code_YoloxDetection_current_image_size / 2) - ((code_YoloxDetection_current_xmin + code_YoloxDetection_current_xmax) / 2)) / 1000;
		} else 
		if (data_move_ == _controller_data_pQ489YMH3IPPW) {
			value.linear.x = 0.2;
			value.angular.z = ((code_YoloxDetection_current_image_size / 2) - ((code_YoloxDetection_current_xmin + code_YoloxDetection_current_xmax) / 2)) / 1000;
		} else 
		if (data_move_ == _controller_data_p8OLNEMNMTTVZ) {
			value.linear.x = 0.0;
			value.angular.z = 0.3;
		}
		
		this->publisher_client_move->publish(value);
	
	}
	
	
	void call_message_halt() {
		auto value = geometry_msgs::msg::Twist();
		
		if (data_halt_ == _controller_data_pV6KEWXT2L92L) {
			value.linear.x = 0.0;
			value.angular.z = 0.0;
		}
		
		this->publisher_client_halt->publish(value);
	
	}
	
	void emit_current_state() {
		std::stringstream output;
		
		output << "{";
		
		output << "\"current\": {" << "";
		output << "\"Scanner\": {";
		output << "\"state\": \"" << "sensing""" << "\",";
		output << "\"variables\": {";
		
		output << "\"distance\": \"" << enum_names[component_Scanner_v_distance_] << "\"";				
		
		output << "}";
		output << "},";
		output << "\"YoloxDetection\": {";
		output << "\"state\": \"" << enum_names[component_YoloxDetection_] << "\",";
		output << "\"variables\": {";
		
		output << "\"current_image_size\": \"" << code_YoloxDetection_current_image_size << "\",";				
		output << "\"current_xmax\": \"" << code_YoloxDetection_current_xmax << "\",";				
		output << "\"current_xmin\": \"" << code_YoloxDetection_current_xmin << "\"";				
		
		output << "}";
		output << "},";
		output << "\"EmergencyStop\": {";
		output << "\"state\": \"" << enum_names[component_EmergencyStop_] << "\",";
		output << "\"variables\": {";
		
		
		output << "}";
		output << "}";
		output << "},";
		output << "\"transitions\": " << serialize_json_vector(taken_transitions) << ",";
		output << "\"definition\": " << "{\"name\":\"PersonFollowing\",\"components\":[{\"name\":\"Scanner\",\"messages\":[\"scan\"],\"services\":[],\"actions\":[],\"behaviour\":{\"variables\":[\"distance\"],\"states\":[{\"name\":\"sensing\",\"initial\":true,\"transitions\":[{\"next\":null,\"id\":\"message_scan_u_response_\",\"type\":\"response\",\"communication\":\"scan\"}]}]}},{\"name\":\"YoloxDetection\",\"messages\":[\"bounding_boxes\"],\"services\":[],\"actions\":[],\"behaviour\":{\"variables\":[\"current_image_size\",\"current_xmax\",\"current_xmin\"],\"states\":[{\"name\":\"initializing\",\"initial\":true,\"transitions\":[{\"next\":\"detected\",\"id\":\"message_bounding_boxes_u_response_\",\"type\":\"response\",\"communication\":\"bounding_boxes\"}]},{\"name\":\"detected\",\"initial\":false,\"transitions\":[{\"next\":\"detected\",\"id\":\"message_bounding_boxes_u_response_\",\"type\":\"response\",\"communication\":\"bounding_boxes\"}]}]}},{\"name\":\"EmergencyStop\",\"messages\":[\"stop\",\"continue\"],\"services\":[],\"actions\":[],\"behaviour\":{\"variables\":[],\"states\":[{\"name\":\"in_service\",\"initial\":true,\"transitions\":[{\"next\":\"stopped\",\"id\":\"message_stop_u_response_\",\"type\":\"response\",\"communication\":\"stop\"}]},{\"name\":\"stopped\",\"initial\":false,\"transitions\":[{\"next\":\"in_service\",\"id\":\"message_continue_u_response_\",\"type\":\"response\",\"communication\":\"continue\"}]}]}},{\"name\":\"TurtlebotPlatform\",\"messages\":[\"move\",\"halt\"],\"services\":[],\"actions\":[]}]}";
		output << "}";
		
		auto msg = std_msgs::msg::String();
		msg.data = output.str();
		
		this->state_information->publish(msg);
	
		taken_transitions.clear();
	}
	
	
	~Controller() {
	}
private:
	// Heart of the controller
	void tick() {
		int nOfDataEvents = 4;
		      controller_Event_ data_events[4] = { data_move_c_pGMPSSSIS1QV7_,data_move_c_pVDT3GHXV702K_,data_move_c_p4XGWRXEQ64QX_,data_halt_c_pRDQU5M38JY49_ };
		
		// Always execute data transitions that are possible
		shuffle_events(data_events, nOfDataEvents);
		
		for (int i = 0; i < nOfDataEvents; i++) {
			controller_EnginePerformEvent(data_events[i]);
		}
		
		int nOfControllableEvents = 2;
		      controller_Event_ controllable_events[2] = { message_move_c_trigger_,message_halt_c_trigger_ };
		
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

std::shared_ptr<Controller> node = nullptr;

// Control synthesis engine
bool assigned = false;

void controller_AssignInputVariables() {
	if (assigned) return;
	
	message_scan_i_response_ = _controller_person;
	
	assigned = true;
}
void controller_InfoEvent(controller_Event_ event, BoolType pre) {
    if (pre) {
    	node->taken_transitions.push_back(controller_event_names[event]);
    	return;
    }
    
    switch (event) {
case message_move_c_trigger_:
	node->call_message_move();
break;
case message_halt_c_trigger_:
	node->call_message_halt();
break;

    default:
		return;	
   	}
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    node = std::make_shared<Controller>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
