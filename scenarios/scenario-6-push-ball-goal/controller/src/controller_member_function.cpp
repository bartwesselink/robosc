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
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist.hpp"
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

controllerEnum code_Scanner_distance = _controller_free;
double code_BallDetector_current_correction = 0.0;
double code_GoalDetector_current_correction = 0.0;

class Controller : public rclcpp::Node {
public:	
	// Enum conversions
	controllerEnum convert_enum_Distance(const sensor_msgs::msg::LaserScan::SharedPtr input) {
		if (input->ranges[40] < 1.0 && input->ranges[320] < 1.0) {
			return _controller_obstructed;
		}
	
		return _controller_free;
	}

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_client_scan;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_client_ball_correction;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_no_ball;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_needs_ajustment;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_no_adjustment;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_ball_front_check;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_client_goal_correction;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_no_goal;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_stop;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_continue;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_client_move;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_client_halt;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_information;
	std::vector<std::string> taken_transitions;

	Controller() : Node("controller") {
		subscriber_client_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Controller::callback_message_scan, this, std::placeholders::_1));
		subscriber_client_ball_correction = this->create_subscription<std_msgs::msg::Float32>("/ball_correction", 10, std::bind(&Controller::callback_message_ball_correction, this, std::placeholders::_1));
		subscriber_client_no_ball = this->create_subscription<std_msgs::msg::Empty>("/no_ball", 10, std::bind(&Controller::callback_message_no_ball, this, std::placeholders::_1));
		subscriber_client_needs_ajustment = this->create_subscription<std_msgs::msg::Empty>("/needs_ajustment", 10, std::bind(&Controller::callback_message_needs_ajustment, this, std::placeholders::_1));
		subscriber_client_no_adjustment = this->create_subscription<std_msgs::msg::Empty>("/no_adjustment", 10, std::bind(&Controller::callback_message_no_adjustment, this, std::placeholders::_1));
		subscriber_client_ball_front_check = this->create_subscription<std_msgs::msg::Empty>("/ball_front_check", 10, std::bind(&Controller::callback_message_ball_front_check, this, std::placeholders::_1));
		subscriber_client_goal_correction = this->create_subscription<std_msgs::msg::Float32>("/goal_correction", 10, std::bind(&Controller::callback_message_goal_correction, this, std::placeholders::_1));
		subscriber_client_no_goal = this->create_subscription<std_msgs::msg::Empty>("/no_goal", 10, std::bind(&Controller::callback_message_no_goal, this, std::placeholders::_1));
		subscriber_client_stop = this->create_subscription<std_msgs::msg::Empty>("/stop", 10, std::bind(&Controller::callback_message_stop, this, std::placeholders::_1));
		subscriber_client_continue = this->create_subscription<std_msgs::msg::Empty>("/continue", 10, std::bind(&Controller::callback_message_continue, this, std::placeholders::_1));
		publisher_client_move = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
		publisher_client_halt = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

		state_information = this->create_publisher<std_msgs::msg::String>("/controller/state", 10);
		timer = this->create_wall_timer(100ms, std::bind(&Controller::tick, this));
		controller_EngineFirstStep();
	}

	void callback_message_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
		message_scan_i_response_ = convert_enum_Distance(msg);
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_scan_u_response_);
	}
	
	
	void callback_message_ball_correction(const std_msgs::msg::Float32::SharedPtr msg) {
		
		code_BallDetector_current_correction = msg->data;
		code_BallDetector_current_correction = msg->data;
		
		
		// Call engine function
		controller_EnginePerformEvent(message_ball_correction_u_response_);
	}
	
	
	void callback_message_no_ball(const std_msgs::msg::Empty::SharedPtr msg) {
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_no_ball_u_response_);
	}
	
	
	void callback_message_needs_ajustment(const std_msgs::msg::Empty::SharedPtr msg) {
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_needs_ajustment_u_response_);
	}
	
	
	void callback_message_no_adjustment(const std_msgs::msg::Empty::SharedPtr msg) {
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_no_adjustment_u_response_);
	}
	
	
	void callback_message_ball_front_check(const std_msgs::msg::Empty::SharedPtr msg) {
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_ball_front_check_u_response_);
	}
	
	
	void callback_message_goal_correction(const std_msgs::msg::Float32::SharedPtr msg) {
		
		code_GoalDetector_current_correction = msg->data;
		
		
		// Call engine function
		controller_EnginePerformEvent(message_goal_correction_u_response_);
	}
	
	
	void callback_message_no_goal(const std_msgs::msg::Empty::SharedPtr msg) {
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_no_goal_u_response_);
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
		
		if (data_move_ == _controller_data_p38VGI0MCWISH) {
			value.linear.x = 0.2;
			value.angular.z = (0.0 - code_BallDetector_current_correction) / 500;
		} else 
		if (data_move_ == _controller_data_p33DNBQLXQB4P) {
			value.linear.x = 0.2;
			value.angular.z = (code_BallDetector_current_correction) / 1000;
		} else 
		if (data_move_ == _controller_data_p6WT7KC41PY1E) {
			value.linear.x = 0.2;
			value.angular.z = (0.0 - code_GoalDetector_current_correction) / 1000;
		} else 
		if (data_move_ == _controller_data_pBDGDOP7H447Z) {
			value.angular.z = 0.5;
		}
		
		this->publisher_client_move->publish(value);
	}
	
	
	void call_message_halt() {
		auto value = geometry_msgs::msg::Twist();
		
		if (data_halt_ == _controller_data_pBWO8PZU9GV4T) {
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
		output << "\"BallDetector\": {";
		output << "\"state\": \"" << enum_names[component_BallDetector_] << "\",";
		output << "\"variables\": {";
		
		output << "\"current_correction\": \"" << code_BallDetector_current_correction << "\"";				
		
		output << "}";
		output << "},";
		output << "\"GoalDetector\": {";
		output << "\"state\": \"" << enum_names[component_GoalDetector_] << "\",";
		output << "\"variables\": {";
		
		output << "\"current_correction\": \"" << code_GoalDetector_current_correction << "\"";				
		
		output << "}";
		output << "},";
		output << "\"EmergencyStop\": {";
		output << "\"state\": \"" << enum_names[component_EmergencyStop_] << "\",";
		output << "\"variables\": {";
		
		
		output << "}";
		output << "}";
		output << "},";
		output << "\"transitions\": " << serialize_json_vector(taken_transitions) << ",";
		output << "\"definition\": " << "{\"name\":\"PushBallGoal\",\"components\":[{\"name\":\"Scanner\",\"messages\":[\"scan\"],\"services\":[],\"actions\":[],\"behaviour\":{\"variables\":[\"distance\"],\"states\":[{\"name\":\"sensing\",\"initial\":true,\"transitions\":[{\"next\":null,\"id\":\"message_scan_u_response_\",\"type\":\"response\",\"communication\":\"scan\"}]}]}},{\"name\":\"BallDetector\",\"messages\":[\"ball_correction\",\"no_ball\",\"needs_ajustment\",\"no_adjustment\",\"ball_front_check\"],\"services\":[],\"actions\":[],\"behaviour\":{\"variables\":[\"current_correction\"],\"states\":[{\"name\":\"awaiting\",\"initial\":true,\"transitions\":[{\"next\":\"no_ball\",\"id\":\"message_no_ball_u_response_\",\"type\":\"response\",\"communication\":\"no_ball\"},{\"next\":\"ball_found\",\"id\":\"message_ball_correction_u_response_\",\"type\":\"response\",\"communication\":\"ball_correction\"}]},{\"name\":\"ball_found\",\"initial\":false,\"transitions\":[{\"next\":\"no_ball\",\"id\":\"message_no_ball_u_response_\",\"type\":\"response\",\"communication\":\"no_ball\"},{\"next\":\"ball_found\",\"id\":\"message_ball_correction_u_response_\",\"type\":\"response\",\"communication\":\"ball_correction\"},{\"next\":\"ball_in_front\",\"id\":\"message_ball_front_check_u_response_\",\"type\":\"response\",\"communication\":\"ball_front_check\"}]},{\"name\":\"ball_in_front\",\"initial\":false,\"transitions\":[{\"next\":\"adjusting\",\"id\":\"message_needs_ajustment_u_response_\",\"type\":\"response\",\"communication\":\"needs_ajustment\"}]},{\"name\":\"adjusting\",\"initial\":false,\"transitions\":[{\"next\":\"ball_in_front\",\"id\":\"message_no_adjustment_u_response_\",\"type\":\"response\",\"communication\":\"no_adjustment\"}]},{\"name\":\"no_ball\",\"initial\":false,\"transitions\":[{\"next\":\"no_ball\",\"id\":\"message_no_ball_u_response_\",\"type\":\"response\",\"communication\":\"no_ball\"},{\"next\":\"ball_found\",\"id\":\"message_ball_correction_u_response_\",\"type\":\"response\",\"communication\":\"ball_correction\"}]}]}},{\"name\":\"GoalDetector\",\"messages\":[\"goal_correction\",\"no_goal\"],\"services\":[],\"actions\":[],\"behaviour\":{\"variables\":[\"current_correction\"],\"states\":[{\"name\":\"awaiting\",\"initial\":true,\"transitions\":[{\"next\":\"no_goal\",\"id\":\"message_no_goal_u_response_\",\"type\":\"response\",\"communication\":\"no_goal\"},{\"next\":\"goal_found\",\"id\":\"message_goal_correction_u_response_\",\"type\":\"response\",\"communication\":\"goal_correction\"}]},{\"name\":\"goal_found\",\"initial\":false,\"transitions\":[{\"next\":\"no_goal\",\"id\":\"message_no_goal_u_response_\",\"type\":\"response\",\"communication\":\"no_goal\"},{\"next\":null,\"id\":\"message_goal_correction_u_response_\",\"type\":\"response\",\"communication\":\"goal_correction\"}]},{\"name\":\"no_goal\",\"initial\":false,\"transitions\":[{\"next\":\"goal_found\",\"id\":\"message_goal_correction_u_response_\",\"type\":\"response\",\"communication\":\"goal_correction\"}]}]}},{\"name\":\"EmergencyStop\",\"messages\":[\"stop\",\"continue\"],\"services\":[],\"actions\":[],\"behaviour\":{\"variables\":[],\"states\":[{\"name\":\"in_service\",\"initial\":true,\"transitions\":[{\"next\":\"stopped\",\"id\":\"message_stop_u_response_\",\"type\":\"response\",\"communication\":\"stop\"}]},{\"name\":\"stopped\",\"initial\":false,\"transitions\":[{\"next\":\"in_service\",\"id\":\"message_continue_u_response_\",\"type\":\"response\",\"communication\":\"continue\"}]}]}},{\"name\":\"TurtlebotPlatform\",\"messages\":[\"move\",\"halt\"],\"services\":[],\"actions\":[]}]}";
		output << "}";
		
		auto msg = std_msgs::msg::String();
		msg.data = output.str();
		
		this->state_information->publish(msg);
	
		taken_transitions.clear();
	}
private:
	// Heart of the controller
	void tick() {
		int nOfDataEvents = 5;
		      controller_Event_ data_events[5] = { data_move_c_pWP9S3X2IRT4E_,data_move_c_p0P5R0EMP81XR_,data_move_c_p2DMZIEAP78KO_,data_move_c_p0WOLQRISXTHD_,data_halt_c_pLTW6HAKO6A8D_ };
		
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

std::shared_ptr<Controller> node_controller = nullptr;

// Control synthesis engine
bool assigned = false;

void controller_AssignInputVariables() {
	if (assigned) return;
	
	message_scan_i_response_ = _controller_free;
	
	assigned = true;
}
void controller_InfoEvent(controller_Event_ event, BoolType pre) {
    if (pre) {
    	node_controller->taken_transitions.push_back(controller_event_names[event]);
    	return;
    }
    
    switch (event) {
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
