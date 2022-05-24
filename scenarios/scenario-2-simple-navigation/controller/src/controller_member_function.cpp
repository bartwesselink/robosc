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
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msg/action/navigate_to_pose.hpp"
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

double code_Nav2_current_x = 0.0;
double code_Nav2_current_y = 0.0;
double code_Nav2_current_z = 0.0;

class Controller : public rclcpp::Node {
public:	
	// Enum conversions

	rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_client_point;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriber_client_initial_pose;
	rclcpp_action::Client<nav2_msg::action::NavigateToPose>::SharedPtr action_client_navigate;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_stop;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_continue;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_information;
	std::vector<std::string> taken_transitions;

	Controller() : Node("controller") {
		subscriber_client_point = this->create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 10, std::bind(&Controller::callback_message_point, this, std::placeholders::_1));
		subscriber_client_initial_pose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10, std::bind(&Controller::callback_message_initial_pose, this, std::placeholders::_1));
		action_client_navigate = rclcpp_action::create_client<nav2_msg::action::NavigateToPose>(this, "/navigate_to_pose");
		subscriber_client_stop = this->create_subscription<std_msgs::msg::Empty>("/stop", 10, std::bind(&Controller::callback_message_stop, this, std::placeholders::_1));
		subscriber_client_continue = this->create_subscription<std_msgs::msg::Empty>("/continue", 10, std::bind(&Controller::callback_message_continue, this, std::placeholders::_1));

		state_information = this->create_publisher<std_msgs::msg::String>("/controller/state", 10);
		timer = this->create_wall_timer(100ms, std::bind(&Controller::tick, this));
		controller_EngineFirstStep();
	}

	void callback_message_point(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
		
		code_Nav2_current_x = msg->point.x;
		
		code_Nav2_current_y = msg->point.y;
		
		code_Nav2_current_z = msg->point.z;
		
		// Call engine function
		controller_EnginePerformEvent(message_point_u_response_);
	}
	
	
	void callback_message_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
		
		
		
		
		
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_initial_pose_u_response_);
	}
	
	
	void response_action_navigate(const rclcpp_action::ClientGoalHandle<nav2_msg::action::NavigateToPose>::WrappedResult & result) {
		
		
	
		fprintf(stderr, "[debug] Received action response\n");
		
		// Call engine function
		controller_EnginePerformEvent(action_navigate_u_response_);
	}
	
	void feedback_action_navigate(rclcpp_action::ClientGoalHandle<nav2_msg::action::NavigateToPose>::SharedPtr, const std::shared_ptr<const nav2_msg::action::NavigateToPose::Feedback> feedback) {
		
		
	
		fprintf(stderr, "[debug] Received action feedback\n");
		
		// Call engine function
		controller_EnginePerformEvent(action_navigate_u_feedback_);
	}
	
	void call_action_navigate() {
		if (!this->action_client_navigate->wait_for_action_server(1s)) {
			controller_EnginePerformEvent(action_navigate_u_error_);
			return;
		}
		auto goal_msg = nav2_msg::action::NavigateToPose::Goal();
	
		if (data_navigate_ == _controller_data_pBFUU6GBMSVGP) {
			goal_msg.pose.pose.position.x = code_Nav2_current_x;
			goal_msg.pose.pose.position.y = code_Nav2_current_y;
			goal_msg.pose.pose.position.z = code_Nav2_current_z;
		}
		
		auto send_options = rclcpp_action::Client<nav2_msg::action::NavigateToPose>::SendGoalOptions();
		send_options.result_callback = std::bind(&Controller::response_action_navigate, this, std::placeholders::_1);
		send_options.feedback_callback = std::bind(&Controller::feedback_action_navigate, this, std::placeholders::_1, std::placeholders::_2);
		this->action_client_navigate->async_send_goal(goal_msg, send_options);
	}
		
	void cancel_action_navigate() {
		this->action_client_navigate->async_cancel_all_goals();
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
		output << "\"Nav2\": {";
		output << "\"state\": \"" << enum_names[component_Nav2_] << "\",";
		output << "\"variables\": {";
		
		output << "\"current_x\": \"" << code_Nav2_current_x << "\",";				
		output << "\"current_y\": \"" << code_Nav2_current_y << "\",";				
		output << "\"current_z\": \"" << code_Nav2_current_z << "\"";				
		
		output << "}";
		output << "},";
		output << "\"EmergencyStop\": {";
		output << "\"state\": \"" << enum_names[component_EmergencyStop_] << "\",";
		output << "\"variables\": {";
		
		
		output << "}";
		output << "}";
		output << "},";
		output << "\"transitions\": " << serialize_json_vector(taken_transitions) << ",";
		output << "\"definition\": " << "{\"name\":\"SimpleNavigation\",\"components\":[{\"name\":\"Nav2\",\"messages\":[\"point\",\"initial_pose\"],\"services\":[],\"actions\":[\"navigate\"],\"behaviour\":{\"variables\":[\"current_x\",\"current_y\",\"current_z\"],\"states\":[{\"name\":\"no_initial_pose\",\"initial\":true,\"transitions\":[{\"next\":\"awaiting_point\",\"id\":\"message_initial_pose_u_response_\",\"type\":\"response\",\"communication\":\"initial_pose\"}]},{\"name\":\"awaiting_point\",\"initial\":false,\"transitions\":[{\"next\":\"has_point\",\"id\":\"message_point_u_response_\",\"type\":\"response\",\"communication\":\"point\"}]},{\"name\":\"has_point\",\"initial\":false,\"transitions\":[{\"next\":\"awaiting_point\",\"id\":\"action_navigate_u_response_\",\"type\":\"response\",\"communication\":\"navigate\"},{\"next\":\"awaiting_point\",\"id\":\"action_navigate_c_cancel_\",\"type\":\"cancel\",\"communication\":\"navigate\"}]}]}},{\"name\":\"EmergencyStop\",\"messages\":[\"stop\",\"continue\"],\"services\":[],\"actions\":[],\"behaviour\":{\"variables\":[],\"states\":[{\"name\":\"in_service\",\"initial\":true,\"transitions\":[{\"next\":\"stopped\",\"id\":\"message_stop_u_response_\",\"type\":\"response\",\"communication\":\"stop\"}]},{\"name\":\"stopped\",\"initial\":false,\"transitions\":[{\"next\":\"in_service\",\"id\":\"message_continue_u_response_\",\"type\":\"response\",\"communication\":\"continue\"}]}]}}]}";
		output << "}";
		
		auto msg = std_msgs::msg::String();
		msg.data = output.str();
		
		this->state_information->publish(msg);
	
		taken_transitions.clear();
	}
private:
	// Heart of the controller
	void tick() {
		int nOfDataEvents = 1;
		      controller_Event_ data_events[1] = { data_navigate_c_p36UE88TBHOXD_ };
		
		// Always execute data transitions that are possible
		shuffle_events(data_events, nOfDataEvents);
		
		for (int i = 0; i < nOfDataEvents; i++) {
			controller_EnginePerformEvent(data_events[i]);
		}
		
		int nOfControllableEvents = 3;
		      controller_Event_ controllable_events[3] = { action_navigate_c_reset_,action_navigate_c_trigger_,action_navigate_c_cancel_ };
		
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
	
	
	assigned = true;
}
void controller_InfoEvent(controller_Event_ event, BoolType pre) {
    if (pre) {
    	node_controller->taken_transitions.push_back(controller_event_names[event]);
    	return;
    }
    
    switch (event) {
case action_navigate_c_trigger_:
	node_controller->call_action_navigate();
break;
case action_navigate_c_cancel_:
	node_controller->cancel_action_navigate();
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
