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
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
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



class Supervisor : public rclcpp::Node {
public:	
	// Enum conversions

	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr supervised_subscriber_client_point;
	rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_client_point;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr supervised_subscriber_client_initial_pose;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriber_client_initial_pose;
	rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_navigate;
	rclcpp_action::Server<nav2_msgs::action::NavigateToPose>::SharedPtr supervised_action_client_navigate;
	std::mutex mutex_supervised_action_client_navigate;
	std::shared_ptr<nav2_msgs::action::NavigateToPose::Goal> data_holder_supervised_action_client_navigate;
	std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle_supervised_action_client_navigate;
	rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr supervised_subscriber_client_stop;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_stop;
	rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr supervised_subscriber_client_continue;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_continue;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr blocked;
	std::vector<std::string> taken_transitions;

	Supervisor() : Node("supervisor") {
		supervised_subscriber_client_point = this->create_publisher<geometry_msgs::msg::PointStamped>("/clicked_point_pT6GTW0MY1FJ7", 10);
		subscriber_client_point = this->create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 10, std::bind(&Supervisor::callback_message_point, this, std::placeholders::_1));
		supervised_subscriber_client_initial_pose = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose_pF0790C71IQRO", 10);
		subscriber_client_initial_pose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10, std::bind(&Supervisor::callback_message_initial_pose, this, std::placeholders::_1));
		supervised_action_client_navigate = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
		      this,
		      "/navigate_to_pose_pZQ6KSO70PFQJ",
		      std::bind(&Supervisor::handle_goal_navigate_supervised, this, std::placeholders::_1, std::placeholders::_2),
		      std::bind(&Supervisor::handle_cancelled_navigate_supervised, this, std::placeholders::_1),
		      std::bind(&Supervisor::handle_accepted_navigate_supervised, this, std::placeholders::_1));
		action_client_navigate = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");
		supervised_subscriber_client_stop = this->create_publisher<std_msgs::msg::Empty>("/stop_pQTYU6IWH9SIJ", 10);
		subscriber_client_stop = this->create_subscription<std_msgs::msg::Empty>("/stop", 10, std::bind(&Supervisor::callback_message_stop, this, std::placeholders::_1));
		supervised_subscriber_client_continue = this->create_publisher<std_msgs::msg::Empty>("/continue_pJDTWAIIH21J4", 10);
		subscriber_client_continue = this->create_subscription<std_msgs::msg::Empty>("/continue", 10, std::bind(&Supervisor::callback_message_continue, this, std::placeholders::_1));

		blocked = this->create_publisher<std_msgs::msg::String>("/blocked", 10);
		timer = this->create_wall_timer(100ms, std::bind(&Supervisor::tick, this));
		controller_EngineFirstStep();
	}

	void callback_message_point(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
		
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_point_u_response_);
		
		// Forward event
		this->supervised_subscriber_client_point->publish(*msg);
	
		this->execute_all_silent();
	}
	
	void callback_message_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
		
		
		
		
		
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_initial_pose_u_response_);
		
		// Forward event
		this->supervised_subscriber_client_initial_pose->publish(*msg);
	
		this->execute_all_silent();
	}
	
	// Act as proxy server
	rclcpp_action::GoalResponse handle_goal_navigate_supervised(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
		this->mutex_supervised_action_client_navigate.lock();
		
		auto transformed = std::make_shared<nav2_msgs::action::NavigateToPose::Goal>(*goal);
		this->data_holder_supervised_action_client_navigate = transformed;
	
		this->mutex_supervised_action_client_navigate.unlock();
	
	    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}
	
	rclcpp_action::CancelResponse handle_cancelled_navigate_supervised(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
	    return this->cancel_action_navigate();
	}
	
	void handle_accepted_navigate_supervised(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
		this->goal_handle_supervised_action_client_navigate = goal_handle;
		if (!controller_EnginePerformEvent(action_navigate_c_trigger_)) {
			this->publish_block("/navigate_to_pose");
		}
	}
	
	// Handle events from actual action server
	void response_action_navigate(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
		
		
		
		this->mutex_supervised_action_client_navigate.lock();
		if (this->data_holder_supervised_action_client_navigate != nullptr) {
			auto transformed = std::make_shared<nav2_msgs::action::NavigateToPose::Result>(*result.result);
	
			switch (result.code) {
				case rclcpp_action::ResultCode::SUCCEEDED:
					this->goal_handle_supervised_action_client_navigate->succeed(transformed);
					break;
				case rclcpp_action::ResultCode::ABORTED:
					this->goal_handle_supervised_action_client_navigate->abort(transformed);
					break;
				case rclcpp_action::ResultCode::CANCELED:
					this->goal_handle_supervised_action_client_navigate->canceled(transformed);
					break;
				default:
					this->goal_handle_supervised_action_client_navigate->abort(transformed);
					break;
		    }
			
			this->data_holder_supervised_action_client_navigate = nullptr;
			this->goal_handle_supervised_action_client_navigate = nullptr;
		}
		this->mutex_supervised_action_client_navigate.unlock();
				
		
		// Call engine function
		controller_EnginePerformEvent(action_navigate_u_response_);
		this->execute_all_silent();
	}
	
	void feedback_action_navigate(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr handle, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
		
		
		
		auto transformed = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>(*feedback);
		this->goal_handle_supervised_action_client_navigate->publish_feedback(transformed);
		
		// Call engine function
		controller_EnginePerformEvent(action_navigate_u_feedback_);
		this->execute_all_silent();
	}
	
	void call_action_navigate() {
		if (!this->action_client_navigate->wait_for_action_server(1s)) {
			controller_EnginePerformEvent(action_navigate_u_error_);
			return;
		}
	
		auto send_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
		send_options.result_callback = std::bind(&Supervisor::response_action_navigate, this, std::placeholders::_1);
		send_options.feedback_callback = std::bind(&Supervisor::feedback_action_navigate, this, std::placeholders::_1, std::placeholders::_2);
		this->action_client_navigate->async_send_goal(*this->data_holder_supervised_action_client_navigate, send_options);
	}
		
	rclcpp_action::CancelResponse cancel_action_navigate() {
		auto result = this->action_client_navigate->async_cancel_all_goals().get();
		
		if (result->return_code == 0) {
			return rclcpp_action::CancelResponse::ACCEPT;
		} else {
			return rclcpp_action::CancelResponse::REJECT;
		}
	}
	void callback_message_stop(const std_msgs::msg::Empty::SharedPtr msg) {
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_stop_u_response_);
		
		// Forward event
		this->supervised_subscriber_client_stop->publish(*msg);
	
		this->execute_all_silent();
	}
	
	void callback_message_continue(const std_msgs::msg::Empty::SharedPtr msg) {
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_continue_u_response_);
		
		// Forward event
		this->supervised_subscriber_client_continue->publish(*msg);
	
		this->execute_all_silent();
	}
	
	
private:
	// Only used to emit state information
	void tick() {
	}
	
	void execute_all_silent() {
		int nOfControllableEvents = 2;
		      controller_Event_ controllable_events[2] = { action_navigate_c_reset_,action_navigate_c_cancel_ };
		
		shuffle_events(controllable_events, nOfControllableEvents);
		
		for (int i = 0; i < nOfControllableEvents; i++) {
			controller_EnginePerformEvent(controllable_events[i]);
		}
	}
	
	void publish_block(std::string identifier) {
		auto msg = std_msgs::msg::String();
		msg.data = identifier;
		
		this->blocked->publish(msg);
	}
	
	rclcpp::TimerBase::SharedPtr timer;
};

std::shared_ptr<Supervisor> node = nullptr;

// Control synthesis engine
bool assigned = false;

void controller_AssignInputVariables() {
	if (assigned) return;
	
	
	assigned = true;
}
void controller_InfoEvent(controller_Event_ event, BoolType pre) {
    if (pre) {
    	node->taken_transitions.push_back(controller_event_names[event]);
    	return;
    }
    
    switch (event) {
case action_navigate_c_trigger_:
	node->call_action_navigate();
break;
case action_navigate_c_cancel_:
	node->cancel_action_navigate();
break;

    default:
		return;	
   	}
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    node = std::make_shared<Supervisor>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
