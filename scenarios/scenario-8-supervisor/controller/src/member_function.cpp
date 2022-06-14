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
#include "geometry_msgs/msg/twist.hpp"
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

double code_LineDetector_current_correction = 0.0;

class Controller : public rclcpp::Node {
public:	
	// Enum conversions

	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_client_correction;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_no_line;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_client_move;
	std::vector<std::string> taken_transitions;

	Controller() : Node("controller") {
		subscriber_client_correction = this->create_subscription<std_msgs::msg::Float32>("/correction", 10, std::bind(&Controller::callback_message_correction, this, std::placeholders::_1));
		subscriber_client_no_line = this->create_subscription<std_msgs::msg::Empty>("/no_line", 10, std::bind(&Controller::callback_message_no_line, this, std::placeholders::_1));
		publisher_client_move = this->create_publisher<geometry_msgs::msg::Twist>("/simple_movement", 10);

		timer = this->create_wall_timer(100ms, std::bind(&Controller::tick, this));
		controller_EngineFirstStep();
	}

	void callback_message_correction(const std_msgs::msg::Float32::SharedPtr msg) {
		
		code_LineDetector_current_correction = msg->data;
		
		
		// Call engine function
		controller_EnginePerformEvent(message_correction_u_response_);
	}
	
	
	void callback_message_no_line(const std_msgs::msg::Empty::SharedPtr msg) {
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_no_line_u_response_);
	}
	
	
	
	
	void call_message_move() {
		auto value = geometry_msgs::msg::Twist();
		
		if (data_move_ == _controller_data_pBNSF3MZ29JX8) {
			value.linear.x = 0.6;
			value.angular.z = (-code_LineDetector_current_correction) / 100;
		}
		
		this->publisher_client_move->publish(value);
	}
private:
	// Heart of the controller
	void tick() {
		int nOfDataEvents = 1;
		      controller_Event_ data_events[1] = { data_move_c_pKM3MRA4SLUW6_ };
		
		// Always execute data transitions that are possible
		shuffle_events(data_events, nOfDataEvents);
		
		for (int i = 0; i < nOfDataEvents; i++) {
			controller_EnginePerformEvent(data_events[i]);
		}
		
		int nOfControllableEvents = 1;
		      controller_Event_ controllable_events[1] = { message_move_c_trigger_ };
		
		shuffle_events(controllable_events, nOfControllableEvents);
		
		for (int i = 0; i < nOfControllableEvents; i++) {
			if (controller_EnginePerformEvent(controllable_events[i])) {
				break;
			}
		}
	}
	
	rclcpp::TimerBase::SharedPtr timer;
};

std::shared_ptr<Controller> node = nullptr;

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
case message_move_c_trigger_:
	node->call_message_move();
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
