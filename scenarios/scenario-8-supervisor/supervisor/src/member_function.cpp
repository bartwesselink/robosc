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

double code_LineDetector_current_correction = 0.0;

class Supervisor : public rclcpp::Node {
public:	
	// Enum conversions

	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr supervised_subscriber_client_correction;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_client_correction;
	rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr supervised_subscriber_client_no_line;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_no_line;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr supervised_publisher_client_move;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_client_move;
	std::mutex mutex_supervised_publisher_client_move;
	geometry_msgs::msg::Twist::SharedPtr data_holder_supervised_publisher_client_move;
	rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr supervised_subscriber_client_stop;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_stop;
	rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr supervised_subscriber_client_continue;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscriber_client_continue;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_information;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr blocked;
	std::vector<std::string> taken_transitions;

	Supervisor() : Node("supervisor") {
		supervised_subscriber_client_correction = this->create_publisher<std_msgs::msg::Float32>("/correction_pTJXJGOJ3LG7I", 10);
		subscriber_client_correction = this->create_subscription<std_msgs::msg::Float32>("/correction", 10, std::bind(&Supervisor::callback_message_correction, this, std::placeholders::_1));
		supervised_subscriber_client_no_line = this->create_publisher<std_msgs::msg::Empty>("/no_line_pWTCO9HEYOG9T", 10);
		subscriber_client_no_line = this->create_subscription<std_msgs::msg::Empty>("/no_line", 10, std::bind(&Supervisor::callback_message_no_line, this, std::placeholders::_1));
		supervised_publisher_client_move = this->create_subscription<geometry_msgs::msg::Twist>("/simple_movement_pEIFRW5LRP7VV", 10, std::bind(&Supervisor::callback_message_move_supervised, this, std::placeholders::_1));
		publisher_client_move = this->create_publisher<geometry_msgs::msg::Twist>("/simple_movement", 10);
		supervised_subscriber_client_stop = this->create_publisher<std_msgs::msg::Empty>("/stop_pZBDPKQYE2EB7", 10);
		subscriber_client_stop = this->create_subscription<std_msgs::msg::Empty>("/stop", 10, std::bind(&Supervisor::callback_message_stop, this, std::placeholders::_1));
		supervised_subscriber_client_continue = this->create_publisher<std_msgs::msg::Empty>("/continue_pK0VB4G3N8KBH", 10);
		subscriber_client_continue = this->create_subscription<std_msgs::msg::Empty>("/continue", 10, std::bind(&Supervisor::callback_message_continue, this, std::placeholders::_1));

		blocked = this->create_publisher<std_msgs::msg::String>("/blocked", 10);
		state_information = this->create_publisher<std_msgs::msg::String>("/state", 10);
		timer = this->create_wall_timer(100ms, std::bind(&Supervisor::tick, this));
		controller_EngineFirstStep();
	}

	void callback_message_correction(const std_msgs::msg::Float32::SharedPtr msg) {
		
		code_LineDetector_current_correction = msg->data;
		
		
		// Call engine function
		controller_EnginePerformEvent(message_correction_u_response_);
		
		// Forward event
		this->supervised_subscriber_client_correction->publish(*msg);
	
		this->execute_all_silent();
	}
	
	void callback_message_no_line(const std_msgs::msg::Empty::SharedPtr msg) {
		
		
		
		// Call engine function
		controller_EnginePerformEvent(message_no_line_u_response_);
		
		// Forward event
		this->supervised_subscriber_client_no_line->publish(*msg);
	
		this->execute_all_silent();
	}
	
	
	void callback_message_move_supervised(const geometry_msgs::msg::Twist::SharedPtr msg) {
		// Call engine function
		this->mutex_supervised_publisher_client_move.lock();
		
		this->data_holder_supervised_publisher_client_move = msg;
		if (!controller_EnginePerformEvent(message_move_c_trigger_)) {
			this->publish_block("/simple_movement");
		}
		this->data_holder_supervised_publisher_client_move = nullptr;
	
		this->mutex_supervised_publisher_client_move.unlock();
	
		this->execute_all_silent();
	}
	
	void call_message_move() {
		this->publisher_client_move->publish(*this->data_holder_supervised_publisher_client_move);
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
	
	
	void emit_current_state() {
		std::stringstream output;
		
		output << "{";
		
		output << "\"current\": {" << "";
		output << "\"LineDetector\": {";
		output << "\"state\": \"" << enum_names[component_LineDetector_] << "\",";
		output << "\"variables\": {";
		
		output << "\"current_correction\": \"" << code_LineDetector_current_correction << "\"";				
		
		output << "}";
		output << "},";
		output << "\"EmergencyStop\": {";
		output << "\"state\": \"" << enum_names[component_EmergencyStop_] << "\",";
		output << "\"variables\": {";
		
		
		output << "}";
		output << "}";
		output << "},";
		output << "\"transitions\": " << serialize_json_vector(taken_transitions) << ",";
		output << "\"definition\": " << "{\"name\":\"LineFollowerSupervised\",\"components\":[{\"name\":\"LineDetector\",\"messages\":[\"correction\",\"no_line\"],\"services\":[],\"actions\":[],\"behaviour\":{\"variables\":[\"current_correction\"],\"states\":[{\"name\":\"no_line\",\"initial\":true,\"transitions\":[{\"next\":\"line_found\",\"id\":\"message_correction_u_response_\",\"type\":\"response\",\"communication\":\"correction\"}]},{\"name\":\"line_found\",\"initial\":false,\"transitions\":[{\"next\":\"no_line\",\"id\":\"message_no_line_u_response_\",\"type\":\"response\",\"communication\":\"no_line\"},{\"next\":null,\"id\":\"message_correction_u_response_\",\"type\":\"response\",\"communication\":\"correction\"}]}]}},{\"name\":\"SimpleMovement\",\"messages\":[\"move\"],\"services\":[],\"actions\":[]},{\"name\":\"EmergencyStop\",\"messages\":[\"stop\",\"continue\"],\"services\":[],\"actions\":[],\"behaviour\":{\"variables\":[],\"states\":[{\"name\":\"in_service\",\"initial\":true,\"transitions\":[{\"next\":\"stopped\",\"id\":\"message_stop_u_response_\",\"type\":\"response\",\"communication\":\"stop\"}]},{\"name\":\"stopped\",\"initial\":false,\"transitions\":[{\"next\":\"in_service\",\"id\":\"message_continue_u_response_\",\"type\":\"response\",\"communication\":\"continue\"}]}]}}]}";
		output << "}";
		
		auto msg = std_msgs::msg::String();
		msg.data = output.str();
		
		this->state_information->publish(msg);
	
		taken_transitions.clear();
	}
private:
	// Only used to emit state information
	void tick() {
		this->emit_current_state();
	}
	
	void execute_all_silent() {
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
case message_move_c_trigger_:
	node->call_message_move();
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
