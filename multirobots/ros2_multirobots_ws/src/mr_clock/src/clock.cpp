#include <iostream>
#include "mr_clock/clock.hpp"


using namespace std;

using std::placeholders::_1;


namespace mr_clock {
	Clock::Clock(int argc, char *argv[], int ms){
		counter = 0;
		step_msg.data = counter;
		rclcpp::init(argc, argv);
		auto node = rclcpp::Node::make_shared("g_clock");
		auto timer_ = node->create_wall_timer(
				chrono::milliseconds(ms),
				bind(&Clock::timer_callback, this));
		step_pub = node->create_publisher<std_msgs::msg::Int64>(
				"/g_clock/step", 100);
		increment_sub = node->create_subscription<std_msgs::msg::Int64>(
				"/g_clock/increment", 1,
				std::bind(&Clock::increment_callback, this, _1));
		timer_callback();
		rclcpp::spin(node);
	}

	void Clock::timer_callback(){
		cout << step_msg.data << endl;
		step_pub->publish(step_msg);
	}

	void Clock::increment_callback(const std_msgs::msg::Int64::SharedPtr msg){
		counter++;
		step_msg.data = counter;
	}
}


