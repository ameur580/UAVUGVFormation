#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int64.hpp"

namespace mr_clock {
	class Clock {
		public:
			Clock(int argc, char *argv[], int ms);
		private:
			void timer_callback();
			void increment_callback(const std_msgs::msg::Int64::SharedPtr msg);
			int counter;
			rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr step_pub;
			std_msgs::msg::Int64  step_msg;
			rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr increment_sub;
	};
}
