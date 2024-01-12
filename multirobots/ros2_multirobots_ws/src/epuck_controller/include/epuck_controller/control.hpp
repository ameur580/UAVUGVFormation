#ifndef B2ROS_H_
#define B2ROS_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mr_interfaces/srv/send_state.hpp"
#include "mr_aggregator/worker.hpp"
#include "epuck_controller/state_req_factory.hpp"
#include "epuck_controller/conf.hpp"

typedef mr_interfaces::srv::SendState Service;

using namespace std;
namespace epuck_controller {

	class Control{
		public:
			explicit Control(int argc, char *argv[]);

			void go();


		private:

			void work(int argc, char *argv[], StateReqFactory *rf);

			void set_velocity(double, double);


			// general
			string my_id;
			rclcpp::Node::SharedPtr node;
			StateReqFactory *rf;
			g_robot::Worker<Conf,Service,StateReqFactory> *worker;


			// velocity
			rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
			geometry_msgs::msg::Twist vel_msg;

			// state
			rclcpp::Subscription<mr_interfaces::msg::State>::SharedPtr state_sub;
			void state_callback(const mr_interfaces::msg::State::SharedPtr msg);

	};

}

#endif
