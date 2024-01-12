#include "epuck_controller/control.hpp"
#include "epuck_controller/control/shared.hpp"
#include "epuck_controller/control/utils.hpp"

#include <iostream>


extern shared_t shared;

using std::placeholders::_1;

using namespace std;

namespace epuck_controller {
	Control::Control(int argc, char *argv[]){
		my_id = string(argv[1]);

		rclcpp::init(argc, argv);

		rclcpp::uninstall_signal_handlers();
		rclcpp::install_signal_handlers(rclcpp::SignalHandlerOptions::None);

		node = rclcpp::Node::make_shared("the_node_"+my_id);


		// velocity
		vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(
				"/model/"+my_id+"/cmd_vel", 100);

		// state
		state_sub = node->create_subscription<mr_interfaces::msg::State>(
				"/averager/state", 1,
				std::bind(&Control::state_callback, this, _1));

		auto req = make_shared<Service::Request>();
		rf = new StateReqFactory(req,node,my_id);

		worker = new g_robot::Worker<Conf, Service, StateReqFactory>(node,my_id ,*rf);		
		cout << "worker created " << endl;
	}


	// general

	void Control::go(){
		double v_linear, v_angular;
		convert_velocity(shared.vl,shared.vr,&v_linear,&v_angular);
		set_velocity(v_linear,v_angular);
		cout << "let's spin" << endl;
		//int i;
		//cin >> i;
		rclcpp::spin(node);
	}


	// velocity
	void Control::set_velocity(double l, double a){
		vel_msg.linear.x = l;
		vel_msg.angular.z = a;
		vel_pub->publish(vel_msg);
	}

	// state

	void Control::state_callback(const mr_interfaces::msg::State::SharedPtr msg){
		shared.obstacle_ahead = msg->ahead;
		shared.obstacle_left = msg->left;
		shared.obstacle_right = msg->right;
		shared.xg = msg->x;
		shared.yg = msg->y;
		shared.thetag = msg->epuck.theta;

		move_to_next_state();

		double linear, angular;
		convert_velocity(shared.vl, shared.vr, &linear, &angular);
		set_velocity(linear, angular);

	}
}
