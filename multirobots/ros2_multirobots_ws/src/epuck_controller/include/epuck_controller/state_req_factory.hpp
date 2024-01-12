#ifndef __STATE_REQ_FACTORY_HPP
#define __STATE_REQ_FACTORY_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "mr_interfaces/srv/send_state.hpp"
#include "mr_interfaces/msg/state.hpp"
#include "mr_aggregator/req_factory.hpp"
#include "epuck_controller/odometry.hpp"
#include "epuck_controller/laser_scan.hpp"
#include "epuck_controller/control/sensors_specific.hpp"

using std::placeholders::_1;

typedef mr_interfaces::srv::SendState::Request MyRequest;

using namespace epuck_controller;

class StateReqFactory:public ReqFactory<MyRequest> {
	public:
		StateReqFactory(MyRequest::SharedPtr req,rclcpp::Node::SharedPtr node, string &id){
			odometry = new Odometry();
			odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
					"/model/"+id+"/odometry", 1, 
					std::bind(&StateReqFactory::odom_callback, this, _1));
			laser_scan = new LaserScan();
			laser_scan_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
					"/model/"+id+"/laser_scan", 1,
					std::bind(&StateReqFactory::laser_scan_callback, this, _1));
			this->req = req;
			cout << "StateReqFactory created " << endl;
		}

		MyRequest::SharedPtr get_request(int step, string &id){
			req->step = step;
			req->id = id;
			req->state.type = mr_interfaces::msg::State::TYPE_EP;
			double l_s[8];
			req->state.ahead = req->state.left = req->state.right=false;
			if (laser_scan->copy(&l_s[0]))
				local_obstacle(&l_s[0],req->state.ahead, req->state.left,req->state.right);
			odometry->get_pose(req->state.x,req->state.y,req->state.z,req->state.epuck.theta);
			return req;
		}
		
		void display_req(string msg, MyRequest::SharedPtr req){
			cout << msg << req << endl;
		}

	private:
		MyRequest::SharedPtr req;
		Odometry *odometry;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
		void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){odometry->callback(msg);}
		LaserScan *laser_scan;
		rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr laser_scan_sub;
		void laser_scan_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){laser_scan->callback(msg);}
};
#endif
