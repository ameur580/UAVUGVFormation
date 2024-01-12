#include <iostream>

#include "epuck_controller/odometry.hpp"
#include "epuck_controller/defs.hpp"

using namespace epuck_controller;
using namespace std;

	Odometry::Odometry(){
		odom = new double[7];
		odom_valid = false;
	}


	bool Odometry::copy(double *odom_cur){
		if (odom_valid){
			memcpy(odom_cur,odom,7*sizeof(double));
		}
		return odom_valid;
	}


	double Odometry::get_heading(){
		return odom[QX]<0?0.5*M_PI-odom[QY]:1.5*M_PI+odom[QY];
	}

	void Odometry::get_pose(double &x, double &y, double &z, double &theta){
		x = odom[X];
		y = odom[Y];
		z = odom[Z];
		theta = get_heading();
	}

	void Odometry::callback(const nav_msgs::msg::Odometry::SharedPtr msg){
		decode(msg);
		odom_valid = true;
	}

	void Odometry::decode(const nav_msgs::msg::Odometry::SharedPtr msg){

		tf2::convert(msg->pose.pose.orientation , q_cur);

		odom[X] = msg->pose.pose.position.x;
		odom[Y] = msg->pose.pose.position.y;
		odom[Z] = msg->pose.pose.position.z;
		odom[QX] = q_cur.x();
		odom[QY] = q_cur.y();
		odom[QZ] = q_cur.z();
		odom[QW] = q_cur.w();
	}

	void Odometry::rotate_by(double *odom_cur,double angle, double *odom_target){
		tf2::Quaternion q_rot, q_target;
		q_rot.setRPY(0.0,0.0,angle*M_PI/180);	
		q_target = q_rot * q_cur;
		q_target.normalize();
		odom_target[X] = odom[X];
		odom_target[Y] = odom[Y];
		odom_target[Z] = odom[Z];
		odom_target[QX] = q_target.x();
		odom_target[QY] = q_target.y();
		odom_target[QZ] = q_target.z();
		odom_target[QW] = q_target.w();
		memcpy(odom_cur,odom,7*sizeof(double));
	}
