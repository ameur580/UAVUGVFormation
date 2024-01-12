#ifndef __MY_ODOMETRY_HPP
#define  __MY_ODOMETRY_HPP

#include "nav_msgs/msg/odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace epuck_controller {

class Odometry {
	public:
		Odometry();
		double *get_odom(){return odom; }
		bool copy(double *odom_cur);
		void callback(const nav_msgs::msg::Odometry::SharedPtr msg);
		void decode(const nav_msgs::msg::Odometry::SharedPtr msg);
		void rotate_by(double *odom_cur,double angle, double *odom_target);
		double get_heading();
		void get_pose(double &x, double &y, double &z, double &theta);
	private:
		double *odom;
		bool odom_valid;
		tf2::Quaternion q_cur;
};

}


#endif
