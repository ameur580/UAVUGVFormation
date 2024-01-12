#include <string.h>
#include <iostream>
#include "epuck_controller/laser_scan.hpp"

using namespace epuck_controller;


LaserScan::LaserScan(){
		laser_scan = new double[8];
		laser_scan_valid = false;
}

	bool LaserScan::copy(double *laser_scan_cur){
		if (laser_scan_valid){
			memcpy(laser_scan_cur,laser_scan,8*sizeof(double));
		} 
		return laser_scan_valid;
	}

	void LaserScan::callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
		decode(msg);
		laser_scan_valid = true;
	}
	void LaserScan::decode(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
		memcpy(laser_scan,&msg->data[0],8*sizeof(double));
	}
