#ifndef __MY_LASERSCAN_HPP
#define __MY_LASERSCAN_HPP

#include "std_msgs/msg/float64_multi_array.hpp"

namespace epuck_controller{

	class LaserScan {
		public:
			LaserScan();
			bool copy(double *laser_scan_cur);
			void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
			void decode(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
		private:
			double *laser_scan;
			bool laser_scan_valid;
	};

}

#endif

