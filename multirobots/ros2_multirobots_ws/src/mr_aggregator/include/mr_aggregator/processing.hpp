#ifndef __PROCESSING_HPP
#define __PROCESSING_HPP

#include "rclcpp/node.hpp"
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

using namespace std;

namespace g_robot {
	template<class Svc, class AMsg>
		class Processing{
			public:
				class Conf;
				typedef Svc Service;
				typedef AMsg AggrMsg;
				virtual void init(rclcpp::Node::SharedPtr) = 0;
				virtual void restart() = 0;
				virtual void in(const std::shared_ptr<typename Service::Request> request) = 0;
				virtual void out(int nb_clients, int step) = 0;
				virtual string out2string() = 0;
		};
}
#endif
