#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace g_robot {
	template <class ServiceT, class RequestT, class ResponseT>
		class Aggregator{
			public:
				Aggregator(int argc, char *argv[]);
			private:
				int nb_clients;
				rclcpp::Subscription<std_msgs::msg::String>::SharedPtr register_sub;
				void register_callback(const std_msgs::msg::String::SharedPtr msg);
			void send_data(const std::shared_ptr<RequestT> request,
          std::shared_ptr<ResponseT>  response);
		};
}
