#include <map>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int64.hpp>
#include <mr_interfaces/srv/provide_data.hpp>
#include <mr_interfaces/srv/management.hpp>
#include "mr_aggregator/aggr_conf.hpp"
#include "mr_aggregator/req_factory.hpp"
#include "mr_aggregator/processing.hpp"
#include "rclcpp/node.hpp"

using std::placeholders::_1;
using namespace std;

typedef mr_interfaces::srv::Management tmng;

namespace g_robot {
	template <typename Conf,class Service, typename IReqFactory>
		class Worker{
			public:
				Worker(rclcpp::Node::SharedPtr node, string id, IReqFactory &rf):req_factory(rf){
					static_assert(std::is_base_of<AggrConf, Conf>::value, 
							"type parameter of this class must derive from AggrConf");
					static_assert(std::is_base_of<ReqFactory<typename Service::Request>, IReqFactory>::value, 
							"type parameter of this class must derive from Processing");

					my_id = id;

					last_step = -1;

					srand(time(nullptr));

					// step
					step_sub = node->create_subscription<std_msgs::msg::Int64>(
							"/g_clock/step", 10, 
							bind(&Worker::step_callback, this, _1));

					// leave/join
					management_client = 
						node->create_client<tmng>(conf.get_manage_service_name());
					management_request = make_shared<tmng::Request>();
					management_request->id = my_id;

					// data exchange
					provide_data_client = 
						node->create_client<Service>(conf.get_provide_data_service_name());

					// register
					management_request->command = "join";
					while (!management_client->wait_for_service(1s)) {
						if (!rclcpp::ok()) {
							RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
							return;
						}
						RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
					}

					auto response = management_client->async_send_request(
							management_request); 

				}


				void stop(){
					// unregister
					management_request->command = "leave";
					auto response = management_client->async_send_request(
							management_request); 
				}
			private:
				Conf conf;
				IReqFactory req_factory;
				string my_id;
				int last_step;
				void register_me();
				rclcpp::Node::SharedPtr node;
				typename rclcpp::Client<Service>::SharedPtr provide_data_client;
				rclcpp::Client<tmng>::SharedPtr management_client;
				tmng::Request::SharedPtr management_request;
				rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr step_sub;

				void step_callback(const std_msgs::msg::Int64::SharedPtr msg){
					if (msg->data != last_step) { 
						last_step = msg->data;

						while (!provide_data_client->wait_for_service(1s)) {
							if (!rclcpp::ok()) {
								RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
								return;
							}
							RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
						}

						auto req = req_factory.get_request(msg->data,my_id);
						//req_factory.display_req("WORKER received ", req);
						auto response = provide_data_client->async_send_request(req);
				//				req_factory.get_request(msg->data,my_id)); 
						//this_thread::sleep_for(chrono::milliseconds(500+rand()%500));
					}
				}
		};
}
