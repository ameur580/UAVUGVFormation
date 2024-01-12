#ifndef __AVERAGER_HPP
#define __AVERAGER_HPP

#include "mr_aggregator/processing.hpp"
#include "mr_aggregator/aggr_conf.hpp"
#include "mr_aggregator/req_factory.hpp"
#include "mr_interfaces/srv/send_state.hpp"
#include "mr_interfaces/msg/state.hpp"
#include "rclcpp/node.hpp"

using namespace g_robot;

typedef mr_interfaces::srv::SendState tsvc;

typedef tsvc::Request MyRequest;
typedef tsvc::Response MyResponse;
typedef mr_interfaces::msg::State MyAggrMsg;

class Averager:public Processing<tsvc,MyAggrMsg> {

	public:

		class Conf: AggrConf {
			public:
				string get_provide_data_service_name(){
					return "/averager/provide_data";
				}

				string get_manage_service_name(){
					return "/averager/manage";
				}
		};


		void init(rclcpp::Node::SharedPtr node) {
			//cout << "AVERAGER init" << endl;
			this->node = node;
			aggregated_pub = node->create_publisher<MyAggrMsg>(
					get_aggregated_topic_name(), 10);
			restart();
		}

		void restart() { 
			aggregated_msg.x = 0.0;
			aggregated_msg.y = 0.0;
			aggregated_msg.z = 0.0;
			aggregated_msg.ahead = false;
			aggregated_msg.left = false;
			aggregated_msg.right = false;
			aggregated_msg.epuck.theta = 0.0;
		}

		void in(std::shared_ptr<MyRequest> r) {

			cout << " ----- Averager: received message type = " << r->state.type << endl;
			aggregated_msg.x += r->state.x;
			aggregated_msg.y += r->state.y;
			aggregated_msg.z += r->state.z;
			aggregated_msg.ahead = aggregated_msg.ahead || r->state.ahead;
			aggregated_msg.left =  aggregated_msg.left ||  r->state.left; 
			aggregated_msg.right = aggregated_msg.right || r->state.right;
			if (r->state.type == mr_interfaces::msg::State::TYPE_EP) {
				aggregated_msg.epuck.theta = r->state.epuck.theta;
			} else if (r->state.type == mr_interfaces::msg::State::TYPE_CF) {
			}
		}

		void out(int nb_clients, int step) {
			aggregated_msg.step = step;
			aggregated_msg.x /= nb_clients;
			aggregated_msg.y /= nb_clients;
			aggregated_msg.z /= nb_clients;
			aggregated_pub->publish(aggregated_msg);
		}

		string out2string(){return string("sum: ");}

		string get_aggregated_topic_name(){
			return "/averager/state";
		}

	private:
		rclcpp::Node::SharedPtr node;
		rclcpp::Publisher<MyAggrMsg>::SharedPtr aggregated_pub;
		MyAggrMsg aggregated_msg;
};

#endif

