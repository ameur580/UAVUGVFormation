#include <map>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int64.hpp>
#include <mr_interfaces/srv/provide_data.hpp>
#include <mr_interfaces/srv/management.hpp>
#include "mr_aggregator/processing.hpp"
#include "mr_aggregator/aggr_conf.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std;

#define T_PARAMS class Proc
#define TEMPLATE template<T_PARAMS>


namespace g_robot {
	TEMPLATE
		class Aggregator{
			public:
				Aggregator(int argc, char *argv[]){
					static_assert(std::is_base_of<Processing<typename Proc::Service,typename Proc::AggrMsg>, Proc>::value, 
							"type parameter of this class must derive from Processing");
					static_assert(std::is_base_of<AggrConf, typename Proc::Conf>::value, 
							"type parameter of this class must derive from AggrConf");

					nb_clients = 0;
					increment_msg.data = 1;

					current_step = 0;

					rclcpp::init(argc, argv);


					auto node = rclcpp::Node::make_shared("g_aggregator");
					increment_pub = node->create_publisher<std_msgs::msg::Int64>(
							"/g_clock/increment", 10);
					step_sub = node->create_subscription<std_msgs::msg::Int64>(
							"/g_clock/step", 10, 
							std::bind(&Aggregator::step_callback, this, _1));

					auto  svc_send_data = node->create_service<typename Proc::Service>(
							conf.get_provide_data_service_name(),
							std::bind(&Aggregator::provide_data,this, _1,_2));

					auto  svc_manage = node->create_service<mr_interfaces::srv::Management>(
							conf.get_manage_service_name(),
							std::bind(&Aggregator::manage,this, _1,_2));

					proc.init(node);
					cycle();
					rclcpp::spin(node);
				}

			private:
				Proc proc;
				typename Proc::Conf conf;
				int current_step;
				int nb_clients;
				int nb_received;
				map<string,int> clients;
				map<string,int> last_step_received;
				std_msgs::msg::Int64  increment_msg;
				rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr increment_pub;
				rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr step_sub;

				void show_clients(){
					for (auto it=clients.begin(); it!=clients.end();++it){
						cout << it->second << "/" << nb_clients << ":" << it->first << endl;
					}	
				}

				std::mutex nb_clients_mutex;
				void add_client(string id){
					//std::lock_guard<std::mutex> guard(nb_clients_mutex);
					nb_clients++;
					clients[id] = nb_clients;
					last_step_received[id] = -1;
					show_clients();
				}


				void remove_client(string id){
					//std::lock_guard<std::mutex> guard(nb_clients_mutex);
					int idx = clients[id];
					clients.erase(id);
					last_step_received.erase(id);
					nb_clients--;
					for (auto it=clients.begin(); it!=clients.end();++it){
						if(it->second>idx) it->second--; 
					}	
				}


				void cycle(){
					nb_received = 0;
					proc.restart();
					increment_pub->publish(increment_msg);
				}

				void manage(
						const std::shared_ptr<mr_interfaces::srv::Management::Request> request,
						std::shared_ptr<mr_interfaces::srv::Management::Response>  response){
					response->ok = true;
					if (request->command == "join"){
						if (clients.count(request->id)==0){
							add_client(request->id);
						} else {
							cout << "Registration duplicate" << endl;
						}

					} else if (request->command == "leave") {
						if (clients.count(request->id)!=0){
							remove_client(request->id);
						} else {
							cout << "Client is not registered" << endl;
						}

					} else
						response->ok = false;
				}

				void provide_data(
						const std::shared_ptr<typename Proc::Service::Request> request,
						std::shared_ptr<typename Proc::Service::Response>  response){
					auto p=clients.find(request->id);
					if (request->step != current_step) {
							cout << "Wrong step (expected:" << current_step << ", received: " << request->step << ")" << endl;
					} else if (p!=clients.end()){
						if (last_step_received[request->id]!=current_step){
							proc.in(request);
							response->ok = true;
							nb_received++;
							last_step_received[request->id] = current_step;
							if (nb_received >= nb_clients){
								proc.out(nb_clients,current_step);
								cycle();
							}
						} else {
							cout << "Already received (" << request->id << ")" << endl;
						}
					} else {
						cout << "Unknown client (" << request->id << ")" << endl;
					}
				}


				void step_callback(const std_msgs::msg::Int64::SharedPtr msg){
					current_step = msg->data;
					if (current_step == 0)
						cycle();
				}

		};
}
