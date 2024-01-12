#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "mr_clock/clock.hpp"

using namespace std;
using namespace mr_clock;

void signal_handler(int signal)
{
	cout << signal << endl; 
	raise(SIGKILL);
}


int main(int argc, char *argv[]){
	signal(SIGINT, signal_handler);
	new Clock(argc, argv, 100);
	return 0;
}


