#include <iostream>
#include "epuck_controller/control/shared.hpp"
#include "epuck_controller/control/utils.hpp"
#include "epuck_controller/monitor.hpp"

extern shared_t shared;

using namespace std;

void monitor(epuck_controller::Control *control){
	cout << "obstacles: " 
		<< shared.obstacle_left 
		<< ", " << shared.obstacle_ahead 
		<< ", " << shared.obstacle_right
	 	<< endl;	

}
