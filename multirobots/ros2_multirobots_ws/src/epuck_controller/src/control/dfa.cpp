#include <iostream>
#include <iomanip>
#include <math.h>
#include "epuck_controller/control/dfa.hpp"
#include "epuck_controller/ccomputations.hpp"
#include "epuck_controller/control/shared.hpp"

extern shared_t shared;

#define DDEC 0.3

using namespace std;

string state_names[] = {
#define X(a) string(#a),
#include "epuck_controller/control/states.def"
#undef X
};

string state_name(state_t s){
	return state_names[s];
}

double epsilon = -1;
// espilon == -1 ==> turn left at obstacle 
// 			(the obstacle will be at the right of the robot)
// espilon == 1 ==> turn right at obstacle 
// 			(the obstacle will be at the left of the robot)


int remote_free_side(){
	return epsilon==-1?!shared.obstacle_right:!shared.obstacle_left;
}

void move_to_next_state(){
	state_t last_state = shared.state;
	double a = angle(shared.xg,shared.yg,shared.xtargetg,shared.ytargetg);
	double d = distance(shared.xg,shared.yg,shared.xtargetg, shared.ytargetg);
	if (d<0.01) {
		shared.state = END_STATE;
	} else if (shared.obstacle_ahead){ 
		shared.state = ENCOUNTERED_OBSTACLE;
	} else if (!remote_free_side()){
		shared.state = ALONG_OBSTACLE;
	} else switch(shared.state){
		case TURN_TOWARDS_TARGET:
			if (fabs(diff_angle(shared.thetag,a))<=5*M_PI/180){
				shared.state = TOWARDS_TARGET;
			}
			break;
		case TOWARDS_TARGET:
			if (fabs(diff_angle(shared.thetag,a))>5*M_PI/180){
				shared.state = TURN_TOWARDS_TARGET;
			}
			break;
		case ENCOUNTERED_OBSTACLE:
			if (!shared.obstacle_ahead){
				shared.state = ALONG_OBSTACLE;
			}
			break;
		case ALONG_OBSTACLE:
			if (remote_free_side()){
				shared.state = CIRCUMVENT_OBSTACLE0;
				shared.c_angle = shared.thetag + epsilon*0.5*M_PI_2;
				if (shared.c_angle<0) shared.c_angle += 2*M_PI;
				if (shared.c_angle>2*M_PI) shared.c_angle -= 2*M_PI;
			}
			break;
		case CIRCUMVENT_OBSTACLE0:
			if (fabs(diff_angle(shared.thetag,shared.c_angle))<=5*M_PI/180){
				shared.state = CIRCUMVENT_OBSTACLE1;
			}
			if (angle_within(a, shared.thetag, 0.5*M_PI_2)){
				shared.state = TURN_TOWARDS_TARGET;
			}
			break;
		case CIRCUMVENT_OBSTACLE1:
			break;
		default:
			shared.state = END_STATE;
	}

	if (shared.state == END_STATE) {
		shared.vl = shared.vr = 0.0;
	} else 	if (shared.state == ENCOUNTERED_OBSTACLE){
		shared.vl = epsilon*DDEC*shared.v;
		shared.vr = -shared.vl;
	} else if (shared.state == ALONG_OBSTACLE || shared.state == TOWARDS_TARGET || shared.state == CIRCUMVENT_OBSTACLE1){
		shared.vr = shared.vl = shared.v;
	} else if (shared.state == TURN_TOWARDS_TARGET){
		double eps = 1.0;
		double d = a-shared.thetag;
		if (d>0){
			if (d<M_PI) eps = -1; // left
			else eps = 1.0; //right
		} else {
			if (d>-M_PI) eps = 1.0; // right
			else eps = -1.0; //left
		}	
		//shared.vl = DDEC*((shared.thetag<a)?-shared.v:shared.v);
		shared.vl = DDEC * eps * shared.v;
		shared.vr = -shared.vl;
	} else if (shared.state == CIRCUMVENT_OBSTACLE0){
		shared.vl = -DDEC*epsilon*shared.v;
		shared.vr = -shared.vl;
	}

	if (shared.state != last_state || shared.state == CIRCUMVENT_OBSTACLE0){
		cout << std::left << std::setfill(' ') << setw(10) <<  shared.thetag*180/M_PI 
			<< state_name(shared.state) << " " 
			<< shared.obstacle_left << " " << shared.obstacle_ahead << " "  << shared.obstacle_right;
		if (shared.state==TURN_TOWARDS_TARGET){
			cout <<"\t"<<(shared.vl<0?"left":"right")<<"\tthetag="<<shared.thetag*180/M_PI<<" a=" << a*180/M_PI;
		}	
		cout << endl;
	}
}
