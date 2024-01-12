#ifndef __SHARED_H
#define __SHARED_H
#include <stdio.h>
#include "epuck_controller/control/dfa.hpp"

/*
#define NB_ROBOTS_MAX 4
#define NB_OBSTACLES_MAX 5
#define NB_BUCKETS_MAX 4
*/

#define BUFSIZE 1024

typedef struct _shared_t {

	double v,vl,vr;
	double x,y, theta;
	double old_x,old_y, old_theta;
	double xg,yg,thetag;
	double xtarget,ytarget;
	double xtargetg,ytargetg;

	int idx;
	int step;
	state_t state;
	char buf[BUFSIZE];

	double *laser_scan;
	bool obstacle_ahead, obstacle_left, obstacle_right;

	// manage state CIRCUMVENT_OBSTACLE
	double c_angle;
} shared_t;
#endif
