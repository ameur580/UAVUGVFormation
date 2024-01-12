#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <iostream>
#include "epuck_controller/control/parameters.h"
#include "epuck_controller/control/sensors_specific.hpp"
#include "epuck_controller/control/shared.hpp"

#define S_THRESHOLD 0.03 

extern shared_t shared;

using namespace std;

bool check (double *a, int nb, ...){
	bool r = false;
	int i=0;
	int idx;
	va_list args;
	va_start(args, nb);
	while (!r && i<nb){
		idx = va_arg(args,int);
		r = r || (a[idx]<S_THRESHOLD);
		i++;
	}
	va_end(args);
	return r;
}

void local_obstacle(double *laser_scan, bool &ahead, bool &left, bool &right){
	right = left = ahead = false;
	ahead = check(&laser_scan[0],AHEAD);
	left = check(&laser_scan[0],LEFT);
	right = check(&laser_scan[0],RIGHT);
}
