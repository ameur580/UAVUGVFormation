
#include "epuck_controller/control/parameters.h"

void convert_velocity(double vl, double vr, double *linear, double *angular){
	*linear = WHEEL_RADIUS * (vr + vl) / 2;
	*angular = WHEEL_RADIUS * (vr - vl) / AXLE_LENGTH ;
}
