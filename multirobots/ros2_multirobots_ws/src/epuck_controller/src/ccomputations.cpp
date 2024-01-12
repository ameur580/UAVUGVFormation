#include <stdio.h>
#include <math.h>
#include "epuck_controller/ccomputations.hpp"




double angle(double x, double y, double xt, double yt){
	double d,a;
	d = distance(x, y, xt, yt);
	a = atan2((yt-y)/d,(xt-x)/d);
	if (a<0)  a += 2*M_PI;
	return a;
}


double distance(double x1, double y1, double x2, double y2){
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

double s_angle(double x, double y, double xt, double yt){
	double d,a;
	d = distance(x, y, xt, yt);
	a = atan2((yt-y)/d,(xt-x)/d);
	while (a>M_PI)  a -= 2*M_PI;
	while (a<-M_PI)  a += 2*M_PI;
	return a;
}

double diff_angle(double a1, double a2){
	double d = a1 - a2;
	if (d>M_PI) d = 2*M_PI-d;
	if (d<-M_PI) d = 2*M_PI+d;
	return d;
}

char * interval_within(double c, double w, char *interval){
	double amin = c-fabs(w), amax=c+fabs(w);
	if (amin>0){
		if (amax<2*M_PI)
			sprintf(interval,"[%f-%f]",amin*180/M_PI,amax*180/M_PI);
		else
			sprintf(interval,"[%f-%f]U[%f-%f]",0.0,amax*180/M_PI-360,
					amin*180/M_PI,360.0);
	} else {
		if (amax<2*M_PI)
			sprintf(interval,"[%f-%f]U[%f-%f]",0.0,amax*180/M_PI,
					amin*180/M_PI+360,360.0);
		else
			sprintf(interval,"[0-360]");
	}
	return interval;
}

int angle_within(double a, double c, double w){
	double amin = c-fabs(w), amax=c+fabs(w);
	if (amin>0){
		if (amax<2*M_PI)
			return (a>=amin && a<=amax);
		else
			return a>=amin || a<=amax-2*M_PI;
	} else {
		if (amax<2*M_PI)
			return (a>=amin+2*M_PI || a<=amax);
		else
			return 1;
	}
}

