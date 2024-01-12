#ifndef __CCOMPUTATIONS_H
#define __CCOMPUTATIONS_H

double angle(double x, double y, double xt, double yt);
double distance(double x1, double y1, double x2, double y2);
double diff_angle(double a1, double a2);
int angle_within(double a, double c, double w);
char * interval_within(double c, double w, char *);

#endif
