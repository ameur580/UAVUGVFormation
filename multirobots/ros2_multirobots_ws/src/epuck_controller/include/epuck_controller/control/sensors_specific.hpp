#ifndef __SENSORS_SPECIFIC_H_
#define __SENSORS_SPECIFIC_H_


enum {
	WEST = 0,
	NORTH_WEST_F,
 	NORTH_WEST_C,
 	NORTH_EAST_C,
 	NORTH_EAST_F,
	EAST,
 	SOUTH_EAST,
 	SOUTH_WEST
};

#define AHEAD 4,NORTH_WEST_F,NORTH_WEST_C,NORTH_EAST_C,NORTH_EAST_F
#define LEFT 3,NORTH_WEST_F,WEST,SOUTH_WEST
#define RIGHT 3,NORTH_EAST_F,EAST,SOUTH_EAST

void local_obstacle(double *,bool &ahead, bool &left, bool &right);

#endif
