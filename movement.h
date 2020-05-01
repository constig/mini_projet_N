#ifndef MOVEMENT_H_
#define  MOVEMENT_H_

//types of intersection, r = possible to go right, f = front, l = left
#define NO_INTERSECTION		0
#define INTERSECTION_R		1
#define INTERSECTION_L		2
#define WALL_COLOR			3
#define INTERSECTION_RL		4
#define INTERSECTION_RF		5
#define INTERSECTION_FL		6
#define INTERSECTION_RFL		7

//goal distances
#define GOAL_DISTANCE_S 		500
#define GOAL_DISTANCE_D 		250

//regulator : comparison of the distances to the wall and its goal + speed corrections
#define SMALL_DIFF_DIAG		60
#define SMALL_DIFF_SIDE		100
#define LARGE_DIFF_SIDE		300
#define COEFF1				5
#define COEFF2				2
#define NO_CORRECTION		0
#define SMALL_CORRECTION		50
#define LARGE_CORRECTION		200

//values of captors
#define MINI_DIST_TOF		30
#define SMALL_DIST_TOF		60
#define MID_DIST_TOF			130
#define LARGE_DIST_TOF		150
#define WALL_CLOSE			250
#define WALL_FAR				150
#define WALL_CLOSE_DIAG		200

//speeds modification
#define SPEED				500
#define SPEED0				0
#define SPEED1				200
#define SPEED2				300
#define SPEED3				400

//starts the movement thread
void movement_start(void);

#endif
