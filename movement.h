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
#define GOAL_DISTANCE_S_MIN	450
#define GOAL_DISTANCE_S_MAX	550
#define GOAL_DISTANCE_D_MIN 	210
#define GOAL_DISTANCE_D_MAX 	250

//regulator : comparison of the distances to the wall and its goal + speed corrections
#define SMALL_DIFF_DIAG		30
#define SMALL_DIFF_SIDE		60
#define LARGE_DIFF_SIDE		300
#define COEFF				2
#define NO_CORRECTION		0
#define SMALL_CORRECTION		50
#define LARGE_CORRECTION		300

//values of captors
#define MINI_DIST_TOF		30
#define SMALL_DIST_TOF		50
#define NO_CORR_DIST_TOF		80
#define COLOR_DIST_TOF_1		85
#define COLOR_DIST_TOF_2		100
#define MID_DIST_TOF			130
#define LARGE_DIST_TOF		150
#define WALL_CLOSE			250
#define WALL_FAR				150
#define WALL_CLOSE_DIAG		150

//speeds modification
#define SPEED				500
#define SPEED0				0
#define SPEED1				200
#define SPEED2				300
#define SPEED3				400

#define ON					1
#define OFF					0

//starts the movement thread
void movement_start(void);

#endif
