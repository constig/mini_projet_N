#ifndef MOVEMENT_H_
#define  MOVEMENT_H_

#define NO_INTERSECTION		0
#define INTERSECTION_R		1
#define INTERSECTION_L		2
#define INTERSECTION_RETURN	3
#define INTERSECTION_RL		4
#define INTERSECTION_RF		5
#define INTERSECTION_FL		6
#define INTERSECTION_RFL		7


//starts the movement thread
void movement_start(void);

#endif
