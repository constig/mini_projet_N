#include "ch.h"
#include "hal.h"
#include <math.h>

//include e-puck library modules
#include <usbcfg.h>
#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <leds.h>
#include <motors.h>

//include created files
#include <main.h>
#include <movement.h>
#include <process_image.h>

/*
 * Thread that manages the robot's movement. It regulates the robot to stay in the middle of
 * the road and retains the color of the road signs to take the right exit at intersections.
 */
static THD_WORKING_AREA(waMovement, 256);
static THD_FUNCTION(Movement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int16_t speed = SPEED;
	int16_t speed_correction = NO_CORRECTION;

    //declaration of variables with values of proximity sensor
    uint16_t capt_diagonal_right;
    uint16_t capt_diagonal_left;
    uint16_t capt_right;
    uint16_t capt_left;
    uint16_t capt_front_TOF;

    //declaration of variable that calculate the width of the road
	uint16_t goal_distance_s = INIT;
	uint16_t goal_distance_s_2 = INIT;
	uint16_t goal_distance_d = INIT;
	uint16_t goal_distance_d_2 = INIT;

    //declaration of variable that compare distances between the wall and their goal distance
    int16_t compare_right;
    int16_t compare_left;
    int16_t compare_diagonal_right;
    int16_t compare_diagonal_left;

	//wall_color : detects when there is a wall/pannel, color_2 : retains color of intersection
    //color_3 : temporary, specific to the red (not to erase the color seen before)
    uint8_t intersection = NO_INTERSECTION;
	uint8_t wall_color = NO_COLOR;
	uint8_t color_2 = NO_COLOR;
	uint8_t color_3 = NO_COLOR;

    while(1){

    		time = chVTGetSystemTime();
    		speed = SPEED;

        //gets the values for each sensors
    		capt_diagonal_right = get_calibrated_prox(1);
        capt_right = get_calibrated_prox(2);
     	capt_left = get_calibrated_prox(5);
     	capt_diagonal_left = get_calibrated_prox(6);
     	capt_front_TOF = VL53L0X_get_dist_mm();

     	goal_distance_s_2 = (capt_right + capt_left)/2;
     	goal_distance_d_2 = (capt_diagonal_right + capt_diagonal_left)/2;

		//to check if the value is plausible
     	if((goal_distance_d_2 > GOAL_DISTANCE_D_MIN) && (goal_distance_d_2 < GOAL_DISTANCE_D_MAX)){
			goal_distance_d = (capt_diagonal_right + capt_diagonal_left)/2;
		}
		if((goal_distance_s_2 > GOAL_DISTANCE_S_MIN) && (goal_distance_s_2 < GOAL_DISTANCE_S_MAX)){
			goal_distance_s = (capt_right + capt_left)/2;
		}

     	//compares sides and diagonals
     	compare_right = capt_right - goal_distance_s;
     	compare_left = capt_left - goal_distance_s;
		compare_diagonal_right = capt_diagonal_right - goal_distance_d;
		compare_diagonal_left = capt_diagonal_left - goal_distance_d;

		//regulator : computes the speed correction
		if(compare_diagonal_left > SMALL_DIFF_DIAG){ //too close to the left wall
			if(compare_left > LARGE_DIFF_SIDE){
				speed_correction = -LARGE_CORRECTION;
			}
			else if(compare_left > SMALL_DIFF_SIDE){
				speed_correction = -compare_left;
			}
			else{
				speed_correction = -SMALL_CORRECTION;
			}
		}

		else if(compare_diagonal_right > SMALL_DIFF_DIAG){ //too close to the right wall
			if(compare_right > LARGE_DIFF_SIDE){
				speed_correction = LARGE_CORRECTION;
			}
			else if(compare_right > SMALL_DIFF_SIDE){
				speed_correction = compare_right;
			}
			else{
				speed_correction = SMALL_CORRECTION;
			}
		}

		else{  //good position
			speed_correction = NO_CORRECTION;
		}

		//detection of a wall and its color (white = wall, color = traffic sign), slows down
		if((capt_front_TOF < MID_DIST_TOF) && (capt_front_TOF > MINI_DIST_TOF)){
			speed = COEFF*capt_front_TOF;
			if((capt_front_TOF > COLOR_DIST_TOF_1) && (capt_front_TOF < COLOR_DIST_TOF_2)){
				wall_color = get_color();
			}
			if(capt_front_TOF < NO_CORR_DIST_TOF){
				speed_correction = NO_CORRECTION;
			}
		}

		//determine if the robot is in an intersection and which one
		if((capt_front_TOF <= SMALL_DIST_TOF) && (capt_right < WALL_FAR) && (capt_left > WALL_CLOSE)){//only choice is right
			intersection = INTERSECTION_R;
		}
		if((capt_front_TOF <= SMALL_DIST_TOF) && (capt_right > WALL_CLOSE) && (capt_left < WALL_FAR)){ //only choice is left
			intersection = INTERSECTION_L;
		}
		if((capt_front_TOF <= SMALL_DIST_TOF) && (capt_right > WALL_CLOSE) && (capt_left > WALL_CLOSE)){//wall (continue or turn back)
			intersection = WALL_COLOR;
		}
		if((capt_front_TOF <= SMALL_DIST_TOF) && (capt_right < WALL_FAR) && (capt_left < WALL_FAR)){//choice between left and right (choose right !)
			intersection = INTERSECTION_RL;
		}
		else if((capt_front_TOF > LARGE_DIST_TOF) && (capt_right < WALL_FAR) && (capt_left > WALL_CLOSE) && (capt_diagonal_left > WALL_CLOSE_DIAG)){// choice between front and right (choose right)
			intersection = INTERSECTION_RF;
		}
		else if((capt_front_TOF > LARGE_DIST_TOF) && (capt_right > WALL_CLOSE) && (capt_left < WALL_FAR) && (capt_diagonal_right > WALL_CLOSE_DIAG)){//choice between front and left (choose front!)
			intersection = INTERSECTION_FL;
		}
		else if((capt_front_TOF > LARGE_DIST_TOF) && (capt_left < WALL_FAR) && (capt_right < WALL_FAR)){//choice between left, right, front (choose right!)
			intersection = INTERSECTION_RFL;
		}

		//regulator if no intersection
		if(intersection == NO_INTERSECTION){
			right_motor_set_speed(speed + speed_correction);
			left_motor_set_speed(speed -  speed_correction);
		}

		//moves the robot at the intersections
		else if(intersection == INTERSECTION_R){ //only choice is right
		    	set_led(LED3, ON);
			right_motor_set_speed(-SPEED1);
			left_motor_set_speed(SPEED3);
			chThdSleepUntilWindowed(time, time + MS2ST(1000));
			right_motor_set_speed(SPEED);
			left_motor_set_speed(SPEED);
			chThdSleepUntilWindowed(time, time + MS2ST(1600));
			intersection = NO_INTERSECTION;
			set_led(LED3, OFF);
		}

		else if(intersection == INTERSECTION_L){ //only choice is left
			set_led(LED7, ON);
			right_motor_set_speed(SPEED3);
			left_motor_set_speed(-SPEED1);
			chThdSleepUntilWindowed(time, time + MS2ST(1000));
			right_motor_set_speed(SPEED);
			left_motor_set_speed(SPEED);
			chThdSleepUntilWindowed(time, time + MS2ST(1600));
			intersection = NO_INTERSECTION;
			set_led(LED7, OFF);
		}

		else if(intersection == WALL_COLOR){ //obstacle
			if(wall_color != RED){
				color_2 = wall_color;
			}
			color_3 = wall_color;
			if(color_3){ //traffic sign
				if(color_3 == RED){ //stop sign : stops for 2sec
					right_motor_set_speed(SPEED0);
					left_motor_set_speed(SPEED0);
					chThdSleepUntilWindowed(time, time + MS2ST(2000));
					right_motor_set_speed(SPEED);
					left_motor_set_speed(SPEED);
					chThdSleepUntilWindowed(time, time + MS2ST(3400));
					intersection = NO_INTERSECTION;
				}
				else{ //other colors:continues and memorizes the color
					right_motor_set_speed(SPEED);
					left_motor_set_speed(SPEED);
					chThdSleepUntilWindowed(time, time + MS2ST(800));
					intersection = NO_INTERSECTION;
				}
			}
			else{ //if white : return
				right_motor_set_speed(SPEED2);
				left_motor_set_speed(-SPEED2);
				chThdSleepUntilWindowed(time, time + MS2ST(2000));
				intersection = NO_INTERSECTION;
			}
		}

		else if(intersection == INTERSECTION_RL){ //choice between right and left (right by default)
			if(color_2 == BLUE){ //goes left
				set_led(LED7, ON);
				right_motor_set_speed(SPEED3);
				left_motor_set_speed(-SPEED1);
				chThdSleepUntilWindowed(time, time + MS2ST(1000));
				set_led(LED7, OFF);
			}
			else{ //if no instruction or false instruction : goes right
				set_led(LED3, ON);
				right_motor_set_speed(-SPEED1);
				left_motor_set_speed(SPEED3);
				chThdSleepUntilWindowed(time, time + MS2ST(1000));
				set_led(LED3, OFF);
			}
			right_motor_set_speed(SPEED);
			left_motor_set_speed(SPEED);
			chThdSleepUntilWindowed(time, time + MS2ST(1700));
			intersection = NO_INTERSECTION;
			color_2 = NO_COLOR;
		}

		else if(intersection == INTERSECTION_RF){ //choice between right and front (right by default)
			right_motor_set_speed(SPEED1);
			left_motor_set_speed(SPEED1);
			chThdSleepUntilWindowed(time, time + MS2ST(800));

			if(color_2 == GREEN){ //goes front
				right_motor_set_speed(SPEED3);
				left_motor_set_speed(SPEED3);
				chThdSleepUntilWindowed(time, time + MS2ST(1800));
			}
			else{
				set_led(LED3, ON);
				right_motor_set_speed(-SPEED1);
				left_motor_set_speed(SPEED3);
				chThdSleepUntilWindowed(time, time + MS2ST(1800));
				set_led(LED3, OFF);
			}
			right_motor_set_speed(SPEED);
			left_motor_set_speed(SPEED);
			chThdSleepUntilWindowed(time, time + MS2ST(2600));
			intersection = NO_INTERSECTION;
			color_2 = NO_COLOR;
		}

		else if(intersection == INTERSECTION_FL){ //choice between front and left (front by default)
			right_motor_set_speed(SPEED1);
			left_motor_set_speed(SPEED1);
			chThdSleepUntilWindowed(time, time + MS2ST(800));
			if(color_2 == BLUE){ //goes left
				set_led(LED7, ON);
				right_motor_set_speed(SPEED3);
				left_motor_set_speed(-SPEED1);
				chThdSleepUntilWindowed(time, time + MS2ST(1800));
				set_led(LED7, OFF);
			}
			else{
				right_motor_set_speed(SPEED3);
				left_motor_set_speed(SPEED3);
				chThdSleepUntilWindowed(time, time + MS2ST(1800));
			}
			right_motor_set_speed(SPEED);
			left_motor_set_speed(SPEED);
			chThdSleepUntilWindowed(time, time + MS2ST(2600));
			intersection = NO_INTERSECTION;
			color_2 = NO_COLOR;
		}

		else if(intersection == INTERSECTION_RFL){ //choice between right, front and left (right by default)
			right_motor_set_speed(SPEED1);
			left_motor_set_speed(SPEED1);
			chThdSleepUntilWindowed(time, time + MS2ST(700));
			if(color_2 == BLUE){ //goes left
				set_led(LED7, ON);
				right_motor_set_speed(SPEED3);
				left_motor_set_speed(-SPEED1);
				chThdSleepUntilWindowed(time, time + MS2ST(1700));
				set_led(LED7, OFF);
			}
			else if(color_2 == GREEN){ //goes front
				right_motor_set_speed(SPEED3);
				left_motor_set_speed(SPEED3);
				chThdSleepUntilWindowed(time, time + MS2ST(1700));
			}
			else{ //goes right
				set_led(LED3, ON);
				right_motor_set_speed(-SPEED1);
				left_motor_set_speed(SPEED3);
				chThdSleepUntilWindowed(time, time + MS2ST(1700));
				set_led(LED3, OFF);
			}
			right_motor_set_speed(SPEED);
			left_motor_set_speed(SPEED);
			chThdSleepUntilWindowed(time, time + MS2ST(2400));
			intersection = NO_INTERSECTION;
			color_2 = NO_COLOR;
		}

		chThdSleepUntilWindowed(time, time + MS2ST(10)); //100Hz
    }
}


void movement_start(void){
	chThdCreateStatic(waMovement, sizeof(waMovement), NORMALPRIO, Movement, NULL);
}
