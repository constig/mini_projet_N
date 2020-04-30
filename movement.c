#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <leds.h>

#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <movement.h>
#include "process_image_mod.h"

//static uint8_t intersection = NO_INTERSECTION;

static THD_WORKING_AREA(waMovement, 256);
static THD_FUNCTION(Movement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed_r = 500;
	int16_t speed_l = 500;
	int16_t speed_correction = 0;

    //declaration of variables with values of proximity sensor
    uint16_t capt_diagonal_right;
    uint16_t capt_diagonal_left;
    uint16_t capt_right;
    uint16_t capt_front_right;
    uint16_t capt_left;
    uint16_t capt_front_TOF;
    uint32_t start_step = 0;
    	uint32_t current_step = 0;

    	int compare1;
	int compare2;
	int compare3;
	int compare4;

	uint8_t intersection = NO_INTERSECTION;

    while(1){

    		time = chVTGetSystemTime();
    		speed_r = 500;
    		speed_l = 500;

        //get the values for each sensors
        capt_front_right = get_calibrated_prox(0);
    		capt_diagonal_right = get_calibrated_prox(1);
        capt_right = get_calibrated_prox(2);
     	capt_left = get_calibrated_prox(5);
     	capt_front_TOF = VL53L0X_get_dist_mm();
     	capt_diagonal_left = get_calibrated_prox(6);

		compare1 = capt_left - GOAL_DISTANCE;
		compare2 = capt_right - GOAL_DISTANCE;
		compare3 = capt_diagonal_right - GOAL_DISTANCE1;
		compare4 = capt_diagonal_left - GOAL_DISTANCE1;


		//computes the speed to give to the motors in function of the distance of the sensors of the robot to the wall

		intersection = NO_INTERSECTION;
		if(compare3>60){
			if(compare1>100){
				speed_correction = -5*compare1;
			}
			else if(compare1>300){
				speed_correction = -200;
			}
			else{
				speed_correction = -50;

			}
		}

		else if(compare4>60){
			if(compare2>100){
				speed_correction = 5*compare2;
			}
			else if(compare2>300){
				speed_correction = 200;
			}
			else{
				speed_correction = 50;

			}
		}

		else{ //good position (in interval (-200, 40) (small interval))
				speed_correction = 0;
		}

		if((capt_front_TOF<130)&&(capt_front_TOF>30)){
			speed_r = capt_front_TOF*2;
			speed_l = capt_front_TOF*2;
			speed_correction = 0;
		}

		if((capt_front_TOF<=60) && (capt_right<150) && (capt_left>250)){//only choice is right
			intersection = INTERSECTION_R;
		}

		if((capt_front_TOF<=60) && (capt_right>250) && (capt_left<150) ){ //only choice is left
			intersection = INTERSECTION_L;
		}

		if((capt_front_TOF<=60) && (capt_right>250) && (capt_left>250) ){//turn back
			intersection = INTERSECTION_RETURN;
			set_front_led(1);
		}

		if((capt_front_TOF<=60)&&(capt_right<150) && (capt_left<150) ){//choice between left and right (choose right !)
			intersection = INTERSECTION_RL;
		}


		else if((capt_front_TOF>150) && (capt_right<150) && (capt_left>250) && (capt_diagonal_left>200) ){// choice between front and right (choose right)
			intersection = INTERSECTION_RF;

		}

		else if((capt_front_TOF>150) && (capt_right>250)&& (capt_left<150) && (capt_diagonal_right>200) ){//choice between front and left (choose front!)
			intersection = INTERSECTION_FL;
		}

		else if((capt_front_TOF>150) && (capt_left<150)&& (capt_right<150) ){//choice between left, right, front (choose right!)
			set_body_led(1);
			/*start_step = right_motor_get_pos();
			current_step = right_motor_get_pos();
			right_motor_set_speed(500);
			left_motor_set_speed(500);
			while(current_step-start_step <= 300){
				current_step = right_motor_get_pos();
			}
			right_motor_set_speed(400);
			left_motor_set_speed(-100);
			while(current_step-start_step <= 850){
				current_step = right_motor_get_pos();
			}*/
			intersection = INTERSECTION_RFL;
		}


		if(intersection == NO_INTERSECTION){
			right_motor_set_speed(speed_r - speed_correction);
			left_motor_set_speed(speed_l +  speed_correction);
		}

		else if(intersection == INTERSECTION_R){ //only choice is right
			right_motor_set_speed(-200);
			left_motor_set_speed(400);
			chThdSleepUntilWindowed(time, time + MS2ST(1000));
			right_motor_set_speed(400);
			left_motor_set_speed(400);
			chThdSleepUntilWindowed(time, time + MS2ST(2000));
			intersection = NO_INTERSECTION;
		}

		else if(intersection == INTERSECTION_L){ //only choice is left
			right_motor_set_speed(400);
			left_motor_set_speed(-200);
			chThdSleepUntilWindowed(time, time + MS2ST(1000));
			right_motor_set_speed(400);
			left_motor_set_speed(400);
			chThdSleepUntilWindowed(time, time + MS2ST(2000));
			intersection = NO_INTERSECTION;
		}

		else if(intersection == INTERSECTION_RETURN){ //return
			right_motor_set_speed(300);
			left_motor_set_speed(-300);
			chThdSleepUntilWindowed(time, time + MS2ST(2000));
			intersection = NO_INTERSECTION;
			set_front_led(0);
		}

		else if(intersection == INTERSECTION_RL){ //choice between right and left (right by default)
			right_motor_set_speed(-200);
			left_motor_set_speed(400);
			chThdSleepUntilWindowed(time, time + MS2ST(1000));
			right_motor_set_speed(400);
			left_motor_set_speed(400);
			chThdSleepUntilWindowed(time, time + MS2ST(2000));
			intersection = NO_INTERSECTION;
		}

		else if(intersection == INTERSECTION_RF){ //choice between right and front (right by default)
			right_motor_set_speed(200);
			left_motor_set_speed(200);
			chThdSleepUntilWindowed(time, time + MS2ST(700));
			right_motor_set_speed(-200);
			left_motor_set_speed(400);
			chThdSleepUntilWindowed(time, time + MS2ST(1700));
			right_motor_set_speed(400);
			left_motor_set_speed(400);
			chThdSleepUntilWindowed(time, time + MS2ST(3000));
			intersection = NO_INTERSECTION;
		}

		else if(intersection == INTERSECTION_FL){ ////choice between front and left (front by default)
			right_motor_set_speed(200);
			left_motor_set_speed(200);
			chThdSleepUntilWindowed(time, time + MS2ST(700));
			right_motor_set_speed(400);
			left_motor_set_speed(400);
			chThdSleepUntilWindowed(time, time + MS2ST(1700));
			right_motor_set_speed(400);
			left_motor_set_speed(400);
			chThdSleepUntilWindowed(time, time + MS2ST(3000));
			intersection = NO_INTERSECTION;
		}

		else if(intersection == INTERSECTION_RFL){ ////choice between right, front and left (right by default)
			right_motor_set_speed(200);
			left_motor_set_speed(200);
			chThdSleepUntilWindowed(time, time + MS2ST(700));
			right_motor_set_speed(-200);
			left_motor_set_speed(400);
			chThdSleepUntilWindowed(time, time + MS2ST(1700));
			right_motor_set_speed(400);
			left_motor_set_speed(400);
			chThdSleepUntilWindowed(time, time + MS2ST(3000));
			intersection = NO_INTERSECTION;
			set_body_led(0);
		}


    }
		chThdSleepUntilWindowed(time, time + MS2ST(10)); //100Hz
}



void movement_start(void){
	chThdCreateStatic(waMovement, sizeof(waMovement), NORMALPRIO, Movement, NULL);
}
