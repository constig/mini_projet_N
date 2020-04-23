#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <sensors/proximity.h>

#include <main.h>
#include <motors.h>
#include <pi_regulator.h>

#include "process_image_mod.h"

//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error =abs(goal - distance);

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error; //plus ou moins 1100 avec ki

    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    int distcapt1;
    int distcapt2;
    int distcapt0;
    int distcapt5;
    int distcapt7;

    int compare1;
    int compare2;

    while(1){
        time = chVTGetSystemTime();
        
        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread
        distcapt0 = get_calibrated_prox(0);
        distcapt2 = get_calibrated_prox(2);
     	distcapt1 = get_calibrated_prox(1);
     	distcapt5 = get_calibrated_prox(5);
		distcapt7 = get_calibrated_prox(7);

        //speed = pi_regulator( dist , GOAL_DISTANCE);
     	speed =500;
       // speed = pi_regulator(get_distance_cm(), GOAL_DISTANCE);
     	compare1 = GOAL_DISTANCE1-distcapt1;
     	compare2 = GOAL_DISTANCE-distcapt2;



        if(compare2<-200){ //trop proche
        	speed_correction = -150;
        }
        else if((compare2>40) && (compare1>40) ){ //trop loin
        			if(compare2<60) {
        				speed_correction = 80;
        			}
        			else {
               	speed_correction = 200;
        			}
         }
        else if((compare2>40) && (compare1<-50) ){
        			if(compare1<-200) {
        	        speed_correction = -200;
        	        	}
        	        	else {
        	        speed_correction = -80;
        	        	}

        //speed_correction = -150;
                 }
        else {
        	speed_correction = 0;
        }
        //if the line is nearly in front of the camera, don't rotate
        /*if(abs(speed_correction) < ROTATION_THRESHOLD){
        	speed_correction = 0;
        }*/


        //applies the speed from the PI regulator and the correction for the rotation
		right_motor_set_speed(speed - speed_correction);
		left_motor_set_speed(speed +  speed_correction);


		if((distcapt0>150) && (distcapt7>150) && (distcapt2>150) && (distcapt5<100) ){ //tourner à gauche pas choix
			right_motor_set_speed(500);
			left_motor_set_speed(0);
			chThdSleepUntilWindowed(time, time + MS2ST(1200));
			}

		if((distcapt0>150) && (distcapt7>150) && (distcapt2<100) && (distcapt5>150) ){//Tourner à droite pas choix
			right_motor_set_speed(0);
			left_motor_set_speed(500);
			chThdSleepUntilWindowed(time, time + MS2ST(1200));
			}

		if((distcapt0>150) && (distcapt7>150) && (distcapt2>150) && (distcapt5>150) ){ //Faire demi-tour
			right_motor_set_speed(250);
			left_motor_set_speed(-250);
			chThdSleepUntilWindowed(time, time + MS2ST(2500));
			}

		if((distcapt0>150) && (distcapt7>150) && (distcapt2<100) && (distcapt5<100) ){//Tourner à droite avec choix
			right_motor_set_speed(0);
			left_motor_set_speed(500);
			chThdSleepUntilWindowed(time, time + MS2ST(1500));
			}

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
