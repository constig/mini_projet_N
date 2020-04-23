#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <sensors/proximity.h>

#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <movement.h>
#include "process_image_mod.h"


static THD_WORKING_AREA(waMovement, 256);
static THD_FUNCTION(Movement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 500; //ABANDON DU PI?????
    int16_t speed_correction = 0;

    //declaration of variables with values of proximity sensor
    int distcapt0;
    int distcapt1;
    int distcapt2;
    int distcapt5;
    int distcapt7;

    //difference between the distance we want and the actual distance for the right wall (corresponds to the sensors 1 and 2)
    int compare1;
    int compare2;

    while(1){
        
    		time = chVTGetSystemTime();

        //get the values for each sensors
        distcapt0 = get_calibrated_prox(0);
        distcapt1 = get_calibrated_prox(1);
        distcapt2 = get_calibrated_prox(2);
     	distcapt5 = get_calibrated_prox(5);
		distcapt7 = get_calibrated_prox(7);


     	compare1 = GOAL_DISTANCE1-distcapt1;
     	compare2 = GOAL_DISTANCE-distcapt2;


     	//computes the speed to give to the motors in function of the distance of the sensors of the robot to the wall

     	if(compare2<-200){ //robot too close to the wall
        		speed_correction = -150;
        }

        else if((compare2>40) && (compare1>40)){ //too far
        		if(compare2<60) {  //almost the good position, just a little correction
        			speed_correction = 80;
        		}
        		else{ //big correction needed
        			speed_correction = 200;
        		}
         }

        else if((compare2>40) && (compare1<-50) ){ //sensor 2 too far because of the orientation but sensor 1 too close (so robot to close)
        		if(compare1>-200) { //little correction needed
        	        speed_correction = -80;
        		}
        	    else{
        	        speed_correction = -200; //big correction needed
        	    }
        }

        else{ //good position (in interval (-200, 40) (small interval))
        		speed_correction = 0;
        }

        //applies the correction for the rotation
		right_motor_set_speed(speed - speed_correction);
		left_motor_set_speed(speed +  speed_correction);


		if((distcapt0>150) && (distcapt7>150) && (distcapt2>150) && (distcapt5<100) ){ //only choice is to turn left
			right_motor_set_speed(500);
			left_motor_set_speed(0);
			chThdSleepUntilWindowed(time, time + MS2ST(1200));
			}

		if((distcapt0>150) && (distcapt7>150) && (distcapt2<100) && (distcapt5>150) ){//only choise is right
			right_motor_set_speed(0);
			left_motor_set_speed(500);
			chThdSleepUntilWindowed(time, time + MS2ST(1200));
			}

		if((distcapt0>150) && (distcapt7>150) && (distcapt2>150) && (distcapt5>150) ){//turn back
			right_motor_set_speed(250);
			left_motor_set_speed(-250);
			chThdSleepUntilWindowed(time, time + MS2ST(2500));
			}

		if((distcapt0>150) && (distcapt7>150) && (distcapt2<100) && (distcapt5<100) ){//choice between left and right (choose right !)
			right_motor_set_speed(0);
			left_motor_set_speed(500);
			chThdSleepUntilWindowed(time, time + MS2ST(1500));
			}

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void movement_start(void){
	chThdCreateStatic(waMovement, sizeof(waMovement), NORMALPRIO, Movement, NULL);
}
