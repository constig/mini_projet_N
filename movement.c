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


static THD_WORKING_AREA(waMovement, 256);
static THD_FUNCTION(Movement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 500; //ABANDON DU PI?????
    int16_t speed_correction = 0;

    //declaration of variables with values of proximity sensor
    uint16_t capt_diagonal_right;
    uint16_t capt_diagonal_left;
    uint16_t capt_right;
    uint16_t capt_left;
    uint16_t capt_front_TOF;
    uint32_t start_step = 0;
    	uint32_t current_step = 0;

    //difference between the distance we want and the actual distance for the right wall (corresponds to the sensors 1 and 2)
    int compare1;
    int compare2;

    while(1){
        
    		set_body_led(1);
    		time = chVTGetSystemTime();

        //get the values for each sensors
        capt_diagonal_right = get_calibrated_prox(1);
        capt_right = get_calibrated_prox(2);
     	capt_left = get_calibrated_prox(5);
     	capt_front_TOF = VL53L0X_get_dist_mm();
     	capt_diagonal_left = get_calibrated_prox(6);


     	compare1 = GOAL_DISTANCE1-capt_diagonal_right;
     	compare2 = GOAL_DISTANCE-capt_right;


     	//computes the speed to give to the motors in function of the distance of the sensors of the robot to the wall
     	//regulator P
     	/*if(compare2<-200){ //robot too close to the wall
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
        }*/

        //applies the correction for the rotation
		right_motor_set_speed(speed - speed_correction);
		left_motor_set_speed(speed +  speed_correction);

		//if((capt_front_right>150) && (capt_front_left>150) && (capt_right>150) && (capt_left<100) ){ //only choice is to turn left
		if((capt_front_TOF<150)&&(capt_right>300) && (capt_left>300)&&(capt_front_TOF>60)){

			set_body_led(1);
			while((capt_front_TOF>60)){
				right_motor_set_speed(500);
				left_motor_set_speed(500);
				capt_front_TOF = VL53L0X_get_dist_mm();
			}
			set_body_led(0);
		}



		else if((capt_front_TOF<70)&&(capt_right<100) && (capt_left>300) ){//only choice is right
			set_front_led(1);
			start_step = left_motor_get_pos();
			current_step = left_motor_get_pos();
			right_motor_set_speed(-200);
			left_motor_set_speed(400);
			while(current_step-start_step <= 400){
				current_step = left_motor_get_pos();
			}
			set_front_led(0);
		}

		else if((capt_front_TOF<70)&&(capt_right>300) && (capt_left<100) ){ //only choice is to turn left
			start_step = right_motor_get_pos();
			current_step = right_motor_get_pos();
			right_motor_set_speed(400);
			left_motor_set_speed(-200);
			while(current_step-start_step <= 400){
				current_step = right_motor_get_pos();
			}
		}

		else if((capt_front_TOF<70)&&(capt_right>300) && (capt_left>300) ){//turn back
			start_step = right_motor_get_pos();
			current_step = right_motor_get_pos();
			right_motor_set_speed(400);
			left_motor_set_speed(-400);
			set_front_led(1);
			while(current_step-start_step <= 630){
				current_step = right_motor_get_pos();
			}
			set_front_led(0);
		}

		else if((capt_front_TOF<70)&&(capt_right<100) && (capt_left<100) ){//choice between left and right (choose right !)
			start_step = right_motor_get_pos();
			current_step = right_motor_get_pos();
			right_motor_set_speed(400);
			left_motor_set_speed(-200);
			while(current_step-start_step <= 400){
				current_step = right_motor_get_pos();
			}
		}


		else if((capt_front_TOF>200) && (capt_diagonal_right<50) && (capt_diagonal_left<50)&& (capt_left>300)&& (capt_right>300) ){//choice between left, right, front (choose right!)
			start_step = right_motor_get_pos();
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
			}

		}// va à gauche

		else if((capt_front_TOF>200) && (capt_right>300)&& (capt_diagonal_right>200) && (capt_diagonal_left<50) && (capt_left>300) ){//choice between left, front (choose front!)
			start_step = right_motor_get_pos();
			current_step = right_motor_get_pos();
			right_motor_set_speed(500);
			left_motor_set_speed(500);
			set_body_led(1);
			while(current_step-start_step <= 450){
				current_step = right_motor_get_pos();
			}
			right_motor_set_speed(400);
			left_motor_set_speed(-100);
			while(current_step-start_step <= 1000){
				current_step = right_motor_get_pos();
			}
			set_body_led(0);
		}

		else if((capt_front_TOF>200) && (capt_diagonal_right<50) &&(capt_right>300)&& (capt_left>300)&& (capt_diagonal_left>200) ){//choice between right, front (choose right!)
			start_step = right_motor_get_pos();
			current_step = right_motor_get_pos();
			right_motor_set_speed(500);
			left_motor_set_speed(500);
			set_body_led(1);
			while(current_step-start_step <= 800){
				current_step = right_motor_get_pos();
			}
			set_body_led(0);
		}


        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void movement_start(void){
	chThdCreateStatic(waMovement, sizeof(waMovement), NORMALPRIO, Movement, NULL);
}
