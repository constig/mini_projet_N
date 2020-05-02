#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <sensors/proximity.h>
#include <leds.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <movement.h>

#include <regulator.h>
#include "process_image_mod.h"

//Declaration of IR bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	po8030_set_awb(0);
	po8030_set_contrast(1);
	//inits the motors
	motors_init();

	//stars the threads for the pi regulator and the processing of the image
	//pi_regulator_start();


	process_image_start();

	messagebus_init(&bus, &bus_lock, &bus_condvar);
	proximity_start();
	VL53L0X_start();
	//regulator_start();
	//movement_start();

	//    int distance;

	//int capteur2;
	//int capteur1;


    /* Infinite loop. */
    while (1) {

   /* 	capteur2 = get_calibrated_prox(2);
    	capteur1 = get_calibrated_prox(1);
    chprintf((BaseSequentialStream *)&SD3, "la dist petite est %d \n la dist grande est %d \n ", capteur2, capteur1);*/
    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
