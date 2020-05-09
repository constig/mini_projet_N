#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ch.h"
#include "hal.h"

//include e-puck library modules
#include "memory_protection.h"
#include <usbcfg.h>
#include <motors.h>
#include <camera/po8030.h>
#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h>

//include created files
#include <main.h>
#include <movement.h>
#include <process_image.h>

//Declaration of IR bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void SendUint8ToComputer(uint8_t* data, uint16_t size) {
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void){
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};
	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void){

    halInit();
    chSysInit();
    mpu_init();

    //bus for proximity sensors
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera and configure
    dcmi_start();
	po8030_start();
	po8030_set_awb(0);
	po8030_set_contrast(1);

	//inits the motors
	motors_init();

	//inits the sensors IR and Time-Of-Flight
	proximity_start();
	VL53L0X_start();

	movement_start();
	process_image_start();

    /* Infinite loop. */
    while (1) {
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
