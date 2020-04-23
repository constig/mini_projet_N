#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <leds.h>

#include <main.h>
#include <camera/po8030.h>

#include "process_image_mod.h"

#define GREEN_CONVERSION      63/127

static float distance_cm = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle


//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
RANGE extract_line_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;
	RANGE range;
	range.begin = INIT;
	range.end = INIT;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = false;
		//search for a begin
		while(stop == false && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
		    {
		        range.begin = i;
		        stop = true;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = false;
		    
		    while(stop == false && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
		        {
		            range.end = i;
		            stop = true;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        line_not_found = true;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = true;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = range.end;
			range.begin = INIT;
			range.end = INIT;
			stop = false;
			wrong_line = true;
		}
	}while(wrong_line);

	if(line_not_found){
		range.begin = INIT;
		range.end = INIT;
		width = last_width;
	}else{
		last_width = width = (range.end - range.begin);
		line_position = (begin + end)/2; //gives the line position.
	}

	//sets a maximum width or returns the measured width
	if((PXTOCM/width) > MAX_DISTANCE){
		range.begin = INIT;
		range.end = PXTOCM/MAX_DISTANCE;
		return range;
		//chprintf((BaseSequentialStream *)&SD3, "relou \n ");

	}else{
		return range;
		//chprintf((BaseSequentialStream *)&SD3, "ok \n ");

	}
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint16_t lineWidth = 0;
	RANGE range2;

	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		//search for a line in the image and gets its width in pixels
		range2 = extract_line_width(image);
		//chprintf((BaseSequentialStream *)&SD3, "debut de range = %d \n, fin de range = %d ok \n ", width.begin, width.end);

		//converts the width into a distance between the robot and the camera
		if(lineWidth){
			distance_cm = PXTOCM/lineWidth;
		}

		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
    }
}




static THD_WORKING_AREA(waAnalysisImage, 256);
static THD_FUNCTION(AnalysisImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    uint8_t *img_buff_ptr;
    	uint8_t image[IMAGE_BUFFER_SIZE] = {0};


    while(1){
    	//Extracts only the red pixels
    			for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
    				//extracts first 5bits of the first byte
    				//takes nothing from the second byte
    				image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
    			}

    }
}







float get_distance_cm(void){
	return distance_cm;
}

uint16_t get_line_position(void){
	return line_position;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
	chThdCreateStatic(waAnalysisImage, sizeof(waAnalysisImage), NORMALPRIO, AnalysisImage, NULL);
}

