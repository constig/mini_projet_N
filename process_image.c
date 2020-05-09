#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

static uint8_t color = NO_COLOR;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 * extracts the blue pixels and normalizes the values
 * returns true if the image is blue
 */
bool extract_line_width_b(uint8_t *buffer){

	uint16_t mean = INIT;
	uint16_t high_mean = INIT;
	int sommation = INIT; //total sum
	int sommation_s = INIT; //sum of the sides
	bool consecutive_lines = false;

	//performs an average of the entire line
	for(uint16_t i = INIT ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}

	mean /= IMAGE_BUFFER_SIZE;
	high_mean = COEFF1*mean;

	for(uint16_t i = INIT ; i < IMAGE_BUFFER_SIZE ; i++){
		if(COEFF2*buffer[i] > high_mean){
			buffer[i] = HIGH_VALUE;
		}
		else{
			buffer[i] = LOW_VALUE;
		}
	}

	for(uint16_t i = IMAGE_CENTER_BEGIN ; i < IMAGE_CENTER_END ; i++){
		if (buffer[i] == HIGH_VALUE){
			sommation += buffer[i];
		}
		else{
			sommation = LOW_VALUE;
		}
		if (sommation > CONSECUTIVE_LINES_B){
			consecutive_lines = true;
		}
	}

	for(uint16_t i = SUM_RIGHT_BEGIN ; i < SUM_RIGHT_END ; i++){
		sommation_s += buffer[i];
	}
	for(uint16_t i = SUM_LEFT_BEGIN ; i < IMAGE_BUFFER_SIZE; i++){
		sommation_s += buffer[i];
	}

	if((consecutive_lines) && (sommation_s < MAX_PEAKS_B)){
		return true;
	}
	else{
		return false;
	}
}

/*
 * extracts the red pixels and normalizes the values
 * returns true if the image is red
 */
bool extract_line_width_r(uint8_t *buffer){

	uint32_t mean = INIT;
	uint16_t high_mean = INIT;
	int sommation = INIT;
	int sommation_s = INIT; //=0 if wrong color
	bool consecutive_lines = false;

	//performs an average
	for(uint16_t i = INIT ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}

	mean /= IMAGE_BUFFER_SIZE;
	high_mean = COEFF1*mean;

	for(uint16_t i = INIT ; i < IMAGE_BUFFER_SIZE ; i++){
		if(COEFF2*buffer[i] > high_mean){
			buffer[i] = HIGH_VALUE;
		}
		else{
			buffer[i] = LOW_VALUE;
		}
	}

	for(uint16_t i = IMAGE_CENTER_BEGIN ; i < IMAGE_CENTER_END ; i++){
		if (buffer[i] == HIGH_VALUE){
			sommation += buffer[i];
		}
		else{
			sommation = LOW_VALUE;
		}
		if (sommation > CONSECUTIVE_LINES_R){
			consecutive_lines = true;
		}
	}

	for(uint16_t i = SUM_RIGHT_BEGIN ; i < SUM_RIGHT_END ; i++){
		sommation_s += buffer[i];
	}
	for(uint16_t i = SUM_LEFT_BEGIN ; i < IMAGE_BUFFER_SIZE; i++){
		sommation_s += buffer[i];
	}

	if((consecutive_lines) && (sommation_s < MAX_PEAKS_R)){ //180 hautes valeurs
		return true;
	}
	else{
		return false;
	}
}

/*
 * extracts the green pixels and normalizes the values
 * returns true if the image is green
 */
bool extract_line_width_g(uint8_t *buffer){

	uint32_t mean = INIT;
	uint16_t high_mean = INIT;
	int sommation = INIT;

	//performs an average
	for(uint16_t i = INIT ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}

	mean /= IMAGE_BUFFER_SIZE;
	high_mean = COEFF1*mean;


	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		if(COEFF2*buffer[i] > high_mean){
			buffer[i] = HIGH_VALUE;
		}
		else{
			buffer[i] = LOW_VALUE;
		}
	}

	for(uint16_t i = IMAGE_CENTER_BEGIN ; i < IMAGE_CENTER_END ; i++){
		sommation += buffer[i];
	}

	if(sommation > CONSECUTIVE_LINES_G){ //180 hautes valeurs
		return true;
	}
	else{
		return false;
	}
}

/* CaptureImage thread that takes the image and deals with its pixels
 */
static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 350 + 351
	po8030_advanced_config(FORMAT_RGB565, 0, 350, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
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

/* ProcessImage thread that analyzes the image in order to
 * determine the color seen
 */
static THD_WORKING_AREA(waProcessImage, 4096);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image_r[IMAGE_BUFFER_SIZE] = {0};//red
	uint8_t image_b[IMAGE_BUFFER_SIZE] = {0};//blue
	uint8_t image_g[IMAGE_BUFFER_SIZE] = {0};//green

	bool send_to_computer = true;
	bool red = false;
	bool blue = false;
	bool green = false;

	//count : calculate the number of time a color has been seen
	uint8_t count_r = INIT;
	uint8_t count_b = INIT;
	uint8_t count_g = INIT;
	uint8_t count_w = INIT;

    while(1){
    		//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			image_r[i/2] = ((uint8_t)img_buff_ptr[i]&0xF8)>>3; //extracts red pixels
			image_b[i/2] = (uint8_t)(img_buff_ptr[i+1]&0x1F); //extracts blue pixels
			image_g[i/2] = ((uint8_t)(img_buff_ptr[i]&0X07)<<5)|((uint8_t)(img_buff_ptr[i+1]&0XE0)>>3);
		}

		//true if it is the color founded
		red = extract_line_width_r(image_r);
		blue = extract_line_width_b(image_b);
		green = extract_line_width_g(image_g);

		if(red){
			count_r++;
			if(count_r >= GOAL_COUNT_COLOR){
				color = RED;
				count_r = INIT;
				count_b = INIT;
				count_g = INIT;
				count_w = INIT;
			}
		}

		if(blue){
			count_b++;
			if(count_b >= GOAL_COUNT_COLOR){
				color = BLUE;
				count_r = INIT;
				count_b = INIT;
				count_g = INIT;
				count_w = INIT;
			}
		}

		if(green){
			count_g++;
			if(count_g >= GOAL_COUNT_COLOR){
				color = GREEN;
				count_r = INIT;
				count_b = INIT;
				count_g = INIT;
				count_w = INIT;
			}
		}

		if((!red) && (!blue) && ((!green))){
			count_w++;
			if(count_w >= GOAL_COUNT_NO_COLOR){
				color = NO_COLOR;
				count_r = INIT;
				count_b = INIT;
				count_g = INIT;
				count_w = INIT;
			}
		}

		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image_r, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
    }
}


uint8_t get_color(void){
	return color;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
