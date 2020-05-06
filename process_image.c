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


bool extract_line_width_b(uint8_t *buffer){

	uint32_t mean = 0;
	uint16_t high_mean = 0;
	int sommation = 0;
	int sommation_s = 0; //sum of the sides
	bool consecutive_lines = false;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}


	mean /= IMAGE_BUFFER_SIZE;
	high_mean = 13*mean;


	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		if(10*buffer[i] > high_mean){
			buffer[i] = 1;
		}
		else{
			buffer[i] = 0;
		}
	}

	for(uint16_t i = 100 ; i < 500 ; i++){
		if (buffer[i] == 1){
			sommation += buffer[i];
		}
		else{
			sommation = 0;
		}
		if (sommation > 40){
			consecutive_lines = 1;
		}
	}

	for(uint16_t i = 0 ; i < 50 ; i++){
		sommation_s += buffer[i];
	}
	for(uint16_t i = 590 ; i < IMAGE_BUFFER_SIZE; i++){
		sommation_s += buffer[i];
	}

	if((consecutive_lines) && (sommation_s < 20)){
		return true;
	}
	else{
		return false;
	}
}

bool extract_line_width_r(uint8_t *buffer){

	uint32_t mean = 0;
	uint16_t high_mean = 0;
	int sommation = 0;
	int sommation_s = 0; //=0 if wrong color
	bool consecutive_lines = false;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}


	mean /= IMAGE_BUFFER_SIZE;
	high_mean = 13*mean;


	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		if(10*buffer[i] > high_mean){
			buffer[i] = 1;
		}
		else{
			buffer[i] = 0;
		}
	}

	for(uint16_t i = 100 ; i < 500 ; i++){
		if (buffer[i] == 1){
			sommation += buffer[i];
		}
		else{
			sommation = 0;
		}
		if (sommation > 100){
			consecutive_lines = true;
		}
	}

	for(uint16_t i = 0 ; i < 50 ; i++){
		sommation_s += buffer[i];
	}
	for(uint16_t i = 590 ; i < IMAGE_BUFFER_SIZE; i++){
		sommation_s += buffer[i];
	}

	if((consecutive_lines) && (sommation_s < 8)){ //180 hautes valeurs
		return true;
	}
	else{
		return false;
	}
}

bool extract_line_width_g(uint8_t *buffer){

	uint32_t mean = 0;
	uint16_t high_mean = 0;
	int sommation = 0;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}


	mean /= IMAGE_BUFFER_SIZE;
	high_mean = 13*mean;


	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		if(10*buffer[i] > high_mean){
			buffer[i] = 1;
		}
		else{
			buffer[i] = 0;
		}
	}

	for(uint16_t i = 100 ; i < 500 ; i++){
		sommation += buffer[i];
	}


	if(sommation > 80){ //180 hautes valeurs
		return true;
	}
	else{
		return false;
	}
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
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


static THD_WORKING_AREA(waProcessImage, 4096); //640 + un tab < 1024 donc si on ajoute 2 tableaux (1920 octets pour 3 tableaux donc il faut une marge
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image_r[IMAGE_BUFFER_SIZE] = {0};//red
	uint8_t image_b[IMAGE_BUFFER_SIZE] = {0};//blue
	uint8_t image_g[IMAGE_BUFFER_SIZE] = {0};//green

	bool send_to_computer = true;
	bool red = 0;
	bool blue = 0;
	bool green = 0;
	uint16_t count_r = 0;
	uint16_t count_b = 0;
	uint16_t count_g = 0;
	uint16_t count_w = 0;
	int test = 0;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			image_r[i/2] = ((uint8_t)img_buff_ptr[i]&0xF8)>>3; //extracts red pixels
			image_b[i/2] = (uint8_t)(img_buff_ptr[i+1]&0x1F); //extracts blue pixels
			image_g[i/2] = ((uint8_t)(img_buff_ptr[i]&0X07)<<5)|((uint8_t)(img_buff_ptr[i+1]&0XE0)>>3);
			//image_g[i/2] = ((uint8_t)(img_buff_ptr[i]&0X07)<<3)|((uint8_t)(img_buff_ptr[i+1]&0XE0)>>5); //extract green pixels
		}

		//red = extract_line_width_r(image_r);
		blue = extract_line_width_b(image_b);
		green = extract_line_width_g(image_g);

		//chprintf((BaseSequentialStream *)&SDU1, "vert : %d, bleu : %d, le rouge : %d \r \n ", green, blue, red);

		if(red){
			count_r++;
			if(count_r >= 10){
				color = RED;
				count_r = 0;
				count_b = 0;
				count_g = 0;
				count_w = 0;
			}
		}

		if(blue){
			count_b++;
			if(count_b >= 10){
				color = BLUE;
				count_r = 0;
				count_b = 0;
				count_g = 0;
				count_w = 0;
			}
		}

		if(green){
			count_g++;
			if(count_g >= 10){
				color = GREEN;
				count_r = 0;
				count_b = 0;
				count_g = 0;
				count_w = 0;
			}
		}

		if((!red) && (!blue) && ((!green))){
			count_w++;
			if(count_w >= 30){
				color = NO_COLOR;
				count_r = 0;
				count_b = 0;
				count_g = 0;
				count_w = 0;
			}
		}

		test++;

		/*if(test>20){
			chprintf((BaseSequentialStream *)&SDU1, "couleur: %d \r \n ", color);
			test = 0;
		}*/

		//int count_r=0;
		//int count_r=0;

		//red detected
		/*if((red>blue) && (red>green)&&(red>200)){
			set_body_led(1);
		}
		else{
			set_body_led(0);
		}*/

		/*if((blue>red) && (blue>green)&&(blue>200)){
			set_front_led(1);
		}
		else{
			set_front_led(0);
		}*/

		/*if((green>blue) && (green>red)&&(green>200)){
			set_body_led(1);
		}
		else{
			set_body_led(0);
		}*/


		//search for a line in the image and gets its width in pixels
		//lineWidth = extract_line_width(image);


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
