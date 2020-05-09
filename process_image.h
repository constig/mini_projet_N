#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H


void process_image_start(void);
uint8_t get_color(void);

#define COEFF1					13
#define COEFF2					10

//values for the normalization of the color
#define HIGH_VALUE				1
#define LOW_VALUE				0

//values to scrool through the pixels of the image
#define IMAGE_CENTER_BEGIN		100
#define IMAGE_CENTER_END			500

//to look for consecutive values 1 to determine the color
#define CONSECUTIVE_LINES_B		40
#define CONSECUTIVE_LINES_R		100
#define CONSECUTIVE_LINES_G		80

//to check if there are peaks on the side of the image to determine
//if the color founded is false
#define SUM_RIGHT_BEGIN  		0
#define SUM_RIGHT_END	  		50
#define SUM_LEFT_BEGIN	  		590

#define MAX_PEAKS_B		  		20
#define MAX_PEAKS_R		  		8

//if a color is founded enough time, it is the right one
#define GOAL_COUNT_COLOR			10
#define GOAL_COUNT_NO_COLOR		30


#endif /* PROCESS_IMAGE_H */
