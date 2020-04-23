#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H


typedef struct{
   uint16_t  begin;
   uint16_t  end;
} RANGE;


float get_distance_cm(void);
//RANGE get_line_position(void);
void process_image_start(void);


#endif /* PROCESS_IMAGE_H */
