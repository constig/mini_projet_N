#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#define IMAGE_BUFFER_SIZE		640

#define INIT						0

//colors : blue = left, green = in front, red = stop, right by default
#define NO_COLOR				0
#define BLUE					1
#define GREEN				2
#define RED					3

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
