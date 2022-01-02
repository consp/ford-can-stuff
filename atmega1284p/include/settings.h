#ifndef __SETTINGS_H__
#define __SETTINGS_H__


#include "pins_arduino.h"

#define CAN0_CS 0 
#define CAN1_CS 1
#define CAN2_CS 2

#define CAN0_INT 31
#define CAN1_INT 16
#define CAN2_INT 15

#define CAN_BUFFER_SIZE 16 

/*
 * using binary mode switches cost of sending to serial from about 180us to 95us
 * sending costs about 350-400 us
 */
#define SLCAN_BASIC
#define SLCAN_BINARY
//#define DEBUG_CAN
//#define WAKEUP_APIM
//#define DEBUG
#endif // __SETTINGS_H__
