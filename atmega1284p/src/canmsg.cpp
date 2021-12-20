#include "canmsg.h"
#include <string.h>

canmsg::canmsg(uint32_t i, uint8_t d[8], uint32_t l)
{
	id = i;
	memset(data, 0x00, 8);
	memcpy(data, d, l);
}

CAN_BUS canmsg::bus() {
	return canbus;
}

CAN_SteeringWheelData_2::CAN_SteeringWheelData_2(uint8_t d[8])
	: canmsg(CAN_SteeringWheelData_2::identifier, d, 8)
{

}
