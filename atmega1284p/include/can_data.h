#ifndef __CAN_DATA_H__
#define __CAN_DATA_H__

typedef struct can_data_t {
    uint8_t bus;
    uint32_t id;
    uint8_t ext;
    uint8_t rtr;
    uint8_t len;
    uint8_t data[8];
} can_data;

#define CAN_HSCAN 0
#define CAN_MSCAN 1
#define CAN_ICAN 2

#endif // __CAN_DATA_H__