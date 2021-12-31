#ifndef __SLCAN_DEF_H__
#define __SLCAN_DEF_H__

#define SLCAN_ERROR 0x07
#define SLCAN_OK 0x0D

#define SLCAN_CMD_POLL_ALL      0x41
#define SLCAN_CANBUS_SWITCH     0x42
#define SLCAN_CMD_CLOSE         0x43
#define SLCAN_CMD_SEND_MODE     0x44
#define SLCAN_CMD_STATUS_FLAGS  0x46
#define SLCAN_CMD_ISR           0x49
#define SLCAN_CMD_OPEN_LISTEN   0x4C
#define SLCAN_CMD_ACC_CODE      0x4D
#define SLCAN_CMD_SERIAL        0x4E
#define SLCAN_CMD_OPEN_NORMAL   0x4F
#define SLCAN_CMD_POLL          0x50
#define SLCAN_CMD_RTR_EXT       0x52
#define SLCAN_CMD_BITRATE       0x53      
#define SLCAN_CMD_TRANSMIT_EXT  0x54
#define SLCAN_CMD_UART_BAUD     0x55
#define SLCAN_CMD_VERSION       0x56
#define SLCAN_CMD_FILTER        0x57
#define SLCAN_CMD_AUTO_POLL     0x58
#define SLCAN_CMD_TIMESTAMP     0x5A
#define SLCAN_CMD_ACC_MASK      0x6D
#define SLCAN_CMD_RTR           0x72
#define SLCAN_CMD_BITRATE_EXT   0x73
#define SLCAN_CMD_TRANSMIT      0x74

#define SLCAN_UART_0 230400
#define SLCAN_UART_1 115200
#define SLCAN_UART_2 57600
#define SLCAN_UART_3 38400
#define SLCAN_UART_4 9600
#define SLCAN_UART_5 2400
#define SLCAN_UART_6 250000
#define SLCAN_UART_7 500000
#define SLCAN_UART_8 1000000
#define SLCAN_UART_9 2000000



#define SLCAN_SERIAL "1234"
#define SLCAN_VERSION "1013"

#define SLCAN_MODE_CLOSED   0
#define SLCAN_MODE_OPEN     1
#define SLCAN_MODE_LISTEN   2
#define SLCAN_MODE_AUTOSEND_ENABLED 4
#define SLCAN_MODE_TIMESTAMP_ENABLED 8
#define SLCAN_MODE_BASIC    16
#define SLCAN_MODE_BINARY   32

#define SLCAN_MSG_LEN 32

#define SLCAN_BINARY_PREAMBLE 0xA5

#define BUS(n) (n & 0xC0000000)
#define RTR(n) (n & 0x20000000)
#define ID(n)  (n & 0x1FFFFFFF)

#define gBUS(x) ((BUS(x.idfield)) >> 30)
#define gRTR(x) ((RTR(x.idfield)) >> 29)
#define gID(x) (ID(x.idfield))

#define gpBUS(x) ((BUS(x->idfield)) >> 30)
#define gpRTR(x) ((RTR(x->idfield)) >> 29)
#define gpID(x) (ID(x->idfield))

#define IDFIELD(x, y, z) (((uint32_t) x) << 30 | ((uint32_t) y) << 29 | ((uint32_t) z))

typedef struct slcan_binary_t {
    uint8_t preamble; 
    uint32_t idfield;
    uint8_t len;
    uint8_t data[8];
    uint8_t crc;
} slcan_binary;


#endif //__SLCAN_DEF_H__
