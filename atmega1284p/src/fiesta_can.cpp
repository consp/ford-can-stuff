#ifdef ARDUINO_CORE
#include <SPI.h>
#endif
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "usart.h"

#include "settings.h"
#include "can_data.h"
#include "slcan_def.h"
#include "mcp2515/mcp_can.h"
#include "crc8.h"

#ifdef ARDUINO_CORE
MCPCAN CAN0;
MCPCAN CAN1;
MCPCAN CAN2;
#else
MCPCAN CAN0;
MCPCAN CAN1;
MCPCAN CAN2;
#endif

//#define DEBUG
//#define TIMING

#ifdef TIMING
#ifdef ARDUINO_CORE
#define TIME(message, code) {\
    unsigned long mu = micros();\
\
    code;\
    unsigned long mu2 = micros();\
    sprintf(msgbuf, "\r" message ": %10ld us", mu2 - mu);\
    Serial.println(msgbuf);\
}
#else
#define TIME(message, code) {\
    unsigned long mu = micros();\
\
    code;\
    unsigned long mu2 = micros();\
    sprintf(msgbuf, "\r" message ": %10ld us", mu2 - mu);\
    uart_putstr(msgbuf);\
}
#endif
#else
#define TIME(message, code) code
#endif

char msgbuf[128];
#ifndef PRINTF_DISABLE
#ifdef ARDUINO_CORE
#define printf(frmt, ...) {\
    snprintf(msgbuf, sizeof(msgbuf), frmt, ##__VA_ARGS__);\
    Serial.print(msgbuf);\
}
#else
#define printf(frmt, ...) {\
    memset(msgbuf, 0x00, sizeof(msgbuf));\
    int v = snprintf(msgbuf, sizeof(msgbuf), frmt, ##__VA_ARGS__);\
    for (int i=0; i<v; i++) uart_putc_noblock(msgbuf[i]);\
}
#endif
#else 
#define printf(frmt, ...) 
#endif
#define SLCAN_MAX_SEND_LEN_STRING "B0T001122338001122334455667788"
#define SLCAN_MAX_SEND_LEN (sizeof(SLCAN_MAX_SEND_LEN_STRING))
slcan_binary can_rx[CAN_BUFFER_SIZE]; // sufficiently big
slcan_binary can_tx[CAN_BUFFER_SIZE];
// must be powers of 2!
#define CAN_BUFFER_MASK (CAN_BUFFER_SIZE - 1)

#ifndef ARDUINO_CORE
uint32_t baudreset;
#endif

volatile int8_t can_rx_first = 0, can_tx_first = 0;
volatile int8_t can_rx_last = 0, can_tx_last = 0;

uint8_t slcan_mode[3] = { SLCAN_MODE_CLOSED, SLCAN_MODE_CLOSED, SLCAN_MODE_CLOSED };

uint8_t flags;
uint32_t rcount = 0;
uint8_t overflow = 0;

volatile uint8_t inttrig0 = 0;
volatile uint8_t inttrig1 = 0;
volatile uint8_t inttrig2 = 0;
uint8_t canmode = SLCAN_MODE_BASIC;


uint32_t id;
uint8_t len;
uint8_t buf[8];
uint8_t resp;

uint32_t forgetmark = 0;

uint8_t mcusr_mirror __attribute__ ((section (".noinit")));

uint8_t loopieloop = 0;
uint8_t cmdbuffer[SLCAN_SERIAL_BUFFER_SIZE];
volatile uint8_t cmdbuffer_first = 0;
volatile uint8_t cmdbuffer_last = 0;
uint8_t cmdend = 0;
uint8_t loopdetect = 0;
volatile uint8_t cnt = 0;

#if SLCAN_SERIAL_BUFFER_SIZE != 256
#define SERIAL_RX_MASK ((uint8_t ) (SLCAN_SERIAL_BUFFER_SIZE - 1))

#define MASK(x) (((uint8_t) x) & SERIAL_RX_MASK)
#else
#define SERIAL_RX_MASK 0xFF 
#define MASK(x) ((uint8_t) x)
#endif

#define cmdbuffer_loc(v) (MASK(cmdbuffer_first + v))
#define cmdbuffer_getc(v) cmdbuffer[cmdbuffer_loc(v)]
#define cmdbuffer_empty() (MASK(cmdbuffer_last) == MASK(cmdbuffer_first))
#define cmdbuffer_full() (MASK(cmdbuffer_last + 1) == MASK(cmdbuffer_first))
// allow overwrite
//((cmdbuffer_first - cmdbuffer_last) & (SLCAN_SERIAL_BUFFER_SIZE - 1))
#ifdef RX_BUFFER_DISCARD
#define cmdbuffer_put(c) { cmdbuffer[(MASK(cmdbuffer_last)] = c; cmdbuffer_last++; if (cmdbuffer_full()) cmdbuffer_first++; }
#else
#define cmdbuffer_put(c) { if (!cmdbuffer_full()) { cmdbuffer[MASK(cmdbuffer_last)] = c; cmdbuffer_last++; } }
#endif
#define cmdbuffer_length() MASK(cmdbuffer_last - cmdbuffer_first)
#define cmdbuffer_free() (SLCAN_SERIAL_BUFFER_SIZE - cmdbuffer_length())
#define cmdbuffer_discard(x) { if (MASK(cmdbuffer_first + x) > MASK(cmdbuffer_last)) cmdbuffer_first = cmdbuffer_last; else cmdbuffer_first += x; }
#define cmdbuffer_ends(x) (cmdbuffer_length() >= x && (cmdbuffer_getc(x-1) == SLCAN_OK))

/*
void get_mcusr(void) \
  __attribute__((naked)) \
  __attribute__((section(".init3")));
void get_mcusr(void)
{
  mcusr_mirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}*/

/***
 * interrupts
 */
void ISR_SETUP(void) {
    // set pins as input
    PCIFR = 0x0F;
#ifndef PRINTF_DISABLE
    volatile uint8_t *ddra = (volatile uint8_t *) &DDRA, *ddrb = (volatile uint8_t *) &DDRB, *ddrc = (volatile uint8_t *) &DDRC, *ddrd = (volatile uint8_t *) &DDRD;
    volatile uint8_t *porta = (volatile uint8_t *) &PORTA, *portb = (volatile uint8_t *) &PORTB, *portc = (volatile uint8_t *) &PORTC, *portd = (volatile uint8_t *) &PORTD;
#endif
    DDRA = 0x05;
    DDRB = 0xA7;
    DDRC = 0x00;
    DDRD = 0x01;

    // make CS pins high
    PORTB |= 0x07;

    /*PCICR = 0b00001101;
    PCMSK0 = 0b10000000;
    PCMSK1 = 0x00;
    PCMSK2 = 0b00000001;
    PCMSK3 = 0b10000000;*/
    printf("ISR Set to %02X %02X %02X %02X %02X\r", PCICR, PCMSK0, PCMSK1, PCMSK2, PCMSK3);
    printf("DD: %02X %02X %02X %02X\r", *ddra, *ddrb, *ddrc, *ddrd);
    printf("PORT: %02X %02X %02X %02X\r", *porta, *portb, *portc, *portd);

    inttrig0 = inttrig1 = inttrig2 = 0;
}

ISR (PCINT0_vect) {
    inttrig0 = (~PINA & 0x80);
}

ISR (PCINT1_vect) {
}

ISR (PCINT2_vect) {
    inttrig1 = (~PINC & 0x01);
}

ISR (PCINT3_vect) {
    inttrig2 = (~PIND & 0x80);
}

#define can_tx_available() ((can_tx_last - can_tx_first) & CAN_BUFFER_MASK)
#define can_tx_full() (((can_tx_first - can_tx_last) & CAN_BUFFER_MASK) == 1)

inline void can_tx_put(uint8_t bus, uint32_t id, uint8_t ext, uint8_t rtr, uint8_t len, uint8_t *buf) {
    if (can_tx_full()) {
	    overflow = 1;
        return;
    }
    overflow = 0;

    can_tx_last = (can_tx_last + 1) & CAN_BUFFER_MASK;
#ifdef DEBUG
    printf("CAN %d, %lu, %d, %d", bus, id, rtr, len);
#endif

    if (len > 8) len = 8;

    can_tx[can_tx_last].idfield = IDFIELD(bus, rtr, id);
    can_tx[can_tx_last].len = len;
    memcpy(can_tx[can_tx_last].data, buf, len);
}

inline void can_tx_put_ringbuf(uint8_t loc) {
    if (can_tx_full()) {
	    overflow = 1;
        return;
    }
    overflow = 0;

    can_tx_last = (can_tx_last + 1) & CAN_BUFFER_MASK;
    uint8_t *d = (uint8_t *) &can_tx[can_tx_last];
    for (int eloc = MASK(loc + sizeof(slcan_binary)); loc != eloc;) *d++ = cmdbuffer_getc(loc++); 
}
inline slcan_binary *can_tx_pull(void) {
    slcan_binary *msg;
    if (can_tx_first == can_tx_last) return 0; // nothing available

    msg = &can_tx[can_tx_first];

    can_tx_first = (can_tx_first + 1) & CAN_BUFFER_MASK;
    return msg;
}

inline void can_tx_reset(void) {
    can_tx_first = 0;
    can_tx_last = 0;
}

#define can_rx_available() ((can_rx_last - can_rx_first) & CAN_BUFFER_MASK)
#define can_rx_full() (((can_rx_first+1) & CAN_BUFFER_MASK) == (can_rx_last & CAN_BUFFER_MASK))

inline void can_rx_put(uint8_t bus, uint32_t id, uint8_t ext, uint8_t rtr, uint8_t len, uint8_t *buf) {
    if (can_rx_full()) {
	    overflow = 1;
        return;
    }
    overflow = 0;

    can_rx_last++;
#ifdef DEBUG
    printf("CAN %d, %lu, %d, %d", bus, id, rtr, len);
#endif

    if (len > 8) len = 8;

    can_rx[can_rx_last & CAN_BUFFER_MASK].idfield = IDFIELD(bus, rtr, id);
    can_rx[can_rx_last & CAN_BUFFER_MASK].len = len;
    memcpy(can_rx[can_rx_last & CAN_BUFFER_MASK].data, buf, len);
}

inline slcan_binary *can_rx_pull(void) {
    slcan_binary *msg;

    if (!can_rx_available()) return NULL; // nothing available

    msg = &can_rx[can_rx_first & CAN_BUFFER_MASK];

    can_rx_first++;
    return msg;
}

inline void can_rx_reset(void) {
    can_rx_first = 0;
    can_rx_last = 0;
    overflow = 0;
}

void printhex(char * str, uint8_t *buf, uint8_t len) {
#ifdef ARDUINO_CORE
    Serial.print(str);
    for (int i = 0; i < len; i++) {
        sprintf(msgbuf, "%02X", buf[i]);
        Serial.print(msgbuf);
    }
    Serial.println("");
#else
    uart_putstr(str);
    for (uint8_t i = 0; i < len; i++) uart_puthex(buf[i]);
#endif
}

#define isdigit(x) (x >= '0' && x <= '9')

uint8_t asc2byte(char chr) {
    uint8_t rVal = 0;
    if (isdigit(chr)) {
        rVal = chr - '0';
    } else if (chr >= 'A' && chr <= 'F') {
        rVal = chr + 10 - 'A';
    } else if (chr >= 'a' && chr <= 'f') {
        rVal = chr + 10 - 'a';
    }
    return rVal;
}

inline void unhex(uint8_t *out, uint8_t *in, size_t len) {
    for (; len > 1; len -= 2) {
        uint8_t val = asc2byte(*in++) << 4;
        *out++ = val | asc2byte(*in++);
    }
}

inline void unhex_ringbuf(uint8_t *out, uint8_t start, size_t len) {
    for (; len > 1; len -= 2) {
        uint8_t val = asc2byte(cmdbuffer_getc(start++)) << 4;
        *out++ = val | asc2byte(cmdbuffer_getc(start++));
    }
}

static inline uint8_t slcan_buffer_check(void) {
    
#ifdef ARDUINO_CORE
    int available = Serial.available();
#else
    uint8_t available = uart_AvailableBytes();
#endif

    while (available && !cmdbuffer_full()) {
#ifdef ARDUINO_CORE
        cmdbuffer_put(Serial.read());
#else
        cmdbuffer_put(uart_getc());
#endif
        available--;
    }
    if (cmdbuffer_empty()) return 0;

#ifdef SLCAN_BINARY
    if ((canmode & SLCAN_MODE_BINARY) && cmdbuffer_length() >= sizeof(slcan_binary) && cmdbuffer_getc(0) == SLCAN_BINARY_PREAMBLE) {
        return 2;
    }
#endif
//    if (slcan_mode[0] == SLCAN_MODE_OPEN) {
        printf("QLURP: %hhd-%hhd %hhdv%hhd\r", (uint8_t) cmdbuffer_length(), (uint8_t) cmdbuffer_free(), cmdbuffer_first & (SLCAN_SERIAL_BUFFER_SIZE - 1), cmdbuffer_last & (SLCAN_SERIAL_BUFFER_SIZE - 1));
//    }

    if (loopdetect > 1) {
        cmdbuffer_first = cmdbuffer_last;
        loopdetect = 0;
    }
    return 1;
}

inline void slcan(void) {
    /*
     * read buffer, send data
     */
    uint8_t response_length = 0;
    uint8_t response[32];
    // recieve
    uint8_t ext = 0, rtr = 0;
    uint8_t data[8];
    uint8_t l = 0; //, port = 0;
    uint32_t id = 0;
    uint8_t bus = 255;
    uint16_t i = 0, j, loopdetect;

    MCPCAN *current_can = NULL;
    loopdetect = 0;
    response_length = 0;

    if (!cmdbuffer_empty()) {
        current_can = NULL;
        bus = 255;

#ifdef SLCAN_BINARY 
        // fast send, now with crc!
        if ((canmode & SLCAN_MODE_BINARY) && cmdbuffer_length() >= sizeof(slcan_binary) && cmdbuffer_getc(0) == SLCAN_BINARY_PREAMBLE) {
#if SLCAN_SERIAL_BUFFER_SIZE != 256
            uint8_t bmsg[sizeof(slcan_binary)];
            for (int i = 0; i < sizeof(slcan_binary); i++) bmsg[i] = cmdbuffer_getc(i);
            uint8_t crc = crc8((bmsg), sizeof(slcan_binary) - 1);
#else
            uint8_t crc = crc8_256(cmdbuffer, cmdbuffer_loc(0), sizeof(slcan_binary) - 1);
#endif
            if (crc == cmdbuffer_getc(sizeof(slcan_binary) - 1)) {
                can_tx_put_ringbuf(cmdbuffer_loc(0));
            }
            cmdbuffer_discard(sizeof(slcan_binary));
            return;
        }
#endif

        if (cmdbuffer_getc(0) == SLCAN_CANBUS_SWITCH && cmdbuffer_length() >= 2) {
            switch (cmdbuffer_getc(1)) {
                case 0X31:
                    current_can = &CAN1;
                    bus = 1;
                    break;
                case 0x32:
                    current_can = &CAN2;
                    bus = 2;
                    break;
                case 0x30:
                    current_can = &CAN0;
                    bus = 0;
                    break;
                default:
                    current_can = NULL;
                    bus = 255;
                    response[0] = SLCAN_ERROR;
                    response_length = 1;
                    break;
            }
            cmdbuffer_discard(2);
        }

        switch (cmdbuffer_getc(0)) {
            case SLCAN_CMD_SEND_MODE:
                if (!cmdbuffer_ends(3)) {
                    response[0] = SLCAN_ERROR;
                    response_length = 1;
                } else {
                    switch(cmdbuffer_getc(1)) {
                        case '1':
                            canmode = (canmode & 0xCF) | SLCAN_MODE_BINARY;
                            break;
                        default:
                        case '0':
                            canmode = (canmode & 0xCF) | SLCAN_MODE_BASIC;
                            break;
                    }
                    response[0] = SLCAN_OK;
                    response_length = 1;
                }
                cmdend = 3;
                break;
            case SLCAN_CMD_SERIAL:
                if (!cmdbuffer_ends(2)) {
                    response[0] = SLCAN_ERROR;
                    response_length = 1;
                    break;
                }
                response[0] = SLCAN_CMD_SERIAL;
                memcpy(&response[1], SLCAN_SERIAL, sizeof(SLCAN_SERIAL) - 1);
                response[sizeof(SLCAN_SERIAL)] = SLCAN_OK;
                response_length = sizeof(SLCAN_SERIAL) + 1;
                cmdend = 2;
                break;
            case SLCAN_CMD_VERSION:
                if (!cmdbuffer_ends(2)) {
                    response[0] = SLCAN_ERROR;
                    response_length = 1;
                    break;
                }
                response[0] = SLCAN_CMD_VERSION;
                memcpy(&response[1], SLCAN_VERSION, sizeof(SLCAN_VERSION) - 1);
                response[sizeof(SLCAN_VERSION)] = SLCAN_OK;
                response_length = sizeof(SLCAN_VERSION) + 1;
                cmdend = 2;
                break;
            case SLCAN_CMD_UART_BAUD:
                if (!cmdbuffer_ends(3)) {
                    response[0] = SLCAN_ERROR;
                    response_length = 1;
                } else {
                    uint32_t newspeed = SLCAN_UART_1;
                    switch (cmdbuffer_getc(1)) {
                        case '0':
                            newspeed = SLCAN_UART_0;
                            break;
                        case '1':
                        default:
                            newspeed = SLCAN_UART_1;
                            break;
                        case '2':
                            newspeed = SLCAN_UART_2;
                            break;
                        case '3':
                            newspeed = SLCAN_UART_3;
                            break;
                        case '4':
                            newspeed = SLCAN_UART_4;
                            break;
                        case '5':
                            newspeed = SLCAN_UART_5;
                            break;
                        case '6':
                            newspeed = SLCAN_UART_6;
                            break;
                        case '7':
                            newspeed = SLCAN_UART_7;
                            break;
                        case '8':
                            newspeed = SLCAN_UART_8;
                            break;
                        case '9':
                            newspeed = SLCAN_UART_9;
                            break;
                    }
#ifdef ARDUINO_CORE
                    Serial.end();
                    Serial.begin(newspeed);
#else
                    baudreset = newspeed;
#endif
                    response[0] = SLCAN_OK;
                    response_length = 1;
                }
                cmdend = 3;
                break;


            case SLCAN_CMD_RTR_EXT:
            case SLCAN_CMD_TRANSMIT_EXT:
                if (cmdbuffer_length() < 10) {
                    response[0] = SLCAN_ERROR;
                    response_length = 1;
                    break;
                }
                ext = 1;
            case SLCAN_CMD_RTR:
            case SLCAN_CMD_TRANSMIT:
                if (cmdbuffer_length() < 5) {
                    response[0] = SLCAN_ERROR;
                    response_length = 1;
                    break;
                }
                // set rtr
                if (cmdbuffer_getc(0) == SLCAN_CMD_RTR || cmdbuffer_getc(0) == SLCAN_CMD_RTR_EXT) rtr = 1;
                l = 0;

                if (ext) {
                    unhex_ringbuf(data, 1, 8);
                } else {
                    cmdbuffer_getc(0) = '0';
                    unhex_ringbuf(data, 0, 4);
                }
                // unhex data
                if (ext && ! rtr) {
                    l = cmdbuffer_getc(9) - '0';
                    if (((uint8_t) (l * 2) + 10) > cmdbuffer_length()) {
                        response[0] = SLCAN_ERROR;
                        response_length = 1;
                        break;
                    }
                    id = ((uint32_t) data[0]) << 24 | ((uint32_t) data[1]) << 16 | ((uint32_t) data[2] << 8) | ((uint32_t) data[3]);
                    unhex_ringbuf(data, 10, l*2);
                    cmdend = 10+(l*2)+1;
                } else if (!rtr) {
                    l = cmdbuffer_getc(4) - '0';
                    if (((l * 2) + 5) > cmdbuffer_length()) {
                        response[0] = SLCAN_ERROR;
                        response_length = 1;
                        break;
                    }
                    id = ((uint32_t) data[0] << 8) | ((uint32_t) data[1]);
                    unhex_ringbuf(data, 5, l * 2);
                    cmdend = 5+(l*2)+1;
                }

                if (bus == 255) bus = 0;

                if (l > 8 || id > 0x1FFFFFFF || bus > 2) {
                    response[0] = SLCAN_ERROR;
                    response_length = 1;
                } else {
                    can_tx_put(bus, id, ext, rtr, l, data);
                    response[0] = 'z' - (id > 0x7ff ? 0x20 : 0);
                    response[1] = SLCAN_OK;
                    response_length = 2;
                }

/*                    if (current_can != NULL) {
                    current_can->sendMsgBuf(id, l, data);
                }*/

                break;
            case SLCAN_CMD_BITRATE:
                if (!cmdbuffer_ends(3)) {
                    response[0] = SLCAN_ERROR;
                    response_length = 1;
                    break;
                }
                switch (cmdbuffer_getc(1)) {
                    case '0':
                        l = CAN_10KBPS;
                        break;
                    case '1':
                        l = CAN_20KBPS;
                        break;
                    case '2':
                        l = CAN_50KBPS;
                        break;
                    case '3':
                        l = CAN_100KBPS;
                        break;
                    case '4':
                        l = CAN_125KBPS;
                        break;
                    case '5':
                        l = CAN_250KBPS;
                        break;
                    case '6':
                        l = CAN_500KBPS;
                        break;
                    case '8':
                        l = CAN_1000KBPS;
                        break;
                    case '7':
                    default:
                        l = 255;
                        break;
                }
                if (l == 255) {
                    response[0] = SLCAN_ERROR;
                    response_length = 1;
                } else {
                    uint8_t rv = 0;
                    if (current_can == &CAN1 || current_can == &CAN0) {
                            rv = MCP_CAN_begin(current_can, MCP_ANY, l, MCP_16MHZ | MCP_CLKOUT_ENABLE);
                    } else if (current_can == &CAN2) {
                            rv = MCP_CAN_begin(current_can, MCP_ANY, l, MCP_16MHZ);
                    } else {
                        rv = SLCAN_ERROR;
                    }
                    if (rv == CAN_OK) response[0] = SLCAN_OK;
                    else response[0] = SLCAN_ERROR;
                    response_length = 1;
                }

                cmdend = 3;
                break;
            case SLCAN_CMD_OPEN_NORMAL:
                if (!cmdbuffer_ends(2)) {
                    response[0] = SLCAN_ERROR;
                    response_length = 1;
                    break;
                }
                if (bus == 255) {
                    i = 0; j = 2;
                } else {
                    i = bus; j = bus;
                }
                for (; i <= j; i++) {
                    slcan_mode[i] = SLCAN_MODE_OPEN;
                }
                // reset buffer
                can_rx_reset();
                response[0] = SLCAN_OK;
                response_length = 1;
                cmdend = 2;
                break;
            case SLCAN_CMD_OPEN_LISTEN:
                if (!cmdbuffer_ends(2)) {
                    response[0] = SLCAN_ERROR;
                    response_length = 1;
                    break;
                }
                if (bus == 255) {
                    i = 0; j = 2;
                } else {
                    i = bus; j = bus;
                }
                for (; i <= j; i++) {
                    slcan_mode[i] = SLCAN_MODE_LISTEN;
                }
                // reset buffer
                can_rx_reset();
                response[0] = SLCAN_OK;
                response_length = 1;
                cmdend = 2;
                break;
            case SLCAN_CMD_CLOSE:
                if (!cmdbuffer_ends(2)) {
                    response[0] = SLCAN_ERROR;
                    response_length = 1;
                    break;
                }
                if (bus == 255) {
                    i = 0; j = 2;
                } else {
                    i = bus; j = bus;
                }
                for (; i <= j; i++) {
                    slcan_mode[i] = SLCAN_MODE_CLOSED;
                }
                can_rx_reset();
                response[0] = SLCAN_OK;
                response_length = 1;
                cmdend = 2;
                break;
            case SLCAN_CMD_STATUS_FLAGS:
                if (!cmdbuffer_ends(2)) {
                    response[0] = SLCAN_ERROR;
                    response_length = 1;
                    break;
                }
                flags = 0x00;
                if (can_rx_full()) {
                    flags |= 0x01;
                }
                if (can_tx_full()) {
                    flags |= 0x02;
                }
                response[0] = 'F';
                sprintf((char *) &response[1], "%02X", flags);
                response[3] = SLCAN_OK;
                response_length = 4;
                cmdend = 2;
                break;
            case SLCAN_CMD_ISR:
                if (!cmdbuffer_ends(2)) {
                    response[0] = SLCAN_ERROR;
                    response_length = 1;
                    break;
                }
                response[0] = 'I';
                sprintf((char *) &response[1], "%02X", PCIFR);
                response[3] = SLCAN_OK;
                response_length = 4;
                cmdend = 2;
                break;
            case SLCAN_CMD_TIMESTAMP:
                if (!cmdbuffer_ends(3)) {
                    response[0] = SLCAN_ERROR;
                    response_length = 1;
                    break;
                }
                switch(cmdbuffer_getc(1)) {
                    case '0':
                        canmode &= ~SLCAN_MODE_TIMESTAMP_ENABLED;
                        break;
                    case '1':
                    default:
                        canmode |= SLCAN_MODE_TIMESTAMP_ENABLED;
                        break;
                }
                response[0] = SLCAN_OK;
                response_length = 1;
                cmdend = 3;
                break;
            // not implemented
            case SLCAN_CMD_FILTER:
            // not supported
            case SLCAN_CMD_ACC_CODE:
            case SLCAN_CMD_ACC_MASK:
            case SLCAN_CMD_AUTO_POLL: // always enabled
            case SLCAN_CMD_BITRATE_EXT:
            case SLCAN_CMD_POLL_ALL:
            case SLCAN_CMD_POLL:
            default:
                response[0] = SLCAN_ERROR;
                response_length = 1;
                break;
        }

        if (cmdend == 0) {
            cmdbuffer_discard(1);
        } else  {
            cmdbuffer_discard(cmdend);
        }

        if (response_length > 64) {
            // shoult never happen!
        } else if (response_length > 0) {
#ifdef ARDUINO_CORE
            Serial.write(response, response_length);
#else
            for (uint8_t nb = 0; nb < response_length; nb++) uart_putc_noblock(response[nb]);
#endif
            response_length = 0;
        }
        loopdetect++;
    }

}

unsigned long ms = 0, time_apim = 0, slcounter = 0;

static void setup() {
    //get_mcusr();
    mcusr_mirror = MCUSR;
    MCUSR = 0;
    wdt_disable();
    can_rx_first = can_rx_last = 0; // reset
    can_tx_first = can_tx_last = 0;

    //disablePower(POWER_SERIAL1);
    //disablePower(POWER_ADC);
#ifdef ARDUINO_CORE
    Serial.begin(2000000);
    Serial.setTimeout(10);
#else
    uart_init(DOUBLE_BAUD_CALC(2000000));
#endif

    // setup pins
    ISR_SETUP();



    // clear all interrupts;
    //


    if(MCP_CAN_begin(&CAN0,MCP_ANY, CAN_500KBPS, MCP_16MHZ | MCP_CLKOUT_ENABLE) == CAN_OK) {
#ifdef ARDUINO_CORE
        Serial.print("CAN0: Init OK!\r");
#else
        uart_puts_P("CAN0: Init OK!\r");
#endif
        MCP_CAN_setMode(&CAN0,MCP_NORMAL);
    } else {
#ifdef ARDUINO_CORE
        Serial.print("CAN0: Init Fail!!!\r");
#else
        uart_puts_P("CAN0: Init Fail!!!\r");
#endif
    }

    if(MCP_CAN_begin(&CAN1,MCP_ANY, CAN_125KBPS, MCP_16MHZ | MCP_CLKOUT_ENABLE) == CAN_OK){
#ifdef ARDUINO_CORE
        Serial.print("CAN1: Init OK!\r");
#else
        uart_puts_P("CAN1: Init OK!\r");
#endif
        MCP_CAN_setMode(&CAN1,MCP_NORMAL);
    } else {
#ifdef ARDUINO_CORE
        Serial.print("CAN1: Init Fail!!!\r");
#else
        uart_puts_P("CAN1: Init Fail!!!\r");
#endif
    }

    if(MCP_CAN_begin(&CAN2,MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK){
#ifdef ARDUINO_CORE
        Serial.print("CAN2: Init OK!\r");
#else
        uart_puts_P("CAN2: Init OK!\r");
#endif
        MCP_CAN_setMode(&CAN2,MCP_NORMAL);
    } else {
#ifdef ARDUINO_CORE
        Serial.print("CAN2: Init Fail!!!\r");
#else
        uart_puts_P("CAN2: Init Fail!!!\r");
#endif
    }

    //ms = slcounter = time_apim = millis();

    MCP_CAN_readMsgBuf(&CAN0,&id, &len, buf);
    MCP_CAN_readMsgBuf(&CAN1,&id, &len, buf);
    MCP_CAN_readMsgBuf(&CAN2,&id, &len, buf);

    if (mcusr_mirror & PORF)
#ifdef ARDUINO_CORE
        Serial.print("PWR RESET\r");
#else
        uart_puts_P("PWR RESET\r");
#endif
    if (mcusr_mirror & EXTRF)
#ifdef ARDUINO_CORE
            Serial.print("EXT RESET\r");
#else
            uart_puts_P("EXT RESET\r");
#endif
    if (mcusr_mirror & BORF)
#ifdef ARDUINO_CORE
            Serial.print("BOD RESET\r");
#else
            uart_puts_P("BOD RESET\r");
#endif
    if (mcusr_mirror & WDRF)
#ifdef ARDUINO_CORE
            Serial.print("WDT RESET\r");
#else
            uart_puts_P("WDT RESET\r");
#endif

    //wdt_enable(WDTO_4S);
}

uint8_t sendbuffer[SLCAN_MSG_LEN];
uint8_t sendbuffer_length = 0;
uint16_t timestamp = 0;
uint32_t counter = 0;

inline void cv_hex(uint8_t data, uint8_t *target) {
    uint8_t d1 = (data & 0xF0) >> 4;
    uint8_t d2 = (data & 0x0F);
    target[0] = d1 > 9 ? 'A' + d1 - 10 : '0' + d1;
    target[1] = d2 > 9 ? 'A' + d2 - 10 : '0' + d2;
}

uint16_t speed = 0;
uint16_t rpm = 0;

uint8_t resetcnt = 66;


slcan_binary apim1 = {
    .preamble = 0xaa,
    .idfield = IDFIELD(2, 0, 0x048),
    .len = 8,
    .data = {0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0xE0, 0x00}
};

inline void send_can(uint8_t d, uint32_t id, uint8_t len, uint8_t *data) {
    switch (d) {
        case (1):
            MCP_CAN_sendMsgBuf(&CAN1,id, len, data);
            break;
        case (2):
            MCP_CAN_sendMsgBuf(&CAN2,id, len, data);
            break;
        case (0):
            MCP_CAN_sendMsgBuf(&CAN0,id, len, data);
            break;
        default:
            d = 255;
            break;
    }
#ifdef DEBUG
    printf("%lu, %d, %d", id, d, len);
#endif
}

inline void send_can_packed(slcan_binary *msg) {
    send_can(gpBUS(msg), gpID(msg), msg->len, msg->data);
}


inline void slcan_response(void) {
    slcan_binary *msg;
    uint8_t m = (slcan_mode[0] & 0x03) | (slcan_mode[1] & 0x03) | (slcan_mode[2] & 0x03); // anyone listening
    uint8_t cnt = 0;
    if (m) {
        do {
        // send data
#ifdef ARDUINO_CORE 
            if (Serial.availableForWrite() < 32) break; // not enough space
#endif
            msg = can_rx_pull();

            if (NULL == msg) break;
#if defined(SLCAN_BINARY)
            if (canmode & SLCAN_MODE_BINARY) {
#pragma message "Binary can mode enabled"
                msg->preamble = SLCAN_BINARY_PREAMBLE;
                msg->crc = crc8((uint8_t *) msg, sizeof(slcan_binary) - 1);
#ifdef ARDUINO_CORE
                Serial.write((uint8_t *)msg, sizeof(slcan_binary));
#else
                for (uint8_t nb = 0; nb < sizeof(slcan_binary); nb++) uart_putc_noblock(((uint8_t *) msg)[nb]);
#endif
#endif
#ifdef SLCAN_BASIC 
            } else {
#else   
            }
#endif
#ifdef SLCAN_BASIC 
#pragma message "Basic can mode enabled"
#ifndef SLCAN_BINARY
            {
#endif
                sendbuffer_length = 0;
                int i;
                sendbuffer[sendbuffer_length++] = SLCAN_CANBUS_SWITCH; // set bus
                sendbuffer[sendbuffer_length++] = 0x30 + gpBUS(msg);
                // copy id
                //
                if (gpID(msg) > 2047) {
                    sendbuffer[sendbuffer_length++] = gpRTR(msg) ? SLCAN_CMD_RTR_EXT : SLCAN_CMD_TRANSMIT_EXT;
                    cv_hex((gpID(msg) & 0xFF000000) >> 24, &sendbuffer[sendbuffer_length]);
                    cv_hex((gpID(msg) & 0x00FF0000) >> 16, &sendbuffer[sendbuffer_length+2]);
                    cv_hex((gpID(msg) & 0x0000FF00) >> 8, &sendbuffer[sendbuffer_length+4]);
                    cv_hex((gpID(msg) & 0x000000FF), &sendbuffer[sendbuffer_length+6]);
                    sendbuffer_length += 8;
                } else {
                    cv_hex((gpID(msg) & 0x0000FF00) >> 8, &sendbuffer[sendbuffer_length]);
                    cv_hex((gpID(msg) & 0x000000FF), &sendbuffer[sendbuffer_length+2]);
                    sendbuffer[sendbuffer_length] = gpRTR(msg) ? SLCAN_CMD_RTR : SLCAN_CMD_TRANSMIT;
                    sendbuffer_length += 4;
                }

                sendbuffer[sendbuffer_length++] = 0x30 + msg->len;

                for (i = 0; i < msg->len; i++) {
                    cv_hex(msg->data[i], &sendbuffer[sendbuffer_length]);
                    sendbuffer_length += 2;
                }

                if (canmode & SLCAN_MODE_TIMESTAMP_ENABLED) {
                    cv_hex((timestamp & 0xFF00) >> 8, &sendbuffer[sendbuffer_length]);
                    cv_hex((timestamp & 0x00FF), &sendbuffer[sendbuffer_length+2]);
                    sendbuffer_length += 4;
                }
                sendbuffer[sendbuffer_length++] = SLCAN_OK;
#ifdef ARDUINO_CORE
                Serial.write(sendbuffer, sendbuffer_length);
#else
                for (uint8_t nb = 0; nb < sendbuffer_length; nb++) uart_putc_noblock(sendbuffer[nb]);
#endif
            }
            cnt++;
#endif
        } while (!inttrig0 && !inttrig1 && !inttrig2 && can_rx_available() && cnt < CAN_BUFFER_SIZE);
    }
}

static void loop() {
    /*
     * Read uart, do things, cleanup interrupt buffer, loop.
     */

    PORTA ^= 0x01;

    if (overflow) PORTA ^= 0x04;

    //if (slcan_mode[0] == SLCAN_MODE_OPEN) printf("0");

    if (can_rx_available()) {
        TIME("CAN Response", slcan_response());
    }

    //if (slcan_mode[0] == SLCAN_MODE_OPEN) printf("1");

    if (slcounter > 100) { // 100ms check, takes about 400us on no message
        if (slcan_buffer_check()) slcan();
        slcounter = 0;
    }


#ifdef DEBUG_CAN
    if (counter > 10000) {
        rpm += 100;
        speed += 250;
        uint8_t dd[8] = { (uint8_t) (((rpm * 4) & 0xFF00) >> 8), (uint8_t) ((rpm*4) & 0x00FF), 0x00, 0x00, (uint8_t) ((speed & 0xFF00) >> 8), (uint8_t) (speed & 0x00FF), 0x00, 0x00};
        uint8_t dd2[8] = { 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        TIME("CAN0 Send",
            MCP_CAN_sendMsgBuf(&CAN0,0x201, 8, dd);
            MCP_CAN_sendMsgBuf(&CAN0,0x420, 8, dd2);
        )

        if (rpm > 2000) rpm = 0;
        if (speed > 20000) speed = 0;
        counter = 0;
    }
#endif


#ifdef WAKEUP_APIM
    if (ms > time_apim + 1000) {
        time_apim = ms;
        TIME("CAN0 Send: ", send_can_packed(&apim1));
    }
#endif

    //if (slcan_mode[0] == SLCAN_MODE_OPEN) printf("2");

    // read buffers out of int
    if (inttrig0 && (slcan_mode[0] & 0x03)) {
        resp = MCP_CAN_readMsgBuf(&CAN0,&id, &len, buf);
        if (CAN_OK == resp && id != 0) {
            TIME("CAN0 Store", can_rx_put(CAN_HSCAN, id, (id & 0x80000000) ? 1 : 0 , (id & 0x40000000) ? 1 : 0, len, buf));
            rcount++;
        }
        inttrig0 = 0;
    }

    //if (slcan_mode[0] == SLCAN_MODE_OPEN) printf("3");

    if (inttrig1 && (slcan_mode[1] & 0x03)) {

        resp = MCP_CAN_readMsgBuf(&CAN1,&id, &len, buf);

        if (CAN_OK == resp && id != 0) {
            can_rx_put(CAN_MSCAN, id, (id & 0x80000000) ? 1 : 0 , (id & 0x40000000) ? 1 : 0, len, buf);
            rcount++;
        }
        inttrig1 = 0;
    }

    //if (slcan_mode[0] == SLCAN_MODE_OPEN) printf("4");

    if (inttrig2 && (slcan_mode[2] & 0x03)) {
        resp = MCP_CAN_readMsgBuf(&CAN2,&id, &len, buf);

        if (CAN_OK == resp && id != 0) {
            can_rx_put(CAN_ICAN, id, (id & 0x80000000) ? 1 : 0 , (id & 0x40000000) ? 1 : 0, len, buf);
            rcount++;
        } else {
            
        }
        inttrig2 = 0;
    }

    //if (slcan_mode[0] == SLCAN_MODE_OPEN) printf("5");
   /* if (counter > 1000) {

        uint8_t regs[5];
        regs[0] = MCP_CAN_getError(&CAN0,);
        regs[1] = MCP_CAN_getInt(&CAN0,);
        regs[2] = 0;//rcount & 0x000000FF;
        regs[3] = can_rx_first;
        regs[4] = can_rx_last;
        can_rx_put(CAN_ICAN, 0x12345678, 1, 0, 5, regs);
        rcount++;
        counter = 0;
    }*/


    // send
    if (can_tx_available()) send_can_packed(can_tx_pull());

    //if (slcan_mode[0] == SLCAN_MODE_OPEN) printf("6");

    if (resetcnt > 0) {
        // if int is missed, set
        inttrig0 = (~PINA & 0x80);
        inttrig1 = (~PINC & 0x01);
        inttrig2 = (~PIND & 0x80);
        resetcnt = 0;
        /*volatile uint8_t e0 = MCP_CAN_getError(&CAN0);
        volatile uint8_t e1 = MCP_CAN_getError(&CAN1);
        volatile uint8_t e2 = MCP_CAN_getError(&CAN2);

        uint8_t r0 = MCP_CAN_checkReceive(&CAN0);
        uint8_t r1 = MCP_CAN_checkReceive(&CAN1);
        uint8_t r2 = MCP_CAN_checkReceive(&CAN2);

        uint8_t p0 = *(&PINA);
        uint8_t p1 = *(&PINB);
        uint8_t p2 = *(&PINC);
        uint8_t p3 = *(&PIND);

        volatile uint8_t i0 = MCP_CAN_getInt(&CAN0);
        volatile uint8_t i1 = MCP_CAN_getInt(&CAN1);
        volatile uint8_t i2 = MCP_CAN_getInt(&CAN2);

        printf("%02X %02X %02X # %02X %02X %02X # %02X %02X %02X %02X # %02X %02X %02X\r", 
                e0, e1, e2,
                r0, r1, r2,
                p0, p1, p2, p3,
                i0, i1, i2);*/
    }

    //if (slcan_mode[0] == SLCAN_MODE_OPEN) printf("\r");

    counter++;
    resetcnt++;
    slcounter++;
    //wdt_reset();
    //
#ifndef ARDUINO_CORE
    if (baudreset) {
        uart_init(BAUD_CALC(baudreset));
        baudreset = 0;
    }
#endif
}

#ifndef ARDUINO_CORE
int main(void) {
    setup();
    while(1) loop();
    return 0;
}
#endif

