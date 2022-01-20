#ifdef ARDUINO_CORE
#include <SPI.h>
#endif
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

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
//
// Disable printf if using global registers
#ifdef GLOBAL_COUNTER
register uint8_t slcounter asm("r2");
#ifndef PRINTF_DISABLE
#define PRINTF_DISABLE
#endif
#else
static uint8_t slcounter;
#endif

#define DISCARD_INTERVAL 4096 
#if defined(TIMING)
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
    uart_put(msgbuf);\
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
    for (int i=0; i<v; i++) uart_putc(msgbuf[i]);\
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
uint16_t baudreset;
#endif

volatile int8_t can_rx_first = 0, can_tx_first = 0;
volatile int8_t can_rx_last = 0, can_tx_last = 0;

static uint8_t slcan_mode[3] = { SLCAN_MODE_CLOSED, SLCAN_MODE_CLOSED, SLCAN_MODE_CLOSED };
static uint8_t canmode = SLCAN_MODE_BASIC;

uint8_t flags;
uint32_t rcount = 0;
uint8_t overflow = 0;

volatile uint8_t inttrig0 = 0;
volatile uint8_t inttrig1 = 0;
volatile uint8_t inttrig2 = 0;


uint32_t forgetmark = 0;

uint8_t mcusr_mirror __attribute__ ((section (".noinit")));

uint8_t loopieloop = 0;
volatile uint8_t cnt = 0;
#define SERIAL_RX_BUFFER_SIZE 256
#define SERIAL_TX_BUFFER_SIZE 256
/**
 * Serial communication
 */

// since we have enough memory, always use 256 bytes
uint8_t serial_rx[SERIAL_RX_BUFFER_SIZE];
uint8_t serial_tx[SERIAL_TX_BUFFER_SIZE];
register uint8_t serial_rx_first asm("r3");
register uint8_t serial_rx_last asm("r4");
register uint8_t serial_tx_first asm("r5");
register uint8_t serial_tx_last asm("r6");
#define SERIAL_RX_MASK (SERIAL_RX_BUFFER_SIZE - 1)
#define MASK(x) ((x) & SERIAL_RX_MASK)

#define serial_rx_loc(v) (serial_rx_first + v)
#define serial_rx_empty() (serial_rx_last == serial_rx_first)
#define serial_rx_full() ((serial_rx_last + 1) == serial_rx_first)
#define serial_rx_getc(x) serial_rx[(uint8_t) (serial_rx_first + x)]
#define serial_rx_length() ((uint8_t)((serial_rx_last - serial_rx_first)))
#define serial_rx_free() ((SERIAL_RX_BUFFER_SIZE - 1) - serial_rx_length())

#define serial_tx_loc(v) (serial_tx_first + v)
#define serial_tx_empty() (serial_tx_last == serial_tx_first)
#define serial_tx_full() ((serial_tx_last + 1) == serial_tx_first)
#define serial_tx_getc(x) serial_tx[(uint8_t) (serial_tx_first + x)]
#define serial_tx_length() ((uint8_t)((serial_tx_last - serial_tx_first)))
#define serial_tx_free() ((SERIAL_TX_BUFFER_SIZE - 1) - serial_tx_length())
static inline void unhex(uint8_t *out, uint8_t *in, size_t len);
static inline void unhex_ringbuf(uint8_t *out, uint8_t start, size_t len);
static inline void hex(uint8_t *out, const uint8_t data);

#define RX_INTE() UCSR0B |= (1 << RXCIE0)
#define RX_INTD() UCSR0B &= ~(1 << RXCIE0)

#define TX_INTE() UCSR0B |= (1 << UDRE0)
#define TX_INTD() UCSR0B &= ~(1 << UDRE0)

#define BAUD(baud) (((F_CPU) / (8UL * baud)) - 1)

static void uart_putc(const uint8_t c) {
    serial_tx[serial_tx_last++] = c;
    TX_INTE();
}

#define serial_rx_discard(x) serial_rx_first = ((uint8_t) serial_rx_first + x)


static void uart_putb(const uint8_t *c, uint8_t l) {
    TX_INTD();

    if (serial_tx_free() < l) l = serial_tx_free();
    while(l--) {
        serial_tx[serial_tx_last++] = *c++;
    }

    TX_INTE();
}

static void uart_puts(const char *c) {
    while(*c != '\0') uart_putc(*c++);
}

static void uart_init(uint16_t baudrate) {
    UBRR0H = (baudrate >> 8);
    UBRR0L = (baudrate & 0xFF);


    UCSR0A = (1 << RXC0) | (1 << UDRE0) | (1 << U2X0); // clear ints, enable double speed
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << UDRE0) | (1 << RXCIE0); // enable ints and 
    UCSR0C = (1 << USBS0) | (1 << UCSZ00) | (1 << UCSZ01);
}

// ISRs for Serial
ISR (USART0_RX_vect) {
    serial_rx[serial_rx_last++] = UDR0;
}

ISR (USART0_UDRE_vect) {
    TX_INTD();
    if (serial_tx_first != serial_tx_last) {
        UDR0 = serial_tx[serial_tx_first++];
        TX_INTE();
    }
}

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
    DDRA = 0x05;
    DDRB = 0xA7;
    DDRC = 0x00;
    DDRD = 0x01;

    // make CS pins high
    PORTB |= 0x07;

    PCICR = 0b00001101;
    PCMSK0 = 0b10000000;
    PCMSK1 = 0x00;
    PCMSK2 = 0b00000001;
    PCMSK3 = 0b10000000;

    inttrig0 = inttrig1 = inttrig2 = 0;

    serial_rx_first = 0; serial_rx_last = 0;
    serial_tx_first = 0; serial_tx_last = 0;
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

#define can_tx_available() (can_tx_last != can_tx_first)
#define can_tx_full() (((can_tx_last+1) & CAN_BUFFER_MASK) == (can_tx_first & CAN_BUFFER_MASK))

static inline void can_tx_put(uint8_t b, uint32_t id, uint8_t ext, uint8_t rtr, uint8_t len, uint8_t *buf) {
    if (((can_tx_last + 1) & CAN_BUFFER_MASK) == (can_tx_first)) {
	    overflow = 1;
        return;
    }
    overflow = 0;

#ifdef DEBUG
    printf("CAN %d, %lu, %d, %d", bus, id, rtr, len);
#endif

    if (len > 8) len = 8;

    can_tx[can_tx_last].idfield = IDFIELD(b, rtr, id);
    can_tx[can_tx_last].len = len;
    memcpy(can_tx[can_tx_last].data, buf, len);
    can_tx_last = (can_tx_last + 1) & CAN_BUFFER_MASK;
}

static inline void can_tx_put_ringbuf(uint8_t loc) {
    if (((can_tx_last + 1) & CAN_BUFFER_MASK) == (can_tx_first)) {
	    overflow = 1;
        uart_putc(can_tx_last);
        uart_putc(can_tx_first);
        return;
    }
    overflow = 0;

    uint8_t *d = (uint8_t *) &can_tx[can_tx_last];
    uint8_t end = sizeof(slcan_binary) + loc;
    while(loc < end) *d++ = serial_rx_getc(loc++); 
    can_tx_last = (can_tx_last + 1) & CAN_BUFFER_MASK;
}
static inline slcan_binary *can_tx_pull(void) {
    slcan_binary *msg;
    if (can_tx_first == can_tx_last) return 0; // nothing available

    msg = &can_tx[can_tx_first];

    can_tx_first = (can_tx_first + 1) & CAN_BUFFER_MASK;
    return msg;
}

static inline void can_tx_reset(void) {
    can_tx_first = 0;
    can_tx_last = 0;
}

#define can_rx_available() (can_rx_last != can_rx_first)
#define can_rx_full() (((can_rx_last+1) & CAN_BUFFER_MASK) == (can_rx_first & CAN_BUFFER_MASK))

static inline void can_rx_put(uint8_t b, uint32_t id, uint8_t ext, uint8_t rtr, uint8_t len, uint8_t *buf) {
    if (can_rx_full()) {
        return;
    }
    overflow = 0;

#ifdef DEBUG
    printf("CAN %d, %lu, %d, %d", b, id, rtr, len);
#endif

    if (len > 8) len = 8;

    can_rx[can_rx_last].idfield = IDFIELD(b, rtr, id);
    can_rx[can_rx_last].len = len;
    memcpy(can_rx[can_rx_last].data, buf, len);
    can_rx_last = (can_rx_last + 1) & CAN_BUFFER_MASK;
}

static inline slcan_binary *can_rx_pull(void) {
    slcan_binary *msg;

    if (!can_rx_available()) return NULL; // nothing available

    msg = &can_rx[can_rx_first];
    can_rx_first = (can_rx_first + 1) & CAN_BUFFER_MASK;
    return msg;
}

static inline void can_rx_reset(void) {
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
    uart_puts(str);
    uint8_t tmp;
    for (uint8_t i = 0; i < len; i++) {
        hex(&tmp, buf[i]);
        uart_putc(tmp);
    }
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

static inline void unhex(uint8_t *out, uint8_t *in, size_t len) {
    for (; len > 1; len -= 2) {
        uint8_t val = asc2byte(*in++) << 4;
        *out++ = val | asc2byte(*in++);
    }
}

static inline void unhex_ringbuf(uint8_t *out, uint8_t start, size_t len) {
    if (len % 2 == 1) {
        *out++ = asc2byte(serial_rx_getc(start++));
        len--;
    }
    for (; len > 1; len -= 2) {
        uint8_t val = asc2byte(serial_rx_getc(start++)) << 4;
        *out++ = val | asc2byte(serial_rx_getc(start++));
    }
}

static inline void hex(uint8_t *out, const uint8_t data) {
    *out = (
            ((data >> 4) < 0xA ? (data >> 4) + 0x30 : ((data >> 4) - 9) + 0x40) << 4
        ) 
        | 
        (
            (data & 0x0F) < 0xA ? (data & 0x0F) + 0x30 : ((data & 0x0F) - 9) + 0x40
        );
}

MCPCAN *current_can = NULL;
uint8_t bus = 255;

uint16_t loopdetect = 0;
uint8_t response_length = 0;
uint8_t response[64];

static inline void slcan(void) {
    /*
     * read buffer, send data
     */
    // recieve
    uint8_t ext = 0, rtr = 0;
    uint8_t data[8];
    uint8_t l = 0; //, port = 0;
    uint32_t id = 0;
    uint16_t i = 0;

    response_length = 0;

    RX_INTE();
#ifdef SLCAN_BINARY 
    // fast send, now with crc!
    if ((canmode & SLCAN_MODE_BINARY) && ((uint8_t) serial_rx_length()) >= ((uint8_t) sizeof(slcan_binary)) && serial_rx_getc(0) == SLCAN_BINARY_PREAMBLE) {
        uint8_t crc = crc8_256(serial_rx, serial_rx_first, sizeof(slcan_binary) - 1);
        if (crc == serial_rx_getc(sizeof(slcan_binary) - 1)) {
            can_tx_put_ringbuf(0);
        }
        serial_rx_discard(sizeof(slcan_binary));
        return;
    }
    uint8_t xt = 0;
    uint8_t found = 0;
    for (int i = 0; i < serial_rx_length(); i++) {
        if (serial_rx_getc(i) == SLCAN_OK) {
            xt = i;
            found = 1;
            loopdetect = 0;
        }
    }
#endif

#define SLCAN_MAX_LEN sizeof("B0T001122338001122334455667788")
    if (!found) {
        if (serial_rx_length() > SLCAN_MAX_LEN) {
            serial_rx_discard(SLCAN_MAX_LEN);
        }
        loopdetect++;
        // nothing found

        if (loopdetect > DISCARD_INTERVAL) {
            serial_rx_first = serial_rx_last = 0;
            loopdetect = 0;
        }        
        return;
    }

    loopdetect = 0;
    bus = 255;
    if (serial_rx_getc(0) == SLCAN_CANBUS_SWITCH && (serial_rx_length() >= 2)) {
        switch (serial_rx_getc(1)) {
            case 0X31:
                current_can = &CAN1;
                bus = 1;
                serial_rx_discard(2);
                xt -= 2;
                break;
            case 0x32:
                current_can = &CAN2;
                bus = 2;
                serial_rx_discard(2);
                xt -= 2;
                break;
            case 0x30:
                current_can = &CAN0;
                bus = 0;
                serial_rx_discard(2);
                xt -= 2;
                break;
            default:
                current_can = &CAN0;
                bus = 255;
                uart_putc(SLCAN_ERROR);
                serial_rx_discard(xt+1);
                xt = 0;
                break;
        }
    }

    switch (serial_rx_getc(0)) {
        case SLCAN_OK:
            break;
        case SLCAN_CMD_SEND_MODE:
            if (xt != 2) {
                uart_putc(SLCAN_ERROR);
            } else {
                switch(serial_rx_getc(1)) {
                    case '1':
                        canmode = (canmode & 0xCF) | SLCAN_MODE_BINARY;
                        break;
                    default:
                    case '0':
                        canmode = (canmode & 0xCF) | SLCAN_MODE_BASIC;
                        break;
                }
                uart_putc(SLCAN_OK);
            }
            break;
        case SLCAN_CMD_SERIAL:
            if (xt != 1) {
                uart_putc(SLCAN_ERROR);
                break;
            }
            response[0] = SLCAN_CMD_SERIAL;
            memcpy(&response[1], SLCAN_SERIAL, sizeof(SLCAN_SERIAL) - 1);
            response[sizeof(SLCAN_SERIAL)] = SLCAN_OK;
            response_length = sizeof(SLCAN_SERIAL) + 1;
            break;
        case SLCAN_CMD_VERSION:
            if (xt != 1) {
                uart_putc(SLCAN_ERROR);
                break;
            }
            response[0] = SLCAN_CMD_VERSION;
            memcpy(&response[1], SLCAN_VERSION, sizeof(SLCAN_VERSION) - 1);
            response[sizeof(SLCAN_VERSION)] = SLCAN_OK;
            response_length = sizeof(SLCAN_VERSION) + 1;
            break;
        case SLCAN_CMD_UART_BAUD:
            if (xt != 2) {
                uart_putc(SLCAN_ERROR);
            } else {
                uint32_t newspeed = SLCAN_UART_1;
                switch (serial_rx_getc(1)) {
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
                baudreset = BAUD(newspeed);
#endif
                uart_putc(SLCAN_OK);
            }
            break;


        case SLCAN_CMD_RTR_EXT:
        case SLCAN_CMD_TRANSMIT_EXT:
            if (serial_rx_length() < 10) {
                break;
            }
            ext = 1;
        case SLCAN_CMD_RTR:
        case SLCAN_CMD_TRANSMIT:
            if (serial_rx_length() < 5) {
                break;
            }
            // find end, if not return
            // set rtr
            if (serial_rx_getc(0) == SLCAN_CMD_RTR || serial_rx_getc(0) == SLCAN_CMD_RTR_EXT) rtr = 1;
            l = 0;
            if (ext) {
                unhex_ringbuf(data, 1, 8);
            } else {
                unhex_ringbuf(data, 1, 3);
            }
            // unhex data
            if (ext && !rtr) {
                l = serial_rx_getc(9) - '0';
                if (((uint8_t) (l * 2) + 10) > serial_rx_length()) {
                    break; // wait for more
                }
                id = ((uint32_t) data[0]) << 24 | ((uint32_t) data[1]) << 16 | ((uint32_t) data[2] << 8) | ((uint32_t) data[3]);
                unhex_ringbuf(data, 10, l*2);
            } else if (!rtr) {
                l = serial_rx_getc(4) - '0';
                if (((uint8_t)((l * 2) + 5)) > serial_rx_length()) {
                    uart_putc(serial_rx_length());
                    uart_putc(serial_rx_getc(4));
                    break; // wait for more
                }
                id = ((uint32_t) data[0] << 8) | ((uint32_t) data[1]);
                unhex_ringbuf(data, 5, l * 2);
            }

            if (bus == 255) bus = 0;

            if (l > 8 || id > 0x1FFFFFFF || bus > 2) {
                if(l > 8) uart_putc('n');
                if(id > 0x1FFFFFFF) uart_putc('m');
                if (bus > 2) uart_putc('b');
                uart_putc(SLCAN_ERROR);
            } else {
                can_tx_put(bus, id, ext, rtr, l, data);
                uart_putc('z' - (id > 0x7ff ? 0x20 : 0));
                uart_putc(SLCAN_OK);
            }

            break;
        case SLCAN_CMD_BITRATE:
            if (xt != 2) {
                uart_putc(SLCAN_ERROR);
                break;
            }
            switch (serial_rx_getc(1)) {
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
                uart_putc(SLCAN_ERROR);
            } else {
                uint8_t rv = 0;
                switch (bus) {
                    case 0:
                        rv = MCP_CAN_begin(&CAN0, MCP_ANY, l, MCP_16MHZ | MCP_CLKOUT_ENABLE);
                        MCP_CAN_setMode(&CAN0,MCP_NORMAL);
                        break;
                    case 1:
                        rv = MCP_CAN_begin(&CAN1, MCP_ANY, l, MCP_16MHZ | MCP_CLKOUT_ENABLE);
                        MCP_CAN_setMode(&CAN1, MCP_NORMAL);
                        break;
                    case 2:
                        rv = MCP_CAN_begin(&CAN2, MCP_ANY, l, MCP_16MHZ);
                        MCP_CAN_setMode(&CAN2, MCP_NORMAL);
                        break;
                    default:
                        rv = MCP_CAN_begin(&CAN0, MCP_ANY, l, MCP_16MHZ | MCP_CLKOUT_ENABLE);
                        MCP_CAN_setMode(&CAN0,MCP_NORMAL);
                        break;
                }               
                if (rv == CAN_OK) uart_putc(SLCAN_OK);
                else uart_putc(SLCAN_ERROR);
            }
            break;
        case SLCAN_CMD_OPEN_NORMAL:
            if (xt != 1) {
                uart_putc(SLCAN_ERROR);
                break;
            }
            if (bus == 255) {
                for (i = 0; i < 3; i++) {
                    slcan_mode[i] = SLCAN_MODE_OPEN;
                }
            } else if (bus < 3) {
                slcan_mode[bus] = SLCAN_MODE_OPEN;
            } else {
                uart_putc(SLCAN_ERROR);
                break;
            }
            // reset buffer
            can_rx_reset();
            uart_putc(SLCAN_OK);
            break;
        case SLCAN_CMD_OPEN_LISTEN:
            if (xt != 1) {
                uart_putc(SLCAN_ERROR);
                break;
            }
            if (bus == 255) {
                for (i = 0; i < 3; i++) {
                    slcan_mode[i] = SLCAN_MODE_LISTEN;
                }
            } else if (bus < 3) {
                slcan_mode[bus] = SLCAN_MODE_LISTEN;
            } else {
                uart_putc(SLCAN_ERROR);
                break;
            }
            // reset buffer
            can_rx_reset();
            uart_putc(SLCAN_OK);
            break;
        case SLCAN_CMD_CLOSE:
            if (xt != 1) {
                uart_putc(SLCAN_ERROR);
                break;
            }
            if (bus == 255) {
                for (i = 0; i < 3; i++) {
                    slcan_mode[i] = SLCAN_MODE_CLOSED;
                }
            } else if (bus < 3) {
                slcan_mode[bus] = SLCAN_MODE_CLOSED;
            } else {
                uart_putc(SLCAN_ERROR);
                break;
            }
            can_rx_reset();
            uart_putc(SLCAN_OK);
            break;
        case SLCAN_CMD_STATUS_FLAGS:
            if (xt != 1) {
                uart_putc(SLCAN_ERROR);
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
            hex(&response[1], flags);
            response[3] = SLCAN_OK;
            response_length = 4;
            break;
        case SLCAN_CMD_ISR:
            if (xt != 1) {
                uart_putc(SLCAN_ERROR);
                break;
            }
            response[0] = 'I';
            hex(&response[1], PCIFR);
            response[3] = SLCAN_OK;
            response_length = 4;
            break;
        case SLCAN_CMD_TIMESTAMP:
            if (xt != 2) {
                uart_putc(SLCAN_ERROR);
                break;
            }
            switch(serial_rx_getc(1)) {
                case '0':
                    canmode &= ~SLCAN_MODE_TIMESTAMP_ENABLED;
                    break;
                case '1':
                default:
                    canmode |= SLCAN_MODE_TIMESTAMP_ENABLED;
                    break;
            }
            uart_putc(SLCAN_OK);
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
            break;
        default:
            break;
    }

    if(found) serial_rx_discard(xt+1);
    


    if (response_length) {
#ifdef ARDUINO_CORE
        Serial.write(response, response_length);
#else
        uart_putb(response, response_length);
#endif
        response_length = 0;
    }

}
#ifdef WAKEUP_APIM
static unsigned long ms = 0;
static unsigned long time_apim = 0;
#endif

static inline void setup() {
    //get_mcusr();
    can_rx_first = can_rx_last = 0; // reset
    can_tx_first = can_tx_last = 0;
    serial_tx_first = 0;
    serial_tx_last = 0;
    serial_rx_first = 0;
    serial_rx_last = 0;

    ISR_SETUP();
    
    uart_init(BAUD(3000000UL));
    //uart_init(BAUD(1000000UL));

    sei();
    // setup pins



    // clear all interrupts;
    //

    CAN0.MCPCS = 0;
    CAN1.MCPCS = 1;
    CAN2.MCPCS = 2;

    if(MCP_CAN_begin(&CAN0,MCP_ANY, CAN_500KBPS, MCP_16MHZ | MCP_CLKOUT_ENABLE) == CAN_OK) {
#ifdef ARDUINO_CORE
        Serial.print("CAN0: Init OK!\r");
#else
        //         xx x x x x x x 
        uart_puts("CAN0: Init OK!\r");
#endif
        MCP_CAN_setMode(&CAN0,MCP_NORMAL);
    } else {
#ifdef ARDUINO_CORE
        Serial.print("CAN0: Init Fail!!!\r");
#else
        uart_puts("CAN0: Init Fail!!!\r");
#endif
    }

    if(MCP_CAN_begin(&CAN1,MCP_ANY, CAN_125KBPS, MCP_16MHZ | MCP_CLKOUT_ENABLE) == CAN_OK){
#ifdef ARDUINO_CORE
        Serial.print("CAN1: Init OK!\r");
#else
        uart_puts("CAN1: Init OK!\r");
#endif
        MCP_CAN_setMode(&CAN1,MCP_NORMAL);
    } else {
#ifdef ARDUINO_CORE
        Serial.print("CAN1: Init Fail!!!\r");
#else
        uart_puts("CAN1: Init Fail!!!\r");
#endif
    }

    if(MCP_CAN_begin(&CAN2,MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK){
#ifdef ARDUINO_CORE
        Serial.print("CAN2: Init OK!\r");
#else
        uart_puts("CAN2: Init OK!\r");
#endif
        MCP_CAN_setMode(&CAN2,MCP_NORMAL);
    } else {
#ifdef ARDUINO_CORE
        Serial.print("CAN2: Init Fail!!!\r");
#else
        uart_puts("CAN2: Init Fail!!!\r");
#endif
    }

    if (mcusr_mirror & PORF)
#ifdef ARDUINO_CORE
        Serial.print("PWR RESET\r");
#else
        uart_puts("PWR RESET\r");
#endif
    if (mcusr_mirror & EXTRF)
#ifdef ARDUINO_CORE
            Serial.print("EXT RESET\r");
#else
        uart_puts("EXT RESET\r");
#endif
    if (mcusr_mirror & BORF)
#ifdef ARDUINO_CORE
            Serial.print("BOD RESET\r");
#else
        uart_puts("BOD RESET\r");
#endif
    if (mcusr_mirror & WDRF)
#ifdef ARDUINO_CORE
            Serial.print("WDT RESET\r");
#else
        uart_puts("WDT RESET\r");
#endif

    //wdt_enable(WDTO_4S);
}

uint8_t sendbuffer[SLCAN_MSG_LEN];
uint8_t sendbuffer_length = 0;
uint16_t timestamp = 0;
uint32_t counter = 0;

static inline void cv_hex(uint8_t data, uint8_t *target) {
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

static inline void send_can(uint8_t d, uint32_t id, uint8_t len, uint8_t *data) {
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

static inline void send_can_packed(slcan_binary *msg) {
    send_can(gpBUS(msg), gpID(msg), msg->len, msg->data);
}


static inline void slcan_response(void) {
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
                uart_putb((uint8_t *) msg, sizeof(slcan_binary));
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
                uart_putb(sendbuffer, sendbuffer_length);
#endif
            }
            cnt++;
#endif
        } while (!inttrig0 && !inttrig1 && !inttrig2 && can_rx_available() && cnt < CAN_BUFFER_SIZE);
    }
}

uint8_t vcnt = 0;
static inline void loop() {
    /*
     * Read uart, do things, cleanup interrupt buffer, loop.
     */

    uint8_t bus, len, resp;
    uint32_t id;
    uint8_t buf[8];

    if (can_rx_available()) {
        slcan_response();
    }


#if SLCAN_CHECK_ITERVAL != 0
    if (slcounter & 0x80) { 
#endif
        if (!serial_rx_empty()) slcan();
#if SLCAN_CHECK_ITERVAL != 0
        slcounter = 0;
    }
#endif


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

    if (inttrig0 && (slcan_mode[0] & 0x03)) {
        resp = MCP_CAN_readMsgBuf(&CAN0,&id, &len, buf);
        if (CAN_OK == resp && id != 0) {
            can_rx_put(CAN_HSCAN, id, (id & 0x80000000) ? 1 : 0 , (id & 0x40000000) ? 1 : 0, len, buf);
            rcount++;
        }
        inttrig0 = 0;
    }

    if (inttrig1 && (slcan_mode[1] & 0x03)) {

        resp = MCP_CAN_readMsgBuf(&CAN1,&id, &len, buf);

        if (CAN_OK == resp && id != 0) {
            can_rx_put(CAN_MSCAN, id, (id & 0x80000000) ? 1 : 0 , (id & 0x40000000) ? 1 : 0, len, buf);
            rcount++;
        }
        inttrig1 = 0;
    }

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
    while(can_tx_available()) {
        send_can_packed(can_tx_pull());
    }

    if (resetcnt & 128) {
        // if int is missed, set
        inttrig0 = ((~PINA) & 0x80);
        inttrig1 = ((~PINC) & 0x01);
        inttrig2 = ((~PIND) & 0x80);
        resetcnt = 0;
        /* volatile uint8_t e0 = MCP_CAN_getError(&CAN0); */
        /* volatile uint8_t e1 = MCP_CAN_getError(&CAN1); */
        /* volatile uint8_t e2 = MCP_CAN_getError(&CAN2); */
        /*  */
        /* uint8_t r0 = MCP_CAN_checkReceive(&CAN0); */
        /* uint8_t r1 = MCP_CAN_checkReceive(&CAN1); */
        /* uint8_t r2 = MCP_CAN_checkReceive(&CAN2); */
        /*  */
        /* uint8_t p0 = *(&PINA); */
        /* uint8_t p1 = *(&PINB); */
        /* uint8_t p2 = *(&PINC); */
        /* uint8_t p3 = *(&PIND); */
        /*  */
        /* volatile uint8_t i0 = MCP_CAN_getInt(&CAN0); */
        /* volatile uint8_t i1 = MCP_CAN_getInt(&CAN1); */
        /* volatile uint8_t i2 = MCP_CAN_getInt(&CAN2); */
        /*  */
        /* printf("%02X %02X %02X # %02X %02X %02X # %02X %02X %02X %02X # %02X %02X %02X\r",  */
        /*         e0, e1, e2, */
        /*         r0, r1, r2, */
        /*         p0, p1, p2, p3, */
        /*         i0, i1, i2); */
    }

    //if (slcan_mode[0] == SLCAN_MODE_OPEN) printf("\r");

    //counter++;
    resetcnt++;
    slcounter++;
    //wdt_reset();
    //
    if (baudreset) {
        uart_init(baudreset);
        baudreset = 0;
    }
    vcnt++;
}

#ifndef ARDUINO_CORE
int main(void) {
    PORTA = 0x01;
    mcusr_mirror = MCUSR;
    PORTA ^= 0x01;
    MCUSR = 0;
    PORTA = 0x00;
    //wdt_disable();
    setup();
    while(1) loop();
    return 0;
}
#endif

