#include <SPI.h>
#include <avr/io.h>


#include "settings.h"
#include "can_data.h"
#include "slcan_def.h"
#include "mcp2515/mcp_can.h"
#include "canmsg.h"
#include "crc8.h"

MCP_CAN CAN0(CAN0_CS);                              
MCP_CAN CAN1(CAN1_CS);                             
MCP_CAN CAN2(CAN2_CS);                            

//#define DEBUG
//#define TIMING

#ifdef TIMING
#define TIME(message, code) {\
    unsigned long mu = micros();\
\
    code;\
    unsigned long mu2 = micros();\
    sprintf(msgbuf, "\r\n" message ": %10ld us", mu2 - mu);\
    Serial.println(msgbuf);\
}
#else
#define TIME(message, code) code
#endif

char msgbuf[128];
#define printf(frmt, ...) {\
    snprintf(msgbuf, sizeof(msgbuf), frmt, ##__VA_ARGS__);\
    Serial.print(msgbuf);\
}

slcan_binary can_buffer[CAN_BUFFER_SIZE]; // sufficiently big
uint8_t can_buffer_first = 0;
uint8_t can_buffer_last = 0;

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

/***
 * interrupts
 */
void ISR_SETUP(void) {
    // set pins as input
    PCIFR = 0x0F;

    volatile uint8_t *ddra = (volatile uint8_t *) &DDRA, *ddrb = (volatile uint8_t *) &DDRB, *ddrc = (volatile uint8_t *) &DDRC, *ddrd = (volatile uint8_t *) &DDRD;
    volatile uint8_t *porta = (volatile uint8_t *) &PORTA, *portb = (volatile uint8_t *) &PORTB, *portc = (volatile uint8_t *) &PORTC, *portd = (volatile uint8_t *) &PORTD;
    DDRA = 0x00;
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
    sprintf(msgbuf, "ISR Set to %02X %02X %02X %02X %02X", PCICR, PCMSK0, PCMSK1, PCMSK2, PCMSK3);
    Serial.println(msgbuf);
    sprintf(msgbuf, "DD: %02X %02X %02X %02X", *ddra, *ddrb, *ddrc, *ddrd);
    Serial.println(msgbuf);
    sprintf(msgbuf, "PORT: %02X %02X %02X %02X", *porta, *portb, *portc, *portd);
    Serial.println(msgbuf);

    inttrig0 = inttrig1 = inttrig2 = 0;
}

ISR (PCINT0_vect) {
    inttrig0 = (~PINA & 0x80);
}

ISR (PCINT2_vect) {
    inttrig1 = (~PINC & 0x01);
}

ISR (PCINT3_vect) {
    inttrig2 = (~PIND & 0x80);
}



inline uint8_t can_data_full(void) {
    return (can_buffer_last == can_buffer_first - 1 || (can_buffer_first == 0 && can_buffer_last == (CAN_BUFFER_SIZE - 1))) ? 1 : 0;
}

inline uint8_t can_data_available(void) {
    return can_buffer_last != can_buffer_first;
}

inline uint8_t can_data_transmit_full(void) {
    return 0;
}

inline void can_data_put(uint8_t bus, uint32_t id, uint8_t ext, uint8_t rtr, uint8_t len, uint8_t *buf) {
    if (can_data_full()) {
	    overflow = 1;
        return;
    }

    can_buffer_last++;
    if (can_buffer_last == CAN_BUFFER_SIZE) {
        can_buffer_last = 0;
    }
#ifdef DEBUG
    sprintf(msgbuf, "CAN %d, %lu, %d, %d", bus, id, rtr, len);
    Serial.println(msgbuf);
#endif

    can_buffer[can_buffer_last].idfield = IDFIELD(bus, rtr, id);
    can_buffer[can_buffer_last].len = len;
    memcpy(can_buffer[can_buffer_last].data, buf, len);
}

inline uint8_t can_data_pull(slcan_binary *msg) {
    if (can_buffer_first == can_buffer_last) return 0; // nothing available

    msg->idfield = can_buffer[can_buffer_first].idfield;
    msg->len = can_buffer[can_buffer_first].len;
    memcpy(msg->data, can_buffer[can_buffer_first].data, can_buffer[can_buffer_first].len);

    if (can_buffer_first == CAN_BUFFER_SIZE - 1) {
        can_buffer_first = 0;
    } else {
        can_buffer_first++;
    }

    return 1;
}

inline void can_data_reset(void) {
    can_buffer_first = 0;
    can_buffer_last = 0;
}

uint8_t slcan_buffer[128];
uint8_t slcan_buffer_length = 0;

void printhex(char * str, uint8_t *buf, uint8_t len) {
    Serial.print(str);
    for (int i = 0; i < len; i++) {
        sprintf(msgbuf, "%02X", buf[i]);
        Serial.print(msgbuf);
    }
    Serial.println("");
}

byte asc2byte(char chr) {
    byte rVal = 0;
    if (isdigit(chr)) {
        rVal = chr - '0';
    } else if (chr >= 'A' && chr <= 'F') {
        rVal = chr + 10 - 'A';
    } else if (chr >= 'a' && chr <= 'f') {
        rVal = chr + 10 - 'a';
    }
    return rVal;
}

void unhex(uint8_t * outP, uint8_t * inP, size_t len) {
    for (; len > 1; len -= 2) {
        byte val = asc2byte(*inP++) << 4;
        *outP++ = val | asc2byte(*inP++);
    }
}

void slcan() {
    /*
     * read buffer, send data
     */
//    can_data canmsg;
    uint8_t response_length = 0;
    uint8_t response[32];
    // recieve
    uint8_t command_end = 0;
    uint8_t ext = 0, rtr = 0;
    uint8_t data[8];
    uint8_t l = 0; //, port = 0;
    uint32_t id = 0;
    uint8_t error = 0;
    uint8_t bus = 255;
    uint8_t i, j;

    MCP_CAN *current_can = NULL;

    do {
        int available = Serial.available();
        if (available) {
            Serial.readBytes(&slcan_buffer[slcan_buffer_length], slcan_buffer_length + available >= 128 ? 128 - (slcan_buffer_length + available) : available);
            slcan_buffer_length += (uint8_t) available;
        }

        // digest incomming message if available
        if (slcan_buffer_length > 0) {
            // look for CR in 0 to slcan_buffer_length
            for (int i = 0; i < slcan_buffer_length; i++) {
                if (slcan_buffer[i] == SLCAN_OK) {
                    command_end = i+1;
                    break;
                }
            }
        }

        if (slcan_buffer_length > 0 && command_end != 0) {
            current_can = NULL;

            if (slcan_buffer[0] == SLCAN_CANBUS_SWITCH && command_end >= 2) {
                switch (slcan_buffer[1]) {
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
                    default:
                        current_can = NULL;
                        bus = 255;
                        break;
                }
                memmove(slcan_buffer, &slcan_buffer[2], slcan_buffer_length - 2);
                command_end -= 2;
                slcan_buffer_length -= 2;
            }

            switch (slcan_buffer[0]) {
                case SLCAN_CMD_SEND_MODE:
                    if (command_end < 2) {
                        response[0] = SLCAN_ERROR;
                        response_length = 1;
                        error = 1;
                    } else {
                        switch(slcan_buffer[1]) {
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
                    break;
                case SLCAN_CMD_SERIAL:
                    response[0] = SLCAN_CMD_SERIAL;
                    memcpy(&response[1], SLCAN_SERIAL, sizeof(SLCAN_SERIAL) - 1);
                    response[sizeof(SLCAN_SERIAL)] = SLCAN_OK;
                    response_length = sizeof(SLCAN_SERIAL) + 1;
                    break;
                case SLCAN_CMD_VERSION:
                    response[0] = SLCAN_CMD_VERSION;
                    memcpy(&response[1], SLCAN_VERSION, sizeof(SLCAN_VERSION) - 1);
                    response[sizeof(SLCAN_VERSION)] = SLCAN_OK;
                    response_length = sizeof(SLCAN_VERSION) + 1;
                    break;
                case SLCAN_CMD_UART_BAUD:
                    if (command_end < 2) {
                        response[0] = SLCAN_ERROR;
                        response_length = 1;
                        error = 1;
                    } else {
                        uint32_t newspeed = SLCAN_UART_1;
                        switch (slcan_buffer[1]) {
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
                        Serial.end();
                        Serial.begin(newspeed);
                        response[0] = SLCAN_OK;
                        response_length = 1;
                    }
                    break;


                case SLCAN_CMD_RTR_EXT:
                case SLCAN_CMD_TRANSMIT_EXT:
                    if (slcan_buffer_length < 10) {
                        response[0] = SLCAN_ERROR;
                        response_length = 1;
                        error = 1;
                        break;
                    }
                    ext = 1;
                case SLCAN_CMD_RTR:
                case SLCAN_CMD_TRANSMIT:
                    if (slcan_buffer_length < 5) {
                        response[0] = SLCAN_ERROR;
                        response_length = 1;
                        error = 1;
                        break;
                    }
                    // set rtr
                    if (slcan_buffer[0] == SLCAN_CMD_RTR || slcan_buffer[0] == SLCAN_CMD_RTR_EXT) rtr = 1;
                    l = 0;
                    /*
                    * Change from normal protocol, 2 MSB are port to send to
                    */

                    // unhex data
                    if (ext && ! rtr) {
                        unhex(data, &slcan_buffer[1], 8);
                        l = slcan_buffer[9] - '0';
                        data[0] &= 0x3F; // strip port
                        id = ((uint32_t) data[0]) << 24 | ((uint32_t) data[1]) << 16 | ((uint32_t) data[2] << 8) | ((uint32_t) data[3]);
                        unhex(data, &slcan_buffer[10], command_end - 10);
                    } else if (!rtr) {
                        slcan_buffer[0] = '0';
                        unhex(data, slcan_buffer, 4);
                        l = slcan_buffer[4] - '0';
                        id = ((uint32_t) data[0] << 8) | ((uint32_t) data[1]);
                        unhex(data, &slcan_buffer[5], command_end - 5);
                    }

                    if (ext) id |= 0x80000000;
                    if (rtr) id |= 0x40000000;

                    if (current_can != NULL) {
                        current_can->sendMsgBuf(id, l, data);
                    }

                    response[0] = SLCAN_OK;
                    response_length = 1;
                    break;
                case SLCAN_CMD_BITRATE:
                    if (slcan_buffer_length < 2) {
                        response[0] = SLCAN_ERROR;
                        response_length = 1;
                        error = 1;
                        break;
                    }
                    switch (slcan_buffer[1]) {
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
                            rv = current_can->begin(MCP_ANY, l, MCP_16MHZ | MCP_CLKOUT_ENABLE);
                        } else if (current_can == &CAN2) {
                            rv = current_can->begin(MCP_ANY, l, MCP_16MHZ);
                        } else {
                            rv = SLCAN_ERROR;
                        }
                        if (rv == CAN_OK) response[0] = SLCAN_OK;
                        else response[0] = SLCAN_ERROR;
                        response_length = 1;
                    }
                    break;
                case SLCAN_CMD_OPEN_NORMAL:
                    if (bus == 255) {
                        i = 0; j = 2;
                    } else {
                        i = bus; j = bus;
                    }
                    for (; i <= j; i++) {
                        slcan_mode[i] = SLCAN_MODE_OPEN;
                    }
                    // reset buffer
                    can_data_reset();
                    response[0] = SLCAN_CMD_OPEN_NORMAL;
                    response_length = 1;
                    break;
                case SLCAN_CMD_OPEN_LISTEN:
                    if (bus == 255) {
                        i = 0; j = 2;
                    } else {
                        i = bus; j = bus;
                    }
                    for (; i <= j; i++) {
                        slcan_mode[i] = SLCAN_MODE_LISTEN;
                    }
                    // reset buffer
                    can_data_reset();
                    response[0] = SLCAN_CMD_OPEN_LISTEN;
                    response_length = 1;
                    break;
                case SLCAN_CMD_CLOSE:
                    if (bus == 255) {
                        i = 0; j = 2;
                    } else {
                        i = bus; j = bus;
                    }
                    for (; i <= j; i++) {
                        slcan_mode[i] = SLCAN_MODE_CLOSED;
                    }
                    can_data_reset();
                    response[0] = SLCAN_CMD_CLOSE;
                    response_length = 1;
                    break;
                case SLCAN_CMD_STATUS_FLAGS:
                    flags = 0x00;
                    if (can_data_full()) {
                        flags |= 0x01;
                    }
                    if (can_data_transmit_full()) {
                        flags |= 0x02;
                    }
                    response[0] = 'F';
                    sprintf((char *) &response[1], "%02X", flags);
                    response[3] = SLCAN_OK;
                    response_length = 4;
                    break;
                case SLCAN_CMD_ISR:
                    response[0] = 'I';
                    sprintf((char *) &response[1], "%02X", PCIFR);
                    response[3] = SLCAN_OK;
                    response_length = 4;
                    break;
                // not implemented
                case SLCAN_CMD_TIMESTAMP:
                    if (slcan_buffer_length < 3) {
                        response[0] = SLCAN_ERROR;
                        response_length = 1;
                        error = 1;
                        break;
                    }
                    switch(slcan_buffer[1]) {
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
                    break;
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
        }

        if (error) {}

        if (command_end > 0) {
            if (slcan_buffer_length > command_end) {
                memmove(slcan_buffer, &slcan_buffer[command_end], slcan_buffer_length - command_end);
                slcan_buffer_length -= command_end;
            } else {
                memset(slcan_buffer, 0x00, command_end);
                slcan_buffer_length = 0;
            }
            command_end = 0;
        }



        if (response_length > 0) {
            Serial.write(response, response_length);
            Serial.flush();
            response_length = 0;
        }
    } while(slcan_buffer_length > 0);
}

unsigned long ms = 0, time_apim = 0, slcounter = 0;

void setup() {
    can_buffer_first = can_buffer_last = 0; // reset

    disablePower(POWER_SERIAL1);
    disablePower(POWER_ADC);
    //disablePower(POWER_TIMER3);
    //disablePower(POWER_TIMER2);
    //disablePower(POWER_TIMER1);

    Serial.begin(1000000);

    // setup pins
    ISR_SETUP();



    // clear all interrupts;
    //


    if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ | MCP_CLKOUT_ENABLE) == CAN_OK) {
        Serial.print("CAN0: Init OK!\r\n");
        CAN0.setMode(MCP_NORMAL);
    } else {
        Serial.print("CAN0: Init Fail!!!\r\n");
    }

    if(CAN1.begin(MCP_ANY, CAN_125KBPS, MCP_16MHZ | MCP_CLKOUT_ENABLE) == CAN_OK){
        Serial.print("CAN1: Init OK!\r\n");
        CAN1.setMode(MCP_NORMAL);
    } else {
        Serial.print("CAN1: Init Fail!!!\r\n");
    }

    if(CAN2.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK){
        Serial.print("CAN2: Init OK!\r\n");
        CAN2.setMode(MCP_NORMAL);
    } else {
        Serial.print("CAN2: Init Fail!!!\r\n");
    }

    ms = slcounter = time_apim = millis();

    CAN0.readMsgBuf(&id, &len, buf);
    CAN1.readMsgBuf(&id, &len, buf);
    CAN2.readMsgBuf(&id, &len, buf);
}

slcan_binary msg;
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
            CAN1.sendMsgBuf(id, len, data);
            break;
        case (2):
            CAN2.sendMsgBuf(id, len, data);
            break;
        case (0):
            CAN0.sendMsgBuf(id, len, data);
            break;
        default:
            d = 255;
            break;
    }
#ifdef DEBUG
    sprintf(msgbuf, "%lu, %d, %d", id, d, len);
    Serial.println(msgbuf);
#endif
}

inline void send_can_packed(slcan_binary *msg) {
    send_can(gpBUS(msg), gpID(msg), msg->len, msg->data);
}


inline void slcan_response(void) {
    uint8_t m = (slcan_mode[0] & 0x03) | (slcan_mode[1] & 0x03) | (slcan_mode[2] & 0x03); // anyone listening
    if (m) {
        do {
        // send data
            can_data_pull(&msg);
#if defined(SLCAN_BINARY) && defined(SLCAN_BASIC)
            if (canmode & SLCAN_MODE_BINARY)
#endif
#ifdef SLCAN_BINARY
#pragma message "Binary can mode enabled"
            {
                msg.preamble = SLCAN_BINARY_PREAMBLE;
                msg.crc = crc8((uint8_t *) &msg, sizeof(slcan_binary) - 1);
                Serial.write((uint8_t *)&msg, sizeof(slcan_binary));
            }
#endif
#if defined(SLCAN_BINARY) && defined(SLCAN_BASIC)
            else
#endif
#ifdef SLCAN_BASIC 
#pragma message "Basic can mode enabled"
            {
                int i;
                sendbuffer[sendbuffer_length++] = SLCAN_CANBUS_SWITCH; // set bus
                sendbuffer[sendbuffer_length++] = 0x30 + gBUS(msg);
                // copy id
                //
                if (gID(msg) > 2047) {
                    sendbuffer[sendbuffer_length++] = gRTR(msg) ? SLCAN_CMD_RTR_EXT : SLCAN_CMD_TRANSMIT_EXT;
                    cv_hex((gID(msg) & 0xFF000000) >> 24, &sendbuffer[sendbuffer_length]);
                    cv_hex((gID(msg) & 0x00FF0000) >> 16, &sendbuffer[sendbuffer_length+2]);
                    cv_hex((gID(msg) & 0x0000FF00) >> 8, &sendbuffer[sendbuffer_length+4]);
                    cv_hex((gID(msg) & 0x000000FF), &sendbuffer[sendbuffer_length+6]);
                    sendbuffer_length += 8;
                } else {
                    cv_hex((gID(msg) & 0x0000FF00) >> 8, &sendbuffer[sendbuffer_length]);
                    cv_hex((gID(msg) & 0x000000FF), &sendbuffer[sendbuffer_length+2]);
                    sendbuffer[sendbuffer_length] = gRTR(msg) ? SLCAN_CMD_RTR : SLCAN_CMD_TRANSMIT;
                    sendbuffer_length += 4;
                }

                sendbuffer[sendbuffer_length++] = 0x30 + msg.len;

                for (i = 0; i < msg.len; i++) {
                    cv_hex(msg.data[i], &sendbuffer[sendbuffer_length]);
                    sendbuffer_length += 2;
                }

                if (canmode & SLCAN_MODE_TIMESTAMP_ENABLED) {
                    cv_hex((timestamp & 0xFF00) >> 8, &sendbuffer[sendbuffer_length]);
                    cv_hex((timestamp & 0x00FF), &sendbuffer[sendbuffer_length+2]);
                    sendbuffer_length += 4;
                }
                sendbuffer[sendbuffer_length++] = SLCAN_OK;
                Serial.write(sendbuffer, sendbuffer_length);
            }
#endif
            sendbuffer_length = 0;
        } while (!inttrig0 && !inttrig1 && !inttrig2 == 0 && can_data_available());
    }
}

void loop() {
    /*
     * Read uart, do things, cleanup interrupt buffer, loop.
     */

    ms = millis();

    if (can_data_available()) {
        TIME("CAN Response", slcan_response());
    }


    if (ms > slcounter + 25) { // 100ms check, takes about 400us on no message
//        TIME("SLCAN Read", slcan());
//    if (counter > 1000) {
        slcan();
        slcounter = ms;
        counter = 0;

    }

#ifdef DEBUG_CAN
    if (counter > 10000) {
        rpm += 100;
        speed += 250;
        uint8_t dd[8] = { (uint8_t) (((rpm * 4) & 0xFF00) >> 8), (uint8_t) ((rpm*4) & 0x00FF), 0x00, 0x00, (uint8_t) ((speed & 0xFF00) >> 8), (uint8_t) (speed & 0x00FF), 0x00, 0x00};
        uint8_t dd2[8] = { 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        TIME("CAN0 Send",
            CAN0.sendMsgBuf(0x201, 8, dd);
            CAN0.sendMsgBuf(0x420, 8, dd2);
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

    // read buffers out of int
    if (inttrig0 && (slcan_mode[0] & 0x03)) {
        resp = CAN0.readMsgBuf(&id, &len, buf);
        if (CAN_OK == resp) {
            TIME("CAN0 Store", can_data_put(CAN_HSCAN, id, (id & 0x80000000) ? 1 : 0 , (id & 0x40000000) ? 1 : 0, len, buf));
            rcount++;
        }
        inttrig0 = 0;
        //digitalWrite(CAN0_INT, 1);
    }
    if (inttrig1 && (slcan_mode[1] & 0x03)) {

        resp = CAN1.readMsgBuf(&id, &len, buf);

        if (CAN_OK == resp) {
            can_data_put(CAN_MSCAN, id, (id & 0x80000000) ? 1 : 0 , (id & 0x40000000) ? 1 : 0, len, buf);
            rcount++;
        }
        inttrig1 = 0;
    }
    if (inttrig2 && (slcan_mode[2] & 0x03)) {
        resp = CAN2.readMsgBuf(&id, &len, buf);

        if (CAN_OK == resp) {
            can_data_put(CAN_ICAN, id, (id & 0x80000000) ? 1 : 0 , (id & 0x40000000) ? 1 : 0, len, buf);
            rcount++;
        }
        inttrig2 = 0;
    }

    if (resetcnt > 8) {
        // if int is missed, set
        inttrig0 = (~PINA & 0x80);
        inttrig1 = (~PINC & 0x01);
        inttrig2 = (~PIND & 0x80);
        resetcnt = 0;
        /*volatile uint8_t e0 = CAN0.getError();
        volatile uint8_t e1 = CAN1.getError();
        volatile uint8_t e2 = CAN2.getError();

        uint8_t r0 = CAN0.checkReceive();
        uint8_t r1 = CAN1.checkReceive();
        uint8_t r2 = CAN2.checkReceive();

        uint8_t p0 = *(&PINA);
        uint8_t p1 = *(&PINB);
        uint8_t p2 = *(&PINC);
        uint8_t p3 = *(&PIND);

        volatile uint8_t i0 = CAN0.getInt();
        volatile uint8_t i1 = CAN1.getInt();
        volatile uint8_t i2 = CAN2.getInt();

        printf("%02X %02X %02X # %02X %02X %02X # %02X %02X %02X %02X # %02X %02X %02X\r", 
                e0, e1, e2,
                r0, r1, r2,
                p0, p1, p2, p3,
                i0, i1, i2);*/
    }


    counter++;
    resetcnt++;
}



