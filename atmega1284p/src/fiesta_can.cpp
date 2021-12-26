#include <SPI.h>


#include "settings.h"
#include "can_data.h"
#include "slcan_def.h"
#include "mcp2515/mcp_can.h"
#include "canmsg.h"

MCP_CAN CAN0(CAN0_CS);                              // CAN0 interface usins CS on digital pin 10
MCP_CAN CAN1(CAN1_CS);                               // CAN1 interface using CS on digital pin 9
MCP_CAN CAN2(CAN2_CS);                               // CAN1 interface using CS on digital pin 9


can_data can_buffer[CAN_BUFFER_SIZE]; // sufficiently big
uint8_t can_buffer_first = 0;
uint8_t can_buffer_last = 0;

uint8_t slcan_mode[3] = { SLCAN_MODE_CLOSED, SLCAN_MODE_CLOSED, SLCAN_MODE_CLOSED };

char msgbuf[128];
uint8_t flags;
uint32_t rcount = 0;
uint8_t overflow = 0;

volatile uint8_t inttrig0 = 0;
volatile uint8_t inttrig1 = 0;
volatile uint8_t inttrig2 = 0;
uint8_t canmode = SLCAN_MODE_BASIC;

#define setv(p, v) *((volatile uint8_t *) &p) = v;
void ISR_SETUP(void) {
    // set pins as input
    PCIFR = 0x0F;
    pinMode(CAN0_INT, INPUT_PULLUP);
    pinMode(CAN1_INT, INPUT_PULLUP);
    pinMode(CAN2_INT, INPUT_PULLUP);

    *(uint8_t *)ADCSRA = 0x00;
    volatile uint8_t *ddra = (volatile uint8_t *) &DDRA, *ddrb = (volatile uint8_t *) &DDRB, *ddrc = (volatile uint8_t *) &DDRC, *ddrd = (volatile uint8_t *) &DDRD;
    volatile uint8_t *porta = (volatile uint8_t *) &PORTA, *portb = (volatile uint8_t *) &PORTB, *portc = (volatile uint8_t *) &PORTC, *portd = (volatile uint8_t *) &PORTD;
/*    *ddra &= 0x7F;
    *ddrc &= 0xFE;
    *ddrd &= 0x7F;
    *porta &= 0x7F;
    *portc &= 0xFE;
    *portd &= 0x7F;*/
    //setv(PORTA, 0x00);
    //setv(PORTC, 0x00);
    //setv(PORTD, 0x00);
/*    *(uint8_t *)DD0 = 0x00;
    *(uint8_t *)DD2 = 0x00;
    *(uint8_t *)DD3 = 0x00;
    *(uint8_t *)PORT0 = 0x00;
    *(uint8_t *)PORT2 = 0x00;
    *(uint8_t *)PORT3 = 0x00;*/
    *digitalPinToPCICR(CAN0_INT) |= (1 << digitalPinToPCICRbit(CAN0_INT));
    *digitalPinToPCICR(CAN1_INT) |= (1 << digitalPinToPCICRbit(CAN1_INT));
    *digitalPinToPCICR(CAN2_INT) |= (1 << digitalPinToPCICRbit(CAN2_INT));
    *digitalPinToPCMSK(CAN0_INT) = 0x80; //_BV(digitalPinToPCMSKbit(CAN0_INT));
    *digitalPinToPCMSK(CAN1_INT) = 0x01; //_BV(digitalPinToPCMSKbit(CAN1_INT));
    *digitalPinToPCMSK(CAN2_INT) = 0x80; //_BV(digitalPinToPCMSKbit(CAN2_INT));
    sprintf(msgbuf, "ISR Set to %02X %02X %02X %02X %02X", PCICR, PCMSK0, PCMSK1, PCMSK2, PCMSK3);
    Serial.println(msgbuf);
    sprintf(msgbuf, "DD: %02X %02X %02X %02X", *ddra, *ddrb, *ddrc, *ddrd);
    Serial.println(msgbuf);
    sprintf(msgbuf, "PORT: %02X %02X %02X %02X", *porta, *portb, *portc, *portd);
    Serial.println(msgbuf);

    inttrig0 = inttrig1 = inttrig2 = 0;
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

    can_buffer[can_buffer_last].bus = bus;
    can_buffer[can_buffer_last].id = id;
    can_buffer[can_buffer_last].ext = ext;
    can_buffer[can_buffer_last].rtr = rtr;
    can_buffer[can_buffer_last].len = len;
    memcpy(can_buffer[can_buffer_last].data, buf, len);
}

inline uint8_t can_data_pull(can_data *msg) {
    if (can_buffer_first == can_buffer_last) return 0; // nothing available

    msg->bus = can_buffer[can_buffer_first].bus;
    msg->id = can_buffer[can_buffer_first].id;
    msg->ext = can_buffer[can_buffer_first].ext;
    msg->rtr = can_buffer[can_buffer_first].rtr;
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

ISR (PCINT0_vect) {
    // we only use one interrupt so no checking needed
    inttrig0 = 1;
}

ISR (PCINT2_vect) {
    // we only use one interrupt so no checking needed
    inttrig1 = 1;
}

ISR (PCINT3_vect) {
    // we only use one interrupt so no checking needed
    inttrig2 = 1;
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
                        rv = current_can->begin(MCP_ANY, l, MCP_8MHZ | MCP_CLKOUT_ENABLE);
                    } else if (current_can == &CAN2) {
                        rv = current_can->begin(MCP_ANY, l, MCP_8MHZ);
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

    if (error) {
    }

    if (command_end > 0) {
        if (slcan_buffer_length - command_end > 0) {
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
}

unsigned long ms = 0, time_apim = 0, slcounter = 0;

void setup() {
    can_buffer_first = can_buffer_last = 0; // reset

/*    pinMode(8, INPUT_PULLUP);
    pinMode(9, INPUT_PULLUP);*/
    Serial.begin(2000000);
    //Serial.begin(500000);

    ISR_SETUP();
    // clear all interrupts;
    //
    pinMode(CAN0_CS, OUTPUT);
    pinMode(CAN1_CS, OUTPUT);
    pinMode(CAN2_CS, OUTPUT);


    if(CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ | MCP_CLKOUT_ENABLE) == CAN_OK) {
        Serial.print("CAN0: Init OK!\r\n");
        CAN0.setMode(MCP_NORMAL);
    } else {
        Serial.print("CAN0: Init Fail!!!\r\n");
    }

    if(CAN1.begin(MCP_STDEXT, CAN_125KBPS, MCP_8MHZ | MCP_CLKOUT_ENABLE) == CAN_OK){
        Serial.print("CAN1: Init OK!\r\n");
        CAN1.setMode(MCP_NORMAL);
    } else {
        Serial.print("CAN1: Init Fail!!!\r\n");
    }

    if(CAN2.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK){
        Serial.print("CAN2: Init OK!\r\n");
        CAN2.setMode(MCP_NORMAL);
    } else {
        Serial.print("CAN2: Init Fail!!!\r\n");
    }

    // clear buffers
    // clear int
    CAN0.resetInt();
    CAN1.resetInt();
    CAN2.resetInt();

    ms = slcounter = time_apim = millis();
}

can_data msg;
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

uint8_t resetcnt = 0;


extern apim_1 APIM_SIGNAL_1;
extern apim_2 APIM_SIGNAL_2;
extern apim_3 APIM_SIGNAL_3;

void send_can(uint8_t d, uint32_t id, uint8_t len, uint8_t *data) {
    
    MCP_CAN *bus = NULL;
    switch (d) {
        case (0):
            bus = &CAN0;
            break;
        case (1):
            bus = &CAN1;
            break;
        case (2):
            bus = &CAN2;
            break;
        default:
            bus = &CAN0;
            break;
    }
    bus->sendMsgBuf(id, len, data);
}


void loop() {
    /*
     * Read uart, do things, cleanup interrupt buffer, loop.
     */

    if (can_data_available()) {
#ifdef DEBUG
        unsigned long mu = micros();
        unsigned long mu3 = 0;
#endif
        uint8_t m = (slcan_mode[0] & 0x03) | (slcan_mode[1] & 0x03) | (slcan_mode[2] & 0x03); // anyone listening
        if (m) {
            // send data
            can_data_pull(&msg);
#if defined(SLCAN_BINARY) && defined(SLCAN_BASIC)
            if (canmode & SLCAN_MODE_BINARY)
#endif
#ifdef SLCAN_BINARY
#pragma message "Binary can mode enabled"
            {
                slcan_binary b;
                b.preamble = SLCAN_BINARY_PREAMBLE;
                b.bus = msg.bus & 0x03;
                b.rtr = msg.rtr;
                b.len = msg.len;
                b.id = msg.id;
                memcpy(b.data, msg.data, msg.len);
#ifdef DEBUG
                mu3 = micros();
#endif
                Serial.write((uint8_t *)&b, sizeof(slcan_binary) - (8 - b.len));
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
                sendbuffer[sendbuffer_length++] = 0x30 + msg.bus;
                // copy id
                //
                if (msg.ext) {
                    sendbuffer[sendbuffer_length++] = msg.rtr ? SLCAN_CMD_RTR_EXT : SLCAN_CMD_TRANSMIT_EXT;
                    cv_hex((msg.id & 0xFF000000) >> 24, &sendbuffer[sendbuffer_length]);
                    cv_hex((msg.id & 0x00FF0000) >> 16, &sendbuffer[sendbuffer_length+2]);
                    cv_hex((msg.id & 0x0000FF00) >> 8, &sendbuffer[sendbuffer_length+4]);
                    cv_hex((msg.id & 0x000000FF), &sendbuffer[sendbuffer_length+6]);
                    sendbuffer_length += 8;
                } else {
                    cv_hex((msg.id & 0x0000FF00) >> 8, &sendbuffer[sendbuffer_length]);
                    cv_hex((msg.id & 0x000000FF), &sendbuffer[sendbuffer_length+2]);
                    sendbuffer[sendbuffer_length] = msg.rtr ? SLCAN_CMD_RTR : SLCAN_CMD_TRANSMIT;
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
#ifdef DEBUG
                mu3 = micros();
#endif
                Serial.write(sendbuffer, sendbuffer_length);
            }
#endif
    	    sendbuffer_length = 0;
        }
#ifdef DEBUG
        unsigned long mu2 = micros();
        sprintf(msgbuf, "\rTT: %10ld us - %10ld us", mu3 - mu, mu2 - mu3);
        Serial.println(msgbuf);
#endif
    }
    ms = millis();


    if (ms > slcounter + 100) { // 100ms check
#ifdef DEBUG
        unsigned long mu = micros();
#endif
        slcan();
        slcounter = ms;
#ifdef DEBUG
        unsigned long mu2 = micros();
        sprintf(msgbuf, "\rSL: %10ld", mu2 - mu);
        Serial.println(msgbuf);
#endif

    }

#ifdef DEBUG_CAN
    if (counter > 10000) {
#ifdef DEBUG
        unsigned long mu = micros();
#endif
        rpm += 100;
        speed += 250;
        uint8_t dd[8] = { (uint8_t) (((rpm * 4) & 0xFF00) >> 8), (uint8_t) ((rpm*4) & 0x00FF), 0x00, 0x00, (uint8_t) ((speed & 0xFF00) >> 8), (uint8_t) (speed & 0x00FF), 0x00, 0x00};
        uint8_t dd2[8] = { 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        CAN0.sendMsgBuf(0x201, 8, dd);
        CAN0.sendMsgBuf(0x420, 8, dd2);

        if (rpm > 2000) rpm = 0;
        if (speed > 20000) speed = 0;
        counter = 0;
#ifdef DEBUG
        unsigned long mu2 = micros();
        sprintf(msgbuf, "\rSD: %10ld", mu2 - mu);
        Serial.println(msgbuf);
#endif
    }
#endif

#ifdef WAKEUP_APIM
    if (ms > time_apim + 1000) {
        time_apim = ms;
        send_can(APIM_SIGNAL_1.bus, APIM_SIGNAL_1.id, APIM_SIGNAL_1.len, APIM_SIGNAL_1.data);
        send_can(APIM_SIGNAL_2.bus, APIM_SIGNAL_2.id, APIM_SIGNAL_2.len, APIM_SIGNAL_2.data);
        send_can(APIM_SIGNAL_3.bus, APIM_SIGNAL_3.id, APIM_SIGNAL_3.len, APIM_SIGNAL_3.data);
    }
#endif

    uint32_t id;
    uint8_t ext;
    uint8_t len;
    uint8_t buf[8];
    uint8_t resp;
    // read buffers out of int
    if (inttrig0 && (slcan_mode[0] & 0x03)) {
#ifdef DEBUG
        unsigned long mu = micros();
#endif

        resp = CAN0.readMsgBuf(&id, &ext, &len, buf);
        if (CAN_OK == resp) {
            can_data_put(CAN_HSCAN, id & 0x1FFFFFFF, (id & 0x80000000) >> 24, (id & 0x40000000) >> 24, len, buf);
            rcount++;
        }
        inttrig0 = 0;
        digitalWrite(CAN0_INT, 1);
#ifdef DEBUG
        unsigned long mu2 = micros();
        sprintf(msgbuf, "\rRD: %10ld", mu2 - mu);
        Serial.println(msgbuf);
#endif
    }
    if (inttrig1 && (slcan_mode[1] & 0x03)) {

        resp = CAN1.readMsgBuf(&id, &ext, &len, buf);

        if (CAN_OK == resp) {
            can_data_put(CAN_MSCAN, id & 0x1FFFFFFF, (id & 0x80000000) >> 24, (id & 0x40000000) >> 24, len, buf);
            rcount++;
        }
        inttrig1 = 0;
        digitalWrite(CAN1_INT, 1);
    }
    if (inttrig2 && (slcan_mode[2] & 0x03)) {
        resp = CAN2.readMsgBuf(&id, &ext, &len, buf);

        if (CAN_OK == resp) {
            can_data_put(CAN_ICAN, id & 0x1FFFFFFF, (id & 0x80000000) >> 24, (id & 0x40000000) >> 24, len, buf);
            rcount++;
        }
        inttrig2 = 0;
        digitalWrite(CAN2_INT, 1);
    }

    if (resetcnt > 66) {
        inttrig0 = inttrig1 = inttrig2 = 1;
        resetcnt = 0;
    }


    counter++;
    resetcnt++;
}



