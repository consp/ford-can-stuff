#include <SPI.h>

#include "settings.h"
#include "can_data.h"
#include "slcan_def.h"
#include "mcp2515/mcp_can.h"
MCP_CAN CAN0(CAN0_CS);                              // CAN0 interface usins CS on digital pin 10
MCP_CAN CAN1(CAN1_CS);                               // CAN1 interface using CS on digital pin 9
MCP_CAN CAN2(CAN2_CS);                               // CAN1 interface using CS on digital pin 9


can_data can_buffer[CAN_BUFFER_SIZE]; // sufficiently big
uint8_t can_buffer_first = 0;
uint8_t can_buffer_last = 0;

uint8_t slcan_mode = SLCAN_MODE_CLOSED;
char msgbuf[128];
uint8_t flags;
uint32_t rcount = 0;
uint8_t overflow = 0;

#define WITHOUT_INTERRUPTION(CODE) {uint8_t sreg = SREG; noInterrupts(); {CODE} SREG = sreg;}

//#define DEBUG
/*
 * PCINT7 for 0
 * PCINT16 for 1
 * PCINT31 for 2 
 */
void ISR_SETUP(void) {
    // set pins as input
    PCIFR = 0x0F; 
    pinMode(CAN0_INT, INPUT);
    pinMode(CAN1_INT, INPUT);
    pinMode(CAN2_INT, INPUT);
    *digitalPinToPCICR(CAN0_INT) |= (1 << digitalPinToPCICRbit(CAN0_INT));
    *digitalPinToPCICR(CAN1_INT) |= (1 << digitalPinToPCICRbit(CAN1_INT));
    *digitalPinToPCICR(CAN2_INT) |= (1 << digitalPinToPCICRbit(CAN2_INT));
    *digitalPinToPCMSK(CAN0_INT) = 0xFF; //_BV(digitalPinToPCMSKbit(CAN0_INT));
    *digitalPinToPCMSK(CAN1_INT) = 0x00; //_BV(digitalPinToPCMSKbit(CAN1_INT));
    *digitalPinToPCMSK(CAN2_INT) = 0x00; //_BV(digitalPinToPCMSKbit(CAN2_INT));
    sprintf(msgbuf, "ISR Set to %02X %02X %02X %02X %02X", PCICR, PCMSK0, PCMSK1, PCMSK2, PCMSK3);
    Serial.println(msgbuf);
}
uint8_t can_data_full(void) {
    return (can_buffer_last == can_buffer_first - 1 || (can_buffer_first == 0 && can_buffer_last == (CAN_BUFFER_SIZE - 1))) ? 1 : 0;
}

uint8_t can_data_available(void) {
    return can_buffer_last != can_buffer_first;
}

uint8_t can_data_transmit_full(void) {
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
    
    CAN0.resetInt();
    CAN1.resetInt();
    CAN2.resetInt();
}

inline void can0_isr(void) {
    uint32_t id = 0;
    uint8_t ext = 0;
    uint8_t len = 0;
    uint8_t buf[8];
    
    uint8_t resp = CAN0.readMsgBuf(&id, &ext, &len, buf);
    
    if (CAN_OK == resp) {
        can_data_put(CAN_HSCAN, id & 0x1FFFFFFF, (id & 0x80000000) >> 24, (id & 0x40000000) >> 24, len, buf);
	rcount++;
    }
}

ISR (PCINT0_vect) {
    // we only use one interrupt so no checking needed
    can0_isr();
}

inline void can1_isr(void) {
    uint32_t id = 0;
    uint8_t ext = 0;
    uint8_t len = 0;
    uint8_t buf[8];
    
    uint8_t resp = CAN1.readMsgBuf(&id, &ext, &len, buf);
    
    if (CAN_OK == resp) {
        can_data_put(CAN_MSCAN, id & 0x1FFFFFFF, (id & 0x80000000) >> 24, (id & 0x40000000) >> 24, len, buf);
	rcount++;
    }
}

ISR (PCINT2_vect) {
    // we only use one interrupt so no checking needed
    can1_isr();
}

inline void can2_isr(void) {
    uint32_t id = 0;
    uint8_t ext = 0;
    uint8_t len = 0;
    uint8_t buf[8];
   
    uint8_t resp = CAN2.readMsgBuf(&id, &ext, &len, buf);

    if (CAN_OK == resp) {
	can_data_put(CAN_ICAN, id & 0x1FFFFFFF, (id & 0x80000000) >> 24, (id & 0x40000000) >> 24, len, buf);
	rcount++;
    }
}

ISR (PCINT3_vect) {
    // we only use one interrupt so no checking needed
    can2_isr();
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
        current_can = &CAN0;

        if (slcan_buffer[0] == SLCAN_CANBUS_SWITCH && command_end >= 2) {
            switch (slcan_buffer[1]) {
                case 0X31:
                    current_can = &CAN1;
                    break;
                case 0x32:
                    current_can = &CAN2;
                    break;
                case 0x30:
                default:
                    current_can = &CAN0;
                    break;
            }
            memmove(slcan_buffer, &slcan_buffer[2], slcan_buffer_length - 2);
            command_end -= 2;
            slcan_buffer_length -= 2;
#ifdef DEBUG
            printhex("New CMD: ", slcan_buffer, command_end);
#endif
        }
        
        switch (slcan_buffer[0]) {
            
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
#ifdef DEBUG     
                printhex("SND: ", slcan_buffer, command_end);
#endif
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
                current_can->sendMsgBuf(id, l, data);
                
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
                    } else {
                        rv = current_can->begin(MCP_ANY, l, MCP_8MHZ);
                    }
                    if (rv == CAN_OK) response[0] = SLCAN_OK;
                    else response[0] = SLCAN_ERROR;
                    response_length = 1;  
                }
                break;
            case SLCAN_CMD_OPEN_NORMAL:
                slcan_mode = SLCAN_MODE_OPEN;
                // reset buffer
                can_data_reset();
                response[0] = SLCAN_CMD_OPEN_NORMAL;
                response_length = 1;
                break;
            case SLCAN_CMD_OPEN_LISTEN:
                slcan_mode = SLCAN_MODE_LISTEN;
                // reset buffer
                can_data_reset();
                response[0] = SLCAN_CMD_OPEN_LISTEN;
                response_length = 1;
                break;
            case SLCAN_CMD_CLOSE:
                slcan_mode = SLCAN_MODE_CLOSED;
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
                        slcan_mode &= ~SLCAN_MODE_TIMESTAMP_ENABLED;
                        break;
                    case '1':
                    default:
                        slcan_mode |= SLCAN_MODE_TIMESTAMP_ENABLED;
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
#ifdef DEBUG
        printhex("Buffer: ", slcan_buffer, slcan_buffer_length);
#endif
    }
        
    if (command_end > 0) {
#ifdef DEBUG
        printhex("Buffer: ", slcan_buffer, slcan_buffer_length);
        printhex("CMD: ", slcan_buffer, command_end);
#endif
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
#ifdef DEBUG
        printhex("resp: ", response, response_length);
#endif
        Serial.write(response, response_length);
        Serial.flush();
        response_length = 0;
    }
}

void setup() {
    can_buffer_first = can_buffer_last = 0; // reset
    
    Serial.begin(1000000);

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

void loop() {
    /*
     * Read uart, do things, cleanup interrupt buffer, loop.
     */
    int i;

    if (can_data_available()) {
#ifdef DEBUG
        sprintf(msgbuf, "Buffer is %d", can_buffer_last - can_buffer_first);
        Serial.println(msgbuf);
#endif
        if (slcan_mode == SLCAN_MODE_OPEN || slcan_mode == SLCAN_MODE_LISTEN) {
            // send data
            can_data_pull(&msg);
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

            if (slcan_mode & SLCAN_MODE_TIMESTAMP_ENABLED) {
                cv_hex((timestamp & 0xFF00) >> 8, &sendbuffer[sendbuffer_length]);
                cv_hex((timestamp & 0x00FF), &sendbuffer[sendbuffer_length+2]);
                sendbuffer_length += 4;
            }
            sendbuffer[sendbuffer_length++] = SLCAN_OK;
            
            Serial.write(sendbuffer, sendbuffer_length);
            Serial.flush();
	    sendbuffer_length = 0;
        }
    }
    
    slcan();
 
    /*counter++;
    if (counter > 20000) {
        counter = 0;
	uint8_t x0 = 0, x1 = 0, x2 = 0, e0 = 0, e1 = 0, e2 = 0, r0 = 0, r1 = 0, r2 = 0;
        x0 = CAN0.checkReceive();
        x1 = CAN1.checkReceive();
        x2 = CAN2.checkReceive();
        e0 = CAN0.getError();
        e1 = CAN1.getError();
        e2 = CAN2.getError();
        r0 = CAN0.errorCountRX();
        r1 = CAN1.errorCountRX();
        r2 = CAN2.errorCountRX();
	uint8_t V1 = digitalRead(CAN0_INT);
	uint8_t V2 = digitalRead(CAN1_INT);
	uint8_t V3 = digitalRead(CAN2_INT);
        sprintf(msgbuf, "E: %02X %02X %02X | %02X %02X %02X | %d %d %d | %d %d %d | %d %02X %lu ", e0, e1, e2, r0, r1, r2, x0, x1, x2, V1, V2, V3, overflow, can_buffer_first <= can_buffer_last ? can_buffer_last - can_buffer_first : CAN_BUFFER_SIZE - (can_buffer_first - can_buffer_last) , rcount);
        Serial.println(msgbuf);
    }*/

    counter++;
    if (counter > 2500) {
	rpm += 100;
	speed += 250;
	uint8_t dd[8] = { (uint8_t) (((rpm * 4) & 0xFF00) >> 8), (uint8_t) ((rpm*4) & 0x00FF), 0x00, 0x00, (uint8_t) ((speed & 0xFF00) >> 8), (uint8_t) (speed & 0x00FF), 0x00, 0x00};
	uint8_t dd2[8] = { 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	CAN0.sendMsgBuf(0x201, 8, dd);
	CAN0.sendMsgBuf(0x420, 8, dd2);

	if (rpm > 2000) rpm = 0;
	if (speed > 20000) speed = 0;
	counter = 0;
    }

   // due to how the PCINT ISRs work we might miss the initial int
   // and need to fix this.
   if (digitalRead(CAN0_INT) == 0) {
	// call ISR anyway
	//
	can0_isr();
   }
   if (digitalRead(CAN1_INT) == 0) {
	// call ISR anyway
	//
	can1_isr();
   }
   if (digitalRead(CAN2_INT) == 0) {
	// call ISR anyway
	//
	can2_isr();
   }
    
}



