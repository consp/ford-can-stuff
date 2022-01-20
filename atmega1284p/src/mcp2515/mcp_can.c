/*
  mcp_can.cpp
  2012 Copyright (c) Seeed Technology Inc.  All right reserved.
  2017 Copyright (c) Cory J. Fowler  All Rights Reserved.
  2021 Copyright (c) Tristan Timmermans All Rights Reserved.

  Author: Loovee
  Contributor: Cory J. Fowler
  2017-09-25
  Contributor: Tristan Timmermans
  2021-01-20

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
  1301  USA
*/
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "mcp2515/mcp_can.h"
#define noInterrupts() cli()
//#define DEBUG_MODE 1

#include "pins_arduino.h"
#define SPI_SPEED 10000000

#ifdef ARDUINO_CORE
#define SPI_INIT SPI.begin();
#define SPI_BEGIN SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0)) 
#define SPI_END SPI.endTransaction()
#define SPI_SELECT(x) MCP2515_SELECT()
#define SPI_DESELECT(x) MCP2515_UNSELECT()
#define SPI_TRANSFER(x) SPI.transfer(x)
#define SPI_TRANSFER_B(x, y) SPI.transfer(x, y)
#define SPI_READ SPI_TRANSFER(0x00)
#else


#define SPI_BUF_SIZE 32
uint8_t spibuf[SPI_BUF_SIZE]; // max 16
volatile uint8_t spi_head;
volatile uint8_t spi_tail;
MCPCAN *globalstate = NULL;

uint8_t initialized = 0;
static void spi_init(uint32_t speed); 
static void spi_init(uint32_t speed) {
    uint8_t sreg = SREG;
    noInterrupts(); // Protect from a scheduler and prevent transactionBegin

    if (!initialized) {
        DDRB |= (1 << 7) | (1 << 5) | (1 << SS); // sck/mosi
        PORTB |= (1 << SS);
        SPCR = (1 << SPE) | (1 << MSTR) | (0 << SPR0) | (0 << SPR1);
        SPSR = (1 << SPI2X);
    }
    initialized++; // reference count
    SREG = sreg;

    spi_head = spi_tail = 0;
}

#define SPI_INTE() SPCR |= (1 << SPIE)
#define SPI_INTD() SPCR &= ~(1 << SPIE)
#define spi_begin 
//uint8_t sreg = SREG; noInterrupts()
#define spi_end
//SREG = sreg

static uint8_t spi_transfer(uint8_t data);
static uint8_t spi_transfer(uint8_t data) {
    SPDR = data;
    asm volatile("nop");
    while (!(SPSR & (1<<SPIF)));
    return SPDR;
}

#define spi_deselect(state) PORTB |= (1 << state->MCPCS)
#define spi_select(state) PORTB &= ~(1 << state->MCPCS)

#define SPI_INIT spi_init(SPI_SPEED)
#define SPI_BEGIN spi_begin
#define SPI_END spi_end
#define SPI_SELECT(x) spi_select(x)
#define SPI_DESELECT(x) spi_deselect(x)
#define SPI_TRANSFER(x) spi_transfer(x)
#define SPI_TRANSFER_B(x, y) for(uint8_t vsn = 0; vsn < y; vsn++) spi_transfer(((uint8_t *) x)[vsn]);
#define SPI_READ() SPI_TRANSFER(0x00)
#endif


#define BUFFER_SIZE 16

#define INLINED

#ifdef INLINED
#define INLINE inline
#else
#define INLINE 
#endif

/*********************************************************************************************************
** Function name:           mcp2515_reset
** Descriptions:            Performs a software reset
*********************************************************************************************************/
void mcp2515_reset(MCPCAN *state)                                      
{
    SPI_BEGIN;
    SPI_SELECT(state);
    SPI_TRANSFER(MCP_RESET);
    SPI_DESELECT(state);
    SPI_END;
    for(int i = 0; i < 5000; i++) asm volatile("nop"); //delay(5); // If the MCP2515 was in sleep mode when the reset command was issued then we need to wait a while for it to reset properly
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegister
** Descriptions:            Read data register
*********************************************************************************************************/
INLINE uint8_t mcp2515_readRegister(MCPCAN *state, const uint8_t address)                                                                     
{
    uint8_t ret;

    SPI_BEGIN;
    SPI_SELECT(state);
    SPI_TRANSFER(MCP_READ);
    SPI_TRANSFER(address);
    ret = SPI_READ();
    SPI_DESELECT(state);
    SPI_END;

    return ret;
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegisterS
** Descriptions:            Reads successive data registers
*********************************************************************************************************/
INLINE void mcp2515_readRegisterS(MCPCAN *state, const uint8_t address, uint8_t values[], const uint8_t n)
{
    uint8_t i;
    SPI_BEGIN;
    SPI_SELECT(state);
    SPI_TRANSFER(MCP_READ);
    SPI_TRANSFER(address);
    // mcp2515 has auto-increment of address-pointer
    for (i=0; i<n; i++) 
        values[i] = SPI_READ();
    SPI_DESELECT(state);
    SPI_END;
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegister
** Descriptions:            Sets data register
*********************************************************************************************************/
INLINE void mcp2515_setRegister(MCPCAN *state, const uint8_t address, const uint8_t value)
{
    SPI_BEGIN;
    SPI_SELECT(state);
    SPI_TRANSFER(MCP_WRITE);
    SPI_TRANSFER(address);
    SPI_TRANSFER(value);
    SPI_DESELECT(state);
    SPI_END;
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegisterS
** Descriptions:            Sets successive data registers
*********************************************************************************************************/
INLINE void mcp2515_setRegisterS(MCPCAN *state, const uint8_t address, uint8_t values[], const uint8_t n)
{
    SPI_BEGIN;
    SPI_SELECT(state);
    SPI_TRANSFER(MCP_WRITE);
    SPI_TRANSFER(address);
    SPI_TRANSFER_B(values, n);	
    SPI_DESELECT(state);
    SPI_END;
}

/*********************************************************************************************************
** Function name:           mcp2515_modifyRegister
** Descriptions:            Sets specific bits of a register
*********************************************************************************************************/
INLINE void mcp2515_modifyRegister(MCPCAN *state, const uint8_t address, const uint8_t mask, const uint8_t data)
{
    SPI_BEGIN;
    SPI_SELECT(state);
    uint8_t buf[4] = { MCP_BITMOD, address, mask, data };
    SPI_TRANSFER_B(buf, 4);
//    SPI_TRANSFER(MCP_BITMOD);
//    SPI_TRANSFER(address);
//    SPI_TRANSFER(mask);
//    SPI_TRANSFER(data);
    SPI_DESELECT(state);
    SPI_END;
}

ISR (SPI_STC_vect)
{
    SPDR = spibuf[spi_tail++];
    if (((spi_tail - spi_head) & SPI_BUF_SIZE) == 0) {
        SPI_DESELECT(globalstate);
        globalstate = NULL;
    }
}

INLINE void mcp2515_async_send(MCPCAN *state, uint8_t *data, uint8_t len) {
    if (((SPI_BUF_SIZE - (spi_head - spi_tail)) & SPI_BUF_SIZE) < len) return; // fail
    if (globalstate != NULL) return;

    globalstate = state;
    while(len--) spibuf[spi_head++] = *data++; // fill buffer
    // init transfer
    
    SPI_INTE();
}

/*********************************************************************************************************
** Function name:           mcp2515_readStatus
** Descriptions:            Reads status register
*********************************************************************************************************/
INLINE uint8_t mcp2515_readStatus(MCPCAN *state)                             
{
    uint8_t i;
    SPI_BEGIN;
    SPI_SELECT(state);
    SPI_TRANSFER(MCP_READ_STATUS);
    i = SPI_READ();
    SPI_DESELECT(state);
    SPI_END;
    return i;
}

/*********************************************************************************************************
** Function name:           setSleepWakeup
** Descriptions:            Enable or disable the wake up interrupt (If disabled the MCP2515 will not be woken up by CAN bus activity)
*********************************************************************************************************/
void setSleepWakeup(MCPCAN *state, const uint8_t enable)
{
    mcp2515_modifyRegister(state, MCP_CANINTE, MCP_WAKIF, enable ? MCP_WAKIF : 0);
}

/*********************************************************************************************************
** Function name:           setMode
** Descriptions:            Sets control mode
*********************************************************************************************************/
uint8_t MCP_CAN_setMode(MCPCAN *state, const uint8_t opMode)
{
    state->mcpMode = opMode;
    return mcp2515_setCANCTRL_Mode(state, state->mcpMode);
}

/*********************************************************************************************************
** Function name:           mcp2515_setCANCTRL_Mode
** Descriptions:            Set control mode
*********************************************************************************************************/
uint8_t mcp2515_setCANCTRL_Mode(MCPCAN *state, const uint8_t newmode)
{
	// If the chip is asleep and we want to change mode then a manual wake needs to be done
	// This is done by setting the wake up interrupt flag
	// This undocumented trick was found at https://github.com/mkleemann/can/blob/master/can_sleep_mcp2515.c
	if((mcp2515_readRegister(state, MCP_CANSTAT) & MODE_MASK) == MCP_SLEEP && newmode != MCP_SLEEP)
	{
		// Make sure wake interrupt is enabled
		uint8_t wakeIntEnabled = mcp2515_readRegister(state, MCP_CANINTE) & MCP_WAKIF;
		if(!wakeIntEnabled)
			mcp2515_modifyRegister(state, MCP_CANINTE, MCP_WAKIF, MCP_WAKIF);

		// Set wake flag (this does the actual waking up)
		mcp2515_modifyRegister(state, MCP_CANINTF, MCP_WAKIF, MCP_WAKIF);

		// Wait for the chip to exit SLEEP and enter LISTENONLY mode.

		// If the chip is not connected to a CAN bus (or the bus has no other powered nodes) it will sometimes trigger the wake interrupt as soon
		// as it's put to sleep, but it will stay in SLEEP mode instead of automatically switching to LISTENONLY mode.
		// In this situation the mode needs to be manually set to LISTENONLY.

		if(mcp2515_requestNewMode(state,MCP_LISTENONLY) != MCP2515_OK)
			return MCP2515_FAIL;

		// Turn wake interrupt back off if it was originally off
		if(!wakeIntEnabled)
			mcp2515_modifyRegister(state, MCP_CANINTE, MCP_WAKIF, 0);
	}

	// Clear wake flag
	mcp2515_modifyRegister(state, MCP_CANINTF, MCP_WAKIF, 0);
	
	return mcp2515_requestNewMode(state, newmode);
}

/*********************************************************************************************************
** Function name:           mcp2515_requestNewMode
** Descriptions:            Set control mode
*********************************************************************************************************/
uint8_t mcp2515_requestNewMode(MCPCAN *state, const uint8_t newmode)
{
    uint32_t counter = 0;

	// Spam new mode request and wait for the operation  to complete
	while(1)
	{
		// Request new mode
		// This is inside the loop as sometimes requesting the new mode once doesn't work (usually when attempting to sleep)
		mcp2515_modifyRegister(state, MCP_CANCTRL, MODE_MASK, newmode); 

		uint8_t statReg = mcp2515_readRegister(state, MCP_CANSTAT);
		if((statReg & MODE_MASK) == newmode) // We're now in the new mode
			return MCP2515_OK;
		else if(counter > 4800000) // Wait no more than 200ms for the operation to complete
			return MCP2515_FAIL;
        counter++;
	}
}


uint8_t mcp2515_requestClkOut(MCPCAN *state, const uint8_t newmode)
{
	uint32_t counter = 0;

	// Spam new mode request and wait for the operation  to complete
	while(1)
	{
		// Request new mode
		// This is inside the loop as sometimes requesting the new mode once doesn't work (usually when attempting to sleep)
		mcp2515_modifyRegister(state, MCP_CANCTRL, MODE_CLKMASK, newmode); 

		uint8_t statReg = mcp2515_readRegister(state, MCP_CANCTRL);
		if((statReg & (MODE_CLKMASK)) == newmode) // We're now in the new mode
			return MCP2515_OK;
		else if(counter > 150000) // Wait no more than 200ms for the operation to complete
		{
			return MCP2515_FAIL;
		}
        counter++;
	}
}

/*********************************************************************************************************
** Function name:           mcp2515_configRate
** Descriptions:            Set baudrate
*********************************************************************************************************/
uint8_t mcp2515_configRate(MCPCAN *state, const uint8_t canSpeed, const uint8_t canClock)            
{
    uint8_t set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canClock & MCP_CLOCK_SELECT)
    {
        case (MCP_8MHZ):
        switch (canSpeed) 
        {
            case (CAN_5KBPS):                                               //   5KBPS                  
            cfg1 = MCP_8MHz_5kBPS_CFG1;
            cfg2 = MCP_8MHz_5kBPS_CFG2;
            cfg3 = MCP_8MHz_5kBPS_CFG3;
            break;

            case (CAN_10KBPS):                                              //  10KBPS                  
            cfg1 = MCP_8MHz_10kBPS_CFG1;
            cfg2 = MCP_8MHz_10kBPS_CFG2;
            cfg3 = MCP_8MHz_10kBPS_CFG3;
            break;

            case (CAN_20KBPS):                                              //  20KBPS                  
            cfg1 = MCP_8MHz_20kBPS_CFG1;
            cfg2 = MCP_8MHz_20kBPS_CFG2;
            cfg3 = MCP_8MHz_20kBPS_CFG3;
            break;

            case (CAN_31K25BPS):                                            //  31.25KBPS                  
            cfg1 = MCP_8MHz_31k25BPS_CFG1;
            cfg2 = MCP_8MHz_31k25BPS_CFG2;
            cfg3 = MCP_8MHz_31k25BPS_CFG3;
            break;

            case (CAN_33K3BPS):                                             //  33.33KBPS                  
            cfg1 = MCP_8MHz_33k3BPS_CFG1;
            cfg2 = MCP_8MHz_33k3BPS_CFG2;
            cfg3 = MCP_8MHz_33k3BPS_CFG3;
            break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_8MHz_40kBPS_CFG1;
            cfg2 = MCP_8MHz_40kBPS_CFG2;
            cfg3 = MCP_8MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_8MHz_50kBPS_CFG1;
            cfg2 = MCP_8MHz_50kBPS_CFG2;
            cfg3 = MCP_8MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_8MHz_80kBPS_CFG1;
            cfg2 = MCP_8MHz_80kBPS_CFG2;
            cfg3 = MCP_8MHz_80kBPS_CFG3;
            break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_8MHz_100kBPS_CFG1;
            cfg2 = MCP_8MHz_100kBPS_CFG2;
            cfg3 = MCP_8MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_8MHz_125kBPS_CFG1;
            cfg2 = MCP_8MHz_125kBPS_CFG2;
            cfg3 = MCP_8MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_8MHz_200kBPS_CFG1;
            cfg2 = MCP_8MHz_200kBPS_CFG2;
            cfg3 = MCP_8MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_8MHz_250kBPS_CFG1;
            cfg2 = MCP_8MHz_250kBPS_CFG2;
            cfg3 = MCP_8MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_8MHz_500kBPS_CFG1;
            cfg2 = MCP_8MHz_500kBPS_CFG2;
            cfg3 = MCP_8MHz_500kBPS_CFG3;
            break;
        
            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_8MHz_1000kBPS_CFG1;
            cfg2 = MCP_8MHz_1000kBPS_CFG2;
            cfg3 = MCP_8MHz_1000kBPS_CFG3;
            break;  

            default:
            set = 0;
	    return MCP2515_FAIL;
            break;
        }
        break;

        case (MCP_16MHZ):
        switch (canSpeed) 
        {
            case (CAN_5KBPS):                                               //   5Kbps
            cfg1 = MCP_16MHz_5kBPS_CFG1;
            cfg2 = MCP_16MHz_5kBPS_CFG2;
            cfg3 = MCP_16MHz_5kBPS_CFG3;
            break;

            case (CAN_10KBPS):                                              //  10Kbps
            cfg1 = MCP_16MHz_10kBPS_CFG1;
            cfg2 = MCP_16MHz_10kBPS_CFG2;
            cfg3 = MCP_16MHz_10kBPS_CFG3;
            break;

            case (CAN_20KBPS):                                              //  20Kbps
            cfg1 = MCP_16MHz_20kBPS_CFG1;
            cfg2 = MCP_16MHz_20kBPS_CFG2;
            cfg3 = MCP_16MHz_20kBPS_CFG3;
            break;

            case (CAN_33K3BPS):                                              //  20Kbps
            cfg1 = MCP_16MHz_33k3BPS_CFG1;
            cfg2 = MCP_16MHz_33k3BPS_CFG2;
            cfg3 = MCP_16MHz_33k3BPS_CFG3;
            break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_16MHz_40kBPS_CFG1;
            cfg2 = MCP_16MHz_40kBPS_CFG2;
            cfg3 = MCP_16MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg2 = MCP_16MHz_50kBPS_CFG2;
            cfg3 = MCP_16MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_16MHz_80kBPS_CFG1;
            cfg2 = MCP_16MHz_80kBPS_CFG2;
            cfg3 = MCP_16MHz_80kBPS_CFG3;
            break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_16MHz_100kBPS_CFG1;
            cfg2 = MCP_16MHz_100kBPS_CFG2;
            cfg3 = MCP_16MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_16MHz_125kBPS_CFG1;
            cfg2 = MCP_16MHz_125kBPS_CFG2;
            cfg3 = MCP_16MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_16MHz_200kBPS_CFG1;
            cfg2 = MCP_16MHz_200kBPS_CFG2;
            cfg3 = MCP_16MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_16MHz_250kBPS_CFG1;
            cfg2 = MCP_16MHz_250kBPS_CFG2;
            cfg3 = MCP_16MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_16MHz_500kBPS_CFG1;
            cfg2 = MCP_16MHz_500kBPS_CFG2;
            cfg3 = MCP_16MHz_500kBPS_CFG3;
            break;
        
            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_16MHz_1000kBPS_CFG1;
            cfg2 = MCP_16MHz_1000kBPS_CFG2;
            cfg3 = MCP_16MHz_1000kBPS_CFG3;
            break;  

            default:
            set = 0;
	    return MCP2515_FAIL;
            break;
        }
        break;
        
        case (MCP_20MHZ):
        switch (canSpeed) 
        {
            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_20MHz_40kBPS_CFG1;
            cfg2 = MCP_20MHz_40kBPS_CFG2;
            cfg3 = MCP_20MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_20MHz_50kBPS_CFG1;
            cfg2 = MCP_20MHz_50kBPS_CFG2;
            cfg3 = MCP_20MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_20MHz_80kBPS_CFG1;
            cfg2 = MCP_20MHz_80kBPS_CFG2;
            cfg3 = MCP_20MHz_80kBPS_CFG3;
            break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_20MHz_100kBPS_CFG1;
            cfg2 = MCP_20MHz_100kBPS_CFG2;
            cfg3 = MCP_20MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_20MHz_125kBPS_CFG1;
            cfg2 = MCP_20MHz_125kBPS_CFG2;
            cfg3 = MCP_20MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_20MHz_200kBPS_CFG1;
            cfg2 = MCP_20MHz_200kBPS_CFG2;
            cfg3 = MCP_20MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_20MHz_250kBPS_CFG1;
            cfg2 = MCP_20MHz_250kBPS_CFG2;
            cfg3 = MCP_20MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_20MHz_500kBPS_CFG1;
            cfg2 = MCP_20MHz_500kBPS_CFG2;
            cfg3 = MCP_20MHz_500kBPS_CFG3;
            break;
        
            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_20MHz_1000kBPS_CFG1;
            cfg2 = MCP_20MHz_1000kBPS_CFG2;
            cfg3 = MCP_20MHz_1000kBPS_CFG3;
            break;  

            default:
            set = 0;
            return MCP2515_FAIL;
            break;
        }
        break;
        
        default:
        set = 0;
	return MCP2515_FAIL;
        break;
    }

    if (canClock & MCP_CLKOUT_ENABLE) {
        cfg3 &= (~SOF_ENABLE);
    }

    if (set) {
        mcp2515_setRegister(state, MCP_CNF1, cfg1);
        mcp2515_setRegister(state, MCP_CNF2, cfg2);
        mcp2515_setRegister(state, MCP_CNF3, cfg3);
        return MCP2515_OK;
    }
     
    return MCP2515_FAIL;
}

/*********************************************************************************************************
** Function name:           mcp2515_initCANBuffers
** Descriptions:            Initialize Buffers, Masks, and Filters
*********************************************************************************************************/
INLINE void mcp2515_initCANBuffers(MCPCAN *state)
{
    uint8_t i, a1, a2, a3;
    
    uint8_t std = 0;               
    uint8_t ext = 1;
    uint32_t ulMask = 0x00, ulFilt = 0x00;


    mcp2515_write_mf(state, MCP_RXM0SIDH, ext, ulMask);			/*Set both masks to 0           */
    mcp2515_write_mf(state, MCP_RXM1SIDH, ext, ulMask);			/*Mask register ignores ext bit */
    
                                                                        /* Set all filters to 0         */
    mcp2515_write_mf(state, MCP_RXF0SIDH, ext, ulFilt);			/* RXB0: extended               */
    mcp2515_write_mf(state, MCP_RXF1SIDH, std, ulFilt);			/* RXB1: standard               */
    mcp2515_write_mf(state, MCP_RXF2SIDH, ext, ulFilt);			/* RXB2: extended               */
    mcp2515_write_mf(state, MCP_RXF3SIDH, std, ulFilt);			/* RXB3: standard               */
    mcp2515_write_mf(state, MCP_RXF4SIDH, ext, ulFilt);
    mcp2515_write_mf(state, MCP_RXF5SIDH, std, ulFilt);

                                                                        /* Clear, deactivate the three  */
                                                                        /* transmit buffers             */
                                                                        /* TXBnCTRL -> TXBnD7           */
    a1 = MCP_TXB0CTRL;
    a2 = MCP_TXB1CTRL;
    a3 = MCP_TXB2CTRL;
    for (i = 0; i < 14; i++) {                                          /* in-buffer loop               */
        mcp2515_setRegister(state, a1, 0);
        mcp2515_setRegister(state, a2, 0);
        mcp2515_setRegister(state, a3, 0);
        a1++;
        a2++;
        a3++;
    }
    mcp2515_setRegister(state, MCP_RXB0CTRL, 0);
    mcp2515_setRegister(state, MCP_RXB1CTRL, 0);
}

/*********************************************************************************************************
** Function name:           mcp2515_init
** Descriptions:            Initialize the controller
*********************************************************************************************************/
INLINE uint8_t mcp2515_init(MCPCAN *state, const uint8_t canIDMode, const uint8_t canSpeed, const uint8_t canClock)
{

  uint8_t res;

    mcp2515_reset(state);
    
    state->mcpMode = MCP_LOOPBACK;

    res = mcp2515_setCANCTRL_Mode(state, MODE_CONFIG);
    if(res > 0)
    {
      return res;
    }

    // Set Baudrate
    if(mcp2515_configRate(state, canSpeed, canClock))
    {
      return res;
    }

    if ( res == MCP2515_OK ) {

                                                                        /* init canbuffers              */
        mcp2515_initCANBuffers(state);

                                                                        /* interrupt mode               */
        mcp2515_setRegister(state, MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);

        //Sets BF pins as GPO
        mcp2515_setRegister(state, MCP_BFPCTRL, MCP_BxBFS_MASK | MCP_BxBFE_MASK);
        //Sets RTS pins as GPI
        mcp2515_setRegister(state, MCP_TXRTSCTRL,0x00);

        switch(canIDMode)
        {
            case (MCP_ANY):
                mcp2515_modifyRegister(state, MCP_RXB0CTRL,
                MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
                MCP_RXB_RX_ANY | MCP_RXB_BUKT_MASK);
                mcp2515_modifyRegister(state, MCP_RXB1CTRL, MCP_RXB_RX_MASK,
                MCP_RXB_RX_ANY);
            break;
/*          The followingn two functions of the MCP2515 do not work, there is a bug in the silicon.
            case (MCP_STD): 
            mcp2515_modifyRegister(state, MCP_RXB0CTRL,
            MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
            MCP_RXB_RX_STD | MCP_RXB_BUKT_MASK );
            mcp2515_modifyRegister(state, MCP_RXB1CTRL, MCP_RXB_RX_MASK,
            MCP_RXB_RX_STD);
            break;

            case (MCP_EXT): 
            mcp2515_modifyRegister(state, MCP_RXB0CTRL,
            MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
            MCP_RXB_RX_EXT | MCP_RXB_BUKT_MASK );
            mcp2515_modifyRegister(state, MCP_RXB1CTRL, MCP_RXB_RX_MASK,
            MCP_RXB_RX_EXT);
            break;
*/
            case (MCP_STDEXT): 
                mcp2515_modifyRegister(state, MCP_RXB0CTRL,
                MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
                MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK );
                mcp2515_modifyRegister(state, MCP_RXB1CTRL, MCP_RXB_RX_MASK,
                MCP_RXB_RX_STDEXT);
            break;
    
            default:
            return MCP2515_FAIL;
            break;
}    


        res = mcp2515_setCANCTRL_Mode(state, state->mcpMode);                                                                
        if(res)
        {
          return res;
        }


	if (canClock & MCP_CLKOUT_ENABLE) {
		uint8_t rv = mcp2515_requestClkOut(state, CLKOUT_ENABLE | CLKOUT_PS1);
		if (rv) {
			res = rv;
		}
	}

    }
    return res;

}

/*********************************************************************************************************
** Function name:           mcp2515_write_id
** Descriptions:            Write CAN ID
*********************************************************************************************************/
void mcp2515_write_id(MCPCAN *state,  const uint8_t mcp_addr, const uint8_t ext, const uint32_t id )
{
    uint16_t canid;
    uint8_t tbufdata[4];

    canid = (uint16_t)(id & 0x0FFFF);

    if ( ext == 1) 
    {
        tbufdata[MCP_EID0] = (uint8_t) (canid & 0xFF);
        tbufdata[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (uint8_t) (canid & 0x03);
        tbufdata[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 5 );
    }
    else 
    {
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 3 );
        tbufdata[MCP_SIDL] = (uint8_t) ((canid & 0x07 ) << 5);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;
    }
    
    mcp2515_setRegisterS(state,  mcp_addr, tbufdata, 4 );
}

/*********************************************************************************************************
** Function name:           mcp2515_write_mf
** Descriptions:            Write Masks and Filters
*********************************************************************************************************/
void mcp2515_write_mf(MCPCAN *state,  const uint8_t mcp_addr, const uint8_t ext, const uint32_t id )
{
    uint16_t canid;
    uint8_t tbufdata[4];

    canid = (uint16_t)(id & 0x0FFFF);

    if ( ext == 1) 
    {
        tbufdata[MCP_EID0] = (uint8_t) (canid & 0xFF);
        tbufdata[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (uint8_t) (canid & 0x03);
        tbufdata[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 5 );
    }
    else 
    {
        tbufdata[MCP_EID0] = (uint8_t) (canid & 0xFF);
        tbufdata[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (uint8_t) ((canid & 0x07) << 5);
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 3 );
    }
    
    mcp2515_setRegisterS(state,  mcp_addr, tbufdata, 4 );
}

/*********************************************************************************************************
** Function name:           mcp2515_read_id
** Descriptions:            Read CAN ID
*********************************************************************************************************/
void mcp2515_read_id(MCPCAN *state,  const uint8_t mcp_addr, uint8_t* ext, uint32_t* id )
{
    uint8_t tbufdata[4];

    *ext = 0;
    *id = 0;

    mcp2515_readRegisterS(state, mcp_addr, tbufdata, 4 );

    *id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

    if ( (tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) ==  MCP_TXB_EXIDE_M ) 
    {
                                                                        /* extended id                  */
        *id = (*id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        *id = (*id<<8) + tbufdata[MCP_EID8];
        *id = (*id<<8) + tbufdata[MCP_EID0];
        *ext = 1;
    }
}

/*********************************************************************************************************
** Function name:           mcp2515_write_canMsg
** Descriptions:            Write message
*********************************************************************************************************/
void mcp2515_write_canMsg(MCPCAN *state,  const uint8_t buffer_sidh_addr)
{
    uint8_t mcp_addr;
    mcp_addr = buffer_sidh_addr;
    mcp2515_setRegisterS(state, mcp_addr+5, state->m_nDta, state->m_nDlc );                  /* write data bytes             */
	
    if ( state->m_nRtr == 1)                                                   /* if RTR set bit in byte       */
        state->m_nDlc |= MCP_RTR_MASK;  

    mcp2515_setRegister(state, (mcp_addr+4), state->m_nDlc );                         /* write the RTR and DLC        */
    mcp2515_write_id(state, mcp_addr, state->m_nExtFlg, state->m_nID );                      /* write CAN id                 */

}

/*********************************************************************************************************
** Function name:           mcp2515_read_canMsg
** Descriptions:            Read message
*********************************************************************************************************/
inline void mcp2515_read_canMsg(MCPCAN *state,  const uint8_t buffer_sidh_addr)        /* read can msg                 */
{
    uint8_t mcp_addr, ctrl;

    mcp_addr = buffer_sidh_addr;

    mcp2515_read_id(state,  mcp_addr, &state->m_nExtFlg,&state->m_nID );

    ctrl = mcp2515_readRegister(state, mcp_addr-1 );
    state->m_nDlc = mcp2515_readRegister(state, mcp_addr+4 );

    if (ctrl & 0x08)
        state->m_nRtr = 1;
    else
        state->m_nRtr = 0;

    state->m_nDlc &= MCP_DLC_MASK;
    mcp2515_readRegisterS(state, mcp_addr+5, &(state->m_nDta[0]), state->m_nDlc );
}

/*********************************************************************************************************
** Function name:           mcp2515_getNextFreeTXBuf
** Descriptions:            Send message
*********************************************************************************************************/
uint8_t mcp2515_getNextFreeTXBuf(MCPCAN *state, uint8_t *txbuf_n)                 /* get Next free txbuf          */
{
    uint8_t res, i, ctrlval;
    uint8_t ctrlregs[MCP_N_TXBUFFERS] = { MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };

    res = MCP_ALLTXBUSY;
    *txbuf_n = 0x00;

                                                                        /* check all 3 TX-Buffers       */
    for (i=0; i<MCP_N_TXBUFFERS; i++) {
        ctrlval = mcp2515_readRegister(state, ctrlregs[i] );
        if ( (ctrlval & MCP_TXB_TXREQ_M) == 0 ) {
            *txbuf_n = ctrlregs[i]+1;                                   /* return SIDH-address of Buffer*/
            
            res = MCP2515_OK;
            return res;                                                 /* ! function exit              */
        }
    }
    return res;
}

/*********************************************************************************************************
** Function name:           MCP_CAN
** Descriptions:            Public function to declare CAN class and the /CS pin.
*********************************************************************************************************/
#ifdef ARDUINO_CORE
void MCP_CAN(MCPCAN *state, uint8_t _CS)
{
    state->MCPCS = _CS;
    SPI_DESELECT(state);
    pinMode(state->MCPCS, OUTPUT);
}
#else
void MCP_CAN(MCPCAN *state, uint8_t _CS_PIN)
{
    state->MCPCS = _CS_PIN;
    DDRB &= ~(1 << _CS_PIN);
    SPI_DESELECT(state);
}
#endif

/*********************************************************************************************************
** Function name:           begin
** Descriptions:            Public function to declare controller initialization parameters.
*********************************************************************************************************/
uint8_t MCP_CAN_begin(MCPCAN *state, uint8_t idmodeset, uint8_t speedset, uint8_t clockset)
{
    uint8_t res;
    SPI_INIT;
    res = mcp2515_init(state, idmodeset, speedset, clockset);
    if (res == MCP2515_OK)
        return CAN_OK;
    
    return CAN_FAILINIT;
}

/*********************************************************************************************************
** Function name:           init_Mask
** Descriptions:            Public function to set mask(s).
*********************************************************************************************************/
uint8_t MCP_CAN_init_Mask_ext(MCPCAN *state, uint8_t num, uint8_t ext, uint32_t ulData)
{
    uint8_t res = MCP2515_OK;
    res = mcp2515_setCANCTRL_Mode(state, MODE_CONFIG);
    if(res > 0){
	    return res;
    }
    
    if (num == 0){
        mcp2515_write_mf(state, MCP_RXM0SIDH, ext, ulData);

    }
    else if(num == 1){
        mcp2515_write_mf(state, MCP_RXM1SIDH, ext, ulData);
    }
    else res =  MCP2515_FAIL;
    
    res = mcp2515_setCANCTRL_Mode(state, state->mcpMode);
    if(res > 0){
	return res;
    }
    return res;
}

/*********************************************************************************************************
** Function name:           init_Mask
** Descriptions:            Public function to set mask(s).
*********************************************************************************************************/
uint8_t MCP_CAN_init_Mask(MCPCAN *state, uint8_t num, uint32_t ulData)
{
    uint8_t res = MCP2515_OK;
    uint8_t ext = 0;
    res = mcp2515_setCANCTRL_Mode(state, MODE_CONFIG);
    if(res > 0){
  return res;
}
    
    if((ulData & 0x80000000) == 0x80000000)
        ext = 1;
    
    if (num == 0){
        mcp2515_write_mf(state, MCP_RXM0SIDH, ext, ulData);

    }
    else if(num == 1){
        mcp2515_write_mf(state, MCP_RXM1SIDH, ext, ulData);
    }
    else res =  MCP2515_FAIL;
    
    res = mcp2515_setCANCTRL_Mode(state, state->mcpMode);
    if(res > 0){
    return res;
  }
    return res;
}

/*********************************************************************************************************
** Function name:           init_Filt
** Descriptions:            Public function to set filter(s).
*********************************************************************************************************/
uint8_t MCP_CAN_init_Filt_ext(MCPCAN *state, uint8_t num, uint8_t ext, uint32_t ulData)
{
    uint8_t res = MCP2515_OK;
    res = mcp2515_setCANCTRL_Mode(state, MODE_CONFIG);
    if(res > 0)
    {
      return res;
    }
    
    switch( num )
    {
        case 0:
        mcp2515_write_mf(state, MCP_RXF0SIDH, ext, ulData);
        break;

        case 1:
        mcp2515_write_mf(state, MCP_RXF1SIDH, ext, ulData);
        break;

        case 2:
        mcp2515_write_mf(state, MCP_RXF2SIDH, ext, ulData);
        break;

        case 3:
        mcp2515_write_mf(state, MCP_RXF3SIDH, ext, ulData);
        break;

        case 4:
        mcp2515_write_mf(state, MCP_RXF4SIDH, ext, ulData);
        break;

        case 5:
        mcp2515_write_mf(state, MCP_RXF5SIDH, ext, ulData);
        break;

        default:
        res = MCP2515_FAIL;
    }
    
    res = mcp2515_setCANCTRL_Mode(state, state->mcpMode);
    if(res > 0)
    {
      return res;
    }
    
    return res;
}

/*********************************************************************************************************
** Function name:           init_Filt
** Descriptions:            Public function to set filter(s).
*********************************************************************************************************/
uint8_t MCP_CAN_init_Filt(MCPCAN *state, uint8_t num, uint32_t ulData)
{
    uint8_t res = MCP2515_OK;
    uint8_t ext = 0;
    
    res = mcp2515_setCANCTRL_Mode(state, MODE_CONFIG);
    if(res > 0)
    {
      return res;
    }
    
    if((ulData & 0x80000000) == 0x80000000)
        ext = 1;
    
    switch( num )
    {
        case 0:
        mcp2515_write_mf(state, MCP_RXF0SIDH, ext, ulData);
        break;

        case 1:
        mcp2515_write_mf(state, MCP_RXF1SIDH, ext, ulData);
        break;

        case 2:
        mcp2515_write_mf(state, MCP_RXF2SIDH, ext, ulData);
        break;

        case 3:
        mcp2515_write_mf(state, MCP_RXF3SIDH, ext, ulData);
        break;

        case 4:
        mcp2515_write_mf(state, MCP_RXF4SIDH, ext, ulData);
        break;

        case 5:
        mcp2515_write_mf(state, MCP_RXF5SIDH, ext, ulData);
        break;

        default:
        res = MCP2515_FAIL;
    }
    
    res = mcp2515_setCANCTRL_Mode(state, state->mcpMode);
    if(res > 0)
    {
      return res;
    }
    
    return res;
}

/*********************************************************************************************************
** Function name:           setMsg
** Descriptions:            Set can message, such as dlc, id, dta[] and so on
*********************************************************************************************************/
uint8_t MCP_CAN_setMsg(MCPCAN *state, uint32_t id, uint8_t rtr, uint8_t ext, uint8_t len, uint8_t *pData)
{
    int i = 0;
    state->m_nID     = id;
    state->m_nRtr    = rtr;
    state->m_nExtFlg = ext;
    state->m_nDlc    = len;
    for(i = 0; i<len; i++)
        state->m_nDta[i] = pData[i];
	
    return MCP2515_OK;
}

/*********************************************************************************************************
** Function name:           clearMsg
** Descriptions:            Set all messages to zero
*********************************************************************************************************/
INLINE uint8_t MCP_CAN_clearMsg(MCPCAN *state)
{
    state->m_nID       = 0;
    state->m_nDlc      = 0;
    state->m_nExtFlg   = 0;
    state->m_nRtr      = 0;
    state->m_nfilhit   = 0;
    for(int i = 0; i<state->m_nDlc; i++ )
      state->m_nDta[i] = 0x00;

    return MCP2515_OK;
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            Send message
*********************************************************************************************************/
uint8_t MCP_CAN_sendMsg(MCPCAN *state)
{
    uint8_t res, txbuf_n;
    uint16_t uiTimeOut = 0;

    do {
        res = mcp2515_getNextFreeTXBuf(state, &txbuf_n);                       /* info = addr.                 */
        uiTimeOut++;
    } while (res == MCP_ALLTXBUSY && (uiTimeOut < TIMEOUTVALUE));

    /*if(uiTimeOut == TIMEOUTVALUE) 
    {   
        return CAN_GETTXBFTIMEOUT;                                    
    }*/
    uiTimeOut = 0;
    mcp2515_write_canMsg(state, txbuf_n);
    mcp2515_modifyRegister(state, txbuf_n-1 , MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M );
    
    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            Send message to transmitt buffer
*********************************************************************************************************/
uint8_t MCP_CAN_sendMsgBuf_ext(MCPCAN *state, uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf)
{
    uint8_t res;
	
    MCP_CAN_setMsg(state, id, 0, ext, len, buf);
    res = MCP_CAN_sendMsg(state);
    
    return res;
}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            Send message to transmitt buffer
*********************************************************************************************************/
uint8_t MCP_CAN_sendMsgBuf(MCPCAN *state, uint32_t id, uint8_t len, uint8_t *buf)
{
    uint8_t ext = 0, rtr = 0;
    uint8_t res;
    
    if((id & 0x80000000) == 0x80000000)
        ext = 1;
 
    if((id & 0x40000000) == 0x40000000)
        rtr = 1;
        
    MCP_CAN_setMsg(state, id, rtr, ext, len, buf);
    res = MCP_CAN_sendMsg(state);
    
    return res;
}

/*********************************************************************************************************
** Function name:           readMsg
** Descriptions:            Read message
*********************************************************************************************************/
INLINE uint8_t MCP_CAN_readMsg(MCPCAN *state)
{
    uint8_t stat, res;

    stat = mcp2515_readStatus(state);

    if ( stat & MCP_STAT_RX0IF )                                        /* Msg in Buffer 0              */
    {
        mcp2515_read_canMsg(state,  MCP_RXBUF_0);
        mcp2515_modifyRegister(state, MCP_CANINTF, MCP_RX0IF, 0);
        res = CAN_OK;
    }
    else if ( stat & MCP_STAT_RX1IF )                                   /* Msg in Buffer 1              */
    {
        mcp2515_read_canMsg(state,  MCP_RXBUF_1);
        mcp2515_modifyRegister(state, MCP_CANINTF, MCP_RX1IF, 0);
        res = CAN_OK;
    }
    else  {
        res = CAN_NOMSG;
    }
    
    return res;
}

void resetInt(MCPCAN *state)
{
    mcp2515_read_canMsg(state, MCP_RXBUF_0);
    mcp2515_read_canMsg(state, MCP_RXBUF_1);
    mcp2515_modifyRegister(state, MCP_CANINTF, MCP_RX0IF | MCP_RX1IF | MCP_ERRIF, 0);
}

uint8_t getInt(MCPCAN *state) {
    return mcp2515_readRegister(state, MCP_CANINTF);
}
/*********************************************************************************************************
** Function name:           readMsgBuf
** Descriptions:            Public function, Reads message from receive buffer.
*********************************************************************************************************/
uint8_t MCP_CAN_readMsgBuf_ext(MCPCAN *state, uint32_t *id, uint8_t *ext, uint8_t *len, uint8_t buf[])
{
    if(MCP_CAN_readMsg(state) == CAN_NOMSG)
	return CAN_NOMSG;
	
    *id  = state->m_nID;
    *len = state->m_nDlc;
    *ext = state->m_nExtFlg;
    for(int i = 0; i<state->m_nDlc; i++)
        buf[i] = state->m_nDta[i];

    return CAN_OK;
}


/*********************************************************************************************************
** Function name:           readMsgBuf
** Descriptions:            Public function, Reads message from receive buffer.
*********************************************************************************************************/
uint8_t MCP_CAN_readMsgBuf(MCPCAN *state, uint32_t *id, uint8_t *len, uint8_t buf[])
{
    if(MCP_CAN_readMsg(state) == CAN_NOMSG) {
    	return CAN_NOMSG;
    }

    if (state->m_nExtFlg)
        state->m_nID |= 0x80000000;

    if (state->m_nRtr)
        state->m_nID |= 0x40000000;
	
    *id  = state->m_nID;
    *len = state->m_nDlc;
    
    for(int i = 0; i < state->m_nDlc; i++)
        buf[i] = state->m_nDta[i];

    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           checkReceive
** Descriptions:            Public function, Checks for received data.  (Used if not using the interrupt output)
*********************************************************************************************************/
uint8_t MCP_CAN_checkReceive(MCPCAN *state)
{
    uint8_t res;
    res = mcp2515_readStatus(state);                                         /* RXnIF in Bit 1 and 0         */
    if ( res & MCP_STAT_RXIF_MASK )
        return CAN_MSGAVAIL;
    else 
        return CAN_NOMSG;
}

/*********************************************************************************************************
** Function name:           checkError
** Descriptions:            Public function, Returns error register data.
*********************************************************************************************************/
uint8_t MCP_CAN_checkError(MCPCAN *state)
{
    uint8_t eflg = mcp2515_readRegister(state, MCP_EFLG);

    if ( eflg & MCP_EFLG_ERRORMASK ) 
        return CAN_CTRLERROR;
    else
        return CAN_OK;
}

/*********************************************************************************************************
** Function name:           getError
** Descriptions:            Returns error register value.
*********************************************************************************************************/
uint8_t MCP_CAN_getError(MCPCAN *state)
{
    return mcp2515_readRegister(state, MCP_EFLG);
}

/*********************************************************************************************************
** Function name:           mcp2515_errorCountRX
** Descriptions:            Returns REC register value
*********************************************************************************************************/
uint8_t MCP_CAN_errorCountRX(MCPCAN *state)                             
{
    return mcp2515_readRegister(state, MCP_REC);
}

/*********************************************************************************************************
** Function name:           mcp2515_errorCountTX
** Descriptions:            Returns TEC register value
*********************************************************************************************************/
uint8_t MCP_CAN_errorCountTX(MCPCAN *state)                             
{
    return mcp2515_readRegister(state, MCP_TEC);
}

/*********************************************************************************************************
** Function name:           mcp2515_enOneShotTX
** Descriptions:            Enables one shot transmission mode
*********************************************************************************************************/
uint8_t MCP_CAN_enOneShotTX(MCPCAN *state)                             
{
    mcp2515_modifyRegister(state, MCP_CANCTRL, MODE_ONESHOT, MODE_ONESHOT);
    if((mcp2515_readRegister(state, MCP_CANCTRL) & MODE_ONESHOT) != MODE_ONESHOT)
	    return CAN_FAIL;
    else
	    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           mcp2515_disOneShotTX
** Descriptions:            Disables one shot transmission mode
*********************************************************************************************************/
uint8_t MCP_CAN_disOneShotTX(MCPCAN *state)                             
{
    mcp2515_modifyRegister(state, MCP_CANCTRL, MODE_ONESHOT, 0);
    if((mcp2515_readRegister(state, MCP_CANCTRL) & MODE_ONESHOT) != 0)
        return CAN_FAIL;
    else
        return CAN_OK;
}

/*********************************************************************************************************
** Function name:           mcp2515_abortTX
** Descriptions:            Aborts any queued transmissions
*********************************************************************************************************/
uint8_t MCP_CAN_abortTX(MCPCAN *state)                             
{
    mcp2515_modifyRegister(state, MCP_CANCTRL, ABORT_TX, ABORT_TX);
	
    // Maybe check to see if the TX buffer transmission request bits are cleared instead?
    if((mcp2515_readRegister(state, MCP_CANCTRL) & ABORT_TX) != ABORT_TX)
	    return CAN_FAIL;
    else
	    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           setGPO
** Descriptions:            Public function, Checks for r
*********************************************************************************************************/
uint8_t MCP_CAN_setGPO(MCPCAN *state, uint8_t data)
{
    mcp2515_modifyRegister(state, MCP_BFPCTRL, MCP_BxBFS_MASK, (data<<4));
	    
    return 0;
}

/*********************************************************************************************************
** Function name:           getGPI
** Descriptions:            Public function, Checks for r
*********************************************************************************************************/
uint8_t MCP_CAN_getGPI(MCPCAN *state)
{
    uint8_t res;
    res = mcp2515_readRegister(state, MCP_TXRTSCTRL) & MCP_BxRTS_MASK;
    return (res >> 3);
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
