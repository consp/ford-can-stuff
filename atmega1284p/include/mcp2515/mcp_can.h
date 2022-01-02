/*
  mcp_can.h
  2012 Copyright (c) Seeed Technology Inc.  All right reserved.
  2017 Copyright (c) Cory J. Fowler  All Rights Reserved.

  Author:Loovee
  Contributor: Cory J. Fowler
  2017-09-25
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
#ifndef _MCP2515_H_
#define _MCP2515_H_

#include "mcp_can_dfs.h"
#define MAX_CHAR_IN_MESSAGE 8

class MCP_CAN
{
    private:
    
    uint8_t   m_nExtFlg;                                                  // Identifier Type
                                                                        // Extended (29 bit) or Standard (11 bit)
    uint32_t  m_nID;                                                      // CAN ID
    uint8_t   m_nDlc;                                                     // Data Length Code
    uint8_t   m_nDta[MAX_CHAR_IN_MESSAGE];                                // Data array
    uint8_t   m_nRtr;                                                     // Remote request flag
    uint8_t   m_nfilhit;                                                  // The number of the filter that matched the message
    uint8_t   MCPCS;                                                      // Chip Select pin number
    uint8_t   mcpMode;                                                    // Mode to return to after configurations are performed.
   
/*********************************************************************************************************
 *  mcp2515 driver function 
 *********************************************************************************************************/
   // private:
   private:

    void mcp2515_reset(void);                                           // Soft Reset MCP2515

    uint8_t mcp2515_readRegister(const uint8_t address);                    // Read MCP2515 register
    
    void mcp2515_readRegisterS(const uint8_t address,                     // Read MCP2515 successive registers
                                     uint8_t values[], 
                               const uint8_t n);

    void mcp2515_setRegister(const uint8_t address,                       // Set MCP2515 register
                             const uint8_t value);

    void mcp2515_setRegisterS(const uint8_t address,                      // Set MCP2515 successive registers
                              uint8_t values[],
                              const uint8_t n);

    void mcp2515_initCANBuffers(void);

    void mcp2515_modifyRegister(const uint8_t address,                    // Set specific bit(s) of a register
                                const uint8_t mask,
                                const uint8_t data);

    uint8_t mcp2515_readStatus(void);                                     // Read MCP2515 Status
    uint8_t mcp2515_setCANCTRL_Mode(const uint8_t newmode);                 // Set mode
    uint8_t mcp2515_requestNewMode(const uint8_t newmode);                  // Set mode
    uint8_t mcp2515_requestClkOut(const uint8_t newmode); 			// Set new clkmode
    uint8_t mcp2515_configRate(const uint8_t canSpeed,                      // Set baudrate

                             const uint8_t canClock);
                             
    uint8_t mcp2515_init(const uint8_t canIDMode,                           // Initialize Controller
                       const uint8_t canSpeed,
                       const uint8_t canClock);

    void mcp2515_write_mf( const uint8_t mcp_addr,                        // Write CAN Mask or Filter
                           const uint8_t ext,
                           const uint32_t id );

    void mcp2515_write_id( const uint8_t mcp_addr,                        // Write CAN ID
                           const uint8_t ext,
                           const uint32_t id );

    void mcp2515_read_id( const uint8_t mcp_addr,                         // Read CAN ID
      uint8_t* ext,
                                uint32_t* id );

    void mcp2515_write_canMsg( const uint8_t buffer_sidh_addr );          // Write CAN message
    void mcp2515_read_canMsg( const uint8_t buffer_sidh_addr);            // Read CAN message
    uint8_t mcp2515_getNextFreeTXBuf(uint8_t *txbuf_n);                     // Find empty transmit buffer

/*********************************************************************************************************
 *  CAN operator function
 *********************************************************************************************************/

    uint8_t setMsg(uint32_t id, uint8_t rtr, uint8_t ext, uint8_t len, uint8_t *pData);        // Set message
    uint8_t clearMsg();                                                   // Clear all message to zero
    uint8_t readMsg();                                                    // Read message
    uint8_t sendMsg();                                                    // Send message

public:
    MCP_CAN(uint8_t _CS);
    uint8_t begin(uint8_t idmodeset, uint8_t speedset, uint8_t clockset);       // Initilize controller prameters
    uint8_t init_Mask(uint8_t num, uint8_t ext, uint32_t ulData);               // Initilize Mask(s)
    uint8_t init_Mask(uint8_t num, uint32_t ulData);                          // Initilize Mask(s)
    uint8_t init_Filt(uint8_t num, uint8_t ext, uint32_t ulData);               // Initilize Filter(s)
    uint8_t init_Filt(uint8_t num, uint32_t ulData);                          // Initilize Filter(s)
    void setSleepWakeup(uint8_t enable);                                  // Enable or disable the wake up interrupt (If disabled the MCP2515 will not be woken up by CAN bus activity)
    uint8_t setMode(uint8_t opMode);                                        // Set operational mode
    uint8_t sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf);      // Send message to transmit buffer
    uint8_t sendMsgBuf(uint32_t id, uint8_t len, uint8_t *buf);                 // Send message to transmit buffer
    uint8_t readMsgBuf(uint32_t *id, uint8_t *ext, uint8_t *len, uint8_t *buf);   // Read message from receive buffer
    uint8_t readMsgBuf(uint32_t *id, uint8_t *len, uint8_t *buf);               // Read message from receive buffer
    void resetInt();
    uint8_t getInt(void);
    uint8_t checkReceive(void);                                           // Check for received data
    uint8_t checkError(void);                                             // Check for errors
    uint8_t getError(void);                                               // Check for errors
    uint8_t errorCountRX(void);                                           // Get error count
    uint8_t errorCountTX(void);                                           // Get error count
    uint8_t enOneShotTX(void);                                            // Enable one-shot transmission
    uint8_t disOneShotTX(void);                                           // Disable one-shot transmission
    uint8_t abortTX(void);                                                // Abort queued transmission(s)
    uint8_t setGPO(uint8_t data);                                           // Sets GPO
    uint8_t getGPI(void);                                                 // Reads GPI
};

#endif
/*********************************************************************************************************
 *  END FILE
 *********************************************************************************************************/
