/*
  mcp_can.h
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
#ifndef _MCP2515_H_
#define _MCP2515_H_

#include "mcp_can_dfs.h"
#define MAX_CHAR_IN_MESSAGE 8

typedef struct MCPCAN_t
{
    uint32_t  m_nID;                                                      // CAN ID
    uint8_t   m_nExtFlg;                                                  // Identifier Type
    uint8_t   m_nDlc;                                                     // Data Length Code
    uint8_t   m_nDta[MAX_CHAR_IN_MESSAGE];                                // Data array
    uint8_t   m_nRtr;                                                     // Remote request flag
    uint8_t   m_nfilhit;                                                  // The number of the filter that matched the message
    uint8_t   MCPCS;                                                      // Chip Select pin number
    uint8_t   mcpMode;                                                    // Mode to return to after configurations are performed.
} MCPCAN; 
/*********************************************************************************************************
 *  mcp2515 driver functions in C 
 *********************************************************************************************************/

void    mcp2515_reset(MCPCAN *state);                                           // Soft Reset MCP2515
uint8_t mcp2515_readRegister(MCPCAN *state, const uint8_t address);                    // Read MCP2515 register

void    mcp2515_readRegisterS(MCPCAN *state, const uint8_t address,                     // Read MCP2515 successive registers
                                uint8_t values[], 
                                const uint8_t n);

void    mcp2515_setRegister(MCPCAN *state, const uint8_t address,                       // Set MCP2515 register
                            const uint8_t value);

void    mcp2515_setRegisterS(MCPCAN *state, const uint8_t address,                      // Set MCP2515 successive registers
                                uint8_t values[],
                                const uint8_t n);

void    mcp2515_initCANBuffers(MCPCAN *state);

void    mcp2515_modifyRegister(MCPCAN *state, const uint8_t address,                    // Set specific bit(s) of a register
                            const uint8_t mask,
                            const uint8_t data);

uint8_t mcp2515_readStatus(MCPCAN *state);                                     // Read MCP2515 Status
uint8_t mcp2515_setCANCTRL_Mode(MCPCAN *state, const uint8_t newmode);                 // Set mode
uint8_t mcp2515_requestNewMode(MCPCAN *state, const uint8_t newmode);                  // Set mode
uint8_t mcp2515_requestClkOut(MCPCAN *state, const uint8_t newmode); 			// Set new clkmode
uint8_t mcp2515_configRate(MCPCAN *state, const uint8_t canSpeed,                      // Set baudrate
                            const uint8_t canClock);
                            
uint8_t mcp2515_init(MCPCAN *state, const uint8_t canIDMode,                           // Initialize Controller
                        const uint8_t canSpeed,
                        const uint8_t canClock);

void    mcp2515_write_mf(MCPCAN *state, const uint8_t mcp_addr,                        // Write CAN Mask or Filter
                            const uint8_t ext,
                            const uint32_t id );

void    mcp2515_write_id(MCPCAN *state, const uint8_t mcp_addr,                        // Write CAN ID
                            const uint8_t ext,
                            const uint32_t id );

void    mcp2515_read_id(MCPCAN *state, const uint8_t mcp_addr,                         // Read CAN ID
                        uint8_t* ext,
                        uint32_t* id );

void    mcp2515_write_canMsg(MCPCAN *state, const uint8_t buffer_sidh_addr );          // Write CAN message
void    mcp2515_read_canMsg(MCPCAN *state, const uint8_t buffer_sidh_addr);            // Read CAN message
uint8_t mcp2515_getNextFreeTXBuf(MCPCAN *state, uint8_t *txbuf_n);                     // Find empty transmit buffer

/*********************************************************************************************************
 *  CAN operator function
 *********************************************************************************************************/

uint8_t MCP_CAN_setMsg(MCPCAN *state, uint32_t id, uint8_t rtr, uint8_t ext, uint8_t len, uint8_t *pData);        // Set message
uint8_t MCP_CAN_clearMsg(MCPCAN *state);                                                   // Clear all message to zero
uint8_t MCP_CAN_readMsg(MCPCAN *state);                                                    // Read message
uint8_t MCP_CAN_sendMsg(MCPCAN *state);                                                    // Send message

void    MCP_CAN(MCPCAN *state, uint8_t _CS);
uint8_t MCP_CAN_begin(MCPCAN *state, uint8_t idmodeset, uint8_t speedset, uint8_t clockset);       // Initilize controller prameters
uint8_t MCP_CAN_init_Mask_ext(MCPCAN *state, uint8_t num, uint8_t ext, uint32_t ulData);               // Initilize Mask(s)
uint8_t MCP_CAN_init_Mask(MCPCAN *state, uint8_t num, uint32_t ulData);                          // Initilize Mask(s)
uint8_t MCP_CAN_init_Filt_ext(MCPCAN *state, uint8_t num, uint8_t ext, uint32_t ulData);               // Initilize Filter(s)
uint8_t MCP_CAN_init_Filt(MCPCAN *state, uint8_t num, uint32_t ulData);                          // Initilize Filter(s)
void    MCP_CAN_setSleepWakeup(MCPCAN *state, uint8_t enable);                                  // Enable or disable the wake up interrupt (If disabled the MCP2515 will not be woken up by CAN bus activity)
uint8_t MCP_CAN_setMode(MCPCAN *state, uint8_t opMode);                                        // Set operational mode
uint8_t MCP_CAN_sendMsgBuf_ext(MCPCAN *state, uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf);      // Send message to transmit buffer
uint8_t MCP_CAN_sendMsgBuf(MCPCAN *state, uint32_t id, uint8_t len, uint8_t *buf);                 // Send message to transmit buffer
uint8_t MCP_CAN_readMsgBuf_ext(MCPCAN *state, uint32_t *id, uint8_t *ext, uint8_t *len, uint8_t *buf);   // Read message from receive buffer
uint8_t MCP_CAN_readMsgBuf(MCPCAN *state, uint32_t *id, uint8_t *len, uint8_t *buf);               // Read message from receive buffer
void    MCP_CAN_resetInt(MCPCAN *state);
uint8_t MCP_CAN_getInt(MCPCAN *state);
uint8_t MCP_CAN_checkReceive(MCPCAN *state);                                           // Check for received data
uint8_t MCP_CAN_checkError(MCPCAN *state);                                             // Check for errors
uint8_t MCP_CAN_getError(MCPCAN *state);                                               // Check for errors
uint8_t MCP_CAN_errorCountRX(MCPCAN *state);                                           // Get error count
uint8_t MCP_CAN_errorCountTX(MCPCAN *state);                                           // Get error count
uint8_t MCP_CAN_enOneShotTX(MCPCAN *state);                                            // Enable one-shot transmission
uint8_t MCP_CAN_disOneShotTX(MCPCAN *state);                                           // Disable one-shot transmission
uint8_t MCP_CAN_abortTX(MCPCAN *state);                                                // Abort queued transmission(s)
uint8_t MCP_CAN_setGPO(MCPCAN *state, uint8_t data);                                           // Sets GPO
uint8_t MCP_CAN_getGPI(MCPCAN *state);                                                 // Reads GPI

#endif
/*********************************************************************************************************
 *  END FILE
 *********************************************************************************************************/
