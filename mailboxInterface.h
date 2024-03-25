/*
 * Copyright (C) 2019-2024, HENSOLDT Cyber GmbH
 * 
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * For commercial licensing, contact: info.cyber@hensoldt.net
 */

#pragma once

/* Includes -------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Defines -------------------------------------------------------------*/
#define CODE_REQUEST		            0x00000000
#define CODE_RESPONSE_SUCCESS	        0x80000000
#define CODE_RESPONSE_FAILURE	        0x80000001

#define PROPTAG_GET_FIRMWARE_REVISION	0x00000001
#define PROPTAG_GET_BOARD_MODEL		    0x00010001
#define PROPTAG_GET_BOARD_REVISION	    0x00010002
#define PROPTAG_GET_MAC_ADDRESS		    0x00010003
#define PROPTAG_GET_BOARD_SERIAL	    0x00010004
#define PROPTAG_GET_ARM_MEMORY		    0x00010005
#define PROPTAG_GET_VC_MEMORY		    0x00010006
#define PROPTAG_SET_POWER_STATE		    0x00028001
#define PROPTAG_GET_CLOCK_RATE		    0x00030002
#define PROPTAG_GET_TEMPERATURE		    0x00030006
#define PROPTAG_GET_EDID_BLOCK		    0x00030020
#define PROPTAG_GET_DISPLAY_DIMENSIONS  0x00040003
#define PROPTAG_GET_COMMAND_LINE	    0x00050001
#define PROPTAG_END			            0x00000000

#define MAILBOX_CHANNEL                 0x8

/* Type declarations -------------------------------------------------------------*/
typedef struct MailboxInterface_PropertyBuffer
{
	uint32_t	nBufferSize;			// bytes
	uint32_t	nCode;
	uint8_t	    Tags[0];
}
MailboxInterface_PropertyBuffer;

typedef struct MailboxInterface_PropertyTag
{
	uint32_t	nTagId;
	uint32_t	nValueBufSize;			// bytes, multiple of 4
	uint32_t	nValueLength;			// bytes
}
MailboxInterface_PropertyTag;

/* Function declarations -------------------------------------------------------------*/
bool MailboxInterface_getTag(uint32_t nTagId, void *pTag, unsigned nTagSize, unsigned nRequestParmSize);