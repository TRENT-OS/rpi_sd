/*
 * Copyright (C) 2019-2024, HENSOLDT Cyber GmbH
 * 
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * For commercial licensing, contact: info.cyber@hensoldt.net
 */

#include <string.h>

#include "mailboxInterface.h"

#include <circleos.h>
#include <circleenv/bcm2835.h>
#include <circleenv/bcm2711.h>
#include <circleenv/synchronize.h>

#include <camkes.h>

/* Defines ----------------------------------------------------------------*/
/*
	See: https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
*/
#define VALUE_LENGTH_RESPONSE	(1 << 31)

//	the actual mailbox base address is 0x3F00B880 but when mapping the
//	base address has to be PAGE_SIZE alligned
#define MAILBOX_BASE            ((unsigned long)mailboxBaseReg + 0x880)
#define MAILBOX0_READ           (MAILBOX_BASE + 0x00)
#define MAILBOX0_STATUS         (MAILBOX_BASE + 0x18)
#define MAILBOX1_WRITE          (MAILBOX_BASE + 0x20)
#define MAILBOX1_STATUS         (MAILBOX_BASE + 0x38)

#define MAILBOX_STATUS_EMPTY    0x40000000
#define MAILBOX_STATUS_FULL     0x80000000

/* Private function prototypes ----------------------------------------------------------------*/
static uint64_t read32(uint64_t nAddress);
static void write32(uint64_t nAddress, uint64_t nValue);
void MailboxFlush();
unsigned MailboxRead(unsigned channel);
void MailboxWrite(unsigned channel, unsigned nData);
unsigned MailboxWriteRead(unsigned channel, unsigned nData);

/* Public functions ----------------------------------------------------------------*/
bool MailboxInterface_getTag(uint32_t nTagId, void *pTag, unsigned nTagSize, unsigned nRequestParmSize)
{
	unsigned nBufferSize = sizeof (MailboxInterface_PropertyBuffer) + nTagSize + sizeof (uint32_t);

	MailboxInterface_PropertyBuffer *pBuffer = (MailboxInterface_PropertyBuffer *)dma_alloc(DMA_PAGE_SIZE, DMA_ALIGNEMENT);
	pBuffer->nBufferSize	= nBufferSize;
	pBuffer->nCode 			= CODE_REQUEST;

	memcpy (pBuffer->Tags, pTag, nTagSize);

	MailboxInterface_PropertyTag *pHeader = (MailboxInterface_PropertyTag *) pBuffer->Tags;

	pHeader->nTagId 		= nTagId;
	pHeader->nValueBufSize 	= nTagSize - sizeof (MailboxInterface_PropertyTag);
	pHeader->nValueLength 	= nRequestParmSize & ~VALUE_LENGTH_RESPONSE;

	uint32_t *pEndTag 	= (uint32_t *) (pBuffer->Tags + nTagSize);
	*pEndTag 			= PROPTAG_END;
	uintptr_t physAddr 		= dma_getPhysicalAddr(pBuffer);
	
	uint32_t nBufferAddress = BUS_ADDRESS ((uint32_t) physAddr);

	if (MailboxWriteRead (MAILBOX_CHANNEL, nBufferAddress) != nBufferAddress)
	{
        dma_free(pBuffer, DMA_ALIGNEMENT);
		return false;
	}

	DataMemBarrier ();

	if (pBuffer->nCode != CODE_RESPONSE_SUCCESS)
	{
        dma_free(pBuffer, DMA_ALIGNEMENT);
		return false;
	}

	if (!(pHeader->nValueLength & VALUE_LENGTH_RESPONSE))
	{
        dma_free(pBuffer, DMA_ALIGNEMENT);
		return false;
	}

	pHeader->nValueLength &= ~VALUE_LENGTH_RESPONSE;
	if (pHeader->nValueLength == 0)
	{
        dma_free(pBuffer, DMA_ALIGNEMENT);
		return false;
	}

	memcpy (pTag, pBuffer->Tags, nTagSize);

	dma_free(pBuffer, DMA_ALIGNEMENT);

	return true;
}

/* Private functions ----------------------------------------------------------------*/
unsigned MailboxRead(unsigned channel)
{
	unsigned nResult;
	do
	{
		while (read32 (MAILBOX0_STATUS) & MAILBOX_STATUS_EMPTY)
		{
			// do nothing
		}
		nResult = read32 (MAILBOX0_READ);
	}
	while ((nResult & 0xF) != MAILBOX_CHANNEL);		// channel number is in the lower 4 bits
	return nResult & ~0xF;
}

void MailboxWrite(unsigned channel, unsigned nData)
{
	while (read32 (MAILBOX1_STATUS) & MAILBOX_STATUS_FULL)
	{
		// do nothing
	}
	write32 (MAILBOX1_WRITE, channel | nData);	// channel number is in the lower 4 bits
}

unsigned MailboxWriteRead(unsigned channel, unsigned nData)
{
	DataMemBarrier();

	MailboxFlush();

	MailboxWrite(channel, nData);

	unsigned nResult = MailboxRead(channel);

	DataMemBarrier();

	return nResult;
}

void MailboxFlush()
{
	while (!(read32 (MAILBOX0_STATUS) & MAILBOX_STATUS_EMPTY))
	{
		read32 (MAILBOX0_READ);

		MsDelay(20);
	}
}

static uint64_t read32(uint64_t nAddress)
{
	return *(volatile uint64_t *) nAddress;
}

static void write32(uint64_t nAddress, uint64_t nValue)
{
	*(volatile uint64_t *) nAddress = nValue;
}