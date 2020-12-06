/**
 * Copyright (C) 2019, Hensoldt Cyber GmbH
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include <circleos.h>

#include "lib_debug/Debug.h"

#include "environment.h"
#include "mailboxInterface.h"
#include <circleenv/bcm2835.h>
#include <circleenv/bcm2711.h>
#include <circleenv/synchronize.h>
#include <circleenv/memio.h>

#include "TimeServer.h"

#include <camkes.h>
#include <camkes/dma.h>

static const if_OS_Timer_t timer =
    IF_OS_TIMER_ASSIGN(
        timeServer_rpc,
        timeServer_notify);

/* Environment functions -------------------------------------------------------------*/
void* dma_alloc (unsigned nSize, unsigned alignement)
{
    // we are setting cached to false to allocate non-cached DMA memory for the
    // NIC driver
    return camkes_dma_alloc(nSize, alignement, false);
}

void dma_free (void* pBlock, unsigned alignement)
{
    camkes_dma_free(pBlock, alignement);
}

uintptr_t dma_getPhysicalAddr(void* ptr)
{
    return camkes_dma_get_paddr(ptr);
}

void LogWrite (const char* pSource, unsigned Severity, const char* pMessage,
               ...)
{
    va_list args;
    va_start (args, pMessage);

    switch (Severity)
    {
    case USPI_LOG_ERROR:
        printf("\n ERROR %s: - ", pSource);
        break;
    case USPI_LOG_WARNING:
        printf("\n WARNING %s: - ", pSource);
        break;
    case USPI_LOG_NOTICE:
        printf("\n NOTICE %s: - ", pSource);
        break;
    case USPI_LOG_DEBUG:
        printf("\n DEBUG %s: - ", pSource);
        break;
    }

    vprintf(pMessage, args);
	printf("\n");
    va_end(args);
}

void MsDelay (unsigned nMilliSeconds)
{
    usDelay(1000 * nMilliSeconds);
}

void usDelay (unsigned nMicroSeconds)
{
	nsDelay(1000 * nMicroSeconds);
}

void nsDelay (unsigned nNanoSeconds)
{
    OS_Error_t err;

    if ((err = TimeServer_sleep(&timer, TimeServer_PRECISION_NSEC,
                                nNanoSeconds)) != OS_SUCCESS)
    {
        Debug_LOG_ERROR("TimeServer_sleep() failed with %d", err);
    }
}

unsigned StartKernelTimer (unsigned nHzDelay, TKernelTimerHandler* pHandler,
                           void* pParam, void* pContext)
{
    Debug_LOG_WARNING("Not implemented!");
    return 0;
}

void CancelKernelTimer (unsigned hTimer)
{
    Debug_LOG_WARNING("Not implemented!");
}

/*
	Not necessary => used for configuring the usb interrupt
	which is in our environment done by camkes
*/
void ConnectInterrupt (unsigned nIRQ, TInterruptHandler* pHandler, void* pParam)
{
    Debug_LOG_WARNING("Not implemented!");
}

void DisconnectInterrupt (unsigned nIRQ)
{
    Debug_LOG_WARNING("Not implemented!");
}

int SetPowerStateOn (unsigned nDeviceId)
{
    PropertyTagPowerState PowerState;
    PowerState.nDeviceId = nDeviceId;
    PowerState.nState = POWER_STATE_ON | POWER_STATE_WAIT;

    if (!MailboxInterface_getTag (PROPTAG_SET_POWER_STATE, &PowerState,
                                  sizeof PowerState, 8)
        || (PowerState.nState & POWER_STATE_NO_DEVICE)
        || !(PowerState.nState & POWER_STATE_ON))
    {
        Debug_LOG_ERROR("Failed to set power state on!");
        return 0;
    }

    return 1;
}

// Get the current base clock rate in Hz
int GetBaseClock (void)
{
	PropertyTagClockRate TagClockRate;
#if RASPPI <= 3
	TagClockRate.nClockId = CLOCK_ID_EMMC;
#else
	TagClockRate.nClockId = CLOCK_ID_EMMC2;
#endif
	if (!MailboxInterface_getTag (PROPTAG_GET_CLOCK_RATE, &TagClockRate, sizeof TagClockRate, 4))
	{
		Debug_LOG_ERROR ("Cannot get clock rate");
		TagClockRate.nRate = 0;
	}

	return TagClockRate.nRate;
}

unsigned GetClockRate (uint32_t nClockId) 
{
    PropertyTagClockRate TagClockRate;
	TagClockRate.nClockId = nClockId;
	if (MailboxInterface_getTag (PROPTAG_GET_CLOCK_RATE, &TagClockRate, sizeof TagClockRate, 4))
	{
		return TagClockRate.nRate;
	}

	// if clock rate can not be requested, use a default rate
	unsigned nResult = 0;

	switch (nClockId)
	{
	case CLOCK_ID_EMMC:
		nResult = 100000000;
		break;

	case CLOCK_ID_UART:
		nResult = 48000000;
		break;

	case CLOCK_ID_CORE:
		if (RASPPI < 3)
		{
			nResult = 250000000;
		}
		else
		{
			nResult = 300000000;		// TODO
		}
		break;

	default:
		assert (0);
		break;
	}

	return nResult;
}

int SetSDHostClock (uint32_t *msg, size_t length)
{
	struct
	{
        MailboxInterface_PropertyTag	Tag;
		uint32_t		msg[length];
	}
	PACKED SetSDHOSTClock;

	// memcpy (SetSDHOSTClock.msg, msg, sizeof *msg);
	memcpy (SetSDHOSTClock.msg, msg, length * 4);

#define PROPTAG_SET_SDHOST_CLOCK 0x00038042
	if (!MailboxInterface_getTag (PROPTAG_SET_SDHOST_CLOCK, &SetSDHOSTClock, sizeof SetSDHOSTClock, 3*4))
	{
		Debug_LOG_ERROR("MailboxInterface_getTag() failed.");
		return 0;
	}

	// memcpy (msg, SetSDHOSTClock.msg, sizeof *msg);
	memcpy (msg, SetSDHOSTClock.msg, length * 4);

    return 1;
}

int SetGPIOState (uint32_t state)
{
	PropertyTagGPIOState GPIOState;
	GPIOState.nGPIO = EXP_GPIO_BASE * 4;
	// GPIOState.nGPIO = EXP_GPIO_NUM;
	GPIOState.nState = state;
#define PROPTAG_SET_SET_GPIO_STATE	0x00038041
	if (!MailboxInterface_getTag(PROPTAG_SET_SET_GPIO_STATE, &GPIOState, sizeof GPIOState, 8))
	{
		Debug_LOG_ERROR("MailboxInterface_getTag() failed.");
		return 0;
	}
    return 1;
}

int GetMachineModel (void) 
{
    // PropertyTagBoardModel TagBoardModel;
	// if (!MailboxInterface_getTag (PROPTAG_GET_BOARD_MODEL, &TagBoardModel, sizeof TagBoardModel, 0))
	// {
	// 	Debug_LOG_ERROR ("Cannot get board model.");
    //     TagBoardModel.nBoardModel = MachineModel3BPlus;
	// }

    // return TagBoardModel.nBoardModel;
	return MachineModel3BPlus;
}

unsigned GetClockTicks (void)
{
	DataMemBarrier ();

	unsigned nResult = read32 (ARM_SYSTIMER_CLO);

	DataMemBarrier ();

	return nResult;
}
