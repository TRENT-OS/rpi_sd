//
// uspios.h
//
// External functions used by the USPi library
//
// USPi - An USB driver for Raspberry Pi written in C
// Copyright (C) 2014-2018  R. Stange <rsta2@o2online.de>
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#ifndef _uspios_h
#define _uspios_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//
// System configuration
//
// (Change this before you build the USPi library!)
//
#define GPU_L2_CACHE_ENABLED		// normally enabled (can be disabled in config.txt)

#define HZ	100			// timer ticks / second (set this to your timer interrupt frequency)

// Default keyboard map (enable only one)
//#define USPI_DEFAULT_KEYMAP_DE
//#define USPI_DEFAULT_KEYMAP_ES
//#define USPI_DEFAULT_KEYMAP_FR
//#define USPI_DEFAULT_KEYMAP_IT
#define USPI_DEFAULT_KEYMAP_UK
//#define USPI_DEFAULT_KEYMAP_US

// Undefine this if you want to use your own implementation of the functions in uspi/util.h
#define USPI_PROVIDE_MEM_FUNCTIONS	// mem*()
#define USPI_PROVIDE_STR_FUNCTIONS	// str*()

//
// Memory allocation
//
// (Must work from interrupt context)
//
#define DMA_PAGE_SIZE	4096
#define DMA_ALIGNEMENT 	4096

// void *malloc (unsigned nSize);		// result must be 4-byte aligned
// void free (void *pBlock);

void *dma_alloc (unsigned nSize, unsigned alignement);
void dma_free (void *pBlock, unsigned alignement);
uintptr_t dma_getPhysicalAddr(void *ptr);

//
// Timer
//
void MsDelay (unsigned nMilliSeconds);	
void usDelay (unsigned nMicroSeconds);
void nsDelay (unsigned nNanoSeconds);

#ifndef AARCH64
	typedef unsigned TKernelTimerHandle;
#else
	typedef unsigned long TKernelTimerHandle;
#endif

typedef void TKernelTimerHandler (TKernelTimerHandle hTimer, void *pParam, void *pContext);

// returns the timer handle (hTimer)
unsigned StartKernelTimer (unsigned	        nHzDelay,	// in HZ units (see "system configuration" above)
			   TKernelTimerHandler *pHandler,
			   void *pParam, void *pContext);	// handed over to the timer handler

void CancelKernelTimer (unsigned hTimer);

//
// Interrupt handling
//
typedef void TInterruptHandler (void *pParam);

// USPi uses USB IRQ 9
void ConnectInterrupt (unsigned nIRQ, TInterruptHandler *pHandler, void *pParam);
void DisconnectInterrupt (unsigned nIRQ);
//
// Property tags (ARM -> VC)
//
// See: https://github.com/raspberrypi/firmware/wiki/Mailboxes
//

// returns 0 on failure
int SetPowerStateOn (unsigned nDeviceId);	// "set power state" to "on", wait until completed

unsigned GetClockRate (uint32_t nClockId);

// returns 0 on failure
int GetBaseClock (void);

int SetSDHostClock (uint32_t *msg, size_t length);

int GetMachineModel (void);

unsigned GetClockTicks (void);

int SetGPIOState (uint32_t state);
//
// Logging
//

// Severity (change this before building if you want different values)
// Renamed so it does not clash with other (system) error codes
#define USPI_LOG_ERROR		1
#define USPI_LOG_WARNING	2
#define USPI_LOG_NOTICE		3
#define USPI_LOG_DEBUG		4

void LogWrite (const char *pSource,		// short name of module
	       unsigned	   Severity,		// see above
	       const char *pMessage, ...);	// uses printf format options

//
// Debug support
//
#ifndef NDEBUG

// display "assertion failed" message and halt
void uspi_assertion_failed (const char *pExpr, const char *pFile, unsigned nLine);

// display hex dump (pSource can be 0)
void DebugHexdump (const void *pBuffer, unsigned nBufLen, const char *pSource /* = 0 */);

#endif

#ifdef __cplusplus
}
#endif

#endif
