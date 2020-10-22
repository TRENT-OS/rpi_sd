//
// emmc.h
//
// Provides an interface to the EMMC controller and commands for interacting
// with an sd card
//
// Copyright (C) 2013 by John Cronin <jncronin@tysos.org>
//
// Modified for Circle by R. Stange
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
#ifndef _SDCard_emmc_h
#define _SDCard_emmc_h

// #include <circleenv/device.h>
// #include <circleenv/interrupt.h>
// #include <circleenv/timer.h>
// #include <circle/actled.h>
//#include <circle/gpiopin.h>
// #include <circle/fs/partitionmanager.h>
// #include <circleenv/logger.h>
#include <circleenv/types.h>
#include <circleenv/sysconfig.h>
#ifdef USE_SDHOST
	#include "./bcm2837_sdhost.h"
#endif

typedef struct TSCR			// SD configuration register
{
	u32	scr[2];
	u32	sd_bus_widths;
	int	sd_version;
} TSCR;

typedef struct TEMMCDevice
{
	// CInterruptSystem *m_pInterruptSystem;
	// CTimer		 *m_pTimer;
	// CActLED		 *m_pActLED;

// #ifndef USE_SDHOST
// #if RASPPI >= 2
// 	CGPIOPin	  m_GPIO34_39[6];	// WiFi
// 	CGPIOPin	  m_GPIO48_53[6];	// SD card
// #endif
// #endif

	u64 m_ullOffset;

	// CPartitionManager *m_pPartitionManager;

#ifdef USE_SDHOST
	// CSDHOSTDevice m_Host;
#else
	u32 m_hci_ver;
#endif

	// was: struct emmc_block_dev
	u32 m_device_id[4];

	u32 m_card_supports_sdhc;
	u32 m_card_supports_18v;
	u32 m_card_ocr;
	u32 m_card_rca;
#ifndef USE_SDHOST
	u32 m_last_interrupt;
#endif
	u32 m_last_error;

	TSCR *m_pSCR;

	int m_failed_voltage_switch;

	u32 m_last_cmd_reg;
	u32 m_last_cmd;
	u32 m_last_cmd_success;
	u32 m_last_r0;
	u32 m_last_r1;
	u32 m_last_r2;
	u32 m_last_r3;

	void *m_buf;
	int m_blocks_to_transfer;
	size_t m_block_size;
#ifndef USE_SDHOST
	int m_card_removal;
	u32 m_base_clock;
#endif
} TEMMCDevice;

//public:
int EMMCDevice (void *vaddr);
void _EMMCDevice (void);

boolean emmc_initialize (void);

int emmc_read (void *pBuffer, size_t nCount);
int emmc_write (const void *pBuffer, size_t nCount);

u64 emmc_seek (u64 ullOffset);

const u32 *GetID (void);

//private:
#ifndef USE_SDHOST
// int PowerOn (void);
void PowerOff (void);

// u32 GetBaseClock (void);
u32 GetClockDivider (u32 base_clock, u32 target_rate);
int SwitchClockRate (u32 base_clock, u32 target_rate);

int ResetCmd (void);
int ResetDat (void);
#endif

void IssueCommandInt (u32 cmd_reg, u32 argument, int timeout);
#ifndef USE_SDHOST
void HandleCardInterrupt (void);
void HandleInterrupts (void);
#endif
boolean IssueCommand (u32 command, u32 argument, int timeout);

int CardReset (void);
int CardInit (void);

int EnsureDataMode (void);
int DoDataCommand (int is_write, u8 *buf, size_t buf_size, u32 block_no);
int DoRead (u8 *buf, size_t buf_size, u32 block_no);
int DoWrite (u8 *buf, size_t buf_size, u32 block_no);

off_t emmc_capacity(void);

#ifndef USE_SDHOST
int TimeoutWait (unsigned reg, unsigned mask, int value, unsigned usec);
#endif

int emmc_block_size();

// void usDelay (unsigned usec);

// static void LogWrite (TLogSeverity Severity, const char *pMessage, ...);
	// CInterruptSystem *m_pInterruptSystem;
	// CTimer		 *m_pTimer;
	// CActLED		 *m_pActLED;

// #ifndef USE_SDHOST
// #if RASPPI >= 2
// 	CGPIOPin	  m_GPIO34_39[6];	// WiFi
// 	CGPIOPin	  m_GPIO48_53[6];	// SD card
// #endif
// #endif

	u64 m_ullOffset;

	// CPartitionManager *m_pPartitionManager;

#ifdef USE_SDHOST
	// CSDHOSTDevice m_Host;
#else
	u32 m_hci_ver;
#endif

	// was: struct emmc_block_dev
	u32 m_device_id[4];

	u32 m_card_supports_sdhc;
	u32 m_card_supports_18v;
	u32 m_card_ocr;
	u32 m_card_rca;
#ifndef USE_SDHOST
	u32 m_last_interrupt;
#endif
	u32 m_last_error;

	TSCR *m_pSCR;

	int m_failed_voltage_switch;

	u32 m_last_cmd_reg;
	u32 m_last_cmd;
	u32 m_last_cmd_success;
	u32 m_last_r0;
	u32 m_last_r1;
	u32 m_last_r2;
	u32 m_last_r3;

	void *m_buf;
	int m_blocks_to_transfer;
	size_t m_block_size;
#ifndef USE_SDHOST
	int m_card_removal;
	u32 m_base_clock;
#endif

#endif
