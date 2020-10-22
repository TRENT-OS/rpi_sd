//
// mmchost.h
//
// Circle - A C++ bare metal environment for Raspberry Pi
// Copyright (C) 2020  R. Stange <rsta2@o2online.de>
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
#ifndef _SDCard_mmchost_h
#define _SDCard_mmchost_h

#include <circleenv/types.h>
#include <circle/mmc.h>
// #include <circle/bcm2837_sdhost.h>

typedef struct TMMCHost
{
	mmc_host_t *m_Data;
} TMMCHost;

//public:
void MMCHost (void);
void _MMCHost (void);

void mmc_host_SetClock (unsigned nClockHz);
void mmc_host_SetBusWidth (unsigned nBits);	// 1 or 4 bits

int mmc_host_Command (mmc_command_t *pCommand, unsigned nRetries);

mmc_host_t *GetMMCHost (void);

//protected:

void sg_miter_start (
	sg_mapping_iter_t *sg_miter, 
	void *sg, unsigned sg_len,
	unsigned flags);

bool sg_miter_next (sg_mapping_iter_t *sg_miter);
void sg_miter_stop (sg_mapping_iter_t *sg_miter);

//private:
void RequestSync (mmc_request_t *pRequest);

int PrepareRequest (mmc_request_t *pRequest);

mmc_host_t m_Data;
#endif
