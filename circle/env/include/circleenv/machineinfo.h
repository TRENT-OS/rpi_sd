//
// machineinfo.h
//
// Circle - A C++ bare metal environment for Raspberry Pi
// Copyright (C) 2016-2020  R. Stange <rsta2@o2online.de>
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
#ifndef _circle_machineinfo_h
#define _circle_machineinfo_h

// #include "./bcmpropertytags.h"
//#include <circle/gpiopin.h>
#include "./macros.h"
#include "./types.h"

typedef enum TMachineModel
{
	MachineModelA,
	MachineModelBRelease1MB256,
	MachineModelBRelease2MB256,
	MachineModelBRelease2MB512,
	MachineModelAPlus,
	MachineModelBPlus,
	MachineModelZero,
	MachineModelZeroW,
	MachineModel2B,
	MachineModel3B,
	MachineModel3APlus,
	MachineModel3BPlus,
	MachineModelCM,
	MachineModelCM3,
	MachineModelCM3Plus,
	MachineModel4B,
	MachineModelUnknown
} TMachineModel_e;

typedef enum TSoCType
{
	SoCTypeBCM2835,
	SoCTypeBCM2836,
	SoCTypeBCM2837,
	SoCTypeBCM2711,
	SoCTypeUnknown
} TSoCType_e;

typedef enum TDeviceId
{
	DeviceI2CMaster,
	DeviceUnkown
} TDeviceId_e;

#define ACTLED_PIN_MASK		0x3F
#define ACTLED_ACTIVE_LOW	0x40
#define ACTLED_VIRTUAL_PIN	0x80
#define ACTLED_UNKNOWN		(ACTLED_VIRTUAL_PIN | 0)
// unsigned GetClockRate (u32 nClockId);	// see circle/bcmpropertytags.h for nClockId
#define GPIO_CLOCK_SOURCE_ID_MAX	15		// source ID is 0-15
#define GPIO_CLOCK_SOURCE_UNUSED	0		// returned for unused clock sources

	// DMA channel resource management
#define DMA_CHANNEL_MAX		12			// channels 0-12 are supported
#define DMA_CHANNEL__MASK	0x0F			// explicit channel number
#define DMA_CHANNEL_NONE	0x80			// returned if no channel available
#define DMA_CHANNEL_NORMAL	0x81			// normal DMA engine requested
#define DMA_CHANNEL_LITE	0x82			// lite (or normal) DMA engine requested

#endif
