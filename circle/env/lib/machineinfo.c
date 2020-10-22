//
// machineinfo.cpp
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
#include <circleenv/machineinfo.h>
//#include <circle/gpioclock.h>
#include <circleenv/sysconfig.h>
#include <circleenv/assert.h>

unsigned GetClockRate (u32 nClockId) 
{
	TPropertyTag Tags;
	TPropertyTagClockRate TagClockRate;
	TagClockRate.nClockId = nClockId;
	if (BcmPropertyTagsGetTag (&Tags,PROPTAG_GET_CLOCK_RATE, &TagClockRate, sizeof(TagClockRate), 4))
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
		nResult = 250000000;
		// if (m_nModelMajor < 3)
		// {
		// 	nResult = 250000000;
		// }
		// else
		// {
		// 	nResult = 300000000;		// TODO
		// }
		break;

	default:
		assert (0);
		break;
	}

	return nResult;
}