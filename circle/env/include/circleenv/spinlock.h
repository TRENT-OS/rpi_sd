//
// spinlock.h
//
// Circle - A C++ bare metal environment for Raspberry Pi
// Copyright (C) 2015-2018  R. Stange <rsta2@o2online.de>
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
#ifndef _circle_spinlock_h
#define _circle_spinlock_h

#include "./sysconfig.h"
#include "./synchronize.h"
#include "./types.h"

#ifdef ARM_ALLOW_MULTI_CORE

class CSpinLock
{
public:
	// nTargetLevel is the maximum execution level from which the spin lock is used.
	// This has been a boolean parameter before and valid values were TRUE and FALSE.
	// These parameters are still working, but are deprecated. Use the *_LEVEL defines
	// from circle/sysconfig.h instead!
	CSpinLock (unsigned nTargetLevel = IRQ_LEVEL);
	~CSpinLock (void);

	void Acquire (void);
	void Release (void);

	static void Enable (void);

private:
	unsigned m_nTargetLevel;

	u32 m_nLocked;

	static boolean s_bEnabled;
};

#else

typedef struct TSpinLock
{
	unsigned m_nTargetLevel;
} TSpinLock;

void SpinLock (
	TSpinLock *pThis,
	unsigned nTargetLevel);

void _SpinLock (
	TSpinLock *pThis);

void SpinLockAcquire (TSpinLock *pThis);

void SpinLockRelease (TSpinLock *pThis);

#endif

#endif
