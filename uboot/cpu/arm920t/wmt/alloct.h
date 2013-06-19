/*++ 
Copyright (c) 2010 WonderMedia Technologies, Inc.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details. You
should have received a copy of the GNU General Public License along with this
program. If not, see http://www.gnu.org/licenses/>.

WonderMedia Technologies, Inc.
10F, 529, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C.
--*/


#ifndef __ALLOCT_H__
#define __ALLOCT_H__

#if !defined(__ADAPTER_H__)
#include "adapter.h"
#include "mem.h"
#endif


/*---------------------  Export Definitions -------------------------*/

/*---------------------  Export Types  ------------------------------*/

/*---------------------  Export Macros ------------------------------*/

/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/
#ifdef __cplusplus
extern "C" {                            /* Assume C declarations for C++ */
#endif /* __cplusplus */

BOOL
ALCbAllocateRdrMemory(
    PSAdapterInfo pAdapter
    );

BOOL
ALCbAllocateTdrMemory(
    PSAdapterInfo pAdapter
    );

BOOL
ALCbAllocateTdrBuffer(
    PSAdapterInfo pAdapter,
    SAllocMap asMaps[],
    UINT    cbTD
    );

VOID
ALCvChainRdrMemory(
    PSAdapterInfo pAdapter
    );

VOID
ALCvChainTdrMemory(
    PSAdapterInfo pAdapter
    );

VOID
ALCvFreeRdrMemory(
    PSAdapterInfo pAdapter
    );

VOID
ALCvFreeTdrMemory(
    PSAdapterInfo pAdapter
    );

VOID
ALCvSetDefaultRD(
    PSAdapterInfo pAdapter
    );

VOID
ALCvSetDefaultTD(
    PSAdapterInfo pAdapter
    );

#ifdef __cplusplus
}                                       /* End of extern "C" { */
#endif /* __cplusplus */

#endif // __ALLOCT_H__
