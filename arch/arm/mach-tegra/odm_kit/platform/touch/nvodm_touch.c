/*
 * Copyright (c) 2006-2009 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "nvodm_touch.h"
#include "nvodm_touch_int.h"

#if defined(NV_TOUCH_TPK)
#include "nvodm_touch_tpk.h"
#endif

#if defined(NV_TOUCH_AK4183)
#include "ak4183/nvodm_touch_ak4183.h"
#endif

#if defined(NV_TOUCH_AT168)
#include "at168/nvodm_touch_at168.h"
#endif

static NvU32 driverindex=0;

NvU32 NvOdmTouchDriverIndex(NvU32* pindex)
{
    if(pindex) driverindex=*pindex;
	return driverindex;
}
/** Implementation for the NvOdm TouchPad */

NvBool
NvOdmTouchDeviceOpen( NvOdmTouchDeviceHandle *hDevice )
{
    NvBool ret = NV_FALSE;
	//return NV_FALSE;

#if defined(NV_TOUCH_TPK)
    if(ret==NV_FALSE&&driverindex==1) 
	{ 
		ret = TPK_Open(hDevice);
	}
   
#endif

#if defined(NV_TOUCH_AT168)
   if(ret==NV_FALSE&&driverindex==2) 
   	{ 
		ret = AT168_Open(hDevice);
	}
#endif

#if defined(NV_TOUCH_AK4183)
    if(ret==NV_FALSE&&driverindex==4)
	{ 
		ret = AK4183_Open(hDevice);
	}
    
#endif



    return ret;
}


void
NvOdmTouchDeviceGetCapabilities(NvOdmTouchDeviceHandle hDevice, NvOdmTouchCapabilities* pCapabilities)
{
    hDevice->GetCapabilities(hDevice, pCapabilities);
}


NvBool
NvOdmTouchReadCoordinate( NvOdmTouchDeviceHandle hDevice, NvOdmTouchCoordinateInfo *coord)
{
    return hDevice->ReadCoordinate(hDevice, coord);
}

NvBool
NvOdmTouchReadInitData( NvOdmTouchDeviceHandle hDevice, NvOdmTouchInitDataInfo *InitData)
{
	return hDevice->ReadInitData(hDevice, InitData);
}

NvBool
NvOdmTouchGetSampleRate(NvOdmTouchDeviceHandle hDevice, NvOdmTouchSampleRate* pTouchSampleRate)
{
    return hDevice->GetSampleRate(hDevice, pTouchSampleRate);
}

void NvOdmTouchDeviceClose(NvOdmTouchDeviceHandle hDevice)
{
    hDevice->Close(hDevice);    
}

NvBool NvOdmTouchEnableInterrupt(NvOdmTouchDeviceHandle hDevice, NvOdmOsSemaphoreHandle hInterruptSemaphore)
{
    return hDevice->EnableInterrupt(hDevice, hInterruptSemaphore);
}

NvBool NvOdmTouchHandleInterrupt(NvOdmTouchDeviceHandle hDevice)
{
    return hDevice->HandleInterrupt(hDevice);
}

NvBool
NvOdmTouchSetSampleRate(NvOdmTouchDeviceHandle hDevice, NvU32 SampleRate)
{
    return hDevice->SetSampleRate(hDevice, SampleRate);
}


NvBool
NvOdmTouchPowerControl(NvOdmTouchDeviceHandle hDevice, NvOdmTouchPowerModeType mode)
{
    return hDevice->PowerControl(hDevice, mode);
}

void
NvOdmTouchPowerOnOff(NvOdmTouchDeviceHandle hDevice, NvBool OnOff)
{
    hDevice->PowerOnOff(hDevice, OnOff);
}


NvBool
NvOdmTouchOutputDebugMessage(NvOdmTouchDeviceHandle hDevice)
{
    return hDevice->OutputDebugMessage;
}

NvBool
NvOdmTouchGetCalibrationData(NvOdmTouchDeviceHandle hDevice, NvU32 NumOfCalibrationData, NvS32* pRawCoordBuffer)
{
    return hDevice->GetCalibrationData(hDevice, NumOfCalibrationData, pRawCoordBuffer);
}

void
NvOdmTouchSetCalibration(NvOdmTouchDeviceHandle hDevice)
{
    hDevice->SetCalibration(hDevice);
}

NvBool
NvOdmTouchBurnBootloader(NvOdmTouchDeviceHandle hDevice)
{
    return hDevice->BurnBootloader(hDevice);
}

void
NvOdmTouchSetBaseline(NvOdmTouchDeviceHandle hDevice)
{
    hDevice->SetBaseline(hDevice);
}

void
NvOdmTouchSetCalibrateResult(NvOdmTouchDeviceHandle hDevice)
{
    hDevice->SetCalibrateResult(hDevice);
}
