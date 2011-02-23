/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
 * All rights reserved. 
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the NVIDIA Corporation nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "nvodm_services.h"
#include "nvodm_touch_at168.h"
#include "nvodm_query_discovery.h"
#include "nvodm_touch_at168_int.h"


#include "nvos.h"
#include "nvodm_query.h"

#include "../nvodm_touch_int.h"
#include "nvodm_touch.h"

#include "nvodm_query_gpio.h"

#include <linux/delay.h>

#include <linux/fs.h> 
#include <asm/uaccess.h> 
#include <linux/mm.h> 
#define AT168_I2C_SPEED_KHZ                          200//400
#define AT168_I2C_TIMEOUT                            2000//500
#define AT168_DEBOUNCE_TIME_MS 		0
#define AT168_TOUCH_DEVICE_GUID 			NV_ODM_GUID('a','t','e','1','6','8','t','s')

#define AT168_WRITE(dev, reg, byte) AT168_WriteRegister(dev, reg, byte)
#define AT168_READ(dev, reg, buffer, len) AT168_ReadRegisterSafe(dev, reg, buffer, len)

typedef struct AT168_TouchDeviceRec
{
	NvOdmTouchDevice OdmTouch;
	NvOdmTouchCapabilities Caps;
	NvOdmServicesI2cHandle hOdmI2c;
	NvOdmServicesGpioHandle hGpio;
	NvOdmServicesPmuHandle hPmu;
	NvOdmGpioPinHandle hPinReset;
	NvOdmGpioPinHandle hPinInterrupt;
	NvOdmServicesGpioIntrHandle hGpioIntr;
	NvOdmOsSemaphoreHandle hIntSema;
	NvBool PrevFingers;
	NvU32 DeviceAddr;
	NvU32 SampleRate;
	NvU32 SleepMode;
	NvBool PowerOn;
	NvU32 VddId;    
	NvU32 ChipRevisionId; //Id=0x01:AT168 chip on Concorde1
	                  //id=0x02:AT168 chip with updated firmware on Concorde2
	NvU32 I2cClockSpeedKHz;
} AT168_TouchDevice;

#if 0
//MAX and MIN of coord (x y)
#define AT168_MAX_X		(1024) //(4992)//(4096)//(1024)
#define AT168_MAX_Y		(600) //(2816)//(4096)//(600)
#define AT168_MIN_X		(0)
#define AT168_MIN_Y		(0)
#endif

static NvOdmTouchCapabilities AT168_Capabilities =
{
	.IsMultiTouchSupported = NV_TRUE,
	.MaxNumberOfFingerCoordReported = 2,
	.IsRelativeDataSupported = NV_FALSE,
	.MaxNumberOfRelativeCoordReported = 1,
	.MaxNumberOfWidthReported =1,
	.MaxNumberOfPressureReported = 1,
	.Gesture = NvOdmTouchGesture_Not_Supported,
	.IsWidthSupported = NV_TRUE,
	.IsPressureSupported = NV_TRUE,
	.IsFingersSupported = NV_TRUE,
	.XMinPosition = 0,//AT168_MIN_X,
	.YMinPosition = 0,//AT168_MIN_Y,
	.XMaxPosition = 1024,//AT168_MAX_X,
	.YMaxPosition = 600,//AT168_MAX_Y,
	.Orientation = 0, //0,//NvOdmTouchOrientation_V_FLIP,//NvOdmTouchOrientation_H_FLIP
	.Version = 0, 
};
static NvBool AT168_Bootloader_Read (AT168_TouchDevice *hTouch, NvU8* buffer, NvU32 NumBytes)
{
	NvOdmI2cStatus Error;
	NvOdmI2cTransactionInfo TransactionInfo;
	NvU32 TempI2cClockSpeedKHz;

	//hTouch->DeviceAddr = (NvU32)((0x5d << 1) | 0x01);	//change I2C  to bootloader address 0x5d
	TempI2cClockSpeedKHz = hTouch->I2cClockSpeedKHz;
	hTouch->I2cClockSpeedKHz = 100;		//change I2C  freq to 100KHz

	TransactionInfo.Address = hTouch->DeviceAddr;
	TransactionInfo.Buf = buffer;
	TransactionInfo.Flags = 0;
	TransactionInfo.NumBytes = NumBytes;
    
	do
	{
		Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
		                            &TransactionInfo,
		                            1,
		                            hTouch->I2cClockSpeedKHz,
		                            AT168_I2C_TIMEOUT);
	} while (Error == NvOdmI2cStatus_Timeout);

	if (Error != NvOdmI2cStatus_Success)
	{
		NvOsDebugPrintf("I2C Read Failure = %d  addr=0x%x \n", Error,
		                   hTouch->DeviceAddr);
		
		hTouch->I2cClockSpeedKHz = TempI2cClockSpeedKHz;		//change I2C  freq back to Normal
		//hTouch->DeviceAddr = (NvU32)(0x5c << 1);	//change I2C back to normal 0x5c
		return NV_FALSE;
	}
	hTouch->I2cClockSpeedKHz = TempI2cClockSpeedKHz;		//change I2C  freq back to Normal
	//hTouch->DeviceAddr = (NvU32)(0x5c << 1);	//change I2C back to normal 0x5c
	return NV_TRUE;
}
static NvBool AT168_Bootloader_Write(AT168_TouchDevice* hTouch, NvU8 *buffer, NvU8 NumBytes)
{
	NvOdmI2cTransactionInfo TransactionInfo;
	NvOdmI2cStatus err;
	NvU32 TempI2cClockSpeedKHz;

	//hTouch->DeviceAddr = (NvU32)(0x5d << 1);	//change I2C  to bootloader address 0x5d
	TempI2cClockSpeedKHz = hTouch->I2cClockSpeedKHz;
	hTouch->I2cClockSpeedKHz = 100;		//change I2C  freq to 100KHz

	TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
	TransactionInfo.Address = hTouch->DeviceAddr;
	TransactionInfo.NumBytes = NumBytes;
	TransactionInfo.Buf = buffer;
	err = NvOdmI2cTransaction(hTouch->hOdmI2c,
								&TransactionInfo,
								1,
								hTouch->I2cClockSpeedKHz,
								AT168_I2C_TIMEOUT);
	if (err != NvOdmI2cStatus_Success) {
		NvOsDebugPrintf("NvOdmTouch: AT168_Bootloader_Write i2c transaction failture = %d, address=0x%x\n", err, hTouch->DeviceAddr);
		hTouch->I2cClockSpeedKHz = TempI2cClockSpeedKHz;		//change I2C  freq back to Normal
		//hTouch->DeviceAddr = (NvU32)(0x5c << 1);	//change I2C back to normal 0x5c
		return NV_FALSE;
	}
	hTouch->I2cClockSpeedKHz = TempI2cClockSpeedKHz;		//change I2C  freq back to Normal
	//hTouch->DeviceAddr = (NvU32)(0x5c << 1);	//change I2C back to normal 0x5c
	return NV_TRUE;
}

static NvBool AT168_WriteRegister (AT168_TouchDevice* hTouch, NvU8 reg, NvU8 val)
{
	NvOdmI2cStatus Error;
	NvOdmI2cTransactionInfo TransactionInfo;
	NvU8 arr[2];

	arr[0] = reg;
	arr[1] = val;

	TransactionInfo.Address = hTouch->DeviceAddr;
	TransactionInfo.Buf = arr;
	TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
	TransactionInfo.NumBytes = 2;

	do
	{
		Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
		                            &TransactionInfo,
		                            1,
		                            hTouch->I2cClockSpeedKHz,
		                            AT168_I2C_TIMEOUT);
	} while (Error == NvOdmI2cStatus_Timeout); 

	if (Error != NvOdmI2cStatus_Success)
	{
		NvOsDebugPrintf("I2C Write Failure = %d (addr=0x%x, reg=0x%x, val=0x%0x)\n", Error, 
		                   hTouch->DeviceAddr, reg, val);
		return NV_FALSE;
	}
	
	return NV_TRUE;
}

static NvBool AT168_ReadRegisterOnce (AT168_TouchDevice* hTouch, NvU8 reg, NvU8* buffer, NvU32 len)
{
	NvOdmI2cStatus Error;
	NvOdmI2cTransactionInfo TransactionInfo[2 ];


	TransactionInfo[0].Address = hTouch->DeviceAddr;
	TransactionInfo[0].Buf = &reg;
	TransactionInfo[0].Flags = NVODM_I2C_IS_WRITE | NVODM_I2C_USE_REPEATED_START;
	TransactionInfo[0].NumBytes = 1;

	TransactionInfo[1].Address = hTouch->DeviceAddr | 0x1;
	TransactionInfo[1].Buf = buffer;
	TransactionInfo[1].Flags = 0;
	TransactionInfo[1].NumBytes = len;
    
	do
	{
		Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
		                            TransactionInfo,
		                            2,
		                            hTouch->I2cClockSpeedKHz,
		                            AT168_I2C_TIMEOUT);
	} while (Error == NvOdmI2cStatus_Timeout);

	if (Error != NvOdmI2cStatus_Success)
	{
		NvOsDebugPrintf("I2C Read Failure = %d (addr=0x%x, reg=0x%x)\n", Error,
		                   hTouch->DeviceAddr, reg);
		return NV_FALSE;
	}

	return NV_TRUE;
}

static NvBool AT168_ReadRegisterSafe (AT168_TouchDevice* hTouch, NvU8 reg, NvU8* buffer, NvU32 len)
{
    
	if (!AT168_ReadRegisterOnce(hTouch, reg, buffer, len))
		return NV_FALSE;

	return NV_TRUE;
}

static void InitOdmTouch (NvOdmTouchDevice* Dev)
{
	Dev->Close              = AT168_Close;
	Dev->GetCapabilities    = AT168_GetCapabilities;
	Dev->ReadCoordinate     = AT168_ReadCoordinate;
	Dev->EnableInterrupt    = AT168_EnableInterrupt;
	Dev->HandleInterrupt    = AT168_HandleInterrupt;
	Dev->GetSampleRate      = NULL;
	Dev->SetSampleRate      = NULL;
	Dev->PowerControl       = AT168_PowerControl;
	Dev->PowerOnOff         = AT168_PowerOnOff;
	Dev->GetCalibrationData = NULL;
	Dev->SetCalibration = AT168_SetCalibration;
	Dev->BurnBootloader = AT168_BurnBootloader;
	Dev->OutputDebugMessage = NV_FALSE;
}

void AT168_SetCalibration(NvOdmTouchDeviceHandle hDevice)
{
	AT168_TouchDevice* hTouch = (AT168_TouchDevice*)hDevice;
	
	int counts=6;
	while(counts){	
	//set the SPECOP reg and the touchscreen will calibration by itself;
		if(NV_TRUE==AT168_WRITE(hTouch, AT168_SPECOP, AT168_SPECOP_CALIBRATION_VALUE)) break;
		counts--;
		msleep(10);
	}
	NvOsDebugPrintf("AT168_SetCalibration OK .\n");
}
NvBool AT168_BurnSintexBootloader(NvOdmTouchDeviceHandle hDevice)
{
	NvOsDebugPrintf("AT168_BurnSintexBootloader begin \n");
	AT168_TouchDevice* hTouch = (AT168_TouchDevice*)hDevice;

	NvU8 ResetNum = 0;
	NvU8 status[4];

	//Second step : read the bootloader source file
	const char *filename = "/data/SintekBootloader";
	const char *filename1 = "/sdcard/SintekBootloader";
	const char *filename2 = "/sdcard1/SintekBootloader";
	const char *filename3 = "/sdcard2/SintekBootloader";
	const char *filename4 = "/sdcard3/SintekBootloader";
	const char *filename5 = "/sdcard/sdcard/SintekBootloader";
	const char *filename6 = "/sdcard/sdcard1/SintekBootloader";
	const char *filename7 = "/sdcard/sdcard2/SintekBootloader";
	const char *filename8 = "/sdcard/sdcard3/SintekBootloader";

	struct file *filp; 
	struct file *filp1;
	struct file *filp2;
	struct file *filp3;
	struct file *filp4;
	struct file *filp5;
	struct file *filp6;
	struct file *filp7;
	struct file *filp8;

	struct inode *inode; 
	mm_segment_t fs; 
	off_t fsize; 
	
	NvU8 *buf; 
	NvU8 sendbuf[144]; 
	NvBool isodd = NV_TRUE;
	int i = 0, j = 0;
	int k = 0;
	
	unsigned long magic; 
	NvU32 PinStateValue;
	NvU8 crc_value[1];
	
	printk("start download %s\n", filename); 

	fs=get_fs(); 
	set_fs(KERNEL_DS); 
	
	filp=filp_open(filename,O_RDONLY|O_LARGEFILE,0); 	//data
	if(IS_ERR(filp))	
	{
		printk("can not open file, please check the file in data !\r\n"); 
		//filp_close(filp,NULL);
		//return NV_FALSE;
	}
	else
	{
		printk("Open file SintekBootloader in data success \r\n");
		inode=filp->f_dentry->d_inode; 
		magic=inode->i_sb->s_magic; 
		AT168_PRINTF(("<1>file system magic:%li \n",magic)); 
		AT168_PRINTF(("<1>super blocksize:%li \n",inode->i_sb->s_blocksize)); 
		AT168_PRINTF(("<1>inode %li \n",inode->i_ino));
		fsize=inode->i_size; 
		printk("Sintek bootloader file size:%d \n",fsize);
		buf=(NvU8 *) kmalloc(fsize+1,GFP_ATOMIC); 
		filp->f_op->read(filp, buf, fsize,&(filp->f_pos));
		filp_close(filp,NULL);
		goto OpenSintekFileSuccess;
	}

	filp1=filp_open(filename1,O_RDONLY|O_LARGEFILE,0); 	//sdcard
	if(IS_ERR(filp1))	
	{
		printk("can not open file, please check the file in sdcard !\r\n"); 
		//filp_close(filp1,NULL);
		//return NV_FALSE;
	}
	else
	{
		printk("Open file SintekBootloader in sdcard success \r\n");
		inode=filp1->f_dentry->d_inode; 
		magic=inode->i_sb->s_magic; 
		AT168_PRINTF(("<1>file system magic:%li \n",magic)); 
		AT168_PRINTF(("<1>super blocksize:%li \n",inode->i_sb->s_blocksize)); 
		AT168_PRINTF(("<1>inode %li \n",inode->i_ino));
		fsize=inode->i_size; 
		printk("Sintek bootloader file size:%d \n",fsize);
		buf=(NvU8 *) kmalloc(fsize+1,GFP_ATOMIC); 
		filp1->f_op->read(filp1, buf, fsize,&(filp1->f_pos));
		filp_close(filp1,NULL);
		goto OpenSintekFileSuccess;
	}

	filp2=filp_open(filename2,O_RDONLY|O_LARGEFILE,0); 	//sdcard1
	if(IS_ERR(filp2))	
	{
		printk("can not open file, please check the file in sdcard1 !\r\n"); 
		//filp_close(filp2,NULL);
		//return NV_FALSE;
	}
	else
	{
		printk("Open file SintekBootloader in sdcard1 success \r\n");
		inode=filp2->f_dentry->d_inode; 
		magic=inode->i_sb->s_magic; 
		AT168_PRINTF(("<1>file system magic:%li \n",magic)); 
		AT168_PRINTF(("<1>super blocksize:%li \n",inode->i_sb->s_blocksize)); 
		AT168_PRINTF(("<1>inode %li \n",inode->i_ino));
		fsize=inode->i_size; 
		printk("Sintek bootloader file size:%d \n",fsize);
		buf=(NvU8 *) kmalloc(fsize+1,GFP_ATOMIC); 
		filp2->f_op->read(filp2, buf, fsize,&(filp2->f_pos));
		filp_close(filp2,NULL);
		goto OpenSintekFileSuccess;
	}

	filp3=filp_open(filename3,O_RDONLY|O_LARGEFILE,0); 	//sdcard2
	if(IS_ERR(filp3))	
	{
		printk("can not open file, please check the file in sdcard2 !\r\n"); 
		//filp_close(filp3,NULL);
		//return NV_FALSE;
	}
	else
	{
		printk("Open file SintekBootloader in sdcard2 success \r\n");
		inode=filp3->f_dentry->d_inode; 
		magic=inode->i_sb->s_magic; 
		AT168_PRINTF(("<1>file system magic:%li \n",magic)); 
		AT168_PRINTF(("<1>super blocksize:%li \n",inode->i_sb->s_blocksize)); 
		AT168_PRINTF(("<1>inode %li \n",inode->i_ino));
		fsize=inode->i_size; 
		printk("Sintek bootloader file size:%d \n",fsize);
		buf=(NvU8 *) kmalloc(fsize+1,GFP_ATOMIC); 
		filp3->f_op->read(filp3, buf, fsize,&(filp3->f_pos));
		filp_close(filp3,NULL);
		goto OpenSintekFileSuccess;
	}

	filp4=filp_open(filename4,O_RDONLY|O_LARGEFILE,0); 	//sdcard3
	if(IS_ERR(filp4))	
	{
		printk("can not open file, please check the file in sdcard3 !\r\n"); 
		//filp_close(filp4,NULL);
		//return NV_FALSE;
	}
	else
	{
		printk("Open file SintekBootloader in sdcard3 success \r\n");
		inode=filp4->f_dentry->d_inode; 
		magic=inode->i_sb->s_magic; 
		AT168_PRINTF(("<1>file system magic:%li \n",magic)); 
		AT168_PRINTF(("<1>super blocksize:%li \n",inode->i_sb->s_blocksize)); 
		AT168_PRINTF(("<1>inode %li \n",inode->i_ino));
		fsize=inode->i_size; 
		printk("Sintek bootloader file size:%d \n",fsize);
		buf=(NvU8 *) kmalloc(fsize+1,GFP_ATOMIC); 
		filp4->f_op->read(filp4, buf, fsize,&(filp4->f_pos));
		filp_close(filp4,NULL);
		goto OpenSintekFileSuccess;
	}

	filp5=filp_open(filename5,O_RDONLY|O_LARGEFILE,0); 	//sdcard/sdcard
	if(IS_ERR(filp5))	
	{
		printk("can not open file, please check the file in sdcard/sdcard !\r\n"); 
		//filp_close(filp5,NULL);
		//return NV_FALSE;
	}
	else
	{
		printk("Open file SintekBootloader in sdcard/sdcard success \r\n");
		inode=filp5->f_dentry->d_inode; 
		magic=inode->i_sb->s_magic; 
		AT168_PRINTF(("<1>file system magic:%li \n",magic)); 
		AT168_PRINTF(("<1>super blocksize:%li \n",inode->i_sb->s_blocksize)); 
		AT168_PRINTF(("<1>inode %li \n",inode->i_ino));
		fsize=inode->i_size; 
		printk("Sintek bootloader file size:%d \n",fsize);
		buf=(NvU8 *) kmalloc(fsize+1,GFP_ATOMIC); 
		filp5->f_op->read(filp5, buf, fsize,&(filp5->f_pos));
		filp_close(filp5,NULL);
		goto OpenSintekFileSuccess;
	}

	filp6=filp_open(filename6,O_RDONLY|O_LARGEFILE,0); 	//sdcard/sdcard1
	if(IS_ERR(filp6))	
	{
		printk("can not open file, please check the file in sdcard/sdcard1 !\r\n"); 
		//filp_close(filp6,NULL);
		//return NV_FALSE;
	}
	else
	{
		printk("Open file SintekBootloader in sdcard/sdcard1 success \r\n");
		inode=filp6->f_dentry->d_inode; 
		magic=inode->i_sb->s_magic; 
		AT168_PRINTF(("<1>file system magic:%li \n",magic)); 
		AT168_PRINTF(("<1>super blocksize:%li \n",inode->i_sb->s_blocksize)); 
		AT168_PRINTF(("<1>inode %li \n",inode->i_ino));
		fsize=inode->i_size; 
		printk("Sintek bootloader file size:%d \n",fsize);
		buf=(NvU8 *) kmalloc(fsize+1,GFP_ATOMIC); 
		filp6->f_op->read(filp6, buf, fsize,&(filp6->f_pos));
		filp_close(filp6,NULL);
		goto OpenSintekFileSuccess;
	}

	filp7=filp_open(filename7,O_RDONLY|O_LARGEFILE,0); 	//sdcard/sdcard2
	if(IS_ERR(filp7))	
	{
		printk("can not open file, please check the file in sdcard/sdcard2 !\r\n"); 
		//filp_close(filp7,NULL);
		//return NV_FALSE;
	}
	else
	{
		printk("Open file SintekBootloader in sdcard/sdcard2 success \r\n");
		inode=filp7->f_dentry->d_inode; 
		magic=inode->i_sb->s_magic; 
		AT168_PRINTF(("<1>file system magic:%li \n",magic)); 
		AT168_PRINTF(("<1>super blocksize:%li \n",inode->i_sb->s_blocksize)); 
		AT168_PRINTF(("<1>inode %li \n",inode->i_ino));
		fsize=inode->i_size; 
		printk("Sintek bootloader file size:%d \n",fsize);
		buf=(NvU8 *) kmalloc(fsize+1,GFP_ATOMIC); 
		filp7->f_op->read(filp7, buf, fsize,&(filp7->f_pos));
		filp_close(filp7,NULL);
		goto OpenSintekFileSuccess;
	}

	filp8=filp_open(filename8,O_RDONLY|O_LARGEFILE,0); 	//sdcard/sdcard3
	if(IS_ERR(filp8))	
	{
		printk("can not open file, please check the file in sdcard/sdcard3!\r\n"); 
		//filp_close(filp8,NULL);
		printk("---Open SintekBootloader Fail . Now return False. --- \r\n"); 
		return NV_FALSE;
	}
	else
	{
		printk("Open file SintekBootloader in sdcard/sdcard3 success \r\n");
		inode=filp8->f_dentry->d_inode; 
		magic=inode->i_sb->s_magic; 
		AT168_PRINTF(("<1>file system magic:%li \n",magic)); 
		AT168_PRINTF(("<1>super blocksize:%li \n",inode->i_sb->s_blocksize)); 
		AT168_PRINTF(("<1>inode %li \n",inode->i_ino));
		fsize=inode->i_size; 
		printk("Sintek bootloader file size:%d \n",fsize);
		buf=(NvU8 *) kmalloc(fsize+1,GFP_ATOMIC); 
		filp8->f_op->read(filp8, buf, fsize,&(filp8->f_pos));
		filp_close(filp8,NULL);
		goto OpenSintekFileSuccess;
	}

OpenSintekFileSuccess:	
	set_fs(fs);

RetryBurnSintek:
	//************************************************************************************//
	//Re-start the touch panel,  then goto bootloader status
	NvOdmGpioSetState(hTouch->hGpio,
	                hTouch->hPinReset,
	                NvOdmGpioPinActiveState_Low);
	msleep(5);
	NvOdmGpioSetState(hTouch->hGpio,
	                hTouch->hPinReset,
	                NvOdmGpioPinActiveState_High);
	msleep(20);	//must send bootloader I2C address after reset in 2ms-50ms
	
	if(!(AT168_Bootloader_Read(hTouch, status, 4)))
	{
		ResetNum ++;
		if(ResetNum > 3)
		{
			NvOsDebugPrintf("AT168_BurnSintexBootloader --  Read status fail  return NV_FALSE \n");
			return NV_FALSE;
		}
		else
		{
			NvOsDebugPrintf("AT168_BurnSintexBootloader --  Read status fail  try %d \n", ResetNum);
			goto RetryBurnSintek;
		}
	}
	else
	{
		NvOsDebugPrintf("AT168_BurnSintexBootloader --Read status OK  ( 0x%x 0x%x 0x%x 0x%x ) \n", status[0], status[1], status[2], status[3]);
	}
	//************************************************************************************//
	k = 0;
	//printk("<1>The File Content  is:\n");
	for(i = 1;i < fsize-1; i++)		//ignore the first '$', so i =1
	{
		if(('$' == buf[i]) ||(0x0d == buf[i]) || (0x0a == buf[i]))
		{
			if('$' == buf[i])
			{
				//printk(" $  buf[%d] = 0x%x \n",i, buf[i]);
				//printk(" j = %d \n",j);
				j = 0;
				
			}
			else
			{
				//printk(" buf go to line end \n");
			}
		}
		else
		{
			if(NV_TRUE == isodd)
			{
				sendbuf[j] = 0;
				if(('0'<=buf[i])&&(buf[i]<='9'))
				{
					sendbuf[j] = 16*(buf[i] - 0x30);
				}
				else if(('A'<=buf[i])&&(buf[i]<='F'))
				{
					sendbuf[j] = 16*(buf[i] - 'A' + 0x0a);
				}
				else
				{
					printk(" odd  NV_FALSE buf[%d] = 0x%x \n",i, buf[i]);
					goto RetryBurnSintek;
					//return NV_FALSE;
				}
				isodd = NV_FALSE;
				//printk("odd  buf[%d] = 0x%x  --- sendbuf [%d] = 0x%x \n",i, buf[i], j, sendbuf[j]);
			}
			else //even
			{
				if(('0'<=buf[i])&&(buf[i]<='9'))
				{
					sendbuf[j] += (buf[i] - 0x30);
				}
				else if(('A'<=buf[i])&&(buf[i]<='F'))
				{
					sendbuf[j] += (buf[i] - 'A' + 0x0a);
				}
				else
				{
					printk(" even NV_FALSE buf[%d] = 0x%x \n",i, buf[i]);
					goto RetryBurnSintek;
					//return NV_FALSE;
				}
				isodd = NV_TRUE;
				//printk("even  buf[%d] = 0x%x  --- sendbuf [%d] = 0x%x \n",i, buf[i], j, sendbuf[j]);
				#if 0
				if(sendbuf[j] > 0x0f)
				{
					printk("%x",sendbuf[j]);
				}
				else
				{
					printk("0%x",sendbuf[j]);
				}
				#endif
				
				j++;

				//Third step : write bootloader command, download 143 bytes every time
				
				if(143 == j)
				{
					//printk("\n");
					//printk("sendbuf line %d is ",k);
					k++;

					if(!(AT168_Bootloader_Write(hTouch, sendbuf, 143)))
					{
						NvOsDebugPrintf("AT168_BurnSintexBootloader step 3  AT168_Bootloader_Write fail \n");
						goto RetryBurnSintek;
						//return NV_FALSE;
					}
					else
					{
						//NvOsDebugPrintf("AT168_BurnSintexBootloader step 3  AT168_Bootloader_Write success \n");
					}

					if(0x01 != sendbuf[0])	//0x01 is the last line
					{
						//NvOsDebugPrintf("AT168_BurnSintexBootloader step 3  sendbuf[0] is not 0x01 \n");
						msleep(1);
						do
						{
							//NvOsDebugPrintf("AT168_BurnBootloader step 3  NvOdmGpioGetState begin \n");
							//msleep(2);
							NvOdmGpioGetState( hTouch->hGpio, hTouch->hPinInterrupt, &PinStateValue);
							//msleep(2);
							//NvOsDebugPrintf("AT168_BurnBootloader step 3  NvOdmGpioGetState end \n");
						}while(PinStateValue);
						msleep(1);
					//}

					//CRC check
					//NvOsDebugPrintf("AT168_BurnSintexBootloader step 3  CRC check begin \n");
					if(!(AT168_Bootloader_Read(hTouch, crc_value, 1)))
					{
						NvOsDebugPrintf("AT168_BurnSintexBootloader step 3  AT168_Bootloader_Read fail \n");
						goto RetryBurnSintek;
						//return NV_FALSE;
					}
					else
					{
						NvOsDebugPrintf("AT168_BurnSintexBootloader step 3 line %d return the crc_value[0] is 0x%x \n", k, crc_value[0]);
						if((crc_value[0] >= 0x80) && (0x01 != sendbuf[0]))  // if first bit is '1', crc value is fail
						{
							NvOsDebugPrintf("AT168_BurnSintexBootloader step 3  CRC  fail j = %d\n",j);
							printk("sendbuf line %d is ",k);
							int ii = 0;
							do
							{
								if(sendbuf[ii] > 0x0f)
								{
									printk("%x",sendbuf[ii]);
								}
								else
								{
									printk("0%x",sendbuf[ii]);
								}
								ii++;
							}while(ii < j);
							printk("\n");
							
							goto RetryBurnSintek;
							//return NV_FALSE;;           // return Fail
						}
						else
						{
							//NvOsDebugPrintf("AT168_BurnSintexBootloader step 3  CRC  success \n");
						}
					}
					}
				}
			}
		}
	}
	
	msleep(3000);
	
	//Fourth step : Force reset and calibration
	NvOdmGpioSetState(hTouch->hGpio,
	                hTouch->hPinReset,
	                NvOdmGpioPinActiveState_Low);
	msleep(5);
	NvOdmGpioSetState(hTouch->hGpio,
	                hTouch->hPinReset,
	                NvOdmGpioPinActiveState_High);
	msleep(60);
	//do calibration
	hTouch->DeviceAddr = (NvU32)(0x5c << 1);	//change I2C back to normal 0x5c
	if(AT168_WRITE(hTouch, AT168_SPECOP, AT168_SPECOP_CALIBRATION_VALUE))
	{
		NvOsDebugPrintf("AT168_BurnSintexBootloader do calibration OK .\n");
	}
	else
	{
		NvOsDebugPrintf("AT168_BurnSintexBootloader do calibration fail .\n");
	}
	
	return NV_TRUE;
}
NvBool AT168_BurnCandoBootloader(NvOdmTouchDeviceHandle hDevice)
{
	
	NvOsDebugPrintf("AT168_BurnCandoBootloader begin \n");
	AT168_TouchDevice* hTouch = (AT168_TouchDevice*)hDevice;

	//**************************************************************************************//
	//Second step : prepare before write to touch panel
	//------ goto the Cando bootloader mode ------
	NvU16 g_CRCReceived;		//for check the local_crc
	NvU32 PinStateValue;		//for check the int gpio state
	NvU8	readbuff[9];
	NvU8	checkintstatusnum = 0;
	
	// Step 0: Reboot TP boot-loader
	NvU8 TP_ReBootLoadCommand[1];
	TP_ReBootLoadCommand[0] = 0xC0; //command : TP_ReBootLoad
	NvU8 RetryC0Num = 0;
RetryBurnCando:
	if(!(AT168_Bootloader_Write(hTouch, TP_ReBootLoadCommand, 1)))
	{
		NvOsDebugPrintf("AT168_BurnCandoBootloader step 2.1  AT168_Bootloader_Write 0xC0 fail \n");
		if(RetryC0Num > 100)
			return NV_FALSE;
		else
			RetryC0Num++;
		goto RetryBurnCando;
	}
	else
	{
		AT168_PRINTF(("AT168_BurnCandoBootloader step 2.1  AT168_Bootloader_Write 0xC0 success \n"));
	}
	msleep(20);	//delay 1ms for process to be completed

	//**************************************************************************************//

	//------ Enable FLASH writes and erases ------
	NvU8 nLoop = 0;
	NvU8 FlashKeycommand[4];
	FlashKeycommand[0] = 3;	//BLSize
	FlashKeycommand[1] = 0xF7;	//BL_SetFlashKeyCodes
	FlashKeycommand[2] = 0xA5; 	//FLASH_KEY0
	FlashKeycommand[3] = 0xF1;  //FLASH_KEY1
		
	while(nLoop<5)
	{
		if(!(AT168_Bootloader_Write(hTouch, FlashKeycommand, 4)))
		{
			NvOsDebugPrintf("AT168_BurnCandoBootloader step 2.2  AT168_Bootloader_Write FlashKeycommand looping time %d \n", nLoop);
			nLoop++;
		}
		else
		{
			AT168_PRINTF(("AT168_BurnCandoBootloader step 2.2  AT168_Bootloader_Write  FlashKeycommand success \n"));
			break;
		}
		
		if( nLoop == 5 )
		{
			NvOsDebugPrintf("AT168_BurnCandoBootloader step 2.2  AT168_Bootloader_Write  FlashKeycommand fail \n");
			//return NV_FALSE;
			goto RetryBurnCando;
		}
	}
	
	//wait for gpio_int change to high
	msleep(1);
	checkintstatusnum = 0;
	do
	{
		AT168_PRINTF(("AT168_BurnCandoBootloader step 2.2  NvOdmGpioGetState begin \n"));
		msleep(2);
		NvOdmGpioGetState( hTouch->hGpio, hTouch->hPinInterrupt, &PinStateValue);
		AT168_PRINTF(("AT168_BurnCandoBootloader step 2.2  NvOdmGpioGetState end \n"));
		checkintstatusnum+=2;
		if(checkintstatusnum > 5000)
		{
			NvOsDebugPrintf("AT168_BurnCandoBootloader step 2.2  NvOdmGpioGetState checkintstatusnum up 5 second, give up \n");
			return NV_FALSE;
		}
	}while(!PinStateValue);
	msleep(1);
	
	//**************************************************************************************//
	//First step : read the bootloader source file
	const char *filename = "/data/CandoBootloader";
	const char *filename1 = "/sdcard/CandoBootloader";
	const char *filename2 = "/sdcard1/CandoBootloader";
	const char *filename3 = "/sdcard2/CandoBootloader";
	const char *filename4 = "/sdcard3/CandoBootloader";
	const char *filename5 = "/sdcard/sdcard/CandoBootloader";
	const char *filename6 = "/sdcard/sdcard1/CandoBootloader";
	const char *filename7 = "/sdcard/sdcard2/CandoBootloader";
	const char *filename8 = "/sdcard/sdcard3/CandoBootloader";
	
	struct file *filp; 
	struct file *filp1;
	struct file *filp2;
	struct file *filp3;
	struct file *filp4;
	struct file *filp5;
	struct file *filp6;
	struct file *filp7;
	struct file *filp8;
	
	struct inode *inode; 
	mm_segment_t fs; 
	off_t fsize; 
	unsigned long magic;
	NvU8 *buf; 

	NvBool isodd = NV_TRUE;
	NvU16 linenum;
	//int i = 0, j = 0;

	printk("start download %s\n", filename); 

	fs=get_fs(); 
	set_fs(KERNEL_DS); 

	filp=filp_open(filename,O_RDONLY|O_LARGEFILE,0); 	//data
	if(IS_ERR(filp))	
	{
		printk("can not open file, please check the file in data !\r\n"); 
		//filp_close(filp,NULL);
		//return NV_FALSE;
	}
	else
	{
		printk("Open file SintekBootloader in data success \r\n");
		inode=filp->f_dentry->d_inode; 
		magic=inode->i_sb->s_magic; 
		AT168_PRINTF(("<1>file system magic:%li \n",magic)); 
		AT168_PRINTF(("<1>super blocksize:%li \n",inode->i_sb->s_blocksize)); 
		AT168_PRINTF(("<1>inode %li \n",inode->i_ino));
		fsize=inode->i_size; 
		printk("Sintek bootloader file size:%d \n",fsize);
		buf=(NvU8 *) kmalloc(fsize+1,GFP_ATOMIC); 
		filp->f_op->read(filp, buf, fsize,&(filp->f_pos));
		filp_close(filp,NULL);
		goto OpenCandoFileSuccess;
	}

	filp1=filp_open(filename1,O_RDONLY|O_LARGEFILE,0); 	//sdcard
	if(IS_ERR(filp1))	
	{
		printk("can not open file, please check the file in sdcard !\r\n"); 
		//filp_close(filp1,NULL);
		//return NV_FALSE;
	}
	else
	{
		printk("Open file SintekBootloader in sdcard success \r\n");
		inode=filp1->f_dentry->d_inode; 
		magic=inode->i_sb->s_magic; 
		AT168_PRINTF(("<1>file system magic:%li \n",magic)); 
		AT168_PRINTF(("<1>super blocksize:%li \n",inode->i_sb->s_blocksize)); 
		AT168_PRINTF(("<1>inode %li \n",inode->i_ino));
		fsize=inode->i_size; 
		printk("Sintek bootloader file size:%d \n",fsize);
		buf=(NvU8 *) kmalloc(fsize+1,GFP_ATOMIC); 
		filp1->f_op->read(filp1, buf, fsize,&(filp1->f_pos));
		filp_close(filp1,NULL);
		goto OpenCandoFileSuccess;
	}

	filp2=filp_open(filename2,O_RDONLY|O_LARGEFILE,0); 	//sdcard1
	if(IS_ERR(filp2))	
	{
		printk("can not open file, please check the file in sdcard1 !\r\n"); 
		//filp_close(filp2,NULL);
		//return NV_FALSE;
	}
	else
	{
		printk("Open file SintekBootloader in sdcard1 success \r\n");
		inode=filp2->f_dentry->d_inode; 
		magic=inode->i_sb->s_magic; 
		AT168_PRINTF(("<1>file system magic:%li \n",magic)); 
		AT168_PRINTF(("<1>super blocksize:%li \n",inode->i_sb->s_blocksize)); 
		AT168_PRINTF(("<1>inode %li \n",inode->i_ino));
		fsize=inode->i_size; 
		printk("Sintek bootloader file size:%d \n",fsize);
		buf=(NvU8 *) kmalloc(fsize+1,GFP_ATOMIC); 
		filp2->f_op->read(filp2, buf, fsize,&(filp2->f_pos));
		filp_close(filp2,NULL);
		goto OpenCandoFileSuccess;
	}

	filp3=filp_open(filename3,O_RDONLY|O_LARGEFILE,0); 	//sdcard2
	if(IS_ERR(filp3))	
	{
		printk("can not open file, please check the file in sdcard2 !\r\n"); 
		//filp_close(filp3,NULL);
		//return NV_FALSE;
	}
	else
	{
		printk("Open file SintekBootloader in sdcard2 success \r\n");
		inode=filp3->f_dentry->d_inode; 
		magic=inode->i_sb->s_magic; 
		AT168_PRINTF(("<1>file system magic:%li \n",magic)); 
		AT168_PRINTF(("<1>super blocksize:%li \n",inode->i_sb->s_blocksize)); 
		AT168_PRINTF(("<1>inode %li \n",inode->i_ino));
		fsize=inode->i_size; 
		printk("Sintek bootloader file size:%d \n",fsize);
		buf=(NvU8 *) kmalloc(fsize+1,GFP_ATOMIC); 
		filp3->f_op->read(filp3, buf, fsize,&(filp3->f_pos));
		filp_close(filp3,NULL);
		goto OpenCandoFileSuccess;
	}

	filp4=filp_open(filename4,O_RDONLY|O_LARGEFILE,0); 	//sdcard3
	if(IS_ERR(filp4))	
	{
		printk("can not open file, please check the file in sdcard3 !\r\n"); 
		//filp_close(filp4,NULL);
		//return NV_FALSE;
	}
	else
	{
		printk("Open file SintekBootloader in sdcard3 success \r\n");
		inode=filp4->f_dentry->d_inode; 
		magic=inode->i_sb->s_magic; 
		AT168_PRINTF(("<1>file system magic:%li \n",magic)); 
		AT168_PRINTF(("<1>super blocksize:%li \n",inode->i_sb->s_blocksize)); 
		AT168_PRINTF(("<1>inode %li \n",inode->i_ino));
		fsize=inode->i_size; 
		printk("Sintek bootloader file size:%d \n",fsize);
		buf=(NvU8 *) kmalloc(fsize+1,GFP_ATOMIC); 
		filp4->f_op->read(filp4, buf, fsize,&(filp4->f_pos));
		filp_close(filp4,NULL);
		goto OpenCandoFileSuccess;
	}

	filp5=filp_open(filename5,O_RDONLY|O_LARGEFILE,0); 	//sdcard/sdcard
	if(IS_ERR(filp5))	
	{
		printk("can not open file, please check the file in sdcard/sdcard !\r\n"); 
		//filp_close(filp5,NULL);
		//return NV_FALSE;
	}
	else
	{
		printk("Open file SintekBootloader in sdcard/sdcard success \r\n");
		inode=filp5->f_dentry->d_inode; 
		magic=inode->i_sb->s_magic; 
		AT168_PRINTF(("<1>file system magic:%li \n",magic)); 
		AT168_PRINTF(("<1>super blocksize:%li \n",inode->i_sb->s_blocksize)); 
		AT168_PRINTF(("<1>inode %li \n",inode->i_ino));
		fsize=inode->i_size; 
		printk("Sintek bootloader file size:%d \n",fsize);
		buf=(NvU8 *) kmalloc(fsize+1,GFP_ATOMIC); 
		filp5->f_op->read(filp5, buf, fsize,&(filp5->f_pos));
		filp_close(filp5,NULL);
		goto OpenCandoFileSuccess;
	}

	filp6=filp_open(filename6,O_RDONLY|O_LARGEFILE,0); 	//sdcard/sdcard1
	if(IS_ERR(filp6))	
	{
		printk("can not open file, please check the file in sdcard/sdcard1 !\r\n"); 
		//filp_close(filp6,NULL);
		//return NV_FALSE;
	}
	else
	{
		printk("Open file SintekBootloader in sdcard/sdcard1 success \r\n");
		inode=filp6->f_dentry->d_inode; 
		magic=inode->i_sb->s_magic; 
		AT168_PRINTF(("<1>file system magic:%li \n",magic)); 
		AT168_PRINTF(("<1>super blocksize:%li \n",inode->i_sb->s_blocksize)); 
		AT168_PRINTF(("<1>inode %li \n",inode->i_ino));
		fsize=inode->i_size; 
		printk("Sintek bootloader file size:%d \n",fsize);
		buf=(NvU8 *) kmalloc(fsize+1,GFP_ATOMIC); 
		filp6->f_op->read(filp6, buf, fsize,&(filp6->f_pos));
		filp_close(filp6,NULL);
		goto OpenCandoFileSuccess;
	}

	filp7=filp_open(filename7,O_RDONLY|O_LARGEFILE,0); 	//sdcard/sdcard2
	if(IS_ERR(filp7))	
	{
		printk("can not open file, please check the file in sdcard/sdcard2 !\r\n"); 
		//filp_close(filp7,NULL);
		//return NV_FALSE;
	}
	else
	{
		printk("Open file SintekBootloader in sdcard/sdcard2 success \r\n");
		inode=filp7->f_dentry->d_inode; 
		magic=inode->i_sb->s_magic; 
		AT168_PRINTF(("<1>file system magic:%li \n",magic)); 
		AT168_PRINTF(("<1>super blocksize:%li \n",inode->i_sb->s_blocksize)); 
		AT168_PRINTF(("<1>inode %li \n",inode->i_ino));
		fsize=inode->i_size; 
		printk("Sintek bootloader file size:%d \n",fsize);
		buf=(NvU8 *) kmalloc(fsize+1,GFP_ATOMIC); 
		filp7->f_op->read(filp7, buf, fsize,&(filp7->f_pos));
		filp_close(filp7,NULL);
		goto OpenCandoFileSuccess;
	}

	
	filp8=filp_open(filename8,O_RDONLY|O_LARGEFILE,0); 	//sdcard/sdcard2
	if(IS_ERR(filp8))	
	{
		printk("can not open file, please check the file in sdcard/sdcard2 !\r\n"); 
		//filp_close(filp8,NULL);
		//return NV_FALSE;
	}
	else
	{
		printk("Open file SintekBootloader in sdcard/sdcard2 success \r\n");
		inode=filp8->f_dentry->d_inode; 
		magic=inode->i_sb->s_magic; 
		AT168_PRINTF(("<1>file system magic:%li \n",magic)); 
		AT168_PRINTF(("<1>super blocksize:%li \n",inode->i_sb->s_blocksize)); 
		AT168_PRINTF(("<1>inode %li \n",inode->i_ino));
		fsize=inode->i_size; 
		printk("Sintek bootloader file size:%d \n",fsize);
		buf=(NvU8 *) kmalloc(fsize+1,GFP_ATOMIC); 
		filp8->f_op->read(filp8, buf, fsize,&(filp8->f_pos));
		filp_close(filp,NULL);
		goto OpenCandoFileSuccess;
	}

OpenCandoFileSuccess:

	set_fs(fs);
	
	//**************************************************************************************//
	//------ Erase last user page to clear the Validation Signature (page containing address 0x7800) ------
	NvU8 Erasecommand[4];
	Erasecommand[0] = 3;	//BLSize
	Erasecommand[1] = 0xF1;	//BL_ErasePage
	Erasecommand[2] = 0x7800 & 0xFF; 	//(eraseAddress&0xFF)
	Erasecommand[3] = 0x7800 >> 8; 	//(eraseAddress>>8)

	if(!(AT168_Bootloader_Write(hTouch, Erasecommand, 4)))
	{
		NvOsDebugPrintf("AT168_BurnCandoBootloader step 2.3  AT168_Bootloader_Write  Erasecommand fail \n");
		goto RetryBurnCando;
		//return NV_FALSE;
	}
	else
	{
		AT168_PRINTF(("AT168_BurnCandoBootloader step 2.3  AT168_Bootloader_Write  Erasecommand success \n"));
	}
	
	//wait for gpio_int change to high
	msleep(1);
	checkintstatusnum = 0;
	do
	{
		AT168_PRINTF(("AT168_BurnCandoBootloader step 2.3  NvOdmGpioGetState begin \n"));
		msleep(2);
		NvOdmGpioGetState( hTouch->hGpio, hTouch->hPinInterrupt, &PinStateValue);
		AT168_PRINTF(("AT168_BurnCandoBootloader step 2.3  NvOdmGpioGetState end \n"));
		checkintstatusnum+=2;
		if(checkintstatusnum > 5000)
		{
			NvOsDebugPrintf("AT168_BurnCandoBootloader step 2.3  NvOdmGpioGetState checkintstatusnum up 5 second, give up \n");
			return NV_FALSE;
		}
	}while(!PinStateValue);
	msleep(1);
	
	//**************************************************************************************//
	//Third step : write FW to touch panel,  Erase/Write/CRC Flash pages
	int k = 0;
	NvU8 nPageNum;
	NvU16 nWriteAddress;
	NvU8 WriteEraseCommand[4];
	NvU8 WriteCommand[36];
	NvU16 nByteWriteOffset = 0;
	NvU8 index = 0;
	NvU16 local_crc = 0;
	
	for (nPageNum = 0x04; nPageNum <= 0x1E; nPageNum++) // TP_FW_BEGIN_PAGE: 0x04 ;   TP_FW_END_PAGE : 0x1E;
	{
		nWriteAddress = (nPageNum * 1024);
		
		// Erase page before writing to it
		WriteEraseCommand[0] = 3;	//BLSize
		WriteEraseCommand[1] = 0xF1;	//BL_ErasePage
		WriteEraseCommand[2] = nWriteAddress & 0xFF; 	//(nWriteAddress&0xFF)
		WriteEraseCommand[3] = nWriteAddress >> 8; 	//(nWriteAddress>>8)

		if(!(AT168_Bootloader_Write(hTouch, WriteEraseCommand, 4)))
		{
			NvOsDebugPrintf("AT168_BurnCandoBootloader step 3.1  AT168_Bootloader_Write  WriteEraseCommand fail \n");
			goto RetryBurnCando;
			//return NV_FALSE;
		}
		else
		{
			AT168_PRINTF(("AT168_BurnCandoBootloader step 3.1  AT168_Bootloader_Write  WriteEraseCommand success \n"));
		}
		//wait for gpio_int change to high
		msleep(1);
		checkintstatusnum = 0;
		do
		{
			AT168_PRINTF(("AT168_BurnCandoBootloader step 3.1  Erase NvOdmGpioGetState begin \n"));
			msleep(2);
			NvOdmGpioGetState( hTouch->hGpio, hTouch->hPinInterrupt, &PinStateValue);
			AT168_PRINTF(("AT168_BurnCandoBootloader step 3.1  Erase NvOdmGpioGetState end \n"));
			checkintstatusnum+=2;
			if(checkintstatusnum > 5000)
			{
				NvOsDebugPrintf("AT168_BurnCandoBootloader step 3.1 Erase  NvOdmGpioGetState checkintstatusnum up 5 second, give up \n");
				return NV_FALSE;
			}
		}while(!PinStateValue);
		
		//get the return from TP
		memset(readbuff, 0x00, 9);
		if(!(AT168_Bootloader_Read(hTouch, readbuff, 9)))
		{
			NvOsDebugPrintf("AT168_BurnCandoBootloader step 3.1 Write  AT168_Bootloader_Read fail \n");
			goto RetryBurnCando;
			//return NV_FALSE;
		}
		else
		{
			if(0x72 == readbuff[1])	// BLR_WriteBytes : 0x72
			{
				AT168_PRINTF(("AT168_BurnCandoBootloader step 3.1 Write readbuff[1] is BLR_WriteBytes \n"));
				if((readbuff[0] >= 4) && (0x00 == readbuff[2]))	//COMMAND_SUCCESS : 0x00
					g_CRCReceived = (readbuff[3] | (((NvU16)readbuff[4])<<8));
				else
					g_CRCReceived = 0xFFFF;
			}
			AT168_PRINTF(("AT168_BurnCandoBootloader step 3.1 Write  return the status are 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", readbuff[0], readbuff[1], readbuff[2], readbuff[3], readbuff[4], readbuff[5], readbuff[6], readbuff[7], readbuff[8]));
		}
		msleep(1);

		nByteWriteOffset = 0;

		// Write 1024 bytes to FLASH
		while(nByteWriteOffset < 1024)
		{
			WriteCommand[0] = 35;	//BLSize
			WriteCommand[1] = 0xF2;	//BL_WriteBytes
			WriteCommand[2] = (nWriteAddress+nByteWriteOffset) & 0xFF ;
			WriteCommand[3] = (nWriteAddress+nByteWriteOffset) >>8;

			for (index = 0; index < 32; index++ )
			{
				WriteCommand[index+4] = buf[nWriteAddress+nByteWriteOffset+index];
			}

			if(!(AT168_Bootloader_Write(hTouch, WriteCommand, 36)))
			{
				NvOsDebugPrintf("AT168_BurnCandoBootloader step 3.2  AT168_Bootloader_Write  WriteCommand fail \n");
				goto RetryBurnCando;
				//return NV_FALSE;
			}
			else
			{
				AT168_PRINTF(("AT168_BurnCandoBootloader step 3.2  AT168_Bootloader_Write  WriteCommand success nPageNum = %x \n", nPageNum));
			}
			
			//wait for gpio_int change to high
			msleep(1);
			checkintstatusnum = 0;
			do
			{
				AT168_PRINTF(("AT168_BurnCandoBootloader step 3.2  Write NvOdmGpioGetState begin \n"));
				msleep(2);
				NvOdmGpioGetState( hTouch->hGpio, hTouch->hPinInterrupt, &PinStateValue);
				AT168_PRINTF(("AT168_BurnCandoBootloader step 3.2  Write NvOdmGpioGetState end \n"));
				checkintstatusnum+=2;
				if(checkintstatusnum > 5000)
				{
					NvOsDebugPrintf("AT168_BurnCandoBootloader step 3.2 Write  NvOdmGpioGetState checkintstatusnum up 5 second, give up \n");
					return NV_FALSE;
				}
			}while(!PinStateValue);
			msleep(1);
			//get the return from TP
			memset(readbuff, 0x00, 9);
			if(!(AT168_Bootloader_Read(hTouch, readbuff, 9)))
			{
				NvOsDebugPrintf("AT168_BurnCandoBootloader step 3.2 Write  AT168_Bootloader_Read fail \n");
				goto RetryBurnCando;
				//return NV_FALSE;
			}
			else
			{
				if(0x72 == readbuff[1])	// BLR_WriteBytes : 0x72
				{
					AT168_PRINTF(("AT168_BurnCandoBootloader step 3.2 Write readbuff[1] is BLR_WriteBytes \n"));
					if((readbuff[0] >= 4) && (0x00 == readbuff[2]))	//COMMAND_SUCCESS : 0x00
						g_CRCReceived = (readbuff[3] | (((NvU16)readbuff[4])<<8));
					else
						g_CRCReceived = 0xFFFF;
				}
				AT168_PRINTF(("AT168_BurnCandoBootloader step 3.2 Write  return the status are 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", readbuff[0], readbuff[1], readbuff[2], readbuff[3], readbuff[4], readbuff[5], readbuff[6], readbuff[7], readbuff[8]));
			}

			//check the local_crc
			local_crc = 0;
			for( index = 0; index < 32; index++ )
			{
				local_crc = local_crc ^ WriteCommand[index+4];
				for(k = 0; k < 8; k++)
				{
				      if (local_crc & 0x01)
				      {
				         local_crc = local_crc >> 1;
				         local_crc ^= 0x8408;		//CRC_POLY : 0x8408
				      }
				      else
				      {
				         local_crc = local_crc >> 1;
				      }
				}
			}

			AT168_PRINTF(("AT168_BurnCandoBootloader step 3.2  local_crc is (%x)..g_CRCReceived is (%x)..... \n", local_crc, g_CRCReceived));

			if( local_crc != g_CRCReceived )
			{
				NvOsDebugPrintf("AT168_BurnCandoBootloader step 3.2  AT168_Bootloader_Write  CRC check failed........ \n");
				goto RetryBurnCando;
				//return NV_FALSE;
			}
			
			nByteWriteOffset += 32;
		}
		NvOsDebugPrintf("AT168_BurnCandoBootloader step 3.2  AT168_Bootloader_Write  OK nPageNum is  %x....... \n", nPageNum);
	}

	//**************************************************************************************//
	// Fourth step : Write Signature
	NvU8 WriteSignaturecommand[4];
	WriteSignaturecommand[0] = 3;	//BLSize
	WriteSignaturecommand[1] = 0xF4;		//BL_WriteValidation
	WriteSignaturecommand[2] = 0xC2;
	WriteSignaturecommand[3] = 0x3D;

	if(!(AT168_Bootloader_Write(hTouch, WriteSignaturecommand, 4)))
	{
		NvOsDebugPrintf("AT168_BurnCandoBootloader step 4.1  AT168_Bootloader_Write  WriteSignaturecommand fail \n");
		goto RetryBurnCando;
		//return NV_FALSE;
	}
	else
	{
		AT168_PRINTF(("AT168_BurnCandoBootloader step 4.1  AT168_Bootloader_Write  WriteSignaturecommand success \n"));
	}
	
	//wait for gpio_int change to high
	msleep(1);
	checkintstatusnum = 0;
	do
	{
		AT168_PRINTF(("AT168_BurnCandoBootloader step 4.1  NvOdmGpioGetState begin \n"));
		msleep(2);
		NvOdmGpioGetState( hTouch->hGpio, hTouch->hPinInterrupt, &PinStateValue);
		AT168_PRINTF(("AT168_BurnCandoBootloader step 4.1  NvOdmGpioGetState end \n"));
		checkintstatusnum+=2;
		if(checkintstatusnum > 5000)
		{
			NvOsDebugPrintf("AT168_BurnCandoBootloader step 4.1  NvOdmGpioGetState checkintstatusnum up 5 second, give up \n");
			return NV_FALSE;
		}
	}while(!PinStateValue);
	msleep(1);
	
	//**************************************************************************************//
	// Fifths step : Clear the Flash Key Codes
	NvU8 ClearFlashKeycommand[4];
	ClearFlashKeycommand[0] = 3;	//BLSize
	ClearFlashKeycommand[1] = 0xF7;		//BL_SetFlashKeyCodes
	ClearFlashKeycommand[2] = 0;
	ClearFlashKeycommand[3] = 0;

	if(!(AT168_Bootloader_Write(hTouch, ClearFlashKeycommand, 4)))
	{
		NvOsDebugPrintf("AT168_BurnCandoBootloader step 5.1  AT168_Bootloader_Write  ClearFlashKeycommand fail \n");
		goto RetryBurnCando;
		//return NV_FALSE;
	}
	else
	{
		AT168_PRINTF(("AT168_BurnCandoBootloader step 5.1  AT168_Bootloader_Write  ClearFlashKeycommand success \n"));
	}
	
	//wait for gpio_int change to high
	msleep(1);
	checkintstatusnum = 0;
	do
	{
		AT168_PRINTF(("AT168_BurnCandoBootloader step 5.1  NvOdmGpioGetState begin \n"));
		msleep(2);
		NvOdmGpioGetState( hTouch->hGpio, hTouch->hPinInterrupt, &PinStateValue);
		AT168_PRINTF(("AT168_BurnCandoBootloader step 5.1  NvOdmGpioGetState end \n"));
		checkintstatusnum+=2;
		if(checkintstatusnum > 5000)
		{
			NvOsDebugPrintf("AT168_BurnCandoBootloader step 5.1  NvOdmGpioGetState checkintstatusnum up 5 second, give up \n");
			return NV_FALSE;
		}
	}while(!PinStateValue);
	msleep(1);
	
	//**************************************************************************************//
	// Sixth step : Exit boot-loader
	NvU8 ExitBootloadercommand[2];
	ExitBootloadercommand[0] = 1;	//BLSize
	ExitBootloadercommand[1] = 0xF5;		//BL_ExitBootload

	if(!(AT168_Bootloader_Write(hTouch, ExitBootloadercommand, 2)))
	{
		NvOsDebugPrintf("AT168_BurnCandoBootloader step 6.1  AT168_Bootloader_Write  ExitBootloadercommand fail \n");
		goto RetryBurnCando;
		//return NV_FALSE;
	}
	else
	{
		AT168_PRINTF(("AT168_BurnCandoBootloader step 6.1  AT168_Bootloader_Write  ExitBootloadercommand success \n"));
	}
	
	//wait for gpio_int change to high
	msleep(1);
	checkintstatusnum = 0;
	do
	{
		AT168_PRINTF(("AT168_BurnCandoBootloader step 6.1  NvOdmGpioGetState begin \n"));
		msleep(2);
		NvOdmGpioGetState( hTouch->hGpio, hTouch->hPinInterrupt, &PinStateValue);
		AT168_PRINTF(("AT168_BurnCandoBootloader step 6.1  NvOdmGpioGetState end \n"));
		checkintstatusnum+=2;
		if(checkintstatusnum > 5000)
		{
			NvOsDebugPrintf("AT168_BurnCandoBootloader step 6.1  NvOdmGpioGetState checkintstatusnum up 5 second, give up \n");
			return NV_FALSE;
		}
	}while(!PinStateValue);
	msleep(5);
	
	//**************************************************************************************//
	// Seventh step : Reset TP
	NvU8 ResetTPcommand[2];
	ResetTPcommand[0] = 1;	//BLSize
	ResetTPcommand[1] = 0xFB;		//BL_ExitBootload

	nLoop = 0;
	while(nLoop<3)
	{
		if(!(AT168_Bootloader_Write(hTouch, ResetTPcommand, 2)))
		{
			NvOsDebugPrintf("AT168_BurnCandoBootloader step 7.1  AT168_Bootloader_Write  ResetTPcommand fail \n");
			//return NV_FALSE;
		}
		else
		{
			AT168_PRINTF("AT168_BurnCandoBootloader step 7.1  AT168_Bootloader_Write  ResetTPcommand success \n");
		}
		nLoop ++;
		msleep(3);
		NvOsDebugPrintf("AT168_BurnCandoBootloader step 7.1  AT168_Bootloader_Write  ResetTPcommand nLoop is %d \n", nLoop);
		if(nLoop == 3)
		{
			break;
		}
	}
	
	//**************************************************************************************//
	NvOsDebugPrintf("AT168_BurnCandoBootloader end \n");
	return NV_TRUE;
}
//For burn touchscreen bootloader
NvBool AT168_BurnBootloader(NvOdmTouchDeviceHandle hDevice)
{
	NvU8 status[4]; 
	AT168_TouchDevice* hTouch = (AT168_TouchDevice*)hDevice;
	
	//First step : get in the bootloader
	NvOdmGpioSetState(hTouch->hGpio,
	                hTouch->hPinReset,
	                NvOdmGpioPinActiveState_Low);
	msleep(5);
	NvOdmGpioSetState(hTouch->hGpio,
	                hTouch->hPinReset,
	                NvOdmGpioPinActiveState_High);
	msleep(20);	//must send bootloader I2C address after reset in 2ms-50ms

	hTouch->DeviceAddr = (NvU32)(0x5d << 1);	//change I2C  to bootloader address 0x5d
	
	if(!(AT168_Bootloader_Read(hTouch, status, 4)))
	{
		NvOsDebugPrintf("AT168_BurnBootloader --  Read status fail  maybe is Cando TS \n");
		hTouch->DeviceAddr = (NvU32)(0x5c << 1);	//change I2C back to normal 0x5c
		if(!(AT168_BurnCandoBootloader(hDevice)))
		{
			hTouch->DeviceAddr = (NvU32)(0x5c << 1);	//change I2C back to normal 0x5c
			return NV_FALSE;
		}
	}
	else
	{
		NvOsDebugPrintf("AT168_BurnBootloader --  Read status OK, now is Sintek TS \n");
		//NvOsDebugPrintf("AT168_BurnBootloader -- return the status are 0x%x 0x%x 0x%x 0x%x \n", status[0], status[1], status[2], status[3]);
		
		hTouch->DeviceAddr = (NvU32)(0x5d << 1);	//change I2C  to bootloader address 0x5d
		if(!(AT168_BurnSintexBootloader(hDevice)))
		{
			hTouch->DeviceAddr = (NvU32)(0x5c << 1);	//change I2C back to normal 0x5c
			return NV_FALSE;
		}
	}
	
	hTouch->DeviceAddr = (NvU32)(0x5c << 1);	//change I2C back to normal 0x5c
	return NV_TRUE;
}

static void AT168_GpioIsr(void *arg)
{
	AT168_TouchDevice* hTouch = (AT168_TouchDevice*)arg;

	/* Signal the touch thread to read the sample. After it is done reading the
	* sample it should re-enable the interrupt. */
	NvOdmOsSemaphoreSignal(hTouch->hIntSema);            
}

NvBool AT168_ReadCoordinate (NvOdmTouchDeviceHandle hDevice, NvOdmTouchCoordinateInfo* coord)
{
	AT168_TouchDevice* hTouch = (AT168_TouchDevice*)hDevice;

	NvU8 data[10] = {0};

	//Read once
	if(!AT168_READ(hTouch, AT168_TOUCH_NUM, data, (AT168_POS_Y1_HI - AT168_TOUCH_NUM + 1)))
	{
		NvOsDebugPrintf("NvOdmTouch_at168: Read Coord fail .\n");
		return NV_FALSE;
	}
	else
	{
		//AT168_PRINTF(("NvOdmTouch_at168:  AT168_ReadCoordinate success .\n"));
	}	

	coord->additionalInfo.Fingers = data[AT168_TOUCH_NUM];

	if((1 == coord->additionalInfo.Fingers) || (2 == coord->additionalInfo.Fingers))
	{
		coord->xcoord =
		coord->additionalInfo.multi_XYCoords[0][0] =
			(data[AT168_POS_X0_HI] << 8) | (data[AT168_POS_X0_LO]);

		coord->ycoord =
	        coord->additionalInfo.multi_XYCoords[0][1] =
			(data[AT168_POS_Y0_HI] << 8) | (data[AT168_POS_Y0_LO]);
		
		if(2 == coord->additionalInfo.Fingers)
		{
			coord->additionalInfo.multi_XYCoords[1][0] =
			(data[AT168_POS_X1_HI] << 8) | (data[AT168_POS_X1_LO]);

			coord->additionalInfo.multi_XYCoords[1][1] =
			(data[AT168_POS_Y1_HI] << 8) | (data[AT168_POS_Y1_LO]);
		}
		else if(1 == coord->additionalInfo.Fingers)
		{
			coord->additionalInfo.multi_XYCoords[1][0] = 0;//AT168_MIN_X;  //will be NvOdmTouchOrientation_H_FLIP
			coord->additionalInfo.multi_XYCoords[1][1] = 0;//AT168_MIN_Y;
		}
	}
	else  //Fingers is 0
	{
		coord->additionalInfo.Fingers = 0; //Please reset Fingers 0;

		coord->xcoord =
		coord->additionalInfo.multi_XYCoords[0][0] = 0;//AT168_MIN_X;
		coord->ycoord =
	        coord->additionalInfo.multi_XYCoords[0][1] = 0;//AT168_MIN_Y;

		coord->additionalInfo.multi_XYCoords[1][0] = 0;//AT168_MIN_X;
		coord->additionalInfo.multi_XYCoords[1][1] = 0;//AT168_MIN_Y;
	}
 	
	AT168_PRINTF(("==AT168_READ---FingerNum = %d  x[0]=%d y[0]=%d x[1]=%d y[1]=%d ===\n", 
				coord->additionalInfo.Fingers,
				coord->additionalInfo.multi_XYCoords[0][0], coord->additionalInfo.multi_XYCoords[0][1], 
				coord->additionalInfo.multi_XYCoords[1][0], coord->additionalInfo.multi_XYCoords[1][1]));

	
	//Set if NvOdmTouchSampleIgnore     //MAX fingers fit two
	coord->fingerstate = 0;		//Reset the fingerstate
	if ((coord->additionalInfo.Fingers !=0 ) && ( coord->additionalInfo.multi_XYCoords[0][0] <= 0 ||
                coord->additionalInfo.multi_XYCoords[0][0] >= AT168_Capabilities.XMaxPosition ||
                coord->additionalInfo.multi_XYCoords[0][1] <= 0 || 
                coord->additionalInfo.multi_XYCoords[0][1] >= AT168_Capabilities.YMaxPosition))
	{
		coord->fingerstate = NvOdmTouchSampleIgnore;
	}
	if ((coord->additionalInfo.Fingers ==2 ) && ( coord->additionalInfo.multi_XYCoords[1][0] <= 0 ||
                coord->additionalInfo.multi_XYCoords[1][0] >= AT168_Capabilities.XMaxPosition ||
                coord->additionalInfo.multi_XYCoords[1][1] <= 0 ||
                coord->additionalInfo.multi_XYCoords[1][1] >= AT168_Capabilities.YMaxPosition))
	{
		coord->fingerstate = NvOdmTouchSampleIgnore;
	}

	#if 0
	int i = 0;
	do
	{
		AT168_PRINTF(("NvOdmTouch_at168:  data[%d] = (0x%x)---\n", i, data[i]));
		i++;
	}while(i<10);
	#endif

	return NV_TRUE;

}

void AT168_GetCapabilities (NvOdmTouchDeviceHandle hDevice, NvOdmTouchCapabilities* pCapabilities)
{
	AT168_TouchDevice* hTouch = (AT168_TouchDevice*)hDevice;
	*pCapabilities = hTouch->Caps;
}

NvBool AT168_PowerOnOff (NvOdmTouchDeviceHandle hDevice, NvBool OnOff)
{
	AT168_PRINTF(("NvOdm Touch: AT168_PowerOnOff OnOff=%d \n", OnOff));
	return NV_TRUE;
	#if 0
	AT168_TouchDevice* hTouch = (AT168_TouchDevice*)hDevice;
	if(!OnOff)
	{
		NvOdmGpioInterruptMask(hTouch->hGpioIntr,NV_TRUE);
	}
	else
	{
		msleep(100);
		//Force reset, for some hexing touchsceeen can not boot up
		NvOdmGpioSetState(hTouch->hGpio,
	                	hTouch->hPinReset,
	                	NvOdmGpioPinActiveState_Low);
		msleep(5);
		NvOdmGpioSetState(hTouch->hGpio,
	               		hTouch->hPinReset,
	                	NvOdmGpioPinActiveState_High);
		msleep(60);

		NvOdmGpioInterruptMask(hTouch->hGpioIntr,NV_FALSE);		
	}

	return NV_TRUE;
	#endif
}

NvBool AT168_Open (NvOdmTouchDeviceHandle* hDevice)
{
	AT168_TouchDevice* hTouch;
	NvU32 i;
	NvU32 found = 0;
	NvU32 I2cInstance = 0;

	NvU32 GpioPort[2] = {0};
	NvU32 GpioPin[2] = {0};
	int GpioNum = 0;

	AT168_PRINTF(("===***NvOdm Touch: AT168_Open***===\n"));

	const NvOdmPeripheralConnectivity *pConnectivity = NULL;

	hTouch = NvOdmOsAlloc(sizeof(AT168_TouchDevice));
	if (!hTouch) return NV_FALSE;

	NvOdmOsMemset(hTouch, 0, sizeof(AT168_TouchDevice));
	/* set function pointers */
	InitOdmTouch(&hTouch->OdmTouch);
	pConnectivity = NvOdmPeripheralGetGuid(AT168_TOUCH_DEVICE_GUID);
	if (!pConnectivity)
	{
		NvOsDebugPrintf("NvOdm Touch : pConnectivity is NULL Error \n");
		goto fail;
	}
	if (pConnectivity->Class != NvOdmPeripheralClass_HCI)
	{
		NvOsDebugPrintf("NvOdm Touch : didn't find any periperal in discovery query for touch device Error \n");
		goto fail;
	}
	for (i = 0; i < pConnectivity->NumAddress; i++)
	{
		switch (pConnectivity->AddressList[i].Interface)
		{
			case NvOdmIoModule_I2c:
				hTouch->DeviceAddr = (pConnectivity->AddressList[i].Address << 1);
				I2cInstance = pConnectivity->AddressList[i].Instance;
				found |= 1;
				break;
			case NvOdmIoModule_Gpio:
				GpioPort[GpioNum] = pConnectivity->AddressList[i].Instance;
				GpioPin[GpioNum++] = pConnectivity->AddressList[i].Address;
				found |= 2;
				break;
			case NvOdmIoModule_Vdd:
				hTouch->VddId = pConnectivity->AddressList[i].Address;
				found |= 4;
				break;
			default:
				break;
		}
	}
	if ((found & 3) != 3)
	{
		NvOsDebugPrintf("NvOdm Touch : peripheral connectivity problem \n");
		goto fail;
	}

	hTouch->hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c, I2cInstance);
	if (!hTouch->hOdmI2c)
	{
		NvOsDebugPrintf("NvOdm Touch : NvOdmI2cOpen Error \n");
		goto fail;
	}
	
	hTouch->hGpio = NvOdmGpioOpen();

	if (!hTouch->hGpio)
	{
		NvOsDebugPrintf("NvOdm Touch : NvOdmGpioOpen Error \n");
		goto fail;
	}
	/**********************************************************/
	//note : 
	// Acquiring Pin Handles for all the two Gpio Pins
	// First entry is Reset (Num 0)
	// Second entry should be Interrupt (NUm 1)
	
	//reset
	hTouch->hPinReset = NvOdmGpioAcquirePinHandle(hTouch->hGpio, GpioPort[0], GpioPin[0]);
	if (!hTouch->hPinReset)
	{
		NvOsDebugPrintf("NvOdm Touch : Couldn't get GPIO hPinReset \n");
		goto fail;
	}
	NvOdmGpioSetState(hTouch->hGpio,
	                hTouch->hPinReset,
	                NvOdmGpioPinActiveState_High);

	NvOdmGpioConfig(hTouch->hGpio,
	                hTouch->hPinReset,
	                NvOdmGpioPinMode_Output);
	                
	#if 1
	//Force reset, for some hexing touchsceeen can not boot up
	NvOdmGpioSetState(hTouch->hGpio,
	                hTouch->hPinReset,
	                NvOdmGpioPinActiveState_Low);
	msleep(5);
	NvOdmGpioSetState(hTouch->hGpio,
	                hTouch->hPinReset,
	                NvOdmGpioPinActiveState_High);
	msleep(60);
	#endif

	//int
	hTouch->hPinInterrupt = NvOdmGpioAcquirePinHandle(hTouch->hGpio, GpioPort[1], GpioPin[1]);
	if (!hTouch->hPinInterrupt) {
		NvOsDebugPrintf("NvOdm Touch : Couldn't get GPIO hPinInterrupt \n");
		goto fail;
	}
	NvOdmGpioConfig(hTouch->hGpio, hTouch->hPinInterrupt, NvOdmGpioPinMode_InputData);

	/**********************************************************/
	/* set default capabilities */
	NvOdmOsMemcpy(&hTouch->Caps, &AT168_Capabilities, sizeof(NvOdmTouchCapabilities));
	/* set default I2C speed */
	hTouch->I2cClockSpeedKHz = AT168_I2C_SPEED_KHZ;
	/**********************************************************/
	#if Calibration_in_Boot_enable
	//set the SPECOP reg
	if (!AT168_WRITE(hTouch, AT168_SPECOP, AT168_SPECOP_CALIBRATION_VALUE))//0x03
	        goto fail;
	#endif	
	/**********************************************************/
	NvU8 InitData[8] = {0};
	if(!AT168_READ(hTouch, AT168_XMAX_LO, InitData, (AT168_VERSION_PROTOCOL - AT168_XMAX_LO + 1)))
	{
		NvOsDebugPrintf("NvOdmTouch_at168:  AT168_Open AT168_READ InitData fail .\n");
		//return NV_FALSE;
	}

	#if 0
	int j = 0;
	do
	{
		AT168_PRINTF(("NvOdmTouch_at168: InitData[%d] = 0x%x---\n", j, InitData[j]));
		j++;

	}while(j < 8);
	#endif
	
	//Set the Max and Min position
	AT168_Capabilities.XMinPosition = 0; //AT168_MIN_X;
	AT168_Capabilities.YMinPosition = 0; //AT168_MIN_Y;
	AT168_Capabilities.XMaxPosition = ((InitData[1] << 8) | (InitData[0])); //AT168_MAX_X;
	AT168_Capabilities.YMaxPosition = ((InitData[3] << 8) | (InitData[2])); //AT168_MAX_Y;
	
	//Set the Version
	AT168_Capabilities.Version = ((InitData[4] << 24) | (InitData[5] << 16) | (InitData[6] << 8) | (InitData[7]) );


	#if 1	 //for old version of hexing touchscreen , when hexing FW update, mask them	
	if((AT168_Capabilities.XMaxPosition == 1024) && (AT168_Capabilities.YMaxPosition == 600))	
	{		
		NvOsDebugPrintf("NvOdmTouch_at168: ---Sintek touchscreen--- .\n");	
	}	
	else if((AT168_Capabilities.XMaxPosition == 4096) && (AT168_Capabilities.YMaxPosition == 4096))	
	{		
		NvOsDebugPrintf("NvOdmTouch_at168: ---Cando touchscreen--- .\n");	
	}	
	else if((AT168_Capabilities.XMaxPosition == 4096) && (AT168_Capabilities.YMaxPosition == 2560))
	{		
		NvOsDebugPrintf("NvOdmTouch_at168: ---UniDisplay touchscreen--- .\n");	
	}	
	else	
	{		
		NvOsDebugPrintf("NvOdmTouch_at168: ---Maybe Old Sintek FW touchscreen--- .\n");		
		AT168_Capabilities.XMaxPosition = 1024;		
		AT168_Capabilities.YMaxPosition = 600;		
		AT168_Capabilities.Version = 0x02000012;	
	}	
	#endif
	
	AT168_PRINTF(("NvOdmTouch_at168: now xMAX is %d   yMAx is %d.\n", AT168_Capabilities.XMaxPosition, AT168_Capabilities.YMaxPosition));
	/* change the touchscreen capabilities */
	NvOdmOsMemcpy(&hTouch->Caps, &AT168_Capabilities, sizeof(NvOdmTouchCapabilities));
	/**********************************************************/
	*hDevice = &hTouch->OdmTouch;
	
	NvOsDebugPrintf("===NvOdmTouch_at168: AT168_Open success===\n");	
	return NV_TRUE;

 fail:
	AT168_Close(&hTouch->OdmTouch);
	return NV_FALSE;
}


NvBool AT168_EnableInterrupt (NvOdmTouchDeviceHandle hDevice, NvOdmOsSemaphoreHandle hIntSema)
{
	AT168_TouchDevice* hTouch = (AT168_TouchDevice*)hDevice;

	if (NvOdmGpioInterruptRegister(hTouch->hGpio, &hTouch->hGpioIntr,
        hTouch->hPinInterrupt, NvOdmGpioPinMode_InputInterruptLow, AT168_GpioIsr,
        (void*)hTouch, AT168_DEBOUNCE_TIME_MS) == NV_FALSE)
    	{
        	NvOsDebugPrintf("===Nvodm Touch:AT168_EnableInterrupt NvOdmGpioInterruptRegister fail!=== \n");
		return NV_FALSE;
    	}
	hTouch->hIntSema = hIntSema;
	return NV_TRUE;
}


NvBool AT168_HandleInterrupt(NvOdmTouchDeviceHandle hDevice)
{
	NvU32 PinStateValue;
	AT168_TouchDevice* hTouch = (AT168_TouchDevice*)hDevice;
	
	NvOdmGpioGetState( hTouch->hGpio, hTouch->hPinInterrupt, &PinStateValue);
	if (!PinStateValue) {
		return NV_FALSE;
	}
	else {
		NvOdmGpioInterruptDone(hTouch->hGpioIntr);
	}
	return NV_TRUE;

}


NvBool AT168_PowerControl (NvOdmTouchDeviceHandle hDevice, NvOdmTouchPowerModeType mode)
{
    switch (mode) 
	{
		case NvOdmTouch_PowerMode_0:
		case NvOdmTouch_PowerMode_1:
		case NvOdmTouch_PowerMode_2:
		case NvOdmTouch_PowerMode_3:
			break;
		default:
			break;
	}
	return NV_TRUE;
}


void AT168_Close (NvOdmTouchDeviceHandle hDevice)
{
	AT168_TouchDevice* hTouch = (AT168_TouchDevice*)hDevice;

	if (!hTouch) return;

	if (hTouch->hGpio)
	{
		if (hTouch->hPinInterrupt)
		{
			if (hTouch->hGpioIntr)
			NvOdmGpioInterruptUnregister(hTouch->hGpio, hTouch->hPinInterrupt, hTouch->hGpioIntr);
			NvOdmGpioReleasePinHandle(hTouch->hGpio, hTouch->hPinInterrupt);
		}
		NvOdmGpioClose(hTouch->hGpio);
	}

	if (hTouch->hOdmI2c)
		NvOdmI2cClose(hTouch->hOdmI2c);

	NvOdmOsFree(hTouch);
}

