/*
 * File        : kernel/driver/input/misc
 * Description : Driver for isl29023 ambient light sensor driver for android system.
 * Athor       : LiuZheng <xmlz@malata.com>
 * Date        : 2010/06/03
 */

#ifndef __ISL29023_H_INCLUDED__
#define __ISL29023_H_INCLUDED__

#define __ISL29023_GENERIC_DEBUG__			0


/* 
 * Debug log
 */
#if (__ISL29023_GENERIC_DEBUG__)
#define logd(x...)		do { printk(x); } while(0) 
#else
#define logd(x...)		do {} while(0)
#endif

/*
 * I2c parameter
 */
#define ISL29023_I2C_INSTANCE				0
#define ISL29023_I2C_ADDRESS				(0x88)	
#define ISL29023_I2C_SPEED_KHZ				(100)
#define ISL29023_I2C_TIMEOUT_MS				(1000)

#if (__ISL29023_GENERIC_DEBUG__)
#define ISL29023_UPDATE_INTERVAL			(300)
#else
#define ISL29023_UPDATE_INTERVAL			(300)
#endif

/*
 * Register
 */
#define ISL29023_REG_COMMAND_I				(0x00)
#define ISL29023_REG_COMMAND_II				(0x01)
#define ISL29023_REG_DATA_LSB				(0x02)
#define ISL29023_REG_DATA_MSB				(0x03)
#define ISL29023_REG_INT_LT_LSB				(0x04)
#define ISL29023_REG_INT_LT_MSB				(0x05)
#define ISL29023_REG_INT_HT_LSB				(0x06)
#define ISL29023_REG_INT_HT_MSB				(0x07)

#endif
