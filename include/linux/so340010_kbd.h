/*
* File        : kernel/driver/input/keyboard
* Description : Driver for so340010.kbd touch button driver for android system.
* Athor       : Conlin <xmln@malata.com>
* Date        : 2011/01/09
*/

#ifndef __SO340010_KBD_H_INCLUDED__

#define __SO340010_KBD_H_INCLUDED__

#define __SO340010_GENERIC_DEBUG__        0
#define __I2C_SNAG_DETECTED__           0

#define TAG             "SO340010: "

#if (__SO340010_GENERIC_DEBUG__)
#define logd(x...)      do { printk(x); } while(0)
#else
#define logd(x...)      do {} while(0)
#endif

#define SO340010_I2C_INSTANCE         0
#define SO340010_I2C_ADDRESS            0x58


#if (defined(CONFIG_7265C_V20)||defined(CONFIG_7323C_V21)||defined(CONFIG_7332C_V21)||defined(CONFIG_7113C_V10))
#define SO340010_I2C_SPEED          200
#elif (defined(CONFIG_7373C_V20))
#define SO340010_I2C_SPEED          400
#else
#define SO340010_I2C_SPEED          100
#endif

#define SO340010_I2C_TIMEOUT            NV_WAIT_INFINITE

struct so340010_kbd_platform_data{
	int i2c_instance;
	int i2c_address;
	int i2c_speed;
	int i2c_timeout;
};

#endif
