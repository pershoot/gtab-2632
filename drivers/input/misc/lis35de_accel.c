/*
 * File        : drivers/input/misc/lis35de.c
 * Description : Driver for lis35de accelemerater.
 *
 * By          : LiuZheng (xmlz@malata.com)	2010/04/24
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>

#include <linux/platform_device.h>
//#include <linux/tegra_devices.h>

#include <nvodm_services.h>
#include "lis35de_accel.h"

#define FLIP_X				1
#define FLIP_Y				2
#define FLIP_Z				4

#define TAG					"LIS35DE:  "

struct lis35de_dev {
	struct input_dev *input_dev;
	
	unsigned int i2c_instance;
	unsigned int i2c_address;
	NvOdmServicesI2cHandle i2c;

	struct delayed_work update_work;	
	unsigned int update_interval;
	unsigned int enable;

	int x;
	int y;
	int z;

	int x_calibrate;
	int y_calibrate;
	int z_calibrate;

	struct semaphore sem;

	unsigned int flag;
};

static struct lis35de_dev s_lis35de_dev;

static ssize_t 
lis35de_read_sysfs_control(struct device *device, struct device_attribute *attr, char *buffer);
static ssize_t 
lis35de_read_sysfs_data(struct device *device, struct device_attribute *attr, char *buffer);
static ssize_t 
lis35de_read_sysfs_calibrate(struct device *device, struct device_attribute *attr, char *buffer);
static ssize_t 
lis35de_read_sysfs_flip(struct device *device, struct device_attribute *attr, char *buffer);

static ssize_t 
lis35de_write_sysfs_control(struct device *device, struct device_attribute *attr, const char *buffer, size_t count);
static ssize_t 
lis35de_write_sysfs_calibrate(struct device *device, struct device_attribute *attr, const char *buffer, size_t count);
static ssize_t
lis35de_write_sysfs_flip(struct device *device, struct device_attribute *attr, const char *buffer, size_t count);

#if (__LIS35DE_GENERIC_DEBUG__)
static ssize_t 
lis35de_read_sysfs_debug(struct device *device, struct device_attribute *attr, char *buffer);
static ssize_t
lis35de_write_sysfs_debug(struct device *device, struct device_attribute *attr, const char *buffer, size_t count);
#endif

static DEVICE_ATTR(delay, 0777, lis35de_read_sysfs_control, lis35de_write_sysfs_control);
static DEVICE_ATTR(enable, 0777, lis35de_read_sysfs_control,lis35de_write_sysfs_control);
static DEVICE_ATTR(x, 0777, lis35de_read_sysfs_data, NULL);
static DEVICE_ATTR(y, 0777, lis35de_read_sysfs_data, NULL);
static DEVICE_ATTR(z, 0777, lis35de_read_sysfs_data, NULL);
static DEVICE_ATTR(x_calibrate, 0777, lis35de_read_sysfs_calibrate, lis35de_write_sysfs_calibrate);
static DEVICE_ATTR(y_calibrate, 0777, lis35de_read_sysfs_calibrate, lis35de_write_sysfs_calibrate);
static DEVICE_ATTR(z_calibrate, 0777, lis35de_read_sysfs_calibrate, lis35de_write_sysfs_calibrate);
static DEVICE_ATTR(x_flip, 0777, lis35de_read_sysfs_flip, lis35de_write_sysfs_flip);
static DEVICE_ATTR(y_flip, 0777, lis35de_read_sysfs_flip, lis35de_write_sysfs_flip);
static DEVICE_ATTR(z_flip, 0777, lis35de_read_sysfs_flip, lis35de_write_sysfs_flip);
#if (__LIS35DE_GENERIC_DEBUG__)
static DEVICE_ATTR(debug, 0777, lis35de_read_sysfs_debug, lis35de_write_sysfs_debug);
#endif

static int lis35de_i2c_read(struct lis35de_dev *dev, unsigned char mem_address, unsigned char *buffer, unsigned int num_read)
{
	NvOdmI2cStatus I2cStatus;
	NvOdmI2cTransactionInfo I2cTransactionInfo[2];

	mem_address |= 0x80;

	I2cTransactionInfo[0].Flags = NVODM_I2C_IS_WRITE | NVODM_I2C_USE_REPEATED_START;
	I2cTransactionInfo[0].Address = dev->i2c_address;
	I2cTransactionInfo[0].Buf = &mem_address;
	I2cTransactionInfo[0].NumBytes = 1;
	I2cTransactionInfo[1].Flags = 0;
	I2cTransactionInfo[1].Address = dev->i2c_address|0x01;
	I2cTransactionInfo[1].Buf = buffer;
	I2cTransactionInfo[1].NumBytes = num_read;

	I2cStatus = NvOdmI2cTransaction(dev->i2c, I2cTransactionInfo, 2, LIS35DE_I2C_SPEED_KHZ, LIS35DE_I2C_TIMEOUT_MS);
	if (I2cStatus != NvOdmI2cStatus_Success) {
		logd("lis35de_i2c_read problem, err = %d\r\n", I2cStatus);
		return -1;
	}
	return 0;
}

#define LIS35DE_MAX_WRITE_LENGTH			8
static int lis35de_i2c_write(struct lis35de_dev *dev, unsigned char mem_address, unsigned char *buffer, unsigned int num)
{
	unsigned char i;
	unsigned char write_buffer[LIS35DE_MAX_WRITE_LENGTH+1];
	NvOdmI2cStatus I2cStatus;
	NvOdmI2cTransactionInfo I2cTransactionInfo;

	write_buffer[0] = mem_address | 0x80;
	for (i = 0; i < num; i++) {
		write_buffer[i+1] = *buffer++;
	}
	I2cTransactionInfo.Flags = NVODM_I2C_IS_WRITE;
	I2cTransactionInfo.Address = dev->i2c_address;
	I2cTransactionInfo.Buf = write_buffer;
	I2cTransactionInfo.NumBytes = num+1;
	I2cStatus = NvOdmI2cTransaction(dev->i2c, &I2cTransactionInfo, 1, LIS35DE_I2C_SPEED_KHZ, LIS35DE_I2C_TIMEOUT_MS);
	if (I2cStatus != NvOdmI2cStatus_Success) {
		logd("lis35de_i2c_write problem, err = %d\r\n", I2cStatus);
		return -1;
	}
	return 0;
}

static inline int lis35de_i2c_read_8(struct lis35de_dev *dev, unsigned char mem_address, unsigned char *buffer)
{
	return lis35de_i2c_read(dev, mem_address, buffer, 1);
}

static inline int lis35de_i2c_write_8(struct lis35de_dev *dev, unsigned char mem_address, unsigned char buffer)
{
	return lis35de_i2c_write(dev, mem_address, &buffer, 1);
}

static inline int lis35de_power_onoff(struct lis35de_dev *dev, bool onoff)
{
	unsigned char data = onoff ? 0x47 : 0;
	return lis35de_i2c_write_8(dev, LIS35DE_REG_CTRL_1, data );
}

static int lis35de_get_raw_xyz(struct lis35de_dev *dev, int *x, int *y, int *z)
{
	unsigned char cx, cy, cz;
	signed char sx, sy, sz;
	if (lis35de_i2c_read_8(dev, LIS35DE_REG_OUTX, &cx) 
		|| lis35de_i2c_read_8(dev, LIS35DE_REG_OUTY, &cy)
		|| lis35de_i2c_read_8(dev, LIS35DE_REG_OUTZ, &cz)) 
	{
		return -1;
	}
	sx = (char)cx; sy = (char)cy; sz = (char)cz;

	//sz -= 44;	

	*x = sx * LIS35DE_PRECISION_MG;
	*y = sy * LIS35DE_PRECISION_MG;
	*z = sz * LIS35DE_PRECISION_MG;

	logd("accelerometer raw data: x=%d, y=%d, z=%d\r\n", *x, *y, *z);

	return 0;
}

static int
lis35de_set_enable(struct lis35de_dev *dev, int enable)
{
	if (dev->enable == enable) {
		return 0;
	}
	if(NULL==dev->input_dev) return -EINVAL;
	
	logd(TAG "lis35de_set_ebale = %d", enable);
	down(&dev->sem);
	if (enable) {
		/* enable device and start work */
		if (lis35de_i2c_write_8(dev, LIS35DE_REG_CTRL_1, 0x47 )) {
			up(&dev->sem);
			goto exit_failed;
		}
		schedule_delayed_work(&dev->update_work, msecs_to_jiffies(dev->update_interval));
		dev->enable = true;
	} else {
		/* try to poweroff the device and cancel delayed work sync */
		lis35de_i2c_write_8(dev, LIS35DE_REG_CTRL_1, 0);
		cancel_delayed_work_sync(&dev->update_work);
		dev->enable = false;
	}

	up(&dev->sem);
	return 0;
exit_failed:
	return -EINVAL;
}

static int
lis35de_set_delay(struct lis35de_dev *dev, int delay)
{
	/* minmum 20ms */
	if (delay < 20) {
		delay = 20;
	}
	dev->update_interval = delay;
	return 0;
}

static ssize_t 
lis35de_read_sysfs_control(struct device *device, struct device_attribute *attr, char *buffer)
{
	struct lis35de_dev *dev = &s_lis35de_dev;

	if (attr == &dev_attr_delay) {
		return sprintf(buffer, "%d", dev->update_interval);
	} else if (attr == &dev_attr_enable) {	
		return sprintf(buffer, ((dev->enable) ? "1" : "0"));
	}

	return 0;
}

static ssize_t
lis35de_write_sysfs_control(struct device *device, struct device_attribute *attr, const char *buffer, size_t count)
{	
	int tol;
	struct lis35de_dev *dev = &s_lis35de_dev;

	logd(TAG "write_sysfs_control\r\n");

	if (attr == &dev_attr_delay) {
		logd(TAG "write sysfs_delay\r\n");
		tol = simple_strtol(buffer, NULL, 10);
		lis35de_set_delay(dev, tol);
	} else if (attr == &dev_attr_enable) {
		logd(TAG "write sysfs_enable\r\n");
		tol = (simple_strtol(buffer, NULL, 10)) ? 1 : 0;
		lis35de_set_enable(dev, tol);
	}

	return count;
}

static ssize_t
lis35de_read_sysfs_data(struct device *device, struct device_attribute *attr, char *buffer)
{
	struct lis35de_dev *dev = &s_lis35de_dev;
	
	if (attr == &dev_attr_x)
		return sprintf(buffer, "%d", dev->x);
	else if (attr == &dev_attr_y)
		return sprintf(buffer, "%d", dev->y);
	else if (attr == &dev_attr_z)
		return sprintf(buffer, "%d", dev->z);

	return 0;
}

static ssize_t
lis35de_read_sysfs_calibrate(struct device *device, struct device_attribute *attr, char *buffer)
{
	struct lis35de_dev *dev = &s_lis35de_dev;
	
	if (attr == &dev_attr_x_calibrate)
		return sprintf(buffer, "%d", dev->x_calibrate);
	else if (attr == &dev_attr_y_calibrate)
		return sprintf(buffer, "%d", dev->y_calibrate);
	else if (attr == &dev_attr_z_calibrate)
		return sprintf(buffer, "%d", dev->z_calibrate);

	return 0;
}

static ssize_t 
lis35de_write_sysfs_calibrate(struct device *device, struct device_attribute *attr, const char *buffer, size_t count)
{
	struct lis35de_dev *dev = &s_lis35de_dev;

	if (attr == &dev_attr_x_calibrate)
		dev->x_calibrate = simple_strtol(buffer, NULL, 10);
	else if (attr == &dev_attr_y_calibrate)
		dev->y_calibrate = simple_strtol(buffer, NULL, 10);
	else if (attr == &dev_attr_z_calibrate)
		dev->z_calibrate = simple_strtol(buffer, NULL, 10);
	
	return count;
}

static ssize_t
lis35de_read_sysfs_flip(struct device *device, struct device_attribute *attr, char *buffer)
{	
	struct lis35de_dev *dev = &s_lis35de_dev;

	if (attr == &dev_attr_x_flip)
		return sprintf(buffer, "%d", (dev->flag&FLIP_X)?1:0);
	else if (attr == &dev_attr_y_flip)
		return sprintf(buffer, "%d", (dev->flag&FLIP_Y)?1:0);
	else if (attr == &dev_attr_z_flip)
		return sprintf(buffer, "%d", (dev->flag&FLIP_Z)?1:0);

	return 0;
}

static ssize_t
lis35de_write_sysfs_flip(struct device *device, struct device_attribute *attr, const char *buffer, size_t count)
{
	struct lis35de_dev *dev = &s_lis35de_dev;

	if (attr == &dev_attr_x_flip) {
		if (simple_strtol(buffer, NULL , 10))
			dev->flag |= FLIP_X;
	} else if (attr == &dev_attr_y_flip) {
		if (simple_strtol(buffer, NULL , 10))
			dev->flag |= FLIP_X;
	} else if (attr == &dev_attr_z_flip) {
		if (simple_strtol(buffer, NULL , 10))
			dev->flag |= FLIP_X;
	}
	return count;
}

#if (__LIS35DE_GENERIC_DEBUG__)

static ssize_t
lis35de_read_sysfs_debug(struct device *device, struct device_attribute *attr, char *buffer)
{		
	unsigned char *useage = "for sensor debug";
	memcpy(buffer, useage, strlen(useage)+1);

	return strlen(useage)+1;
}

static ssize_t
lis35de_write_sysfs_debug(struct device *device, struct device_attribute *attr, const char *buffer, size_t count)
{
	return count;
}

#endif

static int lis35de_add_sysfs_entry(struct device *dev)
{
	device_create_file(dev, &dev_attr_delay);
	device_create_file(dev, &dev_attr_enable);
	device_create_file(dev, &dev_attr_x);
	device_create_file(dev, &dev_attr_y);
	device_create_file(dev, &dev_attr_z);
	device_create_file(dev, &dev_attr_x_calibrate);
	device_create_file(dev, &dev_attr_y_calibrate);
	device_create_file(dev, &dev_attr_z_calibrate);
	device_create_file(dev, &dev_attr_x_flip);
	device_create_file(dev, &dev_attr_y_flip);
	device_create_file(dev, &dev_attr_z_flip);
#if (__LIS35DE_GENERIC_DEBUG__)
	device_create_file(dev, &dev_attr_debug);
#endif

	return 0;
}

static void lis35de_remove_sysfs_entry(struct device *dev)
{
	device_remove_file(dev, &dev_attr_delay);
	device_remove_file(dev, &dev_attr_enable);
	device_remove_file(dev, &dev_attr_x);
	device_remove_file(dev, &dev_attr_y);
	device_remove_file(dev, &dev_attr_z);
	device_remove_file(dev, &dev_attr_x_calibrate);
	device_remove_file(dev, &dev_attr_z_calibrate);
#if (__LIS35DE_GENERIC_DEBUG__)
	device_remove_file(dev, &dev_attr_debug);
#endif
}

static void lis35de_update_work_func(struct work_struct *work)
{
	int x, y, z;
	struct lis35de_dev *dev = &s_lis35de_dev;
	
	if (!lis35de_get_raw_xyz(dev, &x, &y, &z)) {
		if (x != dev->x || y != dev->y || z != dev->z) {
			dev->x = ((dev->flag & FLIP_X) ? -x : x) + dev->x_calibrate;
			dev->y = ((dev->flag & FLIP_Y) ? -y : y) + dev->y_calibrate;
			dev->z = ((dev->flag & FLIP_Z) ? -z : z) + dev->z_calibrate;
		
			logd(TAG "report x=%d, y=%d, z=%d\r\n", dev->x, dev->y, dev->z);	

			input_report_abs(dev->input_dev, ABS_X, -dev->x);	// -
			input_report_abs(dev->input_dev, ABS_Y, -dev->y);	// -
			input_report_abs(dev->input_dev, ABS_Z, dev->z);
			input_sync(dev->input_dev);
		}
	}

	/* check if the work is still run */
	if (dev->enable) {
		schedule_delayed_work(&dev->update_work, msecs_to_jiffies(dev->update_interval));
	}
}


static int lis35de_probe(struct platform_device *pdev)
{
	struct lis35de_dev *dev;

	{//disable interrupt gpio, set to input, or maybe damage g_sensor
		int gpio_int=8*('j'-'a') + 0;
		int ret = -ENODEV;
		ret = gpio_request(gpio_int, "lis35de_int");
        	if (ret < 0)
        	{
            		logd("lis35de_probe request lis35de_int gpio fail\r\n");
        	}
        	else
        	{
	        	ret = gpio_direction_input(gpio_int);
	        	if (ret < 0)
	        	{
		            	logd("lis35de_probe request lis35de_int gpio fail 1\r\n");
	        	}
		}
	}

	dev = &s_lis35de_dev;
	memset(dev, 0, sizeof(struct lis35de_dev));

	dev->i2c_instance = LIS35DE_I2C_INSTANCE;
	dev->i2c_address = LIS35DE_I2C_ADDRESS;
	dev->update_interval = LIS35DE_UPDATE_INTERVAL;
	dev->flag = FLIP_X | FLIP_Y;
	dev->i2c = NvOdmI2cOpen(NvOdmIoModule_I2c, dev->i2c_instance);
	if (!dev->i2c) {
		goto failed_open_i2c;
	}
	
	dev->input_dev = input_allocate_device();
	if (dev->input_dev == NULL) {
		goto failed_alloc_input;
	}
	set_bit(EV_SYN, dev->input_dev->evbit);
	set_bit(EV_KEY, dev->input_dev->evbit);
	set_bit(EV_ABS, dev->input_dev->evbit);
	input_set_abs_params(dev->input_dev, ABS_X, -128, 128, 0, 0);
	input_set_abs_params(dev->input_dev, ABS_Y, -128, 128, 0, 0);
	input_set_abs_params(dev->input_dev, ABS_Z, -128, 128, 0, 0);
	dev->input_dev->name = "accelerometer";
	if (input_register_device(dev->input_dev))  {
		goto failed_register_input;
	}
	
	init_MUTEX(&dev->sem);

	/* init delayed work */
	INIT_DELAYED_WORK(&dev->update_work, lis35de_update_work_func);

	/* add sysfs */
	lis35de_add_sysfs_entry(&pdev->dev);
	printk("lis35de_probe success\r\n");
	return 0;
failed_register_input:
	input_free_device(dev->input_dev);
failed_alloc_input:
	NvOdmI2cClose(dev->i2c);
failed_open_i2c:
	//kfree(dev);
	dev->input_dev=NULL;
	//logd("lis35de_probe failed\r\n");
	printk("lis35de_probe success\r\n");
	return -EINVAL;
}

static int lis35de_remove(struct platform_device *pdev)
{
	struct lis35de_dev *accelerometer;

	accelerometer = (struct lis35de_dev*) platform_get_drvdata(pdev);
	if (!accelerometer) {
		return -1;
	}

	return 0;
}

#if 0
static int lis35de_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct lis35de_dev *accelerometer;

	accelerometer = (struct lis35de_dev *)platform_get_drvdata(pdev);
	if (!accelerometer) {
		return -1;
	}
	accelerometer->power_off = true;

	return 0;
}

static int lis35de_resume(struct platform_device *pdev)
{
	struct lis35de_dev *accelerometer;
	accelerometer = (struct lis35de_dev *)platform_get_drvdata(pdev);
	
	if (!accelerometer) {
		return -1;
	}
	accelerometer->power_off = false;

	return 0;
}
#endif

static struct platform_driver lis35de_driver = {
	.probe = lis35de_probe,
	.remove = lis35de_remove,
#if 0
	.suspend = lis35de_suspend,
	.resume = lis35de_resume,
#endif
	.driver = {
		.name = "accelerometer",
		.owner = THIS_MODULE,
	},
};

static int lis35de_init(void)
{
	return platform_driver_register(&lis35de_driver);
}

static void lis35de_exit(void)
{
	platform_driver_unregister(&lis35de_driver);
}

module_init(lis35de_init);
module_exit(lis35de_exit);
_
MODULE_DESCRIPTION("SMBA1101 accelerometer for lis35de");
