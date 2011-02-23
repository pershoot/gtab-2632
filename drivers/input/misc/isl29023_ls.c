/*
 * File : /kernel/driver/input/misc/isl29023_ls.c
 * Description : ISL29023 android light sensor driver implement.
 * Author : LiuZheng <xmlz@malata.com>
 * Date : 2010/06/02
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include "nvodm_services.h"
#include "isl29023_ls.h"

/*
 * ISL29023_REG_COMMAND_I command
 */
#define ISL29023_POWER_DOWN					(0x00)
#define ISL29023_ALS_CONTINUE				(0xA0)

/*
 * ISL29023_REG_COMMAND_II command
 * with a Rext = 499KO
 *
 * B3-B2 : Resolution 	
 * 	00 = 2^16, 01 = 2^12
 *	10 = 2^8,  11 = 2^4
 * B1-B0 : Range 
 * 	00 = 1000,  01 = 4000 
 * 	10 = 16000, 11 = 64000
 * 
 * ISL29023_MODE_RANGE: 		resolution=2^16 range=64000
 * ISL29023_MODE_RESOLUTION:    resolution=2^16 range=1000
 *
 */
#define ISL29023_MODE_RANGE					(0x03)
#define ISL29023_MODE_RESOLUTION			(0)

#define ISL29023_R2M_RANGE					(64000*1000/65536)
#define ISL29023_R2M_RESOLUTION				(1000*1000/65536)

/*
 * ACCURACY TYPE
 */
#define ISL29023_ACCURACY_HIGH				1
#define ISL29023_ACCURACY_MEDIUM			2
#define ISL29023_ACCURACY_LOW				3

#define ACCURACY_LEVEL						7

#define ISL29023_DEFAULT_ACCURACY			ISL29023_ACCURACY_MEDIUM

/*
 * LUX table,  in mLux unit.
 */
static int accuracy_lux_table[ACCURACY_LEVEL] = {
	100,		/* Level 0: 0 ~ 100 mLUX */
	1000,		/* Level 1: 100 ~ 1000 mLUX */
	10000,		/* Level 2: 1000 ~ 10000 mLUX */
	100000, 	/* Level 3: 10000 ~ 100000 mLUX */
	1000000,	/* Level 4: 100000 ~ 1000000 */
	10000000,	/* Level 5: 1000000 ~ 1000000 */
	100000000, 	/* Level 6: 10000000 ~ ... */
};

static int high_resolution_table[ACCURACY_LEVEL] = {
	1, 		
	10,
	10, 
	50,
	100, 
	500,
	1000
};

static int medium_resolution_table[ACCURACY_LEVEL] = {
	10,
	50,
	500,
	1000,
	2000,
	5000,
	10000
};

static int low_resolution_table[ACCURACY_LEVEL] = {
	20,
	100,
	2000, 
	10000,
	20000,
	50000,
	100000
};

struct isl29023_dev
{
	unsigned int i2c_instance;
	unsigned int i2c_address;
	unsigned int i2c_timeout;	// ms
	unsigned int i2c_speed;		// KHZ
	NvOdmServicesI2cHandle i2c;

	struct semaphore sem;
	
	struct delayed_work update_work;
	unsigned int update_interval;
	unsigned int enable;
	unsigned int raw_mlux;
	unsigned int mlux;
	struct input_dev *input_dev;
	unsigned int mode;

	int accuracy;
};

struct isl29023_dev s_isl29023_dev;

int inline rawdata_to_mlux(struct isl29023_dev *dev, unsigned int rawdata, unsigned int *mlux)
{
	switch (dev->mode) {
		case ISL29023_MODE_RANGE:
			*mlux = rawdata * ISL29023_R2M_RANGE;
			break;
		case ISL29023_MODE_RESOLUTION:
			*mlux = rawdata * ISL29023_R2M_RESOLUTION;
			break;
		default:
			logd("rawdata_to_mlux: invalid mode");
			return -1;
	}

	return 0;
}

static int isl29023_i2c_write_byte(struct isl29023_dev *dev, unsigned char cmd, unsigned char data)
{
	unsigned char buffer[2];
	NvOdmI2cStatus I2cStatus;
	NvOdmI2cTransactionInfo TransactionInfo;

	buffer[0] = cmd;
	buffer[1] = data;

	TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
	TransactionInfo.Address = dev->i2c_address; 
	TransactionInfo.Buf = buffer;
	TransactionInfo.NumBytes = 2;

	I2cStatus = NvOdmI2cTransaction(dev->i2c, &TransactionInfo, 
			1, dev->i2c_speed, dev->i2c_timeout);

	if (I2cStatus != NvOdmI2cStatus_Success) {	
		logd ("ISL29023: isl29023_i2c_write_byte failed(%d). i2c_address=%d\r\n", I2cStatus, dev->i2c_address);
		return -EINVAL;
	}
	return 0;
}

static int isl29023_i2c_read_byte(struct isl29023_dev *dev, unsigned char cmd, unsigned char *data)
{
	NvOdmI2cStatus I2cStatus;
	NvOdmI2cTransactionInfo TransactionInfo[2];

	TransactionInfo[0].Flags = NVODM_I2C_IS_WRITE | NVODM_I2C_USE_REPEATED_START;
	TransactionInfo[0].Address = dev->i2c_address;
	TransactionInfo[0].Buf = &cmd;
	TransactionInfo[0].NumBytes = 1;

	TransactionInfo[1].Flags = 0;
	TransactionInfo[1].Address = dev->i2c_address | 0x1;
	TransactionInfo[1].Buf = data;
	TransactionInfo[1].NumBytes = 1;

	I2cStatus = NvOdmI2cTransaction(dev->i2c, TransactionInfo, 
			2, dev->i2c_speed, dev->i2c_timeout);

	if (I2cStatus != NvOdmI2cStatus_Success) {
		logd ("ISL29023: isl29023_i2c_read_byte failed(%d). i2c_address=%d\r\n", I2cStatus, dev->i2c_address);
		return -EINVAL;
	}
	return 0;
}

static int isl29023_dump(struct isl29023_dev *dev)
{
	int i;
	unsigned char buffer;

	logd("-----------------------------------------------\r\n");
	for (i = 0; i < 8; i++) {
		isl29023_i2c_read_byte(dev, i, &buffer);
		logd ("0x%0x = 0x%0x\r\n", i, buffer);
	}
	logd("-----------------------------------------------\r\n");
}

#if 0
static int isl29023_i2c_receive_word(struct isl29023_dev *dev, unsigned short *data)
{
	unsigned char buffer[2];
	NvOdmI2cStatus I2cStatus;
	NvOdmI2cTransactionInfo TransactionInfo;

	TransactionInfo.Flags = 0;
	TransactionInfo.Address = dev->i2c_address;
	TransactionInfo.Buf = buffer;
	TransactionInfo.NumBytes = 2;

	I2cStatus = NvOdmI2cTransaction(dev->i2c, &TransactionInfo,
			1, dev->i2c_speed, dev->i2c_timeout);

	if (I2cStatus != NvOdmI2cStatus_Success) {
		return -EINVAL;
	}
	
	*data = buffer[0] + (buffer[1] << 8);
	return 0;
}

static int isl29023_i2c_send_byte(struct isl29023_dev *dev, unsigned char data)
{
	NvOdmI2cStatus I2cStatus;
	NvOdmI2cTransactionInfo TransactionInfo;

	TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
	TransactionInfo.Address = dev->i2c_address;
	TransactionInfo.Buf = &data;
	TransactionInfo.NumBytes = 1;

	I2cStatus = NvOdmI2cTransaction(dev->i2c, &TransactionInfo, 
			1, dev->i2c_speed, dev->i2c_timeout);
	
	if (I2cStatus != NvOdmI2cStatus_Success) {
		return -EINVAL;
	}
	
	return 0;
}
#endif

static int inline isl29023_set_mode(struct isl29023_dev *dev, unsigned int mode)
{
	if (dev->mode == mode) {
		return 0;
	}
	
	if (isl29023_i2c_write_byte(dev, ISL29023_REG_COMMAND_II, mode)) {
		return -EINVAL;
	}
	dev->mode = mode;

	if (mode == ISL29023_MODE_RANGE)
		logd ("ISL29023 : isl29023_set_mode to %s", 
				(mode == ISL29023_MODE_RANGE) ? "RANGE" 
				: ((mode == ISL29023_MODE_RESOLUTION) ? "RESOLUTION" : "OTHER"));
	return 0;
}

static int inline isl29023_read_data(struct isl29023_dev *dev, unsigned int *mlux)
{
	unsigned int raw_data;
	unsigned char lux_lsb, lux_msb;
	unsigned char data;

	if (isl29023_i2c_read_byte(dev, ISL29023_REG_DATA_LSB, &lux_lsb) 
		|| isl29023_i2c_read_byte(dev, ISL29023_REG_DATA_MSB, &lux_msb)) {
		return -EINVAL;
	}

	raw_data = lux_lsb | (lux_msb << 8);
	rawdata_to_mlux(dev, raw_data, mlux);

	return 0;
}

static int isl29023_mlux_raw_to_accuracy(int mlux)
{
	int i;
	const int *table;
	struct isl29023_dev *dev = &s_isl29023_dev;

	switch (dev->accuracy) {
		case ISL29023_ACCURACY_HIGH:
			table = high_resolution_table;
			break;
		case ISL29023_ACCURACY_MEDIUM:
			table = medium_resolution_table;
			break;
		case ISL29023_ACCURACY_LOW:
			table = low_resolution_table;
			break;
		default:
			table = low_resolution_table;
	}

	for (i = 0; i < ACCURACY_LEVEL; i++) {
		if (i == (ACCURACY_LEVEL - 1)) {
			break;
		}
		if (mlux < accuracy_lux_table[i]) {
			break;
		}
	}

	logd("i=%d", i);

	return table[i];
}

static void isl29023_update_work_func(struct work_struct *work)
{
	int update_interval;
	unsigned int mlux = 0;
	struct isl29023_dev *dev = &s_isl29023_dev;

	logd("isl29023_update_work_func\r\n");	
	if (isl29023_read_data(dev, &mlux)) {
		return;
	}

	switch (dev->mode) {
		case ISL29023_MODE_RANGE:
			logd ("in range mode\r\n");
			if (mlux < 20000) {
				isl29023_set_mode(dev, ISL29023_MODE_RESOLUTION);
				goto schedule;
			}
			break;
		case ISL29023_MODE_RESOLUTION:
			logd ("in resolution mode\r\n");
			if (mlux > 980000) {
				isl29023_set_mode(dev, ISL29023_MODE_RANGE);
				goto schedule;
			}
			break;
		default:
			break;
	}
	
#if (__ISL29023_GENERIC_DEBUG__)
	isl29023_dump(dev);
#endif

	if (mlux != dev->raw_mlux) {
		dev->raw_mlux = mlux;
		int resolution  = isl29023_mlux_raw_to_accuracy(mlux);
		mlux = (mlux % resolution > resolution / 2) ?
			(mlux - (mlux + resolution) % resolution) : (mlux - mlux % resolution);
		if (mlux != dev->mlux) {
			dev->mlux = mlux;
			input_report_abs(dev->input_dev, ABS_MISC, mlux);
			input_sync(dev->input_dev);
		}
		logd("resolution=%d\r\n", resolution);
		logd("raw_mlux=%d\r\n", dev->raw_mlux);
		logd("mlux=%d\r\n", dev->mlux );
	}
	
schedule:
	update_interval = dev->update_interval > 20 ? dev->update_interval : 20;
	if (dev->enable) {
		schedule_delayed_work(&dev->update_work, msecs_to_jiffies(update_interval));
	}
	return;
}

/*
 * This method may be access by multiple thread, and it may be block.
 */
static int isl29023_set_enable(struct isl29023_dev *dev, int enable)
{
	if(dev->input_dev==NULL) return 0;
	down(&dev->sem);
	if (enable == dev->enable) {
		logd("isl29023_set_enable() not change.");
		up(&dev->sem);
		return 0;
	}

	if (enable) {
		/* open sensor and start schedule timer */
		logd("isl29023_set_enable() enbale\r\n");
		if (isl29023_i2c_write_byte(dev, ISL29023_REG_COMMAND_I, ISL29023_ALS_CONTINUE)
			|| isl29023_i2c_write_byte(dev, ISL29023_REG_COMMAND_II, ISL29023_MODE_RANGE)) {
			up(&dev->sem);
			logd("isl29023_set enable failed\r\n");
			return -EINVAL;
		}
		dev->enable = enable;
		dev->mode = ISL29023_MODE_RANGE;
		schedule_delayed_work(&dev->update_work, msecs_to_jiffies(dev->update_interval));
	} else {
		/* stop the schedule timer and try to close the device */
		logd("isl29023_enable() disable\r\n");
		dev->enable = enable;
		cancel_delayed_work_sync(&dev->update_work);
		isl29023_i2c_write_byte(dev, ISL29023_REG_COMMAND_I, ISL29023_POWER_DOWN);
	}
	up(&dev->sem);
	return 0;
}

static int isl29023_set_delay(struct isl29023_dev *dev, int ms) 
{
	/* make sure the update interval is not too small, 
	 * becase this may eat a lot of cpu.
	 */

	logd("isl29023_set_delay delay=%dms", ms);

#if (__ISL29023_GENERIC_DEBUG__)
	return 0;
#else
	if (ms < 20) 
		ms = 20;;
#if 0
	dev->update_interval = ms;
#endif
	return 0;
#endif
}

static ssize_t 
isl29023_read_sysfs_delay(struct device *device, struct device_attribute* attr, char *buf)
{
	struct isl29023_dev *dev = &s_isl29023_dev;
	return sprintf(buf, "%d", dev->update_interval);
}

static ssize_t 
isl29023_write_sysfs_delay(struct device *device, struct device_attribute *attr, const char *buffer, size_t count)
{
	int value;

	struct isl29023_dev *dev = &s_isl29023_dev;

	value = simple_strtol(buffer, NULL, 10);
	
	logd("-----------------------------------------------------\r\n");
	logd("isl29023_write_sysfs_delay delay=%d\r\n", value);
	logd("-----------------------------------------------------\r\n");
	
//	isl29023_set_delay(dev, value);

	return count;
}

static ssize_t
isl29023_read_sysfs_enable(struct device *device, struct device_attribute* attr, char *buf)
{
	struct isl29023_dev *dev = &s_isl29023_dev;
	return sprintf(buf, "%d", dev->enable);
}

static ssize_t
isl29023_write_sysfs_enable(struct device *device, struct device_attribute *attr, const char *buffer, size_t count)
{
	int value;
	struct isl29023_dev *dev = &s_isl29023_dev;

	value = simple_strtol(buffer, NULL, 10);
	
	logd("-----------------------------------------------------\r\n");
	logd("isl29023_write_sysfs_enable enbale=%d, count=%d\r\n", value, count);
	logd("-----------------------------------------------------\r\n");

	value = value ? 1 : 0;
	isl29023_set_enable(dev, value);
	return count;
}

#if (__ISL29023_GENERIC_DEBUG__)
static ssize_t
isl29023_read_sysfs_debug(struct device *device, struct device_attribute *attr, char *buffer)
{
	char *useage = "useage echo [command] > [file]\r\n \
					       report: report data now)";
	memcpy(buffer, (void*)useage, strlen(useage)+1);
	
	return sizeof(*useage);
}


static ssize_t
isl29023_write_sysfs_debug(struct device *device, struct device_attribute *attr, const char *buffer, size_t count)
{
	struct isl29023_dev *dev = &s_isl29023_dev;

	logd("isl29023_write_sysfs_debug\r\n");
	input_report_abs(dev->input_dev, ABS_MISC, dev->mlux);
	input_sync(dev->input_dev);
	logd("trace1\r\n");

	return count;
}
#endif

static ssize_t
isl29023_read_sysfs_mlux(struct device *device, struct device_attribute *attr, char *buffer)
{
	struct isl29023_dev *dev = &s_isl29023_dev;
	return sprintf(buffer, "%d", dev->mlux );
}

static ssize_t
isl29023_read_sysfs_raw_mlux(struct device *device, struct device_attribute *attr, char *buffer)
{
	struct isl29023_dev *dev = &s_isl29023_dev;
	return sprintf(buffer, "%d", dev->raw_mlux);
}

static ssize_t
isl29023_read_sysfs_mode(struct device *device, struct device_attribute *attr, char *buffer)
{
	struct isl29023_dev *dev = &s_isl29023_dev;
	return sprintf(buffer, ((dev->mode==ISL29023_MODE_RANGE) ? "range" : "resolution"));
}

static ssize_t
isl29023_read_sysfs_accuracy(struct device *device, struct device_attribute *attr, char *buffer)
{
	struct isl29023_dev *dev = &s_isl29023_dev;
	return sprintf(buffer, "accuracy_%s", (dev->accuracy == ISL29023_ACCURACY_HIGH) ? "high" : 
			((dev->accuracy == ISL29023_ACCURACY_MEDIUM) ? "medium" : "low"));
}

/* TODO: how to supprt change accuracy ? */
static ssize_t 
isl29023_write_sysfs_accuracy(struct device *device, struct device_attribute *attr, const char *buffer, size_t count)
{
	return count;
}

static DEVICE_ATTR(delay, 0777, isl29023_read_sysfs_delay, isl29023_write_sysfs_delay);
static DEVICE_ATTR(enable, 0777, isl29023_read_sysfs_enable, isl29023_write_sysfs_enable);
static DEVICE_ATTR(mlux, 0777, isl29023_read_sysfs_mlux, NULL);
static DEVICE_ATTR(raw_mlux, 0777, isl29023_read_sysfs_raw_mlux, NULL);
static DEVICE_ATTR(mode, 0777, isl29023_read_sysfs_mode, NULL);
static DEVICE_ATTR(accuracy, 0777, isl29023_read_sysfs_accuracy, isl29023_write_sysfs_accuracy);

#if (__ISL29023_GENERIC_DEBUG__)
static DEVICE_ATTR(debug, 0777, isl29023_read_sysfs_debug, isl29023_write_sysfs_debug);
#endif

static int isl29023_add_sysfs_entry(struct device *device)
{
	device_create_file(device, &dev_attr_enable);
	device_create_file(device, &dev_attr_delay);
	device_create_file(device, &dev_attr_mlux);
	device_create_file(device, &dev_attr_raw_mlux);
	device_create_file(device, &dev_attr_mode);
	device_create_file(device, &dev_attr_accuracy);
#if (__ISL29023_GENERIC_DEBUG__)
	device_create_file(device, &dev_attr_debug);
#endif
	return 0;
}

static int isl29023_probe(struct platform_device* pdev)
{
	struct input_dev *input_dev;
	struct isl29023_dev *dev;

	logd("isl29023_probe...\r\n");

	dev = &s_isl29023_dev;

	memset(dev, 0, sizeof(struct isl29023_dev));

	dev->i2c_instance = ISL29023_I2C_INSTANCE;
	dev->i2c_address = ISL29023_I2C_ADDRESS;
	dev->i2c_timeout = ISL29023_I2C_TIMEOUT_MS;
	dev->i2c_speed = ISL29023_I2C_SPEED_KHZ;
	dev->update_interval = ISL29023_UPDATE_INTERVAL;
	
	/* Set default accuracy */
	dev->accuracy = ISL29023_DEFAULT_ACCURACY;

	dev->i2c = NvOdmI2cOpen(NvOdmIoModule_I2c, dev->i2c_instance);
	if (!dev->i2c) {
		return -EINVAL;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		goto failed_allocate_input;
	}
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_MISC, -0, 0, 0, 0);
	input_dev->name = "light_sensor";
	if (input_register_device(input_dev)) {
		goto failed_register_input;
	}
	dev->input_dev = input_dev;

	isl29023_add_sysfs_entry(&pdev->dev);

	if (isl29023_i2c_write_byte(dev, ISL29023_REG_COMMAND_I, 0) 
		|| isl29023_i2c_write_byte(dev, ISL29023_REG_COMMAND_II, 0)) {
		goto failed_init_sensor;
	}
#if (__ISL29023_GENERIC_DEBUG__)
	isl29023_dump(dev);
#endif
	
	init_MUTEX(&dev->sem);

	INIT_DELAYED_WORK(&dev->update_work, isl29023_update_work_func);
	printk("ISL29023: init success\r\n");
	return 0;

//failed:
	isl29023_i2c_write_byte(dev, ISL29023_REG_COMMAND_I, ISL29023_POWER_DOWN);
failed_init_sensor:
	input_unregister_device(input_dev);
failed_register_input:
	input_free_device(input_dev);
failed_allocate_input:
	NvOdmI2cClose(dev->i2c);
	dev->input_dev=NULL;
	//logd("ISL29023: Failed to init device\r\n");
	printk("ISL29023: Failed to init device\r\n");
	return -EINVAL;
}

static int isl29023_remove(struct platform_device *pdev)
{
	return 0;
}

static int isl29023_suspend(struct platform_device *pdev, pm_message_t state)
{
//	struct isl29023_dev *dev = &s_isl29023_dev;
//	isl29023_set_enable(dev, 0);
	return 0;
}

static int isl29023_resume(struct platform_device *pdev)
{ 
	/* reinit this device */
	return 0;
}

static struct platform_driver isl29023_driver = {
	.probe = isl29023_probe, 
	.remove = isl29023_remove, 
	.suspend = isl29023_suspend, 
	.resume = isl29023_resume, 
	.driver = {
		.name = "light_sensor", 
		.owner = THIS_MODULE, 
	}
};

static int isl29023_init(void) 
{
	return platform_driver_register(&isl29023_driver);
}

void isl29023_exit(void)
{
	platform_driver_unregister(&isl29023_driver);
}

module_init(isl29023_init);
module_exit(isl29023_exit);
MODULE_DESCRIPTION("Isl29023 light sensor driver for android system");
