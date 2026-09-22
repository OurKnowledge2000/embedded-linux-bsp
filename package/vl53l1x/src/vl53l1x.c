// SPDX-License-Identifier: GPL-2.0
/*
 * vl53l1x.c - character device driver for the ST VL53L1X
 *             time-of-flight ranging sensor.
 *
 * Exposes one measurement per open() through /dev/vl53l1x0:
 *
 *     # cat /dev/vl53l1x0
 *     412
 *
 * The value is the crosstalk-corrected range in millimetres. If the
 * sensor reports a range status other than "valid", -1 is returned
 * instead.
 *
 * When the device tree supplies an interrupt for the sensor's GPIO1
 * "measurement ready" output, read() sleeps on a wait queue and is
 * woken by a threaded interrupt handler. If no interrupt is available
 * (for example when the device was instantiated by hand through
 * /sys/bus/i2c/devices/i2c-1/new_device) the driver falls back to
 * polling the data-ready flag over I2C, so the same module works both
 * ways.
 *
 * A running count of interrupts is exported at
 * /sys/class/vl53l1x/vl53l1x0/irq_count, which is the easy way to prove
 * that readings really are interrupt driven rather than quietly polled.
 *
 * The register map and initialisation sequence follow ST's VL53L1X
 * Ultra Lite Driver (UM2356 / VL53L1X_api.c). The sensor requires a
 * 91-byte configuration block to be written before it will range at
 * all; there is no simple "read a register to get distance" mode.
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define DRV_NAME			"vl53l1x"

/*
 * Register addresses are 16 bit and transmitted big endian, which is
 * why this driver does its own i2c_transfer() calls rather than using
 * the smbus helpers.
 */
#define VL53L1X_REG_VHV_TIMEOUT_BOUND	0x0008
#define VL53L1X_REG_VHV_LOOP_BOUND	0x000B
#define VL53L1X_REG_CONFIG_START	0x002D
#define VL53L1X_REG_GPIO_HV_MUX_CTRL	0x0030
#define VL53L1X_REG_GPIO_TIO_HV_STATUS	0x0031
#define VL53L1X_REG_PHASECAL_TIMEOUT	0x004B
#define VL53L1X_REG_TIMEOUT_MACROP_A_HI	0x005E
#define VL53L1X_REG_VCSEL_PERIOD_A	0x0060
#define VL53L1X_REG_TIMEOUT_MACROP_B_HI	0x0061
#define VL53L1X_REG_VCSEL_PERIOD_B	0x0063
#define VL53L1X_REG_VALID_PHASE_HIGH	0x0069
#define VL53L1X_REG_SD_CONFIG_WOI_SD0	0x0078
#define VL53L1X_REG_SD_CONFIG_PHASE_SD0	0x007A
#define VL53L1X_REG_SYSTEM_INT_CLEAR	0x0086
#define VL53L1X_REG_SYSTEM_MODE_START	0x0087
#define VL53L1X_REG_RESULT_RANGE_STATUS	0x0089
#define VL53L1X_REG_RESULT_DISTANCE	0x0096
#define VL53L1X_REG_FW_SYSTEM_STATUS	0x00E5
#define VL53L1X_REG_MODEL_ID		0x010F

#define VL53L1X_MODEL_ID		0xEACC

#define VL53L1X_MODE_STOP		0x00
#define VL53L1X_MODE_RANGE		0x40

/* Range status 9 means "valid range" in the ULD status table. */
#define VL53L1X_RANGE_STATUS_VALID	9

#define VL53L1X_BOOT_TIMEOUT_MS		100
#define VL53L1X_RANGE_TIMEOUT_MS	1000

#define VL53L1X_RXBUF_SIZE		8

/*
 * Default configuration block, written to registers 0x2D..0x87.
 * Taken from ST's VL53L1X_api.c. Most entries are documented there as
 * "not user modifiable"; the ones this driver overrides afterwards are
 * distance mode and timing budget.
 */
static const u8 vl53l1x_default_config[] = {
	0x00, /* 0x2d */
	0x00, /* 0x2e */
	0x00, /* 0x2f */
	0x01, /* 0x30 : interrupt polarity */
	0x02, /* 0x31 */
	0x00, /* 0x32 */
	0x02, /* 0x33 */
	0x08, /* 0x34 */
	0x00, /* 0x35 */
	0x08, /* 0x36 */
	0x10, /* 0x37 */
	0x01, /* 0x38 */
	0x01, /* 0x39 */
	0x00, /* 0x3a */
	0x00, /* 0x3b */
	0x00, /* 0x3c */
	0x00, /* 0x3d */
	0xff, /* 0x3e */
	0x00, /* 0x3f */
	0x0f, /* 0x40 */
	0x00, /* 0x41 */
	0x00, /* 0x42 */
	0x00, /* 0x43 */
	0x00, /* 0x44 */
	0x00, /* 0x45 */
	0x20, /* 0x46 : interrupt on new sample ready */
	0x0b, /* 0x47 */
	0x00, /* 0x48 */
	0x00, /* 0x49 */
	0x02, /* 0x4a */
	0x0a, /* 0x4b */
	0x21, /* 0x4c */
	0x00, /* 0x4d */
	0x00, /* 0x4e */
	0x05, /* 0x4f */
	0x00, /* 0x50 */
	0x00, /* 0x51 */
	0x00, /* 0x52 */
	0x00, /* 0x53 */
	0xc8, /* 0x54 */
	0x00, /* 0x55 */
	0x00, /* 0x56 */
	0x38, /* 0x57 */
	0xff, /* 0x58 */
	0x01, /* 0x59 */
	0x00, /* 0x5a */
	0x08, /* 0x5b */
	0x00, /* 0x5c */
	0x00, /* 0x5d */
	0x01, /* 0x5e */
	0xdb, /* 0x5f */
	0x0f, /* 0x60 */
	0x01, /* 0x61 */
	0xf1, /* 0x62 */
	0x0d, /* 0x63 */
	0x01, /* 0x64 : sigma threshold MSB */
	0x68, /* 0x65 : sigma threshold LSB */
	0x00, /* 0x66 : min count rate MSB */
	0x80, /* 0x67 : min count rate LSB */
	0x08, /* 0x68 */
	0xb8, /* 0x69 */
	0x00, /* 0x6a */
	0x00, /* 0x6b */
	0x00, /* 0x6c : inter-measurement period, byte 3 */
	0x00, /* 0x6d */
	0x0f, /* 0x6e */
	0x89, /* 0x6f : inter-measurement period, byte 0 */
	0x00, /* 0x70 */
	0x00, /* 0x71 */
	0x00, /* 0x72 : distance threshold high MSB */
	0x00, /* 0x73 */
	0x00, /* 0x74 : distance threshold low MSB */
	0x00, /* 0x75 */
	0x00, /* 0x76 */
	0x01, /* 0x77 */
	0x0f, /* 0x78 */
	0x0d, /* 0x79 */
	0x0e, /* 0x7a */
	0x0e, /* 0x7b */
	0x00, /* 0x7c */
	0x00, /* 0x7d */
	0x02, /* 0x7e */
	0xc7, /* 0x7f : ROI centre */
	0xff, /* 0x80 : ROI width/height */
	0x9b, /* 0x81 */
	0x00, /* 0x82 */
	0x00, /* 0x83 */
	0x00, /* 0x84 */
	0x01, /* 0x85 */
	0x00, /* 0x86 : clear interrupt */
	0x00, /* 0x87 : mode start, left stopped */
};

struct vl53l1x {
	struct i2c_client	*client;
	struct device		*dev;
	struct cdev		cdev;
	dev_t			devt;
	struct mutex		lock;

	int			irq;
	wait_queue_head_t	wq;
	bool			sample_ready;
	int			last_distance;
	unsigned long		irq_count;

	u8			int_polarity;

	/*
	 * kmalloc'd along with this struct, so both buffers are safe to
	 * hand to i2c_transfer() on controllers that use DMA.
	 */
	u8			rxbuf[VL53L1X_RXBUF_SIZE];
	u8			xfer[2 + sizeof(vl53l1x_default_config)];
};

static struct class *vl53l1x_class;

/* ------------------------------------------------------------------ */
/* Register access                                                     */
/* ------------------------------------------------------------------ */

static int vl53l1x_write(struct vl53l1x *v, u16 reg, const u8 *val, size_t len)
{
	struct i2c_msg msg;
	int ret;

	if (len > sizeof(v->xfer) - 2)
		return -EINVAL;

	v->xfer[0] = reg >> 8;
	v->xfer[1] = reg & 0xff;
	memcpy(&v->xfer[2], val, len);

	msg.addr  = v->client->addr;
	msg.flags = 0;
	msg.len   = len + 2;
	msg.buf   = v->xfer;

	ret = i2c_transfer(v->client->adapter, &msg, 1);
	if (ret < 0)
		return ret;

	return ret == 1 ? 0 : -EIO;
}

static int vl53l1x_read(struct vl53l1x *v, u16 reg, u8 *val, size_t len)
{
	struct i2c_msg msgs[2];
	int ret;

	if (len > sizeof(v->rxbuf))
		return -EINVAL;

	v->xfer[0] = reg >> 8;
	v->xfer[1] = reg & 0xff;

	msgs[0].addr  = v->client->addr;
	msgs[0].flags = 0;
	msgs[0].len   = 2;
	msgs[0].buf   = v->xfer;

	msgs[1].addr  = v->client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len   = len;
	msgs[1].buf   = v->rxbuf;

	ret = i2c_transfer(v->client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	memcpy(val, v->rxbuf, len);
	return 0;
}

static int vl53l1x_wr8(struct vl53l1x *v, u16 reg, u8 val)
{
	return vl53l1x_write(v, reg, &val, 1);
}

static int vl53l1x_wr16(struct vl53l1x *v, u16 reg, u16 val)
{
	u8 buf[2] = { val >> 8, val & 0xff };

	return vl53l1x_write(v, reg, buf, sizeof(buf));
}

static int vl53l1x_rd8(struct vl53l1x *v, u16 reg, u8 *val)
{
	return vl53l1x_read(v, reg, val, 1);
}

static int vl53l1x_rd16(struct vl53l1x *v, u16 reg, u16 *val)
{
	u8 buf[2];
	int ret;

	ret = vl53l1x_read(v, reg, buf, sizeof(buf));
	if (ret)
		return ret;

	*val = (buf[0] << 8) | buf[1];
	return 0;
}

/* ------------------------------------------------------------------ */
/* Sensor bring-up                                                     */
/* ------------------------------------------------------------------ */

static int vl53l1x_wait_boot(struct vl53l1x *v)
{
	unsigned long deadline = jiffies +
				 msecs_to_jiffies(VL53L1X_BOOT_TIMEOUT_MS);
	u8 status;
	int ret;

	do {
		ret = vl53l1x_rd8(v, VL53L1X_REG_FW_SYSTEM_STATUS, &status);
		if (ret)
			return ret;
		if (status & 0x01)
			return 0;
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));

	return -ETIMEDOUT;
}

static int vl53l1x_data_ready(struct vl53l1x *v, bool *ready)
{
	u8 status;
	int ret;

	ret = vl53l1x_rd8(v, VL53L1X_REG_GPIO_TIO_HV_STATUS, &status);
	if (ret)
		return ret;

	*ready = (status & 0x01) == v->int_polarity;
	return 0;
}

static int vl53l1x_poll_data_ready(struct vl53l1x *v)
{
	unsigned long deadline = jiffies +
				 msecs_to_jiffies(VL53L1X_RANGE_TIMEOUT_MS);
	bool ready = false;
	int ret;

	do {
		ret = vl53l1x_data_ready(v, &ready);
		if (ret)
			return ret;
		if (ready)
			return 0;
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));

	return -ETIMEDOUT;
}

/* Long distance mode: up to roughly 4 m, more affected by ambient light. */
static int vl53l1x_set_long_mode(struct vl53l1x *v)
{
	int ret;

	ret = vl53l1x_wr8(v, VL53L1X_REG_PHASECAL_TIMEOUT, 0x0A);
	if (ret)
		return ret;
	ret = vl53l1x_wr8(v, VL53L1X_REG_VCSEL_PERIOD_A, 0x0F);
	if (ret)
		return ret;
	ret = vl53l1x_wr8(v, VL53L1X_REG_VCSEL_PERIOD_B, 0x0D);
	if (ret)
		return ret;
	ret = vl53l1x_wr8(v, VL53L1X_REG_VALID_PHASE_HIGH, 0xB8);
	if (ret)
		return ret;
	ret = vl53l1x_wr16(v, VL53L1X_REG_SD_CONFIG_WOI_SD0, 0x0F0D);
	if (ret)
		return ret;

	return vl53l1x_wr16(v, VL53L1X_REG_SD_CONFIG_PHASE_SD0, 0x0E0E);
}

/* 100 ms timing budget, long distance mode. */
static int vl53l1x_set_timing_budget(struct vl53l1x *v)
{
	int ret;

	ret = vl53l1x_wr16(v, VL53L1X_REG_TIMEOUT_MACROP_A_HI, 0x02E1);
	if (ret)
		return ret;

	return vl53l1x_wr16(v, VL53L1X_REG_TIMEOUT_MACROP_B_HI, 0x0388);
}

static int vl53l1x_init_sensor(struct vl53l1x *v)
{
	u16 model_id;
	u8 mux;
	int ret;

	ret = vl53l1x_rd16(v, VL53L1X_REG_MODEL_ID, &model_id);
	if (ret) {
		dev_err(&v->client->dev, "failed to read model ID: %d\n", ret);
		return ret;
	}

	if (model_id != VL53L1X_MODEL_ID) {
		dev_err(&v->client->dev,
			"unexpected model ID 0x%04x (expected 0x%04x)\n",
			model_id, VL53L1X_MODEL_ID);
		return -ENODEV;
	}

	ret = vl53l1x_wait_boot(v);
	if (ret) {
		dev_err(&v->client->dev, "sensor firmware did not boot: %d\n",
			ret);
		return ret;
	}

	ret = vl53l1x_write(v, VL53L1X_REG_CONFIG_START,
			    vl53l1x_default_config,
			    sizeof(vl53l1x_default_config));
	if (ret) {
		dev_err(&v->client->dev, "failed to write config block: %d\n",
			ret);
		return ret;
	}

	/* Cache the interrupt polarity the config block just programmed. */
	ret = vl53l1x_rd8(v, VL53L1X_REG_GPIO_HV_MUX_CTRL, &mux);
	if (ret)
		return ret;
	v->int_polarity = !((mux & 0x10) >> 4);

	dev_dbg(&v->client->dev, "GPIO_HV_MUX_CTRL=0x%02x, int_polarity=%u\n",
		mux, v->int_polarity);

	/*
	 * The ULD requires one throwaway measurement after the config
	 * block before the VHV settings below can be applied. This one
	 * is always polled: the interrupt is not hooked up yet.
	 */
	ret = vl53l1x_wr8(v, VL53L1X_REG_SYSTEM_MODE_START,
			  VL53L1X_MODE_RANGE);
	if (ret)
		return ret;

	ret = vl53l1x_poll_data_ready(v);
	if (ret) {
		dev_err(&v->client->dev, "no first measurement: %d\n", ret);
		return ret;
	}

	ret = vl53l1x_wr8(v, VL53L1X_REG_SYSTEM_INT_CLEAR, 0x01);
	if (ret)
		return ret;
	ret = vl53l1x_wr8(v, VL53L1X_REG_SYSTEM_MODE_START,
			  VL53L1X_MODE_STOP);
	if (ret)
		return ret;
	ret = vl53l1x_wr8(v, VL53L1X_REG_VHV_TIMEOUT_BOUND, 0x09);
	if (ret)
		return ret;
	ret = vl53l1x_wr8(v, VL53L1X_REG_VHV_LOOP_BOUND, 0x00);
	if (ret)
		return ret;

	ret = vl53l1x_set_long_mode(v);
	if (ret)
		return ret;

	return vl53l1x_set_timing_budget(v);
}

/*
 * Read out a completed measurement and clear the interrupt so the
 * sensor can assert it again for the next one. Caller holds the lock.
 */
static int vl53l1x_read_sample(struct vl53l1x *v, int *distance_mm)
{
	u16 distance;
	u8 status;
	int ret;

	ret = vl53l1x_rd8(v, VL53L1X_REG_RESULT_RANGE_STATUS, &status);
	if (ret)
		return ret;

	ret = vl53l1x_rd16(v, VL53L1X_REG_RESULT_DISTANCE, &distance);
	if (ret)
		return ret;

	ret = vl53l1x_wr8(v, VL53L1X_REG_SYSTEM_INT_CLEAR, 0x01);
	if (ret)
		return ret;

	if ((status & 0x1F) == VL53L1X_RANGE_STATUS_VALID)
		*distance_mm = distance;
	else
		*distance_mm = -1;

	return 0;
}

/* ------------------------------------------------------------------ */
/* Interrupt handling                                                  */
/* ------------------------------------------------------------------ */

/*
 * Threaded handler: the sensor is on I2C, so servicing the interrupt
 * requires sleeping bus transactions and cannot be done in hard
 * interrupt context.
 *
 * This handler must fully consume the interrupt before it returns.
 * GPIO1 is a latched, level triggered line: the sensor holds it
 * asserted until SYSTEM__INTERRUPT_CLEAR is written, so a handler that
 * returns without clearing it is immediately re-entered, forever. That
 * is why the measurement is read out here rather than in read() - the
 * read is what clears the latch and lets the line deassert.
 *
 * The result is cached for whichever reader wakes up next.
 */
static irqreturn_t vl53l1x_irq_thread(int irq, void *data)
{
	struct vl53l1x *v = data;
	int distance;
	int ret;

	mutex_lock(&v->lock);

	v->irq_count++;

	/*
	 * No data-ready check here, deliberately. The interrupt is
	 * itself the data-ready signal - that is the entire reason
	 * GPIO1 is wired up. Re-confirming it by reading
	 * GPIO__TIO_HV_STATUS means interpreting the sensor's interrupt
	 * polarity convention, which does not match its own
	 * documentation on this part, and getting it wrong leaves the
	 * latch uncleared and the level triggered line asserted
	 * forever.
	 *
	 * vl53l1x_read_sample() clears the latch, so the line always
	 * deasserts regardless of what the measurement turned out to
	 * be. An out-of-range or otherwise invalid measurement still
	 * reports as -1 through the range status check.
	 */
	ret = vl53l1x_read_sample(v, &distance);
	if (!ret) {
		v->last_distance = distance;
		v->sample_ready = true;
		mutex_unlock(&v->lock);
		wake_up_interruptible(&v->wq);
		return IRQ_HANDLED;
	}

	/* Readout failed; clear the latch anyway so we do not spin. */
	vl53l1x_wr8(v, VL53L1X_REG_SYSTEM_INT_CLEAR, 0x01);
	dev_warn_ratelimited(&v->client->dev,
			     "failed to read measurement: %d\n", ret);

	mutex_unlock(&v->lock);

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* Character device                                                    */
/* ------------------------------------------------------------------ */

static int vl53l1x_open(struct inode *inode, struct file *file)
{
	struct vl53l1x *v = container_of(inode->i_cdev, struct vl53l1x, cdev);

	file->private_data = v;
	return 0;
}

static int vl53l1x_acquire(struct vl53l1x *v, int *distance_mm)
{
	long remaining;
	int ret;

	if (v->irq > 0) {
		/*
		 * Drop any cached sample first so that every read()
		 * returns a measurement taken after the call started,
		 * rather than whatever the last interrupt happened to
		 * leave behind.
		 */
		if (mutex_lock_interruptible(&v->lock))
			return -ERESTARTSYS;
		v->sample_ready = false;
		mutex_unlock(&v->lock);

		remaining = wait_event_interruptible_timeout(
			v->wq, READ_ONCE(v->sample_ready),
			msecs_to_jiffies(VL53L1X_RANGE_TIMEOUT_MS));

		if (remaining < 0)
			return remaining;
		if (remaining == 0) {
			dev_warn(v->dev,
				 "timed out waiting for measurement interrupt\n");
			return -ETIMEDOUT;
		}

		if (mutex_lock_interruptible(&v->lock))
			return -ERESTARTSYS;

		*distance_mm = v->last_distance;
		ret = 0;

		mutex_unlock(&v->lock);

		return ret;
	}

	/* No interrupt wired up: poll the data-ready flag instead. */
	if (mutex_lock_interruptible(&v->lock))
		return -ERESTARTSYS;

	ret = vl53l1x_poll_data_ready(v);
	if (!ret)
		ret = vl53l1x_read_sample(v, distance_mm);

	mutex_unlock(&v->lock);

	return ret;
}

static ssize_t vl53l1x_cdev_read(struct file *file, char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct vl53l1x *v = file->private_data;
	char text[16];
	int distance;
	int ret;
	int len;

	if (*ppos > 0)
		return 0;

	ret = vl53l1x_acquire(v, &distance);
	if (ret)
		return ret;

	len = scnprintf(text, sizeof(text), "%d\n", distance);

	return simple_read_from_buffer(buf, count, ppos, text, len);
}

static const struct file_operations vl53l1x_fops = {
	.owner	= THIS_MODULE,
	.open	= vl53l1x_open,
	.read	= vl53l1x_cdev_read,
};

/* ------------------------------------------------------------------ */
/* sysfs                                                               */
/* ------------------------------------------------------------------ */

static ssize_t irq_count_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct vl53l1x *v = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%lu\n", READ_ONCE(v->irq_count));
}
static DEVICE_ATTR_RO(irq_count);

/* ------------------------------------------------------------------ */
/* Driver model                                                        */
/* ------------------------------------------------------------------ */

static int vl53l1x_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct vl53l1x *v;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;

	v = devm_kzalloc(dev, sizeof(*v), GFP_KERNEL);
	if (!v)
		return -ENOMEM;

	v->client = client;
	v->irq = client->irq;
	mutex_init(&v->lock);
	init_waitqueue_head(&v->wq);
	i2c_set_clientdata(client, v);

	ret = vl53l1x_init_sensor(v);
	if (ret)
		return ret;

	ret = alloc_chrdev_region(&v->devt, 0, 1, DRV_NAME);
	if (ret) {
		dev_err(dev, "failed to allocate char device region: %d\n",
			ret);
		return ret;
	}

	cdev_init(&v->cdev, &vl53l1x_fops);
	v->cdev.owner = THIS_MODULE;

	ret = cdev_add(&v->cdev, v->devt, 1);
	if (ret) {
		dev_err(dev, "failed to add char device: %d\n", ret);
		goto err_region;
	}

	v->dev = device_create(vl53l1x_class, dev, v->devt, v, "%s%d",
			       DRV_NAME, MINOR(v->devt));
	if (IS_ERR(v->dev)) {
		ret = PTR_ERR(v->dev);
		dev_err(dev, "failed to create device node: %d\n", ret);
		goto err_cdev;
	}

	ret = device_create_file(v->dev, &dev_attr_irq_count);
	if (ret) {
		dev_err(dev, "failed to create irq_count attribute: %d\n",
			ret);
		goto err_device;
	}

	if (v->irq > 0) {
		ret = devm_request_threaded_irq(dev, v->irq, NULL,
						vl53l1x_irq_thread,
						IRQF_ONESHOT, DRV_NAME, v);
		if (ret) {
			dev_err(dev, "failed to request IRQ %d: %d\n",
				v->irq, ret);
			goto err_attr;
		}
	}

	/* Free running: a new sample every timing budget period. */
	ret = vl53l1x_wr8(v, VL53L1X_REG_SYSTEM_MODE_START,
			  VL53L1X_MODE_RANGE);
	if (ret) {
		dev_err(dev, "failed to start ranging: %d\n", ret);
		goto err_attr;
	}

	dev_info(dev, "VL53L1X ranging at /dev/%s%d (%s)\n",
		 DRV_NAME, MINOR(v->devt),
		 v->irq > 0 ? "interrupt driven" : "polled, no IRQ in DT");

	return 0;

err_attr:
	device_remove_file(v->dev, &dev_attr_irq_count);
err_device:
	device_destroy(vl53l1x_class, v->devt);
err_cdev:
	cdev_del(&v->cdev);
err_region:
	unregister_chrdev_region(v->devt, 1);

	vl53l1x_wr8(v, VL53L1X_REG_SYSTEM_MODE_START, VL53L1X_MODE_STOP);

	return ret;
}

static void vl53l1x_remove(struct i2c_client *client)
{
	struct vl53l1x *v = i2c_get_clientdata(client);

	vl53l1x_wr8(v, VL53L1X_REG_SYSTEM_MODE_START, VL53L1X_MODE_STOP);

	device_remove_file(v->dev, &dev_attr_irq_count);
	device_destroy(vl53l1x_class, v->devt);
	cdev_del(&v->cdev);
	unregister_chrdev_region(v->devt, 1);
}

static const struct i2c_device_id vl53l1x_id[] = {
	{ DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, vl53l1x_id);

static const struct of_device_id vl53l1x_of_match[] = {
	{ .compatible = "st,vl53l1x" },
	{ }
};
MODULE_DEVICE_TABLE(of, vl53l1x_of_match);

static struct i2c_driver vl53l1x_driver = {
	.driver = {
		.name		= DRV_NAME,
		.of_match_table	= vl53l1x_of_match,
	},
	.probe		= vl53l1x_probe,
	.remove		= vl53l1x_remove,
	.id_table	= vl53l1x_id,
};

static int __init vl53l1x_module_init(void)
{
	int ret;

	vl53l1x_class = class_create(DRV_NAME);
	if (IS_ERR(vl53l1x_class))
		return PTR_ERR(vl53l1x_class);

	ret = i2c_add_driver(&vl53l1x_driver);
	if (ret)
		class_destroy(vl53l1x_class);

	return ret;
}
module_init(vl53l1x_module_init);

static void __exit vl53l1x_module_exit(void)
{
	i2c_del_driver(&vl53l1x_driver);
	class_destroy(vl53l1x_class);
}
module_exit(vl53l1x_module_exit);

MODULE_DESCRIPTION("ST VL53L1X time-of-flight ranging sensor");
MODULE_LICENSE("GPL");
