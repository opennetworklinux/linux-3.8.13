/*
 * Freescale Data Collection Manager (DCM) device driver
 *
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
 * Author: Timur Tabi <timur@freescale.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Inside the FPGA of some Freescale QorIQ (PowerPC) reference boards is a
 * microprocessor called the General Purpose Processor (GSMA).  Running on
 * the GSMA is the Data Collection Manager (DCM), which is used to
 * periodically read and tally voltage, current, and temperature measurements
 * from the on-board sensors.  You can use this feature to measure power
 * consumption while running tests, without having the host CPU perform those
 * measurements.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/of_platform.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

/* sysfs commands for 'control' */
#define SYSFS_DCM_CMD_STOP	0
#define SYSFS_DCM_CMD_START	1

/* Crecords can be either voltage, current, or temperature */
enum crecord_type {
	CR_V,	/* voltage */
	CR_C,	/* current */
	CR_T	/* temperature */
};

#define MAX_FREQUENCY		48	/* max freq (Hz) the DCM supports */
#define DATA_ADDR		0x80	/* data address in DCM SRAM */

struct crecord {
	__be16	curr;		/* last sample read */
	__be16	max;		/* maximum of all samples read */
	__be16	qty1;		/* number of samples read  */
	u8	qty2;
	__be32	acc;		/* sum of all samples read */
} __packed;
#define MAX_CRECORDS	((0x100 - DATA_ADDR) / sizeof(struct crecord))

struct om_info {
	__be16	version;	/* DCM version number */
	u8	prescale;	/* prescale value used (should be 0) */
	u8	timer;		/* timer */
	u8	count;		/* number of CRECORDS */
	__be16	address;	/* address of CRECORD array in SRAM */
	u8	res[3];
} __packed;

#define MAX_FUNCTION	9
/**
  * struct dcm_board - board-specific information
  * @compatible: the 'compatible' property to search for
  * @mask: enable mask for the OM_ENABLE command
  * @convert_iout: board-specific function to convert an IOUT to milliamps
  * @convert_tout: board-specific function to convert a TOUT to degrees Celsius
  * @ical: calibration factor for .convert_iout function
  * @num: number of crecords (equal to the number of '1's in @mask)
  * @names: array of names of each crecord
  * @types: array of types of each crecord
  * @voltage_fun: the index of voltage convert function
  */
struct dcm_board {
	const char *compatible;
	u16 mask;
	unsigned int (*convert_iout)(u16 iout, unsigned int ical);
	unsigned int (*convert_tout)(u16 tout);
	unsigned int ical;
	unsigned int num;
	const char *names[MAX_CRECORDS];
	enum crecord_type types[MAX_CRECORDS];
	unsigned int voltage_fun[4];
};

/* PIXIS register status bits */
#define PX_OCMD_MSG	(1 << 0)
#define PX_OACK_ERR	(1 << 1)
#define PX_OACK_ACK	(1 << 0)	/* OACK is sometimes called MACK */

/* DCM commands */
#define OM_END			0x00
#define OM_SETDLY		0x01
#define OM_RST0			0x02
#define OM_RST1			0x03
#define OM_CHKDLY		0x04
#define OM_PWR			0x05
#define OM_WAKE			0x07
#define OM_GETMEM		0x08
#define OM_SETMEM		0x09
#define OM_SCLR			0x10
#define OM_START		0x11
#define OM_STOP			0x12
#define OM_GET			0x13
#define OM_ENABLE		0x14
#define OM_TIMER		0x15
#define OM_SETV			0x30
#define OM_INFO			0x31

struct fsl_dcm_data {
	struct device *dev;
	const struct dcm_board *board;
	void __iomem *base;	/* PIXIS/QIXIS base address */
	u8 __iomem *addr;	/* SRAM address */
	u8 __iomem *data;	/* SRAM data */
	u8 __iomem *ocmd;	/* DCM command/status */
	u8 __iomem *omsg;	/* DCM message */
	u8 __iomem *mack;	/* DCM acknowledge */
	struct crecord rec[MAX_CRECORDS];
	u8 timer;
	int running;
};

/*
 * Converts a 16-bit VOUT value from the Zilker ZL6100 into a voltage value,
 * in millivolts.
 */
static unsigned int voltage_from_zl6100(u16 vout)
{
	return (1000UL * vout) / (1 << 13);
}

/* unit is mv */
static unsigned int voltage_from_ina220(u16 vout)
{
	return (vout >> 3) * 4;
}

static unsigned int (*voltage_convert[2])(u16 vout) = {
	voltage_from_zl6100, voltage_from_ina220
};
/*
 * Converts a 16-bit IOUT from the Texas Instruments INA220 chip into a
 * current value, in milliamps.  'ical' is a board-specific calibration
 * factor.
 */
static unsigned int current_from_ina220(u16 vout, unsigned int ical)
{
	unsigned long c;

	/* milliamp = 1000 * ((vout / 100) / cal-factor) */
	/*          = (vout * 100000) / ical         */
	c = vout * 1000 * 100;
	c /= ical;

	return c;
}

/*
 * Converts a 16-bit TOUT value from the sensor device into a temperature
 * value, in degrees Celsius.
 */
static unsigned int temp_from_u16(u16 tout)
{
	return tout;
}

/*
 * Write a byte to an address in SRAM
 */
static void write_sram(struct fsl_dcm_data *dcm, u8 offset, u8 v)
{
	out_8(dcm->addr, offset);
	out_8(dcm->data, v);
}

/*
 * Read a byte from an address in SRAM
 */
static u8 read_sram(struct fsl_dcm_data *dcm, u8 offset)
{
	out_8(dcm->addr, offset);

	return in_8(dcm->data);
}

/*
 * True TRUE if we can read/write SRAM, FALSE otherwise.
 *
 * If the SRAM is unavailable, it's probably because the DCM is busy with it.
 */
static int is_sram_available(struct fsl_dcm_data *dcm)
{
	u8 ack, cmd;

	cmd = in_8(dcm->ocmd);
	ack = in_8(dcm->mack);

	if ((cmd & PX_OCMD_MSG) || (ack & PX_OACK_ACK)) {
		dev_dbg(dcm->dev, "dcm is not ready (cmd=%02X mack=%02X)\n",
			 cmd, ack);
		return 0;
	}

	return 1;
}

/*
 * Loads and program into SRAM, then tells the DCM to run it, and then waits
 * for it to finish.
 */
static int run_program(struct fsl_dcm_data *dcm, u8 addr,
		unsigned int len, ...)
{
	u8 v, n;
	va_list args;

	if (addr + len > 0xff) {
		dev_err(dcm->dev, "address/length of %u/%u is out of bounds\n",
		       addr, len);
		return 0;
	}

	/* load the program into SRAM */
	va_start(args, len);
	for (n = addr; n < addr + len; n++) {
		v = va_arg(args, int);
		write_sram(dcm, n, v);
	}
	va_end(args);

	/* start the DCM */
	out_8(dcm->omsg, addr);
	out_8(dcm->ocmd, PX_OCMD_MSG);

	/* wait for ack or error */
	v = spin_event_timeout(in_8(dcm->mack) & (PX_OACK_ERR | PX_OACK_ACK),
			       50000, 1000);
	if ((!v) || (v & PX_OACK_ERR)) {
		dev_err(dcm->dev, "timeout or error waiting for start ack\n");
		return 0;
	}

	/* 4. allow the host to read SRAM */
	out_8(dcm->ocmd, 0);

	/* 5. wait for DCM to stop (ack == 0) or error (err == 1) */
	spin_event_timeout(
		((v = in_8(dcm->mack)) & (PX_OACK_ERR | PX_OACK_ACK))
		!= PX_OACK_ACK, 50000, 1000);

	/* 6. check for error or timeout */
	if (v & (PX_OACK_ERR | PX_OACK_ACK)) {
		dev_err(dcm->dev, "timeout or error waiting for stop ack\n");
		return 0;
	}

	return 1;
}

#define TRATE0	241122		/* t-rate if prescale==0, in millihertz */
#define TRATE1	38579330	/* t-rate if prescale==1, in millihertz */

/*
 * Empirical tests show that any frequency higher than 48Hz is unreliable.
 */
static int set_dcm_frequency(struct fsl_dcm_data *dcm,
		unsigned long frequency) {
	unsigned long timer;

	if (!is_sram_available(dcm)) {
		dev_err(dcm->dev, "dcm is busy\n");
		return 0;
	}

	/* Restrict the frequency to a supported range. */
	frequency = clamp_t(unsigned long, frequency, 1, MAX_FREQUENCY);

	/* We only support prescale == 0 */
	timer = TRATE0 / frequency;
	dcm->timer = ((timer / 1000) - 1) & 0xff;

	return run_program(dcm, 0, 6, OM_TIMER, 0, dcm->timer, 0, 0, OM_END);
}

static int copy_from_sram(struct fsl_dcm_data *dcm, unsigned int addr,
			  void *buf, unsigned int len)
{
	u8 *p = buf;
	unsigned int i;

	if (addr + len > 0xff) {
		dev_err(dcm->dev, "address/length of %u/%u is out of bounds\n",
		       addr, len);
		return 0;
	}

	for (i = 0; i < len; i++)
		p[i] = read_sram(dcm, addr + i);

	return 1;
}

/*
 * Tells the DCM which channels to collect data on.
 */
static int select_dcm_channels(struct fsl_dcm_data *dcm, u16 mask)
{
	if (!is_sram_available(dcm)) {
		dev_err(dcm->dev, "dcm is busy\n");
		return 0;
	}

	return run_program(dcm, 0, 4, OM_ENABLE,
		((mask >> 8) & 0xFF), mask & 0xFF, OM_END);
}

/*
 * Tells the DCM to start data collection.  If the DCM is currently running,
 * it is restarted.  Any currently collected data is cleared.
 */
int start_data_collection(struct fsl_dcm_data *dcm)
{
	if (!is_sram_available(dcm)) {
		dev_err(dcm->dev, "dcm is busy\n");
		return 0;
	}

	if (dcm->running)
		dev_dbg(dcm->dev, "restarting\n");

	dcm->running = true;

	return run_program(dcm, 0, 4, OM_STOP, OM_SCLR, OM_START, OM_END);
}

/*
 * Tells the DCM to stop data collection.  Collected data is copied from
 * SRAM into a local buffer.
 */
int stop_data_collection(struct fsl_dcm_data *dcm)
{
	if (!dcm->running) {
		dev_dbg(dcm->dev, "dcm is already stopped\n");
		return 1;
	}

	if (!is_sram_available(dcm)) {
		dev_err(dcm->dev, "dcm is busy\n");
		return 0;
	}

	if (!run_program(dcm, 0, 4, OM_STOP, OM_GET, DATA_ADDR, OM_END)) {
		dev_err(dcm->dev, "could not stop monitoring\n");
		return 0;
	}

	if (!copy_from_sram(dcm, DATA_ADDR, dcm->rec,
			    dcm->board->num * sizeof(struct crecord))) {
		dev_err(dcm->dev, "could not copy sensor data\n");
		return 0;
	}

	dcm->running = 0;
	return 1;
}

ssize_t fsl_dcm_sysfs_control_show(struct device *dev,
	struct device_attribute *attr, char *buf) {
	struct fsl_dcm_data *dcm = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", dcm->running ? "running" : "stoppped");
}

ssize_t fsl_dcm_sysfs_control_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count) {
	struct fsl_dcm_data *dcm = dev_get_drvdata(dev);
	unsigned long pm_cmd;
	int ret;

	ret = strict_strtoul(buf, 10, &pm_cmd);
	if (ret)
		return ret;

	switch (pm_cmd) {
	case SYSFS_DCM_CMD_START:
		ret = start_data_collection(dcm);
		if (!ret)
			dev_err(dev, "failed to start power monitoring.\n");
		break;
	case SYSFS_DCM_CMD_STOP:
		ret = stop_data_collection(dcm);
		if (!ret)
			dev_err(dev, "failed to stop power monitoring\n");
		break;
	default:
		return -EIO;
	}

	return count;
}

/* Calculate the average, even if 'count' is zero */
#define AVG(sum, count)	((sum) / ((count) ?: 1))
#define MAX_RECORD 9
static ssize_t fsl_dcm_sysfs_result(struct device *dev,
	struct device_attribute *attr, char *buf) {
	struct fsl_dcm_data *dcm = dev_get_drvdata(dev);
	const struct dcm_board *board = dcm->board;
	unsigned int i, vindex;
	ssize_t len;
	char *str;
	unsigned int num[MAX_RECORD], max[MAX_RECORD], avg[MAX_RECORD];
	const char *unit[MAX_RECORD];

	len = sprintf(buf,
		"Name                         Average\n"
		"====================         ================\n");

	vindex = 0;
	for (i = 0; i < board->num; i++) {
		num[i] = (dcm->rec[i].qty1 << 8) | (dcm->rec[i].qty2);

		switch (board->types[i]) {
		case CR_V:
			max[i] = voltage_convert[board->voltage_fun[vindex]](
					dcm->rec[i].max);
			avg[i] = voltage_convert[board->voltage_fun[vindex]](
					AVG(dcm->rec[i].acc, num[i]));
			vindex++;

			unit[i] = "mV";
			break;
		case CR_C:
			max[i] = board->convert_iout(dcm->rec[i].max,
					board->ical);
			avg[i] = board->convert_iout(AVG(dcm->rec[i].acc,
						num[i]), board->ical);
			unit[i] = "mA";
			break;
		case CR_T:
			max[i] = board->convert_tout(dcm->rec[i].max);
			avg[i] = board->convert_tout(AVG(dcm->rec[i].acc,
						num[i]));
			unit[i] = "C ";
			break;
		default:
			continue;
		}
	}

	str = strstr(board->compatible, "tetra-fpga");
	if (str) { /* T4240 board */
		unsigned int idd;

		idd = avg[2] + avg[3] + avg[4] + avg[5];
		len += sprintf(buf + len,
				"CPU voltage:                 %-6u (mV)\n",
				avg[0]);
		len += sprintf(buf + len,
				"CPU current:                 %-6u (mA)\n",
				idd);
		len += sprintf(buf + len,
				"DDR voltage:                 %-6u (mV)\n",
				avg[6]);
		len += sprintf(buf + len,
				"DDR current:                 %-6u (mA)\n",
				avg[7]);
		len += sprintf(buf + len,
				"CPU temperature:             %-6u (C)\n",
				avg[8]);

	} else { /* for else */
		for (i = 0; i < board->num; i++)
			len += sprintf(buf + len,
				"%-8s                     %-6d %s\n",
				board->names[i], avg[i], unit[i]);
	}

	return len;
}

ssize_t fsl_dcm_sysfs_info(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct fsl_dcm_data *dcm = dev_get_drvdata(dev);
	struct om_info info;
	ssize_t len;

	if (!is_sram_available(dcm)) {
		dev_err(dev, "dcm is busy\n");
		return 0;
	}

	if (!run_program(dcm, 0, 3, OM_INFO, DATA_ADDR, OM_END)) {
		dev_err(dev, "could not run 'info' program\n");
		return 0;
	}

	if (!copy_from_sram(dcm, DATA_ADDR, &info, sizeof(info))) {
		dev_err(dev, "could not copy 'info' data\n");
		return 0;
	}

	len = sprintf(buf, "DCM Version: %u\n", info.version);
	len += sprintf(buf + len, "Prescale: %u\n", info.prescale);
	len += sprintf(buf + len, "Timer: %u\n", info.timer);
	len += sprintf(buf + len, "Number of CRECORDs: %u\n", info.count);
	len += sprintf(buf + len, "CRECORD Address: %u\n", info.address);

	return len;
}

ssize_t fsl_dcm_sysfs_frequency_show(struct device *dev,
	struct device_attribute *attr, char *buf) {
	struct fsl_dcm_data *dcm = dev_get_drvdata(dev);
	unsigned long frequency;

	frequency = TRATE0 / (dcm->timer + 1);

	return sprintf(buf, "%lu Hz\n", frequency / 1000);
}

ssize_t fsl_dcm_sysfs_frequency_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fsl_dcm_data *dcm = dev_get_drvdata(dev);
	unsigned long frequency;
	int ret;

	ret = strict_strtoul(buf, 10, &frequency);
	if (ret)
		return ret;

	set_dcm_frequency(dcm, frequency);

	return count;
}

static DEVICE_ATTR(control, 0666, fsl_dcm_sysfs_control_show,
		   fsl_dcm_sysfs_control_store);
static DEVICE_ATTR(result, 0444, fsl_dcm_sysfs_result, NULL);
static DEVICE_ATTR(info, 0444, fsl_dcm_sysfs_info, NULL);
static DEVICE_ATTR(frequency, 0666, fsl_dcm_sysfs_frequency_show,
		   fsl_dcm_sysfs_frequency_store);

static const struct attribute_group fsl_dcm_attr_group = {
	.attrs = (struct attribute * []) {
		&dev_attr_control.attr,
		&dev_attr_result.attr,
		&dev_attr_info.attr,
		&dev_attr_frequency.attr,
		NULL,
	},
};

static int fsl_dcm_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_dcm_data *dcm;
	int ret;
	u8 ver;

	dcm = kzalloc(sizeof(struct fsl_dcm_data), GFP_KERNEL);
	if (!dcm)
		return -ENOMEM;

	dcm->base = of_iomap(np, 0);
	if (!dcm->base) {
		dev_err(&pdev->dev, "could not map fpga node\n");
		ret = -ENOMEM;
		goto error_kzalloc;
	}

	dcm->dev = &pdev->dev;
	dcm->board = pdev->dev.platform_data;

	/*
	 * write 0x1F to GDC register then read GDD register
	 * to get GMSA version.
	 * 0x00: v1  -> pixis
	 * 0x01: v2  -> qixis
	 */
	out_8(dcm->base + 0x16, 0x1F);
	ver = in_8(dcm->base + 0x17);
	if (ver == 0x0) {
		dcm->addr = dcm->base + 0x0a;
		dcm->data = dcm->base + 0x0d;
	} else if (ver == 0x01) {
		dcm->addr = dcm->base + 0x12;
		dcm->data = dcm->base + 0x13;
	}

	dcm->ocmd = dcm->base + 0x14;
	dcm->omsg = dcm->base + 0x15;
	dcm->mack = dcm->base + 0x18;

	/* Check to make sure the DCM is enable and working */
	if (!is_sram_available(dcm)) {
		dev_err(&pdev->dev, "dcm is not responding\n");
		ret = -ENODEV;
		goto error_iomap;
	}

	dev_set_drvdata(&pdev->dev, dcm);

	ret = sysfs_create_group(&pdev->dev.kobj, &fsl_dcm_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "could not create sysfs group\n");
		goto error_iomap;
	}

	if (!select_dcm_channels(dcm, dcm->board->mask)) {
		dev_err(&pdev->dev, "could not set crecord mask\n");
		ret = -ENODEV;
		goto error_sysfs;
	}

	/* Set the timer to the fastest support rate. */
	if (!set_dcm_frequency(dcm, 1)) {
		dev_err(&pdev->dev, "could not set frequency\n");
		ret = -ENODEV;
		goto error_sysfs;
	}

	return 0;

error_sysfs:
	sysfs_remove_group(&pdev->dev.kobj, &fsl_dcm_attr_group);

error_iomap:
	iounmap(dcm->base);

error_kzalloc:
	kfree(dcm);

	return ret;
}

static int fsl_dcm_remove(struct platform_device *pdev)
{
	struct fsl_dcm_data *dcm = dev_get_drvdata(&pdev->dev);

	stop_data_collection(dcm);

	sysfs_remove_group(&pdev->dev.kobj, &fsl_dcm_attr_group);

	iounmap(dcm->base);
	kfree(dcm);

	return 0;
}

static struct platform_driver fsl_dcm_platform_driver = {
	.probe = fsl_dcm_probe,
	.remove = fsl_dcm_remove,
	.driver	= {
		.name = "fsl-dcm",
		.owner = THIS_MODULE,
	},
};

static const struct dcm_board dcm_types[] = {
	{ /* 4-v, 1-t */
		"fsl,p1022ds-fpga",
		0x155,
		NULL,
		temp_from_u16,
		0,
		5,
		/* Current measurements are not reliable on this board */
		{"Vdd", "OVdd", "S/XVdd", "GVdd", "CPU_Tj"},
		{CR_V, CR_V, CR_V, CR_V, CR_T},
		{0, 0, 0, 0},
	},
	{ /* 4-v, 4-i, 1-t */
		"fsl,p5020ds-fpga",
		0x1ff,
		current_from_ina220,
		temp_from_u16,
		21064,
		9,
		{"Vdd_CA", "Idd_CA", "Vdd_CB", "Idd_CB", "Vdd_PL", "Idd_PL",
			"GVdd", "GIdd", "CPU_Tj"},
		{CR_V, CR_C, CR_V, CR_C, CR_V, CR_C, CR_V, CR_C, CR_T},
		{0, 0, 0, 0},
	},
	{ /* 2-v, 6-c, 1-t */
		"fsl,tetra-fpga",
		0x1ff,
		current_from_ina220,
		temp_from_u16,
		10000,
		9,
		{"VDD", "IDD", "IDD_ph0", "IDD_ph1", "IDD_ph2", "IDD_ph3",
			"GVDD", "GIDD", "TEMP"},
		{CR_V, CR_C, CR_C, CR_C, CR_C, CR_C, CR_V, CR_C, CR_T},
		{1, 0},
	},
};

static int __init fsl_dcm_init(void)
{
	struct platform_device *pdev;
	struct device_node *np = NULL;
	unsigned int i;
	int ret;

	/* Look for a supported platform */
	for (i = 0; i < ARRAY_SIZE(dcm_types); i++) {
		np = of_find_compatible_node(NULL, NULL,
					     dcm_types[i].compatible);
		if (np)
			break;
	}
	if (!np) {
		pr_debug("fsl-dcm: unsupported platform\n");
		return -ENODEV;
	}

	/* We found a supported platform, so register a platform driver */
	ret = platform_driver_register(&fsl_dcm_platform_driver);
	if (ret) {
		pr_err("fsl-dcm: could not register platform driver\n");
		goto error_np;
	}

	/* We need to create a device and add the data for this platform */
	pdev = platform_device_alloc(fsl_dcm_platform_driver.driver.name, 0);
	if (!pdev) {
		ret = -ENOMEM;
		goto error_drv;
	}

	/* Pass the device_node pointer to the probe function */
	pdev->dev.of_node = np;

	/* Pass the DCM platform data */
	ret = platform_device_add_data(pdev, &dcm_types[i],
				 sizeof(struct dcm_board));
	if (ret) {
		pr_err("fsl-dcm: could not register platform driver\n");
		goto error_dev;
	}

	/* This will call the probe function */
	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("fsl-dcm: could not register platform driver\n");
		goto error_dev;
	}

	of_node_put(np);

	pr_info("Freescale Data Collection Module is installed.\n");
	return 0;

error_dev:
	platform_device_unregister(pdev);

error_drv:
	platform_driver_unregister(&fsl_dcm_platform_driver);

error_np:
	of_node_put(np);

	return ret;
}

static void __exit fsl_dcm_exit(void)
{
	platform_driver_unregister(&fsl_dcm_platform_driver);
}

MODULE_AUTHOR("Timur Tabi <timur@freescale.com>");
MODULE_DESCRIPTION("Freescale Data Collection Manager driver");
MODULE_LICENSE("GPL v2");

module_init(fsl_dcm_init);
module_exit(fsl_dcm_exit);
