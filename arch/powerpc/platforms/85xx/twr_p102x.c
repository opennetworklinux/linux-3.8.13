/*
 * Copyright 2010-2011, 2013 Freescale Semiconductor, Inc.
 *
 * Author: Michael Johnston <michael.johnston@freescale.com>
 *
 * Description:
 * TWR-P102x Board Setup
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/fsl_devices.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/memblock.h>

#ifdef CONFIG_FB_SSD1289
#include <linux/platform_data/video-twrfb.h>
#endif

#include <asm/time.h>
#include <asm/machdep.h>
#include <asm/pci-bridge.h>
#include <mm/mmu_decl.h>
#include <asm/prom.h>
#include <asm/udbg.h>
#include <asm/mpic.h>
#include <asm/qe.h>
#include <asm/qe_ic.h>
#include <asm/fsl_guts.h>

#include <sysdev/fsl_soc.h>
#include <sysdev/fsl_pci.h>
#include "smp.h"

#include "mpc85xx.h"

static void __init twr_p1025_pic_init(void)
{
	struct mpic *mpic;

#ifdef CONFIG_QUICC_ENGINE
	struct device_node *np;
#endif

	mpic = mpic_alloc(NULL, 0, MPIC_BIG_ENDIAN |
			MPIC_SINGLE_DEST_CPU,
			0, 256, " OpenPIC  ");

	BUG_ON(mpic == NULL);
	mpic_init(mpic);

#ifdef CONFIG_QUICC_ENGINE
	np = of_find_compatible_node(NULL, NULL, "fsl,qe-ic");
	if (np) {
		qe_ic_init(np, 0, qe_ic_cascade_low_mpic,
				qe_ic_cascade_high_mpic);
		of_node_put(np);
	} else
		printk(KERN_ERR "Could not find qe-ic node\n");
#endif
}

/* ************************************************************************
 *
 * Setup the architecture
 *
 */
static void __init twr_p1025_setup_arch(void)
{
#ifdef CONFIG_QUICC_ENGINE
	struct device_node *np;
#endif

	if (ppc_md.progress)
		ppc_md.progress("twr_p1025_setup_arch()", 0);

	mpc85xx_smp_init();

	fsl_pci_assign_primary();

#ifdef CONFIG_QUICC_ENGINE
	np = of_find_compatible_node(NULL, NULL, "fsl,qe");

	if (!np) {
		np = of_find_node_by_name(NULL, "qe");
		if (!np) {
			printk(KERN_ERR "Could not find Quicc Engine node\n");
			goto qe_fail;
		}
	}

	qe_reset();
	of_node_put(np);

	np = of_find_node_by_name(NULL, "par_io");
	if (np) {
		struct device_node *ucc;

		par_io_init(np);
		of_node_put(np);

		for_each_node_by_name(ucc, "ucc")
			par_io_of_config(ucc);
	}

#if defined(CONFIG_UCC_GETH) || defined(CONFIG_SERIAL_QE)
	if (machine_is(twr_p1025)) {
		struct ccsr_guts __iomem *guts;

		np = of_find_node_by_name(NULL, "global-utilities");
		if (np) {
			guts = of_iomap(np, 0);
			if (!guts)
				pr_err("twr_p1025: could not map global utilities register\n");
			else {
			/* P1025 has pins muxed for QE and other functions. To
			 * enable QE UEC mode, we need to set bit QE0 for UCC1
			 * in Eth mode, QE0 and QE3 for UCC5 in Eth mode, QE9
			 * and QE12 for QE MII management signals in PMUXCR
			 * register.
			 */

			printk(KERN_INFO "P1025 pinmux configured for QE\n");

			/* Set QE mux bits in PMUXCR */
			setbits32(&guts->pmuxcr, MPC85xx_PMUXCR_QE(0) |
					MPC85xx_PMUXCR_QE(3) |
					MPC85xx_PMUXCR_QE(9) |
					MPC85xx_PMUXCR_QE(12));
			iounmap(guts);

#if defined(CONFIG_SERIAL_QE)
			/* On P1025TWR board, the UCC7 acted as UART port.
			 * However, The UCC7's CTS pin is low level in default,
			 * it will impact the transmission in full duplex
			 * communication. So disable the Flow control pin PA18.
			 * The UCC7 UART just can use RXD and TXD pins.
			 */
			par_io_config_pin(0, 18, 0, 0, 0, 0);
#endif
			/* Drive PB29 to CPLD low - CPLD will then change
			 * muxing from LBC to QE */
			par_io_config_pin(1, 29, 1, 0, 0, 0);
			par_io_data_set(1, 29, 0);
			}
			of_node_put(np);
		}
	}
#endif

qe_fail:
#endif	/* CONFIG_QUICC_ENGINE */

	printk(KERN_INFO "TWR-P1025 board from Freescale Semiconductor\n");
}

machine_arch_initcall(twr_p1025, mpc85xx_common_publish_devices);

static int __init twr_p1025_probe(void)
{
	unsigned long root = of_get_flat_dt_root();

	return of_flat_dt_is_compatible(root, "fsl,TWR-P1025");

}

define_machine(twr_p1025) {
	.name			= "TWR-P1025",
	.probe			= twr_p1025_probe,
	.setup_arch		= twr_p1025_setup_arch,
	.init_IRQ		= twr_p1025_pic_init,
#ifdef CONFIG_PCI
	.pcibios_fixup_bus	= fsl_pcibios_fixup_bus,
#endif
	.get_irq		= mpic_get_irq,
	.restart		= fsl_rstcr_restart,
	.calibrate_decr		= generic_calibrate_decr,
	.progress		= udbg_progress,
};

#ifdef CONFIG_FB_SSD1289
static struct fsl_ssd1289_fb_display fsl_ssd1289_data = {
	.width		= 320,
	.height		= 240,
	.xres		= 320,
	.yres		= 240,
	.bpp		= 16,
};

static int __init p1025twr_ssd1289_init(void)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct resource res[2];
	int ret;

	np = of_find_compatible_node(NULL, NULL, "ssd1289");
	if (!np) {
		printk(KERN_ERR "Get display ssd1289 device node fails\n");
		return -ENODEV;
	}

	memset(res, 0, sizeof(res));
	ret = of_address_to_resource(np, 0, &res[0]);
	if (ret) {
		printk(KERN_ERR "Failed to get resource 0\n");
		return -ENODEV;
	}
	ret = of_address_to_resource(np, 1, &res[1]);
	if (ret) {
		printk(KERN_ERR "Failed to get resource 1\n");
		return -ENODEV;
	}
	pdev = platform_device_alloc("ssd1289", 0);
	if (!pdev) {
		printk(KERN_ERR "Failed to alloc platform_device\n");
		return -ENODEV;
	}
	ret = platform_device_add_resources(pdev, res, ARRAY_SIZE(res));
	if (ret)
		goto unreg;

	ret = platform_device_add_data(pdev, &fsl_ssd1289_data,
					sizeof(fsl_ssd1289_data));
	if (ret)
		goto unreg;

	ret = platform_device_add(pdev);
	if (ret)
		goto unreg;

	return 0;

unreg:
	platform_device_del(pdev);
	return -ENODEV;

}

machine_device_initcall(twr_p1025, p1025twr_ssd1289_init);
#endif
