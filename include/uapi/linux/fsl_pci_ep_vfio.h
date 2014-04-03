/*
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * Author: Minghuan Lian <Minghuan.Lian@freescale.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#ifndef _FSL_PCI_EP_VFIO_H
#define _FSL_PCI_EP_VFIO_H

#include <linux/vfio.h>

enum {
	PCI_EP_TYPE_PF,
	PCI_EP_TYPE_VF,
};

enum PCI_EP_REGION_TYPE {
	PCI_EP_REGION_IBWIN,
	PCI_EP_REGION_OBWIN,
	PCI_EP_REGION_VF_IBWIN,
	PCI_EP_REGION_VF_OBWIN,
	PCI_EP_REGION_REGS,
	PCI_EP_REGION_CONFIG,
};

enum PCI_EP_REGION_INDEX {
	PCI_EP_WIN0_INDEX,
	PCI_EP_WIN1_INDEX,
	PCI_EP_WIN2_INDEX,
	PCI_EP_WIN3_INDEX,
	PCI_EP_WIN4_INDEX,
	PCI_EP_WIN5_INDEX,
};

#define PCI_EP_MSI_WIN_INDEX PCI_EP_WIN1_INDEX
#define PCI_EP_CCSR_WIN_INDEX PCI_EP_WIN0_INDEX
#define PCI_EP_DEFAULT_OW_INDEX PCI_EP_WIN0_INDEX

struct pci_ep_win {
	uint64_t pci_addr;
	uint64_t cpu_addr;
	uint64_t size;
	uint64_t offset;
	uint32_t attr;
	uint32_t type;
	uint32_t idx;
};

#define VFIO_DEVICE_SET_WIN_INFO	_IO(VFIO_TYPE, VFIO_BASE + 20)
#define VFIO_DEVICE_GET_WIN_INFO	_IO(VFIO_TYPE, VFIO_BASE + 21)

struct pci_ep_info {
	uint32_t type;
	uint32_t pf_idx;
	uint32_t vf_idx;
	uint32_t iw_num;
	uint32_t ow_num;
	uint32_t vf_iw_num;
	uint32_t vf_ow_num;
};

#endif
