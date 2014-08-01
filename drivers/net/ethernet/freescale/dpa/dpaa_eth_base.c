/* Copyright 2008-2013 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_net.h>
#include <linux/etherdevice.h>
#include <linux/kthread.h>
#include <linux/percpu.h>
#include <linux/highmem.h>
#include <linux/sort.h>
#include <linux/fsl_qman.h>
#include "dpaa_eth.h"
#include "dpaa_eth_common.h"

static int dpa_bp_cmp(const void *dpa_bp0, const void *dpa_bp1)
{
	return ((struct dpa_bp *)dpa_bp0)->size -
			((struct dpa_bp *)dpa_bp1)->size;
}

struct dpa_bp * __cold __must_check /* __attribute__((nonnull)) */
dpa_bp_probe(struct platform_device *_of_dev, size_t *count)
{
	int			 i, lenp, na, ns;
	struct device		*dev;
	struct device_node	*dev_node;
	const phandle		*phandle_prop;
	const uint32_t		*bpid;
	const uint32_t		*bpool_cfg;
	struct dpa_bp		*dpa_bp;

	dev = &_of_dev->dev;

	/* The default is one, if there's no property */
	*count = 1;

	/* Get the buffer pools to be used */
	phandle_prop = of_get_property(dev->of_node,
					"fsl,bman-buffer-pools", &lenp);

	if (phandle_prop)
		*count = lenp / sizeof(phandle);
	else {
		dev_err(dev,
			"missing fsl,bman-buffer-pools device tree entry\n");
		return ERR_PTR(-EINVAL);
	}

	dpa_bp = devm_kzalloc(dev, *count * sizeof(*dpa_bp), GFP_KERNEL);
	if (unlikely(dpa_bp == NULL)) {
		dev_err(dev, "devm_kzalloc() failed\n");
		return ERR_PTR(-ENOMEM);
	}

	dev_node = of_find_node_by_path("/");
	if (unlikely(dev_node == NULL)) {
		dev_err(dev, "of_find_node_by_path(/) failed\n");
		return ERR_PTR(-EINVAL);
	}

	na = of_n_addr_cells(dev_node);
	ns = of_n_size_cells(dev_node);

	for (i = 0; i < *count && phandle_prop; i++) {
		of_node_put(dev_node);
		dev_node = of_find_node_by_phandle(phandle_prop[i]);
		if (unlikely(dev_node == NULL)) {
			dev_err(dev, "of_find_node_by_phandle() failed\n");
			return ERR_PTR(-EFAULT);
		}

		if (unlikely(!of_device_is_compatible(dev_node, "fsl,bpool"))) {
			dev_err(dev,
				"!of_device_is_compatible(%s, fsl,bpool)\n",
				dev_node->full_name);
			dpa_bp = ERR_PTR(-EINVAL);
			goto _return_of_node_put;
		}

		bpid = of_get_property(dev_node, "fsl,bpid", &lenp);
		if ((bpid == NULL) || (lenp != sizeof(*bpid))) {
			dev_err(dev, "fsl,bpid property not found.\n");
			dpa_bp = ERR_PTR(-EINVAL);
			goto _return_of_node_put;
		}
		dpa_bp[i].bpid = *bpid;

		bpool_cfg = of_get_property(dev_node, "fsl,bpool-ethernet-cfg",
					&lenp);
		if (bpool_cfg && (lenp == (2 * ns + na) * sizeof(*bpool_cfg))) {
			const uint32_t *seed_pool;

			dpa_bp[i].config_count =
				(int)of_read_number(bpool_cfg, ns);
			dpa_bp[i].size	= of_read_number(bpool_cfg + ns, ns);
			dpa_bp[i].paddr	=
				of_read_number(bpool_cfg + 2 * ns, na);

			seed_pool = of_get_property(dev_node,
					"fsl,bpool-ethernet-seeds", &lenp);
			dpa_bp[i].seed_pool = !!seed_pool;

		} else {
			dev_err(dev,
				"Missing/invalid fsl,bpool-ethernet-cfg device tree entry for node %s\n",
				dev_node->full_name);
			dpa_bp = ERR_PTR(-EINVAL);
			goto _return_of_node_put;
		}
	}

	sort(dpa_bp, *count, sizeof(*dpa_bp), dpa_bp_cmp, NULL);

	return dpa_bp;

_return_of_node_put:
	if (dev_node)
		of_node_put(dev_node);

	return dpa_bp;
}

int dpa_bp_shared_port_seed(struct dpa_bp *bp)
{
	/* In MAC-less and Shared-MAC scenarios the physical
	 * address of the buffer pool in device tree is set
	 * to 0 to specify that another entity (USDPAA) will
	 * allocate and seed the buffers
	 */
	if (!bp->paddr)
		return 0;

	/* allocate memory region for buffers */
	devm_request_mem_region(bp->dev, bp->paddr,
			bp->size * bp->config_count, KBUILD_MODNAME);
	bp->vaddr = devm_ioremap_prot(bp->dev, bp->paddr,
			bp->size * bp->config_count, 0);
	if (bp->vaddr == NULL) {
		pr_err("Could not map memory for pool %d\n", bp->bpid);
		return -EIO;
	}

	/* seed pool with buffers from that memory region */
	if (bp->seed_pool) {
		int count = bp->target_count;
		size_t addr = bp->paddr;

		while (count) {
			struct bm_buffer bufs[8];
			int num_bufs = 0;

			do {
				BUG_ON(addr > 0xffffffffffffull);
				bufs[num_bufs].bpid = bp->bpid;
				bm_buffer_set64(&bufs[num_bufs++], addr);
				addr += bp->size;

			} while (--count && (num_bufs < 8));

			while (bman_release(bp->pool, bufs, num_bufs, 0))
				cpu_relax();
		}
	}

	return 0;
}

int dpa_bp_create(struct net_device *net_dev, struct dpa_bp *dpa_bp,
		size_t count)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	int i;

	priv->dpa_bp = dpa_bp;
	priv->bp_count = count;

	for (i = 0; i < count; i++) {
		int err;
		err = dpa_bp_alloc(&dpa_bp[i]);
		if (err < 0) {
			dpa_bp_free(priv, dpa_bp);
			priv->dpa_bp = NULL;
			return err;
		}
	}

	return 0;
}

