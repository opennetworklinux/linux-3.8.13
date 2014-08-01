/* Copyright 2008-2012 Freescale Semiconductor, Inc.
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

#define pr_fmt(fmt) \
	KBUILD_MODNAME ": %s:%hu:%s() " fmt, \
	KBUILD_BASENAME".c", __LINE__, __func__

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/netdevice.h>

#include "dpaa_eth.h"
#include "dpaa_eth_common.h"
#include "mac.h"
#include "lnxwrp_fsl_fman.h"

#include "error_ext.h"	/* GET_ERROR_TYPE, E_OK */

#include "../fman/inc/flib/fsl_fman_dtsec.h"
#include "../fman/inc/flib/fsl_fman_tgec.h"
#include "../fman/inc/flib/fsl_fman_memac.h"
#include "../fman/src/wrapper/lnxwrp_sysfs_fm.h"

#define MAC_DESCRIPTION "FSL FMan MAC API based driver"

MODULE_LICENSE("Dual BSD/GPL");

MODULE_AUTHOR("Emil Medve <Emilian.Medve@Freescale.com>");

MODULE_DESCRIPTION(MAC_DESCRIPTION);

struct mac_priv_s {
	struct fm_mac_dev *fm_mac;
};

const char	*mac_driver_description __initconst = MAC_DESCRIPTION;
const size_t	 mac_sizeof_priv[] = {
	[DTSEC] = sizeof(struct mac_priv_s),
	[XGMAC] = sizeof(struct mac_priv_s),
	[MEMAC] = sizeof(struct mac_priv_s)
};

static const enet_mode_t _100[] = {
	[PHY_INTERFACE_MODE_MII]	= e_ENET_MODE_MII_100,
	[PHY_INTERFACE_MODE_RMII]	= e_ENET_MODE_RMII_100
};

static const enet_mode_t _1000[] = {
	[PHY_INTERFACE_MODE_GMII]	= e_ENET_MODE_GMII_1000,
	[PHY_INTERFACE_MODE_SGMII]	= e_ENET_MODE_SGMII_1000,
	[PHY_INTERFACE_MODE_TBI]	= e_ENET_MODE_TBI_1000,
	[PHY_INTERFACE_MODE_RGMII]	= e_ENET_MODE_RGMII_1000,
	[PHY_INTERFACE_MODE_RGMII_ID]	= e_ENET_MODE_RGMII_1000,
	[PHY_INTERFACE_MODE_RGMII_RXID]	= e_ENET_MODE_RGMII_1000,
	[PHY_INTERFACE_MODE_RGMII_TXID]	= e_ENET_MODE_RGMII_1000,
	[PHY_INTERFACE_MODE_RTBI]	= e_ENET_MODE_RTBI_1000
};

static enet_mode_t __cold __attribute__((nonnull))
macdev2enetinterface(const struct mac_device *mac_dev)
{
	switch (mac_dev->max_speed) {
	case SPEED_100:
		return _100[mac_dev->phy_if];
	case SPEED_1000:
		return _1000[mac_dev->phy_if];
	case SPEED_10000:
		return e_ENET_MODE_XGMII_10000;
	default:
		return e_ENET_MODE_MII_100;
	}
}

static void mac_exception(handle_t _mac_dev, e_FmMacExceptions exception)
{
	struct mac_device	*mac_dev;

	mac_dev = (struct mac_device *)_mac_dev;

	if (e_FM_MAC_EX_10G_RX_FIFO_OVFL == exception) {
		/* don't flag RX FIFO after the first */
		fm_mac_set_exception(mac_dev->get_mac_handle(mac_dev),
		    e_FM_MAC_EX_10G_RX_FIFO_OVFL, false);
		dev_err(mac_dev->dev, "10G MAC got RX FIFO Error = %x\n",
				exception);
	}

	dev_dbg(mac_dev->dev, "%s:%s() -> %d\n", KBUILD_BASENAME".c", __func__,
		exception);
}

static int __cold init(struct mac_device *mac_dev)
{
	int					_errno;
	struct mac_priv_s	*priv;
	t_FmMacParams		param;
	uint32_t			version;

	priv = macdev_priv(mac_dev);

	param.baseAddr =  (typeof(param.baseAddr))(uintptr_t)devm_ioremap(
		mac_dev->dev, mac_dev->res->start, 0x2000);
	param.enetMode	= macdev2enetinterface(mac_dev);
	memcpy(&param.addr, mac_dev->addr, min(sizeof(param.addr),
		sizeof(mac_dev->addr)));
	param.macId		= mac_dev->cell_index;
	param.h_Fm		= (handle_t)mac_dev->fm;
	param.mdioIrq		= NO_IRQ;
	param.f_Exception	= mac_exception;
	param.f_Event		= mac_exception;
	param.h_App		= mac_dev;

	priv->fm_mac = fm_mac_config(&param);
	if (unlikely(priv->fm_mac == NULL)) {
		_errno = -EINVAL;
		goto _return;
	}

	fm_mac_set_handle(mac_dev->fm_dev, priv->fm_mac,
		(macdev2enetinterface(mac_dev) != e_ENET_MODE_XGMII_10000) ?
			param.macId : param.macId + FM_MAX_NUM_OF_1G_MACS);

	_errno = fm_mac_config_max_frame_length(priv->fm_mac,
					  fm_get_max_frm());
	if (unlikely(_errno < 0))
		goto _return_fm_mac_free;

	if (macdev2enetinterface(mac_dev) != e_ENET_MODE_XGMII_10000) {
		/* 10G always works with pad and CRC */
		_errno = fm_mac_config_pad_and_crc(priv->fm_mac, true);
		if (unlikely(_errno < 0))
			goto _return_fm_mac_free;

		_errno = fm_mac_config_half_duplex(priv->fm_mac,
				mac_dev->half_duplex);
		if (unlikely(_errno < 0))
			goto _return_fm_mac_free;
	} else {
		_errno = fm_mac_config_reset_on_init(priv->fm_mac, true);
		if (unlikely(_errno < 0))
			goto _return_fm_mac_free;
	}

	_errno = fm_mac_init(priv->fm_mac);
	if (unlikely(_errno < 0))
		goto _return_fm_mac_free;

#ifndef CONFIG_FMAN_MIB_CNT_OVF_IRQ_EN
	/* For 1G MAC, disable by default the MIB counters overflow interrupt */
	if (macdev2enetinterface(mac_dev) != e_ENET_MODE_XGMII_10000) {
		_errno = fm_mac_set_exception(mac_dev->get_mac_handle(mac_dev),
				e_FM_MAC_EX_1G_RX_MIB_CNT_OVFL, FALSE);
		if (unlikely(_errno < 0))
			goto _return_fm_mac_free;
	}
#endif /* !CONFIG_FMAN_MIB_CNT_OVF_IRQ_EN */

	/* For 10G MAC, disable Tx ECC exception */
	if (macdev2enetinterface(mac_dev) == e_ENET_MODE_XGMII_10000) {
		_errno = fm_mac_set_exception(mac_dev->get_mac_handle(mac_dev),
					  e_FM_MAC_EX_10G_1TX_ECC_ER, FALSE);
		if (unlikely(_errno < 0))
			goto _return_fm_mac_free;
	}

	_errno = fm_mac_get_version(priv->fm_mac, &version);
	if (unlikely(_errno < 0))
		goto _return_fm_mac_free;

	dev_info(mac_dev->dev, "FMan %s version: 0x%08x\n",
		((macdev2enetinterface(mac_dev) != e_ENET_MODE_XGMII_10000) ?
			"dTSEC" : "XGEC"), version);

	goto _return;


_return_fm_mac_free:
	fm_mac_free(mac_dev->get_mac_handle(mac_dev));

_return:
	return _errno;
}

static int __cold memac_init(struct mac_device *mac_dev)
{
	int			_errno;
	struct mac_priv_s	*priv;
	t_FmMacParams		param;

	priv = macdev_priv(mac_dev);

	param.baseAddr =  (typeof(param.baseAddr))(uintptr_t)devm_ioremap(
		mac_dev->dev, mac_dev->res->start, 0x2000);
	param.enetMode	= macdev2enetinterface(mac_dev);
	memcpy(&param.addr, mac_dev->addr, sizeof(mac_dev->addr));
	param.macId		= mac_dev->cell_index;
	param.h_Fm		= (handle_t)mac_dev->fm;
	param.mdioIrq		= NO_IRQ;
	param.f_Exception	= mac_exception;
	param.f_Event		= mac_exception;
	param.h_App		= mac_dev;

	priv->fm_mac = fm_mac_config(&param);
	if (unlikely(priv->fm_mac == NULL)) {
		_errno = -EINVAL;
		goto _return;
	}

	_errno = fm_mac_config_max_frame_length(priv->fm_mac, fm_get_max_frm());
	if (unlikely(_errno < 0))
		goto _return_fm_mac_free;

	_errno = fm_mac_config_reset_on_init(priv->fm_mac, true);
	if (unlikely(_errno < 0))
		goto _return_fm_mac_free;

	_errno = fm_mac_init(priv->fm_mac);
	if (unlikely(_errno < 0))
		goto _return_fm_mac_free;

	dev_info(mac_dev->dev, "FMan MEMAC\n");

	goto _return;

_return_fm_mac_free:
	fm_mac_free(priv->fm_mac);

_return:
	return _errno;
}

static int __cold start(struct mac_device *mac_dev)
{
	int	 _errno;
	struct phy_device *phy_dev = mac_dev->phy_dev;

	_errno = fm_mac_enable(mac_dev->get_mac_handle(mac_dev));

	if (!_errno && phy_dev) {
		if (macdev2enetinterface(mac_dev) != e_ENET_MODE_XGMII_10000)
			phy_start(phy_dev);
		else if (phy_dev->drv->read_status)
			phy_dev->drv->read_status(phy_dev);
	}

	return _errno;
}

static int __cold stop(struct mac_device *mac_dev)
{
	if (mac_dev->phy_dev &&
		(macdev2enetinterface(mac_dev) != e_ENET_MODE_XGMII_10000))
		phy_stop(mac_dev->phy_dev);

	return fm_mac_disable(mac_dev->get_mac_handle(mac_dev));
}

static int __cold set_multi(struct net_device *net_dev,
			    struct mac_device *mac_dev)
{
	struct mac_priv_s	*mac_priv;
	struct mac_address	*old_addr, *tmp;
	struct netdev_hw_addr	*ha;
	int			_errno;

	mac_priv = macdev_priv(mac_dev);

	/* Clear previous address list */
	list_for_each_entry_safe(old_addr, tmp, &mac_dev->mc_addr_list, list) {
		_errno = fm_mac_remove_hash_mac_addr(mac_priv->fm_mac,
				(t_EnetAddr *)old_addr->addr);
		if (_errno < 0)
			return _errno;

		list_del(&old_addr->list);
		kfree(old_addr);
	}

	/* Add all the addresses from the new list */
	netdev_for_each_mc_addr(ha, net_dev) {
		_errno = fm_mac_add_hash_mac_addr(mac_priv->fm_mac,
				(t_EnetAddr *)ha->addr);
		if (_errno < 0)
			return _errno;

		tmp = kmalloc(sizeof(struct mac_address), GFP_ATOMIC);
		if (!tmp) {
			dev_err(mac_dev->dev, "Out of memory\n");
			return -ENOMEM;
		}
		memcpy(tmp->addr, ha->addr, ETH_ALEN);
		list_add(&tmp->list, &mac_dev->mc_addr_list);
	}
	return 0;
}

static void adjust_link(struct net_device *net_dev)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct proxy_device *proxy_dev = (struct proxy_device *)priv->peer;
	struct mac_device *mac_dev = proxy_dev ? proxy_dev->mac_dev :
		priv->mac_dev;
	struct phy_device *phy_dev = mac_dev->phy_dev;

	fm_mac_adjust_link(mac_dev->get_mac_handle(mac_dev),
			phy_dev->link, phy_dev->speed, phy_dev->duplex);
}

/* Initializes driver's PHY state, and attaches to the PHY.
 * Returns 0 on success.
 */
static int dtsec_init_phy(struct net_device *net_dev,
			  struct mac_device *mac_dev)
{
	struct phy_device	*phy_dev;

	if (!mac_dev->phy_node)
		phy_dev = phy_connect(net_dev, mac_dev->fixed_bus_id,
				&adjust_link, 0, mac_dev->phy_if);
	else
		phy_dev = of_phy_connect(net_dev, mac_dev->phy_node,
				&adjust_link, 0, mac_dev->phy_if);
	if (unlikely(phy_dev == NULL) || IS_ERR(phy_dev)) {
		netdev_err(net_dev, "Could not connect to PHY %s\n",
				mac_dev->phy_node ?
					mac_dev->phy_node->full_name :
					mac_dev->fixed_bus_id);
		return phy_dev == NULL ? -ENODEV : PTR_ERR(phy_dev);
	}

	/* Remove any features not supported by the controller */
	phy_dev->supported &= mac_dev->if_support;
	phy_dev->advertising = phy_dev->supported;

	mac_dev->phy_dev = phy_dev;

	return 0;
}

static int xgmac_init_phy(struct net_device *net_dev,
			  struct mac_device *mac_dev)
{
	struct phy_device *phy_dev;

	if (!mac_dev->phy_node)
		phy_dev = phy_attach(net_dev, mac_dev->fixed_bus_id, 0,
				mac_dev->phy_if);
	else
		phy_dev = of_phy_attach(net_dev, mac_dev->phy_node, 0,
				mac_dev->phy_if);
	if (unlikely(phy_dev == NULL) || IS_ERR(phy_dev)) {
		netdev_err(net_dev, "Could not attach to PHY %s\n",
				mac_dev->phy_node ?
					mac_dev->phy_node->full_name :
					mac_dev->fixed_bus_id);
		return phy_dev == NULL ? -ENODEV : PTR_ERR(phy_dev);
	}

	phy_dev->supported &= mac_dev->if_support;
	phy_dev->advertising = phy_dev->supported;

	mac_dev->phy_dev = phy_dev;

	return 0;
}

static int memac_init_phy(struct net_device *net_dev,
			  struct mac_device *mac_dev)
{
	struct phy_device       *phy_dev;

	if (macdev2enetinterface(mac_dev) == e_ENET_MODE_XGMII_10000) {
		if (!mac_dev->phy_node) {
			mac_dev->phy_dev = NULL;
			return 0;
		} else
			phy_dev = of_phy_attach(net_dev, mac_dev->phy_node, 0,
				mac_dev->phy_if);
	} else {
		if (!mac_dev->phy_node)
			phy_dev = phy_connect(net_dev, mac_dev->fixed_bus_id,
				&adjust_link, 0, mac_dev->phy_if);
		else
			phy_dev = of_phy_connect(net_dev, mac_dev->phy_node,
				&adjust_link, 0, mac_dev->phy_if);
	}

	if (unlikely(phy_dev == NULL) || IS_ERR(phy_dev)) {
		netdev_err(net_dev, "Could not connect to PHY %s\n",
			mac_dev->phy_node ?
				mac_dev->phy_node->full_name :
				mac_dev->fixed_bus_id);
		return phy_dev == NULL ? -ENODEV : PTR_ERR(phy_dev);
	}

	/* Remove any features not supported by the controller */
	phy_dev->supported &= mac_dev->if_support;
	phy_dev->advertising = phy_dev->supported;

	mac_dev->phy_dev = phy_dev;

	return 0;
}

static int __cold uninit(struct fm_mac_dev *fm_mac_dev)
{
	int			 _errno, __errno;

	_errno = fm_mac_disable(fm_mac_dev);
	__errno = fm_mac_free(fm_mac_dev);

	if (unlikely(__errno < 0))
		_errno = __errno;

	return _errno;
}

static struct fm_mac_dev *get_mac_handle(struct mac_device *mac_dev)
{
	const struct mac_priv_s	*priv;
	priv = macdev_priv(mac_dev);
	return priv->fm_mac;
}

static int dtsec_dump_regs(struct mac_device *h_mac, char *buf, int nn)
{
	struct dtsec_regs	*p_mm = (struct dtsec_regs *) h_mac->vaddr;
	int			i = 0, n = nn;

	FM_DMP_SUBTITLE(buf, n, "\n");

	FM_DMP_TITLE(buf, n, p_mm, "FM MAC - DTSEC-%d", h_mac->cell_index);

	FM_DMP_V32(buf, n, p_mm, tsec_id);
	FM_DMP_V32(buf, n, p_mm, tsec_id2);
	FM_DMP_V32(buf, n, p_mm, ievent);
	FM_DMP_V32(buf, n, p_mm, imask);
	FM_DMP_V32(buf, n, p_mm, ecntrl);
	FM_DMP_V32(buf, n, p_mm, ptv);
	FM_DMP_V32(buf, n, p_mm, tmr_ctrl);
	FM_DMP_V32(buf, n, p_mm, tmr_pevent);
	FM_DMP_V32(buf, n, p_mm, tmr_pemask);
	FM_DMP_V32(buf, n, p_mm, tctrl);
	FM_DMP_V32(buf, n, p_mm, rctrl);
	FM_DMP_V32(buf, n, p_mm, maccfg1);
	FM_DMP_V32(buf, n, p_mm, maccfg2);
	FM_DMP_V32(buf, n, p_mm, ipgifg);
	FM_DMP_V32(buf, n, p_mm, hafdup);
	FM_DMP_V32(buf, n, p_mm, maxfrm);

	FM_DMP_V32(buf, n, p_mm, macstnaddr1);
	FM_DMP_V32(buf, n, p_mm, macstnaddr2);

	for (i = 0; i < 7; ++i) {
		FM_DMP_V32(buf, n, p_mm, macaddr[i].exact_match1);
		FM_DMP_V32(buf, n, p_mm, macaddr[i].exact_match2);
	}

	FM_DMP_V32(buf, n, p_mm, car1);
	FM_DMP_V32(buf, n, p_mm, car2);

	return n;
}

static int xgmac_dump_regs(struct mac_device *h_mac, char *buf, int nn)
{
	struct tgec_regs	*p_mm = (struct tgec_regs *) h_mac->vaddr;
	int			n = nn;

	FM_DMP_SUBTITLE(buf, n, "\n");
	FM_DMP_TITLE(buf, n, p_mm, "FM MAC - TGEC -%d", h_mac->cell_index);

	FM_DMP_V32(buf, n, p_mm, tgec_id);
	FM_DMP_V32(buf, n, p_mm, command_config);
	FM_DMP_V32(buf, n, p_mm, mac_addr_0);
	FM_DMP_V32(buf, n, p_mm, mac_addr_1);
	FM_DMP_V32(buf, n, p_mm, maxfrm);
	FM_DMP_V32(buf, n, p_mm, pause_quant);
	FM_DMP_V32(buf, n, p_mm, rx_fifo_sections);
	FM_DMP_V32(buf, n, p_mm, tx_fifo_sections);
	FM_DMP_V32(buf, n, p_mm, rx_fifo_almost_f_e);
	FM_DMP_V32(buf, n, p_mm, tx_fifo_almost_f_e);
	FM_DMP_V32(buf, n, p_mm, hashtable_ctrl);
	FM_DMP_V32(buf, n, p_mm, mdio_cfg_status);
	FM_DMP_V32(buf, n, p_mm, mdio_command);
	FM_DMP_V32(buf, n, p_mm, mdio_data);
	FM_DMP_V32(buf, n, p_mm, mdio_regaddr);
	FM_DMP_V32(buf, n, p_mm, status);
	FM_DMP_V32(buf, n, p_mm, tx_ipg_len);
	FM_DMP_V32(buf, n, p_mm, mac_addr_2);
	FM_DMP_V32(buf, n, p_mm, mac_addr_3);
	FM_DMP_V32(buf, n, p_mm, rx_fifo_ptr_rd);
	FM_DMP_V32(buf, n, p_mm, rx_fifo_ptr_wr);
	FM_DMP_V32(buf, n, p_mm, tx_fifo_ptr_rd);
	FM_DMP_V32(buf, n, p_mm, tx_fifo_ptr_wr);
	FM_DMP_V32(buf, n, p_mm, imask);
	FM_DMP_V32(buf, n, p_mm, ievent);

	return n;
}

static int memac_dump_regs(struct mac_device *h_mac, char *buf, int nn)
{
	struct memac_regs	*p_mm = (struct memac_regs *) h_mac->vaddr;
	int			i = 0, n = nn;

	FM_DMP_SUBTITLE(buf, n, "\n");
	FM_DMP_TITLE(buf, n, p_mm, "FM MAC - MEMAC -%d", h_mac->cell_index);

	FM_DMP_V32(buf, n, p_mm, command_config);
	FM_DMP_V32(buf, n, p_mm, mac_addr0.mac_addr_l);
	FM_DMP_V32(buf, n, p_mm, mac_addr0.mac_addr_u);
	FM_DMP_V32(buf, n, p_mm, maxfrm);
	FM_DMP_V32(buf, n, p_mm, hashtable_ctrl);
	FM_DMP_V32(buf, n, p_mm, ievent);
	FM_DMP_V32(buf, n, p_mm, tx_ipg_length);
	FM_DMP_V32(buf, n, p_mm, imask);

	for (i = 0; i < 4; ++i)
		FM_DMP_V32(buf, n, p_mm, pause_quanta[i]);

	for (i = 0; i < 4; ++i)
		FM_DMP_V32(buf, n, p_mm, pause_thresh[i]);

	FM_DMP_V32(buf, n, p_mm, rx_pause_status);

	for (i = 0; i < MEMAC_NUM_OF_PADDRS; ++i) {
		FM_DMP_V32(buf, n, p_mm, mac_addr[i].mac_addr_l);
		FM_DMP_V32(buf, n, p_mm, mac_addr[i].mac_addr_u);
	}

	FM_DMP_V32(buf, n, p_mm, lpwake_timer);
	FM_DMP_V32(buf, n, p_mm, sleep_timer);
	FM_DMP_V32(buf, n, p_mm, statn_config);
	FM_DMP_V32(buf, n, p_mm, if_mode);
	FM_DMP_V32(buf, n, p_mm, if_status);
	FM_DMP_V32(buf, n, p_mm, hg_config);
	FM_DMP_V32(buf, n, p_mm, hg_pause_quanta);
	FM_DMP_V32(buf, n, p_mm, hg_pause_thresh);
	FM_DMP_V32(buf, n, p_mm, hgrx_pause_status);
	FM_DMP_V32(buf, n, p_mm, hg_fifos_status);
	FM_DMP_V32(buf, n, p_mm, rhm);
	FM_DMP_V32(buf, n, p_mm, thm);

	return n;
}

int fm_mac_dump_regs(struct mac_device *h_mac, char *buf, int nn)
{
	int	n = nn;

	n = h_mac->dump_mac_regs(h_mac, buf, n);

	return n;
}


static void __cold setup_dtsec(struct mac_device *mac_dev)
{
	mac_dev->init_phy	= dtsec_init_phy;
	mac_dev->init		= init;
	mac_dev->start		= start;
	mac_dev->stop		= stop;
	mac_dev->set_promisc	= fm_mac_set_promiscuous;
	mac_dev->change_addr    = fm_mac_modify_mac_addr;
	mac_dev->set_multi      = set_multi;
	mac_dev->uninit		= uninit;
	mac_dev->ptp_enable		= fm_mac_enable_1588_time_stamp;
	mac_dev->ptp_disable		= fm_mac_disable_1588_time_stamp;
	mac_dev->get_mac_handle		= get_mac_handle;
	mac_dev->set_tx_pause		= fm_mac_set_tx_pause_frames;
	mac_dev->set_rx_pause		= fm_mac_set_rx_ignore_pause_frames;
	mac_dev->fm_rtc_enable		= fm_rtc_enable;
	mac_dev->fm_rtc_disable		= fm_rtc_disable;
	mac_dev->fm_rtc_get_cnt		= fm_rtc_get_cnt;
	mac_dev->fm_rtc_set_cnt		= fm_rtc_set_cnt;
	mac_dev->fm_rtc_get_drift	= fm_rtc_get_drift;
	mac_dev->fm_rtc_set_drift	= fm_rtc_set_drift;
	mac_dev->fm_rtc_set_alarm	= fm_rtc_set_alarm;
	mac_dev->fm_rtc_set_fiper	= fm_rtc_set_fiper;
	mac_dev->dump_mac_regs		= dtsec_dump_regs;
}

static void __cold setup_xgmac(struct mac_device *mac_dev)
{
	mac_dev->init_phy	= xgmac_init_phy;
	mac_dev->init		= init;
	mac_dev->start		= start;
	mac_dev->stop		= stop;
	mac_dev->set_promisc	= fm_mac_set_promiscuous;
	mac_dev->change_addr    = fm_mac_modify_mac_addr;
	mac_dev->set_multi      = set_multi;
	mac_dev->uninit		= uninit;
	mac_dev->get_mac_handle	= get_mac_handle;
	mac_dev->set_tx_pause	= fm_mac_set_tx_pause_frames;
	mac_dev->set_rx_pause	= fm_mac_set_rx_ignore_pause_frames;
	mac_dev->dump_mac_regs	= xgmac_dump_regs;
}

static void __cold setup_memac(struct mac_device *mac_dev)
{
	mac_dev->init_phy	= memac_init_phy;
	mac_dev->init		= memac_init;
	mac_dev->start		= start;
	mac_dev->stop		= stop;
	mac_dev->set_promisc	= fm_mac_set_promiscuous;
	mac_dev->change_addr    = fm_mac_modify_mac_addr;
	mac_dev->set_multi      = set_multi;
	mac_dev->uninit		= uninit;
	mac_dev->get_mac_handle		= get_mac_handle;
	mac_dev->set_tx_pause		= fm_mac_set_tx_pause_frames;
	mac_dev->set_rx_pause		= fm_mac_set_rx_ignore_pause_frames;
	mac_dev->fm_rtc_enable		= fm_rtc_enable;
	mac_dev->fm_rtc_disable		= fm_rtc_disable;
	mac_dev->fm_rtc_get_cnt		= fm_rtc_get_cnt;
	mac_dev->fm_rtc_set_cnt		= fm_rtc_set_cnt;
	mac_dev->fm_rtc_get_drift	= fm_rtc_get_drift;
	mac_dev->fm_rtc_set_drift	= fm_rtc_set_drift;
	mac_dev->fm_rtc_set_alarm	= fm_rtc_set_alarm;
	mac_dev->fm_rtc_set_fiper	= fm_rtc_set_fiper;
	mac_dev->dump_mac_regs		= memac_dump_regs;
}

void (*const mac_setup[])(struct mac_device *mac_dev) = {
	[DTSEC] = setup_dtsec,
	[XGMAC] = setup_xgmac,
	[MEMAC] = setup_memac
};
