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

#include <linux/string.h>

#include "dpaa_eth.h"

static int __cold dpa_get_settings(struct net_device *net_dev,
		struct ethtool_cmd *et_cmd)
{
	int			 _errno;
	struct dpa_priv_s	*priv;

	priv = netdev_priv(net_dev);

	if (priv->mac_dev == NULL) {
		netdev_info(net_dev, "This is a MAC-less interface\n");
		return -ENODEV;
	}
	if (unlikely(priv->mac_dev->phy_dev == NULL)) {
		netdev_dbg(net_dev, "phy device not initialized\n");
		return 0;
	}

	_errno = phy_ethtool_gset(priv->mac_dev->phy_dev, et_cmd);
	if (unlikely(_errno < 0))
		netdev_err(net_dev, "phy_ethtool_gset() = %d\n", _errno);

	return _errno;
}

static int __cold dpa_set_settings(struct net_device *net_dev,
		struct ethtool_cmd *et_cmd)
{
	int			 _errno;
	struct dpa_priv_s	*priv;

	priv = netdev_priv(net_dev);

	if (priv->mac_dev == NULL) {
		netdev_info(net_dev, "This is a MAC-less interface\n");
		return -ENODEV;
	}
	if (unlikely(priv->mac_dev->phy_dev == NULL)) {
		netdev_err(net_dev, "phy device not initialized\n");
		return -ENODEV;
	}

	_errno = phy_ethtool_sset(priv->mac_dev->phy_dev, et_cmd);
	if (unlikely(_errno < 0))
		netdev_err(net_dev, "phy_ethtool_sset() = %d\n", _errno);

	return _errno;
}

static void __cold dpa_get_drvinfo(struct net_device *net_dev,
		struct ethtool_drvinfo *drvinfo)
{
	int		 _errno;

	strncpy(drvinfo->driver, KBUILD_MODNAME,
		sizeof(drvinfo->driver) - 1)[sizeof(drvinfo->driver)-1] = 0;
	strncpy(drvinfo->version, VERSION,
		sizeof(drvinfo->driver) - 1)[sizeof(drvinfo->version)-1] = 0;
	_errno = snprintf(drvinfo->fw_version, sizeof(drvinfo->fw_version),
			  "%X", 0);

	if (unlikely(_errno >= sizeof(drvinfo->fw_version))) {
		/* Truncated output */
		netdev_notice(net_dev, "snprintf() = %d\n", _errno);
	} else if (unlikely(_errno < 0)) {
		netdev_warn(net_dev, "snprintf() = %d\n", _errno);
		memset(drvinfo->fw_version, 0, sizeof(drvinfo->fw_version));
	}
	strncpy(drvinfo->bus_info, dev_name(net_dev->dev.parent->parent),
		sizeof(drvinfo->bus_info)-1)[sizeof(drvinfo->bus_info)-1] = 0;
}

uint32_t __cold dpa_get_msglevel(struct net_device *net_dev)
{
	return ((struct dpa_priv_s *)netdev_priv(net_dev))->msg_enable;
}

void __cold dpa_set_msglevel(struct net_device *net_dev, uint32_t msg_enable)
{
	((struct dpa_priv_s *)netdev_priv(net_dev))->msg_enable = msg_enable;
}

int __cold dpa_nway_reset(struct net_device *net_dev)
{
	int			 _errno;
	struct dpa_priv_s	*priv;

	priv = netdev_priv(net_dev);

	if (priv->mac_dev == NULL) {
		netdev_info(net_dev, "This is a MAC-less interface\n");
		return -ENODEV;
	}
	if (unlikely(priv->mac_dev->phy_dev == NULL)) {
		netdev_err(net_dev, "phy device not initialized\n");
		return -ENODEV;
	}

	_errno = 0;
	if (priv->mac_dev->phy_dev->autoneg) {
		_errno = phy_start_aneg(priv->mac_dev->phy_dev);
		if (unlikely(_errno < 0))
			netdev_err(net_dev, "phy_start_aneg() = %d\n",
					_errno);
	}

	return _errno;
}

void __cold dpa_get_ringparam(struct net_device *net_dev,
		struct ethtool_ringparam *et_ringparam)
{
	et_ringparam->rx_max_pending	   = 0;
	et_ringparam->rx_mini_max_pending  = 0;
	et_ringparam->rx_jumbo_max_pending = 0;
	et_ringparam->tx_max_pending	   = 0;

	et_ringparam->rx_pending	   = 0;
	et_ringparam->rx_mini_pending	   = 0;
	et_ringparam->rx_jumbo_pending	   = 0;
	et_ringparam->tx_pending	   = 0;
}

void __cold dpa_get_pauseparam(struct net_device *net_dev,
		struct ethtool_pauseparam *et_pauseparam)
{
	struct dpa_priv_s	*priv;

	priv = netdev_priv(net_dev);

	if (priv->mac_dev == NULL) {
		netdev_info(net_dev, "This is a MAC-less interface\n");
		return;
	}
	if (unlikely(priv->mac_dev->phy_dev == NULL)) {
		netdev_err(net_dev, "phy device not initialized\n");
		return;
	}

	et_pauseparam->autoneg	= priv->mac_dev->autoneg_pause;
	et_pauseparam->rx_pause	= priv->mac_dev->rx_pause;
	et_pauseparam->tx_pause	= priv->mac_dev->tx_pause;
}

int __cold dpa_set_pauseparam(struct net_device *net_dev,
		struct ethtool_pauseparam *et_pauseparam)
{
	struct dpa_priv_s	*priv;
	int _errno;
	bool en;

	priv = netdev_priv(net_dev);

	if (priv->mac_dev == NULL) {
		netdev_info(net_dev, "This is a MAC-less interface\n");
		return -ENODEV;
	}
	if (unlikely(priv->mac_dev->phy_dev == NULL)) {
		netdev_err(net_dev, "phy device not initialized\n");
		return -ENODEV;
	}

	en = et_pauseparam->rx_pause ? true : false;
	_errno = priv->mac_dev->set_rx_pause(
			priv->mac_dev->get_mac_handle(priv->mac_dev), en);
	if (unlikely(_errno < 0)) {
		netdev_err(net_dev, "set_rx_pause() = %d\n", _errno);
		return _errno;
	}

	en = et_pauseparam->tx_pause ? true : false;
	_errno = priv->mac_dev->set_tx_pause(
			priv->mac_dev->get_mac_handle(priv->mac_dev), en);
	if (unlikely(_errno < 0)) {
		netdev_err(net_dev, "set_tx_pause() = %d\n", _errno);
		return _errno;
	}

	priv->mac_dev->autoneg_pause = et_pauseparam->autoneg;
	priv->mac_dev->rx_pause = et_pauseparam->rx_pause;
	priv->mac_dev->tx_pause = et_pauseparam->tx_pause;

	return 0;
}

const struct ethtool_ops dpa_ethtool_ops = {
	.get_settings = dpa_get_settings,
	.set_settings = dpa_set_settings,
	.get_drvinfo = dpa_get_drvinfo,
	.get_msglevel = dpa_get_msglevel,
	.set_msglevel = dpa_set_msglevel,
	.nway_reset = dpa_nway_reset,
	.get_ringparam = dpa_get_ringparam,
	.get_pauseparam = dpa_get_pauseparam,
	.set_pauseparam = dpa_set_pauseparam,
	.self_test = NULL, /* TODO invoke the cold-boot unit-test? */
	.get_ethtool_stats = NULL, /* TODO other stats, currently in debugfs */
	.get_link = ethtool_op_get_link,
};

