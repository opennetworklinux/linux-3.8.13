/*
 * drivers/net/gianfar_1588.c
 *
 * Copyright 2008-2012 Freescale Semiconductor, Inc.
 * Copyright 2009-2011 IXXAT Automation GmbH
 *
 * Author: Anup Gangwar <anup.gangwar@freescale.com>
 *	   Yashpal Dutta <yashpal.dutta@freescale.com>
 *
 * Gianfar Ethernet Driver -- IEEE 1588 interface functionality
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/vmalloc.h>
#include <linux/of.h>
#include <linux/if_vlan.h>
#include <linux/net_tstamp.h>
#include <linux/of_platform.h>
#include <linux/proc_fs.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/udp.h>
#include "gianfar.h"

static struct gfar_ptp_attr_t ptp_attr;

static bool gfar_ptp_init_circ(struct gfar_ptp_circular *buf, int size)
{
	struct circ_buf *circ_buf = &buf->circ_buf;
	circ_buf->buf = vmalloc(size * sizeof(struct gfar_ptp_data));

	if (!circ_buf->buf)
		return 1;

	circ_buf->head = 0;
	circ_buf->tail = 0;
	buf->size = size;
	spin_lock_init(&buf->ptp_lock);

	return 0;
}

static bool gfar_ptp_is_full(struct gfar_ptp_circular *buf)
{
	struct circ_buf *circ_buf = &buf->circ_buf;

	return (CIRC_SPACE(circ_buf->head, circ_buf->tail,
				buf->size) == 0);
}

static bool gfar_ptp_is_empty(struct gfar_ptp_circular *buf)
{
	struct circ_buf *circ_buf = &buf->circ_buf;

	return (CIRC_CNT(circ_buf->head, circ_buf->tail,
				buf->size) == 0);
}

static bool gfar_ptp_insert(struct gfar_ptp_circular *buf,
			struct gfar_ptp_data *data)
{
	struct gfar_ptp_data *tmp;
	struct circ_buf *circ_buf = &buf->circ_buf;
	unsigned int head;
	unsigned long flags;

	spin_lock_irqsave(&buf->ptp_lock, flags);
	if (gfar_ptp_is_full(buf)) {
		spin_unlock_irqrestore(&buf->ptp_lock, flags);
		return 1;
	}

	head = circ_buf->head;
	tmp = (struct gfar_ptp_data *)circ_buf->buf + head;
	memcpy(tmp, data, sizeof(struct gfar_ptp_data));
	circ_buf->head = (head + 1) & (buf->size - 1);
	spin_unlock_irqrestore(&buf->ptp_lock, flags);

	return 0;
}

static int gfar_ptp_is_ident_match(struct gfar_ptp_ident *dst,
				  struct gfar_ptp_ident *src)
{
	int ret;

	if ((dst->version != src->version) ||
			(dst->msg_type != src->msg_type))
		return 0;

	if ((dst->netw_prot == src->netw_prot)
			|| src->netw_prot == GFAR_PTP_PROT_DONTCARE) {
		if (dst->seq_id != src->seq_id)
			return 0;

		ret = memcmp(dst->snd_port_id, src->snd_port_id,
				GFAR_PTP_SOURCE_PORT_LENGTH);
		if (ret)
			return 0;
		else
			return 1;
	}

	return 0;
}

static bool gfar_ptp_find_and_remove(struct gfar_ptp_circular *buf,
				struct gfar_ptp_ident *ident,
				struct gfar_ptp_time *ts)
{
	struct circ_buf *circ_buf = &buf->circ_buf;
	unsigned int size = buf->size;
	unsigned int head, tail, idx;
	struct gfar_ptp_data *tmp, *tmp2;
	struct gfar_ptp_ident *tmp_ident;
	unsigned long flags;

	spin_lock_irqsave(&buf->ptp_lock, flags);
	if (gfar_ptp_is_empty(buf)) {
		spin_unlock_irqrestore(&buf->ptp_lock, flags);
		return 1;
	}

	head = circ_buf->head;
	tail = idx = circ_buf->tail;

	while (idx != head) {
		tmp = (struct gfar_ptp_data *)(circ_buf->buf) + idx;
		tmp_ident = &tmp->ident;
		if (gfar_ptp_is_ident_match(tmp_ident, ident))
				break;

		/* get next */
		idx = (idx + 1) & (size - 1);
	}

	/* not found ? */
	if (idx == head) {
		spin_unlock_irqrestore(&buf->ptp_lock, flags);
		return 1;
	}

	*ts = tmp->ts;

	if (idx != tail) {
		if (CIRC_CNT(idx, tail, size) > TS_ACCUMULATION_THRESHOLD) {
			tail = circ_buf->tail =
				(idx - TS_ACCUMULATION_THRESHOLD) & (size - 1);
		}

		while (CIRC_CNT(idx, tail, size) > 0) {
			tmp = (struct gfar_ptp_data *)(circ_buf->buf) + idx;
			idx = (idx - 1) & (size - 1);
			tmp2 = (struct gfar_ptp_data *)(circ_buf->buf) + idx;
			*tmp = *tmp2;
		}
	}

	/* set tail pointer to postion after found */
	circ_buf->tail = (tail + 1) & (size - 1);

	spin_unlock_irqrestore(&buf->ptp_lock, flags);

	return 0;
}

/*
 * Parse packets if they are PTP.
 * The PTP header can be found in an IPv4, IPv6 or in an IEEE802.3
 * ethernet frame. The function returns the position of the PTP packet
 * or NULL, if no PTP found.
 */
static void *gfar_ptp_parse_packet(struct sk_buff *skb, u16 *eth_type)
{
	struct iphdr *iph;
	struct udphdr *udph;
	struct ipv6hdr *ipv6h;
	void *pos = skb->data + ETH_ALEN + ETH_ALEN;
	u8 *ptp_loc = NULL;

	*eth_type = be16_to_cpup(pos);

	/* Check if outer vlan tag is here */
	if (*eth_type == GFAR_VLAN_QINQ_1 ||
			*eth_type == GFAR_VLAN_QINQ_2 ||
			*eth_type == GFAR_VLAN_QINQ_3 ||
			*eth_type == GFAR_VLAN_QINQ_4) {
		pos += GFAR_VLAN_TAG_LEN;
		*eth_type = be16_to_cpup(pos);
	}

	/* Check if inner tag is here */
	if (*eth_type == ETH_P_8021Q) {
		pos += GFAR_VLAN_TAG_LEN;
		*eth_type = be16_to_cpup(pos);
	}

	/* set pos after ethertype */
	pos += GFAR_ETHTYPE_LEN;

	switch (*eth_type) {
	case ETH_P_1588:
		ptp_loc = pos;
		if ((ptp_loc[0] & 0xF) <= 3) {
			/* long enough ? */
			if (skb->len >= ((ptp_loc - skb->data) +
						GFAR_PTP_HEADER_SZE))
				return ptp_loc;
		}
		break;
	case ETH_P_IP:
		iph = (struct iphdr *)pos;

		if (ntohs(iph->protocol) != IPPROTO_UDP)
			return NULL;

		pos += iph->ihl * 4;
		udph = (struct udphdr *)pos;

		/*
		 * check the destination port address
		 * ( 319 (0x013F) = PTP event port )
		 */
		if (ntohs(udph->dest) != GFAR_PTP_EVENT_PORT)
			return NULL;

		ptp_loc = pos + sizeof(struct udphdr);
		/* long enough ? */
		if (skb->len >= ((ptp_loc - skb->data)
				+ GFAR_PTP_HEADER_SZE))
			return ptp_loc;
		break;
	case ETH_P_IPV6:
		ipv6h = (struct ipv6hdr *)pos;

		if (ntohs(ipv6h->nexthdr) != IPPROTO_UDP)
			return NULL;

		pos += sizeof(struct ipv6hdr);
		udph = (struct udphdr *)pos;

		/*
		 * check the destination port address
		 * ( 319 (0x013F) = PTP event port )
		 */
		if (ntohs(udph->dest) != GFAR_PTP_EVENT_PORT)
			return NULL;

		ptp_loc = pos + sizeof(struct udphdr);
		/* long enough ? */
		if (skb->len >= ((ptp_loc - skb->data)
				+ GFAR_PTP_HEADER_SZE))
			return ptp_loc;
		break;
	default:
		break;
	}

	return NULL; /* no PTP frame */
}

/*
 * Store the tx hardware timestamp with additional information
 * in the gfar internal ringbuffer for timestamps.
 */
void gfar_ptp_store_txstamp(struct net_device *dev, struct sk_buff *skb,
						struct gfar_ptp_time *tx_ts)
{
	struct gfar_ptp_data tmp_tx_time;
	struct gfar_private *priv = netdev_priv(dev);
	void  *ptp_loc;
	u16 eth_type;

	ptp_loc = gfar_ptp_parse_packet(skb, &eth_type);
	if (ptp_loc == NULL)
		return;

	/* store identification data */
	switch (eth_type) {
	case ETH_P_IP:
		tmp_tx_time.ident.netw_prot = GFAR_PTP_PROT_IPV4;
		break;
	case ETH_P_IPV6:
		tmp_tx_time.ident.netw_prot = GFAR_PTP_PROT_IPV6;
		break;
	case ETH_P_1588:
		tmp_tx_time.ident.netw_prot = GFAR_PTP_PROT_802_3;
		break;
	default:
		return;
	}

	tmp_tx_time.ident.version = (*(u8 *)(ptp_loc + 1)) & 0X0F;
	tmp_tx_time.ident.msg_type = (*(u8 *)(ptp_loc)) & 0x0F;
	tmp_tx_time.ident.seq_id = be16_to_cpup(ptp_loc +
					GFAR_PTP_HEADER_SEQ_OFFS);
	memcpy(tmp_tx_time.ident.snd_port_id,
	ptp_loc + GFAR_PTP_SPID_OFFS, GFAR_PTP_SOURCE_PORT_LENGTH);

	/* store tx timestamp */
	tmp_tx_time.ts = *tx_ts;

	/* insert timestamp in circular buffer */
	gfar_ptp_insert(&(priv->tx_timestamps), &tmp_tx_time);
}

/*
 * Store the rx hardware timestamp with additional information
 * in the gfar internal ringbuffer for timestamps.
 */
void gfar_ptp_store_rxstamp(struct net_device *dev, struct sk_buff *skb,
						struct gfar_ptp_time *rx_ts)
{
	struct gfar_ptp_data tmp_rx_time;
	struct gfar_private *priv = netdev_priv(dev);
	void *ptp_loc;
	u16 eth_type;

	ptp_loc = gfar_ptp_parse_packet(skb, &eth_type);
	if (ptp_loc == NULL)
		return;

	/* store identification data */
	switch (eth_type) {
	case ETH_P_IP:
		tmp_rx_time.ident.netw_prot = GFAR_PTP_PROT_IPV4;
		break;
	case ETH_P_IPV6:
		tmp_rx_time.ident.netw_prot = GFAR_PTP_PROT_IPV6;
		break;
	case ETH_P_1588:
		tmp_rx_time.ident.netw_prot = GFAR_PTP_PROT_802_3;
		break;
	default:
		return;
	}

	tmp_rx_time.ident.version = (*(u8 *)(ptp_loc + 1)) & 0X0F;
	tmp_rx_time.ident.msg_type = (*(u8 *)(ptp_loc)) & 0x0F;
	tmp_rx_time.ident.seq_id = be16_to_cpup(ptp_loc +
					GFAR_PTP_HEADER_SEQ_OFFS);
	memcpy(tmp_rx_time.ident.snd_port_id,
		ptp_loc + GFAR_PTP_SPID_OFFS,
		GFAR_PTP_SOURCE_PORT_LENGTH);

	/* store rx timestamp */
	tmp_rx_time.ts = *rx_ts;

	/* insert timestamp in circular buffer */
	gfar_ptp_insert(&(priv->rx_timestamps), &tmp_rx_time);
}

/*
 * nominal_frequency - This function calculates the nominal frequency.
 * nominal frequency is the desired clock frequency.
 * @sysclock_freq: Timer Oscillator Frequency
 *
 * Description:
 *  Returns the nominal frequency which is calculated on the following
 *  basis.
 *  nominal frequency should be less than the Timer Oscillator frequency.
 *  nominal frequency should be a factor of 1000.
 *
 *  Eg If Timer Oscillator frequency is 400.
 *     then nominal frequency can be 200.
 *
 *     If Timer Oscillator frequency is 600.
 *     then nominal frequency can be 500.
 *
 *     If Timer Oscillator frequency is 333.
 *     then nominal frequency can be 200.
 */
static u32 nominal_frequency(u32 sysclock_freq)
{
	u32 remainder = 0;

	sysclock_freq /= 1000000;
	remainder = sysclock_freq % 100;
	if (remainder) {
		sysclock_freq = sysclock_freq - remainder;
		sysclock_freq += 100;
	}

	while ((1000 % (sysclock_freq -= 100)))
		continue;

	return sysclock_freq * 1000000;
}

static int gfar_ptp_cal_attr(u32 sysclk_freq)
{
	ptp_attr.sysclock_freq = sysclk_freq;

	ptp_attr.nominal_freq =
		nominal_frequency(ptp_attr.sysclock_freq);
	ptp_attr.tmr_fiper1 = ONE_GIGA;

	/* TCLK_PERIOD = 10^9/Nominal_Frequency in HZ */
	ptp_attr.tclk_period =
		1000000000 / ptp_attr.nominal_freq;

	return 0;
}

/*set Fiper Trigger Alarm */
static void gfar_set_fiper_alarm(struct net_device *dev,
		struct gfar_ptp_time *alarm)
{
	struct gfar_private *priv = netdev_priv(dev);
	u64 act_time;

	act_time = alarm->sec * NANOSEC_PER_SEC + alarm->nsec;
	gfar_write(&(priv->ptimer->tmr_alarm1_l), lower_32_bits(act_time));
	gfar_write(&(priv->ptimer->tmr_alarm1_h), upper_32_bits(act_time));

	gfar_write(&(priv->ptimer->tmr_fiper1), ptp_attr.tmr_fiper1
			- ptp_attr.tclk_period);
}

/* Set the 1588 timer counter registers */
static void gfar_set_1588cnt(struct net_device *dev,
				struct gfar_ptp_time *gfar_time)
{
	struct gfar_private *priv = netdev_priv(dev);
	u64 act_time;

	act_time = gfar_time->sec * NANOSEC_PER_SEC + gfar_time->nsec;

	/* We must write the tmr_cnt_l register first */
	gfar_write(&priv->ptimer->tmr_cnt_l, lower_32_bits(act_time));
	gfar_write(&priv->ptimer->tmr_cnt_h, upper_32_bits(act_time));

	/* restart fiper two second later */
	gfar_time->sec += 2;
	gfar_time->nsec = 0;
	gfar_set_fiper_alarm(dev, gfar_time);
}

void gfar_cnt_to_ptp_time(u32 high, u32 low, struct gfar_ptp_time *ptp_time)
{
	u64 sec;
	u32 nsec;

	sec = make64(high, low);
	nsec = do_div(sec, NANOSEC_PER_SEC);
	ptp_time->nsec = nsec;
	ptp_time->sec = sec;
}

static void gfar_get_curr_cnt(struct gfar_regs_1588 __iomem *ptimer,
					struct gfar_ptp_time *curr_time)
{
	u32 high1, high2, low1, low2;

	/* do not accept a time at the rollover from low to high */
	do {
		high1 = gfar_read(&ptimer->tmr_cnt_h);
		low1  = gfar_read(&ptimer->tmr_cnt_l);
		high2 = gfar_read(&ptimer->tmr_cnt_h);
		low2 = gfar_read(&ptimer->tmr_cnt_l);
	} while (high1 != high2);

	gfar_cnt_to_ptp_time(high2, low2, curr_time);
}

/* Get both the time-stamps and use the larger one */
u64 gfar_get_tx_timestamp(struct gfar __iomem *regs)
{
	u32 high1, high2, low1, low2;
	u64 ts1, ts2;

	/* Read the nsec register first */
	low1 = gfar_read(&regs->tmr_txts1_l);
	high1 = gfar_read(&regs->tmr_txts1_h);
	low2 = gfar_read(&regs->tmr_txts2_l);
	high2 = gfar_read(&regs->tmr_txts2_h);

	ts1 = make64(high1, low1);
	ts2 = make64(high2, low2);

	return (ts1 > ts2) ? ts1 : ts2;
}

int gfar_ioctl_1588(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct gfar_private *priv = netdev_priv(dev);
	struct gfar_ptp_time act_time;
	struct gfar_ptp_data ptp_ts_data;
	struct gfar_ptp_data *ptp_dat_user;
	struct gfar __iomem *regs = priv->gfargrp[0].regs;
	u32 adj;

	if (!(priv->device_flags & FSL_GIANFAR_DEV_HAS_TIMER))
		return -ERANGE;

	switch (cmd) {
	case PTP_ENBL_TXTS_IOCTL:
		priv->hwts_tx_en_ioctl = 1;
		/* add RTPE bit - enable timestamp insertion on tx packets */
		gfar_write(&(priv->ptimer->tmr_ctrl),
			gfar_read(&(priv->ptimer->tmr_ctrl)) | TMR_RTPE);
		break;
	case PTP_DSBL_TXTS_IOCTL:
		priv->hwts_tx_en_ioctl = 0;
		/*
		 * remove RTPE bit - disable timestamp insertion
		 * on tx packets
		 */
		gfar_write(&(priv->ptimer->tmr_ctrl),
			gfar_read(&(priv->ptimer->tmr_ctrl)) & ~TMR_RTPE);
		break;
	case PTP_ENBL_RXTS_IOCTL:
		priv->hwts_rx_en_ioctl = 1;
		gfar_write(&regs->rctrl,
				gfar_read(&regs->rctrl) | RCTRL_TS_ENABLE);
		break;
	case PTP_DSBL_RXTS_IOCTL:
		priv->hwts_rx_en_ioctl = 0;
		gfar_write(&regs->rctrl,
				gfar_read(&regs->rctrl) & ~RCTRL_TS_ENABLE);
		break;
	case PTP_GET_TX_TIMESTAMP:
		ptp_dat_user = (struct gfar_ptp_data *)ifr->ifr_data;
		if (copy_from_user(&ptp_ts_data.ident,
			&ptp_dat_user->ident, sizeof(ptp_ts_data.ident)))
			return -EINVAL;

		if (gfar_ptp_find_and_remove(&(priv->tx_timestamps),
				&ptp_ts_data.ident, &ptp_ts_data.ts))
			return -EAGAIN;

		if (copy_to_user(&ptp_dat_user->ts, &ptp_ts_data.ts,
						sizeof(ptp_ts_data.ts)))
			return -EFAULT;
		break;
	case PTP_GET_RX_TIMESTAMP:
		ptp_dat_user = (struct gfar_ptp_data *)ifr->ifr_data;
		if (copy_from_user(&ptp_ts_data.ident,
			&ptp_dat_user->ident, sizeof(ptp_ts_data.ident)))
			return -EINVAL;

		if (gfar_ptp_find_and_remove(&(priv->rx_timestamps),
				&ptp_ts_data.ident, &ptp_ts_data.ts))
			return -EAGAIN;

		if (copy_to_user(&ptp_dat_user->ts,
				&ptp_ts_data.ts, sizeof(ptp_ts_data.ts)))
			return -EFAULT;
		break;
	case PTP_SET_TIME:
		if (copy_from_user(&act_time, ifr->ifr_data,
							sizeof(act_time)))
			return -EINVAL;

		gfar_set_1588cnt(dev, &act_time);
		break;
	case PTP_GET_TIME:
		gfar_get_curr_cnt(priv->ptimer, &act_time);
		if (copy_to_user(ifr->ifr_data, &act_time,
					sizeof(act_time)))
			return -EFAULT;
		break;
	case PTP_SET_FIPER_ALARM:
		if (copy_from_user(&act_time, ifr->ifr_data,
							sizeof(act_time)))
			return -EINVAL;

		gfar_set_fiper_alarm(dev, &act_time);
		break;
	case PTP_SET_ADJ:
		if (copy_from_user(&adj, ifr->ifr_data, sizeof(adj)))
			return -EINVAL;

		/* assign new value directly */
		gfar_write(&priv->ptimer->tmr_add, adj);
		break;
	case PTP_GET_ADJ:
		/*
		 * return initial timer add value
		 * to calculate drift correction
		 */
		if (copy_to_user(ifr->ifr_data, &ptp_attr.freq_comp,
					sizeof(ptp_attr.freq_comp)))
			return -EFAULT;
		break;
	case PTP_CLEANUP_TS:
		/* reset tx-timestamping buffer */
		priv->tx_timestamps.circ_buf.head = 0;
		priv->tx_timestamps.circ_buf.tail = 0;
		priv->tx_timestamps.size = DEFAULT_PTP_TX_BUF_SZ;
		/* reset rx-timestamping buffer */
		priv->rx_timestamps.circ_buf.head = 0;
		priv->rx_timestamps.circ_buf.tail = 0;
		priv->rx_timestamps.size = DEFAULT_PTP_RX_BUF_SZ;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int get_of_u32(struct device_node *node, char *str, u32 *val)
{
	int plen;
	const u32 *prop = of_get_property(node, str, &plen);

	if (!prop || plen != sizeof(*prop))
		return -1;

	*val = *prop;

	return 0;
}

/*
 * Resource required for accessing 1588 Timer Registers. There are few 1588
 * modules registers which are present in eTSEC1 memory space only. The second
 * reg entry there in denotes the 1588 regs.
 */
int gfar_ptp_init(struct device_node *np, struct gfar_private *priv)
{
	struct device_node *node;
	const phandle *timer_handle;
	u32 tmr_prsc, cksel, sysclk;
	u64 freq_comp;

	timer_handle = of_get_property(np, "ptimer-handle", NULL);
	if (!timer_handle)
		return -ENODEV;

	node = of_find_node_by_phandle(*timer_handle);
	if (!node)
		return -ENODEV;

	if (get_of_u32(node, "fsl,tmr-prsc", &tmr_prsc) ||
		get_of_u32(node, "timer-frequency", &sysclk) ||
		get_of_u32(node, "fsl,clock-source-select", &cksel))
		return -ENODEV;

	priv->ptimer = of_iomap(node, 0);
	if (!priv->ptimer)
		return -ENOMEM;

	gfar_ptp_cal_attr(sysclk);

	if (of_get_property(node, "fsl,ts-to-buffer", NULL))
		priv->device_flags |= FSL_GIANFAR_DEV_HAS_TS_TO_BUFFER;

	gfar_write(&(priv->ptimer->tmr_prsc), tmr_prsc);
	gfar_write(&(priv->ptimer->tmr_fiper1), ptp_attr.tmr_fiper1
		 - ptp_attr.tclk_period);

	gfar_write(&(priv->ptimer->tmr_alarm1_l), TMR_ALARM1_L);
	gfar_write(&(priv->ptimer->tmr_alarm1_h), TMR_ALARM1_H);

	/* Need to mask the TCLK bits as they are initialized with 1 */
	gfar_write(&(priv->ptimer->tmr_ctrl),
		(gfar_read(&(priv->ptimer->tmr_ctrl))
			 & ~TMR_CTRL_TCLK_MASK) | (ptp_attr.tclk_period << 16));

	/*
	 * initialize TMR_ADD with the initial frequency compensation value:
	 * freq_compensation = ceil(2^32 / frequency ratio)
	 * frequency ratio = sysclock frequency / nominal frequency
	 */
	freq_comp = ((u64)2 << 31) * ptp_attr.nominal_freq;
	if (do_div(freq_comp, ptp_attr.sysclock_freq))
		freq_comp++;
	ptp_attr.freq_comp = lower_32_bits(freq_comp);
	gfar_write(&(priv->ptimer->tmr_add), ptp_attr.freq_comp);

	gfar_write(&(priv->ptimer->tmr_ctrl),
		(gfar_read(&(priv->ptimer->tmr_ctrl)) & ~TMR_CTRL_CKSEL_MASK) |
		TMR_CTRL_ENABLE | cksel | TMR_CTRL_FIPER_START);

	/* initialize circular buffer for tx timestamps */
	if (gfar_ptp_init_circ(&(priv->tx_timestamps),
					DEFAULT_PTP_TX_BUF_SZ))
		goto txbuf;

	/* initialize circular buffer for rx timestamps */
	if (gfar_ptp_init_circ(&(priv->rx_timestamps),
					DEFAULT_PTP_RX_BUF_SZ))
		goto rxbuf;

	gfar_1588_start(priv);

	return 0;

rxbuf:
	vfree(priv->tx_timestamps.circ_buf.buf);
txbuf:
	iounmap(priv->ptimer);

	return -EBUSY;
}

void gfar_ptp_cleanup(struct gfar_private *priv)
{
	if (priv->ptimer)
		iounmap(priv->ptimer);

	if (priv->tx_timestamps.circ_buf.buf)
		vfree(priv->tx_timestamps.circ_buf.buf);

	if (priv->rx_timestamps.circ_buf.buf)
		vfree(priv->rx_timestamps.circ_buf.buf);
}

/* 1588 Module intialization */
void gfar_1588_start(struct gfar_private *priv)
{
	if (!priv->ptimer)
		return;

	priv->hwts_tx_en = 0;
	priv->hwts_rx_en = 0;
	priv->hwts_tx_en_ioctl = 0;
	priv->hwts_rx_en_ioctl = 0;

	priv->device_flags |= FSL_GIANFAR_DEV_HAS_TIMER;
}

/* When PTP is disabled this routing is called */
void gfar_1588_stop(struct gfar_private *priv)
{
	if (!(priv->device_flags & FSL_GIANFAR_DEV_HAS_TIMER))
		return;

	priv->hwts_tx_en = 0;
	priv->hwts_rx_en = 0;
	priv->hwts_tx_en_ioctl = 0;
	priv->hwts_rx_en_ioctl = 0;

	priv->device_flags &= ~FSL_GIANFAR_DEV_HAS_TIMER;
}
