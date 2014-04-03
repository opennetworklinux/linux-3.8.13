/*
 * Copyright 2008-2012 Freescale Semiconductor Inc.
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

#ifndef __DPA_COMMON_H
#define __DPA_COMMON_H

#include <linux/kernel.h>	/* pr_*() */
#include <linux/device.h>	/* dev_*() */
#include <linux/smp.h>		/* smp_processor_id() */

#define __hot

/* Simple enum of FQ types - used for array indexing */
enum port_type {RX, TX};

/* More detailed FQ types - used for fine-grained WQ assignments */
enum dpa_fq_type {
	FQ_TYPE_RX_DEFAULT = 1, /* Rx Default FQs */
	FQ_TYPE_RX_ERROR,       /* Rx Error FQs */
	FQ_TYPE_RX_PCD,         /* User-defined PCDs */
	FQ_TYPE_TX,             /* "Real" Tx FQs */
	FQ_TYPE_TX_CONFIRM,     /* Tx Confirmation FQs (actually Rx FQs) */
	FQ_TYPE_TX_ERROR,       /* Tx Error FQs (these are actually Rx FQs) */
#ifdef CONFIG_FSL_DPAA_TX_RECYCLE
	FQ_TYPE_TX_RECYCLE,	/* Tx FQs for recycleable frames only */
#endif
};

/* TODO: This structure should be renamed & moved to the FMD wrapper */
struct dpa_buffer_layout_s {
	uint16_t	priv_data_size;
	bool		parse_results;
	bool		time_stamp;
	bool		hash_results;
	uint8_t		manip_extra_space;
	uint16_t	data_align;
};

#define DPA_TX_PRIV_DATA_SIZE	16
#define DPA_PARSE_RESULTS_SIZE sizeof(t_FmPrsResult)
#define DPA_TIME_STAMP_SIZE 8
#define DPA_HASH_RESULTS_SIZE 8


#define dpaa_eth_init_port(type, port, param, errq_id, defq_id, buf_layout,\
			   frag_enabled) \
{ \
	param.errq = errq_id; \
	param.defq = defq_id; \
	param.priv_data_size = buf_layout->priv_data_size; \
	param.parse_results = buf_layout->parse_results; \
	param.hash_results = buf_layout->hash_results; \
	param.frag_enable = frag_enabled; \
	param.time_stamp = buf_layout->time_stamp; \
	param.manip_extra_space = buf_layout->manip_extra_space; \
	param.data_align = buf_layout->data_align; \
	fm_set_##type##_port_params(port, &param); \
}

#endif	/* __DPA_COMMON_H */
