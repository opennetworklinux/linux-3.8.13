/* Copyright 2008-2013 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
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

/*
 * DPA Statistics Application Programming Interface implementation
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include "lnxwrp_fm.h"
#include "dpaa_eth.h"

/* DPA offloading layer includes */
#include "dpa_stats.h"
#include "dpa_classifier.h"

/* FMD includes */
#include "fm_pcd_ext.h"

#define STATS_VAL_SIZE 4
#define UNSUPPORTED_CNT_SEL -1
#define CLASSIF_STATS_SHIFT 4
#define WORKQUEUE_MAX_ACTIVE 3

/* Global dpa_stats component */
struct dpa_stats *gbl_dpa_stats;

static int get_cnt_cls_tbl_frag_stats(struct dpa_stats_req_cb *req_cb,
				      struct dpa_stats_cnt_cb *cnt_cb);

static int get_cnt_cls_tbl_match_stats(struct dpa_stats_req_cb *req_cb,
				       struct dpa_stats_cnt_cb *cnt_cb);

static int get_cnt_cls_tbl_hash_stats(struct dpa_stats_req_cb *req_cb,
				      struct dpa_stats_cnt_cb *cnt_cb);

static int get_cnt_cls_tbl_index_stats(struct dpa_stats_req_cb *req_cb,
				       struct dpa_stats_cnt_cb *cnt_cb);

static int get_cnt_ccnode_match_stats(struct dpa_stats_req_cb *req_cb,
				      struct dpa_stats_cnt_cb *cnt_cb);

static int get_cnt_ccnode_hash_stats(struct dpa_stats_req_cb *req_cb,
				     struct dpa_stats_cnt_cb *cnt_cb);

static int get_cnt_ccnode_index_stats(struct dpa_stats_req_cb *req_cb,
				      struct dpa_stats_cnt_cb *cnt_cb);

static void async_req_work_func(struct work_struct *work);

/* check that the provided params are valid */
static int check_dpa_stats_params(const struct dpa_stats_params *params)
{
	/* Check init parameters */
	if (!params) {
		pr_err("Invalid DPA Stats parameters handle\n");
		return -EINVAL;
	}

	/* There must be at least one counter */
	if (params->max_counters == 0 ||
	    params->max_counters > DPA_STATS_MAX_NUM_OF_COUNTERS) {
		pr_err("Invalid DPA Stats number of counters\n");
		return -EDOM;
	}

	if (!params->storage_area) {
		pr_err("Invalid DPA Stats storage area\n");
		return -EINVAL;
	}

	if (params->storage_area_len < STATS_VAL_SIZE) {
		pr_err("Invalid DPA Stats storage area length\n");
		return -EINVAL;
	}

	return 0;
}

static int check_tbl_cls_counter(struct dpa_stats_cnt_cb *cnt_cb,
				 struct dpa_stats_lookup_key *entry)
{
	t_FmPcdCcKeyStatistics stats;
	int err;

	switch (cnt_cb->tbl_cb.type) {
	case DPA_CLS_TBL_HASH:
		err = FM_PCD_HashTableFindNGetKeyStatistics(entry->cc_node,
				entry->key.size, entry->key.byte, &stats);
		if (err != 0) {
			pr_err("Couldn't retrieve Classif Table statistics\n");
			return -EIO;
		}
		cnt_cb->f_get_cnt_stats = get_cnt_cls_tbl_hash_stats;
		break;
	case DPA_CLS_TBL_INDEXED:
		err = FM_PCD_MatchTableGetKeyStatistics(
				entry->cc_node, entry->key.byte[0], &stats);
		if (err != 0) {
			pr_err("Invalid Classifier Table counter parameters\n");
			return -EIO;
		}
		cnt_cb->f_get_cnt_stats = get_cnt_cls_tbl_index_stats;
		break;
	case DPA_CLS_TBL_EXACT_MATCH:
		err = FM_PCD_MatchTableFindNGetKeyStatistics(entry->cc_node,
				entry->key.size, entry->key.byte,
				entry->key.mask, &stats);
		if (err != 0) {
			pr_err("Invalid Classifier Table counter parameters\n");
			return -EINVAL;
		}
		cnt_cb->f_get_cnt_stats = get_cnt_cls_tbl_match_stats;
		break;
	default:
		pr_err("Invalid table type\n");
		return -EINVAL;
	}
	return 0;
}

static int check_ccnode_counter(struct dpa_stats_cnt_cb *cnt_cb,
				enum dpa_stats_classif_node_type ccnode_type,
				struct dpa_offload_lookup_key *key)
{
	t_FmPcdCcKeyStatistics stats;
	int err;

	switch (ccnode_type) {
	case DPA_STATS_CLASSIF_NODE_HASH:
		err = FM_PCD_HashTableFindNGetKeyStatistics(
				cnt_cb->ccnode_cb.cc_node,
				key->size, key->byte, &stats);
		if (err != 0) {
			pr_err("Couldn't retrieve Classif Table statistics\n");
			return -EIO;
		}
		cnt_cb->f_get_cnt_stats = get_cnt_ccnode_hash_stats;
		break;
	case DPA_STATS_CLASSIF_NODE_INDEXED:
		err = FM_PCD_MatchTableGetKeyStatistics(
				cnt_cb->ccnode_cb.cc_node,
				key->byte[0], &stats);
		if (err != 0) {
			pr_err("Invalid Classifier Table counter parameters\n");
			return -EIO;
		}
		cnt_cb->f_get_cnt_stats = get_cnt_ccnode_index_stats;
		break;
	case DPA_STATS_CLASSIF_NODE_EXACT_MATCH:
		err = FM_PCD_MatchTableFindNGetKeyStatistics(
				cnt_cb->ccnode_cb.cc_node, key->size, key->byte,
				key->mask, &stats);
		if (err != 0) {
			pr_err("Invalid Classifier Table counter parameters\n");
			return -EINVAL;
		}
		cnt_cb->f_get_cnt_stats = get_cnt_ccnode_match_stats;
		break;
	default:
		pr_err("Invalid table type\n");
		return -EINVAL;
	}
	return 0;
}

static int get_new_cnt(struct dpa_stats *dpa_stats,
		       struct dpa_stats_cnt_cb **cnt_cb)
{
	struct dpa_stats_cnt_cb *new_cnt;
	uint32_t id;
	int i;

	/* Acquire DPA Stats instance lock */
	mutex_lock(&dpa_stats->lock);

	/* Get an id for new Counter */
	if (cq_get_4bytes(dpa_stats->cnt_id_cq, &id) < 0) {
		pr_err("No more unused counter ids\n");
		mutex_unlock(&dpa_stats->lock);
		return -EDOM;
	}

	/*
	 * Use 'used_cnt_ids' array in order to store counter ids that are
	 * 'in use' . Array can be further used to remove counters
	 */
	for (i = 0; i < dpa_stats->config.max_counters; i++)
		if (dpa_stats->used_cnt_ids[i] == DPA_OFFLD_INVALID_OBJECT_ID)
			break;

	if (i == dpa_stats->config.max_counters) {
		pr_err("All counters have been used\n");
		cq_put_4bytes(dpa_stats->cnt_id_cq, id);
		mutex_unlock(&dpa_stats->lock);
		return -EDOM;
	}

	/* Acquire a preallocated Counter Control Block  */
	new_cnt = &dpa_stats->cnts_cb[id];
	new_cnt->id = id;
	new_cnt->index = i;

	/* Store on the current position the counter id */
	dpa_stats->used_cnt_ids[i] = id;

	/* Release DPA Stats instance lock */
	mutex_unlock(&dpa_stats->lock);

	*cnt_cb = new_cnt;

	return 0;
}

static int get_new_req(struct dpa_stats *dpa_stats,
		       int *dpa_stats_req_id,
		       struct dpa_stats_req_cb **req_cb)
{
	struct dpa_stats_req_cb *new_req;
	uint32_t id;
	int i;

	/* Acquire DPA Stats instance lock */
	mutex_lock(&dpa_stats->lock);

	/* Get an id for a new request */
	if (cq_get_4bytes(dpa_stats->req_id_cq, &id) < 0) {
		pr_err("No more unused request ids\n");
		mutex_unlock(&dpa_stats->lock);
		return -EDOM;
	}

	/*
	 * Use 'used_req_ids' array in order to store requests ids that are
	 * 'in use' . Array can be further used to remove requests
	 */
	for (i = 0; i < DPA_STATS_MAX_NUM_OF_REQUESTS; i++)
		if (dpa_stats->used_req_ids[i] == DPA_OFFLD_INVALID_OBJECT_ID)
			break;

	if (i == DPA_STATS_MAX_NUM_OF_REQUESTS) {
		pr_err("All requests have been used\n");
		cq_put_4bytes(dpa_stats->req_id_cq, id);
		mutex_unlock(&dpa_stats->lock);
		return -EDOM;
	}

	/* Acquire a preallocated Request Control Block  */
	new_req = &dpa_stats->reqs_cb[id];
	new_req->id = id;
	new_req->index = i;

	/* Store on the current position the request id */
	dpa_stats->used_req_ids[i] = id;

	/* Release DPA Stats instance lock */
	mutex_unlock(&dpa_stats->lock);

	*req_cb = new_req;
	*dpa_stats_req_id = id;

	return 0;
}

static int put_cnt(struct dpa_stats *dpa_stats, struct dpa_stats_cnt_cb *cnt_cb)
{
	int err = 0;

	/* Acquire DPA Stats instance lock */
	mutex_lock(&dpa_stats->lock);

	/* Release the Counter id in the Counter IDs circular queue */
	err = cq_put_4bytes(dpa_stats->cnt_id_cq, cnt_cb->id);
	if (err < 0) {
		pr_err("Could not release the counter id %d\n", cnt_cb->id);
		return -EDOM;
	}

	/* Mark the Counter id as 'not used' */
	dpa_stats->used_cnt_ids[cnt_cb->index] =
						DPA_OFFLD_INVALID_OBJECT_ID;

	/* Clear all 'cnt_cb' information  */
	cnt_cb->index = DPA_OFFLD_INVALID_OBJECT_ID;
	cnt_cb->id = DPA_STATS_MAX_NUM_OF_COUNTERS;
	cnt_cb->bytes_num = 0;
	cnt_cb->f_get_cnt_stats = NULL;

	switch (cnt_cb->type) {
	case DPA_STATS_CNT_ETH:
	case DPA_STATS_CNT_REASS:
	case DPA_STATS_CNT_FRAG:
	case DPA_STATS_CNT_POLICER:
		memset(&cnt_cb->gen_cb, 0, sizeof(cnt_cb->gen_cb));
		break;
	case DPA_STATS_CNT_CLASSIF_TBL:
		memset(&cnt_cb->tbl_cb, 0, sizeof(cnt_cb->tbl_cb));
		break;
	case DPA_STATS_CNT_CLASSIF_NODE:
		memset(&cnt_cb->ccnode_cb, 0, sizeof(cnt_cb->ccnode_cb));
		break;
	case DPA_STATS_CNT_IPSEC:
		memset(&cnt_cb->ipsec_cb, 0, sizeof(cnt_cb->ipsec_cb));
		break;
	default:
		break;
	}

	/* Release DPA Stats instance lock */
	mutex_unlock(&dpa_stats->lock);

	return 0;
}

static int put_req(struct dpa_stats *dpa_stats, struct dpa_stats_req_cb *req_cb)
{
	int err = 0;

	/* Acquire DPA Stats instance lock */
	mutex_lock(&dpa_stats->lock);

	/* Release the Counter id in the Counter IDs circular queue */
	err = cq_put_4bytes(dpa_stats->req_id_cq, req_cb->id);
	if (err < 0) {
		pr_err("Could not release the counter id %d\n", req_cb->id);
		mutex_unlock(&dpa_stats->lock);
		return -EDOM;
	}

	/* Mark the Counter id as 'not used' */
	dpa_stats->used_req_ids[req_cb->index] =
						DPA_OFFLD_INVALID_OBJECT_ID;

	/* Clear all 'req_cb' information by setting them to a maximum value */
	req_cb->index = DPA_OFFLD_INVALID_OBJECT_ID;
	req_cb->id = DPA_STATS_MAX_NUM_OF_REQUESTS;
	req_cb->bytes_num = 0;
	req_cb->cnts_num = 0;
	req_cb->request_area = NULL;
	req_cb->request_done = NULL;

	/* Release DPA Stats instance lock */
	mutex_unlock(&dpa_stats->lock);

	return 0;
}

static int init_cnts_resources(struct dpa_stats *dpa_stats)
{
	struct dpa_stats_params config = dpa_stats->config;
	int i;

	/* Create circular queue that holds free counter IDs */
	dpa_stats->cnt_id_cq = cq_new(config.max_counters, sizeof(int));
	if (!dpa_stats->cnt_id_cq) {
		pr_err("Could not create Counter IDs circular queue\n");
		return -ENOMEM;
	}

	/* Fill the circular queue with ids */
	for (i = 0; i < config.max_counters; i++)
		if (cq_put_4bytes(dpa_stats->cnt_id_cq, i) < 0) {
			pr_err("Could not fill Counter IDs circular queue\n");
			return -EDOM;
		}

	/* Allocate array to store counter ids that are 'in use' */
	dpa_stats->used_cnt_ids = kmalloc(
			config.max_counters * sizeof(uint32_t), GFP_KERNEL);
	if (!dpa_stats->used_cnt_ids) {
		pr_err("No more memory for used counter ids array\n");
		return -ENOMEM;
	}
	memset(dpa_stats->used_cnt_ids, DPA_OFFLD_INVALID_OBJECT_ID,
			config.max_counters * sizeof(uint32_t));

	/* Allocate array to store counters control blocks */
	dpa_stats->cnts_cb = kzalloc(config.max_counters *
			sizeof(struct dpa_stats_cnt_cb), GFP_KERNEL);
	if (!dpa_stats->cnts_cb) {
		pr_err("No more memory for used counters control blocks\n");
		return -ENOMEM;
	}

	for (i = 0; i < config.max_counters; i++) {
		mutex_init(&dpa_stats->cnts_cb[i].lock);
		dpa_stats->cnts_cb[i].dpa_stats = dpa_stats;
		dpa_stats->cnts_cb[i].index = DPA_OFFLD_INVALID_OBJECT_ID;
	}

	return 0;
}

static int free_cnts_resources(struct dpa_stats *dpa_stats)
{
	uint32_t id, i;
	int err = 0;

	for (i = 0; i < dpa_stats->config.max_counters; i++) {
		mutex_lock(&dpa_stats->lock);
		id = dpa_stats->used_cnt_ids[i];
		mutex_unlock(&dpa_stats->lock);

		if (id != DPA_OFFLD_INVALID_OBJECT_ID)
			/* Release the counter id in the Counter IDs cq */
			err = put_cnt(dpa_stats, &dpa_stats->cnts_cb[id]);
			if (err < 0) {
				pr_err("Failed to release a counter id\n");
				return err;
			}
	}

	/* Release counters IDs circular queue */
	if (dpa_stats->cnt_id_cq) {
		cq_delete(dpa_stats->cnt_id_cq);
		dpa_stats->cnt_id_cq = NULL;
	}

	/* Release counters control blocks */
	kfree(dpa_stats->cnts_cb);
	dpa_stats->cnts_cb = NULL;

	/* Release counters 'used ids' array */
	kfree(dpa_stats->used_cnt_ids);
	dpa_stats->used_cnt_ids = NULL;

	return 0;
}

static int init_reqs_resources(struct dpa_stats *dpa_stats)
{
	int i;

	/*
	 * Create work queue to defer work when asynchronous
	 * counters requests are received
	 */
	dpa_stats->async_req_workqueue = alloc_workqueue("async_req_workqueue",
			WQ_UNBOUND | WQ_MEM_RECLAIM, WORKQUEUE_MAX_ACTIVE);
	if (!dpa_stats->async_req_workqueue) {
		pr_err("Creating async request work queue failed\n");
		return -ENOSPC;
	}

	/* Create circular queue that holds free counter request IDs */
	dpa_stats->req_id_cq = cq_new(
			DPA_STATS_MAX_NUM_OF_REQUESTS, sizeof(int));
	if (!dpa_stats->req_id_cq) {
		pr_err("Could not create Request IDs circular queue\n");
		return -ENOMEM;
	}

	/* Fill the circular queue with ids */
	for (i = 0; i < DPA_STATS_MAX_NUM_OF_REQUESTS; i++)
		if (cq_put_4bytes(dpa_stats->req_id_cq, i) < 0) {
			pr_err("Could not fill Request IDs circular queue\n");
			return -EDOM;
		}

	/* Allocate array to store requests ids that are 'in use' */
	dpa_stats->used_req_ids = kmalloc(DPA_STATS_MAX_NUM_OF_REQUESTS *
			sizeof(uint32_t), GFP_KERNEL);
	if (!dpa_stats->used_req_ids) {
		pr_err("No more memory for used req ids array\n");
		return -ENOMEM;
	}
	memset(dpa_stats->used_req_ids, DPA_OFFLD_INVALID_OBJECT_ID,
			DPA_STATS_MAX_NUM_OF_REQUESTS * sizeof(uint32_t));

	/* Allocate array to store requests control blocks */
	dpa_stats->reqs_cb = kzalloc(DPA_STATS_MAX_NUM_OF_REQUESTS *
				sizeof(struct dpa_stats_req_cb), GFP_KERNEL);
	if (!dpa_stats->reqs_cb) {
		pr_err("No more memory for requests control blocks\n");
		return -ENOMEM;
	}

	/* Allocate array to store the counter ids */
	for (i = 0; i < DPA_STATS_MAX_NUM_OF_REQUESTS; i++) {
		dpa_stats->reqs_cb[i].config.cnts_ids =
				kzalloc(DPA_STATS_MAX_NUM_OF_COUNTERS *
						sizeof(int), GFP_KERNEL);
		if (!dpa_stats->reqs_cb[i].config.cnts_ids) {
			pr_err("No more memory for array of counter ids\n");
			return -ENOMEM;
		}

		/* Initialize work to be done for each request */
		INIT_WORK(&dpa_stats->reqs_cb[i].async_req_work,
						async_req_work_func);
	}

	return 0;
}

static int free_reqs_resources(struct dpa_stats *dpa_stats)
{
	struct dpa_stats_req_cb *req_cb = NULL;
	uint32_t id, i;
	int err = 0;

	for (i = 0; i <  DPA_STATS_MAX_NUM_OF_REQUESTS; i++) {
		mutex_lock(&dpa_stats->lock);
		id = dpa_stats->used_req_ids[i];
		mutex_unlock(&dpa_stats->lock);

		if (id != DPA_OFFLD_INVALID_OBJECT_ID) {
			req_cb = &dpa_stats->reqs_cb[id];

			flush_work(&req_cb->async_req_work);

			/* Release the request id in the Requests IDs cq */
			err = put_req(dpa_stats, req_cb);
			if (err < 0) {
				pr_err("Failed to release a request id\n");
				return err;
			}

			/* Release the array of counter ids */
			kfree(req_cb->config.cnts_ids);
			req_cb->config.cnts_ids = NULL;
		}
	}

	/* Release requests IDs circular queue */
	if (dpa_stats->req_id_cq) {
		cq_delete(dpa_stats->req_id_cq);
		dpa_stats->req_id_cq = NULL;
	}

	/* Release requests control blocks */
	kfree(dpa_stats->reqs_cb);
	dpa_stats->reqs_cb = NULL;

	/* Release requests 'used ids' array */
	kfree(dpa_stats->used_req_ids);
	dpa_stats->used_req_ids = NULL;

	/* destroy asynchronous requests workqueue */
	if (dpa_stats->async_req_workqueue) {
		destroy_workqueue(dpa_stats->async_req_workqueue);
		dpa_stats->async_req_workqueue = NULL;
	}

	return 0;
}

/* cleanup DPA Stats */
static void free_resources(void)
{
	struct dpa_stats *dpa_stats;

	/* Sanity check */
	if (!gbl_dpa_stats) {
		pr_err("DPA stats instance is not initialized\n");
		return;
	}
	dpa_stats = gbl_dpa_stats;

	/* free resources occupied by counters control blocks */
	free_cnts_resources(dpa_stats);

	/* free resources occupied by requests control blocks */
	free_reqs_resources(dpa_stats);

	kfree(dpa_stats);
	gbl_dpa_stats = NULL;
}

static int treat_cnts_request(struct dpa_stats *dpa_stats,
			      struct dpa_stats_req_cb *req_cb)
{
	struct dpa_stats_cnt_request_params params = req_cb->config;
	struct dpa_stats_cnt_cb *cnt_cb = NULL;
	int id = 0, err = 0;
	uint32_t i = 0;

	for (i = 0; i < params.cnts_ids_len; i++) {
		id = params.cnts_ids[i];

		/* Get counter's control block */
		cnt_cb = &dpa_stats->cnts_cb[id];

		/* Acquire counter lock */
		mutex_lock(&cnt_cb->lock);

		cnt_cb->info.reset = req_cb->config.reset_cnts;

		/* Call counter's retrieve function */
		err = cnt_cb->f_get_cnt_stats(req_cb, cnt_cb);
		if (err < 0) {
			pr_err("Failed to retrieve counter values\n");
			mutex_unlock(&cnt_cb->lock);
			unblock_sched_cnts(dpa_stats, params.cnts_ids,
					   params.cnts_ids_len);
			return err;
		}

		/*
		 * Update number of bytes and number of counters
		 * successfully written so far
		 */
		req_cb->bytes_num += cnt_cb->bytes_num;
		req_cb->cnts_num += 1;

		mutex_unlock(&cnt_cb->lock);
	}

	unblock_sched_cnts(dpa_stats, params.cnts_ids, params.cnts_ids_len);

	return 0;
}

static void create_cnt_eth_stats(struct dpa_stats *dpa_stats)
{
	/* DPA_STATS_CNT_ETH_DROP_PKTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][0] =
			offsetof(struct t_FmMacStatistics, eStatsDropEvents);
	/* DPA_STATS_CNT_ETH_BYTES */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][1] =
			offsetof(struct t_FmMacStatistics, ifInOctets);
	/* DPA_STATS_CNT_ETH_PKTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][2] =
			offsetof(struct t_FmMacStatistics, ifInPkts);
	/* DPA_STATS_CNT_ETH_BC_PKTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][3] =
			offsetof(struct t_FmMacStatistics, ifInBcastPkts);
	/* DPA_STATS_CNT_ETH_MC_PKTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][4] =
			offsetof(struct t_FmMacStatistics, ifInMcastPkts);
	/* DPA_STATS_CNT_ETH_CRC_ALIGN_ERR */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][5] =
			offsetof(struct t_FmMacStatistics, eStatCRCAlignErrors);
	/* DPA_STATS_CNT_ETH_UNDERSIZE_PKTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][6] =
			offsetof(struct t_FmMacStatistics, eStatUndersizePkts);
	/* DPA_STATS_CNT_ETH_OVERSIZE_PKTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][7] =
			offsetof(struct t_FmMacStatistics, eStatOversizePkts);
	/* DPA_STATS_CNT_ETH_FRAGMENTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][8] =
			offsetof(struct t_FmMacStatistics, eStatFragments);
	/* DPA_STATS_CNT_ETH_JABBERS */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][9] =
			offsetof(struct t_FmMacStatistics, eStatJabbers);
	/* DPA_STATS_CNT_ETH_64BYTE_PKTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][10] =
			offsetof(struct t_FmMacStatistics, eStatPkts64);
	/* DPA_STATS_CNT_ETH_65_127BYTE_PKTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][11] =
			offsetof(struct t_FmMacStatistics, eStatPkts65to127);
	/* DPA_STATS_CNT_ETH_128_255BYTE_PKTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][12] =
			offsetof(struct t_FmMacStatistics, eStatPkts128to255);
	/* DPA_STATS_CNT_ETH_256_511BYTE_PKTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][13] =
			offsetof(struct t_FmMacStatistics, eStatPkts256to511);
	/* DPA_STATS_CNT_ETH_512_1023BYTE_PKTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][14] =
			offsetof(struct t_FmMacStatistics, eStatPkts512to1023);
	/* DPA_STATS_CNT_ETH_1024_1518BYTE_PKTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][15] =
			offsetof(struct t_FmMacStatistics, eStatPkts1024to1518);
	/* DPA_STATS_CNT_ETH_OUT_PKTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][16] =
			offsetof(struct t_FmMacStatistics, ifOutPkts);
	/* DPA_STATS_CNT_ETH_OUT_DROP_PKTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][17] =
			offsetof(struct t_FmMacStatistics, ifOutDiscards);
	/* DPA_STATS_CNT_ETH_OUT_BYTES */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][18] =
			offsetof(struct t_FmMacStatistics, ifOutOctets);
	/* DPA_STATS_CNT_ETH_IN_ERRORS */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][19] =
			offsetof(struct t_FmMacStatistics, ifInErrors);
	/* DPA_STATS_CNT_ETH_OUT_ERRORS */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][20] =
			offsetof(struct t_FmMacStatistics, ifOutErrors);
	/* DPA_STATS_CNT_ETH_IN_UNICAST_PKTS : not supported on dTSEC MAC */
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][21] =
			offsetof(struct t_FmMacStatistics, ifInUcastPkts);
	/* DPA_STATS_CNT_ETH_OUT_UNICAST_PKTS : not supported on dTSEC MAC*/
	dpa_stats->stats_sel[DPA_STATS_CNT_ETH][22] =
			offsetof(struct t_FmMacStatistics, ifOutUcastPkts);
}

static void create_cnt_reass_stats(struct dpa_stats *dpa_stats)
{
	/* DPA_STATS_CNT_REASS_TIMEOUT */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][0] =
			offsetof(struct t_FmPcdManipReassemIpStats, timeout);
	/* DPA_STATS_CNT_REASS_RFD_POOL_BUSY */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][1] = offsetof(
			struct t_FmPcdManipReassemIpStats, rfdPoolBusy);
	/* DPA_STATS_CNT_REASS_INT_BUFF_BUSY */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][2] = offsetof(
			struct t_FmPcdManipReassemIpStats, internalBufferBusy);
	/* DPA_STATS_CNT_REASS_EXT_BUFF_BUSY */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][3] = offsetof(
			struct t_FmPcdManipReassemIpStats, externalBufferBusy);
	/* DPA_STATS_CNT_REASS_SG_FRAGS */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][4] = offsetof(
			struct t_FmPcdManipReassemIpStats, sgFragments);
	/* DPA_STATS_CNT_REASS_DMA_SEM */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][5] = offsetof(
			struct t_FmPcdManipReassemIpStats,
			dmaSemaphoreDepletion);
#if (DPAA_VERSION >= 11)
	/* DPA_STATS_CNT_REASS_NON_CONSISTENT_SP */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][6] = offsetof(
			struct t_FmPcdManipReassemIpStats,
			nonConsistentSp);
#else
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][6] = UNSUPPORTED_CNT_SEL;
#endif /* (DPAA_VERSION >= 11) */
	/* DPA_STATS_CNT_REASS_IPv4_FRAMES */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][8] = offsetof(
			struct t_FmPcdManipReassemIpStats,
			specificHdrStatistics[0].successfullyReassembled);
	/* DPA_STATS_CNT_REASS_IPv4_FRAGS_VALID */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][9] = offsetof(
			struct t_FmPcdManipReassemIpStats,
			specificHdrStatistics[0].validFragments);
	/* DPA_STATS_CNT_REASS_IPv4_FRAGS_TOTAL */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][10] = offsetof(
			struct t_FmPcdManipReassemIpStats,
			specificHdrStatistics[0].processedFragments);
	/* DPA_STATS_CNT_REASS_IPv4_FRAGS_MALFORMED */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][11] = offsetof(
			struct t_FmPcdManipReassemIpStats,
			specificHdrStatistics[0].malformedFragments);
	/* DPA_STATS_CNT_REASS_IPv4_FRAGS_DISCARDED */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][12] = offsetof(
			struct t_FmPcdManipReassemIpStats,
			specificHdrStatistics[0].discardedFragments);
	/* DPA_STATS_CNT_REASS_IPv4_AUTOLEARN_BUSY */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][13] = offsetof(
			struct t_FmPcdManipReassemIpStats,
			specificHdrStatistics[0].autoLearnBusy);
	/* DPA_STATS_CNT_REASS_IPv4_EXCEED_16FRAGS */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][14] = offsetof(
			struct t_FmPcdManipReassemIpStats,
			specificHdrStatistics[0].moreThan16Fragments);
	/* DPA_STATS_CNT_REASS_IPv6_FRAMES */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][16] = offsetof(
			struct t_FmPcdManipReassemIpStats,
			specificHdrStatistics[1].successfullyReassembled);
	/* DPA_STATS_CNT_REASS_IPv6_FRAGS_VALID */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][17] = offsetof(
			struct t_FmPcdManipReassemIpStats,
			specificHdrStatistics[1].validFragments);
	/* DPA_STATS_CNT_REASS_IPv6_FRAGS_TOTAL */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][18] = offsetof(
			struct t_FmPcdManipReassemIpStats,
			specificHdrStatistics[1].processedFragments);
	/* DPA_STATS_CNT_REASS_IPv6_FRAGS_MALFORMED */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][19] = offsetof(
			struct t_FmPcdManipReassemIpStats,
			specificHdrStatistics[1].malformedFragments);
	/* DPA_STATS_CNT_REASS_IPv6_FRAGS_DISCARDED */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][20] = offsetof(
			struct t_FmPcdManipReassemIpStats,
			specificHdrStatistics[1].discardedFragments);
	/* DPA_STATS_CNT_REASS_IPv6_AUTOLEARN_BUSY */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][21] = offsetof(
			struct t_FmPcdManipReassemIpStats,
			specificHdrStatistics[1].autoLearnBusy);
	/* DPA_STATS_CNT_REASS_IPv6_EXCEED_16FRAGS */
	dpa_stats->stats_sel[DPA_STATS_CNT_REASS][22] = offsetof(
			struct t_FmPcdManipReassemIpStats,
			specificHdrStatistics[1].moreThan16Fragments);
}

static void create_cnt_frag_stats(struct dpa_stats *dpa_stats)
{
	/* DPA_STATS_CNT_FRAG_TOTAL_FRAMES */
	dpa_stats->stats_sel[DPA_STATS_CNT_FRAG][0] =
			offsetof(struct t_FmPcdManipFragIpStats, totalFrames);
	/* DPA_STATS_CNT_FRAG_FRAMES */
	dpa_stats->stats_sel[DPA_STATS_CNT_FRAG][1] = offsetof(
			struct t_FmPcdManipFragIpStats, fragmentedFrames);
	/* DPA_STATS_CNT_FRAG_GEN_FRAGS */
	dpa_stats->stats_sel[DPA_STATS_CNT_FRAG][2] = offsetof(
			struct t_FmPcdManipFragIpStats, generatedFragments);
}

static void create_cnt_plcr_stats(struct dpa_stats *dpa_stats)
{
	/* DPA_STATS_CNT_PLCR_GREEN_PKTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_POLICER][0] =
			e_FM_PCD_PLCR_PROFILE_GREEN_PACKET_TOTAL_COUNTER;
	/* DPA_STATS_CNT_PLCR_YELLOW_PKTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_POLICER][1] =
			e_FM_PCD_PLCR_PROFILE_YELLOW_PACKET_TOTAL_COUNTER;
	/* DPA_STATS_CNT_PLCR_RED_PKTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_POLICER][2] =
			e_FM_PCD_PLCR_PROFILE_RED_PACKET_TOTAL_COUNTER;
	/* DPA_STATS_CNT_PLCR_RECOLOR_YELLOW_PKTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_POLICER][3] =
		e_FM_PCD_PLCR_PROFILE_RECOLOURED_YELLOW_PACKET_TOTAL_COUNTER;
	/* DPA_STATS_CNT_PLCR_RECOLOR_RED_PKTS */
	dpa_stats->stats_sel[DPA_STATS_CNT_POLICER][4] =
		e_FM_PCD_PLCR_PROFILE_RECOLOURED_RED_PACKET_TOTAL_COUNTER;
}

static void create_classif_stats(struct dpa_stats *dpa_stats)
{
	/* DPA_STATS_CNT_CLASSIF_BYTES */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][0] =
			offsetof(struct t_FmPcdCcKeyStatistics, byteCount);
	/* DPA_STATS_CNT_CLASSIF_PACKETS */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][1] =
			offsetof(struct t_FmPcdCcKeyStatistics, frameCount);
#if (DPAA_VERSION >= 11)
	/* DPA_STATS_CNT_CLASSIF_RANGE1 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][2] = offsetof(
			struct t_FmPcdCcKeyStatistics,
			frameLengthRangeCount[0]);
	/* DPA_STATS_CNT_CLASSIF_RANGE2 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][3] = offsetof(
			struct t_FmPcdCcKeyStatistics,
			frameLengthRangeCount[1]);
	/* DPA_STATS_CNT_CLASSIF_RANGE3 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][4] = offsetof(
			struct t_FmPcdCcKeyStatistics,
			frameLengthRangeCount[2]);
	/* DPA_STATS_CNT_CLASSIF_RANGE4 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][5] = offsetof(
			struct t_FmPcdCcKeyStatistics,
			frameLengthRangeCount[3]);
	/* DPA_STATS_CNT_CLASSIF_RANGE5 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][6] = offsetof(
			struct t_FmPcdCcKeyStatistics,
			frameLengthRangeCount[4]);
	/* DPA_STATS_CNT_CLASSIF_RANGE6 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][7] = offsetof(
			struct t_FmPcdCcKeyStatistics,
			frameLengthRangeCount[5]);
	/* DPA_STATS_CNT_CLASSIF_RANGE7 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][8] = offsetof(
			struct t_FmPcdCcKeyStatistics,
			frameLengthRangeCount[6]);
	/* DPA_STATS_CNT_CLASSIF_RANGE8 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][9] = offsetof(
			struct t_FmPcdCcKeyStatistics,
			frameLengthRangeCount[7]);
	/* DPA_STATS_CNT_CLASSIF_RANGE9 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][10] = offsetof(
			struct t_FmPcdCcKeyStatistics,
			frameLengthRangeCount[8]);
	/* DPA_STATS_CNT_CLASSIF_RANGE10 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][11] = offsetof(
			struct t_FmPcdCcKeyStatistics,
			frameLengthRangeCount[9]);
#else
	/* DPA_STATS_CNT_CLASSIF_RANGE1 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][2] =
			UNSUPPORTED_CNT_SEL;
	/* DPA_STATS_CNT_CLASSIF_RANGE2 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][3] =
			UNSUPPORTED_CNT_SEL;
	/* DPA_STATS_CNT_CLASSIF_RANGE3 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][4] =
			UNSUPPORTED_CNT_SEL;
	/* DPA_STATS_CNT_CLASSIF_RANGE4 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][5] =
			UNSUPPORTED_CNT_SEL;
	/* DPA_STATS_CNT_CLASSIF_RANGE5 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][6] =
			UNSUPPORTED_CNT_SEL;
	/* DPA_STATS_CNT_CLASSIF_RANGE6 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][7] =
			UNSUPPORTED_CNT_SEL;
	/* DPA_STATS_CNT_CLASSIF_RANGE7 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][8] =
			UNSUPPORTED_CNT_SEL;
	/* DPA_STATS_CNT_CLASSIF_RANGE8 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][9] =
			UNSUPPORTED_CNT_SEL;
	/* DPA_STATS_CNT_CLASSIF_RANGE9 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][10] =
			UNSUPPORTED_CNT_SEL;
	/* DPA_STATS_CNT_CLASSIF_RANGE10 */
	dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE][11] =
			UNSUPPORTED_CNT_SEL;
#endif
}

static void create_cnt_ipsec_stats(struct dpa_stats *dpa_stats)
{
	/* DPA_STATS_CNT_NUM_OF_BYTES */
	dpa_stats->stats_sel[DPA_STATS_CNT_IPSEC][0] = offsetof(
			struct dpa_ipsec_sa_stats, bytes_count);
	/* DPA_STATS_CNT_NUM_OF_PACKETS */
	dpa_stats->stats_sel[DPA_STATS_CNT_IPSEC][1] = offsetof(
			struct dpa_ipsec_sa_stats, packets_count);
}

static int copy_key_descriptor(const struct dpa_offload_lookup_key *src,
			       struct dpa_offload_lookup_key *dst)
{
	/* Check that key byte pointer is valid */
	if (!src->byte) {
		pr_err("Key byte pointer can't be NULL\n");
		return -EINVAL;
	}

	/* Check that key size is not zero */
	if (src->size == 0) {
		pr_err("Key size can't be zero\n");
		return -EINVAL;
	}

	/* Allocate memory to store the key byte array */
	dst->byte = kmalloc(src->size, GFP_KERNEL);
	if (!dst->byte) {
		pr_err("No more memory for key byte\n");
		return -ENOMEM;
	}
	memcpy(dst->byte, src->byte, src->size);

	/* If there is a valid key mask pointer */
	if (src->mask) {
		/* Allocate memory to store the key mask array */
		dst->mask = kmalloc(src->size, GFP_KERNEL);
		if (!dst->mask) {
			pr_err("No more memory for key mask\n");
			kfree(dst->byte);
			return -ENOMEM;
		}
		memcpy(dst->mask, src->mask, src->size);
	} else
		dst->mask = NULL;

	/* Store the key size */
	dst->size = src->size;

	return 0;
}

static t_Handle get_fman_mac_handle(struct device_node *parent_dev_node,
				    int port_id,
				    char *mac_name)
{
	struct device_node *dev_node, *tmp_node = NULL;
	struct mac_device  *mac_dev = NULL;
	const uint32_t	*cell_index;
	int lenp;

	while ((dev_node = of_find_compatible_node(tmp_node, NULL,
			mac_name)) != NULL) {
		if (parent_dev_node == of_get_parent(dev_node)) {
			cell_index = of_get_property(
					dev_node, "cell-index", &lenp);
			if (*cell_index == port_id) {
				mac_dev = dev_get_drvdata(&
					of_find_device_by_node(dev_node)->dev);
				return mac_dev->get_mac_handle(mac_dev);
			}
		}

		tmp_node = dev_node;
	}

	return NULL;
}

static struct device_node *get_fman_dev_node(int fman_id)
{
	struct device_node *dev_node, *tmp_node = NULL;
	const uint32_t *cell_index;
	int lenp;

	while ((dev_node = of_find_compatible_node(tmp_node, NULL, "fsl,fman"))
			!= NULL) {
		cell_index = of_get_property(dev_node, "cell-index", &lenp);
		if (*cell_index == fman_id)
			break;

		tmp_node = dev_node;
	}

	return dev_node;
}

static int get_fm_mac(struct dpa_stats_cnt_eth_src src, void **mac)
{
	struct device_node *dev_node = NULL;
	t_Handle *fm_mac = NULL;
	char *mac_name;

	/* Get FMAN device node */
	dev_node = get_fman_dev_node(src.engine_id);
	if (!dev_node) {
		pr_err("FMan device node couldn't be found\n");
		return -EINVAL;
	}

	if (src.eth_id > DPA_STATS_ETH_1G_PORT5) {
		/* Get Ethernet device node first for DTSEC case 10G port*/
		mac_name = "fsl,fman-10g-mac";
		src.eth_id -= DPA_STATS_ETH_10G_PORT0;

		fm_mac = get_fman_mac_handle(dev_node, src.eth_id, mac_name);
		if (!fm_mac) {
			/* Get Ethernet device node for MEMAC case 10G port */
			mac_name = "fsl,fman-memac";
			fm_mac = get_fman_mac_handle(
					dev_node, src.eth_id, mac_name);
			if (!fm_mac) {
				pr_err("Ethernet device node couldn't be found\n");
				return -EINVAL;
			}
		}
	} else {
		/* Get Ethernet device node first for DTSEC case 1G port*/
		mac_name = "fsl,fman-1g-mac";

		fm_mac = get_fman_mac_handle(dev_node, src.eth_id, mac_name);
		if (!fm_mac) {
			/* Get Ethernet device node for MEMAC case 1G port*/
			mac_name = "fsl,fman-memac";
			fm_mac = get_fman_mac_handle(
					dev_node, src.eth_id, mac_name);
			if (!fm_mac) {
				pr_err("Ethernet device node couldn't be found\n");
				return -EINVAL;
			}
		}
	}

	/* Return FM MAC handle */
	*mac = fm_mac;

	return 0;
}

static void cnt_sel_to_stats(struct stats_info *stats_info,
			     int *stats_sel,
			     uint32_t cnt_sel)
{
	uint32_t bitVal = 0, bitPos = 0, cntPos = 1;

	while (cnt_sel > 0) {
		bitVal = cnt_sel & 0x00000001;
		stats_info->stats_off[cntPos - bitVal] = stats_sel[bitPos++];
		cntPos += bitVal;
		cnt_sel >>= 1;
	}

	stats_info->stats_num = cntPos - 1;
}

static int set_frag_manip(int td, struct dpa_stats_lookup_key *entry)
{
	struct dpa_cls_tbl_action action;
	struct t_FmPcdManipStats stats;
	int err = 0;

	err = dpa_classif_table_lookup_by_key(td, &entry->key, &action);
	if (err != 0) {
		pr_err("Unable to retrieve next action parameters\n");
		return -EINVAL;
	}

	if (action.type != DPA_CLS_TBL_ACTION_ENQ) {
		pr_err("Fragmentation statistics per flow are "
			"supported only for action enqueue\n");
		return -EINVAL;
	}

	entry->frag = dpa_classif_get_frag_hm_handle(action.enq_params.hmd);
	if (!entry->frag) {
		pr_err("Unable to retrieve fragmentation handle\n");
		return -EINVAL;
	}

	/* Check the user-provided fragmentation handle */
	err = FM_PCD_ManipGetStatistics(entry->frag, &stats);
	if (err < 0) {
		pr_err("Invalid Fragmentation manip handle\n");
		return -EINVAL;
	}

	return 0;
}

static int set_cnt_eth_cb(struct dpa_stats_cnt_cb *cnt_cb,
			  const struct dpa_stats_cnt_params *params)
{
	struct dpa_stats *dpa_stats = cnt_cb->dpa_stats;
	uint32_t cnt_sel = params->eth_params.cnt_sel;
	t_Handle fm_mac = NULL;
	int	 err = 0;

	if (!dpa_stats) {
		pr_err("Invalid argument: NULL DPA Stats instance\n");
		return -EFAULT;
	}

	/* Check Ethernet counter selection */
	if (cnt_sel == 0 || cnt_sel > DPA_STATS_CNT_ETH_ALL) {
		pr_err("Invalid Ethernet counter selection\n");
		return -EINVAL;
	}

	/* Decrease one to obtain the mask for all statistics */
	if (cnt_sel == DPA_STATS_CNT_ETH_ALL)
		cnt_sel -= 1;

	/* Get FM MAC handle */
	err = get_fm_mac(params->eth_params.src, &fm_mac);
	if (err != 0) {
		pr_err("Could not obtain FM MAC handle!\n");
		return -EINVAL;
	}

	cnt_cb->gen_cb.objs[0] = fm_mac;
	cnt_cb->members_num = 1;

	/* Map Ethernet counter selection to FM MAC statistics */
	cnt_sel_to_stats(&cnt_cb->info,
			 dpa_stats->stats_sel[DPA_STATS_CNT_ETH], cnt_sel);

	/* Set number of bytes that will be written by this counter */
	cnt_cb->bytes_num = STATS_VAL_SIZE * cnt_cb->info.stats_num;

	return 0;
}

static int set_cnt_reass_cb(struct dpa_stats_cnt_cb *cnt_cb,
			    const struct dpa_stats_cnt_params *params)
{
	struct dpa_stats *dpa_stats = cnt_cb->dpa_stats;
	uint32_t cnt_sel = params->reass_params.cnt_sel;
	struct t_FmPcdManipStats stats;
	int err;

	if (!dpa_stats) {
		pr_err("Invalid argument: NULL DPA Stats instance\n");
		return -EFAULT;
	}

	/* User can combine counters only from a group */
	if (!((cnt_sel != 0 && cnt_sel <= DPA_STATS_CNT_REASS_GEN_ALL) ||
		(cnt_sel >= DPA_STATS_CNT_REASS_IPv4_FRAMES &&
		cnt_sel <= DPA_STATS_CNT_REASS_IPv4_ALL) ||
		(cnt_sel >= DPA_STATS_CNT_REASS_IPv6_FRAMES &&
		cnt_sel <= DPA_STATS_CNT_REASS_IPv6_ALL))) {
		pr_err("Invalid Reassembly counter selection\n");
		return -EINVAL;
	}

	cnt_cb->gen_cb.objs[0] = params->reass_params.reass;
	cnt_cb->members_num = 1;

	/* Check the user-provided reassembly manip */
	err = FM_PCD_ManipGetStatistics(params->reass_params.reass, &stats);
	if (err < 0) {
		pr_err("Invalid Reassembly manip handle\n");
		return -EINVAL;
	}

	/* Based on user option, change mask to all statistics in one group */
	if (cnt_sel == DPA_STATS_CNT_REASS_GEN_ALL)
		cnt_sel -= 1;
	else if (cnt_sel == DPA_STATS_CNT_REASS_IPv4_ALL)
		cnt_sel = (cnt_sel - 1) &
			~(DPA_STATS_CNT_REASS_IPv4_FRAMES - 1);
	else if (cnt_sel == DPA_STATS_CNT_REASS_IPv6_ALL)
		cnt_sel = (cnt_sel - 1) &
			~(DPA_STATS_CNT_REASS_IPv6_FRAMES - 1);

	/* Map Reassembly counter selection to Manip statistics */
	cnt_sel_to_stats(&cnt_cb->info,
			 dpa_stats->stats_sel[DPA_STATS_CNT_REASS], cnt_sel);

	/* Set number of bytes that will be written by this counter */
	cnt_cb->bytes_num = STATS_VAL_SIZE * cnt_cb->info.stats_num;

	return 0;
}

static int set_cnt_frag_cb(struct dpa_stats_cnt_cb *cnt_cb,
			   const struct dpa_stats_cnt_params *params)
{
	struct dpa_stats *dpa_stats = cnt_cb->dpa_stats;
	uint32_t cnt_sel = params->frag_params.cnt_sel;
	struct t_FmPcdManipStats stats;
	int err;

	if (!dpa_stats) {
		pr_err("Invalid argument: NULL DPA Stats instance\n");
		return -EFAULT;
	}

	/* Check Fragmentation counter selection */
	if (cnt_sel == 0 || cnt_sel > DPA_STATS_CNT_FRAG_ALL) {
		pr_err("Invalid Fragmentation counter selection\n");
		return -EINVAL;
	}

	cnt_cb->gen_cb.objs[0] = params->frag_params.frag;
	cnt_cb->members_num = 1;

	/* Check the user-provided fragmentation handle */
	err = FM_PCD_ManipGetStatistics(params->frag_params.frag, &stats);
	if (err < 0) {
		pr_err("Invalid Fragmentation manip handle\n");
		return -EINVAL;
	}

	/* Decrease one to obtain the mask for all statistics */
	if (cnt_sel == DPA_STATS_CNT_FRAG_ALL)
		cnt_sel -= 1;

	/* Map Fragmentation counter selection to Manip statistics */
	cnt_sel_to_stats(&cnt_cb->info,
			 dpa_stats->stats_sel[DPA_STATS_CNT_FRAG], cnt_sel);

	/* Set number of bytes that will be written by this counter */
	cnt_cb->bytes_num = STATS_VAL_SIZE * cnt_cb->info.stats_num;

	return 0;
}

static int set_cnt_plcr_cb(struct dpa_stats_cnt_cb *cnt_cb,
			   const struct dpa_stats_cnt_params *params)
{
	struct dpa_stats *dpa_stats = cnt_cb->dpa_stats;
	uint32_t cnt_sel = params->reass_params.cnt_sel;

	if (!dpa_stats) {
		pr_err("Invalid argument: NULL DPA Stats instance\n");
		return -EFAULT;
	}

	/* Check Policer counter selection */
	if (cnt_sel == 0 || cnt_sel > DPA_STATS_CNT_PLCR_ALL) {
		pr_err("Invalid Policer counter selection\n");
		return -EINVAL;
	}

	cnt_cb->gen_cb.objs[0] = params->plcr_params.plcr;
	cnt_cb->members_num = 1;

	/* Decrease one to obtain the mask for all statistics */
	if (cnt_sel == DPA_STATS_CNT_PLCR_ALL)
		cnt_sel -= 1;

	/* Map Policer counter selection to policer statistics */
	cnt_sel_to_stats(&cnt_cb->info,
			 dpa_stats->stats_sel[DPA_STATS_CNT_POLICER], cnt_sel);

	/* Set number of bytes that will be written by this counter */
	cnt_cb->bytes_num = STATS_VAL_SIZE * cnt_cb->info.stats_num;

	return 0;
}

static int set_cnt_classif_tbl_cb(struct dpa_stats_cnt_cb *cnt_cb,
				  const struct dpa_stats_cnt_params *params)
{
	struct dpa_stats_cnt_classif_tbl_cb *cnt_tbl_cb = &cnt_cb->tbl_cb;
	struct dpa_stats *dpa_stats = cnt_cb->dpa_stats;
	struct dpa_stats_cnt_classif_tbl prm = params->classif_tbl_params;
	struct dpa_cls_tbl_params cls_tbl;
	uint32_t cnt_sel = prm.cnt_sel;
	int err = 0, frag_stats = -1;

	if (!dpa_stats) {
		pr_err("Invalid argument: NULL DPA Stats instance\n");
		return -EFAULT;
	}

	/* Check Classifier Table counter selection */
	if (cnt_sel >= DPA_STATS_CNT_CLASSIF_BYTES &&
	    cnt_sel <= DPA_STATS_CNT_CLASSIF_ALL) {

		/* Entire group of counters was selected */
		if (cnt_sel == DPA_STATS_CNT_CLASSIF_ALL)
			cnt_sel -= 1;
		frag_stats = 0;

	} else if (cnt_sel >= DPA_STATS_CNT_FRAG_TOTAL_FRAMES &&
		   cnt_sel <= DPA_STATS_CNT_FRAG_ALL) {

		/* Entire group of counters was selected */
		if (cnt_sel == DPA_STATS_CNT_FRAG_ALL)
			cnt_sel -= 1;
		frag_stats = 1;

	} else {
		pr_err("Invalid Classifier Table counter selection\n");
		return -EINVAL;
	}

	err = dpa_classif_table_get_params(prm.td, &cls_tbl);
	if (err != 0) {
		pr_err("Invalid Classifier Table descriptor\n");
		return -EINVAL;
	}

	/* Copy the key descriptor */
	err = copy_key_descriptor(&prm.key, &cnt_tbl_cb->keys[0].key);
	if (err != 0) {
		pr_err("Unable to copy key descriptor\n");
		return -EINVAL;
	}

	/* Store CcNode handle and set number of keys to one */
	cnt_tbl_cb->keys[0].cc_node = cls_tbl.cc_node;
	cnt_tbl_cb->keys[0].valid = TRUE;
	cnt_cb->members_num = 1;

	/* Store DPA Classifier Table type */
	cnt_tbl_cb->type = cls_tbl.type;

	/* Check the Classifier Table counter */
	err = check_tbl_cls_counter(cnt_cb, &cnt_tbl_cb->keys[0]);
	if (err != 0)
		return -EINVAL;

	if (frag_stats) {
		err = set_frag_manip(prm.td, &cnt_tbl_cb->keys[0]);
		if (err < 0) {
			pr_err("Invalid Fragmentation manip handle\n");
			return -EINVAL;
		}
		/* Map Classifier Table counter selection to Frag stats */
		cnt_sel_to_stats(&cnt_cb->info,
			dpa_stats->stats_sel[DPA_STATS_CNT_FRAG], cnt_sel);

		/* Change the retrieve routine */
		cnt_cb->f_get_cnt_stats = get_cnt_cls_tbl_frag_stats;
	} else
		/* Map Classifier Table counter selection to CcNode stats */
		cnt_sel_to_stats(&cnt_cb->info,
			dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE],
			cnt_sel >> CLASSIF_STATS_SHIFT);

	/* Set number of bytes that will be written by this counter */
	cnt_cb->bytes_num = STATS_VAL_SIZE * cnt_cb->info.stats_num;

	return 0;
}

static int set_cnt_ccnode_cb(struct dpa_stats_cnt_cb *cnt_cb,
			     const struct dpa_stats_cnt_params *params)
{
	struct dpa_stats *dpa_stats = cnt_cb->dpa_stats;
	struct dpa_stats_cnt_classif_node prm = params->classif_node_params;
	int err = 0;

	if (!dpa_stats) {
		pr_err("Invalid argument: NULL DPA Stats instance\n");
		return -EFAULT;
	}

	/* Check Classification Node counter selection */
	if (prm.cnt_sel == 0 ||  prm.cnt_sel > DPA_STATS_CNT_CLASSIF_ALL) {
		pr_err("Invalid Classif_Node counter selection\n");
		return -EINVAL;
	}

	/* Copy the key descriptor */
	err = copy_key_descriptor(&prm.key, &cnt_cb->ccnode_cb.keys[0]);
	if (err != 0) {
		pr_err("Unable to copy key descriptor\n");
		return -EINVAL;
	}

	/* Store CcNode handle and set number of keys to one */
	cnt_cb->ccnode_cb.cc_node = prm.cc_node;
	cnt_cb->members_num = 1;

	/* Check the Classifier Node counter parameters */
	err = check_ccnode_counter(cnt_cb,
				   prm.ccnode_type, &cnt_cb->ccnode_cb.keys[0]);
	if (err != 0) {
		pr_err("Invalid Classif Node counter parameters\n");
		return -EINVAL;
	}

	/* Map Classif Node counter selection to CcNode statistics */
	cnt_sel_to_stats(&cnt_cb->info,
		dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE],
		prm.cnt_sel >> CLASSIF_STATS_SHIFT);

	/* Set number of bytes that will be written by this counter */
	cnt_cb->bytes_num = STATS_VAL_SIZE * cnt_cb->info.stats_num;

	return 0;
}

static int set_cnt_ipsec_cb(struct dpa_stats_cnt_cb *cnt_cb,
			    const struct dpa_stats_cnt_params *params)
{
	struct dpa_stats *dpa_stats = cnt_cb->dpa_stats;
	struct dpa_ipsec_sa_stats stats;
	uint32_t cnt_sel = params->ipsec_params.cnt_sel;
	int err = 0;

	if (!dpa_stats) {
		pr_err("Invalid argument: NULL DPA Stats instance\n");
		return -EFAULT;
	}

	/* Map IPSec counter selection to statistics */
	if (cnt_sel == DPA_STATS_CNT_NUM_OF_BYTES) {
		cnt_cb->info.stats_off[0] = dpa_stats->stats_sel[
			DPA_STATS_CNT_IPSEC][DPA_STATS_CNT_NUM_OF_BYTES];
		cnt_cb->info.stats_num = 1;
	} else if (cnt_sel == DPA_STATS_CNT_NUM_OF_PACKETS) {
		cnt_cb->info.stats_off[0] = dpa_stats->stats_sel[
			DPA_STATS_CNT_IPSEC][DPA_STATS_CNT_NUM_OF_PACKETS];
		cnt_cb->info.stats_num = 1;
	} else if (cnt_sel == DPA_STATS_CNT_NUM_ALL) {
		cnt_cb->info.stats_off[0] = dpa_stats->stats_sel[
			DPA_STATS_CNT_IPSEC][DPA_STATS_CNT_NUM_OF_BYTES];
		cnt_cb->info.stats_off[1] = dpa_stats->stats_sel[
			DPA_STATS_CNT_IPSEC][DPA_STATS_CNT_NUM_OF_PACKETS];
		cnt_cb->info.stats_num = 2;
	} else {
		pr_err("Invalid IPSec counter selection\n");
		return -EINVAL;
	}

	cnt_cb->ipsec_cb.sa_id[0] = params->ipsec_params.sa_id;
	cnt_cb->ipsec_cb.valid[0] = TRUE;
	cnt_cb->members_num = 1;

	err = dpa_ipsec_sa_get_stats(cnt_cb->ipsec_cb.sa_id[0], &stats);
	if (err < 0) {
		pr_err("Invalid IPSec counter parameters\n");
		return -EINVAL;
	}

	/* Set number of bytes that will be written by this counter */
	cnt_cb->bytes_num = STATS_VAL_SIZE * cnt_cb->info.stats_num;

	return 0;
}

static int set_cls_cnt_eth_cb(struct dpa_stats_cnt_cb *cnt_cb,
			      const struct dpa_stats_cls_cnt_params *params)
{
	struct dpa_stats *dpa_stats = cnt_cb->dpa_stats;
	uint32_t cnt_sel = params->eth_params.cnt_sel;
	t_Handle fm_mac = NULL;
	uint32_t i = 0;
	int err = 0;

	if (!dpa_stats) {
		pr_err("Invalid argument: NULL DPA Stats instance\n");
		return -EFAULT;
	}

	/* Check Ethernet counter selection */
	if (params->eth_params.cnt_sel == 0 ||
	    params->eth_params.cnt_sel > DPA_STATS_CNT_ETH_ALL) {
		pr_err("Invalid Ethernet counter selection\n");
		return -EINVAL;
	}

	/* Decrease one to obtain the mask for all statistics */
	if (cnt_sel == DPA_STATS_CNT_ETH_ALL)
		cnt_sel -= 1;

	for (i = 0; i < params->class_members; i++) {
		/* Get FM MAC handle */
		err = get_fm_mac(params->eth_params.src[i], &fm_mac);
		if (err != 0) {
			pr_err("Could not obtain FM MAC handle!\n");
			return -EINVAL;
		}

		cnt_cb->gen_cb.objs[i] = fm_mac;
	}

	cnt_cb->members_num = params->class_members;

	/* Map Ethernet counter selection to FM MAC statistics */
	cnt_sel_to_stats(&cnt_cb->info,
			 dpa_stats->stats_sel[DPA_STATS_CNT_ETH], cnt_sel);

	/* Set number of bytes that will be written by this counter */
	cnt_cb->bytes_num = cnt_cb->members_num *
				STATS_VAL_SIZE * cnt_cb->info.stats_num;
	return 0;
}

static int set_cls_cnt_reass_cb(struct dpa_stats_cnt_cb *cnt_cb,
				const struct dpa_stats_cls_cnt_params *params)
{
	struct dpa_stats *dpa_stats = cnt_cb->dpa_stats;
	struct t_FmPcdManipStats stats;
	uint32_t cnt_sel = params->reass_params.cnt_sel;
	uint32_t i = 0;
	int err = 0;

	if (!dpa_stats) {
		pr_err("Invalid argument: NULL DPA Stats instance\n");
		return -EFAULT;
	}

	/* User can combine counters only from a group */
	if (!((cnt_sel != 0 && cnt_sel <= DPA_STATS_CNT_REASS_GEN_ALL) ||
	      (cnt_sel >= DPA_STATS_CNT_REASS_IPv4_FRAMES &&
	       cnt_sel <= DPA_STATS_CNT_REASS_IPv4_ALL) ||
	      (cnt_sel >= DPA_STATS_CNT_REASS_IPv6_FRAMES &&
	       cnt_sel <= DPA_STATS_CNT_REASS_IPv6_ALL))) {
		pr_err("Invalid Reassembly counter selection\n");
		return -EINVAL;
	}

	cnt_cb->members_num = params->class_members;

	for (i = 0; i < params->class_members; i++) {
		cnt_cb->gen_cb.objs[i] = params->reass_params.reass[i];

		/* Check the user-provided reassembly manip */
		err = FM_PCD_ManipGetStatistics(cnt_cb->gen_cb.objs[i], &stats);
		if (err < 0) {
			pr_err("Invalid Reassembly manip handle\n");
			return -EINVAL;
		}
	}

	/* Based on user option, change mask to all statistics in one group */
	if (cnt_sel == DPA_STATS_CNT_REASS_GEN_ALL)
		cnt_sel -= 1;
	else if (cnt_sel == DPA_STATS_CNT_REASS_IPv4_ALL)
		cnt_sel = (cnt_sel - 1) &
			~(DPA_STATS_CNT_REASS_IPv4_FRAMES - 1);
	else if (cnt_sel == DPA_STATS_CNT_REASS_IPv6_ALL)
		cnt_sel = (cnt_sel - 1) &
			~(DPA_STATS_CNT_REASS_IPv6_FRAMES - 1);

	/* Map Reassembly counter selection to Manip statistics */
	cnt_sel_to_stats(&cnt_cb->info,
			 dpa_stats->stats_sel[DPA_STATS_CNT_REASS], cnt_sel);

	/* Set number of bytes that will be written by this counter */
	cnt_cb->bytes_num = cnt_cb->members_num *
				STATS_VAL_SIZE * cnt_cb->info.stats_num;
	return 0;
}

static int set_cls_cnt_frag_cb(struct dpa_stats_cnt_cb *cnt_cb,
			       const struct dpa_stats_cls_cnt_params *params)
{
	struct dpa_stats *dpa_stats = cnt_cb->dpa_stats;
	uint32_t cnt_sel = params->frag_params.cnt_sel, i;
	struct t_FmPcdManipStats stats;
	int err;

	if (!dpa_stats) {
		pr_err("Invalid argument: NULL DPA Stats instance\n");
		return -EFAULT;
	}

	/* Check Fragmentation counter selection */
	if ((cnt_sel == 0) || (cnt_sel > DPA_STATS_CNT_FRAG_ALL)) {
		pr_err("Invalid Fragmentation counter selection\n");
		return -EINVAL;
	}

	cnt_cb->members_num = params->class_members;

	for (i = 0; i < params->class_members; i++) {
		cnt_cb->gen_cb.objs[i] = params->frag_params.frag[i];

		/* Check the user-provided fragmentation handle */
		err = FM_PCD_ManipGetStatistics(cnt_cb->gen_cb.objs[i], &stats);
		if (err < 0) {
			pr_err("Invalid Fragmentation manip handle\n");
			return -EINVAL;
		}
	}

	/* Decrease one to obtain the mask for all statistics */
	if (cnt_sel == DPA_STATS_CNT_FRAG_ALL)
		cnt_sel -= 1;

	/* Map Fragmentation counter selection to Manip statistics */
	cnt_sel_to_stats(&cnt_cb->info,
			 dpa_stats->stats_sel[DPA_STATS_CNT_FRAG], cnt_sel);

	/* Set number of bytes that will be written by this counter */
	cnt_cb->bytes_num = cnt_cb->members_num *
				STATS_VAL_SIZE * cnt_cb->info.stats_num;
	return 0;
}

static int set_cls_cnt_plcr_cb(struct dpa_stats_cnt_cb *cnt_cb,
			       const struct dpa_stats_cls_cnt_params *params)
{
	struct dpa_stats *dpa_stats = cnt_cb->dpa_stats;
	struct stats_info *info = &cnt_cb->info;
	uint32_t cnt_sel = params->plcr_params.cnt_sel, i;

	if (!dpa_stats) {
		pr_err("Invalid argument: NULL DPA Stats instance\n");
		return -EFAULT;
	}

	/* Check Policer counter selection */
	if (cnt_sel == 0 || cnt_sel > DPA_STATS_CNT_PLCR_ALL) {
		pr_err("Invalid Policer counter selection\n");
		return -EINVAL;
	}

	cnt_cb->members_num = params->class_members;

	for (i = 0; i < params->class_members; i++) {
		cnt_cb->gen_cb.objs[i] = params->plcr_params.plcr[i];
		/* Check the user-provided policer handle */
		FM_PCD_PlcrProfileGetCounter(cnt_cb->gen_cb.objs[i],
				info->stats_off[0]);
		/*
		 * in case of bad counter the error will be displayed at
		 * creation time
		 */
	}

	/* Decrease one to obtain the mask for all statistics */
	if (cnt_sel == DPA_STATS_CNT_PLCR_ALL)
		cnt_sel -= 1;

	/* Map Policer counter selection to policer statistics */
	cnt_sel_to_stats(&cnt_cb->info,
			 dpa_stats->stats_sel[DPA_STATS_CNT_POLICER], cnt_sel);

	/* Set number of bytes that will be written by this counter */
	cnt_cb->bytes_num = cnt_cb->members_num *
				STATS_VAL_SIZE * cnt_cb->info.stats_num;
	return 0;
}

static int set_cls_cnt_classif_tbl_pair(
		struct dpa_stats_cnt_classif_tbl_cb *cnt_tbl_cb, int td,
		const struct dpa_offload_lookup_key_pair *pair,
		struct dpa_stats_lookup_key *lookup_key)
{
	struct dpa_cls_tbl_params cls_tbl;
	struct dpa_offload_lookup_key tbl_key;
	struct dpa_cls_tbl_action action;
	int err = 0;

	/* Check that key byte is not NULL */
	if (!pair->first_key.byte) {
		pr_err("Invalid argument: NULL key byte pointer\n");
		return -EFAULT;
	}

	/* Copy first key descriptor parameters*/
	err = copy_key_descriptor(&pair->first_key, &tbl_key);
	if (err != 0) {
		pr_err("Unable to copy key descriptor\n");
		return -EINVAL;
	}

	/* Use the first key of the pair to lookup in the classifier
	 * table the next table connected on a "next-action" */
	err = dpa_classif_table_lookup_by_key(td, &tbl_key, &action);
	if (err != 0) {
		pr_err("Unable to retrieve next action parameters\n");
		return -EINVAL;
	}

	if (action.type != DPA_CLS_TBL_ACTION_NEXT_TABLE) {
		pr_err("Double key is supported only if "
				"two tables are connected\n");
		return -EINVAL;
	}

	/* Get CcNode from new table descriptor */
	err = dpa_classif_table_get_params(
			action.next_table_params.next_td, &cls_tbl);
	if (err != 0) {
		pr_err("Unable to retrieve next table parameters\n");
		return -EINVAL;
	}

	/* Store DPA Classifier Table type */
	cnt_tbl_cb->type = cls_tbl.type;

	/* Store CcNode handle */
	lookup_key->cc_node = cls_tbl.cc_node;

	/* Set as lookup key the second key descriptor from the pair */
	err = copy_key_descriptor(&pair->second_key, &lookup_key->key);
	if (err != 0) {
		pr_err("Unable to copy key descriptor\n");
		return -EINVAL;
	}

	return 0;
}

static int set_cls_cnt_classif_tbl_cb(struct dpa_stats_cnt_cb *cnt_cb,
				 const struct dpa_stats_cls_cnt_params *params)
{
	struct dpa_stats_cnt_classif_tbl_cb *cnt_tbl_cb = &cnt_cb->tbl_cb;
	struct dpa_stats_cls_cnt_classif_tbl prm = params->classif_tbl_params;
	struct dpa_stats *dpa_stats = cnt_cb->dpa_stats;
	struct dpa_cls_tbl_params cls_tbl;
	uint32_t i = 0, cnt_sel = prm.cnt_sel;
	int err = 0, frag_stats = -1;

	/* Check Classifier Table descriptor */
	if (params->classif_tbl_params.td == DPA_OFFLD_INVALID_OBJECT_ID) {
		pr_err("Invalid Classifier Table descriptor\n");
		return -EINVAL;
	}

	/* Check Classifier Table counter selection */
	if (cnt_sel >= DPA_STATS_CNT_CLASSIF_BYTES &&
	    cnt_sel <= DPA_STATS_CNT_CLASSIF_ALL) {

		/* Entire group of counters was selected */
		if (cnt_sel == DPA_STATS_CNT_CLASSIF_ALL)
			cnt_sel -= 1;
		frag_stats = 0;

	} else if (cnt_sel >= DPA_STATS_CNT_FRAG_TOTAL_FRAMES &&
		 cnt_sel <= DPA_STATS_CNT_FRAG_ALL) {

		/* Entire group of counters was selected */
		if (cnt_sel == DPA_STATS_CNT_FRAG_ALL)
			cnt_sel -= 1;
		frag_stats = 1;

	} else {
		pr_err("Invalid Classifier Table counter selection\n");
		return -EINVAL;
	}

	cnt_tbl_cb->td = params->classif_tbl_params.td;
	cnt_cb->members_num = params->class_members;

	switch (prm.key_type) {
	case DPA_STATS_CLASSIF_SINGLE_KEY:
		/* Get CcNode from table descriptor */
		err = dpa_classif_table_get_params(prm.td, &cls_tbl);
		if (err != 0) {
			pr_err("Invalid argument: Table descriptor\n");
			return -EINVAL;
		}

		/* Store DPA Classifier Table type */
		cnt_tbl_cb->type = cls_tbl.type;

		for (i = 0; i < params->class_members; i++) {
			/* Store CcNode handle */
			cnt_tbl_cb->keys[i].cc_node = cls_tbl.cc_node;

			if (!prm.keys[i].byte) {
				/* Key is not valid for now */
				cnt_tbl_cb->keys[i].valid = FALSE;
				continue;
			}

			/* Copy the key descriptor */
			err = copy_key_descriptor(&prm.keys[i],
						  &cnt_tbl_cb->keys[i].key);
			if (err != 0) {
				pr_err("Unable to copy key descriptor\n");
				return -EINVAL;
			}

			/* Check the Classifier Table counter */
			err = check_tbl_cls_counter(cnt_cb,
						    &cnt_tbl_cb->keys[i]);
			if (err != 0)
				return -EINVAL;

			cnt_tbl_cb->keys[i].valid = TRUE;
		}
		break;
	case DPA_STATS_CLASSIF_PAIR_KEY:
		for (i = 0; i < params->class_members; i++) {
			if (!prm.pairs[i].first_key.byte) {
				/* Key is not valid for now */
				cnt_tbl_cb->keys[i].valid = FALSE;
				continue;
			}

			err = set_cls_cnt_classif_tbl_pair(cnt_tbl_cb, prm.td,
					&prm.pairs[i], &cnt_tbl_cb->keys[i]);
			if (err != 0) {
				pr_err("Unable to set the key pair\n");
				return -EINVAL;
			}

			/* Check the Classifier Table counter */
			err = check_tbl_cls_counter(cnt_cb,
						    &cnt_tbl_cb->keys[i]);
			if (err != 0)
				return -EINVAL;

			cnt_tbl_cb->keys[i].valid = TRUE;
		}
		break;
	default:
		pr_err("Invalid argument: key type\n");
		return -EINVAL;
	}

	if (frag_stats) {
		/* For every valid key, retrieve the hmcd */
		for (i = 0; i < params->class_members; i++) {
			if (!cnt_tbl_cb->keys[i].valid)
				continue;

			err = set_frag_manip(prm.td, &cnt_cb->tbl_cb.keys[i]);
			if (err < 0) {
				pr_err("Invalid Fragmentation manip handle\n");
				return -EINVAL;
			}
		}

		/* Map Classif Node counter selection to fragmentation stats */
		cnt_sel_to_stats(&cnt_cb->info,
			dpa_stats->stats_sel[DPA_STATS_CNT_FRAG], cnt_sel);

		/* Change the retrieve routine */
		cnt_cb->f_get_cnt_stats = get_cnt_cls_tbl_frag_stats;
	} else
		/* Map Classif Node counter selection to CcNode statistics */
		cnt_sel_to_stats(&cnt_cb->info,
			dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE],
			cnt_sel >> CLASSIF_STATS_SHIFT);

	/* Set number of bytes that will be written by this counter */
	cnt_cb->bytes_num = cnt_cb->members_num *
			STATS_VAL_SIZE * cnt_cb->info.stats_num;
	return 0;
}

static int set_cls_cnt_ccnode_cb(struct dpa_stats_cnt_cb *cnt_cb,
				 const struct dpa_stats_cls_cnt_params *params)
{
	struct dpa_stats *dpa_stats = cnt_cb->dpa_stats;
	struct dpa_stats_cls_cnt_classif_node prm = params->classif_node_params;
	uint32_t i = 0;
	int err = 0;

	if (!dpa_stats) {
		pr_err("Invalid argument: NULL DPA Stats instance\n");
		return -EFAULT;
	}

	/* Check Classification Cc Node counter selection */
	if (prm.cnt_sel == 0 ||  prm.cnt_sel > DPA_STATS_CNT_CLASSIF_ALL) {
		pr_err("Invalid Classif_Node counter selection\n");
		return -EINVAL;
	}

	cnt_cb->ccnode_cb.cc_node = prm.cc_node;
	cnt_cb->members_num = params->class_members;

	for (i = 0; i < params->class_members; i++) {
		/* Copy the key descriptor */
		err = copy_key_descriptor(&prm.keys[i],
				&cnt_cb->ccnode_cb.keys[i]);
		if (err != 0) {
			pr_err("Unable to copy key descriptor\n");
			return -EINVAL;
		}

		/* Check the Classifier Node counter parameters */
		err = check_ccnode_counter(cnt_cb,
				prm.ccnode_type, &cnt_cb->ccnode_cb.keys[i]);
		if (err != 0) {
			pr_err("Invalid Classif Node counter parameters\n");
			return -EINVAL;
		}
	}

	/* Map Classif Node counter selection to CcNode statistics */
	cnt_sel_to_stats(&cnt_cb->info,
			 dpa_stats->stats_sel[DPA_STATS_CNT_CLASSIF_NODE],
			 prm.cnt_sel >> CLASSIF_STATS_SHIFT);

	/* Set number of bytes that will be written by this counter */
	cnt_cb->bytes_num = cnt_cb->members_num *
				STATS_VAL_SIZE * cnt_cb->info.stats_num;
	return 0;
}

static int set_cls_cnt_ipsec_cb(struct dpa_stats_cnt_cb *cnt_cb,
				const struct dpa_stats_cls_cnt_params *prm)
{
	struct dpa_stats_cnt_ipsec_cb *cnt_ipsec_cb = &cnt_cb->ipsec_cb;
	struct dpa_stats *dpa_stats = cnt_cb->dpa_stats;
	struct dpa_ipsec_sa_stats stats;
	uint32_t cnt_sel = prm->ipsec_params.cnt_sel, i = 0;
	int err = 0;

	if (!dpa_stats) {
		pr_err("Invalid argument: NULL DPA Stats instance\n");
		return -EFAULT;
	}

	/* Map IPSec counter selection to statistics */
	if (cnt_sel == DPA_STATS_CNT_NUM_OF_BYTES) {
		cnt_cb->info.stats_off[0] = dpa_stats->stats_sel[
			DPA_STATS_CNT_IPSEC][DPA_STATS_CNT_NUM_OF_BYTES];
		cnt_cb->info.stats_num = 1;
	} else if (cnt_sel  == DPA_STATS_CNT_NUM_OF_PACKETS) {
		cnt_cb->info.stats_off[0] = dpa_stats->stats_sel[
			DPA_STATS_CNT_IPSEC][DPA_STATS_CNT_NUM_OF_PACKETS];
		cnt_cb->info.stats_num = 1;
	} else if (cnt_sel  == DPA_STATS_CNT_NUM_ALL) {
		cnt_cb->info.stats_off[0] = dpa_stats->stats_sel[
			DPA_STATS_CNT_IPSEC][DPA_STATS_CNT_NUM_OF_BYTES];
		cnt_cb->info.stats_off[1] = dpa_stats->stats_sel[
			DPA_STATS_CNT_IPSEC][DPA_STATS_CNT_NUM_OF_PACKETS];
		cnt_cb->info.stats_num = 2;
	} else {
		pr_err("Invalid IPSec counter selection\n");
		return -EINVAL;
	}

	cnt_cb->members_num = prm->class_members;

	for (i = 0; i < prm->class_members; i++) {
		if (prm->ipsec_params.sa_id[i] != DPA_OFFLD_INVALID_OBJECT_ID) {
			cnt_ipsec_cb->sa_id[i] = prm->ipsec_params.sa_id[i];
			cnt_ipsec_cb->valid[i] = TRUE;

			err = dpa_ipsec_sa_get_stats(cnt_cb->ipsec_cb.sa_id[i],
					&stats);
			if (err < 0) {
				pr_err("Invalid IPSec counter parameters\n");
				return -EINVAL;
			}
		} else {
			cnt_ipsec_cb->valid[i] = FALSE;
		}
	}

	/* Set number of bytes that will be written by this counter */
	cnt_cb->bytes_num = cnt_cb->members_num *
			STATS_VAL_SIZE * cnt_cb->info.stats_num;
	return 0;
}

int set_classif_tbl_member(const struct dpa_stats_cls_member_params *prm,
			   int member_index,
			   struct dpa_stats_cnt_cb *cnt_cb)
{
	struct dpa_stats_cnt_classif_tbl_cb *tbl_cb = &cnt_cb->tbl_cb;
	uint32_t i = 0;
	int err = 0;

	/* Check that counter is of type Classifier table */
	if (cnt_cb->type != DPA_STATS_CNT_CLASSIF_TBL) {
		pr_err("Operation permitted only on counter "
				"type DPA_STATS_CNT_CLASSIF_TBL\n");
		return -EINVAL;
	}

	/* Check that member index does not exceeds class size */
	if (member_index < 0 || member_index >= cnt_cb->members_num) {
		pr_err("Member index is out of class counter size\n");
		return -EINVAL;
	}

	/* Release the old key memory */
	kfree(tbl_cb->keys[member_index].key.byte);
	tbl_cb->keys[member_index].key.byte = NULL;

	kfree(tbl_cb->keys[member_index].key.mask);
	tbl_cb->keys[member_index].key.mask = NULL;

	/* Reset the statistics */
	for (i = 0; i < cnt_cb->info.stats_num; i++) {
		cnt_cb->info.stats[member_index][i] = 0;
		cnt_cb->info.last_stats[member_index][i] = 0;
	}

	if ((prm->type == DPA_STATS_CLS_MEMBER_SINGLE_KEY && !prm->key.byte) ||
	    (prm->type == DPA_STATS_CLS_MEMBER_PAIR_KEY &&
			    !prm->pair.first_key.byte)) {
		/* Mark the key as invalid */
		tbl_cb->keys[member_index].valid = FALSE;
		return 0;
	} else {
		tbl_cb->keys[member_index].valid = TRUE;

		if (prm->type == DPA_STATS_CLS_MEMBER_SINGLE_KEY) {
			/* Copy the key descriptor */
			err = copy_key_descriptor(&prm->key,
					&tbl_cb->keys[member_index].key);
			if (err != 0) {
				pr_err("Unable to copy key descriptor\n");
				return -EINVAL;
			}
		} else {
			err = set_cls_cnt_classif_tbl_pair(tbl_cb, tbl_cb->td,
				&prm->pair, &tbl_cb->keys[member_index]);
			if (err != 0) {
				pr_err("Unable to configure the key pair\n");
				return -EINVAL;
			}
		}
		if (cnt_cb->f_get_cnt_stats != get_cnt_cls_tbl_frag_stats) {
			err = check_tbl_cls_counter(cnt_cb,
					&tbl_cb->keys[member_index]);
			if (err != 0)
				return -EINVAL;
		} else{
			err = set_frag_manip(tbl_cb->td,
					&tbl_cb->keys[member_index]);
			if (err < 0) {
				pr_err("Invalid Fragmentation manip handle\n");
				return -EINVAL;
			}
		}
	}

	return 0;
}

int set_ipsec_member(const struct dpa_stats_cls_member_params *params,
		     int member_idx,
		     struct dpa_stats_cnt_cb *cnt_cb)
{
	struct dpa_stats_cnt_ipsec_cb *ipsec_cb = &cnt_cb->ipsec_cb;
	uint32_t i = 0;

	/* Check that counter is of type IPSec */
	if (cnt_cb->type != DPA_STATS_CNT_IPSEC) {
		pr_err("Operation permitted only on counter "
				"type DPA_STATS_CNT_IPSEC\n");
		return -EINVAL;
	}

	/* Check that member index does not exceeds class size */
	if (member_idx < 0 || member_idx >= cnt_cb->members_num) {
		pr_err("Member index is out of class counter size\n");
		return -EINVAL;
	}

	/* Reset the statistics */
	for (i = 0; i < cnt_cb->info.stats_num; i++) {
		cnt_cb->info.stats[member_idx][i] = 0;
		cnt_cb->info.last_stats[member_idx][i] = 0;
	}

	if (params->sa_id == DPA_OFFLD_INVALID_OBJECT_ID) {
		/* Mark that corresponding SA id as invalid */
		ipsec_cb->valid[member_idx] = FALSE;
	} else {
		/* Mark the corresponding SA id as valid */
		ipsec_cb->valid[member_idx] = TRUE;
		ipsec_cb->sa_id[member_idx] = params->sa_id;
	}

	return 0;
}

static inline void get_cnt_32bit_stats(struct dpa_stats_req_cb *req_cb,
				       struct stats_info *stats_info,
				       void *stats, uint32_t idx)
{
	uint32_t j = 0;
	uint64_t stats_val;

	for (j = 0; j < stats_info->stats_num; j++) {

		if (stats_info->stats_off[j] == UNSUPPORTED_CNT_SEL) {
			/* Write the memory location */
			memset(req_cb->request_area, 0, STATS_VAL_SIZE);

			/* Update the memory pointer */
			req_cb->request_area += STATS_VAL_SIZE;
			continue;
		}

		/* Get statistics value */
		stats_val = (uint64_t)(*((uint32_t *)
				(stats + stats_info->stats_off[j])));

		/* Check for rollover */
		if (stats_val < stats_info->last_stats[idx][j])
			stats_info->stats[idx][j] +=
				((unsigned long int)0xffffffff -
				stats_info->last_stats[idx][j]) + stats_val;
		else
			stats_info->stats[idx][j] += stats_val -
				stats_info->last_stats[idx][j];

		/* Store the current value as the last read value */
		stats_info->last_stats[idx][j] = stats_val;

		/* Write the memory location */
		*(uint32_t *)(req_cb->request_area) =
				(uint32_t)stats_info->stats[idx][j];

		/* Update the memory pointer */
		req_cb->request_area += STATS_VAL_SIZE;

		if (stats_info->reset)
			stats_info->stats[idx][j] = 0;
	}
}

static inline void get_cnt_64bit_stats(struct dpa_stats_req_cb *req_cb,
				       struct stats_info *stats_info,
				       void *stats, uint32_t idx)
{
	uint32_t j = 0;
	uint64_t stats_val;

	for (j = 0; j < stats_info->stats_num; j++) {
		/* Get statistics value */
		stats_val = *((uint64_t *)(stats + stats_info->stats_off[j]));

		/* Check for rollover */
		if (stats_val < stats_info->last_stats[idx][j])
			stats_info->stats[idx][j] +=
				((unsigned long int)0xffffffff -
				stats_info->last_stats[idx][j]) + stats_val;
		else
			stats_info->stats[idx][j] += stats_val -
				stats_info->last_stats[idx][j];

		/* Store the current value as the last read value */
		stats_info->last_stats[idx][j] = stats_val;

		/* Write the memory location */
		*(uint32_t *)(req_cb->request_area) =
				(uint32_t)stats_info->stats[idx][j];

		/* Update the memory pointer */
		req_cb->request_area += STATS_VAL_SIZE;

		if (stats_info->reset)
			stats_info->stats[idx][j] = 0;
	}
}

static int get_cnt_eth_stats(struct dpa_stats_req_cb *req_cb,
			     struct dpa_stats_cnt_cb *cnt_cb)
{
	t_FmMacStatistics stats;
	uint32_t i = 0;
	int err = 0;

	for (i = 0; i < cnt_cb->members_num; i++) {
		err = FM_MAC_GetStatistics(cnt_cb->gen_cb.objs[i], &stats);
		if (err != 0) {
			pr_err("Couldn't retrieve Ethernet Counter value\n");
			return -ENOENT;
		}

		get_cnt_64bit_stats(req_cb, &cnt_cb->info, (void *)&stats, i);
	}

	return 0;
}

static int get_cnt_reass_stats(struct dpa_stats_req_cb *req_cb,
			       struct dpa_stats_cnt_cb *cnt_cb)
{
	struct t_FmPcdManipStats stats;
	uint32_t i = 0;
	int err = 0;

	for (i = 0; i < cnt_cb->members_num; i++) {
		err = FM_PCD_ManipGetStatistics(cnt_cb->gen_cb.objs[i], &stats);
		if (err < 0) {
			pr_err("Couldn't retrieve Reassembly statistics\n");
			return -ESRCH;
		}

		get_cnt_32bit_stats(req_cb, &cnt_cb->info,
				&stats.u.reassem.u.ipReassem, i);
	}

	return 0;
}

static int get_cnt_frag_stats(struct dpa_stats_req_cb *req_cb,
			      struct dpa_stats_cnt_cb *cnt_cb)
{
	struct t_FmPcdManipStats stats;
	uint32_t i = 0;
	int err = 0;

	for (i = 0; i < cnt_cb->members_num; i++) {
		err = FM_PCD_ManipGetStatistics(cnt_cb->gen_cb.objs[i], &stats);
		if (err < 0) {
			pr_err("Couldn't retrieve Fragmentation statistics\n");
			return -EINTR;
		}

		get_cnt_32bit_stats(req_cb, &cnt_cb->info,
				&stats.u.frag.u.ipFrag, i);
	}

	return 0;
}

static int get_cnt_plcr_stats(struct dpa_stats_req_cb *req_cb,
			      struct dpa_stats_cnt_cb *cnt_cb)
{
	struct stats_info *info = &cnt_cb->info;
	uint64_t stats_val = 0;
	uint32_t i = 0, j = 0;

	for (i = 0; i < cnt_cb->members_num; i++) {
		for (j = 0; j < info->stats_num; j++) {
			stats_val = (uint64_t)FM_PCD_PlcrProfileGetCounter(
				cnt_cb->gen_cb.objs[i], info->stats_off[j]);

			/* Check for rollover */
			if (stats_val < info->last_stats[i][j])
				info->stats[i][j] +=
					((unsigned long int)0xffffffff -
					info->last_stats[i][j]) + stats_val;
			else
				info->stats[i][j] += stats_val -
					info->last_stats[i][j];

			/* Store the current value as the last read value */
			info->last_stats[i][j] = stats_val;

			/* Write the memory location */
			*(uint32_t *)(req_cb->request_area) =
					(uint32_t)info->stats[i][j];

			/* Update the memory pointer */
			req_cb->request_area += STATS_VAL_SIZE;

			if (info->reset)
				info->stats[i][j] = 0;
		}
	}

	return 0;
}

static int get_cnt_cls_tbl_match_stats(struct dpa_stats_req_cb *req_cb,
				       struct dpa_stats_cnt_cb *cnt_cb)
{
	t_FmPcdCcKeyStatistics stats;
	uint32_t i = 0;
	int err = 0;

	for (i = 0; i < cnt_cb->members_num; i++) {
		if (!cnt_cb->tbl_cb.keys[i].valid) {
			/* Write the memory location */
			memset(req_cb->request_area, 0,
				cnt_cb->info.stats_num * STATS_VAL_SIZE);

			/* Update the memory pointer */
			req_cb->request_area += STATS_VAL_SIZE *
					cnt_cb->info.stats_num;
			continue;
		}
		err = FM_PCD_MatchTableFindNGetKeyStatistics(
				cnt_cb->tbl_cb.keys[i].cc_node,
				cnt_cb->tbl_cb.keys[i].key.size,
				cnt_cb->tbl_cb.keys[i].key.byte,
				cnt_cb->tbl_cb.keys[i].key.mask, &stats);
		if (err != 0) {
			pr_err("Couldn't retrieve Classif Table statistics\n");
			return -EIO;
		}
		get_cnt_32bit_stats(req_cb, &cnt_cb->info, &stats, i);
	}

	return 0;
}

static int get_cnt_cls_tbl_hash_stats(struct dpa_stats_req_cb *req_cb,
				      struct dpa_stats_cnt_cb *cnt_cb)
{
	t_FmPcdCcKeyStatistics stats;
	uint32_t i = 0;
	int err = 0;

	for (i = 0; i < cnt_cb->members_num; i++) {
		if (!cnt_cb->tbl_cb.keys[i].valid) {
			/* Write the memory location */
			memset(req_cb->request_area, 0,
				cnt_cb->info.stats_num * STATS_VAL_SIZE);

			/* Update the memory pointer */
			req_cb->request_area += STATS_VAL_SIZE *
					cnt_cb->info.stats_num;
			continue;
		}
		err = FM_PCD_HashTableFindNGetKeyStatistics(
				cnt_cb->tbl_cb.keys[i].cc_node,
				cnt_cb->tbl_cb.keys[i].key.size,
				cnt_cb->tbl_cb.keys[i].key.byte,
				&stats);
		if (err != 0) {
			pr_err("Couldn't retrieve Classif Table statistics\n");
			return -EIO;
		}
		get_cnt_32bit_stats(req_cb, &cnt_cb->info, &stats, i);
	}

	return 0;
}

static int get_cnt_cls_tbl_index_stats(struct dpa_stats_req_cb *req_cb,
				       struct dpa_stats_cnt_cb *cnt_cb)
{
	t_FmPcdCcKeyStatistics stats;
	uint32_t i = 0;
	int err = 0;

	for (i = 0; i < cnt_cb->members_num; i++) {
		if (!cnt_cb->tbl_cb.keys[i].valid) {
			/* Write the memory location */
			memset(req_cb->request_area, 0,
				cnt_cb->info.stats_num * STATS_VAL_SIZE);

			/* Update the memory pointer */
			req_cb->request_area += STATS_VAL_SIZE *
					cnt_cb->info.stats_num;
			continue;
		}
		err = FM_PCD_MatchTableGetKeyStatistics(
				cnt_cb->tbl_cb.keys[i].cc_node,
				cnt_cb->tbl_cb.keys[i].key.byte[0],
				&stats);
		if (err != 0) {
			pr_err("Couldn't retrieve Classif Table statistics\n");
			return -EIO;
		}
		get_cnt_32bit_stats(req_cb, &cnt_cb->info, &stats, i);
	}

	return 0;
}

static int get_cnt_cls_tbl_frag_stats(struct dpa_stats_req_cb *req_cb,
				      struct dpa_stats_cnt_cb *cnt_cb)
{
	struct t_FmPcdManipStats stats;
	uint32_t i = 0;
	int err = 0;

	for (i = 0; i < cnt_cb->members_num; i++) {
		if (!cnt_cb->tbl_cb.keys[i].valid) {
			/* Write the memory location */
			memset(req_cb->request_area, 0,
				cnt_cb->info.stats_num * STATS_VAL_SIZE);

			/* Update the memory pointer */
			req_cb->request_area += STATS_VAL_SIZE *
					cnt_cb->info.stats_num;
			continue;
		}

		err = FM_PCD_ManipGetStatistics(
				cnt_cb->tbl_cb.keys[i].frag, &stats);
		if (err < 0) {
			pr_err("Couldn't retrieve Fragmentation statistics\n");
			return -EINTR;
		}
		get_cnt_32bit_stats(req_cb,
				&cnt_cb->info, &stats.u.frag.u.ipFrag, i);
	}

	return 0;
}

static int get_cnt_ccnode_match_stats(struct dpa_stats_req_cb *req_cb,
				      struct dpa_stats_cnt_cb *cnt_cb)
{
	t_FmPcdCcKeyStatistics stats;
	uint32_t i = 0;
	int err = 0;

	for (i = 0; i < cnt_cb->members_num; i++) {
		err = FM_PCD_MatchTableFindNGetKeyStatistics(
				cnt_cb->ccnode_cb.cc_node,
				cnt_cb->ccnode_cb.keys[i].size,
				cnt_cb->ccnode_cb.keys[i].byte,
				cnt_cb->ccnode_cb.keys[i].mask, &stats);
		if (err != 0) {
			pr_err("Couldn't retrieve Classif Node statistics\n");
			return -ENXIO;
		}

		get_cnt_32bit_stats(req_cb, &cnt_cb->info, (void *)&stats, i);
	}
	return 0;
}

static int get_cnt_ccnode_hash_stats(struct dpa_stats_req_cb *req_cb,
				     struct dpa_stats_cnt_cb *cnt_cb)
{
	t_FmPcdCcKeyStatistics stats;
	uint32_t i = 0;
	int err = 0;

	for (i = 0; i < cnt_cb->members_num; i++) {
		err = FM_PCD_HashTableFindNGetKeyStatistics(
				cnt_cb->ccnode_cb.cc_node,
				cnt_cb->ccnode_cb.keys[i].size,
				cnt_cb->ccnode_cb.keys[i].byte, &stats);
		if (err != 0) {
			pr_err("Couldn't retrieve Classif Node statistics\n");
			return -ENXIO;
		}

		get_cnt_32bit_stats(req_cb, &cnt_cb->info, (void *)&stats, i);
	}
	return 0;
}

static int get_cnt_ccnode_index_stats(struct dpa_stats_req_cb *req_cb,
				      struct dpa_stats_cnt_cb *cnt_cb)
{
	t_FmPcdCcKeyStatistics stats;
	uint32_t i = 0;
	int err = 0;

	for (i = 0; i < cnt_cb->members_num; i++) {
		err = FM_PCD_MatchTableGetKeyStatistics(
				cnt_cb->ccnode_cb.cc_node,
				cnt_cb->ccnode_cb.keys[i].byte[0], &stats);
		if (err != 0) {
			pr_err("Couldn't retrieve Classif Node statistics\n");
			return -ENXIO;
		}

		get_cnt_32bit_stats(req_cb, &cnt_cb->info, (void *)&stats, i);
	}
	return 0;
}

static int get_cnt_ipsec_stats(struct dpa_stats_req_cb *req_cb,
			       struct dpa_stats_cnt_cb *cnt_cb)
{
	struct dpa_ipsec_sa_stats stats;
	uint32_t i = 0;
	int err = 0;

	for (i = 0; i < cnt_cb->members_num; i++) {
		if (!cnt_cb->ipsec_cb.valid[i]) {
			/* Write the memory location */
			memset(req_cb->request_area, 0,
				cnt_cb->info.stats_num * STATS_VAL_SIZE);

			/* Update the memory pointer */
			req_cb->request_area +=
				STATS_VAL_SIZE * cnt_cb->info.stats_num;

			continue;
		}

		err = dpa_ipsec_sa_get_stats(cnt_cb->ipsec_cb.sa_id[i], &stats);
		if (err < 0) {
			pr_err("Couldn't retrieve IPSec statistics\n");
			return -E2BIG;
		}

		get_cnt_32bit_stats(req_cb, &cnt_cb->info, &stats, i);
	}

	return 0;
}

static void async_req_work_func(struct work_struct *work)
{
	struct dpa_stats_req_cb *req_cb = NULL;
	struct dpa_stats *dpa_stats = NULL;
	int err = 0;

	dpa_stats = gbl_dpa_stats;

	req_cb = container_of(work, struct dpa_stats_req_cb, async_req_work);
	BUG_ON(!req_cb);

	err = treat_cnts_request(dpa_stats, req_cb);
	if (err < 0) {
		pr_err("Failed to retrieve counter values\n");
		req_cb->bytes_num = err;
	}

	/* Notify the application */
	req_cb->request_done(0, req_cb->config.storage_area_offset,
			req_cb->cnts_num, req_cb->bytes_num);

	/* Release the request control block */
	err = put_req(dpa_stats, req_cb);
	if (err < 0)
		pr_err("Failed to release request control block\n");

	return;
}

int dpa_stats_init(const struct dpa_stats_params *params, int *dpa_stats_id)
{
	struct dpa_stats *dpa_stats = NULL;
	int err = 0;

	/* Multiple DPA Stats instances are not currently supported */
	unused(dpa_stats_id);

	/* Sanity checks */
	if (gbl_dpa_stats) {
		pr_err("dpa_stats component already initialized.\n");
		pr_err("Multiple DPA Stats Instances are not supported.\n");
		return -EPERM;
	}

	/* Check user-provided parameters */
	err = check_dpa_stats_params(params);
	if (err < 0)
		return err;

	/* Control block allocation */
	dpa_stats = kzalloc(sizeof(struct dpa_stats), GFP_KERNEL);
	if (!dpa_stats) {
		pr_err("Could not allocate memory for control block.\n");
		return -ENOMEM;
	}

	/* Store parameters */
	dpa_stats->config = *params;

	/* Initialize DPA Stats instance lock */
	mutex_init(&dpa_stats->lock);
	mutex_init(&dpa_stats->sched_cnt_lock);

	/* Allocate and initialize resources occupied by counters */
	err = init_cnts_resources(dpa_stats);
	if (err < 0) {
		free_resources();
		return err;
	}

	/* Allocate and initialize requests control block  */
	err = init_reqs_resources(dpa_stats);
	if (err < 0) {
		free_resources();
		return err;
	}

	/* Map each Ethernet counter selection to a FM-MAC statistics */
	create_cnt_eth_stats(dpa_stats);

	/* Map Reassembly counters to FMAN Reassembly statistics */
	create_cnt_reass_stats(dpa_stats);

	/* Map Fragmentation counters to FMAN Fragmentation statistics */
	create_cnt_frag_stats(dpa_stats);

	/* Map Policer counters to FMAN Policer statistics */
	create_cnt_plcr_stats(dpa_stats);

	/* Map Classifier counters to FMAN Classifier statistics */
	create_classif_stats(dpa_stats);

	/* Map IPSec counters  */
	create_cnt_ipsec_stats(dpa_stats);

	gbl_dpa_stats = dpa_stats;

	return 0;
}
EXPORT_SYMBOL(dpa_stats_init);

int dpa_stats_create_counter(int dpa_stats_id,
			     const struct dpa_stats_cnt_params *params,
			     int *dpa_stats_cnt_id)
{
	struct dpa_stats *dpa_stats = NULL;
	struct dpa_stats_cnt_cb *cnt_cb = NULL;
	int err = 0, err_rb = 0;

	/* multiple DPA Stats instances are not currently supported */
	unused(dpa_stats_id);

	if (!gbl_dpa_stats) {
		pr_err("dpa_stats component is not initialized\n");
		return -EPERM;
	}

	if (!dpa_stats_cnt_id) {
		pr_err("dpa_stats_cnt_id can't be NULL\n");
		return -EINVAL;
	}
	*dpa_stats_cnt_id = DPA_OFFLD_INVALID_OBJECT_ID;

	dpa_stats = gbl_dpa_stats;

	err = get_new_cnt(dpa_stats, &cnt_cb);
	if (err < 0) {
		pr_err("Failed retrieving a preallocated counter\n");
		return err;
	}

	/* Acquire the lock for the counter control block */
	mutex_lock(&cnt_cb->lock);

	switch (params->type) {
	case DPA_STATS_CNT_ETH:
		cnt_cb->type = DPA_STATS_CNT_ETH;
		cnt_cb->f_get_cnt_stats = get_cnt_eth_stats;

		err = set_cnt_eth_cb(cnt_cb, params);
		if (err != 0) {
			pr_err("Failed to create ETH counter\n");
			goto create_counter_err;
		}
		break;
	case DPA_STATS_CNT_REASS:
		cnt_cb->type = DPA_STATS_CNT_REASS;
		cnt_cb->f_get_cnt_stats = get_cnt_reass_stats;

		err = set_cnt_reass_cb(cnt_cb, params);
		if (err != 0) {
			pr_err("Failed to create Reassembly counter\n");
			goto create_counter_err;
		}
		break;
	case DPA_STATS_CNT_FRAG:
		cnt_cb->type = DPA_STATS_CNT_FRAG;
		cnt_cb->f_get_cnt_stats = get_cnt_frag_stats;

		err = set_cnt_frag_cb(cnt_cb, params);
		if (err != 0) {
			pr_err("Failed to create Fragmentation counter\n");
			goto create_counter_err;
		}
		break;
	case DPA_STATS_CNT_POLICER:
		cnt_cb->type = DPA_STATS_CNT_POLICER;
		cnt_cb->f_get_cnt_stats = get_cnt_plcr_stats;

		err = set_cnt_plcr_cb(cnt_cb, params);
		if (err != 0) {
			pr_err("Failed to create Policer counter\n");
			goto create_counter_err;
		}
		break;
	case DPA_STATS_CNT_CLASSIF_TBL:
		cnt_cb->type = DPA_STATS_CNT_CLASSIF_TBL;

		err = set_cnt_classif_tbl_cb(cnt_cb, params);
		if (err != 0) {
			pr_err("Failed to create Classif Table counter\n");
			goto create_counter_err;
		}
		break;
	case DPA_STATS_CNT_CLASSIF_NODE:
		cnt_cb->type = DPA_STATS_CNT_CLASSIF_NODE;

		err = set_cnt_ccnode_cb(cnt_cb, params);
		if (err != 0) {
			pr_err("Failed to create Classif Cc Node counter\n");
			goto create_counter_err;
		}
		break;
	case DPA_STATS_CNT_IPSEC:
		cnt_cb->type = DPA_STATS_CNT_IPSEC;
		cnt_cb->f_get_cnt_stats = get_cnt_ipsec_stats;

		err = set_cnt_ipsec_cb(cnt_cb, params);
		if (err != 0) {
			pr_err("Failed to create IPSec counter\n");
			goto create_counter_err;
		}
		break;
	case DPA_STATS_CNT_TRAFFIC_MNG:
		pr_err("Counter type not supported\n");
		mutex_unlock(&cnt_cb->lock);
		return -EINVAL;
	default:
		pr_err("Invalid counter type\n");
		mutex_unlock(&cnt_cb->lock);
		return -EINVAL;
	};

	/* Counter was created. Return the counter id */
	*dpa_stats_cnt_id = cnt_cb->id;

	/* Unlock the counter control block structure */
	mutex_unlock(&cnt_cb->lock);

	return 0;

create_counter_err:
	/*
	 * An invalid Counter ID is returned if 'put_cnt' succeeds and the
	 * actual reserved Counter ID if it fails. The Counter ID can be used
	 * to try again to free resources by calling dpa_stats_remove_counter
	 */

	*dpa_stats_cnt_id = cnt_cb->id;

	err_rb = put_cnt(dpa_stats, cnt_cb);
	if (!err_rb)
		*dpa_stats_cnt_id = DPA_OFFLD_INVALID_OBJECT_ID;

	/* Unlock the counter control block structure */
	mutex_unlock(&cnt_cb->lock);

	return err;
}
EXPORT_SYMBOL(dpa_stats_create_counter);

int dpa_stats_create_class_counter(int dpa_stats_id,
				  const struct dpa_stats_cls_cnt_params *params,
				  int *dpa_stats_cnt_id)
{
	struct dpa_stats *dpa_stats = NULL;
	struct dpa_stats_cnt_cb *cnt_cb = NULL;
	int err = 0, err_rb = 0;

	/* multiple DPA Stats instances are not currently supported */
	unused(dpa_stats_id);

	if (!gbl_dpa_stats) {
		pr_err("dpa_stats component is not initialized\n");
		return -EPERM;
	}

	if (!dpa_stats_cnt_id) {
		pr_err("dpa_stats_cnt_id can't be NULL\n");
		return -EINVAL;
	}
	*dpa_stats_cnt_id = DPA_OFFLD_INVALID_OBJECT_ID;

	if (params->class_members > DPA_STATS_MAX_NUM_OF_CLASS_MEMBERS) {
		pr_err("exceed maximum number of class members: %d\n",
				DPA_STATS_MAX_NUM_OF_CLASS_MEMBERS);
		return -EINVAL;
	}

	dpa_stats = gbl_dpa_stats;

	err = get_new_cnt(dpa_stats, &cnt_cb);
	if (err < 0) {
		pr_err("Failed retrieving a preallocated counter\n");
		return err;
	}

	/* Acquire the lock for the counter control block */
	mutex_lock(&cnt_cb->lock);

	switch (params->type) {
	case DPA_STATS_CNT_ETH:
		cnt_cb->type = DPA_STATS_CNT_ETH;
		cnt_cb->f_get_cnt_stats = get_cnt_eth_stats;

		err = set_cls_cnt_eth_cb(cnt_cb, params);
		if (err != 0) {
			pr_err("Failed to create ETH counter\n");
			goto create_counter_err;
		}
		break;
	case DPA_STATS_CNT_REASS:
		cnt_cb->type = DPA_STATS_CNT_REASS;
		cnt_cb->f_get_cnt_stats = get_cnt_reass_stats;

		err = set_cls_cnt_reass_cb(cnt_cb, params);
		if (err != 0) {
			pr_err("Failed to create Reassembly counter\n");
			goto create_counter_err;
		}
		break;
	case DPA_STATS_CNT_FRAG:
		cnt_cb->type = DPA_STATS_CNT_FRAG;
		cnt_cb->f_get_cnt_stats = get_cnt_frag_stats;

		err = set_cls_cnt_frag_cb(cnt_cb, params);
		if (err != 0) {
			pr_err("Failed to create Fragmentation counter\n");
			goto create_counter_err;
		}
		break;
	case DPA_STATS_CNT_POLICER:
		cnt_cb->type = DPA_STATS_CNT_POLICER;
		cnt_cb->f_get_cnt_stats = get_cnt_plcr_stats;

		err = set_cls_cnt_plcr_cb(cnt_cb, params);
		if (err != 0) {
			pr_err("Failed to create Policer counter\n");
			goto create_counter_err;
		}
		break;
	case DPA_STATS_CNT_CLASSIF_TBL:
		cnt_cb->type = DPA_STATS_CNT_CLASSIF_TBL;

		err = set_cls_cnt_classif_tbl_cb(cnt_cb, params);
		if (err != 0) {
			pr_err("Failed to create Classif Table counter\n");
			goto create_counter_err;
		}
		break;
	case DPA_STATS_CNT_CLASSIF_NODE:
		cnt_cb->type = DPA_STATS_CNT_CLASSIF_NODE;

		err = set_cls_cnt_ccnode_cb(cnt_cb, params);
		if (err != 0) {
			pr_err("Failed to create Classif Cc Node counter\n");
			goto create_counter_err;
		}
		break;
	case DPA_STATS_CNT_IPSEC:
		cnt_cb->type = DPA_STATS_CNT_IPSEC;
		cnt_cb->f_get_cnt_stats = get_cnt_ipsec_stats;

		err = set_cls_cnt_ipsec_cb(cnt_cb, params);
		if (err != 0) {
			pr_err("Failed to create IPSec counter\n");
			goto create_counter_err;
		}

		break;
	case DPA_STATS_CNT_TRAFFIC_MNG:
		pr_err("Counter type not supported\n");
		mutex_unlock(&cnt_cb->lock);
		return -EINVAL;
	default:
		pr_err("Invalid counter type\n");
		mutex_unlock(&cnt_cb->lock);
		return -EINVAL;
	};

	/* Counter was created. Return the counter id */
	*dpa_stats_cnt_id = cnt_cb->id;

	/* Unlock the counter control block */
	mutex_unlock(&cnt_cb->lock);

	return 0;

create_counter_err:
	/*
	 * An invalid Counter ID is returned if 'put_cnt' succeeds and the
	 * actual reserved Counter ID if it fails. The Counter ID can be used
	 * to try again to free resources by calling dpa_stats_remove_counter
	 */
	*dpa_stats_cnt_id = cnt_cb->id;

	err_rb = put_cnt(dpa_stats, cnt_cb);
	if (!err_rb)
		*dpa_stats_cnt_id = DPA_OFFLD_INVALID_OBJECT_ID;

	/* Unlock the counter control block */
	mutex_unlock(&cnt_cb->lock);

	return err;
}
EXPORT_SYMBOL(dpa_stats_create_class_counter);

int dpa_stats_modify_class_counter(int dpa_stats_cnt_id,
			const struct dpa_stats_cls_member_params *params,
			int member_index)
{
	struct dpa_stats *dpa_stats = NULL;
	struct dpa_stats_cnt_cb *cnt_cb = NULL;
	int err = 0;

	if (!gbl_dpa_stats) {
		pr_err("dpa_stats component is not initialized\n");
		return -EPERM;
	}

	dpa_stats = gbl_dpa_stats;

	if (dpa_stats_cnt_id < 0 ||
			dpa_stats_cnt_id > dpa_stats->config.max_counters) {
		pr_err("Invalid Counter id %d provided\n", dpa_stats_cnt_id);
		return -EINVAL;
	}

	/* Counter scheduled for the retrieve mechanism can't be modified */
	if (cnt_is_sched(dpa_stats, dpa_stats_cnt_id)) {
		pr_err("Counter id %d is in use\n", dpa_stats_cnt_id);
		return -EBUSY;
	}

	/* Get counter control block */
	cnt_cb = &dpa_stats->cnts_cb[dpa_stats_cnt_id];

	/* Acquire counter control block lock */
	err = mutex_trylock(&cnt_cb->lock);
	if (err == 0)
		return -EAGAIN;

	/* Validity check for this counter */
	if (cnt_cb->index == DPA_OFFLD_INVALID_OBJECT_ID) {
		pr_err("Invalid Counter id %d provided\n", dpa_stats_cnt_id);
		mutex_unlock(&cnt_cb->lock);
		return -EINVAL;
	}

	if (params->type == DPA_STATS_CLS_MEMBER_SINGLE_KEY ||
		params->type == DPA_STATS_CLS_MEMBER_PAIR_KEY) {
		/* Modify classifier table class member */
		err = set_classif_tbl_member(params, member_index, cnt_cb);
		if (err < 0) {
			pr_err("Failed to modify class member\n");
			mutex_unlock(&cnt_cb->lock);
			return -EINVAL;
		}

	} else if (params->type == DPA_STATS_CLS_MEMBER_SA_ID) {
		/* Modify IPSec class member */
		err = set_ipsec_member(params, member_index, cnt_cb);
		if (err < 0) {
			pr_err("Failed to modify class member\n");
			mutex_unlock(&cnt_cb->lock);
			return -EINVAL;
		}
	} else {
		pr_err("Invalid member type\n");
		mutex_unlock(&cnt_cb->lock);
		return -EINVAL;
	}

	/* Unlock the counter control block */
	mutex_unlock(&cnt_cb->lock);

	return 0;
}
EXPORT_SYMBOL(dpa_stats_modify_class_counter);

int dpa_stats_remove_counter(int dpa_stats_cnt_id)
{
	struct dpa_stats *dpa_stats = NULL;
	struct dpa_stats_cnt_cb *cnt_cb = NULL;
	int err = 0;
	uint32_t i;

	if (!gbl_dpa_stats) {
		pr_err("dpa_stats component is not initialized\n");
		return -EPERM;
	}

	dpa_stats = gbl_dpa_stats;

	if (dpa_stats_cnt_id < 0 ||
			dpa_stats_cnt_id > dpa_stats->config.max_counters) {
		pr_err("Invalid Counter id %d provided\n", dpa_stats_cnt_id);
		return -EINVAL;
	}

	/* Counter scheduled for the retrieve mechanism can't be removed */
	if (cnt_is_sched(dpa_stats, dpa_stats_cnt_id)) {
		pr_err("Counter id %d is in use\n", dpa_stats_cnt_id);
		return -EBUSY;
	}

	/* Get counter control block */
	cnt_cb = &dpa_stats->cnts_cb[dpa_stats_cnt_id];

	/* Acquire counter control block lock */
	err = mutex_trylock(&cnt_cb->lock);
	if (err == 0)
		return -EAGAIN;

	/* Validity check for this counter */
	if (cnt_cb->index == DPA_OFFLD_INVALID_OBJECT_ID) {
		pr_err("Invalid Counter id %d provided\n", dpa_stats_cnt_id);
		mutex_unlock(&cnt_cb->lock);
		return -EINVAL;
	}

	/* Remove the allocated memory for keys bytes and masks */
	if (cnt_cb->type == DPA_STATS_CNT_CLASSIF_NODE)
		for (i = 0; i < cnt_cb->members_num; i++) {
			kfree(cnt_cb->ccnode_cb.keys[i].byte);
			kfree(cnt_cb->ccnode_cb.keys[i].mask);
		}

	/* Remove the allocated memory for keys bytes and masks */
	if (cnt_cb->type == DPA_STATS_CNT_CLASSIF_TBL)
		for (i = 0; i < cnt_cb->members_num; i++) {
			kfree(cnt_cb->tbl_cb.keys[i].key.byte);
			kfree(cnt_cb->tbl_cb.keys[i].key.mask);
		}

	/* Release the counter id in the Counter IDs circular queue */
	err = put_cnt(dpa_stats, cnt_cb);
	if (err < 0) {
		pr_err("Failed to release a preallocated counter\n");
		mutex_unlock(&cnt_cb->lock);
		return -EINVAL;
	}

	/* Release counter lock */
	mutex_unlock(&cnt_cb->lock);

	return 0;
}
EXPORT_SYMBOL(dpa_stats_remove_counter);

int dpa_stats_get_counters(struct dpa_stats_cnt_request_params params,
			   int *cnts_len,
			   dpa_stats_request_cb request_done)
{
	struct dpa_stats *dpa_stats = NULL;
	struct dpa_stats_req_cb *req_cb = NULL;
	struct dpa_stats_cnt_cb *cnt_cb = NULL;
	int err = 0, cnt_id = 0, req_id = 0;
	uint32_t i = 0;

	if (!gbl_dpa_stats) {
		pr_err("dpa_stats component is not initialized\n");
		return -EPERM;
	}

	/* Check user-provided cnts_len pointer */
	if (!cnts_len) {
		pr_err("Parameter cnts_len can't be NULL\n");
		return -EINVAL;
	}

	/* Check user-provided params.cnts_ids pointer */
	if (!params.cnts_ids) {
		pr_err("Parameter params.cnts_ids can't be NULL\n");
		return -EINVAL;
	}

	dpa_stats = gbl_dpa_stats;

	*cnts_len = 0;

	for (i = 0; i < params.cnts_ids_len; i++) {
		if (params.cnts_ids[i] == DPA_OFFLD_INVALID_OBJECT_ID ||
		    params.cnts_ids[i] > dpa_stats->config.max_counters) {
			pr_err("Invalid Counter id %d provided\n",
					params.cnts_ids[i]);
			return -EINVAL;
		}
	}

	block_sched_cnts(dpa_stats, params.cnts_ids, params.cnts_ids_len);

	/* Calculate number of bytes occupied by the counters */
	for (i = 0; i < params.cnts_ids_len; i++) {
		cnt_id = params.cnts_ids[i];

		/* Get counter's control block */
		cnt_cb = &dpa_stats->cnts_cb[cnt_id];

		/* Acquire counter lock */
		mutex_lock(&cnt_cb->lock);

		/* Check if counter control block is initialized */
		if (cnt_cb->index == DPA_OFFLD_INVALID_OBJECT_ID) {
			pr_err("Invalid Counter id %d provided\n", cnt_id);
			mutex_unlock(&cnt_cb->lock);
			unblock_sched_cnts(dpa_stats, params.cnts_ids,
					   params.cnts_ids_len);
			return -EINVAL;
		}

		*cnts_len += cnt_cb->bytes_num;
		mutex_unlock(&cnt_cb->lock);
	}

	/* Check user-provided parameters */
	if ((params.storage_area_offset + *cnts_len) >
		dpa_stats->config.storage_area_len) {
		pr_err("Invalid offset %d provided\n",
				params.storage_area_offset);
		unblock_sched_cnts(dpa_stats, params.cnts_ids,
				   params.cnts_ids_len);
		return -EINVAL;
	}

	/* Create a new request */
	err = get_new_req(dpa_stats, &req_id, &req_cb);
	if (err < 0) {
		pr_err("Failed retrieving a preallocated request\n");
		/* Release counters locks */
		unblock_sched_cnts(dpa_stats, params.cnts_ids,
				   params.cnts_ids_len);
		return err;
	}

	/* Store user-provided request parameters */
	memcpy(req_cb->config.cnts_ids,
			params.cnts_ids, params.cnts_ids_len * sizeof(int));

	req_cb->config.reset_cnts = params.reset_cnts;
	req_cb->config.storage_area_offset = params.storage_area_offset;
	req_cb->config.cnts_ids_len = params.cnts_ids_len;
	req_cb->request_done = request_done;

	/* Set memory area where the request should write */
	req_cb->request_area = dpa_stats->config.storage_area +
					params.storage_area_offset;

	if (!req_cb->request_done) {
		/* Call is synchronous */
		err = treat_cnts_request(dpa_stats, req_cb);
		if (err < 0)
			pr_err("Failed to retrieve counter values\n");

		err = put_req(dpa_stats, req_cb);

		return err;
	} else {
		/* Call is asynchronous */
		queue_work(dpa_stats->async_req_workqueue,
			   &req_cb->async_req_work);
	}

	return 0;
}
EXPORT_SYMBOL(dpa_stats_get_counters);

int dpa_stats_reset_counters(int *cnts_ids, unsigned int cnts_ids_len)
{
	struct dpa_stats *dpa_stats = NULL;
	struct dpa_stats_cnt_cb *cnt_cb = NULL;
	uint32_t i = 0;
	int err = 0;

	if (!gbl_dpa_stats) {
		pr_err("dpa_stats component is not initialized\n");
		return -EPERM;
	}

	/* Check user-provided cnts_len pointer */
	if (cnts_ids_len == 0) {
		pr_err("Parameter cnts_ids_len can't be 0\n");
		return -EINVAL;
	}

	/* Check user-provided cnts_ids pointer */
	if (!cnts_ids) {
		pr_err("Parameter cnts_ids can't be NULL\n");
		return -EINVAL;
	}

	dpa_stats = gbl_dpa_stats;

	for (i = 0; i < cnts_ids_len; i++)
		if (cnts_ids[i] == DPA_OFFLD_INVALID_OBJECT_ID ||
		    cnts_ids[i] > dpa_stats->config.max_counters) {
			pr_err("Invalid Counter id %d provided\n", cnts_ids[i]);
			return -EINVAL;
		}

	block_sched_cnts(dpa_stats, cnts_ids, cnts_ids_len);

	/* Calculate number of bytes occupied by the counters */
	for (i = 0; i < cnts_ids_len; i++) {
		/* Get counter's control block */
		cnt_cb = &dpa_stats->cnts_cb[cnts_ids[i]];

		/* Acquire counter lock */
		err = mutex_trylock(&cnt_cb->lock);
		if (err == 0) {
			pr_err("Counter %d is being used\n", cnts_ids[i]);
			unblock_sched_cnts(dpa_stats,
					   cnts_ids, cnts_ids_len);
			return -EBUSY;
		}

		/* Check if counter control block is initialized */
		if (cnt_cb->index == DPA_OFFLD_INVALID_OBJECT_ID) {
			pr_err("Invalid Counter id %d provided\n", cnts_ids[i]);
			mutex_unlock(&cnt_cb->lock);
			unblock_sched_cnts(dpa_stats,
					   cnts_ids, cnts_ids_len);
			return -EINVAL;
		}
		memset(&cnt_cb->info.stats, 0, (MAX_NUM_OF_MEMBERS *
		       MAX_NUM_OF_STATS * sizeof(uint64_t)));
		mutex_unlock(&cnt_cb->lock);
	}

	unblock_sched_cnts(dpa_stats, cnts_ids, cnts_ids_len);

	return 0;
}
EXPORT_SYMBOL(dpa_stats_reset_counters);

int dpa_stats_free(int dpa_stats_id)
{
	/* multiple DPA Stats instances are not currently supported */
	unused(dpa_stats_id);

	free_resources();

	return 0;
}
EXPORT_SYMBOL(dpa_stats_free);
