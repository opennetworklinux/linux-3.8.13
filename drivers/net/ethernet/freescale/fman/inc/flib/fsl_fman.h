/*
 * Copyright 2013 Freescale Semiconductor Inc.
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

#ifndef __FSL_FMAN_H
#define __FSL_FMAN_H

#include "common/general.h"


struct fman_ext_pool_params {
	uint8_t                 id;    /**< External buffer pool id */
	uint16_t                size;  /**< External buffer pool buffer size */
};

struct fman_ext_pools {
	uint8_t num_pools_used;        /**< Number of pools use by this port */
	struct fman_ext_pool_params *ext_buf_pool;
					/**< Parameters for each port */
};

struct fman_backup_bm_pools {
	uint8_t		 num_backup_pools; /**< Number of BM backup pools -
					must be smaller than the total number
					of pools defined for the specified
					port.*/
	uint8_t		*pool_ids;      /**< numOfBackupPools pool id's,
					specifying which pools should be used
					only as backup. Pool id's specified
					here must be a subset of the pools
					used by the specified port.*/
};

/**************************************************************************//**
 @Description   A structure for defining BM pool depletion criteria
*//***************************************************************************/
struct fman_buf_pool_depletion {
	bool buf_pool_depletion_enabled;
	bool pools_grp_mode_enable;    /**< select mode in which pause frames
					will be sent after a number of pools
					(all together!) are depleted */
	uint8_t num_pools;             /**< the number of depleted pools that
					will invoke pause frames transmission.
					*/
	bool *pools_to_consider;       /**< For each pool, TRUE if it should be
					considered for depletion (Note - this
					pool must be used by this port!). */
	bool single_pool_mode_enable;  /**< select mode in which pause frames
					will be sent after a single-pool
					is depleted; */
	bool *pools_to_consider_for_single_mode;
				       /**< For each pool, TRUE if it should be
					considered for depletion (Note - this
					pool must be used by this port!) */
	bool has_pfc_priorities;
	bool *pfc_priorities_en;       /**< This field is used by the MAC as
					the Priority Enable Vector in the PFC
					frame which is transmitted */
};

/**************************************************************************//**
 @Description   Enum for defining port DMA swap mode
*//***************************************************************************/
enum fman_dma_swap_option {
	FMAN_DMA_NO_SWP,           /**< No swap, transfer data as is.*/
	FMAN_DMA_SWP_PPC_LE,       /**< The transferred data should be swapped
					in PowerPc Little Endian mode. */
	FMAN_DMA_SWP_BE            /**< The transferred data should be swapped
					in Big Endian mode */
};

/**************************************************************************//**
 @Description   Enum for defining port DMA cache attributes
*//***************************************************************************/
enum fman_dma_cache_option {
	FMAN_DMA_NO_STASH = 0,     /**< Cacheable, no Allocate (No Stashing) */
	FMAN_DMA_STASH = 1         /**< Cacheable and Allocate (Stashing on) */
};

/* sizes */
#define CAPWAP_FRAG_EXTRA_SPACE                 32
#define OFFSET_UNITS                            16
#define MAX_INT_OFFSET                          240
#define MAX_IC_SIZE                             256
#define MAX_EXT_OFFSET                          496
#define MAX_EXT_BUFFER_OFFSET                   511


#endif /* __FSL_FMAN_H */
