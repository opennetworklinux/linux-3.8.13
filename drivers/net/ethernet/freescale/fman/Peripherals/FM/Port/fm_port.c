/*
 * Copyright 2008-2012 Freescale Semiconductor Inc.
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


/******************************************************************************
 @File          fm_port.c

 @Description   FM driver routines implementation.
*//***************************************************************************/
#include "error_ext.h"
#include "std_ext.h"
#include "string_ext.h"
#include "sprint_ext.h"
#include "debug_ext.h"
#include "fm_muram_ext.h"

#include "fm_port.h"


/****************************************/
/*       static functions               */
/****************************************/

static t_Error CheckInitParameters(t_FmPort *p_FmPort)
{
    t_FmPortDriverParam *p_Params = p_FmPort->p_FmPortDriverParam;
    t_Error             ans = E_OK;
    uint32_t            unusedMask;

    if (p_FmPort->imEn)
    {
        if (p_FmPort->portType == e_FM_PORT_TYPE_RX_10G)
            if (p_FmPort->fifoDeqPipelineDepth > 2)
                RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("fifoDeqPipelineDepth for IM 10G can't be larger than 2"));

        if ((ans = FmPortImCheckInitParameters(p_FmPort)) != E_OK)
            return ERROR_CODE(ans);
    }
    else
    {
        /****************************************/
        /*   Rx only                            */
        /****************************************/
        if ((p_FmPort->portType == e_FM_PORT_TYPE_RX) || (p_FmPort->portType == e_FM_PORT_TYPE_RX_10G))
        {
            /* external buffer pools */
            if (!p_Params->extBufPools.numOfPoolsUsed)
                 RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("extBufPools.numOfPoolsUsed=0. At least one buffer pool must be defined"));

            if (FmSpCheckBufPoolsParams(&p_Params->extBufPools, p_Params->p_BackupBmPools, &p_Params->bufPoolDepletion)!= E_OK)
                 RETURN_ERROR(MAJOR, E_INVALID_VALUE, NO_MSG);

            /* Check that part of IC that needs copying is small enough to enter start margin */
            if (p_Params->intContext.size && (p_Params->intContext.size + p_Params->intContext.extBufOffset > p_Params->bufMargins.startMargins))
                RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("intContext.size is larger than start margins"));

            if (p_Params->liodnOffset & ~FM_LIODN_OFFSET_MASK)
                RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("liodnOffset is larger than %d", FM_LIODN_OFFSET_MASK+1));

#ifdef FM_NO_BACKUP_POOLS
    if ((p_FmPort->fmRevInfo.majorRev != 4) && (p_FmPort->fmRevInfo.majorRev < 6))
        if (p_FmPort->p_FmPortDriverParam->p_BackupBmPools)
            RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("BackupBmPools"));
#endif /* FM_NO_BACKUP_POOLS */
        }

        /****************************************/
        /*   Non Rx ports                       */
        /****************************************/
        else
        {
            if (p_Params->deqSubPortal >= FM_MAX_NUM_OF_SUB_PORTALS)
                 RETURN_ERROR(MAJOR, E_INVALID_VALUE, (" deqSubPortal has to be in the range of 0 - %d", FM_MAX_NUM_OF_SUB_PORTALS));

            /* to protect HW internal-context from overwrite */
            if ((p_Params->intContext.size) && (p_Params->intContext.intContextOffset < MIN_TX_INT_OFFSET))
                RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("non-Rx intContext.intContextOffset can't be smaller than %d", MIN_TX_INT_OFFSET));

            if ((p_FmPort->portType == e_FM_PORT_TYPE_TX) || (p_FmPort->portType == e_FM_PORT_TYPE_TX_10G)
                    /* in O/H DEFAULT_notSupported indicates that it is not suppported and should not be checked */
                    || (p_FmPort->fifoDeqPipelineDepth != DEFAULT_notSupported))
            {
                /* Check that not larger than 8 */
                if ((!p_FmPort->fifoDeqPipelineDepth) ||( p_FmPort->fifoDeqPipelineDepth > MAX_FIFO_PIPELINE_DEPTH))
                    RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("fifoDeqPipelineDepth can't be larger than %d", MAX_FIFO_PIPELINE_DEPTH));
            }
        }

        /****************************************/
        /*   Rx Or Offline Parsing              */
        /****************************************/
        if ((p_FmPort->portType == e_FM_PORT_TYPE_RX) || (p_FmPort->portType == e_FM_PORT_TYPE_RX_10G) || (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING))
        {

            if (!p_Params->dfltFqid)
                 RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("dfltFqid must be between 1 and 2^24-1"));
#if defined(FM_CAPWAP_SUPPORT) && defined(FM_LOCKUP_ALIGNMENT_ERRATA_FMAN_SW004)
            if (p_FmPort->p_FmPortDriverParam->bufferPrefixContent.manipExtraSpace % 16)
                RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("bufferPrefixContent.manipExtraSpace has to be devidable by 16"));
#endif /* defined(FM_CAPWAP_SUPPORT) && ... */
        }

        /****************************************/
        /*   All ports                          */
        /****************************************/
        /* common BMI registers values */
        /* Check that Queue Id is not larger than 2^24, and is not 0 */
        if ((p_Params->errFqid & ~0x00FFFFFF) || !p_Params->errFqid)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("errFqid must be between 1 and 2^24-1"));
        if (p_Params->dfltFqid & ~0x00FFFFFF)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("dfltFqid must be between 1 and 2^24-1"));
    }

    /****************************************/
    /*   Rx only                            */
    /****************************************/
    if ((p_FmPort->portType == e_FM_PORT_TYPE_RX) ||
        (p_FmPort->portType == e_FM_PORT_TYPE_RX_10G))
    {
        /* Check that divisible by 256 and not larger than 256 */
        if (p_Params->rxFifoPriElevationLevel % BMI_FIFO_UNITS)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("rxFifoPriElevationLevel has to be divisible by %d", BMI_FIFO_UNITS));
        if (!p_Params->rxFifoPriElevationLevel || (p_Params->rxFifoPriElevationLevel > BMI_MAX_FIFO_SIZE))
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("rxFifoPriElevationLevel has to be in the range of 256 - %d", BMI_MAX_FIFO_SIZE));
        if (p_Params->rxFifoThreshold % BMI_FIFO_UNITS)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("rxFifoThreshold has to be divisible by %d", BMI_FIFO_UNITS));
        if (!p_Params->rxFifoThreshold ||(p_Params->rxFifoThreshold > BMI_MAX_FIFO_SIZE))
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("rxFifoThreshold has to be in the range of 256 - %d", BMI_MAX_FIFO_SIZE));

        /* Check that not larger than 16 */
        if (p_Params->cutBytesFromEnd > FRAME_END_DATA_SIZE)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("cutBytesFromEnd can't be larger than %d", FRAME_END_DATA_SIZE));

        if (FmSpCheckBufMargins(&p_Params->bufMargins)!= E_OK)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, NO_MSG);

        /* extra FIFO size (allowed only to Rx ports) */
        if (p_Params->setSizeOfFifo && (p_FmPort->fifoBufs.extra % BMI_FIFO_UNITS))
             RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("fifoBufs.extra has to be divisible by %d", BMI_FIFO_UNITS));

        if (p_Params->bufPoolDepletion.poolsGrpModeEnable &&
           !p_Params->bufPoolDepletion.numOfPools)
              RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("bufPoolDepletion.numOfPools can not be 0 when poolsGrpModeEnable=TRUE"));
#ifdef FM_CSI_CFED_LIMIT
        if (p_FmPort->fmRevInfo.majorRev == 4)
        {
            /* Check that not larger than 16 */
            if (p_Params->cutBytesFromEnd + p_Params->cheksumLastBytesIgnore > FRAME_END_DATA_SIZE)
                RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("cheksumLastBytesIgnore + cutBytesFromEnd can't be larger than %d", FRAME_END_DATA_SIZE));
        }
#endif /* FM_CSI_CFED_LIMIT */
    }

    /****************************************/
    /*   Non Rx ports                       */
    /****************************************/
    /* extra FIFO size (allowed only to Rx ports) */
    else if (p_FmPort->fifoBufs.extra)
         RETURN_ERROR(MAJOR, E_INVALID_VALUE, (" No fifoBufs.extra for non Rx ports"));

    /****************************************/
    /*   Tx only                            */
    /****************************************/
    if ((p_FmPort->portType == e_FM_PORT_TYPE_TX) ||
        (p_FmPort->portType == e_FM_PORT_TYPE_TX_10G))
    {
        /* Check that divisible by 256 and not larger than 256 */
        if (p_Params->txFifoMinFillLevel % BMI_FIFO_UNITS)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("txFifoMinFillLevel has to be divisible by %d", BMI_FIFO_UNITS));
        if (p_Params->txFifoMinFillLevel > (BMI_MAX_FIFO_SIZE - 256))
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("txFifoMinFillLevel has to be in the range of 0 - %d", BMI_MAX_FIFO_SIZE));
        if (p_Params->txFifoLowComfLevel % BMI_FIFO_UNITS)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("txFifoLowComfLevel has to be divisible by %d", BMI_FIFO_UNITS));
        if (!p_Params->txFifoLowComfLevel || (p_Params->txFifoLowComfLevel > BMI_MAX_FIFO_SIZE))
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("txFifoLowComfLevel has to be in the range of 256 - %d", BMI_MAX_FIFO_SIZE));

        if (p_FmPort->portType == e_FM_PORT_TYPE_TX)
            if (p_FmPort->fifoDeqPipelineDepth > 2)
                RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("fifoDeqPipelineDepth for 1G can't be larger than 2"));
    }

    /****************************************/
    /*   Non Tx Ports                       */
    /****************************************/
    /* If discard override was selected , no frames may be discarded. */
    else if (p_Params->frmDiscardOverride && p_Params->errorsToDiscard)
        RETURN_ERROR(MAJOR, E_CONFLICT,
                     ("errorsToDiscard is not empty, but frmDiscardOverride selected (all discarded frames to be enqueued to error queue)."));

    /****************************************/
    /*   Rx and Offline parsing             */
    /****************************************/
    if ((p_FmPort->portType == e_FM_PORT_TYPE_RX) ||
        (p_FmPort->portType == e_FM_PORT_TYPE_RX_10G) ||
        (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING))
    {
        if (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
            unusedMask = BMI_STATUS_OP_MASK_UNUSED;
        else
            unusedMask = BMI_STATUS_RX_MASK_UNUSED;

        /* Check that no common bits with BMI_STATUS_MASK_UNUSED */
        if (p_Params->errorsToDiscard & unusedMask)
            RETURN_ERROR(MAJOR, E_INVALID_SELECTION, ("errorsToDiscard contains undefined bits"));
    }

    /****************************************/
    /*   Offline Ports                      */
    /****************************************/
#ifdef FM_OP_OPEN_DMA_MIN_LIMIT
    if ((p_FmPort->fmRevInfo.majorRev >= 6) &&
        (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING) &&
        p_Params->setNumOfOpenDmas &&
        (p_FmPort->openDmas.num < MIN_NUM_OF_OP_DMAS))
         RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("For Offline port, openDmas.num can't be smaller than %d", MIN_NUM_OF_OP_DMAS));
#endif /* FM_OP_OPEN_DMA_MIN_LIMIT */

    /****************************************/
    /*   Offline & HC Ports                 */
    /****************************************/
    if ((p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING) ||
        (p_FmPort->portType == e_FM_PORT_TYPE_OH_HOST_COMMAND))
    {
#ifndef FM_FRAME_END_PARAMS_FOR_OP
        if ((p_FmPort->fmRevInfo.majorRev < 6) &&
            (p_FmPort->p_FmPortDriverParam->cheksumLastBytesIgnore != DEFAULT_notSupported))
                 /* this is an indication that user called config for this mode which is not supported in this integration */
                RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("cheksumLastBytesIgnore is available for Rx & Tx ports only"));
#endif /* !FM_FRAME_END_PARAMS_FOR_OP */

#ifndef FM_DEQ_PIPELINE_PARAMS_FOR_OP
        if ((!((p_FmPort->fmRevInfo.majorRev == 4) ||
               (p_FmPort->fmRevInfo.majorRev >= 6))) &&
            (p_FmPort->fifoDeqPipelineDepth != DEFAULT_notSupported))
                /* this is an indication that user called config for this mode which is not supported in this integration */
                RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("fifoDeqPipelineDepth is available for Tx ports only"));
#endif /* !FM_DEQ_PIPELINE_PARAMS_FOR_OP */
    }
    /****************************************/
    /*   All ports                          */
    /****************************************/

    /* Check that not larger than 16 */
    if ((p_Params->cheksumLastBytesIgnore > FRAME_END_DATA_SIZE) && ((p_Params->cheksumLastBytesIgnore != DEFAULT_notSupported)))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("cheksumLastBytesIgnore can't be larger than %d", FRAME_END_DATA_SIZE));

    if (FmSpCheckIntContextParams(&p_Params->intContext)!= E_OK)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, NO_MSG);

    /* common BMI registers values */
    if (p_Params->setNumOfTasks && ((!p_FmPort->tasks.num) || (p_FmPort->tasks.num > MAX_NUM_OF_TASKS)))
         RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("tasks.num can't be larger than %d", MAX_NUM_OF_TASKS));
    if (p_Params->setNumOfTasks && (p_FmPort->tasks.extra > MAX_NUM_OF_EXTRA_TASKS))
         RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("tasks.extra can't be larger than %d", MAX_NUM_OF_EXTRA_TASKS));
    if (p_Params->setNumOfOpenDmas && ((!p_FmPort->openDmas.num) || (p_FmPort->openDmas.num > MAX_NUM_OF_DMAS)))
         RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("openDmas.num can't be larger than %d", MAX_NUM_OF_DMAS));
    if (p_Params->setNumOfOpenDmas && (p_FmPort->openDmas.extra > MAX_NUM_OF_EXTRA_DMAS))
         RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("openDmas.extra can't be larger than %d", MAX_NUM_OF_EXTRA_DMAS));
    if (p_Params->setSizeOfFifo && (!p_FmPort->fifoBufs.num || (p_FmPort->fifoBufs.num > BMI_MAX_FIFO_SIZE)))
         RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("fifoBufs.num has to be in the range of 256 - %d", BMI_MAX_FIFO_SIZE));
    if (p_Params->setSizeOfFifo && (p_FmPort->fifoBufs.num % BMI_FIFO_UNITS))
         RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("fifoBufs.num has to be divisible by %d", BMI_FIFO_UNITS));

#ifdef FM_QMI_NO_DEQ_OPTIONS_SUPPORT
    if (p_FmPort->fmRevInfo.majorRev == 4)
        if (p_FmPort->p_FmPortDriverParam->deqPrefetchOption != DEFAULT_notSupported)
            /* this is an indication that user called config for this mode which is not supported in this integration */
            RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("deqPrefetchOption"));
#endif /* FM_QMI_NO_DEQ_OPTIONS_SUPPORT */

    return E_OK;
}

static t_Error VerifySizeOfFifo(t_FmPort *p_FmPort)
{
    uint32_t                minFifoSizeRequired = 0;

    /*************************/
    /*    TX PORTS           */
    /*************************/
    if ((p_FmPort->portType == e_FM_PORT_TYPE_TX) ||
        (p_FmPort->portType == e_FM_PORT_TYPE_TX_10G))
    {
        minFifoSizeRequired = (uint32_t)DIV_CEIL(p_FmPort->maxFrameLength, BMI_FIFO_UNITS);
        if (p_FmPort->imEn)
            minFifoSizeRequired += 3*BMI_FIFO_UNITS;
        else
            minFifoSizeRequired += (p_FmPort->fifoDeqPipelineDepth+3)*BMI_FIFO_UNITS;

        /* add some margin for back to back capability to improve performance
         * allows the hardware to pipeline new frame dma while the previous
         * frame not yet transmitted. */
        if (p_FmPort->portType == e_FM_PORT_TYPE_TX_10G)
            minFifoSizeRequired += 3*BMI_FIFO_UNITS;
        else
            minFifoSizeRequired += 2*BMI_FIFO_UNITS;
    }

    /*************************/
    /*    RX IM PORTS        */
    /*************************/
    else if (((p_FmPort->portType == e_FM_PORT_TYPE_RX) ||
              (p_FmPort->portType == e_FM_PORT_TYPE_RX_10G)) &&
             p_FmPort->imEn)
        minFifoSizeRequired = (uint32_t)(DIV_CEIL(p_FmPort->maxFrameLength, BMI_FIFO_UNITS) +
                                         (4*BMI_FIFO_UNITS));

    /*************************/
    /*    RX non-IM PORTS    */
    /*************************/
    else if (((p_FmPort->portType == e_FM_PORT_TYPE_RX) ||
              (p_FmPort->portType == e_FM_PORT_TYPE_RX_10G)) &&
             !p_FmPort->imEn)
    {
#ifdef FM_FIFO_ALLOCATION_ALG
        if (p_FmPort->fmRevInfo.majorRev == 4)
        {
            if (p_FmPort->rxPoolsParams.numOfPools == 1)
                minFifoSizeRequired = 8*BMI_FIFO_UNITS;
            else
                minFifoSizeRequired = (uint32_t)(DIV_CEIL(p_FmPort->rxPoolsParams.secondLargestBufSize, BMI_FIFO_UNITS) +
                                                 (7*BMI_FIFO_UNITS));
        }
        else
#endif /* FM_FIFO_ALLOCATION_ALG */
            minFifoSizeRequired = (uint32_t)(DIV_CEIL(p_FmPort->rxPoolsParams.largestBufSize, BMI_FIFO_UNITS) +
                                             (7*BMI_FIFO_UNITS));

        if (p_FmPort->portType == e_FM_PORT_TYPE_RX_10G)
            minFifoSizeRequired += 12*BMI_FIFO_UNITS;
        else
            minFifoSizeRequired += 3*BMI_FIFO_UNITS;
    }

    /* For O/H ports, check fifo size and update if necessary */
    else if ((p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING) ||
             (p_FmPort->portType == e_FM_PORT_TYPE_OH_HOST_COMMAND))
        minFifoSizeRequired = (uint32_t)((p_FmPort->tasks.num + 4)*BMI_FIFO_UNITS);

    /* for all ports - verify size  */
    if (minFifoSizeRequired && (p_FmPort->fifoBufs.num < minFifoSizeRequired))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE,
                     ("User defined FIFO size should be enlarged to %d",
                      minFifoSizeRequired));

    /* check if pool size is not too big */
    /* This is a definition problem in which if the fifo for the RX port
       is lower than the largest pool size the hardware will allocate scatter gather
       buffers even though the frame size can fit in a single buffer. */
    if (((p_FmPort->portType == e_FM_PORT_TYPE_RX) ||
         (p_FmPort->portType == e_FM_PORT_TYPE_RX_10G))
        && !p_FmPort->imEn)
    {
        if (p_FmPort->rxPoolsParams.largestBufSize > p_FmPort->fifoBufs.num)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE,
                         ("Frame larger than port Fifo size (%u) will be split to more "\
                          "than a single buffer (S/G) even if shorter than largest buffer size (%u)",
                          p_FmPort->fifoBufs.num, p_FmPort->rxPoolsParams.largestBufSize));
    }

    return E_OK;
}

static void FmPortDriverParamFree(t_FmPort *p_FmPort)
{
    if (p_FmPort->p_FmPortDriverParam)
    {
        XX_Free(p_FmPort->p_FmPortDriverParam);
        p_FmPort->p_FmPortDriverParam = NULL;
    }
}

static t_Error SetExtBufferPools(t_FmPort *p_FmPort)
{
    t_FmExtPools                *p_ExtBufPools = &p_FmPort->p_FmPortDriverParam->extBufPools;
    t_FmBufPoolDepletion        *p_BufPoolDepletion = &p_FmPort->p_FmPortDriverParam->bufPoolDepletion;
    volatile uint32_t           *p_ExtBufRegs;
    volatile uint32_t           *p_BufPoolDepletionReg;
    bool                        rxPort;
    uint8_t                     orderedArray[FM_PORT_MAX_NUM_OF_EXT_POOLS];
    uint16_t                    sizesArray[BM_MAX_NUM_OF_POOLS];
    uint8_t                     numOfPools;
    int                         i=0, j=0;
    uint32_t                    tmpReg, vector;

    memset(&orderedArray, 0, sizeof(uint8_t) * FM_PORT_MAX_NUM_OF_EXT_POOLS);
    memset(&sizesArray, 0, sizeof(uint16_t) * BM_MAX_NUM_OF_POOLS);
    memcpy(&p_FmPort->extBufPools, p_ExtBufPools, sizeof(t_FmExtPools));

    switch (p_FmPort->portType)
    {
        case (e_FM_PORT_TYPE_RX_10G):
        case (e_FM_PORT_TYPE_RX):
            p_ExtBufRegs = p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_ebmpi;
            p_BufPoolDepletionReg = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rmpd;
            rxPort = TRUE;
            break;
        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
            p_ExtBufRegs = p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_oebmpi;
            p_BufPoolDepletionReg = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ompd;
            rxPort = FALSE;
            break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("Not available for port type"));
    }

    FmSpSetBufPoolsInAscOrderOfBufSizes(p_ExtBufPools, orderedArray, sizesArray);

    /* build the register value */
    for (i=0;i<p_ExtBufPools->numOfPoolsUsed;i++)
    {
        tmpReg = BMI_EXT_BUF_POOL_VALID | BMI_EXT_BUF_POOL_EN_COUNTER;
        tmpReg |= ((uint32_t)orderedArray[i] << BMI_EXT_BUF_POOL_ID_SHIFT);
        tmpReg |= sizesArray[orderedArray[i]];
        /* functionality available only for some deriviatives (limited by config) */
        if (p_FmPort->p_FmPortDriverParam->p_BackupBmPools)
            for (j=0;j<p_FmPort->p_FmPortDriverParam->p_BackupBmPools->numOfBackupPools;j++)
                if (orderedArray[i] == p_FmPort->p_FmPortDriverParam->p_BackupBmPools->poolIds[j])
                {
                    tmpReg |= BMI_EXT_BUF_POOL_BACKUP;
                    break;
                }
        WRITE_UINT32(*(p_ExtBufRegs+i), tmpReg);
    }

    if (p_FmPort->p_FmPortDriverParam->p_BackupBmPools)
        XX_Free(p_FmPort->p_FmPortDriverParam->p_BackupBmPools);

    numOfPools = (uint8_t)(rxPort ? FM_PORT_MAX_NUM_OF_EXT_POOLS:FM_PORT_MAX_NUM_OF_OBSERVED_EXT_POOLS);

    /* clear unused pools */
    for (i=p_ExtBufPools->numOfPoolsUsed;i<numOfPools;i++)
        WRITE_UINT32(*(p_ExtBufRegs+i), 0);

    /* save pools parameters for later use */
    p_FmPort->rxPoolsParams.numOfPools = p_ExtBufPools->numOfPoolsUsed;
    p_FmPort->rxPoolsParams.largestBufSize = sizesArray[orderedArray[p_ExtBufPools->numOfPoolsUsed-1]];
    p_FmPort->rxPoolsParams.secondLargestBufSize = sizesArray[orderedArray[p_ExtBufPools->numOfPoolsUsed-2]];

    /* FMBM_RMPD reg. - pool depletion */
    tmpReg = 0;
    if (p_BufPoolDepletion->poolsGrpModeEnable)
    {
        /* calculate vector for number of pools depletion */
        vector = 0;
        for (i=0;i<BM_MAX_NUM_OF_POOLS;i++)
        {
            if (p_BufPoolDepletion->poolsToConsider[i])
            {
                for (j=0;j<p_ExtBufPools->numOfPoolsUsed;j++)
                {
                    if (i == orderedArray[j])
                    {
                        vector |= 0x80000000 >> j;
                        break;
                    }
                }
            }
        }
        /* configure num of pools and vector for number of pools mode */
        tmpReg |= (((uint32_t)p_BufPoolDepletion->numOfPools - 1) << BMI_POOL_DEP_NUM_OF_POOLS_SHIFT);
        tmpReg |= vector;
    }

    if (p_BufPoolDepletion->singlePoolModeEnable)
    {
        /* calculate vector for number of pools depletion */
        vector = 0;
        for (i=0;i<BM_MAX_NUM_OF_POOLS;i++)
        {
            if (p_BufPoolDepletion->poolsToConsiderForSingleMode[i])
            {
                for (j=0;j<p_ExtBufPools->numOfPoolsUsed;j++)
                {
                    if (i == orderedArray[j])
                     {
                        vector |= 0x00000080 >> j;
                        break;
                    }
                }
            }
        }
        tmpReg |= vector;
    }

#if (DPAA_VERSION >= 11)
    /* fill QbbPEV */
    if (p_BufPoolDepletion->poolsGrpModeEnable ||
        p_BufPoolDepletion->singlePoolModeEnable)
    {
        vector = 0;
        for (i=0; i<FM_MAX_NUM_OF_PFC_PRIORITIES; i++)
        {
            if (p_BufPoolDepletion->pfcPrioritiesEn[i] == TRUE)
            {
                vector |= 0x00008000 >> i;
            }
        }
        tmpReg |= vector;
    }
#endif /* (DPAA_VERSION >= 11) */

    WRITE_UINT32(*p_BufPoolDepletionReg, tmpReg);

    return E_OK;
}

static t_Error ClearPerfCnts(t_FmPort *p_FmPort)
{
    if (p_FmPort->portType != e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
        FM_PORT_ModifyCounter(p_FmPort, e_FM_PORT_COUNTERS_QUEUE_UTIL, 0);
    FM_PORT_ModifyCounter(p_FmPort, e_FM_PORT_COUNTERS_TASK_UTIL, 0);
    FM_PORT_ModifyCounter(p_FmPort, e_FM_PORT_COUNTERS_DMA_UTIL, 0);
    FM_PORT_ModifyCounter(p_FmPort, e_FM_PORT_COUNTERS_FIFO_UTIL, 0);
    return E_OK;
}

static t_Error BmiRxPortInit(t_FmPort *p_FmPort)
{
    t_FmPortRxBmiRegs       *p_Regs = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs;
    uint32_t                tmpReg;
    t_FmPortDriverParam     *p_Params = p_FmPort->p_FmPortDriverParam;
    uint32_t                errorsToEnq = 0;
    t_FmPortPerformanceCnt  performanceContersParams;
    t_Error                 err;

    /* check that port is not busy */
    if (GET_UINT32(p_Regs->fmbm_rcfg) & BMI_PORT_CFG_EN)
         RETURN_ERROR(MAJOR, E_INVALID_STATE,
                      ("Port(%d,%d) is already enabled",p_FmPort->portType, p_FmPort->portId));

    /* Set Config register */
    tmpReg = 0;
    if (p_FmPort->imEn)
        tmpReg |= BMI_PORT_CFG_IM;
    /* No discard - all error frames go to error queue */
    else if (p_Params->frmDiscardOverride)
        tmpReg |= BMI_PORT_CFG_FDOVR;

    WRITE_UINT32(p_Regs->fmbm_rcfg, tmpReg);

    /* Configure dma attributes */
    tmpReg = 0;
    tmpReg |= (uint32_t)p_Params->dmaSwapData << BMI_DMA_ATTR_SWP_SHIFT;
    tmpReg |= (uint32_t)p_Params->dmaIntContextCacheAttr << BMI_DMA_ATTR_IC_CACHE_SHIFT;
    tmpReg |= (uint32_t)p_Params->dmaHeaderCacheAttr << BMI_DMA_ATTR_HDR_CACHE_SHIFT;
    tmpReg |= (uint32_t)p_Params->dmaScatterGatherCacheAttr << BMI_DMA_ATTR_SG_CACHE_SHIFT;
    if (p_Params->dmaWriteOptimize)
        tmpReg |= BMI_DMA_ATTR_WRITE_OPTIMIZE;

    WRITE_UINT32(p_Regs->fmbm_rda, tmpReg);

    /* Configure Rx Fifo params */
    tmpReg = 0;
    tmpReg |= ((p_Params->rxFifoPriElevationLevel/BMI_FIFO_UNITS - 1) << BMI_RX_FIFO_PRI_ELEVATION_SHIFT);
    tmpReg |= ((p_Params->rxFifoThreshold/BMI_FIFO_UNITS - 1) << BMI_RX_FIFO_THRESHOLD_SHIFT);

    WRITE_UINT32(p_Regs->fmbm_rfp, tmpReg);

#ifdef FM_NO_RESTRICT_ON_ACCESS_RSRC
    if (p_FmPort->fmRevInfo.majorRev < 6)
        /* always allow access to the extra resources */
        WRITE_UINT32(p_Regs->fmbm_reth, BMI_RX_FIFO_THRESHOLD_BC);
#endif /* FM_NO_RESTRICT_ON_ACCESS_RSRC */

     /* frame end parameters */
    tmpReg = 0;
    tmpReg |= ((uint32_t)p_Params->cheksumLastBytesIgnore << BMI_RX_FRAME_END_CS_IGNORE_SHIFT);
    tmpReg |= ((uint32_t)p_Params->cutBytesFromEnd<< BMI_RX_FRAME_END_CUT_SHIFT);
#ifdef FM_RX_FIFO_CORRUPT_ERRATA_10GMAC_A006320
    /* zero cutBytesFromEnd field which means that bmi doesn't
       remove further bytes because the MAC already remove the CRC.
       the workaround is relevant only in initial rev of FMan v3.
     */
    if ((p_FmPort->fmRevInfo.majorRev == 6) && (p_FmPort->fmRevInfo.minorRev == 0))
        tmpReg &= 0xffe0ffff;
#endif /* FM_RX_FIFO_CORRUPT_ERRATA_10GMAC_A006320 */
    WRITE_UINT32(p_Regs->fmbm_rfed, tmpReg);

    /* IC parameters */
    tmpReg = 0;
    tmpReg |= (((uint32_t)p_Params->intContext.extBufOffset/OFFSET_UNITS) << BMI_IC_TO_EXT_SHIFT);
    tmpReg |= (((uint32_t)p_Params->intContext.intContextOffset/OFFSET_UNITS) << BMI_IC_FROM_INT_SHIFT);
    tmpReg |= (((uint32_t)p_Params->intContext.size/OFFSET_UNITS)  << BMI_IC_SIZE_SHIFT);

    WRITE_UINT32(p_Regs->fmbm_ricp, tmpReg);

    if (!p_FmPort->imEn)
    {
        /* Call the external Buffer routine which also checks fifo
           size and updates it if necessary */
        /* define external buffer pools and pool depletion*/

        /* check if the largest external buffer pool is large enough */
        if ((p_Params->bufMargins.startMargins +
             MIN_EXT_BUF_SIZE +
             p_Params->bufMargins.endMargins) > p_FmPort->rxPoolsParams.largestBufSize)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE,
                         ("bufMargins.startMargins (%d) + minimum buf size (64) + "
                          "bufMargins.endMargins (%d) is larger than maximum external buffer size (%d)",
                            p_Params->bufMargins.startMargins,
                            p_Params->bufMargins.endMargins,
                            p_FmPort->rxPoolsParams.largestBufSize));

        /* buffer margins */
        tmpReg = 0;
        tmpReg |= (((uint32_t)p_Params->bufMargins.startMargins) << BMI_EXT_BUF_MARG_START_SHIFT);
        tmpReg |= (((uint32_t)p_Params->bufMargins.endMargins) << BMI_EXT_BUF_MARG_END_SHIFT);
#if (DPAA_VERSION >= 11)
        if (p_Params->noScatherGather)
            tmpReg |= BMI_SG_DISABLE;
#endif
        WRITE_UINT32(p_Regs->fmbm_rebm, tmpReg);
    }

    if (p_FmPort->internalBufferOffset)
    {
        tmpReg = (uint32_t)((p_FmPort->internalBufferOffset%OFFSET_UNITS) ?
                            (p_FmPort->internalBufferOffset/OFFSET_UNITS + 1) :
                            (p_FmPort->internalBufferOffset/OFFSET_UNITS));
        p_FmPort->internalBufferOffset = (uint8_t)(tmpReg * OFFSET_UNITS);
        WRITE_UINT32(p_Regs->fmbm_rim, tmpReg << BMI_IM_FOF_SHIFT);
    }

    /* NIA */
    if (p_FmPort->imEn)
        WRITE_UINT32(p_Regs->fmbm_rfne, NIA_ENG_FM_CTL | NIA_FM_CTL_AC_IND_MODE_RX);
    else
    {
        tmpReg = 0;
        if (p_Params->forwardReuseIntContext)
            tmpReg |= BMI_PORT_RFNE_FRWD_RPD;
        /* L3/L4 checksum verify is enabled by default. */
        /*tmpReg |= BMI_PORT_RFNE_FRWD_DCL4C;*/
        WRITE_UINT32(p_Regs->fmbm_rfne, tmpReg | GET_NO_PCD_NIA_BMI_AC_ENQ_FRAME());
    }
    WRITE_UINT32(p_Regs->fmbm_rfene, NIA_ENG_QMI_ENQ | NIA_ORDER_RESTOR);

    /* command attribute */
    tmpReg = BMI_CMD_RX_MR_DEF;
    if (!p_FmPort->imEn)
    {
        tmpReg |= BMI_CMD_ATTR_ORDER;
        if (p_Params->syncReq)
            tmpReg |= BMI_CMD_ATTR_SYNC;
        tmpReg |= ((uint32_t)p_Params->color << BMI_CMD_ATTR_COLOR_SHIFT);
    }

    WRITE_UINT32(p_Regs->fmbm_rfca, tmpReg);

    /* default queues */
    if (!p_FmPort->imEn)
    {
        WRITE_UINT32(p_Regs->fmbm_rfqid, p_Params->dfltFqid);
        WRITE_UINT32(p_Regs->fmbm_refqid, p_Params->errFqid);
    }

    /* set counters */
    WRITE_UINT32(p_Regs->fmbm_rstc, BMI_COUNTERS_EN);

    performanceContersParams.taskCompVal    = (uint8_t)p_FmPort->tasks.num;
    performanceContersParams.queueCompVal   = 1;
    performanceContersParams.dmaCompVal     =(uint8_t) p_FmPort->openDmas.num;
    performanceContersParams.fifoCompVal    = p_FmPort->fifoBufs.num;
    if ((err = FM_PORT_SetPerformanceCountersParams(p_FmPort, &performanceContersParams)) != E_OK)
        RETURN_ERROR(MAJOR, err, NO_MSG);

    WRITE_UINT32(p_Regs->fmbm_rpc, BMI_COUNTERS_EN);

    /* error/status mask  - check that if discard OV is set, no
       discard is required for specific errors.*/
    WRITE_UINT32(p_Regs->fmbm_rfsdm, p_Params->errorsToDiscard);

    errorsToEnq = (RX_ERRS_TO_ENQ & ~p_Params->errorsToDiscard);
    WRITE_UINT32(p_Regs->fmbm_rfsem, errorsToEnq);

    return E_OK;
}

static t_Error BmiTxPortInit(t_FmPort *p_FmPort)
{
    t_FmPortTxBmiRegs   *p_Regs = &p_FmPort->p_FmPortBmiRegs->txPortBmiRegs;
    uint32_t            tmpReg;
    t_FmPortDriverParam *p_Params = p_FmPort->p_FmPortDriverParam;
    /*uint32_t            rateCountUnit;*/
    t_FmPortPerformanceCnt  performanceContersParams;

    /* check that port is not busy */
    if (GET_UINT32(p_Regs->fmbm_tcfg) & BMI_PORT_CFG_EN)
         RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Port is already enabled"));

    tmpReg = 0;
    if (p_FmPort->imEn)
        tmpReg |= BMI_PORT_CFG_IM;

    WRITE_UINT32(p_Regs->fmbm_tcfg, tmpReg);

    /* Configure dma attributes */
    tmpReg = 0;
    tmpReg |= (uint32_t)p_Params->dmaSwapData << BMI_DMA_ATTR_SWP_SHIFT;
    tmpReg |= (uint32_t)p_Params->dmaIntContextCacheAttr << BMI_DMA_ATTR_IC_CACHE_SHIFT;
    tmpReg |= (uint32_t)p_Params->dmaHeaderCacheAttr << BMI_DMA_ATTR_HDR_CACHE_SHIFT;
    tmpReg |= (uint32_t)p_Params->dmaScatterGatherCacheAttr << BMI_DMA_ATTR_SG_CACHE_SHIFT;

    WRITE_UINT32(p_Regs->fmbm_tda, tmpReg);

    /* Configure Tx Fifo params */
    tmpReg = 0;
    tmpReg |= ((p_Params->txFifoMinFillLevel/BMI_FIFO_UNITS) << BMI_TX_FIFO_MIN_FILL_SHIFT);
    tmpReg |= (((uint32_t)p_FmPort->fifoDeqPipelineDepth - 1) << BMI_FIFO_PIPELINE_DEPTH_SHIFT);
    tmpReg |= ((p_Params->txFifoLowComfLevel/BMI_FIFO_UNITS - 1) << BMI_TX_LOW_COMF_SHIFT);

    WRITE_UINT32(p_Regs->fmbm_tfp, tmpReg);

    /* frame end parameters */
    tmpReg = 0;
    tmpReg |= ((uint32_t)p_Params->cheksumLastBytesIgnore << BMI_FRAME_END_CS_IGNORE_SHIFT);

    WRITE_UINT32(p_Regs->fmbm_tfed, tmpReg);

    if (!p_FmPort->imEn)
    {
        /* IC parameters */
        tmpReg = 0;
        tmpReg |= (((uint32_t)p_Params->intContext.extBufOffset/OFFSET_UNITS) << BMI_IC_TO_EXT_SHIFT);
        tmpReg |= (((uint32_t)p_Params->intContext.intContextOffset/OFFSET_UNITS) << BMI_IC_FROM_INT_SHIFT);
        tmpReg |= (((uint32_t)p_Params->intContext.size/OFFSET_UNITS)  << BMI_IC_SIZE_SHIFT);

        WRITE_UINT32(p_Regs->fmbm_ticp, tmpReg);
    }

    /* NIA */
    if (p_FmPort->imEn)
    {
        WRITE_UINT32(p_Regs->fmbm_tfdne, NIA_ENG_FM_CTL | NIA_FM_CTL_AC_IND_MODE_TX);
        WRITE_UINT32(p_Regs->fmbm_tfene, NIA_ENG_FM_CTL | NIA_FM_CTL_AC_IND_MODE_TX);
#if (DPAA_VERSION >= 11)
/* TODO - what should be the TFNE for FMan v3 IM? */
#endif /* (DPAA_VERSION >= 11) */
    }
    else
    {
        WRITE_UINT32(p_Regs->fmbm_tfdne, NIA_ENG_QMI_DEQ);
        WRITE_UINT32(p_Regs->fmbm_tfene, NIA_ENG_QMI_ENQ | NIA_ORDER_RESTOR);
#if (DPAA_VERSION >= 11)
        WRITE_UINT32(p_Regs->fmbm_tfne,
                     (!p_Params->dfltFqid ?
                        BMI_EBD_EN | NIA_BMI_AC_FETCH_ALL_FRAME :
                        NIA_BMI_AC_FETCH_ALL_FRAME));
#endif /* (DPAA_VERSION >= 11) */

        /* The line bellow is a trick so the FM will not release the buffer
           to BM nor will try to enq the frame to QM */
        if (!p_Params->dfltFqid && p_Params->dontReleaseBuf)
        {
            /* override fmbm_tcfqid 0 with a false non-0 value. This will force FM to
             * act according to tfene. Otherwise, if fmbm_tcfqid is 0 the FM will release
             * buffers to BM regardless of fmbm_tfene
             */
            WRITE_UINT32(p_Regs->fmbm_tcfqid, 0xFFFFFF);
            WRITE_UINT32(p_Regs->fmbm_tfene, NIA_ENG_BMI | NIA_BMI_AC_TX_RELEASE);
#if (DPAA_VERSION >= 11)
            WRITE_UINT32(p_Regs->fmbm_tfne,
                         (GET_UINT32(p_Regs->fmbm_tfne) & ~BMI_EBD_EN));
#endif /* (DPAA_VERSION >= 11) */
        }
    }

    /* command attribute */
    tmpReg = BMI_CMD_TX_MR_DEF;
    if (p_FmPort->imEn)
        tmpReg |= BMI_CMD_MR_DEAS;
    else
    {
        tmpReg |= BMI_CMD_ATTR_ORDER;
        tmpReg |= ((uint32_t)p_Params->color << BMI_CMD_ATTR_COLOR_SHIFT);
    }
    WRITE_UINT32(p_Regs->fmbm_tfca, tmpReg);

    /* default queues */
    if (!p_FmPort->imEn)
    {
        if (p_Params->dfltFqid || !p_Params->dontReleaseBuf)
            WRITE_UINT32(p_Regs->fmbm_tcfqid, p_Params->dfltFqid);
        WRITE_UINT32(p_Regs->fmbm_tfeqid, p_Params->errFqid);
    }

    /* statistics & performance counters */
    WRITE_UINT32(p_Regs->fmbm_tstc, BMI_COUNTERS_EN);

    performanceContersParams.taskCompVal    = (uint8_t)p_FmPort->tasks.num;
    performanceContersParams.queueCompVal   = 1;
    performanceContersParams.dmaCompVal     = (uint8_t)p_FmPort->openDmas.num;
    performanceContersParams.fifoCompVal    = p_FmPort->fifoBufs.num;
    FM_PORT_SetPerformanceCountersParams(p_FmPort, &performanceContersParams);

    WRITE_UINT32(p_Regs->fmbm_tpc, BMI_COUNTERS_EN);


    return E_OK;
}

static t_Error BmiOhPortInit(t_FmPort *p_FmPort)
{
    t_FmPortOhBmiRegs       *p_Regs = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs;
    uint32_t                tmpReg, errorsToEnq = 0;
    t_FmPortDriverParam     *p_Params = p_FmPort->p_FmPortDriverParam;
    t_FmPortPerformanceCnt  performanceContersParams;

    /* check that port is not busy */
    if (GET_UINT32(p_Regs->fmbm_ocfg) & BMI_PORT_CFG_EN)
         RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Port is already enabled"));

    /* Configure dma attributes */
    tmpReg = 0;
    tmpReg |= (uint32_t)p_Params->dmaSwapData << BMI_DMA_ATTR_SWP_SHIFT;
    tmpReg |= (uint32_t)p_Params->dmaIntContextCacheAttr << BMI_DMA_ATTR_IC_CACHE_SHIFT;
    tmpReg |= (uint32_t)p_Params->dmaHeaderCacheAttr << BMI_DMA_ATTR_HDR_CACHE_SHIFT;
    tmpReg |= (uint32_t)p_Params->dmaScatterGatherCacheAttr << BMI_DMA_ATTR_SG_CACHE_SHIFT;
    if (p_Params->dmaWriteOptimize)
        tmpReg |= BMI_DMA_ATTR_WRITE_OPTIMIZE;

    WRITE_UINT32(p_Regs->fmbm_oda, tmpReg);

    /* IC parameters */
    tmpReg = 0;
    tmpReg |= (((uint32_t)p_Params->intContext.extBufOffset/OFFSET_UNITS) << BMI_IC_TO_EXT_SHIFT);
    tmpReg |= (((uint32_t)p_Params->intContext.intContextOffset/OFFSET_UNITS) << BMI_IC_FROM_INT_SHIFT);
    tmpReg |= (((uint32_t)p_Params->intContext.size/OFFSET_UNITS)  << BMI_IC_SIZE_SHIFT);

    WRITE_UINT32(p_Regs->fmbm_oicp, tmpReg);

    /* NIA */
    WRITE_UINT32(p_Regs->fmbm_ofdne, NIA_ENG_QMI_DEQ);

    if (p_FmPort->portType==e_FM_PORT_TYPE_OH_HOST_COMMAND)
        WRITE_UINT32(p_Regs->fmbm_ofene, NIA_ENG_QMI_ENQ);
    else
        WRITE_UINT32(p_Regs->fmbm_ofene, NIA_ENG_QMI_ENQ | NIA_ORDER_RESTOR);

    /* command attribute */
    if (p_FmPort->portType==e_FM_PORT_TYPE_OH_HOST_COMMAND)
        tmpReg =  BMI_CMD_MR_DEAS | BMI_CMD_MR_MA;
    else
        tmpReg = BMI_CMD_ATTR_ORDER | BMI_CMD_MR_DEAS | BMI_CMD_MR_MA;

    if (p_Params->syncReq)
        tmpReg |= BMI_CMD_ATTR_SYNC;
    tmpReg |= ((uint32_t)p_Params->color << BMI_CMD_ATTR_COLOR_SHIFT);
    WRITE_UINT32(p_Regs->fmbm_ofca, tmpReg);

    /* No discard - all error frames go to error queue */
    if (p_Params->frmDiscardOverride)
        tmpReg = BMI_PORT_CFG_FDOVR;
    else
        tmpReg = 0;
    WRITE_UINT32(p_Regs->fmbm_ocfg, tmpReg);

    if (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
    {
        WRITE_UINT32(p_Regs->fmbm_ofsdm, p_Params->errorsToDiscard);

        errorsToEnq = (OP_ERRS_TO_ENQ & ~p_Params->errorsToDiscard);
        WRITE_UINT32(p_Regs->fmbm_ofsem, errorsToEnq);

        /* NIA */
        WRITE_UINT32(p_Regs->fmbm_ofne, GET_NO_PCD_NIA_BMI_AC_ENQ_FRAME());

#ifndef FM_NO_OP_OBSERVED_POOLS
        /* Call the external Buffer routine which also checks fifo
           size and updates it if necessary */
        if ((p_FmPort->fmRevInfo.majorRev == 4) &&
            p_Params->enBufPoolDepletion)
        {
            /* define external buffer pools */
            t_Error err = SetExtBufferPools(p_FmPort);
            if (err)
                RETURN_ERROR(MAJOR, err, NO_MSG);
        }
#endif /* FM_NO_OP_OBSERVED_POOLS */
    }
    else
        /* NIA */
        WRITE_UINT32(p_Regs->fmbm_ofne, NIA_ENG_FM_CTL | NIA_FM_CTL_AC_HC);

    /* default queues */
    WRITE_UINT32(p_Regs->fmbm_ofqid, p_Params->dfltFqid);
    WRITE_UINT32(p_Regs->fmbm_oefqid, p_Params->errFqid);

    if (p_FmPort->internalBufferOffset)
    {
        tmpReg = (uint32_t)((p_FmPort->internalBufferOffset % OFFSET_UNITS) ?
                            (p_FmPort->internalBufferOffset/OFFSET_UNITS + 1):
                            (p_FmPort->internalBufferOffset/OFFSET_UNITS));
        p_FmPort->internalBufferOffset = (uint8_t)(tmpReg * OFFSET_UNITS);
        WRITE_UINT32(p_Regs->fmbm_oim, tmpReg << BMI_IM_FOF_SHIFT);
    }
    /* statistics & performance counters */
    WRITE_UINT32(p_Regs->fmbm_ostc, BMI_COUNTERS_EN);

    performanceContersParams.taskCompVal    = (uint8_t)p_FmPort->tasks.num;
    performanceContersParams.queueCompVal   = 0;
    performanceContersParams.dmaCompVal     = (uint8_t)p_FmPort->openDmas.num;
    performanceContersParams.fifoCompVal    = p_FmPort->fifoBufs.num;
    FM_PORT_SetPerformanceCountersParams(p_FmPort, &performanceContersParams);

    WRITE_UINT32(p_Regs->fmbm_opc, BMI_COUNTERS_EN);
#ifdef FM_DEQ_PIPELINE_PARAMS_FOR_OP
    if ((p_FmPort->fmRevInfo.majorRev == 4) ||
        (p_FmPort->fmRevInfo.majorRev >= 6))
    {
        tmpReg = (((uint32_t)p_FmPort->fifoDeqPipelineDepth - 1) << BMI_FIFO_PIPELINE_DEPTH_SHIFT);
        WRITE_UINT32(p_Regs->fmbm_ofp, tmpReg);
    }
#endif /* FM_DEQ_PIPELINE_PARAMS_FOR_OP */

#ifdef FM_FRAME_END_PARAMS_FOR_OP
    if (p_FmPort->fmRevInfo.majorRev >= 6)
    {
        /* frame end parameters */
        tmpReg = 0;
        tmpReg |= ((uint32_t)p_Params->cheksumLastBytesIgnore << BMI_FRAME_END_CS_IGNORE_SHIFT);

        WRITE_UINT32(p_Regs->fmbm_ofed, tmpReg);
    }
#endif /* FM_FRAME_END_PARAMS_FOR_OP */

    return E_OK;
}

static t_Error QmiInit(t_FmPort *p_FmPort)
{
    t_FmPortDriverParam             *p_Params = NULL;
    uint32_t                        tmpReg;

    p_Params = p_FmPort->p_FmPortDriverParam;

    /* check that port is not busy */
    if (((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_RX)) &&
       (GET_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnc) & QMI_PORT_CFG_EN))
         RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Port is already enabled"));

    /* enable & clear counters */
    WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnc, QMI_PORT_CFG_EN_COUNTERS);

    /* The following is  done for non-Rx ports only */
    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_RX))
    {
        if ((p_FmPort->portType == e_FM_PORT_TYPE_TX_10G) ||
            (p_FmPort->portType == e_FM_PORT_TYPE_TX))
        {
            /* define dequeue NIA */
            WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs.fmqm_pndn, NIA_ENG_BMI | NIA_BMI_AC_TX);
            /* define enqueue NIA */
            WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnen, NIA_ENG_BMI | NIA_BMI_AC_TX_RELEASE);
        }
        else  /* for HC & OP */
        {
            WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs.fmqm_pndn, NIA_ENG_BMI | NIA_BMI_AC_FETCH);
            /* define enqueue NIA */
            WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnen, NIA_ENG_BMI | NIA_BMI_AC_RELEASE);
        }

        /* configure dequeue */
        tmpReg = 0;
        if (p_Params->deqHighPriority)
            tmpReg |= QMI_DEQ_CFG_PRI;

        switch (p_Params->deqType)
        {
            case (e_FM_PORT_DEQ_TYPE1):
                tmpReg |= QMI_DEQ_CFG_TYPE1;
                break;
            case (e_FM_PORT_DEQ_TYPE2):
                tmpReg |= QMI_DEQ_CFG_TYPE2;
                break;
            case (e_FM_PORT_DEQ_TYPE3):
                tmpReg |= QMI_DEQ_CFG_TYPE3;
                break;
            default:
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid dequeue type"));
        }

#ifdef FM_QMI_NO_DEQ_OPTIONS_SUPPORT
        if (p_FmPort->fmRevInfo.majorRev != 4)
#endif /* FM_QMI_NO_DEQ_OPTIONS_SUPPORT */
        switch (p_Params->deqPrefetchOption)
        {
            case (e_FM_PORT_DEQ_NO_PREFETCH):
                /* Do nothing - QMI_DEQ_CFG_PREFETCH_WAITING_TNUM | QMI_DEQ_CFG_PREFETCH_1_FRAME = 0 */
                break;
            case (e_FM_PORT_DEQ_PARTIAL_PREFETCH):
                tmpReg |= QMI_DEQ_CFG_PREFETCH_WAITING_TNUM | QMI_DEQ_CFG_PREFETCH_3_FRAMES;
                break;
            case (e_FM_PORT_DEQ_FULL_PREFETCH):
                tmpReg |= QMI_DEQ_CFG_PREFETCH_NO_TNUM | QMI_DEQ_CFG_PREFETCH_3_FRAMES;
                break;
            default:
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid dequeue prefetch option"));
        }
#ifdef FM_QMI_NO_DEQ_OPTIONS_SUPPORT
        if (p_FmPort->fmRevInfo.majorRev != 4)
#endif /* FM_QMI_NO_DEQ_OPTIONS_SUPPORT */
        if (p_Params->deqPrefetchOption == e_FM_PORT_DEQ_NO_PREFETCH)
            FmSetPortPreFetchConfiguration(p_FmPort->h_Fm, p_FmPort->portId, FALSE);
        else
            FmSetPortPreFetchConfiguration(p_FmPort->h_Fm, p_FmPort->portId, TRUE);

        tmpReg |= p_Params->deqByteCnt;
        tmpReg |= (uint32_t)p_Params->deqSubPortal << QMI_DEQ_CFG_SUBPORTAL_SHIFT;

        WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs.fmqm_pndc, tmpReg);
    }
    else /* rx port */
        /* define enqueue NIA */
        WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnen, NIA_ENG_BMI | NIA_BMI_AC_RELEASE);

    return E_OK;
}

static t_Error BmiRxPortCheckAndGetCounterPtr(t_FmPort *p_FmPort, e_FmPortCounters counter, volatile uint32_t **p_Ptr)
{
    t_FmPortRxBmiRegs   *p_BmiRegs = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs;

     /* check that counters are enabled */
    switch (counter)
    {
        case (e_FM_PORT_COUNTERS_CYCLE):
        case (e_FM_PORT_COUNTERS_TASK_UTIL):
        case (e_FM_PORT_COUNTERS_QUEUE_UTIL):
        case (e_FM_PORT_COUNTERS_DMA_UTIL):
        case (e_FM_PORT_COUNTERS_FIFO_UTIL):
        case (e_FM_PORT_COUNTERS_RX_PAUSE_ACTIVATION):
            /* performance counters - may be read when disabled */
            break;
        case (e_FM_PORT_COUNTERS_FRAME):
        case (e_FM_PORT_COUNTERS_DISCARD_FRAME):
        case (e_FM_PORT_COUNTERS_RX_BAD_FRAME):
        case (e_FM_PORT_COUNTERS_RX_LARGE_FRAME):
        case (e_FM_PORT_COUNTERS_RX_FILTER_FRAME):
        case (e_FM_PORT_COUNTERS_RX_LIST_DMA_ERR):
        case (e_FM_PORT_COUNTERS_RX_OUT_OF_BUFFERS_DISCARD):
        case (e_FM_PORT_COUNTERS_DEALLOC_BUF):
        case (e_FM_PORT_COUNTERS_PREPARE_TO_ENQUEUE_COUNTER):
            if (!(GET_UINT32(p_BmiRegs->fmbm_rstc) & BMI_COUNTERS_EN))
               RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter was not enabled"));
            break;
         default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter is not available for Rx ports"));
    }

    /* Set counter */
    switch (counter)
    {
        case (e_FM_PORT_COUNTERS_CYCLE):
            *p_Ptr = &p_BmiRegs->fmbm_rccn;
            break;
        case (e_FM_PORT_COUNTERS_TASK_UTIL):
            *p_Ptr = &p_BmiRegs->fmbm_rtuc;
            break;
        case (e_FM_PORT_COUNTERS_QUEUE_UTIL):
            *p_Ptr = &p_BmiRegs->fmbm_rrquc;
            break;
        case (e_FM_PORT_COUNTERS_DMA_UTIL):
            *p_Ptr = &p_BmiRegs->fmbm_rduc;
            break;
        case (e_FM_PORT_COUNTERS_FIFO_UTIL):
            *p_Ptr = &p_BmiRegs->fmbm_rfuc;
            break;
        case (e_FM_PORT_COUNTERS_RX_PAUSE_ACTIVATION):
            *p_Ptr = &p_BmiRegs->fmbm_rpac;
            break;
        case (e_FM_PORT_COUNTERS_FRAME):
            *p_Ptr = &p_BmiRegs->fmbm_rfrc;
            break;
        case (e_FM_PORT_COUNTERS_DISCARD_FRAME):
            *p_Ptr = &p_BmiRegs->fmbm_rfcd;
            break;
        case (e_FM_PORT_COUNTERS_RX_BAD_FRAME):
            *p_Ptr = &p_BmiRegs->fmbm_rfbc;
            break;
        case (e_FM_PORT_COUNTERS_RX_LARGE_FRAME):
            *p_Ptr = &p_BmiRegs->fmbm_rlfc;
            break;
        case (e_FM_PORT_COUNTERS_RX_FILTER_FRAME):
            *p_Ptr = &p_BmiRegs->fmbm_rffc;
            break;
        case (e_FM_PORT_COUNTERS_RX_LIST_DMA_ERR):
            *p_Ptr = &p_BmiRegs->fmbm_rfldec;
            break;
        case (e_FM_PORT_COUNTERS_RX_OUT_OF_BUFFERS_DISCARD):
            *p_Ptr = &p_BmiRegs->fmbm_rodc;
            break;
        case (e_FM_PORT_COUNTERS_DEALLOC_BUF):
            *p_Ptr = &p_BmiRegs->fmbm_rbdc;
            break;
        case (e_FM_PORT_COUNTERS_PREPARE_TO_ENQUEUE_COUNTER):
            *p_Ptr = &p_BmiRegs->fmbm_rpec;
            break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter is not available for Rx ports"));
    }

    return E_OK;
}

static t_Error BmiTxPortCheckAndGetCounterPtr(t_FmPort *p_FmPort, e_FmPortCounters counter, volatile uint32_t **p_Ptr)
{
    t_FmPortTxBmiRegs   *p_BmiRegs = &p_FmPort->p_FmPortBmiRegs->txPortBmiRegs;

     /* check that counters are enabled */
    switch (counter)
    {
        case (e_FM_PORT_COUNTERS_CYCLE):
        case (e_FM_PORT_COUNTERS_TASK_UTIL):
        case (e_FM_PORT_COUNTERS_QUEUE_UTIL):
        case (e_FM_PORT_COUNTERS_DMA_UTIL):
        case (e_FM_PORT_COUNTERS_FIFO_UTIL):
            /* performance counters - may be read when disabled */
            break;
        case (e_FM_PORT_COUNTERS_FRAME):
        case (e_FM_PORT_COUNTERS_DISCARD_FRAME):
        case (e_FM_PORT_COUNTERS_LENGTH_ERR):
        case (e_FM_PORT_COUNTERS_UNSUPPRTED_FORMAT):
        case (e_FM_PORT_COUNTERS_DEALLOC_BUF):
            if (!(GET_UINT32(p_BmiRegs->fmbm_tstc) & BMI_COUNTERS_EN))
               RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter was not enabled"));
            break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter is not available for Tx ports"));
    }

    /* Set counter */
    switch (counter)
    {
        case (e_FM_PORT_COUNTERS_CYCLE):
           *p_Ptr = &p_BmiRegs->fmbm_tccn;
            break;
        case (e_FM_PORT_COUNTERS_TASK_UTIL):
           *p_Ptr = &p_BmiRegs->fmbm_ttuc;
            break;
        case (e_FM_PORT_COUNTERS_QUEUE_UTIL):
            *p_Ptr = &p_BmiRegs->fmbm_ttcquc;
            break;
        case (e_FM_PORT_COUNTERS_DMA_UTIL):
           *p_Ptr = &p_BmiRegs->fmbm_tduc;
            break;
        case (e_FM_PORT_COUNTERS_FIFO_UTIL):
           *p_Ptr = &p_BmiRegs->fmbm_tfuc;
            break;
        case (e_FM_PORT_COUNTERS_FRAME):
           *p_Ptr = &p_BmiRegs->fmbm_tfrc;
            break;
        case (e_FM_PORT_COUNTERS_DISCARD_FRAME):
           *p_Ptr = &p_BmiRegs->fmbm_tfdc;
            break;
        case (e_FM_PORT_COUNTERS_LENGTH_ERR):
           *p_Ptr = &p_BmiRegs->fmbm_tfledc;
            break;
        case (e_FM_PORT_COUNTERS_UNSUPPRTED_FORMAT):
            *p_Ptr = &p_BmiRegs->fmbm_tfufdc;
            break;
        case (e_FM_PORT_COUNTERS_DEALLOC_BUF):
            *p_Ptr = &p_BmiRegs->fmbm_tbdc;
            break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter is not available for Tx ports"));
    }

    return E_OK;
}

static t_Error BmiOhPortCheckAndGetCounterPtr(t_FmPort *p_FmPort, e_FmPortCounters counter, volatile uint32_t **p_Ptr)
{
    t_FmPortOhBmiRegs   *p_BmiRegs = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs;

    /* check that counters are enabled */
    switch (counter)
    {
        case (e_FM_PORT_COUNTERS_CYCLE):
        case (e_FM_PORT_COUNTERS_TASK_UTIL):
        case (e_FM_PORT_COUNTERS_DMA_UTIL):
        case (e_FM_PORT_COUNTERS_FIFO_UTIL):
            /* performance counters - may be read when disabled */
            break;
        case (e_FM_PORT_COUNTERS_FRAME):
        case (e_FM_PORT_COUNTERS_DISCARD_FRAME):
        case (e_FM_PORT_COUNTERS_RX_LIST_DMA_ERR):
        case (e_FM_PORT_COUNTERS_RX_OUT_OF_BUFFERS_DISCARD):
        case (e_FM_PORT_COUNTERS_WRED_DISCARD):
        case (e_FM_PORT_COUNTERS_LENGTH_ERR):
        case (e_FM_PORT_COUNTERS_UNSUPPRTED_FORMAT):
        case (e_FM_PORT_COUNTERS_DEALLOC_BUF):
            if (!(GET_UINT32(p_BmiRegs->fmbm_ostc) & BMI_COUNTERS_EN))
               RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter was not enabled"));
            break;
        case (e_FM_PORT_COUNTERS_RX_FILTER_FRAME): /* only valid for offline parsing */
            /* only driver uses host command port, so ASSERT rather than  RETURN_ERROR */
            ASSERT_COND(p_FmPort->portType != e_FM_PORT_TYPE_OH_HOST_COMMAND);
            if (!(GET_UINT32(p_BmiRegs->fmbm_ostc) & BMI_COUNTERS_EN))
               RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter was not enabled"));
            break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter(%d) is not available for O/H ports", counter));
    }

    /* Set counter */
    switch (counter)
    {
        case (e_FM_PORT_COUNTERS_CYCLE):
           *p_Ptr = &p_BmiRegs->fmbm_occn;
            break;
        case (e_FM_PORT_COUNTERS_TASK_UTIL):
           *p_Ptr = &p_BmiRegs->fmbm_otuc;
            break;
        case (e_FM_PORT_COUNTERS_DMA_UTIL):
           *p_Ptr = &p_BmiRegs->fmbm_oduc;
            break;
        case (e_FM_PORT_COUNTERS_FIFO_UTIL):
           *p_Ptr = &p_BmiRegs->fmbm_ofuc;
            break;
        case (e_FM_PORT_COUNTERS_FRAME):
           *p_Ptr = &p_BmiRegs->fmbm_ofrc;
            break;
        case (e_FM_PORT_COUNTERS_DISCARD_FRAME):
           *p_Ptr = &p_BmiRegs->fmbm_ofdc;
            break;
        case (e_FM_PORT_COUNTERS_RX_FILTER_FRAME):
           *p_Ptr = &p_BmiRegs->fmbm_offc;
            break;
        case (e_FM_PORT_COUNTERS_RX_LIST_DMA_ERR):
          *p_Ptr = &p_BmiRegs->fmbm_ofldec;
            break;
        case (e_FM_PORT_COUNTERS_WRED_DISCARD):
           *p_Ptr = &p_BmiRegs->fmbm_ofwdc;
            break;
        case (e_FM_PORT_COUNTERS_LENGTH_ERR):
           *p_Ptr = &p_BmiRegs->fmbm_ofledc;
            break;
        case (e_FM_PORT_COUNTERS_UNSUPPRTED_FORMAT):
            *p_Ptr = &p_BmiRegs->fmbm_ofufdc;
            break;
        case (e_FM_PORT_COUNTERS_DEALLOC_BUF):
            *p_Ptr = &p_BmiRegs->fmbm_obdc;
            break;
        case (e_FM_PORT_COUNTERS_RX_OUT_OF_BUFFERS_DISCARD):
            *p_Ptr = &p_BmiRegs->fmbm_oodc;
            break;
        case (e_FM_PORT_COUNTERS_PREPARE_TO_ENQUEUE_COUNTER):
            *p_Ptr = &p_BmiRegs->fmbm_opec;
            break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter is not available for O/H ports"));
    }

    return E_OK;
}

static t_Error AdditionalPrsParams(t_FmPort *p_FmPort, t_FmPcdPrsAdditionalHdrParams *p_HdrParams, uint32_t *p_SoftSeqAttachReg)
{
    uint8_t                     hdrNum, Ipv4HdrNum;
    u_FmPcdHdrPrsOpts           *p_prsOpts;
    uint32_t                    tmpReg = *p_SoftSeqAttachReg, tmpPrsOffset;

    if (IS_PRIVATE_HEADER(p_HdrParams->hdr) || IS_SPECIAL_HEADER(p_HdrParams->hdr))
        RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("No additional parameters for private or special headers."));

    if (p_HdrParams->errDisable)
        tmpReg |= PRS_HDR_ERROR_DIS;

    /* Set parser options */
    if (p_HdrParams->usePrsOpts)
    {
        p_prsOpts = &p_HdrParams->prsOpts;
        switch (p_HdrParams->hdr)
        {
            case (HEADER_TYPE_MPLS):
                if (p_prsOpts->mplsPrsOptions.labelInterpretationEnable)
                    tmpReg |= PRS_HDR_MPLS_LBL_INTER_EN;
                GET_PRS_HDR_NUM(hdrNum, p_prsOpts->mplsPrsOptions.nextParse);
                if (hdrNum == ILLEGAL_HDR_NUM)
                    RETURN_ERROR(MAJOR, E_INVALID_VALUE, NO_MSG);
                GET_PRS_HDR_NUM(Ipv4HdrNum, HEADER_TYPE_IPv4);
                if (hdrNum < Ipv4HdrNum)
                    RETURN_ERROR(MAJOR, E_INVALID_VALUE,
                        ("Header must be equal or higher than IPv4"));
                tmpReg |= ((uint32_t)hdrNum * PRS_HDR_ENTRY_SIZE) << PRS_HDR_MPLS_NEXT_HDR_SHIFT;
                break;
            case (HEADER_TYPE_PPPoE):
                if (p_prsOpts->pppoePrsOptions.enableMTUCheck)
                    tmpReg |= PRS_HDR_PPPOE_MTU_CHECK_EN;
                break;
            case (HEADER_TYPE_IPv6):
                if (p_prsOpts->ipv6PrsOptions.routingHdrEnable)
                    tmpReg |= PRS_HDR_IPV6_ROUTE_HDR_EN;
                break;
            case (HEADER_TYPE_TCP):
                if (p_prsOpts->tcpPrsOptions.padIgnoreChecksum)
                   tmpReg |= PRS_HDR_TCP_PAD_REMOVAL;
                else
                   tmpReg &= ~PRS_HDR_TCP_PAD_REMOVAL;
               break;
            case (HEADER_TYPE_UDP):
                if (p_prsOpts->udpPrsOptions.padIgnoreChecksum)
                   tmpReg |= PRS_HDR_UDP_PAD_REMOVAL;
                else
                   tmpReg &= ~PRS_HDR_UDP_PAD_REMOVAL;
                 break;
            default:
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid header"));
        }
    }

    /* set software parsing (address is devided in 2 since parser uses 2 byte access. */
    if (p_HdrParams->swPrsEnable)
    {
        tmpPrsOffset = FmPcdGetSwPrsOffset(p_FmPort->h_FmPcd, p_HdrParams->hdr, p_HdrParams->indexPerHdr);
        if (tmpPrsOffset == ILLEGAL_BASE)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, NO_MSG);
        tmpReg |= (PRS_HDR_SW_PRS_EN | tmpPrsOffset);
    }
    *p_SoftSeqAttachReg = tmpReg;

    return E_OK;
}

static uint32_t GetPortSchemeBindParams(t_Handle h_FmPort, t_FmPcdKgInterModuleBindPortToSchemes *p_SchemeBind)
{
    t_FmPort                    *p_FmPort = (t_FmPort*)h_FmPort;
    uint32_t                    walking1Mask = 0x80000000, tmp;
    uint8_t                     idx = 0;

    p_SchemeBind->netEnvId = p_FmPort->netEnvId;
    p_SchemeBind->hardwarePortId = p_FmPort->hardwarePortId;
    p_SchemeBind->useClsPlan = p_FmPort->useClsPlan;
    p_SchemeBind->numOfSchemes = 0;
    tmp = p_FmPort->schemesPerPortVector;
    if (tmp)
    {
        while (tmp)
        {
            if (tmp & walking1Mask)
            {
                p_SchemeBind->schemesIds[p_SchemeBind->numOfSchemes] = idx;
                p_SchemeBind->numOfSchemes++;
                tmp &= ~walking1Mask;
            }
            walking1Mask >>= 1;
            idx++;
        }
    }

    return tmp;
}

static t_Error SetPcd(t_FmPort *p_FmPort, t_FmPortPcdParams *p_PcdParams)
{
    t_Error                             err = E_OK;
    uint32_t                            tmpReg;
    volatile uint32_t                   *p_BmiNia=NULL;
    volatile uint32_t                   *p_BmiPrsNia=NULL;
    volatile uint32_t                   *p_BmiPrsStartOffset=NULL;
    volatile uint32_t                   *p_BmiInitPrsResult=NULL;
    volatile uint32_t                   *p_BmiCcBase=NULL;
    uint8_t                             hdrNum, L3HdrNum, greHdrNum;
    int                                 i;
    bool                                isEmptyClsPlanGrp;
    uint32_t                            tmpHxs[FM_PCD_PRS_NUM_OF_HDRS];
    uint16_t                            absoluteProfileId;
    uint8_t                             physicalSchemeId;
    uint32_t                            ccTreePhysOffset;
    t_FmPcdKgInterModuleBindPortToSchemes   schemeBind;

    ASSERT_COND(p_FmPort);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    if (p_FmPort->imEn)
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for non-independant mode ports only"));

    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_RX) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_OH_OFFLINE_PARSING))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx and offline parsing ports only"));

    p_FmPort->netEnvId = FmPcdGetNetEnvId(p_PcdParams->h_NetEnv);

    p_FmPort->pcdEngines = 0;

    /* initialize p_FmPort->pcdEngines field in port's structure */
    switch (p_PcdParams->pcdSupport)
    {
        case (e_FM_PORT_PCD_SUPPORT_NONE):
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("No PCD configuration required if e_FM_PORT_PCD_SUPPORT_NONE selected"));
        case (e_FM_PORT_PCD_SUPPORT_PRS_ONLY):
            p_FmPort->pcdEngines |= FM_PCD_PRS;
            break;
        case (e_FM_PORT_PCD_SUPPORT_PLCR_ONLY):
            p_FmPort->pcdEngines |= FM_PCD_PLCR;
            break;
        case (e_FM_PORT_PCD_SUPPORT_PRS_AND_PLCR):
            p_FmPort->pcdEngines |= FM_PCD_PRS;
            p_FmPort->pcdEngines |= FM_PCD_PLCR;
            break;
        case (e_FM_PORT_PCD_SUPPORT_PRS_AND_KG):
            p_FmPort->pcdEngines |= FM_PCD_PRS;
            p_FmPort->pcdEngines |= FM_PCD_KG;
            break;
        case (e_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_CC):
            p_FmPort->pcdEngines |= FM_PCD_PRS;
            p_FmPort->pcdEngines |= FM_PCD_CC;
            p_FmPort->pcdEngines |= FM_PCD_KG;
            break;
        case (e_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_CC_AND_PLCR):
            p_FmPort->pcdEngines |= FM_PCD_PRS;
            p_FmPort->pcdEngines |= FM_PCD_KG;
            p_FmPort->pcdEngines |= FM_PCD_CC;
            p_FmPort->pcdEngines |= FM_PCD_PLCR;
            break;
        case (e_FM_PORT_PCD_SUPPORT_PRS_AND_CC):
            p_FmPort->pcdEngines |= FM_PCD_PRS;
            p_FmPort->pcdEngines |= FM_PCD_CC;
            break;
        case (e_FM_PORT_PCD_SUPPORT_PRS_AND_CC_AND_PLCR):
            p_FmPort->pcdEngines |= FM_PCD_PRS;
            p_FmPort->pcdEngines |= FM_PCD_CC;
            p_FmPort->pcdEngines |= FM_PCD_PLCR;
            break;
        case (e_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_PLCR):
            p_FmPort->pcdEngines |= FM_PCD_PRS;
            p_FmPort->pcdEngines |= FM_PCD_KG;
            p_FmPort->pcdEngines |= FM_PCD_PLCR;
            break;
#ifdef FM_CAPWAP_SUPPORT
        case (e_FM_PORT_PCD_SUPPORT_CC_ONLY):
            p_FmPort->pcdEngines |= FM_PCD_CC;
            break;
        case (e_FM_PORT_PCD_SUPPORT_CC_AND_KG):
            p_FmPort->pcdEngines |= FM_PCD_CC;
            p_FmPort->pcdEngines |= FM_PCD_KG;
            break;
        case (e_FM_PORT_PCD_SUPPORT_CC_AND_KG_AND_PLCR):
            p_FmPort->pcdEngines |= FM_PCD_CC;
            p_FmPort->pcdEngines |= FM_PCD_KG;
            p_FmPort->pcdEngines |= FM_PCD_PLCR;
            break;
#endif /* FM_CAPWAP_SUPPORT */

        default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("invalid pcdSupport"));
    }

    if ((p_FmPort->pcdEngines & FM_PCD_PRS) &&
        (p_PcdParams->p_PrsParams->numOfHdrsWithAdditionalParams > FM_PCD_PRS_NUM_OF_HDRS))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Port parser numOfHdrsWithAdditionalParams may not exceed %d", FM_PCD_PRS_NUM_OF_HDRS));

    /* check that parameters exist for each and only each defined engine */
    if ((!!(p_FmPort->pcdEngines & FM_PCD_PRS) != !!p_PcdParams->p_PrsParams) ||
        (!!(p_FmPort->pcdEngines & FM_PCD_KG) != !!p_PcdParams->p_KgParams) ||
        (!!(p_FmPort->pcdEngines & FM_PCD_CC) != !!p_PcdParams->p_CcParams))
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("PCD initialization structure is not consistent with pcdSupport"));

    /* get PCD registers pointers */
    switch (p_FmPort->portType)
    {
        case (e_FM_PORT_TYPE_RX_10G):
        case (e_FM_PORT_TYPE_RX):
            p_BmiNia = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfne;
            p_BmiPrsNia = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfpne;
            p_BmiPrsStartOffset = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rpso;
            p_BmiInitPrsResult = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rprai[0];
            p_BmiCcBase = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rccb;
            break;
        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
            p_BmiNia = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofne;
            p_BmiPrsNia = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofpne;
            p_BmiPrsStartOffset = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_opso;
            p_BmiInitPrsResult = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_oprai[0];
            p_BmiCcBase = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_occb;
            break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid port type"));
    }

    /* set PCD port parameter */
    if (p_FmPort->pcdEngines & FM_PCD_CC)
    {
        err = FmPcdCcBindTree(p_FmPort->h_FmPcd,
                              p_PcdParams,
                              p_PcdParams->p_CcParams->h_CcTree,
                              &ccTreePhysOffset,
                              p_FmPort);
        if (err)
            RETURN_ERROR(MAJOR, err, NO_MSG);

        WRITE_UINT32(*p_BmiCcBase, ccTreePhysOffset);
        p_FmPort->ccTreeId = p_PcdParams->p_CcParams->h_CcTree;
    }

    if (p_FmPort->pcdEngines & FM_PCD_KG)
    {
        if (p_PcdParams->p_KgParams->numOfSchemes == 0)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("For ports using Keygen, at least one scheme must be bound. "));

        err = FmPcdKgSetOrBindToClsPlanGrp(p_FmPort->h_FmPcd,
                                           p_FmPort->hardwarePortId,
                                           p_FmPort->netEnvId,
                                           p_FmPort->optArray,
                                           &p_FmPort->clsPlanGrpId,
                                           &isEmptyClsPlanGrp);
         if (err)
             RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("FmPcdKgSetOrBindToClsPlanGrp failed. "));

         p_FmPort->useClsPlan = !isEmptyClsPlanGrp;

        schemeBind.netEnvId = p_FmPort->netEnvId;
        schemeBind.hardwarePortId = p_FmPort->hardwarePortId;
        schemeBind.numOfSchemes = p_PcdParams->p_KgParams->numOfSchemes;
        schemeBind.useClsPlan = p_FmPort->useClsPlan;

        /* for each scheme */
        for (i=0; i<p_PcdParams->p_KgParams->numOfSchemes; i++)
        {
            ASSERT_COND(p_PcdParams->p_KgParams->h_Schemes[i]);
            physicalSchemeId = FmPcdKgGetSchemeId(p_PcdParams->p_KgParams->h_Schemes[i]);
            schemeBind.schemesIds[i] = physicalSchemeId;
            /* build vector */
            p_FmPort->schemesPerPortVector |= 1 << (31 - (uint32_t)physicalSchemeId);
#if (DPAA_VERSION >= 11)
            /*because of the state that VSPE is defined per port - all PCD path should be according to this requirement
             if !VSPE - in port, for relevant scheme VSPE can not be set*/
            if (!p_FmPort->vspe && FmPcdKgGetVspe((p_PcdParams->p_KgParams->h_Schemes[i])))
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("VSPE is not at port level"));
#endif /* (DPAA_VERSION >= 11) */
        }

        err = FmPcdKgBindPortToSchemes(p_FmPort->h_FmPcd, &schemeBind);
        if (err)
            RETURN_ERROR(MAJOR, err, NO_MSG);
    }

    /***************************/
    /* configure NIA after BMI */
    /***************************/
    /* rfne may contain FDCS bits, so first we read them. */
    p_FmPort->savedBmiNia = GET_UINT32(*p_BmiNia) & BMI_RFNE_FDCS_MASK;

    /* If policer is used directly after BMI or PRS */
    if ((p_FmPort->pcdEngines & FM_PCD_PLCR) &&
        ((p_PcdParams->pcdSupport == e_FM_PORT_PCD_SUPPORT_PLCR_ONLY) ||
         (p_PcdParams->pcdSupport == e_FM_PORT_PCD_SUPPORT_PRS_AND_PLCR)))
    {
        if (!p_PcdParams->p_PlcrParams->h_Profile)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Profile should be initialized"));

        absoluteProfileId = (uint16_t)FmPcdPlcrProfileGetAbsoluteId(p_PcdParams->p_PlcrParams->h_Profile);

        if (!FmPcdPlcrIsProfileValid(p_FmPort->h_FmPcd, absoluteProfileId))
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Private port profile not valid."));

        tmpReg = (uint32_t)(absoluteProfileId | NIA_PLCR_ABSOLUTE);

        if (p_FmPort->pcdEngines & FM_PCD_PRS) /* e_FM_PCD_SUPPORT_PRS_AND_PLCR */
            /* update BMI HPNIA */
            WRITE_UINT32(*p_BmiPrsNia, (uint32_t)(NIA_ENG_PLCR | tmpReg));
        else /* e_FM_PCD_SUPPORT_PLCR_ONLY */
            /* update BMI NIA */
            p_FmPort->savedBmiNia |= (uint32_t)(NIA_ENG_PLCR);
    }

#ifdef FM_CAPWAP_SUPPORT
    /* if CC is used directly after BMI */
    if ((p_PcdParams->pcdSupport == e_FM_PORT_PCD_SUPPORT_CC_ONLY) ||
        (p_PcdParams->pcdSupport == e_FM_PORT_PCD_SUPPORT_CC_AND_KG) ||
        (p_PcdParams->pcdSupport == e_FM_PORT_PCD_SUPPORT_CC_AND_KG_AND_PLCR))
    {
        if (p_FmPort->portType != e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
            RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("e_FM_PORT_PCD_SUPPORT_CC_xx available for offline parsing ports only"));
        p_FmPort->savedBmiNia |= (uint32_t)(NIA_ENG_FM_CTL | NIA_FM_CTL_AC_CC);
         /* check that prs start offset == RIM[FOF] */
    }
#endif /* FM_CAPWAP_SUPPORT */

    if (p_FmPort->pcdEngines & FM_PCD_PRS)
    {
        ASSERT_COND(p_PcdParams->p_PrsParams);
        /* if PRS is used it is always first */
        GET_PRS_HDR_NUM(hdrNum, p_PcdParams->p_PrsParams->firstPrsHdr);
        if (hdrNum == ILLEGAL_HDR_NUM)
            RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("Unsupported header."));
        p_FmPort->savedBmiNia |= (uint32_t)(NIA_ENG_PRS | (uint32_t)(hdrNum));
        /* set after parser NIA */
        tmpReg = 0;
        switch (p_PcdParams->pcdSupport)
        {
            case (e_FM_PORT_PCD_SUPPORT_PRS_ONLY):
                WRITE_UINT32(*p_BmiPrsNia, GET_NIA_BMI_AC_ENQ_FRAME(p_FmPort->h_FmPcd));
                break;
            case (e_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_CC):
            case (e_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_CC_AND_PLCR):
                tmpReg = NIA_KG_CC_EN;
            case (e_FM_PORT_PCD_SUPPORT_PRS_AND_KG):
            case (e_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_PLCR):
                if (p_PcdParams->p_KgParams->directScheme)
                {
                    physicalSchemeId = FmPcdKgGetSchemeId(p_PcdParams->p_KgParams->h_DirectScheme);
                    /* check that this scheme was bound to this port */
                    for (i=0 ; i<p_PcdParams->p_KgParams->numOfSchemes; i++)
                        if (p_PcdParams->p_KgParams->h_DirectScheme == p_PcdParams->p_KgParams->h_Schemes[i])
                            break;
                    if (i == p_PcdParams->p_KgParams->numOfSchemes)
                        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Direct scheme is not one of the port selected schemes."));
                    tmpReg |= (uint32_t)(NIA_KG_DIRECT | physicalSchemeId);
                }
                WRITE_UINT32(*p_BmiPrsNia, NIA_ENG_KG | tmpReg);
                break;
            case (e_FM_PORT_PCD_SUPPORT_PRS_AND_CC):
            case (e_FM_PORT_PCD_SUPPORT_PRS_AND_CC_AND_PLCR):
                WRITE_UINT32(*p_BmiPrsNia, (uint32_t)(NIA_ENG_FM_CTL | NIA_FM_CTL_AC_CC));
                break;
            case (e_FM_PORT_PCD_SUPPORT_PRS_AND_PLCR):
                break;
            default:
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid PCD support"));
        }

        /* set start parsing offset */
         WRITE_UINT32(*p_BmiPrsStartOffset, p_PcdParams->p_PrsParams->parsingOffset);

        /************************************/
        /* Parser port parameters           */
        /************************************/
        /* stop before configuring */
        WRITE_UINT32(p_FmPort->p_FmPortPrsRegs->pcac, PRS_CAC_STOP);
        /* wait for parser to be in idle state */
        while (GET_UINT32(p_FmPort->p_FmPortPrsRegs->pcac) & PRS_CAC_ACTIVE) ;

        /* set soft seq attachment register */
        memset(tmpHxs, 0, FM_PCD_PRS_NUM_OF_HDRS*sizeof(uint32_t));

        /* set protocol options */
        for (i=0;p_FmPort->optArray[i];i++)
            switch (p_FmPort->optArray[i])
            {
                case (ETH_BROADCAST):
                    GET_PRS_HDR_NUM(hdrNum, HEADER_TYPE_ETH)
                    tmpHxs[hdrNum] |= (i+1) << PRS_HDR_ETH_BC_SHIFT;
                    break;
                case (ETH_MULTICAST):
                    GET_PRS_HDR_NUM(hdrNum, HEADER_TYPE_ETH)
                    tmpHxs[hdrNum] |= (i+1) << PRS_HDR_ETH_MC_SHIFT;
                    break;
                case (VLAN_STACKED):
                    GET_PRS_HDR_NUM(hdrNum, HEADER_TYPE_VLAN)
                    tmpHxs[hdrNum] |= (i+1)<< PRS_HDR_VLAN_STACKED_SHIFT;
                    break;
                case (MPLS_STACKED):
                    GET_PRS_HDR_NUM(hdrNum, HEADER_TYPE_MPLS)
                    tmpHxs[hdrNum] |= (i+1) << PRS_HDR_MPLS_STACKED_SHIFT;
                    break;
                case (IPV4_BROADCAST_1):
                    GET_PRS_HDR_NUM(hdrNum, HEADER_TYPE_IPv4)
                    tmpHxs[hdrNum] |= (i+1) << PRS_HDR_IPV4_1_BC_SHIFT;
                    break;
                case (IPV4_MULTICAST_1):
                    GET_PRS_HDR_NUM(hdrNum, HEADER_TYPE_IPv4)
                    tmpHxs[hdrNum] |= (i+1) << PRS_HDR_IPV4_1_MC_SHIFT;
                    break;
                case (IPV4_UNICAST_2):
                    GET_PRS_HDR_NUM(hdrNum, HEADER_TYPE_IPv4)
                    tmpHxs[hdrNum] |= (i+1) << PRS_HDR_IPV4_2_UC_SHIFT;
                    break;
                case (IPV4_MULTICAST_BROADCAST_2):
                    GET_PRS_HDR_NUM(hdrNum, HEADER_TYPE_IPv4)
                    tmpHxs[hdrNum] |= (i+1) << PRS_HDR_IPV4_2_MC_BC_SHIFT;
                    break;
                case (IPV6_MULTICAST_1):
                    GET_PRS_HDR_NUM(hdrNum, HEADER_TYPE_IPv6)
                    tmpHxs[hdrNum] |= (i+1) << PRS_HDR_IPV6_1_MC_SHIFT;
                    break;
                case (IPV6_UNICAST_2):
                    GET_PRS_HDR_NUM(hdrNum, HEADER_TYPE_IPv6)
                    tmpHxs[hdrNum] |= (i+1) << PRS_HDR_IPV6_2_UC_SHIFT;
                    break;
                case (IPV6_MULTICAST_2):
                    GET_PRS_HDR_NUM(hdrNum, HEADER_TYPE_IPv6)
                    tmpHxs[hdrNum] |= (i+1) << PRS_HDR_IPV6_2_MC_SHIFT;
                    break;
            }

        if (FmPcdNetEnvIsHdrExist(p_FmPort->h_FmPcd,
                                  p_FmPort->netEnvId, HEADER_TYPE_UDP_ENCAP_ESP))
        {
            p_PcdParams->p_PrsParams->additionalParams
                [p_PcdParams->p_PrsParams->numOfHdrsWithAdditionalParams].hdr = HEADER_TYPE_UDP;
            p_PcdParams->p_PrsParams->additionalParams
                [p_PcdParams->p_PrsParams->numOfHdrsWithAdditionalParams].swPrsEnable = TRUE;
            p_PcdParams->p_PrsParams->numOfHdrsWithAdditionalParams++;
        }

        /* set MPLS default next header - HW reset workaround  */
        GET_PRS_HDR_NUM(hdrNum, HEADER_TYPE_MPLS)
        tmpHxs[hdrNum] |= PRS_HDR_MPLS_LBL_INTER_EN;
        GET_PRS_HDR_NUM(L3HdrNum, HEADER_TYPE_USER_DEFINED_L3);
        tmpHxs[hdrNum] |= (uint32_t)L3HdrNum << PRS_HDR_MPLS_NEXT_HDR_SHIFT;

        /* for GRE, disable errors */
        GET_PRS_HDR_NUM(greHdrNum, HEADER_TYPE_GRE);
        tmpHxs[greHdrNum] |= PRS_HDR_ERROR_DIS;

        /* For UDP remove PAD from L4 checksum calculation */
        GET_PRS_HDR_NUM(hdrNum, HEADER_TYPE_UDP);
        tmpHxs[hdrNum] |= PRS_HDR_UDP_PAD_REMOVAL;
        /* For TCP remove PAD from L4 checksum calculation */
        GET_PRS_HDR_NUM(hdrNum, HEADER_TYPE_TCP);
        tmpHxs[hdrNum] |= PRS_HDR_TCP_PAD_REMOVAL;

        /* config additional params for specific headers */
        for (i=0; i<p_PcdParams->p_PrsParams->numOfHdrsWithAdditionalParams; i++)
        {
            GET_PRS_HDR_NUM(hdrNum, p_PcdParams->p_PrsParams->additionalParams[i].hdr);
            if (hdrNum== ILLEGAL_HDR_NUM)
                RETURN_ERROR(MAJOR, E_INVALID_VALUE, NO_MSG);
            if (hdrNum==NO_HDR_NUM)
                RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Private headers may not use additional parameters"));

            err = AdditionalPrsParams(p_FmPort, &p_PcdParams->p_PrsParams->additionalParams[i], &tmpHxs[hdrNum]);
            if (err)
                RETURN_ERROR(MAJOR, E_INVALID_VALUE, NO_MSG);
        }

        /* Check if ip-reassembly port - need to update NIAs */
        if (p_FmPort->h_IpReassemblyManip)
        {
           /* link to sw parser code for IP Frag - only if no other code is applied. */
            GET_PRS_HDR_NUM(hdrNum, HEADER_TYPE_IPv4)
            if (!(tmpHxs[hdrNum] & PRS_HDR_SW_PRS_EN))
                tmpHxs[hdrNum] |= (PRS_HDR_SW_PRS_EN | IP_FRAG_SW_PATCH_IPv4_LABEL);
            GET_PRS_HDR_NUM(hdrNum, HEADER_TYPE_IPv6)
            if (!(tmpHxs[hdrNum] & PRS_HDR_SW_PRS_EN))
                tmpHxs[hdrNum] |= (PRS_HDR_SW_PRS_EN | IP_FRAG_SW_PATCH_IPv6_LABEL);
        }

        if (FmPcdIsAdvancedOffloadSupported(p_FmPort->h_FmPcd) &&
            (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING))
        {
            /* link to sw parser code for IP Frag - only if no other code is applied. */
            GET_PRS_HDR_NUM(hdrNum, HEADER_TYPE_IPv6)
            if (!(tmpHxs[hdrNum] & PRS_HDR_SW_PRS_EN))
                tmpHxs[hdrNum] |= (PRS_HDR_SW_PRS_EN | IP_FRAG_SW_PATCH_IPv6_LABEL);
        }

#ifdef FM_CAPWAP_SUPPORT
        if (FmPcdNetEnvIsHdrExist(p_FmPort->h_FmPcd,
                                  p_FmPort->netEnvId, HEADER_TYPE_UDP_LITE))
        {
           /* link to sw parser code for udp lite - only if no other code is applied. */
            GET_PRS_HDR_NUM(hdrNum, HEADER_TYPE_USER_DEFINED_L4)
            if (!(tmpHxs[hdrNum] & PRS_HDR_SW_PRS_EN))
                tmpHxs[hdrNum] |= (PRS_HDR_SW_PRS_EN | UDP_LITE_SW_PATCH_LABEL);
        }
#endif /* FM_CAPWAP_SUPPORT */
        for (i=0 ; i<FM_PCD_PRS_NUM_OF_HDRS ; i++)
        {
            /* For all header set LCV as taken from netEnv*/
            WRITE_UINT32(p_FmPort->p_FmPortPrsRegs->hdrs[i].lcv,
                         FmPcdGetLcv(p_FmPort->h_FmPcd, p_FmPort->netEnvId, (uint8_t)i));
            /* set HXS register according to default+Additional params+protocol options */
            WRITE_UINT32(p_FmPort->p_FmPortPrsRegs->hdrs[i].softSeqAttach,  tmpHxs[i]);
        }

        /* set tpid. */
        tmpReg = PRS_TPID_DFLT;
        if (p_PcdParams->p_PrsParams->setVlanTpid1)
        {
            tmpReg &= PRS_TPID2_MASK;
            tmpReg |= (uint32_t)p_PcdParams->p_PrsParams->vlanTpid1 << PRS_PCTPID_SHIFT;
        }
        if (p_PcdParams->p_PrsParams->setVlanTpid2)
        {
            tmpReg &= PRS_TPID1_MASK;
            tmpReg |= (uint32_t)p_PcdParams->p_PrsParams->vlanTpid2;
        }
        WRITE_UINT32(p_FmPort->p_FmPortPrsRegs->pctpid, tmpReg);

        /* enable parser */
        WRITE_UINT32(p_FmPort->p_FmPortPrsRegs->pcac, 0);

        if (p_PcdParams->p_PrsParams->prsResultPrivateInfo)
            p_FmPort->privateInfo = p_PcdParams->p_PrsParams->prsResultPrivateInfo;

    } /* end parser */
    else
        p_FmPort->privateInfo = 0;

    WRITE_UINT32(*p_BmiPrsStartOffset, GET_UINT32(*p_BmiPrsStartOffset) + p_FmPort->internalBufferOffset);

    /* set initial parser result - used for all engines */
    for (i=0;i<FM_PORT_PRS_RESULT_NUM_OF_WORDS;i++)
    {
        if (!i)
            WRITE_UINT32(*(p_BmiInitPrsResult),
                         (uint32_t)(((uint32_t)p_FmPort->privateInfo << BMI_PR_PORTID_SHIFT)
                                    | BMI_PRS_RESULT_HIGH));
        else
        {
            if (i< FM_PORT_PRS_RESULT_NUM_OF_WORDS/2)
                WRITE_UINT32(*(p_BmiInitPrsResult+i), BMI_PRS_RESULT_HIGH);
            else
                WRITE_UINT32(*(p_BmiInitPrsResult+i), BMI_PRS_RESULT_LOW);
        }
    }

    return E_OK;
}

static t_Error DeletePcd(t_FmPort *p_FmPort)
{
    t_Error                             err = E_OK;
    volatile uint32_t                   *p_BmiNia=NULL;
    volatile uint32_t                   *p_BmiPrsStartOffset = NULL;

    ASSERT_COND(p_FmPort);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    if (p_FmPort->imEn)
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for non-independant mode ports only"));

    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_RX) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_OH_OFFLINE_PARSING))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx and offline parsing ports only"));

    if (!p_FmPort->pcdEngines)
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("called for non PCD port"));

    /* get PCD registers pointers */
    switch (p_FmPort->portType)
    {
        case (e_FM_PORT_TYPE_RX_10G):
        case (e_FM_PORT_TYPE_RX):
            p_BmiNia = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfne;
            p_BmiPrsStartOffset = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rpso;
            break;
        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
            p_BmiNia = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofne;
            p_BmiPrsStartOffset = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_opso;
            break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid port type"));
    }

    if ((GET_UINT32(*p_BmiNia) & GET_NO_PCD_NIA_BMI_AC_ENQ_FRAME()) !=
        GET_NO_PCD_NIA_BMI_AC_ENQ_FRAME())
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("port has to be detached previousely"));

    /* "cut" PCD out of the port's flow - go to BMI */
    /* WRITE_UINT32(*p_BmiNia, (p_FmPort->savedBmiNia & BMI_RFNE_FDCS_MASK) | (NIA_ENG_BMI | NIA_BMI_AC_ENQ_FRAME)); */

    if (p_FmPort->pcdEngines | FM_PCD_PRS)
    {
        WRITE_UINT32(*p_BmiPrsStartOffset, 0);

        /* stop parser */
        WRITE_UINT32(p_FmPort->p_FmPortPrsRegs->pcac, PRS_CAC_STOP);
        /* wait for parser to be in idle state */
        while (GET_UINT32(p_FmPort->p_FmPortPrsRegs->pcac) & PRS_CAC_ACTIVE) ;
    }

    if (p_FmPort->pcdEngines & FM_PCD_KG)
    {
        t_FmPcdKgInterModuleBindPortToSchemes   schemeBind;

        /* unbind all schemes */
        p_FmPort->schemesPerPortVector = GetPortSchemeBindParams(p_FmPort, &schemeBind);

        err = FmPcdKgUnbindPortToSchemes(p_FmPort->h_FmPcd, &schemeBind);
        if (err)
            RETURN_ERROR(MAJOR, err, NO_MSG);

        err = FmPcdKgDeleteOrUnbindPortToClsPlanGrp(p_FmPort->h_FmPcd, p_FmPort->hardwarePortId, p_FmPort->clsPlanGrpId);
        if (err)
            RETURN_ERROR(MAJOR, err, NO_MSG);
        p_FmPort->useClsPlan = FALSE;
    }

    if (p_FmPort->pcdEngines & FM_PCD_CC)
    {
        /* unbind - we need to get the treeId too */
        err = FmPcdCcUnbindTree(p_FmPort->h_FmPcd,  p_FmPort->ccTreeId);
        if (err)
            RETURN_ERROR(MAJOR, err, NO_MSG);
    }

    p_FmPort->pcdEngines = 0;

    return E_OK;
}

static t_Error AttachPCD(t_FmPort *p_FmPort)
{
    volatile uint32_t                   *p_BmiNia=NULL;

    ASSERT_COND(p_FmPort);

    /* get PCD registers pointers */
    if (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
        p_BmiNia = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofne;
    else
        p_BmiNia = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfne;

    /* check that current NIA is BMI to BMI */
    if ((GET_UINT32(*p_BmiNia) & ~BMI_RFNE_FDCS_MASK) != GET_NO_PCD_NIA_BMI_AC_ENQ_FRAME())
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION,
                     ("may be called only for ports in BMI-to-BMI state."));

    if (p_FmPort->requiredAction & UPDATE_FMFP_PRC_WITH_ONE_RISC_ONLY)
        if (FmSetNumOfRiscsPerPort(p_FmPort->h_Fm, p_FmPort->hardwarePortId, 1, p_FmPort->orFmanCtrl)!= E_OK)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);

    if (p_FmPort->requiredAction & UPDATE_NIA_CMNE)
    {
        if (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
            WRITE_UINT32(p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ocmne, p_FmPort->savedBmiCmne);
        else
            WRITE_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rcmne, p_FmPort->savedBmiCmne);
    }

    if (p_FmPort->requiredAction & UPDATE_NIA_PNEN)
        WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnen, p_FmPort->savedQmiPnen);

    if (p_FmPort->requiredAction & UPDATE_NIA_FENE)
    {
        if (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
            WRITE_UINT32(p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofene, p_FmPort->savedBmiFene);
        else
            WRITE_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfene, p_FmPort->savedBmiFene);
    }

    if (p_FmPort->requiredAction & UPDATE_NIA_FPNE)
    {
        if (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
            WRITE_UINT32(p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofpne, p_FmPort->savedBmiFpne);
        else
            WRITE_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfpne, p_FmPort->savedBmiFpne);
    }

    WRITE_UINT32(*p_BmiNia, p_FmPort->savedBmiNia);

    if (p_FmPort->requiredAction & UPDATE_NIA_PNDN)
    {
        p_FmPort->origNonRxQmiRegsPndn = GET_UINT32(p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs.fmqm_pndn);
        WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs.fmqm_pndn, p_FmPort->savedNonRxQmiRegsPndn);
    }

    return E_OK;
}

static t_Error DetachPCD(t_FmPort *p_FmPort)
{
    volatile uint32_t                   *p_BmiNia=NULL;

    ASSERT_COND(p_FmPort);

    /* get PCD registers pointers */
    if (p_FmPort->requiredAction & UPDATE_NIA_PNDN)
        WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs.fmqm_pndn, p_FmPort->origNonRxQmiRegsPndn);

    if (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
        p_BmiNia = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofne;
    else
        p_BmiNia = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfne;

    WRITE_UINT32(*p_BmiNia, (p_FmPort->savedBmiNia & BMI_RFNE_FDCS_MASK) |
                            GET_NO_PCD_NIA_BMI_AC_ENQ_FRAME());

    if (FmPcdGetHcHandle(p_FmPort->h_FmPcd))
        FmPcdHcSync(p_FmPort->h_FmPcd);

    if (p_FmPort->requiredAction & UPDATE_NIA_FENE)
    {
        if (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
            WRITE_UINT32(p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofene, NIA_ENG_QMI_ENQ | NIA_ORDER_RESTOR);
        else
            WRITE_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfene, NIA_ENG_QMI_ENQ | NIA_ORDER_RESTOR);
    }

    if (p_FmPort->requiredAction & UPDATE_NIA_PNEN)
        WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnen, NIA_ENG_BMI | NIA_BMI_AC_RELEASE);

    if (p_FmPort->requiredAction & UPDATE_FMFP_PRC_WITH_ONE_RISC_ONLY)
        if (FmSetNumOfRiscsPerPort(p_FmPort->h_Fm, p_FmPort->hardwarePortId, 2, p_FmPort->orFmanCtrl)!= E_OK)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);

    p_FmPort->requiredAction = 0;

    return E_OK;
}


/*****************************************************************************/
/*              Inter-module API routines                                    */
/*****************************************************************************/

void FmPortSetMacsecLcv(t_Handle h_FmPort)
{
    t_FmPort                    *p_FmPort = (t_FmPort*)h_FmPort;
    volatile uint32_t           *p_BmiCfgReg = NULL;
    uint32_t                    macsecEn = BMI_PORT_CFG_EN_MACSEC;
    uint32_t                    lcv, walking1Mask = 0x80000000;
    uint8_t                     cnt = 0;

    SANITY_CHECK_RETURN(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) && (p_FmPort->portType != e_FM_PORT_TYPE_RX))
    {
        REPORT_ERROR(MAJOR, E_INVALID_OPERATION, ("The routine is relevant for Rx ports only"));
        return;
    }

    p_BmiCfgReg = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rcfg;
    /* get LCV for MACSEC */
    if ((p_FmPort->h_FmPcd) && ((lcv = FmPcdGetMacsecLcv(p_FmPort->h_FmPcd, p_FmPort->netEnvId))!= 0))
    {
        while (!(lcv & walking1Mask))
        {
            cnt++;
            walking1Mask >>= 1;
        }

        macsecEn |= (uint32_t)cnt << BMI_PORT_CFG_MS_SEL_SHIFT;
     }

     WRITE_UINT32(*p_BmiCfgReg, GET_UINT32(*p_BmiCfgReg) | macsecEn);
}

void FmPortSetMacsecCmd(t_Handle h_FmPort, uint8_t dfltSci)
{
    t_FmPort                    *p_FmPort = (t_FmPort*)h_FmPort;
    volatile uint32_t           *p_BmiCfgReg = NULL;
    uint32_t                    tmpReg;

    SANITY_CHECK_RETURN(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN(p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    if ((p_FmPort->portType != e_FM_PORT_TYPE_TX_10G) && (p_FmPort->portType != e_FM_PORT_TYPE_TX))
    {
        REPORT_ERROR(MAJOR, E_INVALID_OPERATION, ("The routine is relevant for Tx ports only"));
        return;
    }

    p_BmiCfgReg = &p_FmPort->p_FmPortBmiRegs->txPortBmiRegs.fmbm_tfca;
    tmpReg = GET_UINT32(*p_BmiCfgReg) & ~BMI_CMD_ATTR_MACCMD_MASK;
    tmpReg |= BMI_CMD_ATTR_MACCMD_SECURED;
    tmpReg |= (((uint32_t)dfltSci << BMI_CMD_ATTR_MACCMD_SC_SHIFT) & BMI_CMD_ATTR_MACCMD_SC_MASK);

    WRITE_UINT32(*p_BmiCfgReg, tmpReg);
}

uint8_t FmPortGetNetEnvId(t_Handle h_FmPort)
{
    return ((t_FmPort*)h_FmPort)->netEnvId;
}

uint8_t FmPortGetHardwarePortId(t_Handle h_FmPort)
{
    return ((t_FmPort*)h_FmPort)->hardwarePortId;
}

uint32_t FmPortGetPcdEngines(t_Handle h_FmPort)
{
    return ((t_FmPort*)h_FmPort)->pcdEngines;
}

#if (DPAA_VERSION >= 11)
t_Error FmPortSetGprFunc(t_Handle h_FmPort, e_FmPortGprFuncType gprFunc, void **p_Value)
{
    t_FmPort            *p_FmPort = (t_FmPort*)h_FmPort;
    uint32_t            muramPageOffset;

    ASSERT_COND(p_FmPort);
    ASSERT_COND(p_Value);

    if (p_FmPort->gprFunc != e_FM_PORT_GPR_EMPTY)
    {
        if (p_FmPort->gprFunc != gprFunc)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("gpr was assigned with different func"));
    }
    else
    {
        switch (gprFunc)
        {
            case (e_FM_PORT_GPR_MURAM_PAGE):
                p_FmPort->p_ParamsPage = FM_MURAM_AllocMem(p_FmPort->h_FmMuram,
                                                           256,
                                                           8);
                if (!p_FmPort->p_ParamsPage)
                    RETURN_ERROR(MAJOR, E_NO_MEMORY, ("MURAM alloc for page"));

                IOMemSet32(p_FmPort->p_ParamsPage, 0, 256);
                muramPageOffset = (uint32_t)(XX_VirtToPhys(p_FmPort->p_ParamsPage) -
                                             p_FmPort->fmMuramPhysBaseAddr);
                switch (p_FmPort->portType)
                {
                    case (e_FM_PORT_TYPE_RX_10G):
                    case (e_FM_PORT_TYPE_RX):
                        WRITE_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rgpr, muramPageOffset);
                        break;
                    case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
                        WRITE_UINT32(p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ogpr, muramPageOffset);
                        break;
                    default:
                        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid port type"));
                }
                break;
            default:
                RETURN_ERROR(MAJOR, E_INVALID_SELECTION, NO_MSG);
        }
        p_FmPort->gprFunc = gprFunc;
    }

    switch (p_FmPort->gprFunc)
    {
        case (e_FM_PORT_GPR_MURAM_PAGE):
            *p_Value = p_FmPort->p_ParamsPage;
            break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_SELECTION, NO_MSG);
    }

    return E_OK;
}
#endif /* (DPAA_VERSION >= 11) */

t_Error FmPortGetSetCcParams(t_Handle h_FmPort, t_FmPortGetSetCcParams *p_CcParams)
{
    t_FmPort            *p_FmPort = (t_FmPort*)h_FmPort;
    int                 tmpInt;
    volatile uint32_t   *p_BmiPrsStartOffset = NULL;

    /* this function called from Cc for pass and receive parameters port params between CC and PORT*/

    if ((p_CcParams->getCcParams.type & OFFSET_OF_PR) &&
        (p_FmPort->bufferOffsets.prsResultOffset != ILLEGAL_BASE))
    {
        p_CcParams->getCcParams.prOffset = (uint8_t)p_FmPort->bufferOffsets.prsResultOffset;
        p_CcParams->getCcParams.type &= ~OFFSET_OF_PR;
    }
    if (p_CcParams->getCcParams.type & HW_PORT_ID)
    {
        p_CcParams->getCcParams.hardwarePortId = (uint8_t)p_FmPort->hardwarePortId;
        p_CcParams->getCcParams.type &= ~HW_PORT_ID;
    }
    if ((p_CcParams->getCcParams.type & OFFSET_OF_DATA) &&
        (p_FmPort->bufferOffsets.dataOffset != ILLEGAL_BASE))
    {
        p_CcParams->getCcParams.dataOffset = (uint16_t)p_FmPort->bufferOffsets.dataOffset;
        p_CcParams->getCcParams.type &= ~OFFSET_OF_DATA;
    }
    if (p_CcParams->getCcParams.type & NUM_OF_TASKS)
    {
        p_CcParams->getCcParams.numOfTasks = (uint8_t)p_FmPort->tasks.num;
        p_CcParams->getCcParams.type &= ~NUM_OF_TASKS;
    }
    if (p_CcParams->getCcParams.type & NUM_OF_EXTRA_TASKS)
    {
        p_CcParams->getCcParams.numOfExtraTasks = (uint8_t)p_FmPort->tasks.extra;
        p_CcParams->getCcParams.type &= ~NUM_OF_EXTRA_TASKS;
    }
    if (p_CcParams->getCcParams.type & FM_REV)
    {
        p_CcParams->getCcParams.revInfo.majorRev = p_FmPort->fmRevInfo.majorRev;
        p_CcParams->getCcParams.revInfo.minorRev = p_FmPort->fmRevInfo.minorRev;
        p_CcParams->getCcParams.type &= ~FM_REV;
    }
    if (p_CcParams->getCcParams.type & MANIP_EXTRA_SPACE)
    {
        p_CcParams->getCcParams.internalBufferOffset = p_FmPort->internalBufferOffset;
        p_CcParams->getCcParams.type &= ~MANIP_EXTRA_SPACE;
    }
    if (p_CcParams->getCcParams.type & GET_NIA_FPNE)
    {
        if (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
            p_CcParams->getCcParams.nia = GET_UINT32(p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofpne);
        else
            p_CcParams->getCcParams.nia = GET_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfpne);
        p_CcParams->getCcParams.type &= ~GET_NIA_FPNE;
    }
    if (p_CcParams->getCcParams.type & GET_NIA_PNDN)
    {
        if (p_FmPort->portType != e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid port type"));
        p_CcParams->getCcParams.nia = GET_UINT32(p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs.fmqm_pndn);
        p_CcParams->getCcParams.type &= ~GET_NIA_PNDN;
    }

    if ((p_CcParams->setCcParams.type & UPDATE_FMFP_PRC_WITH_ONE_RISC_ONLY) &&
        !(p_FmPort->requiredAction & UPDATE_FMFP_PRC_WITH_ONE_RISC_ONLY))
    {
        p_FmPort->requiredAction |= UPDATE_FMFP_PRC_WITH_ONE_RISC_ONLY;
        p_FmPort->orFmanCtrl = p_CcParams->setCcParams.orFmanCtrl;
    }

    if ((p_CcParams->setCcParams.type & UPDATE_NIA_PNEN) &&
        !(p_FmPort->requiredAction & UPDATE_NIA_PNEN))
    {
        p_FmPort->savedQmiPnen = p_CcParams->setCcParams.nia;
        p_FmPort->requiredAction |= UPDATE_NIA_PNEN;
    }
    else if (p_CcParams->setCcParams.type & UPDATE_NIA_PNEN)
    {
       if (p_FmPort->savedQmiPnen != p_CcParams->setCcParams.nia)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("PNEN was defined previously different"));
    }

    if ((p_CcParams->setCcParams.type & UPDATE_NIA_PNDN) &&
        !(p_FmPort->requiredAction & UPDATE_NIA_PNDN))
    {
        p_FmPort->savedNonRxQmiRegsPndn = p_CcParams->setCcParams.nia;
        p_FmPort->requiredAction |= UPDATE_NIA_PNDN;
    }
    else if (p_CcParams->setCcParams.type & UPDATE_NIA_PNDN)
    {
        if (p_FmPort->savedNonRxQmiRegsPndn != p_CcParams->setCcParams.nia)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("PNDN was defined previously different"));
    }

    if ((p_CcParams->setCcParams.type & UPDATE_NIA_FENE) &&
        (p_CcParams->setCcParams.overwrite ||
         !(p_FmPort->requiredAction & UPDATE_NIA_FENE)))
    {
        p_FmPort->savedBmiFene = p_CcParams->setCcParams.nia;
        p_FmPort->requiredAction |= UPDATE_NIA_FENE;
    }
    else if (p_CcParams->setCcParams.type & UPDATE_NIA_FENE)
    {
       if (p_FmPort->savedBmiFene != p_CcParams->setCcParams.nia)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("xFENE was defined previously different"));
    }

    if ((p_CcParams->setCcParams.type & UPDATE_NIA_FPNE) &&
        !(p_FmPort->requiredAction & UPDATE_NIA_FPNE))
    {
        p_FmPort->savedBmiFpne = p_CcParams->setCcParams.nia;
        p_FmPort->requiredAction |= UPDATE_NIA_FPNE;
    }
    else if (p_CcParams->setCcParams.type & UPDATE_NIA_FPNE)
    {
       if (p_FmPort->savedBmiFpne != p_CcParams->setCcParams.nia)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("xFPNE was defined previously different"));
    }

    if ((p_CcParams->setCcParams.type & UPDATE_NIA_CMNE) &&
        !(p_FmPort->requiredAction & UPDATE_NIA_CMNE))
    {
        p_FmPort->savedBmiCmne = p_CcParams->setCcParams.nia;
        p_FmPort->requiredAction |= UPDATE_NIA_CMNE;
    }
    else if (p_CcParams->setCcParams.type & UPDATE_NIA_CMNE)
    {
       if (p_FmPort->savedBmiCmne != p_CcParams->setCcParams.nia)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("xCMNE was defined previously different"));
    }

    if ((p_CcParams->setCcParams.type & UPDATE_PSO) &&
        !(p_FmPort->requiredAction & UPDATE_PSO))
    {
        /* get PCD registers pointers */
         switch (p_FmPort->portType)
         {
             case (e_FM_PORT_TYPE_RX_10G):
             case (e_FM_PORT_TYPE_RX):
                 p_BmiPrsStartOffset = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rpso;
                 break;
             case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
                 p_BmiPrsStartOffset = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_opso;
                 break;
             default:
                 RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid port type"));
         }
        /* set start parsing offset */
        tmpInt = (int)GET_UINT32(*p_BmiPrsStartOffset)+ p_CcParams->setCcParams.psoSize;
        if (tmpInt>0)
            WRITE_UINT32(*p_BmiPrsStartOffset, (uint32_t)tmpInt);

        p_FmPort->requiredAction |= UPDATE_PSO;
        p_FmPort->savedPrsStartOffset = p_CcParams->setCcParams.psoSize;

    }
    else if (p_CcParams->setCcParams.type & UPDATE_PSO)
    {
        if (p_FmPort->savedPrsStartOffset != p_CcParams->setCcParams.psoSize)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("parser start offset was defoned previousley different"));
    }

    return E_OK;
}
/*********************** End of inter-module routines ************************/


/****************************************/
/*       API Init unit functions        */
/****************************************/

t_Handle FM_PORT_Config(t_FmPortParams *p_FmPortParams)
{
    t_FmPort            *p_FmPort;
    uintptr_t           baseAddr = p_FmPortParams->baseAddr;
    uint32_t            tmpReg;

    /* Allocate FM structure */
    p_FmPort = (t_FmPort *) XX_Malloc(sizeof(t_FmPort));
    if (!p_FmPort)
    {
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("FM Port driver structure"));
        return NULL;
    }
    memset(p_FmPort, 0, sizeof(t_FmPort));

    /* Allocate the FM driver's parameters structure */
    p_FmPort->p_FmPortDriverParam = (t_FmPortDriverParam *)XX_Malloc(sizeof(t_FmPortDriverParam));
    if (!p_FmPort->p_FmPortDriverParam)
    {
        XX_Free(p_FmPort);
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("FM Port driver parameters"));
        return NULL;
    }
    memset(p_FmPort->p_FmPortDriverParam, 0, sizeof(t_FmPortDriverParam));

    /* Initialize FM port parameters which will be kept by the driver */
    p_FmPort->portType      = p_FmPortParams->portType;
    p_FmPort->portId        = p_FmPortParams->portId;
    p_FmPort->pcdEngines    = FM_PCD_NONE;
    p_FmPort->f_Exception   = p_FmPortParams->f_Exception;
    p_FmPort->h_App         = p_FmPortParams->h_App;
    p_FmPort->h_Fm          = p_FmPortParams->h_Fm;

    /* get FM revision */
    FM_GetRevision(p_FmPort->h_Fm, &p_FmPort->fmRevInfo);

    /* calculate global portId number */
    SW_PORT_ID_TO_HW_PORT_ID(p_FmPort->hardwarePortId, p_FmPort->portType, p_FmPortParams->portId);

    if (p_FmPort->fmRevInfo.majorRev >= 6)
    {
        if ((p_FmPort->portType == e_FM_PORT_TYPE_OH_HOST_COMMAND) &&
            (p_FmPortParams->portId != FM_OH_PORT_ID))
            DBG(WARNING,
                ("Port ID %d is recommended for HC port. Overwriting HW defaults to be suitable for HC.",
                 FM_OH_PORT_ID));

        if ((p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING) &&
            (p_FmPortParams->portId == FM_OH_PORT_ID))
            DBG(WARNING, ("Use non-zero portId for OP port due to insufficient resources on portId 0."));
    }

    /* Initialize FM port parameters for initialization phase only */
    p_FmPort->p_FmPortDriverParam->baseAddr                         = baseAddr;
    /* set memory map pointers */
    p_FmPort->p_FmPortQmiRegs     = (t_FmPortQmiRegs *)UINT_TO_PTR(baseAddr + QMI_PORT_REGS_OFFSET);
    p_FmPort->p_FmPortBmiRegs     = (u_FmPortBmiRegs *)UINT_TO_PTR(baseAddr + BMI_PORT_REGS_OFFSET);
    p_FmPort->p_FmPortPrsRegs     = (t_FmPortPrsRegs *)UINT_TO_PTR(baseAddr + PRS_PORT_REGS_OFFSET);

    p_FmPort->p_FmPortDriverParam->bufferPrefixContent.privDataSize = DEFAULT_PORT_bufferPrefixContent_privDataSize;
    p_FmPort->p_FmPortDriverParam->bufferPrefixContent.passPrsResult= DEFAULT_PORT_bufferPrefixContent_passPrsResult;
    p_FmPort->p_FmPortDriverParam->bufferPrefixContent.passTimeStamp= DEFAULT_PORT_bufferPrefixContent_passTimeStamp;
    p_FmPort->p_FmPortDriverParam->bufferPrefixContent.passAllOtherPCDInfo
                                                                    = DEFAULT_PORT_bufferPrefixContent_passTimeStamp;
    p_FmPort->p_FmPortDriverParam->bufferPrefixContent.dataAlign    = DEFAULT_PORT_bufferPrefixContent_dataAlign;
    p_FmPort->p_FmPortDriverParam->dmaSwapData                      = (e_FmDmaSwapOption)DEFAULT_PORT_dmaSwapData;
    p_FmPort->p_FmPortDriverParam->dmaIntContextCacheAttr           = (e_FmDmaCacheOption)DEFAULT_PORT_dmaIntContextCacheAttr;
    p_FmPort->p_FmPortDriverParam->dmaHeaderCacheAttr               = (e_FmDmaCacheOption)DEFAULT_PORT_dmaHeaderCacheAttr;
    p_FmPort->p_FmPortDriverParam->dmaScatterGatherCacheAttr        = (e_FmDmaCacheOption)DEFAULT_PORT_dmaScatterGatherCacheAttr;
    p_FmPort->p_FmPortDriverParam->dmaWriteOptimize                 = DEFAULT_PORT_dmaWriteOptimize;
    p_FmPort->p_FmPortDriverParam->liodnBase                        = p_FmPortParams->liodnBase;
    p_FmPort->p_FmPortDriverParam->cheksumLastBytesIgnore           = DEFAULT_PORT_cheksumLastBytesIgnore;
    p_FmPort->p_FmPortDriverParam->color                            = DEFAULT_PORT_color;

    p_FmPort->maxFrameLength                                        = DEFAULT_PORT_maxFrameLength;
    /* resource distribution. */
#ifdef FM_NO_GUARANTEED_RESET_VALUES
    if (1) /* if (p_FmPort->fmRevInfo.majorRev < 6) */
    {
        p_FmPort->fifoBufs.num                                      = DEFAULT_PORT_numOfFifoBufs(p_FmPort->portType)*BMI_FIFO_UNITS;
        p_FmPort->fifoBufs.extra                                    = DEFAULT_PORT_extraNumOfFifoBufs*BMI_FIFO_UNITS;
        p_FmPort->openDmas.num                                      = DEFAULT_PORT_numOfOpenDmas(p_FmPort->portType);
        p_FmPort->openDmas.extra                                    = DEFAULT_PORT_extraNumOfOpenDmas(p_FmPort->portType);
        p_FmPort->tasks.num                                         = DEFAULT_PORT_numOfTasks(p_FmPort->portType);
        p_FmPort->tasks.extra                                       = DEFAULT_PORT_extraNumOfTasks(p_FmPort->portType);
    }
    else
#endif /* FM_NO_GUARANTEED_RESET_VALUES */
    {
        if ((p_FmPort->portType == e_FM_PORT_TYPE_OH_HOST_COMMAND) &&
            (p_FmPortParams->portId != FM_OH_PORT_ID))
        {
            /* Overwrite HC defaults */
            p_FmPort->fifoBufs.num      = DEFAULT_PORT_numOfFifoBufs(p_FmPort->portType)*BMI_FIFO_UNITS;
            p_FmPort->fifoBufs.extra    = DEFAULT_PORT_extraNumOfFifoBufs*BMI_FIFO_UNITS;
            p_FmPort->openDmas.num      = DEFAULT_PORT_numOfOpenDmas(p_FmPort->portType);
            p_FmPort->openDmas.extra    = DEFAULT_PORT_extraNumOfOpenDmas(p_FmPort->portType);
            p_FmPort->tasks.num         = DEFAULT_PORT_numOfTasks(p_FmPort->portType);
            p_FmPort->tasks.extra       = DEFAULT_PORT_extraNumOfTasks(p_FmPort->portType);
        }
        else
        {
            p_FmPort->fifoBufs.num                                      = 0;
            p_FmPort->fifoBufs.extra                                    = 0;
            p_FmPort->openDmas.num                                      = 0;
            p_FmPort->openDmas.extra                                    = 0;
            p_FmPort->tasks.num                                         = 0;
            p_FmPort->tasks.extra                                       = 0;
        }
    }

    if (p_FmPort->portType == e_FM_PORT_TYPE_OH_HOST_COMMAND)
        p_FmPort->p_FmPortDriverParam->syncReq                      = DEFAULT_PORT_syncReqForHc;
    else
        p_FmPort->p_FmPortDriverParam->syncReq                      = DEFAULT_PORT_syncReq;

    /* Port type specific initialization: */
    if ((p_FmPort->portType != e_FM_PORT_TYPE_TX) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_TX_10G))
        p_FmPort->p_FmPortDriverParam->frmDiscardOverride           = DEFAULT_PORT_frmDiscardOverride;

    switch (p_FmPort->portType)
    {
    case (e_FM_PORT_TYPE_RX):
    case (e_FM_PORT_TYPE_RX_10G):
        /* Initialize FM port parameters for initialization phase only */
        p_FmPort->p_FmPortDriverParam->cutBytesFromEnd              = DEFAULT_PORT_cutBytesFromEnd;
        p_FmPort->p_FmPortDriverParam->enBufPoolDepletion           = FALSE;
        p_FmPort->p_FmPortDriverParam->frmDiscardOverride           = DEFAULT_PORT_frmDiscardOverride;
#ifdef FM_NO_GUARANTEED_RESET_VALUES
    if (1) /* if (p_FmPort->fmRevInfo.majorRev < 6) */
    {
        p_FmPort->p_FmPortDriverParam->rxFifoPriElevationLevel      = DEFAULT_PORT_rxFifoPriElevationLevel;
        p_FmPort->p_FmPortDriverParam->rxFifoThreshold              = DEFAULT_PORT_rxFifoThreshold;
    }
    else
#endif /* FM_NO_GUARANTEED_RESET_VALUES */
    {
        tmpReg = GET_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfp);
        p_FmPort->p_FmPortDriverParam->rxFifoPriElevationLevel      = (((tmpReg & BMI_RX_FIFO_PRI_ELEVATION_MASK) >> BMI_RX_FIFO_PRI_ELEVATION_SHIFT) + 1) * BMI_FIFO_UNITS ;
        p_FmPort->p_FmPortDriverParam->rxFifoThreshold              = (((tmpReg  & BMI_RX_FIFO_THRESHOLD_MASK) >> BMI_RX_FIFO_THRESHOLD_SHIFT) + 1) * BMI_FIFO_UNITS;
    }

        p_FmPort->p_FmPortDriverParam->bufMargins.endMargins        = DEFAULT_PORT_BufMargins_endMargins;
        p_FmPort->p_FmPortDriverParam->errorsToDiscard              = DEFAULT_PORT_errorsToDiscard;
        p_FmPort->p_FmPortDriverParam->forwardReuseIntContext       = DEFAULT_PORT_forwardIntContextReuse;
#if (DPAA_VERSION >= 11)
        p_FmPort->p_FmPortDriverParam->noScatherGather              = DEFAULT_PORT_noScatherGather;
#endif /* (DPAA_VERSION >= 11) */
        break;

    case (e_FM_PORT_TYPE_TX):
        p_FmPort->p_FmPortDriverParam->dontReleaseBuf               = FALSE;
#ifdef FM_WRONG_RESET_VALUES_ERRATA_FMAN_A005127
        tmpReg = 0x00001013;
        WRITE_UINT32(p_FmPort->p_FmPortBmiRegs->txPortBmiRegs.fmbm_tfp, tmpReg);
#endif /* FM_WRONG_RESET_VALUES_ERRATA_FMAN_A005127 */
    case (e_FM_PORT_TYPE_TX_10G):
#ifdef FM_NO_GUARANTEED_RESET_VALUES
        if (1) /* if (p_FmPort->fmRevInfo.majorRev < 6) */
        {
            p_FmPort->p_FmPortDriverParam->txFifoMinFillLevel       = DEFAULT_PORT_txFifoMinFillLevel;
            p_FmPort->fifoDeqPipelineDepth                          =
                (uint8_t)((p_FmPort->portType == e_FM_PORT_TYPE_TX) ?
                          DEFAULT_PORT_fifoDeqPipelineDepth_1G :
                          DEFAULT_PORT_fifoDeqPipelineDepth_10G);
            p_FmPort->p_FmPortDriverParam->txFifoLowComfLevel       = DEFAULT_PORT_txFifoLowComfLevel;
        }
        else
#endif /* FM_NO_GUARANTEED_RESET_VALUES */
        {
            tmpReg = GET_UINT32(p_FmPort->p_FmPortBmiRegs->txPortBmiRegs.fmbm_tfp);
            p_FmPort->p_FmPortDriverParam->txFifoMinFillLevel =
                ((tmpReg & BMI_TX_FIFO_MIN_FILL_MASK) >> BMI_TX_FIFO_MIN_FILL_SHIFT) * BMI_FIFO_UNITS ;
            p_FmPort->fifoDeqPipelineDepth =
                (uint8_t)(((tmpReg & BMI_FIFO_PIPELINE_DEPTH_MASK) >> BMI_FIFO_PIPELINE_DEPTH_SHIFT) + 1);
            p_FmPort->p_FmPortDriverParam->txFifoLowComfLevel =
                (((tmpReg & BMI_TX_LOW_COMF_MASK) >> BMI_TX_LOW_COMF_SHIFT) + 1) * BMI_FIFO_UNITS;
        }

        p_FmPort->p_FmPortDriverParam->deqType                      = DEFAULT_PORT_deqType;
        p_FmPort->p_FmPortDriverParam->deqPrefetchOption            = DEFAULT_PORT_deqPrefetchOption;
        p_FmPort->p_FmPortDriverParam->deqHighPriority              =
            (bool)((p_FmPort->portType == e_FM_PORT_TYPE_TX) ?
                   DEFAULT_PORT_deqHighPriority_1G :
                   DEFAULT_PORT_deqHighPriority_10G);
        p_FmPort->p_FmPortDriverParam->deqByteCnt                   =
            (uint16_t)((p_FmPort->portType == e_FM_PORT_TYPE_TX) ?
                       DEFAULT_PORT_deqByteCnt_1G :
                       DEFAULT_PORT_deqByteCnt_10G);
        break;
    case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
        p_FmPort->p_FmPortDriverParam->errorsToDiscard              = DEFAULT_PORT_errorsToDiscard;
#if (DPAA_VERSION >= 11)
        p_FmPort->p_FmPortDriverParam->noScatherGather              = DEFAULT_PORT_noScatherGather;
#endif /* (DPAA_VERSION >= 11) */
    case (e_FM_PORT_TYPE_OH_HOST_COMMAND):
        p_FmPort->p_FmPortDriverParam->deqPrefetchOption            = DEFAULT_PORT_deqPrefetchOption_HC;
        p_FmPort->p_FmPortDriverParam->deqHighPriority              = DEFAULT_PORT_deqHighPriority_1G;
        p_FmPort->p_FmPortDriverParam->deqType                      = DEFAULT_PORT_deqType;
        p_FmPort->p_FmPortDriverParam->deqByteCnt                   = DEFAULT_PORT_deqByteCnt_1G;

#ifdef FM_NO_GUARANTEED_RESET_VALUES
    if (1) /* if (p_FmPort->fmRevInfo.majorRev < 6) */
        p_FmPort->fifoDeqPipelineDepth                              = DEFAULT_PORT_fifoDeqPipelineDepth_OH;
    else
#endif /* FM_NO_GUARANTEED_RESET_VALUES */
    {
        tmpReg = GET_UINT32(p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofp);
        p_FmPort->fifoDeqPipelineDepth =
            (uint8_t)(((tmpReg & BMI_FIFO_PIPELINE_DEPTH_MASK) >> BMI_FIFO_PIPELINE_DEPTH_SHIFT) + 1);
        if ((p_FmPort->portType == e_FM_PORT_TYPE_OH_HOST_COMMAND) &&
            (p_FmPortParams->portId != FM_OH_PORT_ID))
        {
            /* Overwrite HC defaults */
            p_FmPort->fifoDeqPipelineDepth = DEFAULT_PORT_fifoDeqPipelineDepth_OH;
        }
    }

#ifndef FM_FRAME_END_PARAMS_FOR_OP
        if (p_FmPort->fmRevInfo.majorRev < 6)
            p_FmPort->p_FmPortDriverParam->cheksumLastBytesIgnore   = DEFAULT_notSupported;
#endif /* !FM_FRAME_END_PARAMS_FOR_OP */

#ifndef FM_DEQ_PIPELINE_PARAMS_FOR_OP
    if (!((p_FmPort->fmRevInfo.majorRev == 4) ||
          (p_FmPort->fmRevInfo.majorRev >= 6)))
            p_FmPort->fifoDeqPipelineDepth                          = DEFAULT_notSupported;
#endif /* !FM_DEQ_PIPELINE_PARAMS_FOR_OP */
        break;

    default:
        XX_Free(p_FmPort->p_FmPortDriverParam);
        XX_Free(p_FmPort);
        REPORT_ERROR(MAJOR, E_INVALID_STATE, ("Invalid port type"));
        return NULL;
    }
#ifdef FM_QMI_NO_DEQ_OPTIONS_SUPPORT
    if (p_FmPort->fmRevInfo.majorRev == 4)
        p_FmPort->p_FmPortDriverParam->deqPrefetchOption = (e_FmPortDeqPrefetchOption)DEFAULT_notSupported;
#endif /* FM_QMI_NO_DEQ_OPTIONS_SUPPORT */

    p_FmPort->imEn = p_FmPortParams->independentModeEnable;

    if (p_FmPort->imEn)
    {
        if ((p_FmPort->portType == e_FM_PORT_TYPE_TX) ||
            (p_FmPort->portType == e_FM_PORT_TYPE_TX_10G))
            p_FmPort->fifoDeqPipelineDepth = DEFAULT_PORT_fifoDeqPipelineDepth_IM;
        FmPortConfigIM(p_FmPort, p_FmPortParams);
    }
    else
    {
        switch (p_FmPort->portType)
        {
        case (e_FM_PORT_TYPE_RX):
        case (e_FM_PORT_TYPE_RX_10G):
            /* Initialize FM port parameters for initialization phase only */
            memcpy(&p_FmPort->p_FmPortDriverParam->extBufPools,
                   &p_FmPortParams->specificParams.rxParams.extBufPools,
                   sizeof(t_FmExtPools));
            p_FmPort->p_FmPortDriverParam->errFqid                      = p_FmPortParams->specificParams.rxParams.errFqid;
            p_FmPort->p_FmPortDriverParam->dfltFqid                     = p_FmPortParams->specificParams.rxParams.dfltFqid;
            p_FmPort->p_FmPortDriverParam->liodnOffset                  = p_FmPortParams->specificParams.rxParams.liodnOffset;
            break;
        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
        case (e_FM_PORT_TYPE_TX):
        case (e_FM_PORT_TYPE_TX_10G):
        case (e_FM_PORT_TYPE_OH_HOST_COMMAND):
            p_FmPort->p_FmPortDriverParam->errFqid                      = p_FmPortParams->specificParams.nonRxParams.errFqid;
            p_FmPort->p_FmPortDriverParam->deqSubPortal                 =
                (uint8_t)(p_FmPortParams->specificParams.nonRxParams.qmChannel & QMI_DEQ_CFG_SUBPORTAL_MASK);
            p_FmPort->p_FmPortDriverParam->dfltFqid                     = p_FmPortParams->specificParams.nonRxParams.dfltFqid;
            break;
        default:
            XX_Free(p_FmPort->p_FmPortDriverParam);
            XX_Free(p_FmPort);
            REPORT_ERROR(MAJOR, E_INVALID_STATE, ("Invalid port type"));
            return NULL;
        }
    }

    memset(p_FmPort->name, 0, (sizeof(char)) * MODULE_NAME_SIZE);
    if (Sprint (p_FmPort->name, "FM-%d-port-%s-%d",
               FmGetId(p_FmPort->h_Fm),
               ((p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING ||
                 (p_FmPort->portType == e_FM_PORT_TYPE_OH_HOST_COMMAND)) ?
                "OH" : (p_FmPort->portType == e_FM_PORT_TYPE_RX ?
                        "1g-RX" : (p_FmPort->portType == e_FM_PORT_TYPE_TX ?
                                   "1g-TX" : (p_FmPort->portType == e_FM_PORT_TYPE_RX_10G ?
                                              "10g-RX" : "10g-TX")))),
               p_FmPort->portId) == 0)
    {
        XX_Free(p_FmPort->p_FmPortDriverParam);
        XX_Free(p_FmPort);
        REPORT_ERROR(MAJOR, E_INVALID_STATE, ("Sprint failed"));
        return NULL;
    }

    p_FmPort->h_Spinlock = XX_InitSpinlock();
    if (!p_FmPort->h_Spinlock)
    {
        XX_Free(p_FmPort->p_FmPortDriverParam);
        XX_Free(p_FmPort);
        REPORT_ERROR(MAJOR, E_INVALID_STATE, ("Sprint failed"));
        return NULL;
    }

    return p_FmPort;
}

/**************************************************************************//**
 @Function      FM_PORT_Init

 @Description   Initializes the FM module

 @Param[in]     h_FmPort - FM module descriptor

 @Return        E_OK on success; Error code otherwise.
*//***************************************************************************/
t_Error FM_PORT_Init(t_Handle h_FmPort)
{
    t_FmPort                        *p_FmPort = (t_FmPort*)h_FmPort;
    t_FmPortDriverParam             *p_Params;
    t_Error                         err = E_OK;
    t_FmInterModulePortInitParams   fmParams;

    SANITY_CHECK_RETURN_ERROR(h_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    err = FmSpBuildBufferStructure(&p_FmPort->p_FmPortDriverParam->intContext,
                                   &p_FmPort->p_FmPortDriverParam->bufferPrefixContent,
                                   &p_FmPort->p_FmPortDriverParam->bufMargins,
                                   &p_FmPort->bufferOffsets,
                                   &p_FmPort->internalBufferOffset);
    if (err != E_OK)
        RETURN_ERROR(MAJOR, err, NO_MSG);
#ifdef FM_HEAVY_TRAFFIC_HANG_ERRATA_FMAN_A005669
    if ((p_FmPort->p_FmPortDriverParam->bcbWorkaround) &&
           (p_FmPort->portType == e_FM_PORT_TYPE_RX))
    {
        p_FmPort->p_FmPortDriverParam->errorsToDiscard |= FM_PORT_FRM_ERR_PHYSICAL;
        if (!p_FmPort->fifoBufs.num)
            p_FmPort->fifoBufs.num = DEFAULT_PORT_numOfFifoBufs(p_FmPort->portType)*BMI_FIFO_UNITS;
        p_FmPort->fifoBufs.num += 4*KILOBYTE;
    }
#endif /* FM_HEAVY_TRAFFIC_HANG_ERRATA_FMAN_A005669 */

    CHECK_INIT_PARAMETERS(p_FmPort, CheckInitParameters);

    p_Params = p_FmPort->p_FmPortDriverParam;

    if  ((p_FmPort->portType == e_FM_PORT_TYPE_RX_10G) ||
         (p_FmPort->portType == e_FM_PORT_TYPE_RX))
        if (!p_FmPort->imEn)
        {
            /* Call the external Buffer routine which also checks fifo
             size and updates it if necessary */
            /* define external buffer pools and pool depletion*/
            err = SetExtBufferPools(p_FmPort);
            if (err)
                RETURN_ERROR(MAJOR, err, NO_MSG);
        }

    /************************************************************/
    /* Call FM module routine for communicating parameters      */
    /************************************************************/
    memset(&fmParams, 0, sizeof(fmParams));
    fmParams.hardwarePortId     = p_FmPort->hardwarePortId;
    fmParams.portType           = (e_FmPortType)p_FmPort->portType;
    fmParams.numOfTasks         = (uint8_t)p_FmPort->tasks.num;
    fmParams.numOfExtraTasks    = (uint8_t)p_FmPort->tasks.extra;
    fmParams.numOfOpenDmas      = (uint8_t)p_FmPort->openDmas.num;
    fmParams.numOfExtraOpenDmas = (uint8_t)p_FmPort->openDmas.extra;
    if (p_FmPort->fifoBufs.num)
    {
        err = VerifySizeOfFifo(p_FmPort);
        if (err)
            RETURN_ERROR(MAJOR, err, NO_MSG);
    }
    fmParams.sizeOfFifo         = p_FmPort->fifoBufs.num;
    fmParams.extraSizeOfFifo    = p_FmPort->fifoBufs.extra;
    fmParams.independentMode    = p_FmPort->imEn;
    fmParams.liodnOffset        = p_Params->liodnOffset;
    fmParams.liodnBase          = p_Params->liodnBase;
    fmParams.deqPipelineDepth   = p_FmPort->fifoDeqPipelineDepth;
    fmParams.maxFrameLength     = p_FmPort->maxFrameLength;
#ifndef FM_DEQ_PIPELINE_PARAMS_FOR_OP
    if ((p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING) ||
        (p_FmPort->portType == e_FM_PORT_TYPE_OH_HOST_COMMAND))
    {
        if (!((p_FmPort->fmRevInfo.majorRev == 4) ||
                (p_FmPort->fmRevInfo.majorRev >= 6)))
            /* HC ports do not have fifoDeqPipelineDepth, but it is needed only
             * for deq threshold calculation.
             */
            fmParams.deqPipelineDepth = 2;
    }
#endif /* !FM_DEQ_PIPELINE_PARAMS_FOR_OP */


    err = FmGetSetPortParams(p_FmPort->h_Fm, &fmParams);
    if (err)
        RETURN_ERROR(MAJOR, err, NO_MSG);

    /* get params for use in init */
    p_FmPort->fmMuramPhysBaseAddr =
        (uint64_t)((uint64_t)(fmParams.fmMuramPhysBaseAddr.low) |
                   ((uint64_t)(fmParams.fmMuramPhysBaseAddr.high) << 32));
    p_FmPort->h_FmMuram = FmGetMuramHandle(p_FmPort->h_Fm);

#ifndef FM_NO_GUARANTEED_RESET_VALUES
    if (p_FmPort->fmRevInfo.majorRev >= 6)
    {
        p_FmPort->tasks.num = fmParams.numOfTasks;
        p_FmPort->tasks.extra = fmParams.numOfExtraTasks;
        p_FmPort->openDmas.num = fmParams.numOfOpenDmas;
        p_FmPort->openDmas.extra = fmParams.numOfExtraOpenDmas;
        p_FmPort->fifoBufs.num = fmParams.sizeOfFifo;
        p_FmPort->fifoBufs.extra = fmParams.extraSizeOfFifo;
    }
#endif /* FM_NO_GUARANTEED_RESET_VALUES */

    /**********************/
    /* Init BMI Registers */
    /**********************/
    switch (p_FmPort->portType)
    {
        case (e_FM_PORT_TYPE_RX_10G):
        case (e_FM_PORT_TYPE_RX):
            err = BmiRxPortInit(p_FmPort);
            if (err)
                RETURN_ERROR(MAJOR, err, NO_MSG);
            break;
        case (e_FM_PORT_TYPE_TX_10G):
        case (e_FM_PORT_TYPE_TX):
            err = BmiTxPortInit(p_FmPort);
            if (err)
                RETURN_ERROR(MAJOR, err, NO_MSG);
            break;
        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
        case (e_FM_PORT_TYPE_OH_HOST_COMMAND):
            err = BmiOhPortInit(p_FmPort);
            if (err)
                RETURN_ERROR(MAJOR, err, NO_MSG);
            break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);
    }

    /**********************/
    /* Init QMI Registers */
    /**********************/
    if (!p_FmPort->imEn && ((err = QmiInit(p_FmPort)) != E_OK))
        RETURN_ERROR(MAJOR, err, NO_MSG);

    if (p_FmPort->imEn && ((err = FmPortImInit(p_FmPort)) != E_OK))
        RETURN_ERROR(MAJOR, err, NO_MSG);

    FmPortDriverParamFree(p_FmPort);

#if (DPAA_VERSION >= 11)
    if ((p_FmPort->portType == e_FM_PORT_TYPE_RX_10G) ||
        (p_FmPort->portType == e_FM_PORT_TYPE_RX) ||
        (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING))
    {
        t_FmPcdCtrlParamsPage   *p_ParamsPage;

        FmPortSetGprFunc(p_FmPort, e_FM_PORT_GPR_MURAM_PAGE, (void**)&p_ParamsPage);
        ASSERT_COND(p_ParamsPage);

        WRITE_UINT32(p_ParamsPage->misc, FM_CTL_PARAMS_PAGE_ALWAYS_ON);
#ifdef FM_OP_NO_VSP_NO_RELEASE_ERRATA_FMAN_A006675
        if (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
        {
            WRITE_UINT32(p_ParamsPage->misc,
                         (GET_UINT32(p_ParamsPage->misc) | FM_CTL_PARAMS_PAGE_OP_FIX_EN));
            WRITE_UINT32(p_ParamsPage->discardMask,
                         GET_UINT32(p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofsdm));
        }
#endif /* FM_OP_NO_VSP_NO_RELEASE_ERRATA_FMAN_A006675 */
#ifdef FM_ERROR_VSP_NO_MATCH_SW006
        if (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
            WRITE_UINT32(p_ParamsPage->errorsDiscardMask,
                         (GET_UINT32(p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofsdm) |
                          GET_UINT32(p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofsem)));
        else
            WRITE_UINT32(p_ParamsPage->errorsDiscardMask,
                         (GET_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfsdm) |
                          GET_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfsem)));
#endif /* FM_ERROR_VSP_NO_MATCH_SW006 */
    }
#endif /* (DPAA_VERSION >= 11) */
 
    return E_OK;
}

/**************************************************************************//**
 @Function      FM_PORT_Free

 @Description   Frees all resources that were assigned to FM module.

                Calling this routine invalidates the descriptor.

 @Param[in]     h_FmPort - FM module descriptor

 @Return        E_OK on success; Error code otherwise.
*//***************************************************************************/
t_Error FM_PORT_Free(t_Handle h_FmPort)
{
    t_FmPort                            *p_FmPort = (t_FmPort*)h_FmPort;
    t_FmInterModulePortFreeParams       fmParams;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);

    if (p_FmPort->pcdEngines)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Trying to free a port with PCD. FM_PORT_DeletePCD must be called first."));

    if (p_FmPort->enabled)
    {
        if (FM_PORT_Disable(p_FmPort) != E_OK)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("FM_PORT_Disable FAILED"));
    }

    if (p_FmPort->imEn)
        FmPortImFree(p_FmPort);

    FmPortDriverParamFree(p_FmPort);

    fmParams.hardwarePortId = p_FmPort->hardwarePortId;
    fmParams.portType = (e_FmPortType)p_FmPort->portType;
    fmParams.deqPipelineDepth = p_FmPort->fifoDeqPipelineDepth;

    FmFreePortParams(p_FmPort->h_Fm, &fmParams);

#if (DPAA_VERSION >= 11)
    if (FmVSPFreeForPort(p_FmPort->h_Fm,
                        p_FmPort->portType,
                        p_FmPort->portId) != E_OK)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("VSP free of port FAILED"));

    if (p_FmPort->p_ParamsPage)
        FM_MURAM_FreeMem(p_FmPort->h_FmMuram, p_FmPort->p_ParamsPage);
#endif /* (DPAA_VERSION >= 11) */

    if (p_FmPort->h_Spinlock)
        XX_FreeSpinlock(p_FmPort->h_Spinlock);

    XX_Free(p_FmPort);

    return E_OK;
}


/*************************************************/
/*       API Advanced Init unit functions        */
/*************************************************/

t_Error FM_PORT_ConfigNumOfOpenDmas(t_Handle h_FmPort, t_FmPortRsrc *p_OpenDmas)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    p_FmPort->p_FmPortDriverParam->setNumOfOpenDmas = TRUE;
    memcpy(&p_FmPort->openDmas, p_OpenDmas, sizeof(t_FmPortRsrc));

    return E_OK;
}

t_Error FM_PORT_ConfigNumOfTasks(t_Handle h_FmPort, t_FmPortRsrc *p_NumOfTasks)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    memcpy(&p_FmPort->tasks, p_NumOfTasks, sizeof(t_FmPortRsrc));
    p_FmPort->p_FmPortDriverParam->setNumOfTasks = TRUE;
    return E_OK;
}

t_Error FM_PORT_ConfigSizeOfFifo(t_Handle h_FmPort, t_FmPortRsrc *p_SizeOfFifo)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    p_FmPort->p_FmPortDriverParam->setSizeOfFifo = TRUE;
    memcpy(&p_FmPort->fifoBufs, p_SizeOfFifo, sizeof(t_FmPortRsrc));

    return E_OK;
}

t_Error FM_PORT_ConfigDeqHighPriority(t_Handle h_FmPort, bool highPri)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);
    if ((p_FmPort->portType == e_FM_PORT_TYPE_RX_10G) || (p_FmPort->portType == e_FM_PORT_TYPE_RX))
        RETURN_ERROR(MAJOR, E_NO_MEMORY, ("not available for Rx ports"));

    p_FmPort->p_FmPortDriverParam->deqHighPriority = highPri;

    return E_OK;
}

t_Error FM_PORT_ConfigDeqType(t_Handle h_FmPort, e_FmPortDeqType deqType)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);
    if ((p_FmPort->portType == e_FM_PORT_TYPE_RX_10G) || (p_FmPort->portType == e_FM_PORT_TYPE_RX))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("not available for Rx ports"));

    p_FmPort->p_FmPortDriverParam->deqType = deqType;

    return E_OK;
}

t_Error FM_PORT_ConfigDeqPrefetchOption(t_Handle h_FmPort, e_FmPortDeqPrefetchOption deqPrefetchOption)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);
    if ((p_FmPort->portType == e_FM_PORT_TYPE_RX_10G) || (p_FmPort->portType == e_FM_PORT_TYPE_RX))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("not available for Rx ports"));
    p_FmPort->p_FmPortDriverParam->deqPrefetchOption = deqPrefetchOption;
    return E_OK;
}

t_Error FM_PORT_ConfigBackupPools(t_Handle h_FmPort, t_FmBackupBmPools *p_BackupBmPools)
{
    t_FmPort            *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);
    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) && (p_FmPort->portType != e_FM_PORT_TYPE_RX))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx ports only"));

    p_FmPort->p_FmPortDriverParam->p_BackupBmPools = (t_FmBackupBmPools *)XX_Malloc(sizeof(t_FmBackupBmPools));
    if (!p_FmPort->p_FmPortDriverParam->p_BackupBmPools)
        RETURN_ERROR(MAJOR, E_NO_MEMORY, ("p_BackupBmPools allocation failed"));
    memcpy(p_FmPort->p_FmPortDriverParam->p_BackupBmPools, p_BackupBmPools, sizeof(t_FmBackupBmPools));

    return E_OK;
}

t_Error FM_PORT_ConfigDeqByteCnt(t_Handle h_FmPort, uint16_t deqByteCnt)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);
    if ((p_FmPort->portType == e_FM_PORT_TYPE_RX_10G) || (p_FmPort->portType == e_FM_PORT_TYPE_RX))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("not available for Rx ports"));

    p_FmPort->p_FmPortDriverParam->deqByteCnt = deqByteCnt;

    return E_OK;
}

t_Error FM_PORT_ConfigBufferPrefixContent(t_Handle h_FmPort, t_FmBufferPrefixContent *p_FmBufferPrefixContent)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    memcpy(&p_FmPort->p_FmPortDriverParam->bufferPrefixContent, p_FmBufferPrefixContent, sizeof(t_FmBufferPrefixContent));
    /* if dataAlign was not initialized by user, we return to driver's deafult */
    if (!p_FmPort->p_FmPortDriverParam->bufferPrefixContent.dataAlign)
        p_FmPort->p_FmPortDriverParam->bufferPrefixContent.dataAlign = DEFAULT_PORT_bufferPrefixContent_dataAlign;

    return E_OK;
}

t_Error FM_PORT_ConfigCheksumLastBytesIgnore(t_Handle h_FmPort, uint8_t cheksumLastBytesIgnore)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    p_FmPort->p_FmPortDriverParam->cheksumLastBytesIgnore = cheksumLastBytesIgnore;

    return E_OK;
}

t_Error FM_PORT_ConfigCutBytesFromEnd(t_Handle h_FmPort, uint8_t cutBytesFromEnd)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);
    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) && (p_FmPort->portType != e_FM_PORT_TYPE_RX))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx ports only"));

    p_FmPort->p_FmPortDriverParam->cutBytesFromEnd = cutBytesFromEnd;

    return E_OK;
}

t_Error FM_PORT_ConfigPoolDepletion(t_Handle h_FmPort, t_FmBufPoolDepletion *p_BufPoolDepletion)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);
    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) && (p_FmPort->portType != e_FM_PORT_TYPE_RX))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx ports only"));

    p_FmPort->p_FmPortDriverParam->enBufPoolDepletion = TRUE;
    memcpy(&p_FmPort->p_FmPortDriverParam->bufPoolDepletion, p_BufPoolDepletion, sizeof(t_FmBufPoolDepletion));

    return E_OK;
}

t_Error FM_PORT_ConfigObservedPoolDepletion(t_Handle h_FmPort, t_FmPortObservedBufPoolDepletion *p_FmPortObservedBufPoolDepletion)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);
    if (p_FmPort->portType != e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for OP ports only"));

    p_FmPort->p_FmPortDriverParam->enBufPoolDepletion = TRUE;
    memcpy(&p_FmPort->p_FmPortDriverParam->bufPoolDepletion,
           &p_FmPortObservedBufPoolDepletion->poolDepletionParams,
           sizeof(t_FmBufPoolDepletion));
    memcpy(&p_FmPort->p_FmPortDriverParam->extBufPools,
           &p_FmPortObservedBufPoolDepletion->poolsParams,
           sizeof(t_FmExtPools));

    return E_OK;
}

t_Error FM_PORT_ConfigExtBufPools(t_Handle h_FmPort, t_FmExtPools *p_FmExtPools)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    if (p_FmPort->portType != e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for OP ports only"));

    memcpy(&p_FmPort->p_FmPortDriverParam->extBufPools, p_FmExtPools, sizeof(t_FmExtPools));

    return E_OK;
}

t_Error FM_PORT_ConfigDontReleaseTxBufToBM(t_Handle h_FmPort)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);
    if ((p_FmPort->portType != e_FM_PORT_TYPE_TX_10G) && (p_FmPort->portType != e_FM_PORT_TYPE_TX))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Tx ports only"));

    p_FmPort->p_FmPortDriverParam->dontReleaseBuf = TRUE;

    return E_OK;
}

t_Error FM_PORT_ConfigDfltColor(t_Handle h_FmPort, e_FmPortColor color)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);
    p_FmPort->p_FmPortDriverParam->color = color;

    return E_OK;
}

t_Error FM_PORT_ConfigSyncReq(t_Handle h_FmPort, bool syncReq)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    if ((p_FmPort->portType == e_FM_PORT_TYPE_TX_10G) && (p_FmPort->portType == e_FM_PORT_TYPE_TX))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("Not available for Tx ports"));

    p_FmPort->p_FmPortDriverParam->syncReq = syncReq;

    return E_OK;
}


t_Error FM_PORT_ConfigFrmDiscardOverride(t_Handle h_FmPort, bool override)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);
    if ((p_FmPort->portType == e_FM_PORT_TYPE_TX_10G) && (p_FmPort->portType == e_FM_PORT_TYPE_TX))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("not available for Tx ports"));

    p_FmPort->p_FmPortDriverParam->frmDiscardOverride = override;

    return E_OK;
}

t_Error FM_PORT_ConfigErrorsToDiscard(t_Handle h_FmPort, fmPortFrameErrSelect_t errs)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);
    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) && (p_FmPort->portType != e_FM_PORT_TYPE_RX) &&
                                                            (p_FmPort->portType != e_FM_PORT_TYPE_OH_OFFLINE_PARSING))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx and offline parsing ports only"));

    p_FmPort->p_FmPortDriverParam->errorsToDiscard = errs;

    return E_OK;
}

t_Error FM_PORT_ConfigDmaSwapData(t_Handle h_FmPort, e_FmDmaSwapOption swapData)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    p_FmPort->p_FmPortDriverParam->dmaSwapData = swapData;

    return E_OK;
}

t_Error FM_PORT_ConfigDmaIcCacheAttr(t_Handle h_FmPort, e_FmDmaCacheOption intContextCacheAttr)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    p_FmPort->p_FmPortDriverParam->dmaIntContextCacheAttr = intContextCacheAttr;

    return E_OK;
}

t_Error FM_PORT_ConfigDmaHdrAttr(t_Handle h_FmPort, e_FmDmaCacheOption headerCacheAttr)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    p_FmPort->p_FmPortDriverParam->dmaHeaderCacheAttr = headerCacheAttr;

    return E_OK;
}

t_Error FM_PORT_ConfigDmaScatterGatherAttr(t_Handle h_FmPort, e_FmDmaCacheOption scatterGatherCacheAttr)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    p_FmPort->p_FmPortDriverParam->dmaScatterGatherCacheAttr = scatterGatherCacheAttr;

    return E_OK;
}

t_Error FM_PORT_ConfigDmaWriteOptimize(t_Handle h_FmPort, bool optimize)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    if ((p_FmPort->portType == e_FM_PORT_TYPE_TX_10G) || (p_FmPort->portType == e_FM_PORT_TYPE_TX))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("Not available for Tx ports"));

    p_FmPort->p_FmPortDriverParam->dmaWriteOptimize = optimize;

    return E_OK;
}

#if (DPAA_VERSION >= 11)
t_Error FM_PORT_ConfigNoScatherGather(t_Handle h_FmPort, bool noScatherGather)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    UNUSED(noScatherGather);
    UNUSED(p_FmPort);

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    p_FmPort->p_FmPortDriverParam->noScatherGather = noScatherGather;

    return E_OK;
}
#endif /* (DPAA_VERSION >= 11) */

t_Error FM_PORT_ConfigForwardReuseIntContext(t_Handle h_FmPort, bool forwardReuse)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) && (p_FmPort->portType != e_FM_PORT_TYPE_RX))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx ports only"));

    p_FmPort->p_FmPortDriverParam->forwardReuseIntContext = forwardReuse;

    return E_OK;
}

t_Error FM_PORT_ConfigMaxFrameLength(t_Handle h_FmPort, uint16_t length)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    p_FmPort->maxFrameLength = length;

    return E_OK;
}

#ifdef FM_HEAVY_TRAFFIC_HANG_ERRATA_FMAN_A005669
t_Error FM_PORT_ConfigBCBWorkaround(t_Handle h_FmPort)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    p_FmPort->p_FmPortDriverParam->bcbWorkaround = TRUE;

    return E_OK;
}
#endif /* FM_HEAVY_TRAFFIC_HANG_ERRATA_FMAN_A005669 */

/****************************************************/
/*       Hidden-DEBUG Only API                      */
/****************************************************/

t_Error FM_PORT_ConfigTxFifoMinFillLevel(t_Handle h_FmPort, uint32_t minFillLevel)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);
    if ((p_FmPort->portType != e_FM_PORT_TYPE_TX_10G) && (p_FmPort->portType != e_FM_PORT_TYPE_TX))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Tx ports only"));

    p_FmPort->p_FmPortDriverParam->txFifoMinFillLevel = minFillLevel;

    return E_OK;
}

t_Error FM_PORT_ConfigFifoDeqPipelineDepth(t_Handle h_FmPort, uint8_t deqPipelineDepth)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    if ((p_FmPort->portType == e_FM_PORT_TYPE_RX_10G) || (p_FmPort->portType == e_FM_PORT_TYPE_RX))
            RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("Not available for Rx ports"));

    if (p_FmPort->imEn)
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("Not available for IM ports!"));

    p_FmPort->fifoDeqPipelineDepth = deqPipelineDepth;

    return E_OK;
}

t_Error FM_PORT_ConfigTxFifoLowComfLevel(t_Handle h_FmPort, uint32_t fifoLowComfLevel)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);
    if ((p_FmPort->portType != e_FM_PORT_TYPE_TX_10G) && (p_FmPort->portType != e_FM_PORT_TYPE_TX))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Tx ports only"));

    p_FmPort->p_FmPortDriverParam->txFifoLowComfLevel = fifoLowComfLevel;

    return E_OK;
}

t_Error FM_PORT_ConfigRxFifoThreshold(t_Handle h_FmPort, uint32_t fifoThreshold)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);
    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) && (p_FmPort->portType != e_FM_PORT_TYPE_RX))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx ports only"));

    p_FmPort->p_FmPortDriverParam->rxFifoThreshold = fifoThreshold;

    return E_OK;
}

t_Error FM_PORT_ConfigRxFifoPriElevationLevel(t_Handle h_FmPort, uint32_t priElevationLevel)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);
    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) && (p_FmPort->portType != e_FM_PORT_TYPE_RX))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx ports only"));

    p_FmPort->p_FmPortDriverParam->rxFifoPriElevationLevel = priElevationLevel;

    return E_OK;
}
/****************************************************/
/*       API Run-time Control unit functions        */
/****************************************************/

t_Error FM_PORT_SetNumOfOpenDmas(t_Handle h_FmPort, t_FmPortRsrc *p_NumOfOpenDmas)
{
    t_FmPort    *p_FmPort = (t_FmPort*)h_FmPort;
    t_Error     err;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    if ((!p_NumOfOpenDmas->num) || (p_NumOfOpenDmas->num > MAX_NUM_OF_DMAS))
         RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("openDmas-num can't be larger than %d", MAX_NUM_OF_DMAS));
    if (p_NumOfOpenDmas->extra > MAX_NUM_OF_EXTRA_DMAS)
         RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("openDmas-extra can't be larger than %d", MAX_NUM_OF_EXTRA_DMAS));
    err = FmSetNumOfOpenDmas(p_FmPort->h_Fm, p_FmPort->hardwarePortId, (uint8_t*)&p_NumOfOpenDmas->num, (uint8_t*)&p_NumOfOpenDmas->extra, FALSE);
    if (err)
        RETURN_ERROR(MAJOR, err, NO_MSG);

    memcpy(&p_FmPort->openDmas, p_NumOfOpenDmas, sizeof(t_FmPortRsrc));

    return E_OK;
}

t_Error FM_PORT_SetNumOfTasks(t_Handle h_FmPort, t_FmPortRsrc *p_NumOfTasks)
{
    t_FmPort    *p_FmPort = (t_FmPort*)h_FmPort;
    t_Error     err;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    /* only driver uses host command port, so ASSERT rather than  RETURN_ERROR */
    ASSERT_COND(p_FmPort->portType != e_FM_PORT_TYPE_OH_HOST_COMMAND);

    if ((!p_NumOfTasks->num) || (p_NumOfTasks->num > MAX_NUM_OF_TASKS))
         RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("NumOfTasks-num can't be larger than %d", MAX_NUM_OF_TASKS));
    if (p_NumOfTasks->extra > MAX_NUM_OF_EXTRA_TASKS)
         RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("NumOfTasks-extra can't be larger than %d", MAX_NUM_OF_EXTRA_TASKS));

    err = FmSetNumOfTasks(p_FmPort->h_Fm, p_FmPort->hardwarePortId, (uint8_t*)&p_NumOfTasks->num, (uint8_t*)&p_NumOfTasks->extra, FALSE);
    if (err)
        RETURN_ERROR(MAJOR, err, NO_MSG);

    /* update driver's struct */
    memcpy(&p_FmPort->tasks, p_NumOfTasks, sizeof(t_FmPortRsrc));
    return E_OK;
}

t_Error FM_PORT_SetSizeOfFifo(t_Handle h_FmPort, t_FmPortRsrc *p_SizeOfFifo)
{
    t_FmPort                            *p_FmPort = (t_FmPort*)h_FmPort;
    t_Error                             err;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    if (!p_SizeOfFifo->num || (p_SizeOfFifo->num > BMI_MAX_FIFO_SIZE))
         RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("SizeOfFifo-num has to be in the range of 256 - %d", BMI_MAX_FIFO_SIZE));
    if (p_SizeOfFifo->num % BMI_FIFO_UNITS)
         RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("SizeOfFifo-num has to be divisible by %d", BMI_FIFO_UNITS));
    if ((p_FmPort->portType == e_FM_PORT_TYPE_RX) || (p_FmPort->portType == e_FM_PORT_TYPE_RX_10G))
    {
        /* extra FIFO size (allowed only to Rx ports) */
         if (p_SizeOfFifo->extra % BMI_FIFO_UNITS)
              RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("SizeOfFifo-extra has to be divisible by %d", BMI_FIFO_UNITS));
    }
    else
        if (p_SizeOfFifo->extra)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, (" No SizeOfFifo-extra for non Rx ports"));

    memcpy(&p_FmPort->fifoBufs, p_SizeOfFifo, sizeof(t_FmPortRsrc));

    /* we do not change user's parameter */
    err = VerifySizeOfFifo(p_FmPort);
    if (err)
        RETURN_ERROR(MAJOR, err, NO_MSG);

    err = FmSetSizeOfFifo(p_FmPort->h_Fm,
                          p_FmPort->hardwarePortId,
                          &p_SizeOfFifo->num,
                          &p_SizeOfFifo->extra,
                          FALSE);
    if (err)
        RETURN_ERROR(MAJOR, err, NO_MSG);

    return E_OK;
}

uint32_t FM_PORT_GetBufferDataOffset(t_Handle h_FmPort)
{
    t_FmPort                    *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_VALUE(p_FmPort, E_INVALID_HANDLE, 0);
    SANITY_CHECK_RETURN_VALUE(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE, 0);

    return p_FmPort->bufferOffsets.dataOffset;
}

uint8_t * FM_PORT_GetBufferICInfo(t_Handle h_FmPort, char *p_Data)
{
    t_FmPort                    *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_VALUE(p_FmPort, E_INVALID_HANDLE, NULL);
    SANITY_CHECK_RETURN_VALUE(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE, NULL);

    if (p_FmPort->bufferOffsets.pcdInfoOffset == ILLEGAL_BASE)
        return NULL;

    return (uint8_t *)PTR_MOVE(p_Data, p_FmPort->bufferOffsets.pcdInfoOffset);
}

t_FmPrsResult * FM_PORT_GetBufferPrsResult(t_Handle h_FmPort, char *p_Data)
{
    t_FmPort                    *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_VALUE(p_FmPort, E_INVALID_HANDLE, NULL);
    SANITY_CHECK_RETURN_VALUE(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE, NULL);

    if (p_FmPort->bufferOffsets.prsResultOffset == ILLEGAL_BASE)
        return NULL;

    return (t_FmPrsResult *)PTR_MOVE(p_Data, p_FmPort->bufferOffsets.prsResultOffset);
}

uint64_t * FM_PORT_GetBufferTimeStamp(t_Handle h_FmPort, char *p_Data)
{
    t_FmPort                    *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_VALUE(p_FmPort, E_INVALID_HANDLE, NULL);
    SANITY_CHECK_RETURN_VALUE(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE, NULL);

    if (p_FmPort->bufferOffsets.timeStampOffset == ILLEGAL_BASE)
        return NULL;

    return (uint64_t *)PTR_MOVE(p_Data, p_FmPort->bufferOffsets.timeStampOffset);
}

uint8_t * FM_PORT_GetBufferHashResult(t_Handle h_FmPort, char *p_Data)
{
    t_FmPort                    *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_VALUE(p_FmPort, E_INVALID_HANDLE, NULL);
    SANITY_CHECK_RETURN_VALUE(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE, NULL);

    if (p_FmPort->bufferOffsets.hashResultOffset == ILLEGAL_BASE)
        return NULL;

    return (uint8_t *)PTR_MOVE(p_Data, p_FmPort->bufferOffsets.hashResultOffset);
}

t_Error FM_PORT_Disable(t_Handle h_FmPort)
{
    t_FmPort                    *p_FmPort = (t_FmPort*)h_FmPort;
    volatile uint32_t           *p_BmiCfgReg = NULL;
    volatile uint32_t           *p_BmiStatusReg = NULL;
    bool                        rxPort = FALSE;
    int                         tries;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    switch (p_FmPort->portType)
    {
        case (e_FM_PORT_TYPE_RX_10G):
        case (e_FM_PORT_TYPE_RX):
            p_BmiCfgReg = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rcfg;
            p_BmiStatusReg = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rst;
            rxPort = TRUE;
            break;
        case (e_FM_PORT_TYPE_TX_10G):
        case (e_FM_PORT_TYPE_TX):
             p_BmiCfgReg = &p_FmPort->p_FmPortBmiRegs->txPortBmiRegs.fmbm_tcfg;
             p_BmiStatusReg = &p_FmPort->p_FmPortBmiRegs->txPortBmiRegs.fmbm_tst;
            break;
        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
        case (e_FM_PORT_TYPE_OH_HOST_COMMAND):
            p_BmiCfgReg = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ocfg;
            p_BmiStatusReg = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ost;
            break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid port type"));
    }

    /* check if port is already disabled */
    if (!(GET_UINT32(*p_BmiCfgReg) & BMI_PORT_CFG_EN))
    {
        if (!rxPort && !p_FmPort->imEn)
        {
            if (!(GET_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnc)& QMI_PORT_CFG_EN))
                /* port is disabled */
                return E_OK;
            else
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Inconsistency: Port's QMI is enabled but BMI disabled"));
        }
        /* port is disabled */
        return E_OK;
    }

    /* Disable QMI */
    if (!rxPort && !p_FmPort->imEn)
    {
        WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnc,
                     GET_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnc) & ~QMI_PORT_CFG_EN);
        /* wait for QMI to finish Handling dequeue tnums */
        tries=1000;
        while ((GET_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pns) & QMI_PORT_STATUS_DEQ_FD_BSY) &&
                --tries)
            XX_UDelay(1);
        if (!tries)
        {
            WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnc,
                         GET_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnc) | QMI_PORT_CFG_EN);
            RETURN_ERROR(MAJOR, E_BUSY, ("%s: can't disable! QMI busy", p_FmPort->name));
        }
    }

    /* Disable BMI */
    WRITE_UINT32(*p_BmiCfgReg, GET_UINT32(*p_BmiCfgReg) & ~BMI_PORT_CFG_EN);

    if (p_FmPort->imEn)
        FmPortImDisable(p_FmPort);

    tries=5000;
    while ((GET_UINT32(*p_BmiStatusReg) & BMI_PORT_STATUS_BSY) &&
            --tries)
        XX_UDelay(1);

    if (!tries)
    {
        if (!rxPort && !p_FmPort->imEn)
            WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnc,
                         GET_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnc) | QMI_PORT_CFG_EN);
        WRITE_UINT32(*p_BmiCfgReg, GET_UINT32(*p_BmiCfgReg) | BMI_PORT_CFG_EN);

        RETURN_ERROR(MAJOR, E_BUSY, ("%s: can't disable! BMI Busy", p_FmPort->name));
    }

    p_FmPort->enabled = 0;

    return E_OK;
}

t_Error FM_PORT_Enable(t_Handle h_FmPort)
{
    t_FmPort                    *p_FmPort = (t_FmPort*)h_FmPort;
    volatile uint32_t           *p_BmiCfgReg = NULL;
    bool                        rxPort = FALSE;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    switch (p_FmPort->portType)
    {
        case (e_FM_PORT_TYPE_RX_10G):
        case (e_FM_PORT_TYPE_RX):
            p_BmiCfgReg = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rcfg;
            rxPort = TRUE;
            break;
        case (e_FM_PORT_TYPE_TX_10G):
        case (e_FM_PORT_TYPE_TX):
             p_BmiCfgReg = &p_FmPort->p_FmPortBmiRegs->txPortBmiRegs.fmbm_tcfg;
            break;
        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
        case (e_FM_PORT_TYPE_OH_HOST_COMMAND):
            p_BmiCfgReg = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ocfg;
            break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid port type"));
    }

    /* check if port is already enabled */
    if (GET_UINT32(*p_BmiCfgReg) & BMI_PORT_CFG_EN)
    {
        if (!rxPort && !p_FmPort->imEn)
        {
            if (GET_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnc)& QMI_PORT_CFG_EN)
                /* port is enabled */
                return E_OK;
            else
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Inconsistency: Port's BMI is enabled but QMI disabled"));
        }
        /* port is enabled */
        return E_OK;
    }

    if (p_FmPort->imEn)
        FmPortImEnable(p_FmPort);

    /* Enable QMI */
    if (!rxPort && !p_FmPort->imEn)
        WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnc,
                     GET_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnc) | QMI_PORT_CFG_EN);

    /* Enable BMI */
    WRITE_UINT32(*p_BmiCfgReg, GET_UINT32(*p_BmiCfgReg) | BMI_PORT_CFG_EN);

    p_FmPort->enabled = 1;

    return E_OK;
}

t_Error FM_PORT_SetRateLimit(t_Handle h_FmPort, t_FmPortRateLimit *p_RateLimit)
{
    t_FmPort            *p_FmPort = (t_FmPort*)h_FmPort;
    uint32_t            tmpRateLimit, tmpRateLimitScale;
    volatile uint32_t   *p_RateLimitReg, *p_RateLimitScaleReg;
    uint8_t             factor, countUnitBit;
    uint16_t            baseGran;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    if ((p_FmPort->portType == e_FM_PORT_TYPE_RX_10G) || (p_FmPort->portType == e_FM_PORT_TYPE_RX) ||
                                                (p_FmPort->portType == e_FM_PORT_TYPE_OH_HOST_COMMAND))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Tx and Offline parsing ports only"));

    switch (p_FmPort->portType)
    {
        case (e_FM_PORT_TYPE_TX_10G):
        case (e_FM_PORT_TYPE_TX):
            p_RateLimitReg = &p_FmPort->p_FmPortBmiRegs->txPortBmiRegs.fmbm_trlmt;
            p_RateLimitScaleReg = &p_FmPort->p_FmPortBmiRegs->txPortBmiRegs.fmbm_trlmts;
            baseGran = 16000;
            break;
        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
            p_RateLimitReg = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_orlmt;
            p_RateLimitScaleReg = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_orlmts;
            baseGran = 10000;
           break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid port type"));
    }

    countUnitBit = (uint8_t)FmGetTimeStampScale(p_FmPort->h_Fm);  /* TimeStamp per nano seconds units */
    /* normally, we use 1 usec as the reference count */
    factor = 1;
    /* if ratelimit is too small for a 1usec factor, multiply the factor */
    while (p_RateLimit->rateLimit < baseGran/factor)
    {
        if (countUnitBit==31)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Rate limit is too small"));

        countUnitBit++;
        factor <<= 1;
    }
    /* if ratelimit is too large for a 1usec factor, it is also larger than max rate*/
    if (p_RateLimit->rateLimit > ((uint32_t)baseGran * (1<<10) * (uint32_t)factor))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Rate limit is too large"));

    tmpRateLimit = (uint32_t)(p_RateLimit->rateLimit*factor/baseGran - 1);

    if (!p_RateLimit->maxBurstSize || (p_RateLimit->maxBurstSize > MAX_BURST_SIZE))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("maxBurstSize must be between 1K and %dk", MAX_BURST_SIZE));

    tmpRateLimitScale = ((31 - (uint32_t)countUnitBit) << BMI_COUNT_RATE_UNIT_SHIFT) | BMI_RATE_LIMIT_EN;

    if (p_FmPort->portType != e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
        tmpRateLimit |= (uint32_t)(p_RateLimit->maxBurstSize - 1) << BMI_MAX_BURST_SHIFT;
    else
    {
#ifndef FM_NO_ADVANCED_RATE_LIMITER

        if ((p_FmPort->fmRevInfo.majorRev == 4) || (p_FmPort->fmRevInfo.majorRev >= 6))
        {
            switch (p_RateLimit->rateLimitDivider)
            {
                case (e_FM_PORT_DUAL_RATE_LIMITER_NONE):
                    break;
                case (e_FM_PORT_DUAL_RATE_LIMITER_SCALE_DOWN_BY_2):
                    tmpRateLimitScale |= BMI_RATE_LIMIT_SCALE_BY_2;
                    break;
                case (e_FM_PORT_DUAL_RATE_LIMITER_SCALE_DOWN_BY_4):
                    tmpRateLimitScale |= BMI_RATE_LIMIT_SCALE_BY_4;
                    break;
                case (e_FM_PORT_DUAL_RATE_LIMITER_SCALE_DOWN_BY_8):
                    tmpRateLimitScale |= BMI_RATE_LIMIT_SCALE_BY_8;
                    break;
                default:
                    break;
            }
            tmpRateLimit |= BMI_RATE_LIMIT_BURST_SIZE_GRAN;
        }
        else
#endif /* ! FM_NO_ADVANCED_RATE_LIMITER */
        {
            if (p_RateLimit->rateLimitDivider != e_FM_PORT_DUAL_RATE_LIMITER_NONE)
                    RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("FM_PORT_ConfigDualRateLimitScaleDown"));

            if (p_RateLimit->maxBurstSize % 1000)
            {
                p_RateLimit->maxBurstSize = (uint16_t)((p_RateLimit->maxBurstSize/1000)+1);
                DBG(WARNING, ("rateLimit.maxBurstSize rounded up to %d", (p_RateLimit->maxBurstSize/1000+1)*1000));
            }
            else
                p_RateLimit->maxBurstSize = (uint16_t)(p_RateLimit->maxBurstSize/1000);
        }
        tmpRateLimit |= (uint32_t)(p_RateLimit->maxBurstSize - 1) << BMI_MAX_BURST_SHIFT;

    }
    WRITE_UINT32(*p_RateLimitScaleReg, tmpRateLimitScale);
    WRITE_UINT32(*p_RateLimitReg, tmpRateLimit);

    return E_OK;
}

t_Error FM_PORT_DeleteRateLimit(t_Handle h_FmPort)
{
    t_FmPort            *p_FmPort = (t_FmPort*)h_FmPort;
    volatile uint32_t   *p_RateLimitReg, *p_RateLimitScaleReg;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_HANDLE);

    if ((p_FmPort->portType == e_FM_PORT_TYPE_RX_10G) || (p_FmPort->portType == e_FM_PORT_TYPE_RX) ||
                                                (p_FmPort->portType == e_FM_PORT_TYPE_OH_HOST_COMMAND))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Tx and Offline parsing ports only"));

    switch (p_FmPort->portType)
    {
        case (e_FM_PORT_TYPE_TX_10G):
        case (e_FM_PORT_TYPE_TX):
            p_RateLimitReg = &p_FmPort->p_FmPortBmiRegs->txPortBmiRegs.fmbm_trlmt;
            p_RateLimitScaleReg = &p_FmPort->p_FmPortBmiRegs->txPortBmiRegs.fmbm_trlmts;
            break;
        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
            p_RateLimitReg = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_orlmt;
            p_RateLimitScaleReg = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_orlmts;
           break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid port type"));
    }

    WRITE_UINT32(*p_RateLimitScaleReg, 0);
    WRITE_UINT32(*p_RateLimitReg, 0);

    return E_OK;
}

t_Error FM_PORT_SetPfcPrioritiesMappingToQmanWQ(t_Handle h_FmPort, uint8_t prio, uint8_t wq)
{
    t_FmPort    *p_FmPort = (t_FmPort*)h_FmPort;
    uint32_t    tmpReg;
    uint32_t    wqTmpReg;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    if ((p_FmPort->portType != e_FM_PORT_TYPE_TX) && (p_FmPort->portType != e_FM_PORT_TYPE_TX_10G))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("PFC mapping is available for Tx ports only"));

    if (prio > 7)
        RETURN_ERROR(MAJOR, E_NOT_IN_RANGE, ("PFC priority (%d) is out of range (0-7)", prio));
    if (wq > 7)
        RETURN_ERROR(MAJOR, E_NOT_IN_RANGE, ("WQ (%d) is out of range (0-7)", wq));

    tmpReg = GET_UINT32(p_FmPort->p_FmPortBmiRegs->txPortBmiRegs.fmbm_tpfcm[0]);
    tmpReg &= ~(0xf << ((7-prio)*4));
    wqTmpReg = ((uint32_t)wq << ((7-prio)*4));
    tmpReg |= wqTmpReg;

    WRITE_UINT32(p_FmPort->p_FmPortBmiRegs->txPortBmiRegs.fmbm_tpfcm[0], tmpReg);

    return E_OK;
}

t_Error FM_PORT_SetFrameQueueCounters(t_Handle h_FmPort, bool enable)
{
    t_FmPort                *p_FmPort = (t_FmPort*)h_FmPort;
    uint32_t                tmpReg;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    tmpReg = GET_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnc);
    if (enable)
        tmpReg |= QMI_PORT_CFG_EN_COUNTERS ;
    else
        tmpReg &= ~QMI_PORT_CFG_EN_COUNTERS;

    WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnc, tmpReg);

    return E_OK;
}

t_Error FM_PORT_SetPerformanceCounters(t_Handle h_FmPort, bool enable)
{
    t_FmPort                *p_FmPort = (t_FmPort*)h_FmPort;
    volatile uint32_t       *p_BmiPcReg = NULL;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    switch (p_FmPort->portType)
    {
        case (e_FM_PORT_TYPE_RX_10G):
        case (e_FM_PORT_TYPE_RX):
            p_BmiPcReg = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rpc;
            break;
        case (e_FM_PORT_TYPE_TX_10G):
        case (e_FM_PORT_TYPE_TX):
            p_BmiPcReg = &p_FmPort->p_FmPortBmiRegs->txPortBmiRegs.fmbm_tpc;
            break;
        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
        case (e_FM_PORT_TYPE_OH_HOST_COMMAND):
            p_BmiPcReg = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_opc;
            break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid port type"));
    }

    if (enable)
        WRITE_UINT32(*p_BmiPcReg, BMI_COUNTERS_EN);
    else
        WRITE_UINT32(*p_BmiPcReg, 0);

    return E_OK;
}

t_Error FM_PORT_SetPerformanceCountersParams(t_Handle h_FmPort, t_FmPortPerformanceCnt *p_FmPortPerformanceCnt)
{
    t_FmPort                *p_FmPort = (t_FmPort*)h_FmPort;
    uint32_t                tmpReg;
    volatile uint32_t       *p_BmiPcpReg = NULL;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);

    switch (p_FmPort->portType)
    {
        case (e_FM_PORT_TYPE_RX_10G):
        case (e_FM_PORT_TYPE_RX):
            p_BmiPcpReg = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rpcp;
            break;
        case (e_FM_PORT_TYPE_TX_10G):
        case (e_FM_PORT_TYPE_TX):
            p_BmiPcpReg = &p_FmPort->p_FmPortBmiRegs->txPortBmiRegs.fmbm_tpcp;
            break;
        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
        case (e_FM_PORT_TYPE_OH_HOST_COMMAND):
            p_BmiPcpReg = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_opcp;
            break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid port type"));
    }

    /* check parameters */
    if (!p_FmPortPerformanceCnt->taskCompVal ||
        (p_FmPortPerformanceCnt->taskCompVal > p_FmPort->tasks.num))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE,
                     ("taskCompVal (%d) has to be in the range of 1 - %d (current value)!",
                      p_FmPortPerformanceCnt->taskCompVal,
                      p_FmPort->tasks.num));
    if (!p_FmPortPerformanceCnt->dmaCompVal ||
        (p_FmPortPerformanceCnt->dmaCompVal > p_FmPort->openDmas.num))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE,
                     ("dmaCompVal (%d) has to be in the range of 1 - %d (current value)!",
                      p_FmPortPerformanceCnt->dmaCompVal,
                      p_FmPort->openDmas.num));
    if (!p_FmPortPerformanceCnt->fifoCompVal ||
        (p_FmPortPerformanceCnt->fifoCompVal > p_FmPort->fifoBufs.num))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE,
                     ("fifoCompVal (%d) has to be in the range of 256 - %d (current value)!",
                      p_FmPortPerformanceCnt->fifoCompVal,
                      p_FmPort->fifoBufs.num));
    if (p_FmPortPerformanceCnt->fifoCompVal % BMI_FIFO_UNITS)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE,
                     ("fifoCompVal (%d) has to be divisible by %d",
                      p_FmPortPerformanceCnt->fifoCompVal,
                      BMI_FIFO_UNITS));

    switch (p_FmPort->portType)
    {
        case (e_FM_PORT_TYPE_RX_10G):
        case (e_FM_PORT_TYPE_RX):
            if (!p_FmPortPerformanceCnt->queueCompVal ||
                (p_FmPortPerformanceCnt->queueCompVal > MAX_PERFORMANCE_RX_QUEUE_COMP))
                RETURN_ERROR(MAJOR, E_INVALID_VALUE,
                             ("performanceCnt.queueCompVal for Rx has to be in the range of 1 - %d",
                              MAX_PERFORMANCE_RX_QUEUE_COMP));
            break;
        case (e_FM_PORT_TYPE_TX_10G):
        case (e_FM_PORT_TYPE_TX):
            if (!p_FmPortPerformanceCnt->queueCompVal ||
                (p_FmPortPerformanceCnt->queueCompVal > MAX_PERFORMANCE_TX_QUEUE_COMP))
                RETURN_ERROR(MAJOR, E_INVALID_VALUE,
                             ("performanceCnt.queueCompVal for Tx has to be in the range of 1 - %d",
                              MAX_PERFORMANCE_TX_QUEUE_COMP));
            break;
        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
        case (e_FM_PORT_TYPE_OH_HOST_COMMAND):
            if (p_FmPortPerformanceCnt->queueCompVal)
                RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("performanceCnt.queueCompVal is not relevant for H/O ports."));
            break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid port type"));
    }

    tmpReg = 0;
    tmpReg |= ((uint32_t)(p_FmPortPerformanceCnt->queueCompVal - 1) << BMI_PERFORMANCE_PORT_COMP_SHIFT);
    tmpReg |= ((uint32_t)(p_FmPortPerformanceCnt->dmaCompVal- 1) << BMI_PERFORMANCE_DMA_COMP_SHIFT);
    tmpReg |= ((uint32_t)(p_FmPortPerformanceCnt->fifoCompVal/BMI_FIFO_UNITS - 1) << BMI_PERFORMANCE_FIFO_COMP_SHIFT);
    if ((p_FmPort->portType != e_FM_PORT_TYPE_OH_OFFLINE_PARSING) && (p_FmPort->portType != e_FM_PORT_TYPE_OH_HOST_COMMAND))
        tmpReg |= ((uint32_t)(p_FmPortPerformanceCnt->taskCompVal - 1) << BMI_PERFORMANCE_TASK_COMP_SHIFT);

    WRITE_UINT32(*p_BmiPcpReg, tmpReg);

    return E_OK;
}

t_Error FM_PORT_AnalyzePerformanceParams(t_Handle h_FmPort)
{
    t_FmPort                *p_FmPort = (t_FmPort*)h_FmPort;
    t_FmPortPerformanceCnt  currParams, savedParams;
    t_Error                 err;
    bool                    underTest, failed = FALSE;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);

    XX_Print("Analyzing Performance parameters for port (type %d, id%d)\n",
             p_FmPort->portType, p_FmPort->portId);

    currParams.taskCompVal    = (uint8_t)p_FmPort->tasks.num;
    if ((p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING) ||
        (p_FmPort->portType == e_FM_PORT_TYPE_OH_HOST_COMMAND))
        currParams.queueCompVal   = 0;
    else
        currParams.queueCompVal   = 1;
    currParams.dmaCompVal     =(uint8_t) p_FmPort->openDmas.num;
    currParams.fifoCompVal    = p_FmPort->fifoBufs.num;

    FM_PORT_SetPerformanceCounters(p_FmPort, FALSE);
    ClearPerfCnts(p_FmPort);
    if ((err = FM_PORT_SetPerformanceCountersParams(p_FmPort, &currParams)) != E_OK)
        RETURN_ERROR(MAJOR, err, NO_MSG);
    FM_PORT_SetPerformanceCounters(p_FmPort, TRUE);
    XX_UDelay(1000000);
    FM_PORT_SetPerformanceCounters(p_FmPort, FALSE);
    if (FM_PORT_GetCounter(p_FmPort, e_FM_PORT_COUNTERS_TASK_UTIL))
    {
        XX_Print ("Max num of defined port tasks (%d) utilized - Please enlarge\n",p_FmPort->tasks.num);
        failed = TRUE;
    }
    if (FM_PORT_GetCounter(p_FmPort, e_FM_PORT_COUNTERS_DMA_UTIL))
    {
        XX_Print ("Max num of defined port openDmas (%d) utilized - Please enlarge\n",p_FmPort->openDmas.num);
        failed = TRUE;
    }
    if (FM_PORT_GetCounter(p_FmPort, e_FM_PORT_COUNTERS_FIFO_UTIL))
    {
        XX_Print("Max size of defined port fifo (%d) utilized - Please enlarge\n",p_FmPort->fifoBufs.num);
        failed = TRUE;
    }
    if (failed)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);

    memset(&savedParams, 0, sizeof(savedParams));
    while (TRUE)
    {
        underTest = FALSE;
        if ((currParams.taskCompVal != 1) && !savedParams.taskCompVal)
        {
            currParams.taskCompVal--;
            underTest = TRUE;
        }
        if ((currParams.dmaCompVal != 1) && !savedParams.dmaCompVal)
        {
            currParams.dmaCompVal--;
            underTest = TRUE;
        }
        if ((currParams.fifoCompVal != BMI_FIFO_UNITS) && !savedParams.fifoCompVal)
        {
            currParams.fifoCompVal -= BMI_FIFO_UNITS;
            underTest = TRUE;
        }
        if (!underTest)
            break;

        ClearPerfCnts(p_FmPort);
        if ((err = FM_PORT_SetPerformanceCountersParams(p_FmPort, &currParams)) != E_OK)
            RETURN_ERROR(MAJOR, err, NO_MSG);
        FM_PORT_SetPerformanceCounters(p_FmPort, TRUE);
        XX_UDelay(1000000);
        FM_PORT_SetPerformanceCounters(p_FmPort, FALSE);

        if (!savedParams.taskCompVal && FM_PORT_GetCounter(p_FmPort, e_FM_PORT_COUNTERS_TASK_UTIL))
            savedParams.taskCompVal = (uint8_t)(currParams.taskCompVal+2);
        if (!savedParams.dmaCompVal && FM_PORT_GetCounter(p_FmPort, e_FM_PORT_COUNTERS_DMA_UTIL))
            savedParams.dmaCompVal = (uint8_t)(currParams.dmaCompVal+2);
        if (!savedParams.fifoCompVal && FM_PORT_GetCounter(p_FmPort, e_FM_PORT_COUNTERS_FIFO_UTIL))
            savedParams.fifoCompVal = currParams.fifoCompVal+(2*BMI_FIFO_UNITS);
    }

    XX_Print("best vals: tasks %d, dmas %d, fifos %d\n",
             savedParams.taskCompVal, savedParams.dmaCompVal, savedParams.fifoCompVal);
    return E_OK;
}

t_Error FM_PORT_SetStatisticsCounters(t_Handle h_FmPort, bool enable)
{
    t_FmPort                *p_FmPort = (t_FmPort*)h_FmPort;
    uint32_t                tmpReg;
    volatile uint32_t       *p_BmiStcReg = NULL;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    switch (p_FmPort->portType)
    {
        case (e_FM_PORT_TYPE_RX_10G):
        case (e_FM_PORT_TYPE_RX):
            p_BmiStcReg = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rstc;
            break;
        case (e_FM_PORT_TYPE_TX_10G):
        case (e_FM_PORT_TYPE_TX):
            p_BmiStcReg = &p_FmPort->p_FmPortBmiRegs->txPortBmiRegs.fmbm_tstc;
            break;
        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
        case (e_FM_PORT_TYPE_OH_HOST_COMMAND):
            p_BmiStcReg = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ostc;
            break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid port type"));
    }

    tmpReg = GET_UINT32(*p_BmiStcReg);

    if (enable)
        tmpReg |= BMI_COUNTERS_EN;
    else
        tmpReg &= ~BMI_COUNTERS_EN;

    WRITE_UINT32(*p_BmiStcReg, tmpReg);

    return E_OK;
}

t_Error FM_PORT_SetErrorsRoute(t_Handle h_FmPort, fmPortFrameErrSelect_t errs)
{
    t_FmPort                *p_FmPort = (t_FmPort*)h_FmPort;
    volatile uint32_t       *p_ErrQReg, *p_ErrDiscard;

    switch (p_FmPort->portType)
    {
        case (e_FM_PORT_TYPE_RX_10G):
        case (e_FM_PORT_TYPE_RX):
            p_ErrQReg = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfsem;
            p_ErrDiscard = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfsdm;
            break;
        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
            p_ErrQReg = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofsem;
            p_ErrDiscard = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofsdm;
            break;
        default:
           RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx and offline parsing ports only"));
    }

    if (GET_UINT32(*p_ErrDiscard) & errs)
        RETURN_ERROR(MAJOR, E_INVALID_SELECTION, ("Selectd Errors that were configured to cause frame discard."));

    WRITE_UINT32(*p_ErrQReg, errs);
 
#ifdef FM_ERROR_VSP_NO_MATCH_SW006
    if (p_FmPort->fmRevInfo.majorRev >= 6)
    {
        t_FmPcdCtrlParamsPage   *p_ParamsPage;

        FmPortSetGprFunc(p_FmPort, e_FM_PORT_GPR_MURAM_PAGE, (void**)&p_ParamsPage);
        ASSERT_COND(p_ParamsPage);
        WRITE_UINT32(p_ParamsPage->errorsDiscardMask, GET_UINT32(*p_ErrDiscard) | errs);
    }
#endif /* FM_ERROR_VSP_NO_MATCH_SW006 */
 
    return E_OK;
}

t_Error FM_PORT_SetAllocBufCounter(t_Handle h_FmPort, uint8_t poolId, bool enable)
{
    t_FmPort                *p_FmPort = (t_FmPort*)h_FmPort;
    uint32_t                tmpReg;
    int                     i;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(poolId<BM_MAX_NUM_OF_POOLS, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) && (p_FmPort->portType != e_FM_PORT_TYPE_RX))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx ports only"));

    for (i=0 ; i< FM_PORT_MAX_NUM_OF_EXT_POOLS ; i++)
    {
        tmpReg = GET_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_ebmpi[i]);
        if ((uint8_t)((tmpReg & BMI_EXT_BUF_POOL_ID_MASK) >> BMI_EXT_BUF_POOL_ID_SHIFT) == poolId)
        {
            if (enable)
                tmpReg |= BMI_EXT_BUF_POOL_EN_COUNTER;
            else
                tmpReg &= ~BMI_EXT_BUF_POOL_EN_COUNTER;
            WRITE_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_ebmpi[i], tmpReg);
            break;
        }
    }
    if (i == FM_PORT_MAX_NUM_OF_EXT_POOLS)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE,("poolId %d is not included in this ports pools", poolId));

    return E_OK;
}

uint32_t FM_PORT_GetCounter(t_Handle h_FmPort, e_FmPortCounters counter)
{
    t_FmPort            *p_FmPort = (t_FmPort*)h_FmPort;
    bool                bmiCounter = FALSE;
    volatile uint32_t   *p_Reg;

    SANITY_CHECK_RETURN_VALUE(p_FmPort, E_INVALID_HANDLE, 0);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    switch (counter)
    {
        case (e_FM_PORT_COUNTERS_DEQ_TOTAL):
        case (e_FM_PORT_COUNTERS_DEQ_FROM_DEFAULT):
        case (e_FM_PORT_COUNTERS_DEQ_CONFIRM):
            /* check that counter is available for the port type */
            if ((p_FmPort->portType == e_FM_PORT_TYPE_RX) ||
                (p_FmPort->portType == e_FM_PORT_TYPE_RX_10G))
            {
                REPORT_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter is not available for Rx ports"));
                return 0;
            }
            bmiCounter = FALSE;
        case (e_FM_PORT_COUNTERS_ENQ_TOTAL):
            bmiCounter = FALSE;
            break;
        default: /* BMI counters (or error - will be checked in BMI routine )*/
            bmiCounter = TRUE;
            break;
    }

    if (bmiCounter)
    {
        switch (p_FmPort->portType)
        {
            case (e_FM_PORT_TYPE_RX_10G):
            case (e_FM_PORT_TYPE_RX):
                if (BmiRxPortCheckAndGetCounterPtr(p_FmPort, counter, &p_Reg))
                {
                    REPORT_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);
                    return 0;
                }
                break;
            case (e_FM_PORT_TYPE_TX_10G):
            case (e_FM_PORT_TYPE_TX):
                if (BmiTxPortCheckAndGetCounterPtr(p_FmPort, counter, &p_Reg))
                {
                    REPORT_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);
                    return 0;
                }
                break;
            case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
            case (e_FM_PORT_TYPE_OH_HOST_COMMAND):
                if (BmiOhPortCheckAndGetCounterPtr(p_FmPort, counter, &p_Reg))
                {
                    REPORT_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);
                    return 0;
                }
                break;
            default:
                REPORT_ERROR(MAJOR, E_INVALID_STATE, ("Unsupported port type"));
                return 0;
        }
        return GET_UINT32(*p_Reg);
    }
    else /* QMI counter */
    {
        /* check that counters are enabled */
        if (!(GET_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnc) & QMI_PORT_CFG_EN_COUNTERS))
        {
            REPORT_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter was not enabled"));
            return 0;
        }

        /* Set counter */
        switch (counter)
        {
           case (e_FM_PORT_COUNTERS_ENQ_TOTAL):
                return GET_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnetfc);
            case (e_FM_PORT_COUNTERS_DEQ_TOTAL):
                return GET_UINT32(p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs.fmqm_pndtfc);
            case (e_FM_PORT_COUNTERS_DEQ_FROM_DEFAULT):
                return GET_UINT32(p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs.fmqm_pndfdc);
            case (e_FM_PORT_COUNTERS_DEQ_CONFIRM):
                return GET_UINT32(p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs.fmqm_pndcc);
            default:
                REPORT_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter is not available"));
                return 0;
        }
    }
}

t_Error FM_PORT_ModifyCounter(t_Handle h_FmPort, e_FmPortCounters counter, uint32_t value)
{
    t_FmPort            *p_FmPort = (t_FmPort*)h_FmPort;
    bool                bmiCounter = FALSE;
    volatile uint32_t   *p_Reg;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    switch (counter)
    {
        case (e_FM_PORT_COUNTERS_DEQ_TOTAL):
        case (e_FM_PORT_COUNTERS_DEQ_FROM_DEFAULT):
        case (e_FM_PORT_COUNTERS_DEQ_CONFIRM ):
            /* check that counter is available for the port type */
            if ((p_FmPort->portType == e_FM_PORT_TYPE_RX) || (p_FmPort->portType == e_FM_PORT_TYPE_RX_10G))
                        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter is not available for Rx ports"));
        case (e_FM_PORT_COUNTERS_ENQ_TOTAL):
            bmiCounter = FALSE;
            break;
        default: /* BMI counters (or error - will be checked in BMI routine )*/
            bmiCounter = TRUE;
            break;
    }

    if (bmiCounter)
    {
        switch (p_FmPort->portType)
        {
            case (e_FM_PORT_TYPE_RX_10G):
            case (e_FM_PORT_TYPE_RX):
               if (BmiRxPortCheckAndGetCounterPtr(p_FmPort, counter, &p_Reg))
                    RETURN_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);
                break;
            case (e_FM_PORT_TYPE_TX_10G):
            case (e_FM_PORT_TYPE_TX):
               if (BmiTxPortCheckAndGetCounterPtr(p_FmPort, counter, &p_Reg))
                    RETURN_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);
                break;
            case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
            case (e_FM_PORT_TYPE_OH_HOST_COMMAND):
               if (BmiOhPortCheckAndGetCounterPtr(p_FmPort, counter, &p_Reg))
                    RETURN_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);
                 break;
            default:
               RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Unsupported port type"));
        }
        WRITE_UINT32(*p_Reg, value);
    }
    else /* QMI counter */
    {

        /* check that counters are enabled */
        if (!(GET_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnc) & QMI_PORT_CFG_EN_COUNTERS))
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter was not enabled"));

        /* Set counter */
        switch (counter)
        {
           case (e_FM_PORT_COUNTERS_ENQ_TOTAL):
                WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->fmqm_pnetfc, value);
                break;
            case (e_FM_PORT_COUNTERS_DEQ_TOTAL):
                WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs.fmqm_pndtfc, value);
                break;
            case (e_FM_PORT_COUNTERS_DEQ_FROM_DEFAULT):
                WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs.fmqm_pndfdc, value);
                break;
            case (e_FM_PORT_COUNTERS_DEQ_CONFIRM):
                WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs.fmqm_pndcc, value);
                break;
            default:
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter is not available"));
        }
    }

    return E_OK;
}

uint32_t FM_PORT_GetAllocBufCounter(t_Handle h_FmPort, uint8_t poolId)
{
    t_FmPort        *p_FmPort = (t_FmPort*)h_FmPort;
    uint32_t        extPoolReg;
    uint8_t         tmpPool;
    uint8_t         i;

    SANITY_CHECK_RETURN_VALUE(p_FmPort, E_INVALID_HANDLE, 0);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX) && (p_FmPort->portType == e_FM_PORT_TYPE_RX_10G))
    {
        REPORT_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter is not available for non-Rx ports"));
        return 0;
    }

    for (i=0;i<FM_PORT_MAX_NUM_OF_EXT_POOLS;i++)
    {
        extPoolReg = GET_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_ebmpi[i]);
        if (extPoolReg & BMI_EXT_BUF_POOL_VALID)
        {
            tmpPool = (uint8_t)((extPoolReg & BMI_EXT_BUF_POOL_ID_MASK) >> BMI_EXT_BUF_POOL_ID_SHIFT);
            if (tmpPool == poolId)
            {
                if (extPoolReg & BMI_EXT_BUF_POOL_EN_COUNTER)
                    return  GET_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_acnt[i]);
                else
                {
                    REPORT_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter is not enabled"));
                    return 0;
                }
            }
        }
    }
    REPORT_ERROR(MAJOR, E_INVALID_STATE, ("Pool %d is not used", poolId));
    return 0;
}

t_Error FM_PORT_ModifyAllocBufCounter(t_Handle h_FmPort, uint8_t poolId, uint32_t value)
{
    t_FmPort        *p_FmPort = (t_FmPort *)h_FmPort;
    uint32_t        extPoolReg;
    uint8_t         tmpPool;
    uint8_t         i;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX) && (p_FmPort->portType == e_FM_PORT_TYPE_RX_10G))
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter is not available for non-Rx ports"));


    for (i=0;i<FM_PORT_MAX_NUM_OF_EXT_POOLS;i++)
    {
        extPoolReg = GET_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_ebmpi[i]);
        if (extPoolReg & BMI_EXT_BUF_POOL_VALID)
        {
            tmpPool = (uint8_t)((extPoolReg & BMI_EXT_BUF_POOL_ID_MASK) >> BMI_EXT_BUF_POOL_ID_SHIFT);
            if (tmpPool == poolId)
            {
                if (extPoolReg & BMI_EXT_BUF_POOL_EN_COUNTER)
                {
                    WRITE_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_acnt[i], value);
                    return E_OK;
                }
                else
                    RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter is not enabled"));
            }
        }
    }
    RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Pool %d is not used", poolId));
}

bool FM_PORT_IsStalled(t_Handle h_FmPort)
{
    t_FmPort    *p_FmPort = (t_FmPort*)h_FmPort;
    t_Error     err;
    bool        isStalled;

    SANITY_CHECK_RETURN_VALUE(p_FmPort, E_INVALID_HANDLE, FALSE);
    SANITY_CHECK_RETURN_VALUE(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE, FALSE);

    err = FmIsPortStalled(p_FmPort->h_Fm, p_FmPort->hardwarePortId, &isStalled);
    if (err != E_OK)
    {
        REPORT_ERROR(MAJOR, err, NO_MSG);
        return TRUE;
    }
    return isStalled;
}

t_Error FM_PORT_ReleaseStalled(t_Handle h_FmPort)
{
    t_FmPort        *p_FmPort = (t_FmPort*)h_FmPort;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    return FmResumeStalledPort(p_FmPort->h_Fm, p_FmPort->hardwarePortId);
}

t_Error FM_PORT_SetRxL4ChecksumVerify(t_Handle h_FmPort, bool l4Checksum)
{
    t_FmPort *p_FmPort = (t_FmPort*)h_FmPort;
    uint32_t tmpReg;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_RX))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx ports only"));

    tmpReg = GET_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfne);
    if (l4Checksum)
        tmpReg &= ~BMI_PORT_RFNE_FRWD_DCL4C;
    else
        tmpReg |= BMI_PORT_RFNE_FRWD_DCL4C;
    WRITE_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfne, tmpReg);

    return E_OK;
}

/*****************************************************************************/
/*       API Run-time PCD Control unit functions                             */
/*****************************************************************************/

#if (DPAA_VERSION >= 11)
t_Error FM_PORT_VSPAlloc(t_Handle h_FmPort, t_FmPortVSPAllocParams *p_VSPParams)
{
    t_FmPort                    *p_FmPort = (t_FmPort*)h_FmPort;
    t_Error                     err = E_OK;
    volatile uint32_t           *p_BmiStorageProfileId = NULL, *p_BmiVspe = NULL;
    uint32_t                    tmpReg = 0, tmp = 0;
    uint16_t                    hwStoragePrflId;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->h_Fm, E_INVALID_HANDLE);
    /*for numOfProfiles = 0 don't call this function*/
    SANITY_CHECK_RETURN_ERROR(p_VSPParams->numOfProfiles, E_INVALID_VALUE);
    /*dfltRelativeId should be in the range of numOfProfiles*/
    SANITY_CHECK_RETURN_ERROR(IN_RANGE(0, p_VSPParams->dfltRelativeId, (p_VSPParams->numOfProfiles - 1)), E_INVALID_VALUE);
    /*p_FmPort should be from Rx type or OP*/
    SANITY_CHECK_RETURN_ERROR(((p_FmPort->portType == e_FM_PORT_TYPE_RX_10G) ||
                               (p_FmPort->portType == e_FM_PORT_TYPE_RX) ||
                               (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING)), E_INVALID_VALUE);
    /*port should be disabled*/
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->enabled, E_INVALID_STATE);
    /*if its called for Rx port relevant Tx Port should be passed (initialized) too and it should be disabled*/
    SANITY_CHECK_RETURN_ERROR(((p_VSPParams->h_FmTxPort &&
                                !((t_FmPort *)(p_VSPParams->h_FmTxPort))->enabled) ||
                               (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING)), E_INVALID_VALUE);
    /*should be called before SetPCD - this port should be without PCD*/
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->pcdEngines, E_INVALID_STATE);

    /*alloc window of VSPs for this port*/
    err = FmVSPAllocForPort(p_FmPort->h_Fm,
                            p_FmPort->portType,
                            p_FmPort->portId,
                            p_VSPParams->numOfProfiles);
    if (err != E_OK)
        RETURN_ERROR(MAJOR, err, NO_MSG);

    /*get absolute VSP ID for dfltRelative*/
    err = FmVSPGetAbsoluteProfileId(p_FmPort->h_Fm,
                                    p_FmPort->portType,
                                    p_FmPort->portId,
                                    p_VSPParams->dfltRelativeId,
                                    &hwStoragePrflId);
    if (err != E_OK)
        RETURN_ERROR(MAJOR, err,NO_MSG);

    /*fill relevant registers for p_FmPort and relative TxPort in the case p_FmPort from Rx type*/
    switch (p_FmPort->portType)
    {
        case (e_FM_PORT_TYPE_RX_10G):
        case (e_FM_PORT_TYPE_RX):
            p_BmiStorageProfileId = &(((t_FmPort *)(p_VSPParams->h_FmTxPort))->p_FmPortBmiRegs->txPortBmiRegs.fmbm_tcfqid);
            p_BmiVspe = &(((t_FmPort *)(p_VSPParams->h_FmTxPort))->p_FmPortBmiRegs->txPortBmiRegs.fmbm_tfne);

            tmpReg = GET_UINT32(*p_BmiStorageProfileId) & ~BMI_SP_ID_MASK;
            tmpReg |= (uint32_t)hwStoragePrflId<<BMI_SP_ID_SHIFT;
            WRITE_UINT32(*p_BmiStorageProfileId, tmpReg);

            tmpReg = GET_UINT32(*p_BmiVspe);
            WRITE_UINT32(*p_BmiVspe, tmpReg | BMI_SP_EN);

            p_BmiStorageProfileId = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfqid;
            p_BmiVspe = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rpp;
            hwStoragePrflId = p_VSPParams->dfltRelativeId;
            break;

        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
            tmpReg = NIA_ENG_BMI | NIA_BMI_AC_FETCH_ALL_FRAME;
            WRITE_UINT32(p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs.fmqm_pndn,tmpReg);

            p_BmiStorageProfileId = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofqid;
            p_BmiVspe = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_opp;
            tmp |= BMI_EBD_EN;
            break;

        default:
            RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx and offline parsing ports only"));
    }

    p_FmPort->vspe = TRUE;
    p_FmPort->dfltRelativeId = p_VSPParams->dfltRelativeId;
 
    tmpReg = GET_UINT32(*p_BmiStorageProfileId) & ~BMI_SP_ID_MASK;
    tmpReg |= (uint32_t)hwStoragePrflId<<BMI_SP_ID_SHIFT;
    WRITE_UINT32(*p_BmiStorageProfileId, tmpReg);

    tmpReg = GET_UINT32(*p_BmiVspe);
    WRITE_UINT32(*p_BmiVspe, tmpReg | BMI_SP_EN | tmp);
    return E_OK;
}
#endif /* (DPAA_VERSION >= 11) */

t_Error FM_PORT_PcdPlcrAllocProfiles(t_Handle h_FmPort, uint16_t numOfProfiles)
{
    t_FmPort                    *p_FmPort = (t_FmPort*)h_FmPort;
    t_Error                     err = E_OK;

    p_FmPort->h_FmPcd = FmGetPcdHandle(p_FmPort->h_Fm);
    ASSERT_COND(p_FmPort->h_FmPcd);

    if (!TRY_LOCK(p_FmPort->h_Spinlock, &p_FmPort->lock))
    {
         DBG(TRACE, ("FM Port Try Lock - BUSY"));
         return ERROR_CODE(E_BUSY);
    }

    if (numOfProfiles)
    {
        err = FmPcdPlcrAllocProfiles(p_FmPort->h_FmPcd, p_FmPort->hardwarePortId, numOfProfiles);
        if (err)
            RETURN_ERROR(MAJOR, err, NO_MSG);
    }
    /* set the port handle within the PCD policer, even if no profiles defined */
    FmPcdPortRegister(p_FmPort->h_FmPcd, h_FmPort, p_FmPort->hardwarePortId);

    RELEASE_LOCK(p_FmPort->lock);

    return E_OK;
}

t_Error FM_PORT_PcdPlcrFreeProfiles(t_Handle h_FmPort)
{
    t_FmPort                    *p_FmPort = (t_FmPort*)h_FmPort;
    t_Error                     err = E_OK;

    if (!TRY_LOCK(p_FmPort->h_Spinlock, &p_FmPort->lock))
    {
         DBG(TRACE, ("FM Port Try Lock - BUSY"));
         return ERROR_CODE(E_BUSY);
    }

    err = FmPcdPlcrFreeProfiles(p_FmPort->h_FmPcd, p_FmPort->hardwarePortId);

    RELEASE_LOCK(p_FmPort->lock);

    if (err)
        RETURN_ERROR(MAJOR, err,NO_MSG);

    return E_OK;
}

t_Error FM_PORT_PcdKgModifyInitialScheme (t_Handle h_FmPort, t_FmPcdKgSchemeSelect *p_FmPcdKgScheme)
{
    t_FmPort                *p_FmPort = (t_FmPort*)h_FmPort;
    volatile uint32_t       *p_BmiHpnia = NULL;
    uint32_t                tmpReg;
    uint8_t                 relativeSchemeId;
    uint8_t                 physicalSchemeId;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->pcdEngines & FM_PCD_KG , E_INVALID_STATE);

    tmpReg = (uint32_t)((p_FmPort->pcdEngines & FM_PCD_CC)? NIA_KG_CC_EN:0);
    switch (p_FmPort->portType)
    {
        case (e_FM_PORT_TYPE_RX_10G):
        case (e_FM_PORT_TYPE_RX):
            p_BmiHpnia = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfpne;
            break;
        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
            p_BmiHpnia = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofpne;
            break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx and offline parsing ports only"));
    }

    if (!TRY_LOCK(p_FmPort->h_Spinlock, &p_FmPort->lock))
    {
         DBG(TRACE, ("FM Port Try Lock - BUSY"));
         return ERROR_CODE(E_BUSY);
    }

    /* if we want to change to direct scheme, we need to check that this scheme is valid */
    if (p_FmPcdKgScheme->direct)
    {
        physicalSchemeId = FmPcdKgGetSchemeId(p_FmPcdKgScheme->h_DirectScheme);
        /* check that this scheme is bound to this port */
        if (!(p_FmPort->schemesPerPortVector &  (uint32_t)(1 << (31 - (uint32_t)physicalSchemeId))))
        {
            RELEASE_LOCK(p_FmPort->lock);
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("called with a scheme that is not bound to this port"));
        }

        relativeSchemeId = FmPcdKgGetRelativeSchemeId(p_FmPort->h_FmPcd, physicalSchemeId);
        if (relativeSchemeId >= FM_PCD_KG_NUM_OF_SCHEMES)
        {
            RELEASE_LOCK(p_FmPort->lock);
            RETURN_ERROR(MAJOR, E_NOT_IN_RANGE, ("called with invalid Scheme "));
        }

        if (!FmPcdKgIsSchemeValidSw(p_FmPcdKgScheme->h_DirectScheme))
        {
            RELEASE_LOCK(p_FmPort->lock);
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("called with uninitialized Scheme "));
        }

        WRITE_UINT32(*p_BmiHpnia, NIA_ENG_KG | tmpReg | NIA_KG_DIRECT | (uint32_t)physicalSchemeId);
    }
    else /* change to indirect scheme */
        WRITE_UINT32(*p_BmiHpnia, NIA_ENG_KG | tmpReg);
    RELEASE_LOCK(p_FmPort->lock);

    return E_OK;
}

t_Error     FM_PORT_PcdPlcrModifyInitialProfile (t_Handle h_FmPort, t_Handle h_Profile)
{
    t_FmPort                        *p_FmPort = (t_FmPort*)h_FmPort;
    volatile uint32_t               *p_BmiNia;
    volatile uint32_t               *p_BmiHpnia;
    uint32_t                        tmpReg;
    uint16_t                        absoluteProfileId = FmPcdPlcrProfileGetAbsoluteId(h_Profile);

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->pcdEngines & FM_PCD_PLCR , E_INVALID_STATE);

    /* check relevance of this routine  - only when policer is used
    directly after BMI or Parser */
    if ((p_FmPort->pcdEngines & FM_PCD_KG) || (p_FmPort->pcdEngines & FM_PCD_CC))
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("relevant only when PCD support mode is e_FM_PCD_SUPPORT_PLCR_ONLY or e_FM_PCD_SUPPORT_PRS_AND_PLCR"));

    switch (p_FmPort->portType)
    {
        case (e_FM_PORT_TYPE_RX_10G):
        case (e_FM_PORT_TYPE_RX):
            p_BmiNia = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfne;
            p_BmiHpnia = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfpne;
            tmpReg = GET_UINT32(*p_BmiNia) & BMI_RFNE_FDCS_MASK;
            break;
        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
            p_BmiNia = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofne;
            p_BmiHpnia = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofpne;
            tmpReg = 0;
            break;
        default:
           RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx and offline parsing ports only"));
    }

    if (!TRY_LOCK(p_FmPort->h_Spinlock, &p_FmPort->lock))
    {
         DBG(TRACE, ("FM Port Try Lock - BUSY"));
         return ERROR_CODE(E_BUSY);
    }

    if (!FmPcdPlcrIsProfileValid(p_FmPort->h_FmPcd, absoluteProfileId))
    {
        RELEASE_LOCK(p_FmPort->lock);
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("Invalid profile"));
    }

    tmpReg = (uint32_t)(NIA_ENG_PLCR | NIA_PLCR_ABSOLUTE | absoluteProfileId);

    if (p_FmPort->pcdEngines & FM_PCD_PRS) /* e_FM_PCD_SUPPORT_PRS_AND_PLCR */
    {
        /* update BMI HPNIA */
        WRITE_UINT32(*p_BmiHpnia, tmpReg);
    }
    else /* e_FM_PCD_SUPPORT_PLCR_ONLY */
    {
        /* rfne may contain FDCS bits, so first we read them. */
        tmpReg |= (GET_UINT32(*p_BmiNia) & BMI_RFNE_FDCS_MASK);
        /* update BMI NIA */
        WRITE_UINT32(*p_BmiNia, tmpReg);
    }
    RELEASE_LOCK(p_FmPort->lock);

    return E_OK;
}

t_Error FM_PORT_PcdCcModifyTree (t_Handle h_FmPort, t_Handle h_CcTree)
{
    t_FmPort                            *p_FmPort = (t_FmPort*)h_FmPort;
    t_Error                             err = E_OK;
    volatile uint32_t                   *p_BmiCcBase=NULL;
    volatile uint32_t                   *p_BmiNia=NULL;
    uint32_t                            ccTreePhysOffset;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_VALUE);

    if (p_FmPort->imEn)
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for non-independent mode ports only"));

    /* get PCD registers pointers */
    switch (p_FmPort->portType)
    {
        case (e_FM_PORT_TYPE_RX_10G):
        case (e_FM_PORT_TYPE_RX):
            p_BmiNia = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfne;
            break;
        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
            p_BmiNia = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofne;
            break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx and offline parsing ports only"));
    }

    /* check that current NIA is BMI to BMI */
    if ((GET_UINT32(*p_BmiNia) & ~BMI_RFNE_FDCS_MASK) != GET_NIA_BMI_AC_ENQ_FRAME(p_FmPort->h_FmPcd))
            RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("may be called only for ports in BMI-to-BMI state."));

    if (p_FmPort->pcdEngines & FM_PCD_CC)
    {
        if (p_FmPort->h_IpReassemblyManip)
        {
            err = FmPcdCcTreeAddIPR(p_FmPort->h_FmPcd,
                                    h_CcTree,
                                    NULL,
                                    p_FmPort->h_IpReassemblyManip,
                                    FALSE);
            if (err != E_OK)
            {
                RETURN_ERROR(MAJOR, err, NO_MSG);
            }
        }
        switch (p_FmPort->portType)
        {
            case (e_FM_PORT_TYPE_RX_10G):
            case (e_FM_PORT_TYPE_RX):
                p_BmiCcBase = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rccb;
                break;
            case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
                p_BmiCcBase = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_occb;
                break;
            default:
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid port type"));
        }

        if (!TRY_LOCK(p_FmPort->h_Spinlock, &p_FmPort->lock))
        {
             DBG(TRACE, ("FM Port Try Lock - BUSY"));
             return ERROR_CODE(E_BUSY);
        }
        err = FmPcdCcBindTree(p_FmPort->h_FmPcd, NULL, h_CcTree, &ccTreePhysOffset, h_FmPort);
        if (err)
        {
            RELEASE_LOCK(p_FmPort->lock);
            RETURN_ERROR(MAJOR, err, NO_MSG);
        }
        WRITE_UINT32(*p_BmiCcBase, ccTreePhysOffset);

        p_FmPort->ccTreeId = h_CcTree;
        RELEASE_LOCK(p_FmPort->lock);
    }
    else
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Coarse Classification not defined for this port."));

    return E_OK;
}

t_Error FM_PORT_AttachPCD(t_Handle h_FmPort)
{
    t_FmPort        *p_FmPort = (t_FmPort*)h_FmPort;
    t_Error         err = E_OK;

    SANITY_CHECK_RETURN_ERROR(h_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    if (p_FmPort->imEn)
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for non-independent mode ports only"));

    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_RX) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_OH_OFFLINE_PARSING))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx and offline parsing ports only"));

    if (!TRY_LOCK(p_FmPort->h_Spinlock, &p_FmPort->lock))
    {
         DBG(TRACE, ("FM Port Try Lock - BUSY"));
         return ERROR_CODE(E_BUSY);
    }

    if (p_FmPort->h_IpReassemblyTree)
        p_FmPort->pcdEngines |= FM_PCD_CC;

    err = AttachPCD(h_FmPort);
    RELEASE_LOCK(p_FmPort->lock);

    return err;
}

t_Error FM_PORT_DetachPCD(t_Handle h_FmPort)
{
    t_FmPort        *p_FmPort = (t_FmPort*)h_FmPort;
    t_Error         err = E_OK;

    SANITY_CHECK_RETURN_ERROR(h_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    if (p_FmPort->imEn)
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for non-independent mode ports only"));

    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_RX) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_OH_OFFLINE_PARSING))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx and offline parsing ports only"));

    if (!TRY_LOCK(p_FmPort->h_Spinlock, &p_FmPort->lock))
    {
         DBG(TRACE, ("FM Port Try Lock - BUSY"));
         return ERROR_CODE(E_BUSY);
    }

    err = DetachPCD(h_FmPort);
    if (err != E_OK)
    {
        RELEASE_LOCK(p_FmPort->lock);
        RETURN_ERROR(MAJOR, err, NO_MSG);
    }

    if (p_FmPort->h_IpReassemblyTree)
        p_FmPort->pcdEngines &= ~FM_PCD_CC;
    RELEASE_LOCK(p_FmPort->lock);

    return E_OK;
}

t_Error FM_PORT_SetPCD(t_Handle h_FmPort, t_FmPortPcdParams *p_PcdParam)
{
    t_FmPort                *p_FmPort = (t_FmPort*)h_FmPort;
    t_Error                 err = E_OK;
    t_FmPortPcdParams       modifiedPcdParams, *p_PcdParams;
    t_FmPcdCcTreeParams     *p_FmPcdCcTreeParams;
    t_FmPortPcdCcParams     fmPortPcdCcParams;
    t_FmPortGetSetCcParams  fmPortGetSetCcParams;

    SANITY_CHECK_RETURN_ERROR(h_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    if (p_FmPort->imEn)
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for non-independent mode ports only"));

    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_RX) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_OH_OFFLINE_PARSING))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx and offline parsing ports only"));

    if (!TRY_LOCK(p_FmPort->h_Spinlock, &p_FmPort->lock))
    {
         DBG(TRACE, ("FM Port Try Lock - BUSY"));
         return ERROR_CODE(E_BUSY);
    }

    p_FmPort->h_FmPcd = FmGetPcdHandle(p_FmPort->h_Fm);
    ASSERT_COND(p_FmPort->h_FmPcd);

    memcpy(&modifiedPcdParams, p_PcdParam, sizeof(t_FmPortPcdParams));
    p_PcdParams = &modifiedPcdParams;
    if (p_PcdParams->h_IpReassemblyManip)
    {
        if ((p_PcdParams->pcdSupport != e_FM_PORT_PCD_SUPPORT_PRS_AND_KG) &&
            (p_PcdParams->pcdSupport != e_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_CC) &&
            (p_PcdParams->pcdSupport != e_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_CC_AND_PLCR) &&
            (p_PcdParams->pcdSupport != e_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_PLCR))
        {
            RELEASE_LOCK(p_FmPort->lock);
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("pcdSupport must have KG for supporting IPR"));
        }
        p_FmPort->h_IpReassemblyManip = p_PcdParams->h_IpReassemblyManip;
        if (!p_PcdParams->p_CcParams)
        {
            if (!((p_PcdParams->pcdSupport == e_FM_PORT_PCD_SUPPORT_PRS_AND_KG) ||
                  (p_PcdParams->pcdSupport == e_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_PLCR)))
            {
                RELEASE_LOCK(p_FmPort->lock);
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("PCD initialization structure is not consistent with pcdSupport"));
            }

            /* No user-tree, need to build internal tree */
            p_FmPcdCcTreeParams = (t_FmPcdCcTreeParams*)XX_Malloc(sizeof(t_FmPcdCcTreeParams));
            if (!p_FmPcdCcTreeParams)
                RETURN_ERROR(MAJOR, E_NO_MEMORY, ("p_FmPcdCcTreeParams"));
            memset(p_FmPcdCcTreeParams, 0, sizeof(t_FmPcdCcTreeParams));
            p_FmPcdCcTreeParams->h_NetEnv = p_PcdParams->h_NetEnv;
            p_FmPort->h_IpReassemblyTree = FM_PCD_CcRootBuild(p_FmPort->h_FmPcd, p_FmPcdCcTreeParams);

            if (!p_FmPort->h_IpReassemblyTree)
            {
                RELEASE_LOCK(p_FmPort->lock);
                XX_Free(p_FmPcdCcTreeParams);
                RETURN_ERROR(MAJOR, E_INVALID_HANDLE, ("FM_PCD_CcBuildTree for IPR failed"));
            }
            if (p_PcdParams->pcdSupport == e_FM_PORT_PCD_SUPPORT_PRS_AND_KG)
                p_PcdParams->pcdSupport = e_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_CC;
            else
                p_PcdParams->pcdSupport = e_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_CC_AND_PLCR;

            memset(&fmPortPcdCcParams, 0, sizeof(t_FmPortPcdCcParams));
            fmPortPcdCcParams.h_CcTree = p_FmPort->h_IpReassemblyTree;
            p_PcdParams->p_CcParams = &fmPortPcdCcParams;
            XX_Free(p_FmPcdCcTreeParams);
        }

        err = FmPcdCcTreeAddIPR(p_FmPort->h_FmPcd,
                                p_PcdParams->p_CcParams->h_CcTree,
                                p_PcdParams->h_NetEnv,
                                p_FmPort->h_IpReassemblyManip,
                                TRUE);
        if (err != E_OK)
        {
            if (p_FmPort->h_IpReassemblyTree)
            {
                FM_PCD_CcRootDelete(p_FmPort->h_IpReassemblyTree);
                p_FmPort->h_IpReassemblyTree = NULL;
            }
            RELEASE_LOCK(p_FmPort->lock);
            RETURN_ERROR(MAJOR, err, NO_MSG);
        }
    }

    if (!FmPcdLockTryLockAll(p_FmPort->h_FmPcd))
    {
        if (p_FmPort->h_IpReassemblyTree)
        {
            FM_PCD_CcRootDelete(p_FmPort->h_IpReassemblyTree);
            p_FmPort->h_IpReassemblyTree = NULL;
        }
        RELEASE_LOCK(p_FmPort->lock);
        DBG(TRACE, ("Try LockAll - BUSY"));
        return ERROR_CODE(E_BUSY);
    }

    err = SetPcd(h_FmPort, p_PcdParams);
    if (err)
    {
        if (p_FmPort->h_IpReassemblyTree)
        {
            FM_PCD_CcRootDelete(p_FmPort->h_IpReassemblyTree);
            p_FmPort->h_IpReassemblyTree = NULL;
        }
        FmPcdLockUnlockAll(p_FmPort->h_FmPcd);
        RELEASE_LOCK(p_FmPort->lock);
        RETURN_ERROR(MAJOR, err, NO_MSG);
    }

    if ((p_FmPort->pcdEngines & FM_PCD_PRS) &&
        (p_PcdParams->p_PrsParams->includeInPrsStatistics))
    {
        err = FmPcdPrsIncludePortInStatistics(p_FmPort->h_FmPcd, p_FmPort->hardwarePortId, TRUE);
        if (err)
        {
            DeletePcd(p_FmPort);
            if (p_FmPort->h_IpReassemblyTree)
            {
                FM_PCD_CcRootDelete(p_FmPort->h_IpReassemblyTree);
                p_FmPort->h_IpReassemblyTree = NULL;
            }
            FmPcdLockUnlockAll(p_FmPort->h_FmPcd);
            RELEASE_LOCK(p_FmPort->lock);
            RETURN_ERROR(MAJOR, err, NO_MSG);
        }
        p_FmPort->includeInPrsStatistics = TRUE;
    }

    FmPcdIncNetEnvOwners(p_FmPort->h_FmPcd, p_FmPort->netEnvId);

    if (FmPcdIsAdvancedOffloadSupported(p_FmPort->h_FmPcd))
    {
        memset(&fmPortGetSetCcParams, 0, sizeof(t_FmPortGetSetCcParams));

        if (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
        {
#ifdef FM_KG_ERASE_FLOW_ID_ERRATA_FMAN_SW004
            if ((p_FmPort->fmRevInfo.majorRev < 6) &&
                (p_FmPort->pcdEngines & FM_PCD_KG))
            {
                int i;
                for (i = 0; i<p_PcdParams->p_KgParams->numOfSchemes; i++)
                    /* The following function must be locked */
                    FmPcdKgCcGetSetParams(p_FmPort->h_FmPcd,
                                          p_PcdParams->p_KgParams->h_Schemes[i],
                                          UPDATE_KG_NIA_CC_WA,
                                          0);
            }
#endif /* FM_KG_ERASE_FLOW_ID_ERRATA_FMAN_SW004 */

            /* Set post-bmi-fetch nia */
            p_FmPort->savedBmiNia &= BMI_RFNE_FDCS_MASK;
            p_FmPort->savedBmiNia |= (NIA_FM_CTL_AC_POST_BMI_FETCH | NIA_ENG_FM_CTL);

            /* Set pre-bmi-fetch nia */
            fmPortGetSetCcParams.setCcParams.type = UPDATE_NIA_PNDN;
#if (DPAA_VERSION >= 11)
            fmPortGetSetCcParams.setCcParams.nia = (NIA_FM_CTL_AC_PRE_BMI_FETCH_FULL_FRAME | NIA_ENG_FM_CTL);
#else
            fmPortGetSetCcParams.setCcParams.nia = (NIA_FM_CTL_AC_PRE_BMI_FETCH_HEADER | NIA_ENG_FM_CTL);
#endif /* (DPAA_VERSION >= 11) */
            if ((err = FmPortGetSetCcParams(p_FmPort, &fmPortGetSetCcParams)) != E_OK)
            {
                DeletePcd(p_FmPort);
                if (p_FmPort->h_IpReassemblyTree)
                {
                    FM_PCD_CcRootDelete(p_FmPort->h_IpReassemblyTree);
                    p_FmPort->h_IpReassemblyTree = NULL;
                }
                FmPcdLockUnlockAll(p_FmPort->h_FmPcd);
                RELEASE_LOCK(p_FmPort->lock);
                RETURN_ERROR(MAJOR, err, NO_MSG);
            }
        }

        FmPcdLockUnlockAll(p_FmPort->h_FmPcd);

        /* Set pop-to-next-step nia */
#if (DPAA_VERSION == 10)
        if (p_FmPort->fmRevInfo.majorRev < 6)
        {
            fmPortGetSetCcParams.setCcParams.type = UPDATE_NIA_PNEN;
            fmPortGetSetCcParams.setCcParams.nia = NIA_FM_CTL_AC_POP_TO_N_STEP | NIA_ENG_FM_CTL;
        }
        else
        {
#endif /* (DPAA_VERSION == 10) */
        fmPortGetSetCcParams.getCcParams.type = GET_NIA_FPNE;
#if (DPAA_VERSION == 10)
        }
#endif /* (DPAA_VERSION == 10) */
        if ((err = FmPortGetSetCcParams(h_FmPort, &fmPortGetSetCcParams)) != E_OK)
        {
            DeletePcd(p_FmPort);
            if (p_FmPort->h_IpReassemblyTree)
            {
                FM_PCD_CcRootDelete(p_FmPort->h_IpReassemblyTree);
                p_FmPort->h_IpReassemblyTree = NULL;
            }
            RELEASE_LOCK(p_FmPort->lock);
            RETURN_ERROR(MAJOR, err, NO_MSG);
        }

        /* Set post-bmi-prepare-to-enq nia */
        fmPortGetSetCcParams.setCcParams.type = UPDATE_NIA_FENE;
        fmPortGetSetCcParams.setCcParams.nia = (NIA_FM_CTL_AC_POST_BMI_ENQ | NIA_ENG_FM_CTL);
        if ((err = FmPortGetSetCcParams(h_FmPort, &fmPortGetSetCcParams)) != E_OK)
        {
            DeletePcd(p_FmPort);
            if (p_FmPort->h_IpReassemblyTree)
            {
                FM_PCD_CcRootDelete(p_FmPort->h_IpReassemblyTree);
                p_FmPort->h_IpReassemblyTree = NULL;
            }
            RELEASE_LOCK(p_FmPort->lock);
            RETURN_ERROR(MAJOR, err, NO_MSG);
        }

        if (p_FmPort->h_IpReassemblyManip)
        {
#if (DPAA_VERSION == 10)
            if (p_FmPort->fmRevInfo.majorRev < 6)
            {
                /* Overwrite post-bmi-prepare-to-enq nia */
                fmPortGetSetCcParams.setCcParams.type = UPDATE_NIA_FENE;
                fmPortGetSetCcParams.setCcParams.nia = (NIA_FM_CTL_AC_POST_BMI_ENQ_ORR | NIA_ENG_FM_CTL | NIA_ORDER_RESTOR);
                fmPortGetSetCcParams.setCcParams.overwrite = TRUE;
            }
            else
            {
#endif /* (DPAA_VERSION == 10) */
            /* Set the ORR bit (for order-restoration) */
            fmPortGetSetCcParams.setCcParams.type = UPDATE_NIA_FPNE;
            fmPortGetSetCcParams.setCcParams.nia = fmPortGetSetCcParams.getCcParams.nia | NIA_ORDER_RESTOR;
#if (DPAA_VERSION == 10)
            }
#endif /* (DPAA_VERSION == 10) */
            if ((err = FmPortGetSetCcParams(h_FmPort, &fmPortGetSetCcParams)) != E_OK)
            {
                DeletePcd(p_FmPort);
                if (p_FmPort->h_IpReassemblyTree)
                {
                    FM_PCD_CcRootDelete(p_FmPort->h_IpReassemblyTree);
                    p_FmPort->h_IpReassemblyTree = NULL;
                }
                RELEASE_LOCK(p_FmPort->lock);
                RETURN_ERROR(MAJOR, err, NO_MSG);
            }
        }
    }
    else
        FmPcdLockUnlockAll(p_FmPort->h_FmPcd);

#if (DPAA_VERSION >= 11)
    {
        t_FmPcdCtrlParamsPage   *p_ParamsPage;

        memset(&fmPortGetSetCcParams, 0, sizeof(t_FmPortGetSetCcParams));

        fmPortGetSetCcParams.setCcParams.type = UPDATE_NIA_CMNE;
        if (FmPcdIsAdvancedOffloadSupported(p_FmPort->h_FmPcd))
            fmPortGetSetCcParams.setCcParams.nia = NIA_FM_CTL_AC_POP_TO_N_STEP | NIA_ENG_FM_CTL;
        else
            fmPortGetSetCcParams.setCcParams.nia = NIA_FM_CTL_AC_NO_IPACC_POP_TO_N_STEP | NIA_ENG_FM_CTL;
        if ((err = FmPortGetSetCcParams(h_FmPort, &fmPortGetSetCcParams)) != E_OK)
        {
            DeletePcd(p_FmPort);
            if (p_FmPort->h_IpReassemblyTree)
            {
                FM_PCD_CcRootDelete(p_FmPort->h_IpReassemblyTree);
                p_FmPort->h_IpReassemblyTree = NULL;
            }
            RELEASE_LOCK(p_FmPort->lock);
            RETURN_ERROR(MAJOR, err, NO_MSG);
         }
 
        FmPortSetGprFunc(p_FmPort, e_FM_PORT_GPR_MURAM_PAGE, (void**)&p_ParamsPage);
        ASSERT_COND(p_ParamsPage);

        if (FmPcdIsAdvancedOffloadSupported(p_FmPort->h_FmPcd))
            WRITE_UINT32(p_ParamsPage->misc, GET_UINT32(p_ParamsPage->misc) | FM_CTL_PARAMS_PAGE_OFFLOAD_SUPPORT_EN);

        if (p_FmPort->h_IpReassemblyManip)
        {
            if (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
                WRITE_UINT32(p_ParamsPage->discardMask,
                             GET_UINT32(p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofsdm));
            else
                WRITE_UINT32(p_ParamsPage->discardMask,
                             GET_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfsdm));
        }
#ifdef FM_ERROR_VSP_NO_MATCH_SW006
        if (p_FmPort->vspe)
            WRITE_UINT32(p_ParamsPage->misc,
                         GET_UINT32(p_ParamsPage->misc) | (p_FmPort->dfltRelativeId & FM_CTL_PARAMS_PAGE_ERROR_VSP_MASK));
#endif /* FM_ERROR_VSP_NO_MATCH_SW006 */
    }
#endif /* (DPAA_VERSION >= 11) */

    err = AttachPCD(h_FmPort);
    if (err)
    {
        DeletePcd(p_FmPort);
        if (p_FmPort->h_IpReassemblyTree)
        {
            FM_PCD_CcRootDelete(p_FmPort->h_IpReassemblyTree);
            p_FmPort->h_IpReassemblyTree = NULL;
        }
        RELEASE_LOCK(p_FmPort->lock);
        RETURN_ERROR(MAJOR, err, NO_MSG);
    }

    RELEASE_LOCK(p_FmPort->lock);

    return err;
}

t_Error FM_PORT_DeletePCD(t_Handle h_FmPort)
{
    t_FmPort                                *p_FmPort = (t_FmPort*)h_FmPort;
    t_Error                                 err = E_OK;

    SANITY_CHECK_RETURN_ERROR(h_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);

    if (p_FmPort->imEn)
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for non-independant mode ports only"));

    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_RX) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_OH_OFFLINE_PARSING))
        RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx and offline parsing ports only"));

    if (!TRY_LOCK(p_FmPort->h_Spinlock, &p_FmPort->lock))
    {
         DBG(TRACE, ("FM Port Try Lock - BUSY"));
         return ERROR_CODE(E_BUSY);
    }

    err = DetachPCD(h_FmPort);
    if (err)
    {
        RELEASE_LOCK(p_FmPort->lock);
        RETURN_ERROR(MAJOR, err, NO_MSG);
    }

    FmPcdDecNetEnvOwners(p_FmPort->h_FmPcd, p_FmPort->netEnvId);

    /* we do it anyway, instead of checking if included */
    if ((p_FmPort->pcdEngines & FM_PCD_PRS) &&
        p_FmPort->includeInPrsStatistics)
    {
        FmPcdPrsIncludePortInStatistics(p_FmPort->h_FmPcd, p_FmPort->hardwarePortId, FALSE);
        p_FmPort->includeInPrsStatistics = FALSE;
    }

    if (!FmPcdLockTryLockAll(p_FmPort->h_FmPcd))
    {
        RELEASE_LOCK(p_FmPort->lock);
        DBG(TRACE, ("Try LockAll - BUSY"));
        return ERROR_CODE(E_BUSY);
    }

    err = DeletePcd(h_FmPort);
    FmPcdLockUnlockAll(p_FmPort->h_FmPcd);
    if (err)
    {
        RELEASE_LOCK(p_FmPort->lock);
        RETURN_ERROR(MAJOR, err, NO_MSG);
    }

    if (p_FmPort->h_IpReassemblyTree)
    {
        err = FM_PCD_CcRootDelete(p_FmPort->h_IpReassemblyTree);
        if (err)
        {
            RELEASE_LOCK(p_FmPort->lock);
            RETURN_ERROR(MAJOR, err, NO_MSG);
        }
        p_FmPort->h_IpReassemblyTree = NULL;
    }
    RELEASE_LOCK(p_FmPort->lock);

    return err;
}

t_Error  FM_PORT_PcdKgBindSchemes (t_Handle h_FmPort, t_FmPcdPortSchemesParams *p_PortScheme)
{
    t_FmPort                                *p_FmPort = (t_FmPort*)h_FmPort;
    t_FmPcdKgInterModuleBindPortToSchemes   schemeBind;
    t_Error                                 err = E_OK;
    uint32_t                                tmpScmVec=0;
    int                                     i;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->pcdEngines & FM_PCD_KG , E_INVALID_STATE);

    schemeBind.netEnvId = p_FmPort->netEnvId;
    schemeBind.hardwarePortId = p_FmPort->hardwarePortId;
    schemeBind.numOfSchemes = p_PortScheme->numOfSchemes;
    schemeBind.useClsPlan = p_FmPort->useClsPlan;
    for (i=0; i<schemeBind.numOfSchemes; i++)
    {
        schemeBind.schemesIds[i] = FmPcdKgGetSchemeId(p_PortScheme->h_Schemes[i]);
        /* build vector */
        tmpScmVec |= 1 << (31 - (uint32_t)schemeBind.schemesIds[i]);
    }

    if (!TRY_LOCK(p_FmPort->h_Spinlock, &p_FmPort->lock))
    {
         DBG(TRACE, ("FM Port Try Lock - BUSY"));
         return ERROR_CODE(E_BUSY);
    }

    err = FmPcdKgBindPortToSchemes(p_FmPort->h_FmPcd, &schemeBind);
    if (err == E_OK)
        p_FmPort->schemesPerPortVector |= tmpScmVec;

#ifdef FM_KG_ERASE_FLOW_ID_ERRATA_FMAN_SW004
    if ((FmPcdIsAdvancedOffloadSupported(p_FmPort->h_FmPcd)) &&
        (p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING) &&
        (p_FmPort->fmRevInfo.majorRev < 6))
    {
        for (i=0; i<p_PortScheme->numOfSchemes; i++)
            FmPcdKgCcGetSetParams(p_FmPort->h_FmPcd, p_PortScheme->h_Schemes[i], UPDATE_KG_NIA_CC_WA, 0);
    }
#endif /* FM_KG_ERASE_FLOW_ID_ERRATA_FMAN_SW004 */

    RELEASE_LOCK(p_FmPort->lock);

    return err;
}

t_Error FM_PORT_PcdKgUnbindSchemes (t_Handle h_FmPort, t_FmPcdPortSchemesParams *p_PortScheme)
{
    t_FmPort                                *p_FmPort = (t_FmPort*)h_FmPort;
    t_FmPcdKgInterModuleBindPortToSchemes   schemeBind;
    t_Error                                 err = E_OK;
    uint32_t                                tmpScmVec=0;
    int                                     i;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->pcdEngines & FM_PCD_KG , E_INVALID_STATE);

    schemeBind.netEnvId = p_FmPort->netEnvId;
    schemeBind.hardwarePortId = p_FmPort->hardwarePortId;
    schemeBind.numOfSchemes = p_PortScheme->numOfSchemes;
    for (i=0; i<schemeBind.numOfSchemes; i++)
    {
        schemeBind.schemesIds[i] = FmPcdKgGetSchemeId(p_PortScheme->h_Schemes[i]);
        /* build vector */
        tmpScmVec |= 1 << (31 - (uint32_t)schemeBind.schemesIds[i]);
    }

    if (!TRY_LOCK(p_FmPort->h_Spinlock, &p_FmPort->lock))
    {
         DBG(TRACE, ("FM Port Try Lock - BUSY"));
         return ERROR_CODE(E_BUSY);
    }

    err = FmPcdKgUnbindPortToSchemes(p_FmPort->h_FmPcd, &schemeBind);
    if (err == E_OK)
        p_FmPort->schemesPerPortVector &= ~tmpScmVec;
    RELEASE_LOCK(p_FmPort->lock);

    return err;
}

t_Error FM_PORT_PcdPrsModifyStartOffset (t_Handle h_FmPort, t_FmPcdPrsStart *p_FmPcdPrsStart)
{
    t_FmPort            *p_FmPort = (t_FmPort*)h_FmPort;
    volatile uint32_t   *p_BmiPrsStartOffset = NULL;
    volatile uint32_t   *p_BmiNia = NULL;
    uint32_t            tmpReg;
    uint8_t             hdrNum;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->pcdEngines & FM_PCD_PRS , E_INVALID_STATE);

    switch (p_FmPort->portType)
    {
        case (e_FM_PORT_TYPE_RX_10G):
        case (e_FM_PORT_TYPE_RX):
            p_BmiPrsStartOffset = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rpso;
            p_BmiNia = &p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rfne;
            tmpReg = GET_UINT32(*p_BmiNia) & BMI_RFNE_FDCS_MASK;
            break;
        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
            p_BmiPrsStartOffset = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_opso;
            p_BmiNia = &p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ofne;
            tmpReg = 0;
            break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("available for Rx and offline parsing ports only"));
    }

    /* check that current NIA is BMI to BMI */
    if ((GET_UINT32(*p_BmiNia) & ~BMI_RFNE_FDCS_MASK) !=
        GET_NIA_BMI_AC_ENQ_FRAME(p_FmPort->h_FmPcd))
            RETURN_ERROR(MAJOR, E_INVALID_OPERATION, ("may be called only for ports in BMI-to-BMI state."));

    if (!TRY_LOCK(p_FmPort->h_Spinlock, &p_FmPort->lock))
    {
         DBG(TRACE, ("FM Port Try Lock - BUSY"));
         return ERROR_CODE(E_BUSY);
    }

    /* set the first header */
    GET_PRS_HDR_NUM(hdrNum, p_FmPcdPrsStart->firstPrsHdr);
    if ((hdrNum == ILLEGAL_HDR_NUM) || (hdrNum == NO_HDR_NUM))
    {
        RELEASE_LOCK(p_FmPort->lock);
        RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("Unsupported header."));
    }
    WRITE_UINT32(*p_BmiNia, (uint32_t)(NIA_ENG_PRS | (uint32_t)hdrNum | tmpReg));

    /* set start parsing offset */
    WRITE_UINT32(*p_BmiPrsStartOffset,
                 (uint32_t)(p_FmPcdPrsStart->parsingOffset +
                            p_FmPort->internalBufferOffset));
    RELEASE_LOCK(p_FmPort->lock);

    return E_OK;
}

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
t_Error FM_PORT_DumpRegs(t_Handle h_FmPort)
{
    t_FmPort            *p_FmPort = (t_FmPort*)h_FmPort;
    t_Error             err = E_OK;
    char                arr[20];
    uint8_t             flag;
    int                 i=0;

    DECLARE_DUMP;

    SANITY_CHECK_RETURN_ERROR(h_FmPort, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_FmPort->p_FmPortDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortQmiRegs, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPort->p_FmPortBmiRegs, E_INVALID_HANDLE);

    memset(arr, 0, sizeof(arr));
    switch (p_FmPort->portType)
    {
        case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
            strcpy(arr, "OFFLINE-PARSING");
            flag = 0;
            break;
        case (e_FM_PORT_TYPE_OH_HOST_COMMAND):
            strcpy(arr, "HOST-COMMAND");
            flag = 0;
            break;
        case (e_FM_PORT_TYPE_RX):
            strcpy(arr, "RX");
            flag = 1;
            break;
        case (e_FM_PORT_TYPE_RX_10G):
            strcpy(arr, "RX-10G");
            flag = 1;
            break;
        case (e_FM_PORT_TYPE_TX):
            strcpy(arr, "TX");
            flag = 2;
            break;
        case (e_FM_PORT_TYPE_TX_10G):
            strcpy(arr, "TX-10G");
            flag = 2;
            break;
        default:
            return ERROR_CODE(E_INVALID_VALUE);
    }

    DUMP_TITLE(NULL,
               ("FMan-Port (%s #%d) registers:",
                arr, p_FmPort->portId));

    err = FmDumpPortRegs(p_FmPort->h_Fm, p_FmPort->hardwarePortId);
    if (err)
        RETURN_ERROR(MAJOR, err, NO_MSG);

    DUMP_TITLE(p_FmPort->p_FmPortBmiRegs, ("Bmi Port Regs"));

    switch (flag)
    {
        case (0):

        DUMP_SUBTITLE(("\n"));
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ocfg);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ost);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_oda);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_oicp);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ofdne);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ofne);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ofca);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ofpne);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_opso);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_opp);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_occb);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_oim);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ofp);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ofed);

        DUMP_TITLE(&(p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_oprai), ("fmbm_oprai"));
        DUMP_SUBSTRUCT_ARRAY(i, FM_PORT_PRS_RESULT_NUM_OF_WORDS)
        {
            DUMP_MEMORY(&(p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_oprai[i]), sizeof(uint32_t));
        }
        DUMP_SUBTITLE(("\n"));
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ofqid );
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_oefqid);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ofsdm );
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ofsem );
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ofene );
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_orlmts);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_orlmt);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ocmne);
        {
#ifndef FM_NO_OP_OBSERVED_POOLS
            if (p_FmPort->fmRevInfo.majorRev == 4)
            {
                DUMP_TITLE(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_oebmpi, ("fmbm_oebmpi"));

                DUMP_SUBSTRUCT_ARRAY(i, FM_PORT_MAX_NUM_OF_OBSERVED_EXT_POOLS)
                {
                    DUMP_MEMORY(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_oebmpi[i], sizeof(uint32_t));
                }
                DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ocgm);
            }
#endif /* !FM_NO_OP_OBSERVED_POOLS */
        }

        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ostc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ofrc );
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ofdc );
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ofledc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ofufdc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_offc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ofwdc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ofldec);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_opc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_opcp);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_occn);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_otuc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_oduc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ofuc);
        DUMP_TITLE(&(p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_odcfg), ("fmbm_odcfg"));
        DUMP_SUBSTRUCT_ARRAY(i, 3)
        {
            DUMP_MEMORY(&(p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_odcfg[i]), sizeof(uint32_t));
        }
        DUMP_SUBTITLE(("\n"));
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs,fmbm_ogpr);
        break;
    case (1):
        DUMP_SUBTITLE(("\n"));
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rcfg);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rst);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rda);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rfp);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_reth);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rfed);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_ricp);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rebm);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rfne);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rfca);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rfpne);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rpso);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rpp);

        DUMP_TITLE(&(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rprai), ("fmbm_rprai"));
        DUMP_SUBSTRUCT_ARRAY(i, FM_PORT_PRS_RESULT_NUM_OF_WORDS)
        {
            DUMP_MEMORY(&(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rprai[i]), sizeof(uint32_t));
        }
        DUMP_SUBTITLE(("\n"));
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rfqid);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_refqid);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rfsdm);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rfsem);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rfene);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rcmne);
        DUMP_TITLE(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_ebmpi, ("fmbm_ebmpi"));
        DUMP_SUBSTRUCT_ARRAY(i, FM_PORT_MAX_NUM_OF_EXT_POOLS)
        {
            DUMP_MEMORY(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_ebmpi[i], sizeof(uint32_t));
        }
        DUMP_TITLE(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_acnt, ("fmbm_acnt"));
        DUMP_SUBSTRUCT_ARRAY(i, FM_PORT_MAX_NUM_OF_EXT_POOLS)
        {
            DUMP_MEMORY(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_acnt[i], sizeof(uint32_t));
        }
        DUMP_TITLE(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rcgm, ("fmbm_rcgm"));
        DUMP_SUBSTRUCT_ARRAY(i, FM_PORT_NUM_OF_CONGESTION_GRPS/32)
        {
            DUMP_MEMORY(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rcgm[i], sizeof(uint32_t));
        }
        DUMP_SUBTITLE(("\n"));
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rmpd);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rstc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rfrc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rfbc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rlfc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rffc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rfcd);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rfldec);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rodc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rpc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rpcp);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rccn);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rtuc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rrquc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rduc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rfuc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rpac);
        DUMP_TITLE(&(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rdcfg), ("fmbm_rdcfg"));
        DUMP_SUBSTRUCT_ARRAY(i, 3)
        {
            DUMP_MEMORY(&(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rdcfg[i]), sizeof(uint32_t));
        }
        DUMP_SUBTITLE(("\n"));
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs,fmbm_rgpr);
        break;
    case (2):

        DUMP_SUBTITLE(("\n"));
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tcfg);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tst);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tda);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tfp);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tfed);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_ticp);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tfdne);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tfca);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tcfqid);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tfeqid);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tfene);
#if (DPAA_VERSION >= 11)
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tfne);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tcmne);
#endif /* (DPAA_VERSION >= 11) */
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_trlmts);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_trlmt);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tstc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tfrc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tfdc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tfledc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tfufdc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tpc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tpcp);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tccn);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_ttuc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_ttcquc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tduc);
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tfuc);
        DUMP_TITLE(&(p_FmPort->p_FmPortBmiRegs->txPortBmiRegs.fmbm_tdcfg), ("fmbm_tdcfg"));
        DUMP_SUBSTRUCT_ARRAY(i, 3)
        {
            DUMP_MEMORY(&(p_FmPort->p_FmPortBmiRegs->txPortBmiRegs.fmbm_tdcfg[i]), sizeof(uint32_t));
        }
        DUMP_SUBTITLE(("\n"));
        DUMP_VAR(&p_FmPort->p_FmPortBmiRegs->txPortBmiRegs,fmbm_tgpr);
        break;

   default:
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid flag"));
    }

    DUMP_TITLE(p_FmPort->p_FmPortQmiRegs, ("Qmi Port Regs"));

    DUMP_VAR(p_FmPort->p_FmPortQmiRegs,fmqm_pnc);
    DUMP_VAR(p_FmPort->p_FmPortQmiRegs,fmqm_pns);
    DUMP_VAR(p_FmPort->p_FmPortQmiRegs,fmqm_pnts);
    DUMP_VAR(p_FmPort->p_FmPortQmiRegs,fmqm_pnen);
    DUMP_VAR(p_FmPort->p_FmPortQmiRegs,fmqm_pnetfc);

    if (flag !=1)
    {
        DUMP_VAR(&p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs,fmqm_pndn);
        DUMP_VAR(&p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs,fmqm_pndc);
        DUMP_VAR(&p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs,fmqm_pndtfc);
        DUMP_VAR(&p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs,fmqm_pndfdc);
        DUMP_VAR(&p_FmPort->p_FmPortQmiRegs->nonRxQmiRegs,fmqm_pndcc);
    }

    return E_OK;
}
#endif /* (defined(DEBUG_ERRORS) && ... */

t_Error FM_PORT_AddCongestionGrps(t_Handle h_FmPort, t_FmPortCongestionGrps *p_CongestionGrps)
{
    t_FmPort            *p_FmPort = (t_FmPort*)h_FmPort;
    bool                tmpArray[FM_PORT_NUM_OF_CONGESTION_GRPS], opPort;
    uint8_t             priorityTmpArray[FM_PORT_NUM_OF_CONGESTION_GRPS];
    int                 i;
    uint8_t             mod;
    uint32_t            tmpReg = 0;
#if (DPAA_VERSION >= 11)
    int                 j;
    t_Error             err;
#endif /* (DPAA_VERSION >= 11) */

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);

    /* un-necessary check of the indexes; probably will be needed in the future when there
       will be more CGs available ....
    for (i=0; i<p_CongestionGrps->numOfCongestionGrpsToConsider; i++)
        if (p_CongestionGrps->congestionGrpsToConsider[i] >= FM_PORT_NUM_OF_CONGESTION_GRPS)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("CG id!"));
    */

#ifdef FM_NO_OP_OBSERVED_CGS
    if ((p_FmPort->fmRevInfo.majorRev != 4) &&
        (p_FmPort->fmRevInfo.majorRev < 6))
    {
        if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) &&
            (p_FmPort->portType != e_FM_PORT_TYPE_RX))
            RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("Available for Rx ports only"));
    }
    else
#endif /* FM_NO_OP_OBSERVED_CGS */
    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_RX) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_OH_OFFLINE_PARSING))
        RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("Available for Rx & OP ports only"));

    opPort = (bool)((p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING) ? TRUE:FALSE);

    /* to minimize memory access (groups may belong to the same regsiter, and may
       be out of order), we first collect all information into a 256 booleans array,
       representing each possible group. */

    memset(&tmpArray, 0, FM_PORT_NUM_OF_CONGESTION_GRPS*sizeof(bool));
    memset(&priorityTmpArray, 0, FM_PORT_NUM_OF_CONGESTION_GRPS*sizeof(uint8_t));

    for (i=0; i<p_CongestionGrps->numOfCongestionGrpsToConsider; i++)
    {
        tmpArray[p_CongestionGrps->congestionGrpsToConsider[i]] = TRUE;

#if (DPAA_VERSION >= 11)
        for (j=0;j<FM_MAX_NUM_OF_PFC_PRIORITIES;j++)
            if (p_CongestionGrps->pfcPrioritiesEn[i][j])
                priorityTmpArray[p_CongestionGrps->congestionGrpsToConsider[i]] |= (0x01 <<(FM_MAX_NUM_OF_PFC_PRIORITIES-j-1));
#endif /* (DPAA_VERSION >= 11) */
    }

    for (i=0; i<FM_PORT_NUM_OF_CONGESTION_GRPS; i++)
    {
        mod = (uint8_t)(i%32);
        /* each 32 congestion groups are represented by a register */
        if (mod == 0) /* first in a 32 bunch of congestion groups, get the currest register state  */
            tmpReg = opPort ? GET_UINT32(p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ocgm):
                              GET_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rcgm[FM_PORT_CG_REG_NUM(i)]);

        /* set in the register, the bit representing the relevant congestion group. */

        if (tmpArray[i])
        {
            tmpReg |=  (0x00000001 << (uint32_t)mod);

#if (DPAA_VERSION >= 11)
            err = FmSetCongestionGroupPFCpriority(p_FmPort->h_Fm,i,priorityTmpArray[i]);
            if (err)
                return err;
#endif /* (DPAA_VERSION >= 11) */
        }

        if (mod == 31) /* last in a 32 bunch of congestion groups - write the corresponding register */
        {
            if (opPort)
                WRITE_UINT32(p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ocgm, tmpReg);
            else
                WRITE_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rcgm[FM_PORT_CG_REG_NUM(i)], tmpReg);
        }
    }

    return  E_OK;
}

t_Error FM_PORT_RemoveCongestionGrps(t_Handle h_FmPort, t_FmPortCongestionGrps *p_CongestionGrps)
{
    t_FmPort            *p_FmPort = (t_FmPort*)h_FmPort;
    bool                tmpArray[FM_PORT_NUM_OF_CONGESTION_GRPS], opPort;
    int                 i;
    uint8_t             mod;
    uint32_t            tmpReg = 0;

    SANITY_CHECK_RETURN_ERROR(p_FmPort, E_INVALID_HANDLE);

    /* un-necessary check of the indexes; probably will be needed in the future when there
       will be more CGs available ....
    for (i=0; i<p_CongestionGrps->numOfCongestionGrpsToConsider; i++)
        if (p_CongestionGrps->congestionGrpsToConsider[i] >= FM_PORT_NUM_OF_CONGESTION_GRPS)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("CG id!"));
    */

#ifdef FM_NO_OP_OBSERVED_CGS
    if ((p_FmPort->fmRevInfo.majorRev != 4) &&
        (p_FmPort->fmRevInfo.majorRev < 6))
    {
        if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) &&
                (p_FmPort->portType != e_FM_PORT_TYPE_RX))
            RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("Available for Rx ports only"));
    }
    else
#endif /* FM_NO_OP_OBSERVED_CGS */
    if ((p_FmPort->portType != e_FM_PORT_TYPE_RX_10G) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_RX) &&
        (p_FmPort->portType != e_FM_PORT_TYPE_OH_OFFLINE_PARSING))
        RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("Available for Rx & OP ports only"));

    opPort = (bool)((p_FmPort->portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING) ? TRUE:FALSE);

    /* to minimize memory access (groups may belong to the same regsiter, and may
    be out of order), we first collect all information into a 256 booleans array,
    representing each possible group. */
    memset(&tmpArray, 0, FM_PORT_NUM_OF_CONGESTION_GRPS*sizeof(bool));
    for (i=0; i<p_CongestionGrps->numOfCongestionGrpsToConsider; i++)
        tmpArray[p_CongestionGrps->congestionGrpsToConsider[i]] = TRUE;

    for (i=0; i<FM_PORT_NUM_OF_CONGESTION_GRPS; i++)
    {
        mod = (uint8_t)(i%32);
        /* each 32 congestion groups are represented by a register */
        if (mod == 0) /* first in a 32 bunch of congestion groups, get the currest register state  */
            tmpReg = opPort ?   GET_UINT32(p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ocgm):
                                GET_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rcgm[FM_PORT_CG_REG_NUM(i)]);

        /* set in the register, the bit representing the relevant congestion group. */
        if (tmpArray[i])
        {
            tmpReg &=  ~(0x00000001 << (uint32_t)mod);

#if (DPAA_VERSION >= 11)
            {
                t_Error err = FmSetCongestionGroupPFCpriority(p_FmPort->h_Fm, i, 0);
                if (err)
                    return err;
            }
#endif /* (DPAA_VERSION >= 11) */
        }
        if (mod == 31) /* last in a 32 bunch of congestion groups - write the corresponding register */
        {
            if (opPort)
                WRITE_UINT32(p_FmPort->p_FmPortBmiRegs->ohPortBmiRegs.fmbm_ocgm, tmpReg);
            else
                WRITE_UINT32(p_FmPort->p_FmPortBmiRegs->rxPortBmiRegs.fmbm_rcgm[FM_PORT_CG_REG_NUM(i)], tmpReg);
        }
    }

    return  E_OK;
}

#if (DPAA_VERSION >= 11)
#endif /* (DPAA_VERSION >= 11) */

