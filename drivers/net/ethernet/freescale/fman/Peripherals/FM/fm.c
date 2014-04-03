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
 @File          fm.c

 @Description   FM driver routines implementation.
*//***************************************************************************/
#include "std_ext.h"
#include "error_ext.h"
#include "xx_ext.h"
#include "string_ext.h"
#include "sprint_ext.h"
#include "debug_ext.h"
#include "fm_muram_ext.h"

#include "fm_common.h"
#include "fm_ipc.h"
#include "fm.h"


/****************************************/
/*       static functions               */
/****************************************/

static volatile bool blockingFlag = FALSE;
static void IpcMsgCompletionCB(t_Handle   h_Fm,
                               uint8_t    *p_Msg,
                               uint8_t    *p_Reply,
                               uint32_t   replyLength,
                               t_Error    status)
{
    UNUSED(h_Fm);UNUSED(p_Msg);UNUSED(p_Reply);UNUSED(replyLength);UNUSED(status);
    blockingFlag = FALSE;
}

static void FreeInitResources(t_Fm *p_Fm)
{
    if (p_Fm->camBaseAddr)
       FM_MURAM_FreeMem(p_Fm->h_FmMuram, UINT_TO_PTR(p_Fm->camBaseAddr));
    if (p_Fm->fifoBaseAddr)
       FM_MURAM_FreeMem(p_Fm->h_FmMuram, UINT_TO_PTR(p_Fm->fifoBaseAddr));
    if (p_Fm->resAddr)
       FM_MURAM_FreeMem(p_Fm->h_FmMuram, UINT_TO_PTR(p_Fm->resAddr));
}

static bool IsFmanCtrlCodeLoaded(t_Fm *p_Fm)
{
    t_FMIramRegs    *p_Iram;

    ASSERT_COND(p_Fm);
    p_Iram = (t_FMIramRegs *)UINT_TO_PTR(p_Fm->baseAddr + FM_MM_IMEM);

    return (bool)!!(GET_UINT32(p_Iram->iready) & IRAM_READY);
}

static t_Error CheckFmParameters(t_Fm *p_Fm)
{
    if (IsFmanCtrlCodeLoaded(p_Fm) && !p_Fm->p_FmDriverParam->resetOnInit)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Old FMan CTRL code is loaded; FM must be reset!"));
#if (DPAA_VERSION < 11)
    if (!p_Fm->p_FmDriverParam->dmaAxiDbgNumOfBeats || (p_Fm->p_FmDriverParam->dmaAxiDbgNumOfBeats > DMA_MODE_MAX_AXI_DBG_NUM_OF_BEATS))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("axiDbgNumOfBeats has to be in the range 1 - %d", DMA_MODE_MAX_AXI_DBG_NUM_OF_BEATS));
#endif /* (DPAA_VERSION < 11) */
    if (p_Fm->p_FmDriverParam->dmaCamNumOfEntries % DMA_CAM_UNITS)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("dmaCamNumOfEntries has to be divisble by %d", DMA_CAM_UNITS));
    if (!p_Fm->p_FmDriverParam->dmaCamNumOfEntries || (p_Fm->p_FmDriverParam->dmaCamNumOfEntries > DMA_MODE_MAX_CAM_NUM_OF_ENTRIES))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("dmaCamNumOfEntries has to be in the range 1 - %d", DMA_MODE_MAX_CAM_NUM_OF_ENTRIES));
    if (p_Fm->p_FmDriverParam->dmaCommQThresholds.assertEmergency > DMA_THRESH_MAX_COMMQ)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("dmaCommQThresholds.assertEmergency can not be larger than %d", DMA_THRESH_MAX_COMMQ));
    if (p_Fm->p_FmDriverParam->dmaCommQThresholds.clearEmergency > DMA_THRESH_MAX_COMMQ)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("dmaCommQThresholds.clearEmergency can not be larger than %d", DMA_THRESH_MAX_COMMQ));
    if (p_Fm->p_FmDriverParam->dmaCommQThresholds.clearEmergency >= p_Fm->p_FmDriverParam->dmaCommQThresholds.assertEmergency)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("dmaCommQThresholds.clearEmergency must be smaller than dmaCommQThresholds.assertEmergency"));
#if (DPAA_VERSION < 11)
    if (p_Fm->p_FmDriverParam->dmaReadBufThresholds.assertEmergency > DMA_THRESH_MAX_BUF)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("dmaReadBufThresholds.assertEmergency can not be larger than %d", DMA_THRESH_MAX_BUF));
    if (p_Fm->p_FmDriverParam->dmaReadBufThresholds.clearEmergency > DMA_THRESH_MAX_BUF)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("dmaReadBufThresholds.clearEmergency can not be larger than %d", DMA_THRESH_MAX_BUF));
    if (p_Fm->p_FmDriverParam->dmaReadBufThresholds.clearEmergency >= p_Fm->p_FmDriverParam->dmaReadBufThresholds.assertEmergency)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("dmaReadBufThresholds.clearEmergency must be smaller than dmaReadBufThresholds.assertEmergency"));
    if (p_Fm->p_FmDriverParam->dmaWriteBufThresholds.assertEmergency > DMA_THRESH_MAX_BUF)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("dmaWriteBufThresholds.assertEmergency can not be larger than %d", DMA_THRESH_MAX_BUF));
    if (p_Fm->p_FmDriverParam->dmaWriteBufThresholds.clearEmergency > DMA_THRESH_MAX_BUF)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("dmaWriteBufThresholds.clearEmergency can not be larger than %d", DMA_THRESH_MAX_BUF));
    if (p_Fm->p_FmDriverParam->dmaWriteBufThresholds.clearEmergency >= p_Fm->p_FmDriverParam->dmaWriteBufThresholds.assertEmergency)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("dmaWriteBufThresholds.clearEmergency must be smaller than dmaWriteBufThresholds.assertEmergency"));
#endif /* (DPAA_VERSION < 11) */
#if (DPAA_VERSION >= 11)
    if ((p_Fm->p_FmDriverParam->dmaDbgCntMode == e_FM_DMA_DBG_CNT_INT_READ_EM)||
            (p_Fm->p_FmDriverParam->dmaDbgCntMode == e_FM_DMA_DBG_CNT_INT_WRITE_EM) ||
            (p_Fm->p_FmDriverParam->dmaDbgCntMode == e_FM_DMA_DBG_CNT_RAW_WAR_PROT))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("dmaDbgCntMode value not supported by this integration."));
    if ((p_Fm->p_FmDriverParam->dmaEmergency.emergencyBusSelect == FM_DMA_MURAM_READ_EMERGENCY)||
            (p_Fm->p_FmDriverParam->dmaEmergency.emergencyBusSelect == FM_DMA_MURAM_WRITE_EMERGENCY))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("emergencyBusSelect value not supported by this integration."));
    if (p_Fm->p_FmDriverParam->dmaStopOnBusError)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("dmaStopOnBusError not supported by this integration."));
#ifdef FM_AID_MODE_NO_TNUM_SW005
    if (p_Fm->p_FmDriverParam->dmaAidMode != e_FM_DMA_AID_OUT_PORT_ID)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("dmaAidMode not supported by this integration."));
#endif /* FM_AID_MODE_NO_TNUM_SW005 */
    if (p_Fm->p_FmDriverParam->dmaAxiDbgNumOfBeats)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("dmaAxiDbgNumOfBeats not supported by this integration."));
#endif /* (DPAA_VERSION >= 11) */

    if (!p_Fm->p_FmStateStruct->fmClkFreq)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("fmClkFreq must be set."));
    if (USEC_TO_CLK(p_Fm->p_FmDriverParam->dmaWatchdog, p_Fm->p_FmStateStruct->fmClkFreq) > DMA_MAX_WATCHDOG)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE,
                     ("dmaWatchdog depends on FM clock. dmaWatchdog(in microseconds) * clk (in Mhz), may not exceed 0x08x", DMA_MAX_WATCHDOG));

#if (DPAA_VERSION >= 11)
    if ((p_Fm->partVSPBase + p_Fm->partNumOfVSPs) > FM_VSP_MAX_NUM_OF_ENTRIES)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("partVSPBase+partNumOfVSPs out of range!!!"));
#endif /* (DPAA_VERSION >= 11) */

    if (p_Fm->p_FmStateStruct->totalFifoSize % BMI_FIFO_UNITS)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("totalFifoSize number has to be divisible by %d", BMI_FIFO_UNITS));
    if (!p_Fm->p_FmStateStruct->totalFifoSize ||
        (p_Fm->p_FmStateStruct->totalFifoSize > BMI_MAX_FIFO_SIZE))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("totalFifoSize number has to be in the range 256 - %d", BMI_MAX_FIFO_SIZE));
    if (!p_Fm->p_FmStateStruct->totalNumOfTasks ||
        (p_Fm->p_FmStateStruct->totalNumOfTasks > BMI_MAX_NUM_OF_TASKS))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("totalNumOfTasks number has to be in the range 1 - %d", BMI_MAX_NUM_OF_TASKS));

#ifdef FM_HAS_TOTAL_DMAS
    if (!p_Fm->p_FmStateStruct->maxNumOfOpenDmas ||
        (p_Fm->p_FmStateStruct->maxNumOfOpenDmas > BMI_MAX_NUM_OF_DMAS))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("maxNumOfOpenDmas number has to be in the range 1 - %d", BMI_MAX_NUM_OF_DMAS));
#endif /* FM_HAS_TOTAL_DMAS */

    if (p_Fm->p_FmDriverParam->thresholds.dispLimit > FPM_MAX_DISP_LIMIT)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("thresholds.dispLimit can't be greater than %d", FPM_MAX_DISP_LIMIT));

    if (!p_Fm->f_Exception)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Exceptions callback not provided"));
    if (!p_Fm->f_BusError)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Exceptions callback not provided"));

#ifdef FM_NO_WATCHDOG
    if ((p_Fm->p_FmStateStruct->revInfo.majorRev == 2) &&
        (p_Fm->p_FmDriverParam->dmaWatchdog))
        RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("watchdog!"));
#endif /* FM_NO_WATCHDOG */

#ifdef FM_ECC_HALT_NO_SYNC_ERRATA_10GMAC_A008
    if ((p_Fm->p_FmStateStruct->revInfo.majorRev < 6) &&
        (p_Fm->p_FmDriverParam->haltOnUnrecoverableEccError))
        RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("HaltOnEccError!"));
#endif /* FM_ECC_HALT_NO_SYNC_ERRATA_10GMAC_A008 */

#ifdef FM_NO_TNUM_AGING
    if ((p_Fm->p_FmStateStruct->revInfo.majorRev != 4) &&
        (p_Fm->p_FmStateStruct->revInfo.majorRev < 6))
        if (p_Fm->tnumAgingPeriod)
        RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("Tnum aging!"));
#endif /* FM_NO_TNUM_AGING */

    /* check that user did not set revision-dependent exceptions */
#ifdef FM_NO_DISPATCH_RAM_ECC
    if ((p_Fm->p_FmStateStruct->revInfo.majorRev != 4) &&
        (p_Fm->p_FmStateStruct->revInfo.majorRev < 6))
        if (p_Fm->p_FmDriverParam->userSetExceptions & FM_EX_BMI_DISPATCH_RAM_ECC)
            RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("exception e_FM_EX_BMI_DISPATCH_RAM_ECC!"));
#endif /* FM_NO_DISPATCH_RAM_ECC */

#ifdef FM_QMI_NO_ECC_EXCEPTIONS
    if (p_Fm->p_FmStateStruct->revInfo.majorRev == 4)
        if (p_Fm->p_FmDriverParam->userSetExceptions & (FM_EX_QMI_SINGLE_ECC | FM_EX_QMI_DOUBLE_ECC))
            RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("exception e_FM_EX_QMI_SINGLE_ECC/e_FM_EX_QMI_DOUBLE_ECC!"));
#endif /* FM_QMI_NO_ECC_EXCEPTIONS */

#ifdef FM_QMI_NO_SINGLE_ECC_EXCEPTION
    if (p_Fm->p_FmStateStruct->revInfo.majorRev >= 6)
        if (p_Fm->p_FmDriverParam->userSetExceptions & FM_EX_QMI_SINGLE_ECC)
            RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("exception e_FM_EX_QMI_SINGLE_ECC!"));
#endif /* FM_QMI_NO_SINGLE_ECC_EXCEPTION */

    return E_OK;
}


static void SendIpcIsr(t_Fm *p_Fm, uint32_t macEvent, uint32_t pendingReg)
{
    ASSERT_COND(p_Fm->guestId == NCSW_MASTER_ID);

    if (p_Fm->intrMng[macEvent].guestId == NCSW_MASTER_ID)
        p_Fm->intrMng[macEvent].f_Isr(p_Fm->intrMng[macEvent].h_SrcHandle);

    /* If the MAC is running on guest-partition and we have IPC session with it,
       we inform him about the event through IPC; otherwise, we ignore the event. */
    else if (p_Fm->h_IpcSessions[p_Fm->intrMng[macEvent].guestId])
    {
        t_Error     err;
        t_FmIpcIsr  fmIpcIsr;
        t_FmIpcMsg  msg;

        memset(&msg, 0, sizeof(msg));
        msg.msgId = FM_GUEST_ISR;
        fmIpcIsr.pendingReg = pendingReg;
        fmIpcIsr.boolErr = FALSE;
        memcpy(msg.msgBody, &fmIpcIsr, sizeof(fmIpcIsr));
        err = XX_IpcSendMessage(p_Fm->h_IpcSessions[p_Fm->intrMng[macEvent].guestId],
                                (uint8_t*)&msg,
                                sizeof(msg.msgId) + sizeof(fmIpcIsr),
                                NULL,
                                NULL,
                                NULL,
                                NULL);
        if (err != E_OK)
            REPORT_ERROR(MINOR, err, NO_MSG);
    }
    else
        DBG(TRACE, ("FM Guest mode, without IPC - can't call ISR!"));
}

static void BmiErrEvent(t_Fm *p_Fm)
{
    uint32_t    event, mask, force;

    event  = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_ievr);
    mask   = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_ier);
    event &= mask;

    /* clear the forced events */
    force = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_ifr);
    if (force & event)
        WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_ifr, force & ~event);

    /* clear the acknowledged events */
    WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_ievr, event);

    if (event & BMI_ERR_INTR_EN_STORAGE_PROFILE_ECC)
        p_Fm->f_Exception(p_Fm->h_App,e_FM_EX_BMI_STORAGE_PROFILE_ECC);
    if (event & BMI_ERR_INTR_EN_LIST_RAM_ECC)
        p_Fm->f_Exception(p_Fm->h_App,e_FM_EX_BMI_LIST_RAM_ECC);
    if (event & BMI_ERR_INTR_EN_STATISTICS_RAM_ECC)
        p_Fm->f_Exception(p_Fm->h_App,e_FM_EX_BMI_STATISTICS_RAM_ECC);
    if (event & BMI_ERR_INTR_EN_DISPATCH_RAM_ECC)
        p_Fm->f_Exception(p_Fm->h_App,e_FM_EX_BMI_DISPATCH_RAM_ECC);
}

static void    QmiErrEvent(t_Fm *p_Fm)
{
    uint32_t    event, mask, force;

    event = GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_eie);
    mask = GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_eien);

    event &= mask;

    /* clear the forced events */
    force = GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_eif);
    if (force & event)
        WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_eif, force & ~event);

    /* clear the acknowledged events */
    WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_eie, event);

    if (event & QMI_ERR_INTR_EN_DOUBLE_ECC)
        p_Fm->f_Exception(p_Fm->h_App,e_FM_EX_QMI_DOUBLE_ECC);
    if (event & QMI_ERR_INTR_EN_DEQ_FROM_DEF)
        p_Fm->f_Exception(p_Fm->h_App,e_FM_EX_QMI_DEQ_FROM_UNKNOWN_PORTID);
}

static void    DmaErrEvent(t_Fm *p_Fm)
{
    uint64_t            addr=0;
    uint32_t            status, mask, tmpReg=0;
    uint8_t             tnum;
    uint8_t             hardwarePortId;
    uint8_t             relativePortId;
    uint16_t            liodn;

    status = GET_UINT32(p_Fm->p_FmDmaRegs->fmdmsr);
    mask = GET_UINT32(p_Fm->p_FmDmaRegs->fmdmmr);

    /* get bus error regs before clearing BER */
    if ((status & DMA_STATUS_BUS_ERR) && (mask & DMA_MODE_BER))
    {
        addr = (uint64_t)GET_UINT32(p_Fm->p_FmDmaRegs->fmdmtal);
        addr |= ((uint64_t)(GET_UINT32(p_Fm->p_FmDmaRegs->fmdmtah)) << 32);

        /* get information about the owner of that bus error */
        tmpReg = GET_UINT32(p_Fm->p_FmDmaRegs->fmdmtcid);
    }

    /* clear set events */
    WRITE_UINT32(p_Fm->p_FmDmaRegs->fmdmsr, status);

    if ((status & DMA_STATUS_BUS_ERR) && (mask & DMA_MODE_BER))
    {
        hardwarePortId = (uint8_t)(((tmpReg & DMA_TRANSFER_PORTID_MASK) >> DMA_TRANSFER_PORTID_SHIFT));
        HW_PORT_ID_TO_SW_PORT_ID(relativePortId, hardwarePortId);
        tnum = (uint8_t)((tmpReg & DMA_TRANSFER_TNUM_MASK) >> DMA_TRANSFER_TNUM_SHIFT);
        liodn = (uint16_t)(tmpReg & DMA_TRANSFER_LIODN_MASK);
        ASSERT_COND(p_Fm->p_FmStateStruct->portsTypes[hardwarePortId] != e_FM_PORT_TYPE_DUMMY);
        p_Fm->f_BusError(p_Fm->h_App,
                         p_Fm->p_FmStateStruct->portsTypes[hardwarePortId],
                         relativePortId,
                         addr,
                         tnum,
                         liodn);
    }
    if (mask & DMA_MODE_ECC)
    {
        if (status & DMA_STATUS_FM_SPDAT_ECC)
            p_Fm->f_Exception(p_Fm->h_App, e_FM_EX_DMA_SINGLE_PORT_ECC);
        if (status & DMA_STATUS_READ_ECC)
            p_Fm->f_Exception(p_Fm->h_App, e_FM_EX_DMA_READ_ECC);
        if (status & DMA_STATUS_SYSTEM_WRITE_ECC)
            p_Fm->f_Exception(p_Fm->h_App, e_FM_EX_DMA_SYSTEM_WRITE_ECC);
        if (status & DMA_STATUS_FM_WRITE_ECC)
            p_Fm->f_Exception(p_Fm->h_App, e_FM_EX_DMA_FM_WRITE_ECC);
    }
}

static void    FpmErrEvent(t_Fm *p_Fm)
{
    uint32_t    event;

    event = GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_ee);

    /* clear the all occurred events */
    WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_ee, event);

    if ((event  & FPM_EV_MASK_DOUBLE_ECC) && (event & FPM_EV_MASK_DOUBLE_ECC_EN))
        p_Fm->f_Exception(p_Fm->h_App,e_FM_EX_FPM_DOUBLE_ECC);
    if ((event  & FPM_EV_MASK_STALL) && (event & FPM_EV_MASK_STALL_EN))
        p_Fm->f_Exception(p_Fm->h_App,e_FM_EX_FPM_STALL_ON_TASKS);
    if ((event  & FPM_EV_MASK_SINGLE_ECC) && (event & FPM_EV_MASK_SINGLE_ECC_EN))
        p_Fm->f_Exception(p_Fm->h_App,e_FM_EX_FPM_SINGLE_ECC);
}

static void    MuramErrIntr(t_Fm *p_Fm)
{
    uint32_t    event, mask;

    event = GET_UINT32(p_Fm->p_FmFpmRegs->fm_rcr);
    mask = GET_UINT32(p_Fm->p_FmFpmRegs->fm_rie);

    ASSERT_COND(event & FPM_RAM_CTL_MURAM_ECC);

    /* clear MURAM event bit */
    /* Prior to V3 this event bit clearing does not work ! ) */
    WRITE_UINT32(p_Fm->p_FmFpmRegs->fm_rcr, event & ~FPM_RAM_CTL_IRAM_ECC);

    ASSERT_COND(event & FPM_RAM_CTL_RAMS_ECC_EN);

    if ((mask & FPM_MURAM_ECC_ERR_EX_EN))
        p_Fm->f_Exception(p_Fm->h_App, e_FM_EX_MURAM_ECC);
}

static void IramErrIntr(t_Fm *p_Fm)
{
    uint32_t    event, mask;

    event = GET_UINT32(p_Fm->p_FmFpmRegs->fm_rcr) ;
    mask = GET_UINT32(p_Fm->p_FmFpmRegs->fm_rie);

    ASSERT_COND(event & FPM_RAM_CTL_IRAM_ECC);

    /* clear the acknowledged events (do not clear IRAM event) */
    /* Prior to V3 this event bit clearing does not work ! ) */
   WRITE_UINT32(p_Fm->p_FmFpmRegs->fm_rcr, event & ~FPM_RAM_CTL_MURAM_ECC);

    ASSERT_COND(event & FPM_RAM_CTL_IRAM_ECC_EN);

    if ((mask & FPM_IRAM_ECC_ERR_EX_EN))
        p_Fm->f_Exception(p_Fm->h_App, e_FM_EX_IRAM_ECC);
}

static void QmiEvent(t_Fm *p_Fm)
{
    uint32_t    event, mask, force;

    event = GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_ie);
    mask = GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_ien);

    event &= mask;

    /* clear the forced events */
    force = GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_if);
    if (force & event)
        WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_if, force & ~event);

    /* clear the acknowledged events */
    WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_ie, event);

    if (event & QMI_INTR_EN_SINGLE_ECC)
        p_Fm->f_Exception(p_Fm->h_App,e_FM_EX_QMI_SINGLE_ECC);
}

static void UnimplementedIsr(t_Handle h_Arg)
{
    UNUSED(h_Arg);

    REPORT_ERROR(MAJOR, E_NOT_SUPPORTED, ("Unimplemented ISR!"));
}

static void UnimplementedFmanCtrlIsr(t_Handle h_Arg, uint32_t event)
{
    UNUSED(h_Arg); UNUSED(event);

    REPORT_ERROR(MAJOR, E_NOT_SUPPORTED, ("Unimplemented FmCtl ISR!"));
}

static void EnableTimeStamp(t_Fm *p_Fm)
{
    uint64_t                fraction;
    uint32_t                integer, tsFrequency, tmpReg;

    ASSERT_COND(p_Fm);
    ASSERT_COND(p_Fm->p_FmStateStruct);
    ASSERT_COND(p_Fm->p_FmStateStruct->count1MicroBit);

    tsFrequency = (uint32_t)(1<<p_Fm->p_FmStateStruct->count1MicroBit); /* in Mhz */

    /* configure timestamp so that bit 8 will count 1 microsecond */
    /* Find effective count rate at TIMESTAMP least significant bits:
       Effective_Count_Rate = 1MHz x 2^8 = 256MHz
       Find frequency ratio between effective count rate and the clock:
       Effective_Count_Rate / CLK e.g. for 600 MHz clock:
       256/600 = 0.4266666... */
    integer = tsFrequency/p_Fm->p_FmStateStruct->fmClkFreq;
    /* we multiply by 2^16 to keep the fraction of the division */
    /* we do not divide back, since we write this value as fraction - see spec */
    fraction = ((tsFrequency << 16) - (integer << 16) * p_Fm->p_FmStateStruct->fmClkFreq) / p_Fm->p_FmStateStruct->fmClkFreq;
    /* we check remainder of the division in order to round up if not integer */
    if (((tsFrequency << 16) - (integer << 16) * p_Fm->p_FmStateStruct->fmClkFreq) % p_Fm->p_FmStateStruct->fmClkFreq)
        fraction++;

    tmpReg = (integer << FPM_TS_INT_SHIFT) | (uint16_t)fraction;
    WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_tsc2, tmpReg);

    /* enable timestamp with original clock */
    WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_tsc1, FPM_TS_CTL_EN);

    p_Fm->p_FmStateStruct->enabledTimeStamp = TRUE;
}

static t_Error ClearIRam(t_Fm *p_Fm)
{
    t_FMIramRegs    *p_Iram;
    int             i;

    ASSERT_COND(p_Fm);
    p_Iram = (t_FMIramRegs *)UINT_TO_PTR(p_Fm->baseAddr + FM_MM_IMEM);

    /* Enable the auto-increment */
    WRITE_UINT32(p_Iram->iadd, IRAM_IADD_AIE);
    while (GET_UINT32(p_Iram->iadd) != IRAM_IADD_AIE) ;

    for (i=0; i < (FM_IRAM_SIZE/4); i++)
        WRITE_UINT32(p_Iram->idata, 0xffffffff);

    WRITE_UINT32(p_Iram->iadd, FM_IRAM_SIZE - 4);
    CORE_MemoryBarrier();
    while (GET_UINT32(p_Iram->idata) != 0xffffffff) ;

    return E_OK;
}

static t_Error LoadFmanCtrlCode(t_Fm *p_Fm)
{
    t_FMIramRegs    *p_Iram;
    int             i;
    uint32_t        tmp;
    uint8_t         compTo16;

    ASSERT_COND(p_Fm);
    p_Iram = (t_FMIramRegs *)UINT_TO_PTR(p_Fm->baseAddr + FM_MM_IMEM);

    /* Enable the auto-increment */
    WRITE_UINT32(p_Iram->iadd, IRAM_IADD_AIE);
    while (GET_UINT32(p_Iram->iadd) != IRAM_IADD_AIE) ;

    for (i=0; i < (p_Fm->p_FmDriverParam->firmware.size / 4); i++)
        WRITE_UINT32(p_Iram->idata, p_Fm->p_FmDriverParam->firmware.p_Code[i]);

    compTo16 = (uint8_t)(p_Fm->p_FmDriverParam->firmware.size % 16);
    if (compTo16)
        for (i=0; i < ((16-compTo16) / 4); i++)
            WRITE_UINT32(p_Iram->idata, 0xffffffff);

    WRITE_UINT32(p_Iram->iadd,p_Fm->p_FmDriverParam->firmware.size-4);
    while (GET_UINT32(p_Iram->iadd) != (p_Fm->p_FmDriverParam->firmware.size-4)) ;

    /* verify that writing has completed */
    while (GET_UINT32(p_Iram->idata) != p_Fm->p_FmDriverParam->firmware.p_Code[(p_Fm->p_FmDriverParam->firmware.size / 4)-1]) ;

    if (p_Fm->p_FmDriverParam->fwVerify)
    {
        WRITE_UINT32(p_Iram->iadd, IRAM_IADD_AIE);
        while (GET_UINT32(p_Iram->iadd) != IRAM_IADD_AIE) ;
        for (i=0; i < (p_Fm->p_FmDriverParam->firmware.size / 4); i++)
        {
            tmp = GET_UINT32(p_Iram->idata);
            if (tmp != p_Fm->p_FmDriverParam->firmware.p_Code[i])
                RETURN_ERROR(MAJOR, E_WRITE_FAILED,
                             ("UCode write error : write 0x%x, read 0x%x",
                              p_Fm->p_FmDriverParam->firmware.p_Code[i],tmp));
        }
        WRITE_UINT32(p_Iram->iadd, 0x0);
    }

    /* Enable patch from IRAM */
    WRITE_UINT32(p_Iram->iready, IRAM_READY);
    XX_UDelay(1000);

    DBG(INFO, ("FMan-Controller code (ver %d.%d.%d) loaded to IRAM.",
               ((uint16_t *)p_Fm->p_FmDriverParam->firmware.p_Code)[2],
               ((uint8_t *)p_Fm->p_FmDriverParam->firmware.p_Code)[6],
               ((uint8_t *)p_Fm->p_FmDriverParam->firmware.p_Code)[7]));

    return E_OK;
}

#ifdef FM_UCODE_NOT_RESET_ERRATA_BUGZILLA6173
static t_Error FwNotResetErratumBugzilla6173WA(t_Fm *p_Fm)
{
    t_FMIramRegs    *p_Iram = (t_FMIramRegs *)UINT_TO_PTR(p_Fm->baseAddr + FM_MM_IMEM);
    uint32_t        tmpReg;

    /* write to IRAM first location the debug instruction */
    WRITE_UINT32(p_Iram->iadd, 0);
    while (GET_UINT32(p_Iram->iadd) != 0) ;
    WRITE_UINT32(p_Iram->idata, FM_FW_DEBUG_INSTRUCTION);

    WRITE_UINT32(p_Iram->iadd, 0);
    while (GET_UINT32(p_Iram->iadd) != 0) ;
    while (GET_UINT32(p_Iram->idata) != FM_FW_DEBUG_INSTRUCTION) ;

    /* Enable patch from IRAM */
    WRITE_UINT32(p_Iram->iready, IRAM_READY);
    CORE_MemoryBarrier();
    XX_UDelay(100);

    /* reset FMAN */
    WRITE_UINT32(p_Fm->p_FmFpmRegs->fm_rstc, FPM_RSTC_FM_RESET);
    CORE_MemoryBarrier();
    XX_UDelay(100);

    /* verify breakpoint debug status register */
    tmpReg = GET_UINT32(*(uint32_t *)UINT_TO_PTR(p_Fm->baseAddr + FM_DEBUG_STATUS_REGISTER_OFFSET));
    if (!tmpReg)
        REPORT_ERROR(MAJOR, E_INVALID_STATE, ("Invalid debug status register value is '0'"));

    /*************************************/
    /* Load FMan-Controller code to IRAM */
    /*************************************/
    if (ClearIRam(p_Fm) != E_OK)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);
    if (p_Fm->p_FmDriverParam->firmware.p_Code &&
        (LoadFmanCtrlCode(p_Fm) != E_OK))
        RETURN_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);
    XX_UDelay(100);

    /* reset FMAN again to start the microcode */
    WRITE_UINT32(p_Fm->p_FmFpmRegs->fm_rstc, FPM_RSTC_FM_RESET);
    CORE_MemoryBarrier();
    XX_UDelay(100);

    if (GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_gs) & QMI_GS_HALT_NOT_BUSY)
    {
        tmpReg = GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_ee);
        /* clear tmpReg event bits in order not to clear standing events */
        tmpReg &= ~(FPM_EV_MASK_DOUBLE_ECC | FPM_EV_MASK_STALL | FPM_EV_MASK_SINGLE_ECC);
        WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_ee, tmpReg | FPM_EV_MASK_RELEASE_FM);
        CORE_MemoryBarrier();
        XX_UDelay(100);
    }

    return E_OK;
}
#endif /* FM_UCODE_NOT_RESET_ERRATA_BUGZILLA6173 */

static void GuestErrorIsr(t_Fm *p_Fm, uint32_t pending)
{
#define FM_G_CALL_1G_MAC_ERR_ISR(_id)   \
do {                                    \
    p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_ERR_1G_MAC0+_id)].f_Isr(p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_ERR_1G_MAC0+_id)].h_SrcHandle);\
} while (0)
#define FM_G_CALL_10G_MAC_ERR_ISR(_id)  \
do {                                    \
    p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_ERR_10G_MAC0+_id)].f_Isr(p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_ERR_10G_MAC0+_id)].h_SrcHandle);\
} while (0)

    /* error interrupts */
    if (pending & ERR_INTR_EN_1G_MAC0)
        FM_G_CALL_1G_MAC_ERR_ISR(0);
    if (pending & ERR_INTR_EN_1G_MAC1)
        FM_G_CALL_1G_MAC_ERR_ISR(1);
    if (pending & ERR_INTR_EN_1G_MAC2)
        FM_G_CALL_1G_MAC_ERR_ISR(2);
    if (pending & ERR_INTR_EN_1G_MAC3)
        FM_G_CALL_1G_MAC_ERR_ISR(3);
    if (pending & ERR_INTR_EN_1G_MAC4)
        FM_G_CALL_1G_MAC_ERR_ISR(4);
    if (pending & ERR_INTR_EN_1G_MAC5)
        FM_G_CALL_1G_MAC_ERR_ISR(5);
    if (pending & ERR_INTR_EN_1G_MAC6)
        FM_G_CALL_1G_MAC_ERR_ISR(6);
    if (pending & ERR_INTR_EN_1G_MAC7)
        FM_G_CALL_1G_MAC_ERR_ISR(7);
    if (pending & ERR_INTR_EN_10G_MAC0)
        FM_G_CALL_10G_MAC_ERR_ISR(0);
    if (pending & ERR_INTR_EN_10G_MAC1)
        FM_G_CALL_10G_MAC_ERR_ISR(1);
}

static void GuestEventIsr(t_Fm *p_Fm, uint32_t pending)
{
#define FM_G_CALL_1G_MAC_ISR(_id)   \
do {                                    \
    p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_1G_MAC0+_id)].f_Isr(p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_1G_MAC0+_id)].h_SrcHandle);\
} while (0)
#define FM_G_CALL_10G_MAC_ISR(_id)   \
do {                                    \
    p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_10G_MAC0+_id)].f_Isr(p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_10G_MAC0+_id)].h_SrcHandle);\
} while (0)

    if (pending & INTR_EN_1G_MAC0)
        FM_G_CALL_1G_MAC_ISR(0);
    if (pending & INTR_EN_1G_MAC1)
        FM_G_CALL_1G_MAC_ISR(1);
    if (pending & INTR_EN_1G_MAC2)
        FM_G_CALL_1G_MAC_ISR(2);
    if (pending & INTR_EN_1G_MAC3)
        FM_G_CALL_1G_MAC_ISR(3);
    if (pending & INTR_EN_1G_MAC4)
        FM_G_CALL_1G_MAC_ISR(4);
    if (pending & INTR_EN_1G_MAC5)
        FM_G_CALL_1G_MAC_ISR(5);
    if (pending & INTR_EN_1G_MAC6)
        FM_G_CALL_1G_MAC_ISR(6);
    if (pending & INTR_EN_1G_MAC7)
        FM_G_CALL_1G_MAC_ISR(7);
    if (pending & INTR_EN_10G_MAC0)
        FM_G_CALL_10G_MAC_ISR(0);
    if (pending & INTR_EN_10G_MAC1)
        FM_G_CALL_10G_MAC_ISR(1);
    if (pending & INTR_EN_TMR)
        p_Fm->intrMng[e_FM_EV_TMR].f_Isr(p_Fm->intrMng[e_FM_EV_TMR].h_SrcHandle);
}

#if (DPAA_VERSION >= 11)
static t_Error SetVSPWindow(t_Handle  h_Fm,
                            uint8_t   hardwarePortId,
                            uint8_t   baseStorageProfile,
                            uint8_t   log2NumOfProfiles)
{
    t_Fm                    *p_Fm = (t_Fm *)h_Fm;
    uint32_t                tmpReg;

    ASSERT_COND(h_Fm);
    ASSERT_COND(hardwarePortId);

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        !p_Fm->p_FmBmiRegs &&
        p_Fm->h_IpcSessions[0])
    {
        t_FmIpcVspSetPortWindow fmIpcVspSetPortWindow;
        t_FmIpcMsg              msg;
        t_Error                 err = E_OK;

        memset(&msg, 0, sizeof(msg));
        memset(&fmIpcVspSetPortWindow, 0, sizeof(t_FmIpcVspSetPortWindow));
        fmIpcVspSetPortWindow.hardwarePortId      = hardwarePortId;
        fmIpcVspSetPortWindow.baseStorageProfile  = baseStorageProfile;
        fmIpcVspSetPortWindow.log2NumOfProfiles   = log2NumOfProfiles;
        msg.msgId                                 = FM_VSP_SET_PORT_WINDOW;
        memcpy(msg.msgBody, &fmIpcVspSetPortWindow, sizeof(t_FmIpcVspSetPortWindow));

        err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                (uint8_t*)&msg,
                                sizeof(msg.msgId),
                                NULL,
                                NULL,
                                NULL,
                                NULL);
        if (err != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
        return E_OK;
    }
    else if (!p_Fm->p_FmBmiRegs)
        RETURN_ERROR(MINOR, E_NOT_SUPPORTED,
                     ("Either IPC or 'baseAddress' is required!"));

    tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_spliodn[hardwarePortId-1]);
    tmpReg |= (uint32_t)((uint32_t)baseStorageProfile & 0x3f) << 16;
    tmpReg |= (uint32_t)log2NumOfProfiles << 28;
    WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_spliodn[hardwarePortId-1], tmpReg);

    return E_OK;
}

static uint8_t AllocVSPsForPartition(t_Handle  h_Fm, uint8_t base, uint8_t numOfProfiles, uint8_t guestId)
{
    t_Fm        *p_Fm = (t_Fm *)h_Fm;
    uint8_t     profilesFound = 0;
    int         i = 0;
    uint32_t    intFlags;

    if (!numOfProfiles)
        return E_OK;

    if ((numOfProfiles > FM_VSP_MAX_NUM_OF_ENTRIES) ||
        (base + numOfProfiles > FM_VSP_MAX_NUM_OF_ENTRIES))
        return (uint8_t)ILLEGAL_BASE;

    if (p_Fm->h_IpcSessions[0])
    {
        t_FmIpcResourceAllocParams  ipcAllocParams;
        t_FmIpcMsg                  msg;
        t_FmIpcReply                reply;
        t_Error                     err;
        uint32_t                    replyLength;

        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        memset(&ipcAllocParams, 0, sizeof(t_FmIpcResourceAllocParams));
        ipcAllocParams.guestId         = p_Fm->guestId;
        ipcAllocParams.num             = p_Fm->partNumOfVSPs;
        ipcAllocParams.base            = p_Fm->partVSPBase;
        msg.msgId                              = FM_VSP_ALLOC;
        memcpy(msg.msgBody, &ipcAllocParams, sizeof(t_FmIpcResourceAllocParams));
        replyLength = sizeof(uint32_t) + sizeof(uint8_t);
        err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                (uint8_t*)&msg,
                                sizeof(msg.msgId) + sizeof(t_FmIpcResourceAllocParams),
                                (uint8_t*)&reply,
                                &replyLength,
                                NULL,
                                NULL);
        if ((err != E_OK) ||
            (replyLength != (sizeof(uint32_t) + sizeof(uint8_t))))
            RETURN_ERROR(MAJOR, err, NO_MSG);
        else
            memcpy((uint8_t*)&p_Fm->partVSPBase, reply.replyBody, sizeof(uint8_t));
        if (p_Fm->partVSPBase == ILLEGAL_BASE)
            RETURN_ERROR(MAJOR, err, NO_MSG);
    }
    if (p_Fm->guestId != NCSW_MASTER_ID)
    {
        DBG(WARNING, ("FM Guest mode, without IPC - can't validate VSP range!"));
        return (uint8_t)ILLEGAL_BASE;
    }

    intFlags = XX_LockIntrSpinlock(p_Fm->h_Spinlock);
    for (i = base; i < base + numOfProfiles; i++)
        if (p_Fm->p_FmSp->profiles[i].profilesMng.ownerId == (uint8_t)ILLEGAL_BASE)
            profilesFound++;
        else
            break;

    if (profilesFound == numOfProfiles)
        for (i = base; i<base + numOfProfiles; i++)
            p_Fm->p_FmSp->profiles[i].profilesMng.ownerId = guestId;
    else
    {
        XX_UnlockIntrSpinlock(p_Fm->h_Spinlock, intFlags);
        return (uint8_t)ILLEGAL_BASE;
    }
    XX_UnlockIntrSpinlock(p_Fm->h_Spinlock, intFlags);

    return base;
}

static void FreeVSPsForPartition(t_Handle  h_Fm, uint8_t base, uint8_t numOfProfiles, uint8_t guestId)
{
    t_Fm    *p_Fm = (t_Fm *)h_Fm;
    int     i = 0;

    ASSERT_COND(p_Fm);

    if (p_Fm->h_IpcSessions[0])
    {
        t_FmIpcResourceAllocParams  ipcAllocParams;
        t_FmIpcMsg                  msg;
        t_FmIpcReply                reply;
        uint32_t                    replyLength;
        t_Error                     err;

        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        memset(&ipcAllocParams, 0, sizeof(t_FmIpcResourceAllocParams));
        ipcAllocParams.guestId         = p_Fm->guestId;
        ipcAllocParams.num             = p_Fm->partNumOfVSPs;
        ipcAllocParams.base            = p_Fm->partVSPBase;
        msg.msgId                              = FM_VSP_FREE;
        memcpy(msg.msgBody, &ipcAllocParams, sizeof(t_FmIpcResourceAllocParams));
        replyLength = sizeof(uint32_t) + sizeof(uint8_t);
        err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                (uint8_t*)&msg,
                                sizeof(msg.msgId) + sizeof(t_FmIpcResourceAllocParams),
                                (uint8_t*)&reply,
                                &replyLength,
                                NULL,
                                NULL);
        if (err != E_OK)
            REPORT_ERROR(MAJOR, err, NO_MSG);
        return;
    }
    if (p_Fm->guestId != NCSW_MASTER_ID)
    {
        DBG(WARNING, ("FM Guest mode, without IPC - can't validate VSP range!"));
        return;
    }

    ASSERT_COND(p_Fm->p_FmSp);

    for (i=base; i<numOfProfiles; i++)
    {
        if (p_Fm->p_FmSp->profiles[i].profilesMng.ownerId == guestId)
           p_Fm->p_FmSp->profiles[i].profilesMng.ownerId = (uint8_t)ILLEGAL_BASE;
        else
            DBG(WARNING, ("Request for freeing storage profile window which wasn't allocated to this partition"));
    }
}
#endif /* (DPAA_VERSION >= 11) */

static t_Error FmGuestHandleIpcMsgCB(t_Handle  h_Fm,
                                     uint8_t   *p_Msg,
                                     uint32_t  msgLength,
                                     uint8_t   *p_Reply,
                                     uint32_t  *p_ReplyLength)
{
    t_Fm            *p_Fm       = (t_Fm*)h_Fm;
    t_FmIpcMsg      *p_IpcMsg   = (t_FmIpcMsg*)p_Msg;

    UNUSED(p_Reply);
    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((msgLength > sizeof(uint32_t)), E_INVALID_VALUE);

#ifdef DISABLE_SANITY_CHECKS
    UNUSED(msgLength);
#endif /* DISABLE_SANITY_CHECKS */

    ASSERT_COND(p_Msg);

    *p_ReplyLength = 0;

    switch (p_IpcMsg->msgId)
    {
        case (FM_GUEST_ISR):
        {
            t_FmIpcIsr ipcIsr;

            memcpy((uint8_t*)&ipcIsr, p_IpcMsg->msgBody, sizeof(t_FmIpcIsr));
            if (ipcIsr.boolErr)
                GuestErrorIsr(p_Fm, ipcIsr.pendingReg);
            else
                GuestEventIsr(p_Fm, ipcIsr.pendingReg);
            break;
        }
        default:
            *p_ReplyLength = 0;
            RETURN_ERROR(MINOR, E_INVALID_SELECTION, ("command not found!!!"));
    }
    return E_OK;
}

static t_Error FmHandleIpcMsgCB(t_Handle  h_Fm,
                                uint8_t   *p_Msg,
                                uint32_t  msgLength,
                                uint8_t   *p_Reply,
                                uint32_t  *p_ReplyLength)
{
    t_Error         err;
    t_Fm            *p_Fm       = (t_Fm*)h_Fm;
    t_FmIpcMsg      *p_IpcMsg   = (t_FmIpcMsg*)p_Msg;
    t_FmIpcReply    *p_IpcReply = (t_FmIpcReply*)p_Reply;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((msgLength >= sizeof(uint32_t)), E_INVALID_VALUE);

#ifdef DISABLE_SANITY_CHECKS
    UNUSED(msgLength);
#endif /* DISABLE_SANITY_CHECKS */

    ASSERT_COND(p_IpcMsg);

    memset(p_IpcReply, 0, (sizeof(uint8_t) * FM_IPC_MAX_REPLY_SIZE));
    *p_ReplyLength = 0;

    switch (p_IpcMsg->msgId)
    {
        case (FM_GET_SET_PORT_PARAMS):
        {
            t_FmIpcPortInInitParams         ipcInitParams;
            t_FmInterModulePortInitParams   initParams;
            t_FmIpcPortOutInitParams        ipcOutInitParams;

            memcpy((uint8_t*)&ipcInitParams, p_IpcMsg->msgBody, sizeof(t_FmIpcPortInInitParams));
            initParams.hardwarePortId = ipcInitParams.hardwarePortId;
            initParams.portType = (e_FmPortType)ipcInitParams.enumPortType;
            initParams.independentMode = (bool)(ipcInitParams.boolIndependentMode);
            initParams.liodnOffset = ipcInitParams.liodnOffset;
            initParams.numOfTasks = ipcInitParams.numOfTasks;
            initParams.numOfExtraTasks = ipcInitParams.numOfExtraTasks;
            initParams.numOfOpenDmas = ipcInitParams.numOfOpenDmas;
            initParams.numOfExtraOpenDmas = ipcInitParams.numOfExtraOpenDmas;
            initParams.sizeOfFifo = ipcInitParams.sizeOfFifo;
            initParams.extraSizeOfFifo = ipcInitParams.extraSizeOfFifo;
            initParams.deqPipelineDepth = ipcInitParams.deqPipelineDepth;
            initParams.maxFrameLength = ipcInitParams.maxFrameLength;
            initParams.liodnBase = ipcInitParams.liodnBase;

            p_IpcReply->error = (uint32_t)FmGetSetPortParams(h_Fm, &initParams);

            ipcOutInitParams.ipcPhysAddr.high = initParams.fmMuramPhysBaseAddr.high;
            ipcOutInitParams.ipcPhysAddr.low = initParams.fmMuramPhysBaseAddr.low;
            ipcOutInitParams.sizeOfFifo = initParams.sizeOfFifo;
            ipcOutInitParams.extraSizeOfFifo = initParams.extraSizeOfFifo;
            ipcOutInitParams.numOfTasks = initParams.numOfTasks;
            ipcOutInitParams.numOfExtraTasks = initParams.numOfExtraTasks;
            ipcOutInitParams.numOfOpenDmas = initParams.numOfOpenDmas;
            ipcOutInitParams.numOfExtraOpenDmas = initParams.numOfExtraOpenDmas;
            memcpy(p_IpcReply->replyBody, (uint8_t*)&ipcOutInitParams, sizeof(ipcOutInitParams));
            *p_ReplyLength = sizeof(uint32_t) + sizeof(t_FmIpcPortOutInitParams);
            break;
        }
        case (FM_SET_SIZE_OF_FIFO):
        {
            t_FmIpcPortRsrcParams   ipcPortRsrcParams;

            memcpy((uint8_t*)&ipcPortRsrcParams, p_IpcMsg->msgBody, sizeof(t_FmIpcPortRsrcParams));
            p_IpcReply->error = (uint32_t)FmSetSizeOfFifo(h_Fm,
                                                          ipcPortRsrcParams.hardwarePortId,
                                                          &ipcPortRsrcParams.val,
                                                          &ipcPortRsrcParams.extra,
                                                          (bool)ipcPortRsrcParams.boolInitialConfig);
            *p_ReplyLength = sizeof(uint32_t);
            break;
        }
        case (FM_SET_NUM_OF_TASKS):
        {
            t_FmIpcPortRsrcParams   ipcPortRsrcParams;

            memcpy((uint8_t*)&ipcPortRsrcParams, p_IpcMsg->msgBody, sizeof(t_FmIpcPortRsrcParams));
            p_IpcReply->error = (uint32_t)FmSetNumOfTasks(h_Fm, ipcPortRsrcParams.hardwarePortId,
                                                          (uint8_t*)&ipcPortRsrcParams.val,
                                                          (uint8_t*)&ipcPortRsrcParams.extra,
                                                          (bool)ipcPortRsrcParams.boolInitialConfig);
            *p_ReplyLength = sizeof(uint32_t);
            break;
        }
        case (FM_SET_NUM_OF_OPEN_DMAS):
        {
            t_FmIpcPortRsrcParams   ipcPortRsrcParams;

            memcpy((uint8_t*)&ipcPortRsrcParams, p_IpcMsg->msgBody, sizeof(t_FmIpcPortRsrcParams));
            p_IpcReply->error = (uint32_t)FmSetNumOfOpenDmas(h_Fm, ipcPortRsrcParams.hardwarePortId,
                                                               (uint8_t*)&ipcPortRsrcParams.val,
                                                               (uint8_t*)&ipcPortRsrcParams.extra,
                                                               (bool)ipcPortRsrcParams.boolInitialConfig);
            *p_ReplyLength = sizeof(uint32_t);
            break;
        }
        case (FM_RESUME_STALLED_PORT):
            *p_ReplyLength = sizeof(uint32_t);
            p_IpcReply->error = (uint32_t)FmResumeStalledPort(h_Fm, p_IpcMsg->msgBody[0]);
            break;
        case (FM_MASTER_IS_ALIVE):
        {
            uint8_t guestId = p_IpcMsg->msgBody[0];
            /* build the FM master partition IPC address */
            memset(p_Fm->fmIpcHandlerModuleName[guestId], 0, (sizeof(char)) * MODULE_NAME_SIZE);
            if (Sprint (p_Fm->fmIpcHandlerModuleName[guestId], "FM_%d_%d",p_Fm->p_FmStateStruct->fmId, guestId) != (guestId<10 ? 6:7))
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Sprint failed"));
            p_Fm->h_IpcSessions[guestId] = XX_IpcInitSession(p_Fm->fmIpcHandlerModuleName[guestId], p_Fm->fmModuleName);
            if (p_Fm->h_IpcSessions[guestId] == NULL)
                RETURN_ERROR(MAJOR, E_NOT_AVAILABLE, ("FM Master IPC session for guest %d", guestId));
            *(uint8_t*)(p_IpcReply->replyBody) = 1;
            *p_ReplyLength = sizeof(uint32_t) + sizeof(uint8_t);
            break;
        }
        case (FM_IS_PORT_STALLED):
        {
            bool tmp;

            p_IpcReply->error = (uint32_t)FmIsPortStalled(h_Fm, p_IpcMsg->msgBody[0], &tmp);
            *(uint8_t*)(p_IpcReply->replyBody) = (uint8_t)tmp;
            *p_ReplyLength = sizeof(uint32_t) + sizeof(uint8_t);
            break;
        }
        case (FM_RESET_MAC):
        {
            t_FmIpcMacParams    ipcMacParams;

            memcpy((uint8_t*)&ipcMacParams, p_IpcMsg->msgBody, sizeof(t_FmIpcMacParams));
            p_IpcReply->error = (uint32_t)FmResetMac(p_Fm,
                                                     (e_FmMacType)(ipcMacParams.enumType),
                                                     ipcMacParams.id);
            *p_ReplyLength = sizeof(uint32_t);
            break;
        }
        case (FM_SET_MAC_MAX_FRAME):
        {
            t_FmIpcMacMaxFrameParams    ipcMacMaxFrameParams;

            memcpy((uint8_t*)&ipcMacMaxFrameParams, p_IpcMsg->msgBody, sizeof(t_FmIpcMacMaxFrameParams));
            err = FmSetMacMaxFrame(p_Fm,
                                  (e_FmMacType)(ipcMacMaxFrameParams.macParams.enumType),
                                  ipcMacMaxFrameParams.macParams.id,
                                  ipcMacMaxFrameParams.maxFrameLength);
            if (err != E_OK)
                REPORT_ERROR(MINOR, err, NO_MSG);
            break;
        }
#if (DPAA_VERSION >= 11)
        case (FM_VSP_ALLOC) :
        {
            t_FmIpcResourceAllocParams  ipcAllocParams;
            uint8_t                     vspBase;
            memcpy(&ipcAllocParams, p_IpcMsg->msgBody, sizeof(t_FmIpcResourceAllocParams));
            vspBase =  AllocVSPsForPartition(h_Fm, ipcAllocParams.base, ipcAllocParams.num, ipcAllocParams.guestId);
            memcpy(p_IpcReply->replyBody, (uint8_t*)&vspBase, sizeof(uint8_t));
            *p_ReplyLength = sizeof(uint32_t) + sizeof(uint8_t);
            break;
        }
        case (FM_VSP_FREE) :
        {
            t_FmIpcResourceAllocParams   ipcAllocParams;
            memcpy(&ipcAllocParams, p_IpcMsg->msgBody, sizeof(t_FmIpcResourceAllocParams));
            FreeVSPsForPartition(h_Fm, ipcAllocParams.base, ipcAllocParams.num, ipcAllocParams.guestId);
            break;
        }
        case (FM_VSP_SET_PORT_WINDOW) :
        {
            t_FmIpcVspSetPortWindow   ipcVspSetPortWindow;
            memcpy(&ipcVspSetPortWindow, p_IpcMsg->msgBody, sizeof(t_FmIpcVspSetPortWindow));
            err = SetVSPWindow(h_Fm,
                                            ipcVspSetPortWindow.hardwarePortId,
                                            ipcVspSetPortWindow.baseStorageProfile,
                                            ipcVspSetPortWindow.log2NumOfProfiles);
            return err;
        }
        case (FM_SET_CONG_GRP_PFC_PRIO) :
        {
            t_FmIpcSetCongestionGroupPfcPriority    fmIpcSetCongestionGroupPfcPriority;
            memcpy(&fmIpcSetCongestionGroupPfcPriority, p_IpcMsg->msgBody, sizeof(t_FmIpcSetCongestionGroupPfcPriority));
            err = FmSetCongestionGroupPFCpriority(h_Fm,
                                                  fmIpcSetCongestionGroupPfcPriority.congestionGroupId,
                                                  fmIpcSetCongestionGroupPfcPriority.priorityBitMap);
            return err;
        }
#endif /* (DPAA_VERSION >= 11) */

        case (FM_FREE_PORT):
        {
            t_FmInterModulePortFreeParams   portParams;
            t_FmIpcPortFreeParams           ipcPortParams;

            memcpy((uint8_t*)&ipcPortParams, p_IpcMsg->msgBody, sizeof(t_FmIpcPortFreeParams));
            portParams.hardwarePortId = ipcPortParams.hardwarePortId;
            portParams.portType = (e_FmPortType)(ipcPortParams.enumPortType);
            portParams.deqPipelineDepth = ipcPortParams.deqPipelineDepth;
            FmFreePortParams(h_Fm, &portParams);
            break;
        }
        case (FM_REGISTER_INTR):
        {
            t_FmIpcRegisterIntr ipcRegIntr;

            memcpy((uint8_t*)&ipcRegIntr, p_IpcMsg->msgBody, sizeof(ipcRegIntr));
            p_Fm->intrMng[ipcRegIntr.event].guestId = ipcRegIntr.guestId;
            break;
        }
        case (FM_GET_PARAMS):
        {
             t_FmIpcParams  ipcParams;
             uint32_t       tmpReg;

            /* Get clock frequency */
            ipcParams.fmClkFreq = p_Fm->p_FmStateStruct->fmClkFreq;

            /* read FM revision register 1 */
            tmpReg = GET_UINT32(p_Fm->p_FmFpmRegs->fm_ip_rev_1);
            ipcParams.majorRev = (uint8_t)((tmpReg & FPM_REV1_MAJOR_MASK) >> FPM_REV1_MAJOR_SHIFT);
            ipcParams.minorRev = (uint8_t)((tmpReg & FPM_REV1_MINOR_MASK) >> FPM_REV1_MINOR_SHIFT);

            memcpy(p_IpcReply->replyBody, (uint8_t*)&ipcParams, sizeof(t_FmIpcParams));
            *p_ReplyLength = sizeof(uint32_t) + sizeof(t_FmIpcParams);
             break;
        }
        case (FM_GET_FMAN_CTRL_CODE_REV):
        {
            t_FmCtrlCodeRevisionInfo        fmanCtrlRevInfo;
            t_FmIpcFmanCtrlCodeRevisionInfo ipcRevInfo;

            p_IpcReply->error = (uint32_t)FM_GetFmanCtrlCodeRevision(h_Fm, &fmanCtrlRevInfo);
            ipcRevInfo.packageRev = fmanCtrlRevInfo.packageRev;
            ipcRevInfo.majorRev = fmanCtrlRevInfo.majorRev;
            ipcRevInfo.minorRev = fmanCtrlRevInfo.minorRev;
            memcpy(p_IpcReply->replyBody, (uint8_t*)&ipcRevInfo, sizeof(t_FmIpcFmanCtrlCodeRevisionInfo));
            *p_ReplyLength = sizeof(uint32_t) + sizeof(t_FmIpcFmanCtrlCodeRevisionInfo);
            break;
        }

        case (FM_DMA_STAT):
        {
            t_FmDmaStatus       dmaStatus;
            t_FmIpcDmaStatus    ipcDmaStatus;

            FM_GetDmaStatus(h_Fm, &dmaStatus);
            ipcDmaStatus.boolCmqNotEmpty = (uint8_t)dmaStatus.cmqNotEmpty;
            ipcDmaStatus.boolBusError = (uint8_t)dmaStatus.busError;
            ipcDmaStatus.boolReadBufEccError = (uint8_t)dmaStatus.readBufEccError;
            ipcDmaStatus.boolWriteBufEccSysError = (uint8_t)dmaStatus.writeBufEccSysError;
            ipcDmaStatus.boolWriteBufEccFmError = (uint8_t)dmaStatus.writeBufEccFmError;
            ipcDmaStatus.boolSinglePortEccError = (uint8_t)dmaStatus.singlePortEccError;
            memcpy(p_IpcReply->replyBody, (uint8_t*)&ipcDmaStatus, sizeof(t_FmIpcDmaStatus));
            *p_ReplyLength = sizeof(uint32_t) + sizeof(t_FmIpcDmaStatus);
            break;
        }
        case (FM_ALLOC_FMAN_CTRL_EVENT_REG):
            p_IpcReply->error = (uint32_t)FmAllocFmanCtrlEventReg(h_Fm, (uint8_t*)p_IpcReply->replyBody);
            *p_ReplyLength = sizeof(uint32_t) + sizeof(uint8_t);
            break;
        case (FM_FREE_FMAN_CTRL_EVENT_REG):
            FmFreeFmanCtrlEventReg(h_Fm, p_IpcMsg->msgBody[0]);
            break;
        case (FM_GET_TIMESTAMP_SCALE):
        {
            uint32_t    timeStamp = FmGetTimeStampScale(h_Fm);

            memcpy(p_IpcReply->replyBody, (uint8_t*)&timeStamp, sizeof(uint32_t));
            *p_ReplyLength = sizeof(uint32_t) + sizeof(uint32_t);
            break;
        }
        case (FM_GET_COUNTER):
        {
            e_FmCounters    inCounter;
            uint32_t        outCounter;

            memcpy((uint8_t*)&inCounter, p_IpcMsg->msgBody, sizeof(uint32_t));
            outCounter = FM_GetCounter(h_Fm, inCounter);
            memcpy(p_IpcReply->replyBody, (uint8_t*)&outCounter, sizeof(uint32_t));
            *p_ReplyLength = sizeof(uint32_t) + sizeof(uint32_t);
            break;
        }
        case (FM_SET_FMAN_CTRL_EVENTS_ENABLE):
        {
            t_FmIpcFmanEvents ipcFmanEvents;

            memcpy((uint8_t*)&ipcFmanEvents, p_IpcMsg->msgBody, sizeof(t_FmIpcFmanEvents));
            FmSetFmanCtrlIntr(h_Fm,
                              ipcFmanEvents.eventRegId,
                              ipcFmanEvents.enableEvents);
            break;
        }
        case (FM_GET_FMAN_CTRL_EVENTS_ENABLE):
        {
            uint32_t    tmp = FmGetFmanCtrlIntr(h_Fm, p_IpcMsg->msgBody[0]);

            memcpy(p_IpcReply->replyBody, (uint8_t*)&tmp, sizeof(uint32_t));
            *p_ReplyLength = sizeof(uint32_t) + sizeof(uint32_t);
            break;
        }
        case (FM_GET_PHYS_MURAM_BASE):
        {
            t_FmPhysAddr        physAddr;
            t_FmIpcPhysAddr     ipcPhysAddr;

            FmGetPhysicalMuramBase(h_Fm, &physAddr);
            ipcPhysAddr.high    = physAddr.high;
            ipcPhysAddr.low     = physAddr.low;
            memcpy(p_IpcReply->replyBody, (uint8_t*)&ipcPhysAddr, sizeof(t_FmIpcPhysAddr));
            *p_ReplyLength = sizeof(uint32_t) + sizeof(t_FmIpcPhysAddr);
            break;
        }
        case (FM_ENABLE_RAM_ECC):
        {
            if (((err = FM_EnableRamsEcc(h_Fm)) != E_OK) ||
                ((err = FM_SetException(h_Fm, e_FM_EX_IRAM_ECC, TRUE)) != E_OK) ||
                ((err = FM_SetException(h_Fm, e_FM_EX_MURAM_ECC, TRUE)) != E_OK))
#if (!(defined(DEBUG_ERRORS)) || (DEBUG_ERRORS == 0))
                UNUSED(err);
#else
                REPORT_ERROR(MINOR, err, NO_MSG);
#endif /* (!(defined(DEBUG_ERRORS)) || (DEBUG_ERRORS == 0)) */
            break;
        }
        case (FM_DISABLE_RAM_ECC):
        {

            if (((err = FM_SetException(h_Fm, e_FM_EX_IRAM_ECC, FALSE)) != E_OK) ||
                ((err = FM_SetException(h_Fm, e_FM_EX_MURAM_ECC, FALSE)) != E_OK) ||
                ((err = FM_DisableRamsEcc(h_Fm)) != E_OK))
#if (!(defined(DEBUG_ERRORS)) || (DEBUG_ERRORS == 0))
                UNUSED(err);
#else
                REPORT_ERROR(MINOR, err, NO_MSG);
#endif /* (!(defined(DEBUG_ERRORS)) || (DEBUG_ERRORS == 0)) */
            break;
        }
        case (FM_SET_NUM_OF_FMAN_CTRL):
        {
            t_FmIpcPortNumOfFmanCtrls   ipcPortNumOfFmanCtrls;

            memcpy((uint8_t*)&ipcPortNumOfFmanCtrls, p_IpcMsg->msgBody, sizeof(t_FmIpcPortNumOfFmanCtrls));
            err = FmSetNumOfRiscsPerPort(h_Fm,
                                         ipcPortNumOfFmanCtrls.hardwarePortId,
                                         ipcPortNumOfFmanCtrls.numOfFmanCtrls,
                                         ipcPortNumOfFmanCtrls.orFmanCtrl);
            if (err != E_OK)
                REPORT_ERROR(MINOR, err, NO_MSG);
            break;
        }
#ifdef FM_TX_ECC_FRMS_ERRATA_10GMAC_A004
        case (FM_10G_TX_ECC_WA):
            p_IpcReply->error = (uint32_t)Fm10GTxEccWorkaround(h_Fm, p_IpcMsg->msgBody[0]);
            *p_ReplyLength = sizeof(uint32_t);
            break;
#endif /* FM_TX_ECC_FRMS_ERRATA_10GMAC_A004 */
        default:
            *p_ReplyLength = 0;
            RETURN_ERROR(MINOR, E_INVALID_SELECTION, ("command not found!!!"));
    }
    return E_OK;
}


/****************************************/
/*       Inter-Module functions         */
/****************************************/
#ifdef FM_TX_ECC_FRMS_ERRATA_10GMAC_A004
t_Error Fm10GTxEccWorkaround(t_Handle h_Fm, uint8_t macId)
{
    t_Fm            *p_Fm = (t_Fm*)h_Fm;
    int             timeout = 1000;
    t_Error         err = E_OK;
    t_FmIpcMsg      msg;
    t_FmIpcReply    reply;
    uint32_t        replyLength;
    uint8_t         rxHardwarePortId, txHardwarePortId;

    if (p_Fm->guestId != NCSW_MASTER_ID)
    {
        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        msg.msgId = FM_10G_TX_ECC_WA;
        msg.msgBody[0] = macId;
        replyLength = sizeof(uint32_t);
        if ((err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                     (uint8_t*)&msg,
                                     sizeof(msg.msgId)+sizeof(macId),
                                     (uint8_t*)&reply,
                                     &replyLength,
                                     NULL,
                                     NULL)) != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
        if (replyLength != sizeof(uint32_t))
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("IPC reply length mismatch"));
        return (t_Error)(reply.error);
    }

    SANITY_CHECK_RETURN_ERROR((macId == 0), E_NOT_SUPPORTED);
    SANITY_CHECK_RETURN_ERROR(IsFmanCtrlCodeLoaded(p_Fm), E_INVALID_STATE);

    SW_PORT_ID_TO_HW_PORT_ID(rxHardwarePortId, e_FM_PORT_TYPE_RX_10G, macId);
    SW_PORT_ID_TO_HW_PORT_ID(txHardwarePortId, e_FM_PORT_TYPE_TX_10G, macId);
    if ((p_Fm->p_FmStateStruct->portsTypes[rxHardwarePortId] != e_FM_PORT_TYPE_DUMMY) ||
        (p_Fm->p_FmStateStruct->portsTypes[txHardwarePortId] != e_FM_PORT_TYPE_DUMMY))
        RETURN_ERROR(MAJOR, E_INVALID_STATE,
                     ("MAC should be initialized prior to Rx and Tx ports!"));
    WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_extc, 0x40000000);
    CORE_MemoryBarrier();
    while ((GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_extc) & 0x40000000) &&
           --timeout) ;
    if (!timeout)
        return ERROR_CODE(E_TIMEOUT);
    return E_OK;
}
#endif /* FM_TX_ECC_FRMS_ERRATA_10GMAC_A004 */

uint16_t FmGetTnumAgingPeriod(t_Handle h_Fm)
{
    t_Fm *p_Fm = (t_Fm *)h_Fm;

    SANITY_CHECK_RETURN_VALUE(p_Fm, E_INVALID_HANDLE, 0);
    SANITY_CHECK_RETURN_VALUE(!p_Fm->p_FmDriverParam, E_INVALID_STATE, 0);

    return p_Fm->tnumAgingPeriod;
}

t_Error FmSetPortPreFetchConfiguration(t_Handle h_Fm,
                                       uint8_t  portNum,
                                       bool     preFetchConfigured)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Fm->p_FmDriverParam, E_INVALID_STATE);

    p_Fm->portsPreFetchConfigured[portNum] = TRUE;
    p_Fm->portsPreFetchValue[portNum] = preFetchConfigured;

    return E_OK;
}

t_Error FmGetPortPreFetchConfiguration(t_Handle h_Fm,
                                       uint8_t  portNum,
                                       bool     *p_PortConfigured,
                                       bool     *p_PreFetchConfigured)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Fm->p_FmDriverParam, E_INVALID_STATE);

    /* If the prefetch wasn't configured yet (not enable or disabled)
       we return the value TRUE as it was already configured */
    if (!p_Fm->portsPreFetchConfigured[portNum])
    {
        *p_PortConfigured = FALSE;
        *p_PreFetchConfigured = FALSE;
    }
    else
    {
        *p_PortConfigured = TRUE;
        *p_PreFetchConfigured = (p_Fm->portsPreFetchConfigured[portNum]);
    }

    return E_OK;
}

t_Error FmSetCongestionGroupPFCpriority(t_Handle    h_Fm,
                                        uint32_t    congestionGroupId,
                                        uint8_t     priorityBitMap)
{
    t_Fm    *p_Fm  = (t_Fm *)h_Fm;

    ASSERT_COND(h_Fm);

    if (congestionGroupId > FM_PORT_NUM_OF_CONGESTION_GRPS)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE,
                     ("Congestion group ID bigger than %d",
                      FM_PORT_NUM_OF_CONGESTION_GRPS));

    if (p_Fm->guestId == NCSW_MASTER_ID)
    {
        uint32_t    *p_Cpg = (uint32_t*)(p_Fm->baseAddr+FM_MM_CGP);
        uint32_t    tmpReg;
        uint32_t    reg_num;
        uint32_t    offset;
        uint32_t    mask;

        ASSERT_COND(p_Fm->baseAddr);
        reg_num = (FM_PORT_NUM_OF_CONGESTION_GRPS-1-(congestionGroupId))/4;
        offset  = (congestionGroupId%4);

        tmpReg = GET_UINT32(p_Cpg[reg_num]);

        /* Adding priorities*/
        if (priorityBitMap)
        {
            if (tmpReg & (0xFF << (offset*8)))
                RETURN_ERROR(MAJOR, E_INVALID_STATE,
                             ("PFC priority for the congestion group is already set!"));
        }
        else /* Deleting priorities */
        {
            mask = (uint32_t)(0xFF << (offset*8));
            tmpReg &= ~mask;
        }

        tmpReg |= (uint32_t)priorityBitMap << (offset*8);
        WRITE_UINT32(p_Cpg[reg_num], tmpReg);
    }

    else if (p_Fm->h_IpcSessions[0])
    {
        t_Error                              err;
        t_FmIpcMsg                           msg;
        t_FmIpcSetCongestionGroupPfcPriority fmIpcSetCongestionGroupPfcPriority;

        memset(&msg, 0, sizeof(msg));
        memset(&fmIpcSetCongestionGroupPfcPriority, 0, sizeof(t_FmIpcSetCongestionGroupPfcPriority));
        fmIpcSetCongestionGroupPfcPriority.congestionGroupId = congestionGroupId;
        fmIpcSetCongestionGroupPfcPriority.priorityBitMap    = priorityBitMap;

        msg.msgId = FM_SET_CONG_GRP_PFC_PRIO;
        memcpy(msg.msgBody, &fmIpcSetCongestionGroupPfcPriority, sizeof(t_FmIpcSetCongestionGroupPfcPriority));

        err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                (uint8_t*)&msg,
                                sizeof(msg.msgId),
                                NULL,
                                NULL,
                                NULL,
                                NULL);
        if (err != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
    }
    else
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("guest without IPC!"));

    return E_OK;
}

uintptr_t FmGetPcdPrsBaseAddr(t_Handle h_Fm)
{
    t_Fm        *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_VALUE(p_Fm, E_INVALID_HANDLE, 0);

    if (!p_Fm->baseAddr)
    {
        REPORT_ERROR(MAJOR, E_INVALID_STATE,
                     ("No base-addr; probably Guest with IPC!"));
        return 0;
    }

    return (p_Fm->baseAddr + FM_MM_PRS);
}

uintptr_t FmGetPcdKgBaseAddr(t_Handle h_Fm)
{
    t_Fm        *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_VALUE(p_Fm, E_INVALID_HANDLE, 0);

    if (!p_Fm->baseAddr)
    {
        REPORT_ERROR(MAJOR, E_INVALID_STATE,
                     ("No base-addr; probably Guest with IPC!"));
        return 0;
    }

    return (p_Fm->baseAddr + FM_MM_KG);
}

uintptr_t FmGetPcdPlcrBaseAddr(t_Handle h_Fm)
{
    t_Fm        *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_VALUE(p_Fm, E_INVALID_HANDLE, 0);

    if (!p_Fm->baseAddr)
    {
        REPORT_ERROR(MAJOR, E_INVALID_STATE,
                     ("No base-addr; probably Guest with IPC!"));
        return 0;
    }

    return (p_Fm->baseAddr + FM_MM_PLCR);
}

#if (DPAA_VERSION >= 11)
uintptr_t FmGetVSPBaseAddr(t_Handle h_Fm)
{
    t_Fm        *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_VALUE(p_Fm, E_INVALID_HANDLE, 0);

    return p_Fm->vspBaseAddr;
}
#endif /* (DPAA_VERSION >= 11) */

t_Handle FmGetMuramHandle(t_Handle h_Fm)
{
    t_Fm        *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_VALUE(p_Fm, E_INVALID_HANDLE, NULL);

    return (p_Fm->h_FmMuram);
}

void FmGetPhysicalMuramBase(t_Handle h_Fm, t_FmPhysAddr *p_FmPhysAddr)
{
    t_Fm            *p_Fm = (t_Fm*)h_Fm;

    if (p_Fm->fmMuramPhysBaseAddr)
    {
        /* General FM driver initialization */
        p_FmPhysAddr->low = (uint32_t)p_Fm->fmMuramPhysBaseAddr;
        p_FmPhysAddr->high = (uint8_t)((p_Fm->fmMuramPhysBaseAddr & 0x000000ff00000000LL) >> 32);
        return;
    }

    ASSERT_COND(p_Fm->guestId != NCSW_MASTER_ID);

    if (p_Fm->h_IpcSessions[0])
    {
        t_Error         err;
        t_FmIpcMsg      msg;
        t_FmIpcReply    reply;
        uint32_t        replyLength;
        t_FmIpcPhysAddr ipcPhysAddr;

        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        msg.msgId = FM_GET_PHYS_MURAM_BASE;
        replyLength = sizeof(uint32_t) + sizeof(t_FmPhysAddr);
        err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                (uint8_t*)&msg,
                                sizeof(msg.msgId),
                                (uint8_t*)&reply,
                                &replyLength,
                                NULL,
                                NULL);
        if (err != E_OK)
        {
            REPORT_ERROR(MINOR, err, NO_MSG);
            return;
        }
        if (replyLength != (sizeof(uint32_t) + sizeof(t_FmPhysAddr)))
        {
            REPORT_ERROR(MINOR, E_INVALID_VALUE,("IPC reply length mismatch"));
            return;
        }
        memcpy((uint8_t*)&ipcPhysAddr, reply.replyBody, sizeof(t_FmIpcPhysAddr));
        p_FmPhysAddr->high = ipcPhysAddr.high;
        p_FmPhysAddr->low  = ipcPhysAddr.low;
    }
    else
        REPORT_ERROR(MINOR, E_NOT_SUPPORTED,
                     ("running in guest-mode without neither IPC nor mapped register!"));
}

#if (DPAA_VERSION >= 11)
t_Error FmVSPAllocForPort (t_Handle        h_Fm,
                           e_FmPortType    portType,
                           uint8_t         portId,
                           uint8_t         numOfVSPs)
{
    t_Fm           *p_Fm = (t_Fm *)h_Fm;
    t_Error        err = E_OK;
    uint32_t       profilesFound, intFlags;
    uint8_t        first, i;
    uint8_t        log2Num;
    uint8_t        swPortIndex=0, hardwarePortId;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);

     if (!numOfVSPs)
        return E_OK;

    if (numOfVSPs > FM_VSP_MAX_NUM_OF_ENTRIES)
        RETURN_ERROR(MINOR, E_INVALID_VALUE, ("numProfiles can not be bigger than %d.",FM_VSP_MAX_NUM_OF_ENTRIES));

    if (!POWER_OF_2(numOfVSPs))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("numProfiles must be a power of 2."));

    LOG2((uint64_t)numOfVSPs, log2Num);

    if ((log2Num == 0) || (p_Fm->partVSPBase == 0))
        first = 0;
    else
        first = 1<<log2Num;

    if (first > (p_Fm->partVSPBase + p_Fm->partNumOfVSPs))
         RETURN_ERROR(MINOR, E_INVALID_VALUE, ("can not allocate storage profile port window"));

    if (first < p_Fm->partVSPBase)
        while (first < p_Fm->partVSPBase)
            first = first + numOfVSPs;

    if ((first + numOfVSPs) > (p_Fm->partVSPBase + p_Fm->partNumOfVSPs))
        RETURN_ERROR(MINOR, E_INVALID_VALUE, ("can not allocate storage profile port window"));

    intFlags = XX_LockIntrSpinlock(p_Fm->h_Spinlock);
    profilesFound = 0;
    for (i=first; i < p_Fm->partVSPBase + p_Fm->partNumOfVSPs; )
    {
        if (!p_Fm->p_FmSp->profiles[i].profilesMng.allocated)
        {
            profilesFound++;
            i++;
            if (profilesFound == numOfVSPs)
                break;
        }
        else
        {
            profilesFound = 0;
            /* advance i to the next aligned address */
            first = i = (uint8_t)(first + numOfVSPs);
        }
    }
    if (profilesFound == numOfVSPs)
        for (i = first; i<first + numOfVSPs; i++)
            p_Fm->p_FmSp->profiles[i].profilesMng.allocated = TRUE;
    else
    {
        XX_UnlockIntrSpinlock(p_Fm->h_Spinlock, intFlags);
        RETURN_ERROR(MINOR, E_FULL, ("No profiles."));
    }

    SW_PORT_ID_TO_HW_PORT_ID(hardwarePortId, portType, portId)
    HW_PORT_ID_TO_SW_PORT_INDX(swPortIndex, hardwarePortId);

    p_Fm->p_FmSp->portsMapping[swPortIndex].numOfProfiles = numOfVSPs;
    p_Fm->p_FmSp->portsMapping[swPortIndex].profilesBase = first;

    if ((err = SetVSPWindow(h_Fm,hardwarePortId, first,log2Num)) != E_OK)
        for (i = first; i < first + numOfVSPs; i++)
            p_Fm->p_FmSp->profiles[i].profilesMng.allocated = FALSE;

    XX_UnlockIntrSpinlock(p_Fm->h_Spinlock, intFlags);

    return err;
}

t_Error FmVSPFreeForPort(t_Handle        h_Fm,
                         e_FmPortType    portType,
                         uint8_t         portId)
{
    t_Fm            *p_Fm = (t_Fm *)h_Fm;
    uint8_t         swPortIndex=0, hardwarePortId, first, numOfVSPs, i;
    uint32_t        intFlags;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);

    SW_PORT_ID_TO_HW_PORT_ID(hardwarePortId, portType, portId)
    HW_PORT_ID_TO_SW_PORT_INDX(swPortIndex, hardwarePortId);

    numOfVSPs = p_Fm->p_FmSp->portsMapping[swPortIndex].numOfProfiles;
    first = p_Fm->p_FmSp->portsMapping[swPortIndex].profilesBase;

    intFlags = XX_LockIntrSpinlock(p_Fm->h_Spinlock);
    for (i = first; i < first + numOfVSPs; i++)
           p_Fm->p_FmSp->profiles[i].profilesMng.allocated = FALSE;
    XX_UnlockIntrSpinlock(p_Fm->h_Spinlock, intFlags);

    p_Fm->p_FmSp->portsMapping[swPortIndex].numOfProfiles = 0;
    p_Fm->p_FmSp->portsMapping[swPortIndex].profilesBase = 0;

    return E_OK;
}
#endif /* (DPAA_VERSION >= 11) */

t_Error FmAllocFmanCtrlEventReg(t_Handle h_Fm, uint8_t *p_EventId)
{
    t_Fm            *p_Fm = (t_Fm*)h_Fm;
    uint8_t         i;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        p_Fm->h_IpcSessions[0])
    {
        t_Error         err;
        t_FmIpcMsg      msg;
        t_FmIpcReply    reply;
        uint32_t        replyLength;

        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        msg.msgId = FM_ALLOC_FMAN_CTRL_EVENT_REG;
        replyLength = sizeof(uint32_t) + sizeof(uint8_t);
        if ((err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                     (uint8_t*)&msg,
                                     sizeof(msg.msgId),
                                     (uint8_t*)&reply,
                                     &replyLength,
                                     NULL,
                                     NULL)) != E_OK)
            RETURN_ERROR(MAJOR, err, NO_MSG);

        if (replyLength != (sizeof(uint32_t) + sizeof(uint8_t)))
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("IPC reply length mismatch"));

        *p_EventId = *(uint8_t*)(reply.replyBody);

        return (t_Error)(reply.error);
    }
    else if (p_Fm->guestId != NCSW_MASTER_ID)
        RETURN_ERROR(MINOR, E_NOT_SUPPORTED,
                     ("running in guest-mode without IPC!"));

    for (i=0;i<FM_NUM_OF_FMAN_CTRL_EVENT_REGS;i++)
        if (!p_Fm->usedEventRegs[i])
        {
            p_Fm->usedEventRegs[i] = TRUE;
            *p_EventId = i;
            break;
        }

    if (i==FM_NUM_OF_FMAN_CTRL_EVENT_REGS)
        RETURN_ERROR(MAJOR, E_BUSY, ("No resource - FMan controller event register."));

    return E_OK;
}

void FmFreeFmanCtrlEventReg(t_Handle h_Fm, uint8_t eventId)
{
    t_Fm        *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN(p_Fm, E_INVALID_HANDLE);

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        p_Fm->h_IpcSessions[0])
    {
        t_Error     err;
        t_FmIpcMsg  msg;

        memset(&msg, 0, sizeof(msg));
        msg.msgId = FM_FREE_FMAN_CTRL_EVENT_REG;
        msg.msgBody[0] = eventId;
        err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                (uint8_t*)&msg,
                                sizeof(msg.msgId)+sizeof(eventId),
                                NULL,
                                NULL,
                                NULL,
                                NULL);
        if (err != E_OK)
            REPORT_ERROR(MINOR, err, NO_MSG);
        return;
    }
    else if (p_Fm->guestId != NCSW_MASTER_ID)
    {
        REPORT_ERROR(MINOR, E_NOT_SUPPORTED,
                     ("running in guest-mode without IPC!"));
        return;
    }

    ((t_Fm*)h_Fm)->usedEventRegs[eventId] = FALSE;
}

void FmSetFmanCtrlIntr(t_Handle h_Fm, uint8_t eventRegId, uint32_t enableEvents)
{
    t_Fm                *p_Fm = (t_Fm*)h_Fm;

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        !p_Fm->p_FmFpmRegs &&
        p_Fm->h_IpcSessions[0])
    {
        t_FmIpcFmanEvents   fmanCtrl;
        t_Error             err;
        t_FmIpcMsg          msg;

        fmanCtrl.eventRegId = eventRegId;
        fmanCtrl.enableEvents = enableEvents;
        memset(&msg, 0, sizeof(msg));
        msg.msgId = FM_SET_FMAN_CTRL_EVENTS_ENABLE;
        memcpy(msg.msgBody, &fmanCtrl, sizeof(fmanCtrl));
        err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                (uint8_t*)&msg,
                                sizeof(msg.msgId)+sizeof(fmanCtrl),
                                NULL,
                                NULL,
                                NULL,
                                NULL);
        if (err != E_OK)
            REPORT_ERROR(MINOR, err, NO_MSG);
        return;
    }
    else if (!p_Fm->p_FmFpmRegs)
    {
        REPORT_ERROR(MINOR, E_NOT_SUPPORTED,
                     ("Either IPC or 'baseAddress' is required!"));
        return;
    }

    ASSERT_COND(eventRegId < FM_NUM_OF_FMAN_CTRL_EVENT_REGS);
    WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_cee[eventRegId], enableEvents);
}

uint32_t FmGetFmanCtrlIntr(t_Handle h_Fm, uint8_t eventRegId)
{
    t_Fm            *p_Fm = (t_Fm*)h_Fm;

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        !p_Fm->p_FmFpmRegs &&
        p_Fm->h_IpcSessions[0])
    {
        t_Error         err;
        t_FmIpcMsg      msg;
        t_FmIpcReply    reply;
        uint32_t        replyLength, ctrlIntr;

        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        msg.msgId = FM_GET_FMAN_CTRL_EVENTS_ENABLE;
        msg.msgBody[0] = eventRegId;
        replyLength = sizeof(uint32_t) + sizeof(uint32_t);
        err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                (uint8_t*)&msg,
                                sizeof(msg.msgId)+sizeof(eventRegId),
                                (uint8_t*)&reply,
                                &replyLength,
                                NULL,
                                NULL);
        if (err != E_OK)
        {
            REPORT_ERROR(MINOR, err, NO_MSG);
            return 0;
        }
        if (replyLength != (sizeof(uint32_t) + sizeof(uint32_t)))
        {
            REPORT_ERROR(MINOR, E_INVALID_VALUE, ("IPC reply length mismatch"));
            return 0;
        }
        memcpy((uint8_t*)&ctrlIntr, reply.replyBody, sizeof(uint32_t));
        return ctrlIntr;
    }
    else if (!p_Fm->p_FmFpmRegs)
    {
        REPORT_ERROR(MINOR, E_NOT_SUPPORTED,
                     ("Either IPC or 'baseAddress' is required!"));
        return 0;
    }

    return GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_cee[eventRegId]);
}

void FmRegisterIntr(t_Handle                h_Fm,
                    e_FmEventModules        module,
                    uint8_t                 modId,
                    e_FmIntrType            intrType,
                    void                    (*f_Isr) (t_Handle h_Arg),
                    t_Handle                h_Arg)
{
    t_Fm                *p_Fm = (t_Fm*)h_Fm;
    int                 event = 0;

    ASSERT_COND(h_Fm);

    GET_FM_MODULE_EVENT(module, modId, intrType, event);
    ASSERT_COND(event < e_FM_EV_DUMMY_LAST);

    /* register in local FM structure */
    p_Fm->intrMng[event].f_Isr = f_Isr;
    p_Fm->intrMng[event].h_SrcHandle = h_Arg;

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        p_Fm->h_IpcSessions[0])
    {
        t_FmIpcRegisterIntr fmIpcRegisterIntr;
        t_Error             err;
        t_FmIpcMsg          msg;

        /* register in Master FM structure */
        fmIpcRegisterIntr.event = (uint32_t)event;
        fmIpcRegisterIntr.guestId = p_Fm->guestId;
        memset(&msg, 0, sizeof(msg));
        msg.msgId = FM_REGISTER_INTR;
        memcpy(msg.msgBody, &fmIpcRegisterIntr, sizeof(fmIpcRegisterIntr));
        err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                (uint8_t*)&msg,
                                sizeof(msg.msgId) + sizeof(fmIpcRegisterIntr),
                                NULL,
                                NULL,
                                NULL,
                                NULL);
        if (err != E_OK)
            REPORT_ERROR(MINOR, err, NO_MSG);
    }
    else if (p_Fm->guestId != NCSW_MASTER_ID)
        REPORT_ERROR(MINOR, E_NOT_SUPPORTED,
                     ("running in guest-mode without IPC!"));
}

void FmUnregisterIntr(t_Handle                  h_Fm,
                        e_FmEventModules        module,
                        uint8_t                 modId,
                        e_FmIntrType            intrType)
{
    t_Fm        *p_Fm = (t_Fm*)h_Fm;
    int         event = 0;

    ASSERT_COND(h_Fm);

    GET_FM_MODULE_EVENT(module, modId,intrType, event);
    ASSERT_COND(event < e_FM_EV_DUMMY_LAST);

    p_Fm->intrMng[event].f_Isr = UnimplementedIsr;
    p_Fm->intrMng[event].h_SrcHandle = NULL;
}

void  FmRegisterFmanCtrlIntr(t_Handle h_Fm, uint8_t eventRegId, void (*f_Isr) (t_Handle h_Arg, uint32_t event), t_Handle    h_Arg)
{
    t_Fm       *p_Fm = (t_Fm*)h_Fm;

    ASSERT_COND(eventRegId<FM_NUM_OF_FMAN_CTRL_EVENT_REGS);

    if (p_Fm->guestId != NCSW_MASTER_ID)
    {
        REPORT_ERROR(MAJOR, E_NOT_SUPPORTED, ("FM in guest-mode"));
        return;
    }

    p_Fm->fmanCtrlIntr[eventRegId].f_Isr = f_Isr;
    p_Fm->fmanCtrlIntr[eventRegId].h_SrcHandle = h_Arg;
}

void  FmUnregisterFmanCtrlIntr(t_Handle h_Fm, uint8_t eventRegId)
{
    t_Fm       *p_Fm = (t_Fm*)h_Fm;

    ASSERT_COND(eventRegId<FM_NUM_OF_FMAN_CTRL_EVENT_REGS);

    if (p_Fm->guestId != NCSW_MASTER_ID)
    {
        REPORT_ERROR(MAJOR, E_NOT_SUPPORTED, ("FM in guest-mode"));
        return;
    }

    p_Fm->fmanCtrlIntr[eventRegId].f_Isr = UnimplementedFmanCtrlIsr;
    p_Fm->fmanCtrlIntr[eventRegId].h_SrcHandle = NULL;
}

void  FmRegisterPcd(t_Handle h_Fm, t_Handle h_FmPcd)
{
    t_Fm       *p_Fm = (t_Fm*)h_Fm;

    if (p_Fm->h_Pcd)
        REPORT_ERROR(MAJOR, E_ALREADY_EXISTS, ("PCD already set"));

    p_Fm->h_Pcd = h_FmPcd;
}

void  FmUnregisterPcd(t_Handle h_Fm)
{
    t_Fm       *p_Fm = (t_Fm*)h_Fm;

    if (!p_Fm->h_Pcd)
        REPORT_ERROR(MAJOR, E_NOT_FOUND, ("PCD handle!"));

    p_Fm->h_Pcd = NULL;
}

t_Handle FmGetPcdHandle(t_Handle h_Fm)
{
    t_Fm       *p_Fm = (t_Fm*)h_Fm;

    return p_Fm->h_Pcd;
}

uint8_t FmGetId(t_Handle h_Fm)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_VALUE(p_Fm, E_INVALID_HANDLE, 0xff);

    return p_Fm->p_FmStateStruct->fmId;
}

t_Error FmSetNumOfRiscsPerPort(t_Handle     h_Fm,
                               uint8_t      hardwarePortId,
                               uint8_t      numOfFmanCtrls,
                               t_FmFmanCtrl orFmanCtrl)
{

    t_Fm                        *p_Fm = (t_Fm*)h_Fm;
    uint32_t                    tmpReg = 0;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(((numOfFmanCtrls > 0) && (numOfFmanCtrls < 3)) , E_INVALID_HANDLE);

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        !p_Fm->p_FmFpmRegs &&
        p_Fm->h_IpcSessions[0])
    {
        t_Error                     err;
        t_FmIpcPortNumOfFmanCtrls   params;
        t_FmIpcMsg                  msg;

        memset(&msg, 0, sizeof(msg));
        params.hardwarePortId = hardwarePortId;
        params.numOfFmanCtrls = numOfFmanCtrls;
        params.orFmanCtrl = orFmanCtrl;
        msg.msgId = FM_SET_NUM_OF_FMAN_CTRL;
        memcpy(msg.msgBody, &params, sizeof(params));
        err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                (uint8_t*)&msg,
                                sizeof(msg.msgId) +sizeof(params),
                                NULL,
                                NULL,
                                NULL,
                                NULL);
        if (err != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
        return E_OK;
    }
    else if (!p_Fm->p_FmFpmRegs)
        RETURN_ERROR(MINOR, E_NOT_SUPPORTED,
                     ("Either IPC or 'baseAddress' is required!"));

    tmpReg = (uint32_t)(hardwarePortId << FPM_PORT_FM_CTL_PORTID_SHIFT);

    /* TODO - maybe to put CTL# according to another criteria */
    if (numOfFmanCtrls == 2)
        tmpReg = FPM_PORT_FM_CTL2 | FPM_PORT_FM_CTL1;

    /* order restoration */
    tmpReg |= (orFmanCtrl << FPM_PRC_ORA_FM_CTL_SEL_SHIFT) | orFmanCtrl;

    WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_prc, tmpReg);

    return E_OK;
}

t_Error FmGetSetPortParams(t_Handle h_Fm,t_FmInterModulePortInitParams *p_PortParams)
{
    t_Fm                    *p_Fm = (t_Fm*)h_Fm;
    t_Error                 err;
    uint32_t                tmpReg, intFlags;
    uint8_t                 hardwarePortId = p_PortParams->hardwarePortId, macId;

    if (p_Fm->guestId != NCSW_MASTER_ID)
    {
        t_FmIpcPortInInitParams     portInParams;
        t_FmIpcPortOutInitParams    portOutParams;
        t_FmIpcMsg                  msg;
        t_FmIpcReply                reply;
        uint32_t                    replyLength;

        portInParams.hardwarePortId     = p_PortParams->hardwarePortId;
        portInParams.enumPortType       = (uint32_t)p_PortParams->portType;
        portInParams.boolIndependentMode= (uint8_t)p_PortParams->independentMode;
        portInParams.liodnOffset        = p_PortParams->liodnOffset;
        portInParams.numOfTasks         = p_PortParams->numOfTasks;
        portInParams.numOfExtraTasks    = p_PortParams->numOfExtraTasks;
        portInParams.numOfOpenDmas      = p_PortParams->numOfOpenDmas;
        portInParams.numOfExtraOpenDmas = p_PortParams->numOfExtraOpenDmas;
        portInParams.sizeOfFifo         = p_PortParams->sizeOfFifo;
        portInParams.extraSizeOfFifo    = p_PortParams->extraSizeOfFifo;
        portInParams.deqPipelineDepth   = p_PortParams->deqPipelineDepth;
        portInParams.maxFrameLength     = p_PortParams->maxFrameLength;
        portInParams.liodnBase          = p_PortParams->liodnBase;

        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        msg.msgId = FM_GET_SET_PORT_PARAMS;
        memcpy(msg.msgBody, &portInParams, sizeof(portInParams));
        replyLength = (sizeof(uint32_t) + sizeof(t_FmIpcPortOutInitParams));
        if ((err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                     (uint8_t*)&msg,
                                     sizeof(msg.msgId) +sizeof(portInParams),
                                     (uint8_t*)&reply,
                                     &replyLength,
                                     NULL,
                                     NULL)) != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
        if (replyLength != (sizeof(uint32_t) + sizeof(t_FmIpcPortOutInitParams)))
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("IPC reply length mismatch"));
        memcpy((uint8_t*)&portOutParams, reply.replyBody, sizeof(t_FmIpcPortOutInitParams));

        p_PortParams->fmMuramPhysBaseAddr.high = portOutParams.ipcPhysAddr.high;
        p_PortParams->fmMuramPhysBaseAddr.low  = portOutParams.ipcPhysAddr.low;
        p_PortParams->numOfTasks = portOutParams.numOfTasks;
        p_PortParams->numOfExtraTasks = portOutParams.numOfExtraTasks;
        p_PortParams->numOfOpenDmas = portOutParams.numOfOpenDmas;
        p_PortParams->numOfExtraOpenDmas = portOutParams.numOfExtraOpenDmas;
        p_PortParams->sizeOfFifo = portOutParams.sizeOfFifo;
        p_PortParams->extraSizeOfFifo = portOutParams.extraSizeOfFifo;

        return (t_Error)(reply.error);
    }

    ASSERT_COND(IN_RANGE(1, hardwarePortId, 63));

    intFlags = XX_LockIntrSpinlock(p_Fm->h_Spinlock);
    if (p_PortParams->independentMode)
    {
        /* set port parameters */
        p_Fm->independentMode = p_PortParams->independentMode;
        /* disable dispatch limit */
        WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_mxd, 0);
    }

    if (p_PortParams->portType == e_FM_PORT_TYPE_OH_HOST_COMMAND)
    {
        if (p_Fm->hcPortInitialized)
        {
            XX_UnlockIntrSpinlock(p_Fm->h_Spinlock, intFlags);
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Only one host command port is allowed."));
        }
        else
            p_Fm->hcPortInitialized = TRUE;
    }
    p_Fm->p_FmStateStruct->portsTypes[hardwarePortId] = p_PortParams->portType;

    err = FmSetNumOfTasks(p_Fm, p_PortParams->hardwarePortId, &p_PortParams->numOfTasks, &p_PortParams->numOfExtraTasks, TRUE);
    if (err)
    {
        XX_UnlockIntrSpinlock(p_Fm->h_Spinlock, intFlags);
        RETURN_ERROR(MAJOR, err, NO_MSG);
    }

#ifdef FM_QMI_NO_DEQ_OPTIONS_SUPPORT
    if (p_Fm->p_FmStateStruct->revInfo.majorRev != 4)
#endif /* FM_QMI_NO_DEQ_OPTIONS_SUPPORT */
    if ((p_PortParams->portType != e_FM_PORT_TYPE_RX) &&
       (p_PortParams->portType != e_FM_PORT_TYPE_RX_10G))
    /* for transmit & O/H ports */
    {
        uint8_t     enqTh;
        uint8_t     deqTh;
        bool        update = FALSE;

        /* update qmi ENQ/DEQ threshold */
        p_Fm->p_FmStateStruct->accumulatedNumOfDeqTnums += p_PortParams->deqPipelineDepth;
        tmpReg = GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_gc);
        enqTh = (uint8_t)(tmpReg>>8);
        /* if enqTh is too big, we reduce it to the max value that is still OK */
        if (enqTh >= (QMI_MAX_NUM_OF_TNUMS - p_Fm->p_FmStateStruct->accumulatedNumOfDeqTnums))
        {
            enqTh = (uint8_t)(QMI_MAX_NUM_OF_TNUMS - p_Fm->p_FmStateStruct->accumulatedNumOfDeqTnums - 1);
            tmpReg &= ~QMI_CFG_ENQ_MASK;
            tmpReg |= ((uint32_t)enqTh << 8);
            update = TRUE;
        }

        deqTh = (uint8_t)tmpReg;
        /* if deqTh is too small, we enlarge it to the min value that is still OK.
         deqTh may not be larger than 63 (QMI_MAX_NUM_OF_TNUMS-1). */
        if ((deqTh <= p_Fm->p_FmStateStruct->accumulatedNumOfDeqTnums)  && (deqTh < QMI_MAX_NUM_OF_TNUMS-1))
        {
            deqTh = (uint8_t)(p_Fm->p_FmStateStruct->accumulatedNumOfDeqTnums + 1);
            tmpReg &= ~QMI_CFG_DEQ_MASK;
            tmpReg |= (uint32_t)deqTh;
            update = TRUE;
        }
        if (update)
            WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_gc, tmpReg);
    }

#ifdef FM_LOW_END_RESTRICTION
    if ((hardwarePortId==0x1) || (hardwarePortId==0x29))
    {
        if (p_Fm->p_FmStateStruct->lowEndRestriction)
        {
            XX_UnlockIntrSpinlock(p_Fm->h_Spinlock, intFlags);
            RETURN_ERROR(MAJOR, E_NOT_AVAILABLE, ("OP #0 cannot work with Tx Port #1."));
        }
        else
            p_Fm->p_FmStateStruct->lowEndRestriction = TRUE;
    }
#endif /* FM_LOW_END_RESTRICTION */

    err = FmSetSizeOfFifo(p_Fm,
                          p_PortParams->hardwarePortId,
                          &p_PortParams->sizeOfFifo,
                          &p_PortParams->extraSizeOfFifo,
                          TRUE);
    if (err)
    {
        XX_UnlockIntrSpinlock(p_Fm->h_Spinlock, intFlags);
        RETURN_ERROR(MAJOR, err, NO_MSG);
    }

    err = FmSetNumOfOpenDmas(p_Fm,
                             p_PortParams->hardwarePortId,
                             &p_PortParams->numOfOpenDmas,
                             &p_PortParams->numOfExtraOpenDmas,
                             TRUE);
    if (err)
    {
        XX_UnlockIntrSpinlock(p_Fm->h_Spinlock, intFlags);
        RETURN_ERROR(MAJOR, err, NO_MSG);
    }

    WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_spliodn[hardwarePortId-1], (uint32_t)p_PortParams->liodnOffset);

    if (p_Fm->p_FmStateStruct->revInfo.majorRev < 6)
    {
        tmpReg = (uint32_t)(hardwarePortId << FPM_PORT_FM_CTL_PORTID_SHIFT);
        if (p_PortParams->independentMode)
        {
            if ((p_PortParams->portType==e_FM_PORT_TYPE_RX) || (p_PortParams->portType==e_FM_PORT_TYPE_RX_10G))
                tmpReg |= (FPM_PORT_FM_CTL1 << FPM_PRC_ORA_FM_CTL_SEL_SHIFT) |FPM_PORT_FM_CTL1;
            else
                tmpReg |= (FPM_PORT_FM_CTL2 << FPM_PRC_ORA_FM_CTL_SEL_SHIFT) |FPM_PORT_FM_CTL2;
        }
        else
        {
            tmpReg |= (FPM_PORT_FM_CTL2|FPM_PORT_FM_CTL1);

            /* order restoration */
            if (hardwarePortId%2)
                tmpReg |= (FPM_PORT_FM_CTL1 << FPM_PRC_ORA_FM_CTL_SEL_SHIFT);
            else
                tmpReg |= (FPM_PORT_FM_CTL2 << FPM_PRC_ORA_FM_CTL_SEL_SHIFT);
        }
        WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_prc, tmpReg);
    }

    /* set LIODN base for this port */
    tmpReg = GET_UINT32(p_Fm->p_FmDmaRegs->fmdmplr[hardwarePortId/2]);
    if (hardwarePortId%2)
    {
        tmpReg &= ~FM_LIODN_BASE_MASK;
        tmpReg |= (uint32_t)p_PortParams->liodnBase;
    }
    else
    {
        tmpReg &= ~(FM_LIODN_BASE_MASK<< DMA_LIODN_SHIFT);
        tmpReg |= (uint32_t)p_PortParams->liodnBase << DMA_LIODN_SHIFT;
    }
    WRITE_UINT32(p_Fm->p_FmDmaRegs->fmdmplr[hardwarePortId/2], tmpReg);

    HW_PORT_ID_TO_SW_PORT_ID(macId, hardwarePortId);

#if defined(FM_MAX_NUM_OF_10G_MACS) && (FM_MAX_NUM_OF_10G_MACS)
    if ((p_PortParams->portType == e_FM_PORT_TYPE_TX_10G) ||
        (p_PortParams->portType == e_FM_PORT_TYPE_RX_10G))
    {
        ASSERT_COND(macId < FM_MAX_NUM_OF_10G_MACS);
        if (p_PortParams->maxFrameLength >= p_Fm->p_FmStateStruct->macMaxFrameLengths10G[macId])
            p_Fm->p_FmStateStruct->portMaxFrameLengths10G[macId] = p_PortParams->maxFrameLength;
        else
            RETURN_ERROR(MINOR, E_INVALID_VALUE, ("Port maxFrameLength is smaller than MAC current MTU"));
    }
    else
#endif /* defined(FM_MAX_NUM_OF_10G_MACS) && ... */
    if ((p_PortParams->portType == e_FM_PORT_TYPE_TX) ||
        (p_PortParams->portType == e_FM_PORT_TYPE_RX))
    {
        ASSERT_COND(macId < FM_MAX_NUM_OF_1G_MACS);
        if (p_PortParams->maxFrameLength >= p_Fm->p_FmStateStruct->macMaxFrameLengths1G[macId])
            p_Fm->p_FmStateStruct->portMaxFrameLengths1G[macId] = p_PortParams->maxFrameLength;
        else
            RETURN_ERROR(MINOR, E_INVALID_VALUE, ("Port maxFrameLength is smaller than MAC current MTU"));
    }

    FmGetPhysicalMuramBase(p_Fm, &p_PortParams->fmMuramPhysBaseAddr);
    XX_UnlockIntrSpinlock(p_Fm->h_Spinlock, intFlags);

    return E_OK;
}

void FmFreePortParams(t_Handle h_Fm,t_FmInterModulePortFreeParams *p_PortParams)
{
    t_Fm                    *p_Fm = (t_Fm*)h_Fm;
    uint32_t                tmpReg, intFlags;
    uint8_t                 hardwarePortId = p_PortParams->hardwarePortId;
    uint8_t                 numOfTasks, macId;
    t_Error                 err;
    t_FmIpcPortFreeParams   portParams;
    t_FmIpcMsg              msg;

    if (p_Fm->guestId != NCSW_MASTER_ID)
    {
        portParams.hardwarePortId = p_PortParams->hardwarePortId;
        portParams.enumPortType = (uint32_t)p_PortParams->portType;
        portParams.deqPipelineDepth = p_PortParams->deqPipelineDepth;
        memset(&msg, 0, sizeof(msg));
        msg.msgId = FM_FREE_PORT;
        memcpy(msg.msgBody, &portParams, sizeof(portParams));
        err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                (uint8_t*)&msg,
                                sizeof(msg.msgId)+sizeof(portParams),
                                NULL,
                                NULL,
                                NULL,
                                NULL);
        if (err != E_OK)
            REPORT_ERROR(MINOR, err, NO_MSG);
        return;
    }

    ASSERT_COND(IN_RANGE(1, hardwarePortId, 63));

    intFlags = XX_LockIntrSpinlock(p_Fm->h_Spinlock);

    if (p_PortParams->portType == e_FM_PORT_TYPE_OH_HOST_COMMAND)
    {
        ASSERT_COND(p_Fm->hcPortInitialized);
        p_Fm->hcPortInitialized = FALSE;
    }

    p_Fm->p_FmStateStruct->portsTypes[hardwarePortId] = e_FM_PORT_TYPE_DUMMY;

    tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1]);
    /* free numOfTasks */
    numOfTasks = (uint8_t)(((tmpReg & BMI_NUM_OF_TASKS_MASK) >> BMI_NUM_OF_TASKS_SHIFT) + 1);
    ASSERT_COND(p_Fm->p_FmStateStruct->accumulatedNumOfTasks >= numOfTasks);
    p_Fm->p_FmStateStruct->accumulatedNumOfTasks -= numOfTasks;

    /* free numOfOpenDmas */
    ASSERT_COND(p_Fm->p_FmStateStruct->accumulatedNumOfOpenDmas >=
        ((tmpReg & BMI_NUM_OF_DMAS_MASK) >> BMI_NUM_OF_DMAS_SHIFT) + 1);
    p_Fm->p_FmStateStruct->accumulatedNumOfOpenDmas -=
        (((tmpReg & BMI_NUM_OF_DMAS_MASK) >> BMI_NUM_OF_DMAS_SHIFT) + 1);

#ifdef FM_HAS_TOTAL_DMAS
    if (p_Fm->p_FmStateStruct->revInfo.majorRev < 6)
    {
        /* update total num of DMA's with committed number of open DMAS, and max uncommitted pool. */
        tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_cfg2) & ~BMI_CFG2_DMAS_MASK;
        tmpReg |= (uint32_t)(p_Fm->p_FmStateStruct->accumulatedNumOfOpenDmas + p_Fm->p_FmStateStruct->extraOpenDmasPoolSize - 1) << BMI_CFG2_DMAS_SHIFT;
        WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_cfg2, tmpReg);
    }
#endif /* FM_HAS_TOTAL_DMAS */

    /* free sizeOfFifo */
    tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_pfs[hardwarePortId-1]);
    ASSERT_COND(p_Fm->p_FmStateStruct->accumulatedFifoSize >=
                (((tmpReg & BMI_FIFO_SIZE_MASK) + 1) * BMI_FIFO_UNITS));
    p_Fm->p_FmStateStruct->accumulatedFifoSize -=
        (((tmpReg & BMI_FIFO_SIZE_MASK) + 1) * BMI_FIFO_UNITS);

#ifdef FM_QMI_NO_DEQ_OPTIONS_SUPPORT
    if (p_Fm->p_FmStateStruct->revInfo.majorRev != 4)
#endif /* FM_QMI_NO_DEQ_OPTIONS_SUPPORT */
    if ((p_PortParams->portType != e_FM_PORT_TYPE_RX) &&
        (p_PortParams->portType != e_FM_PORT_TYPE_RX_10G))
    /* for transmit & O/H ports */
    {
        uint8_t     enqTh;
        uint8_t     deqTh;

        tmpReg = GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_gc);
        /* update qmi ENQ/DEQ threshold */
        p_Fm->p_FmStateStruct->accumulatedNumOfDeqTnums -= p_PortParams->deqPipelineDepth;

        /* p_Fm->p_FmStateStruct->accumulatedNumOfDeqTnums is now smaller,
           so we can enlarge enqTh */
        enqTh = (uint8_t)(QMI_MAX_NUM_OF_TNUMS - p_Fm->p_FmStateStruct->accumulatedNumOfDeqTnums - 1);
        tmpReg &= ~QMI_CFG_ENQ_MASK;
        tmpReg |= ((uint32_t)enqTh << QMI_CFG_ENQ_SHIFT);

         /* p_Fm->p_FmStateStruct->accumulatedNumOfDeqTnums is now smaller,
            so we can reduce deqTh */
        deqTh = (uint8_t)(p_Fm->p_FmStateStruct->accumulatedNumOfDeqTnums + 1);
        tmpReg &= ~QMI_CFG_DEQ_MASK;
        tmpReg |= (uint32_t)deqTh;

        WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_gc, tmpReg);
    }

    HW_PORT_ID_TO_SW_PORT_ID(macId, hardwarePortId);

    /* Delete prefetch configuration*/
    p_Fm->portsPreFetchConfigured[macId] = FALSE;
    p_Fm->portsPreFetchValue[macId]      = FALSE;

#if defined(FM_MAX_NUM_OF_10G_MACS) && (FM_MAX_NUM_OF_10G_MACS)
    if ((p_PortParams->portType == e_FM_PORT_TYPE_TX_10G) ||
        (p_PortParams->portType == e_FM_PORT_TYPE_RX_10G))
    {
        ASSERT_COND(macId < FM_MAX_NUM_OF_10G_MACS);
        p_Fm->p_FmStateStruct->portMaxFrameLengths10G[macId] = 0;
    }
    else
#endif /* defined(FM_MAX_NUM_OF_10G_MACS) && ... */
    if ((p_PortParams->portType == e_FM_PORT_TYPE_TX) ||
        (p_PortParams->portType == e_FM_PORT_TYPE_RX))
    {
        ASSERT_COND(macId < FM_MAX_NUM_OF_1G_MACS);
        p_Fm->p_FmStateStruct->portMaxFrameLengths1G[macId] = 0;
    }

#ifdef FM_LOW_END_RESTRICTION
    if ((hardwarePortId==0x1) || (hardwarePortId==0x29))
        p_Fm->p_FmStateStruct->lowEndRestriction = FALSE;
#endif /* FM_LOW_END_RESTRICTION */
    XX_UnlockIntrSpinlock(p_Fm->h_Spinlock, intFlags);
}

t_Error FmIsPortStalled(t_Handle h_Fm, uint8_t hardwarePortId, bool *p_IsStalled)
{
    t_Fm            *p_Fm = (t_Fm*)h_Fm;
    uint32_t        tmpReg;
    t_Error         err;
    t_FmIpcMsg      msg;
    t_FmIpcReply    reply;
    uint32_t        replyLength;

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        !p_Fm->baseAddr &&
        p_Fm->h_IpcSessions[0])
    {
        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        msg.msgId = FM_IS_PORT_STALLED;
        msg.msgBody[0] = hardwarePortId;
        replyLength = sizeof(uint32_t) + sizeof(uint8_t);
        err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                (uint8_t*)&msg,
                                sizeof(msg.msgId)+sizeof(hardwarePortId),
                                (uint8_t*)&reply,
                                &replyLength,
                                NULL,
                                NULL);
        if (err != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
        if (replyLength != (sizeof(uint32_t) + sizeof(uint8_t)))
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("IPC reply length mismatch"));

        *p_IsStalled = (bool)!!(*(uint8_t*)(reply.replyBody));

        return (t_Error)(reply.error);
    }
    else if (!p_Fm->baseAddr)
        RETURN_ERROR(MINOR, E_NOT_SUPPORTED,
                     ("Either IPC or 'baseAddress' is required!"));

    tmpReg = GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_ps[hardwarePortId]);
    *p_IsStalled = (bool)!!(tmpReg & FPM_PS_STALLED);

    return E_OK;
}

t_Error FmResumeStalledPort(t_Handle h_Fm, uint8_t hardwarePortId)
{
    t_Fm            *p_Fm = (t_Fm*)h_Fm;
    uint32_t        tmpReg;
    t_Error         err;
    bool            isStalled;

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        !p_Fm->baseAddr &&
        p_Fm->h_IpcSessions[0])
    {
        t_FmIpcMsg      msg;
        t_FmIpcReply    reply;
        uint32_t        replyLength;

        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        msg.msgId = FM_RESUME_STALLED_PORT;
        msg.msgBody[0] = hardwarePortId;
        replyLength = sizeof(uint32_t);
        err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                (uint8_t*)&msg,
                                sizeof(msg.msgId) + sizeof(hardwarePortId),
                                (uint8_t*)&reply,
                                &replyLength,
                                NULL,
                                NULL);
        if (err != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
        if (replyLength != sizeof(uint32_t))
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("IPC reply length mismatch"));
        return (t_Error)(reply.error);
    }
    else if (!p_Fm->baseAddr)
        RETURN_ERROR(MINOR, E_NOT_SUPPORTED,
                     ("Either IPC or 'baseAddress' is required!"));

    if (p_Fm->p_FmStateStruct->revInfo.majorRev >= 6)
        RETURN_ERROR(MINOR, E_NOT_AVAILABLE, ("Not available for this FM revision!"));

    /* Get port status */
    err = FmIsPortStalled(h_Fm, hardwarePortId, &isStalled);
    if (err)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Can't get port status"));
    if (!isStalled)
        return E_OK;

    tmpReg = (uint32_t)((hardwarePortId << FPM_PORT_FM_CTL_PORTID_SHIFT) | FPM_PRC_REALSE_STALLED);
    WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_prc, tmpReg);

    return E_OK;
}

t_Error FmResetMac(t_Handle h_Fm, e_FmMacType type, uint8_t macId)
{
    t_Fm                *p_Fm = (t_Fm*)h_Fm;
    uint32_t            bitMask;

#ifndef FM_MAC_RESET
    if (p_Fm->p_FmStateStruct->revInfo.majorRev >= 6)
        return E_OK;
#endif /*(FM_MAC_RESET >= 11)*/

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        !p_Fm->baseAddr &&
        p_Fm->h_IpcSessions[0])
    {
        t_FmIpcMacParams    macParams;
        t_Error             err;
        t_FmIpcMsg          msg;
        t_FmIpcReply        reply;
        uint32_t            replyLength;

        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        macParams.id = macId;
        macParams.enumType = (uint32_t)type;
        msg.msgId = FM_RESET_MAC;
        memcpy(msg.msgBody,  &macParams, sizeof(macParams));
        replyLength = sizeof(uint32_t);
        err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                (uint8_t*)&msg,
                                sizeof(msg.msgId)+sizeof(macParams),
                                (uint8_t*)&reply,
                                &replyLength,
                                NULL,
                                NULL);
        if (err != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
        if (replyLength != sizeof(uint32_t))
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("IPC reply length mismatch"));
        return (t_Error)(reply.error);
    }
    else if (!p_Fm->baseAddr)
        RETURN_ERROR(MINOR, E_NOT_SUPPORTED,
                     ("Either IPC or 'baseAddress' is required!"));

    /* Get the relevant bit mask */
    if (type == e_FM_MAC_10G)
    {
        switch (macId)
        {
            case (0):
                bitMask = FPM_RSTC_10G0_RESET;
                break;
            case (1):
                bitMask = FPM_RSTC_10G1_RESET;
                break;
            default:
                RETURN_ERROR(MINOR, E_INVALID_VALUE, ("Illegal MAC Id"));
        }
    }
    else
    {
        switch (macId)
        {
            case (0):
                bitMask = FPM_RSTC_1G0_RESET;
                break;
            case (1):
                bitMask = FPM_RSTC_1G1_RESET;
                break;
            case (2):
                bitMask = FPM_RSTC_1G2_RESET;
                break;
            case (3):
                bitMask = FPM_RSTC_1G3_RESET;
                break;
            case (4):
                bitMask = FPM_RSTC_1G4_RESET;
                break;
            case (5):
                bitMask = FPM_RSTC_1G5_RESET;
                break;
            case (6):
                bitMask = FPM_RSTC_1G6_RESET;
                break;
            case (7):
                bitMask = FPM_RSTC_1G7_RESET;
                break;
            default:
                RETURN_ERROR(MINOR, E_INVALID_VALUE, ("Illegal MAC Id"));
        }
    }

    /* reset */
    WRITE_UINT32(p_Fm->p_FmFpmRegs->fm_rstc, bitMask);

    return E_OK;
}

t_Error FmSetMacMaxFrame(t_Handle h_Fm, e_FmMacType type, uint8_t macId, uint16_t mtu)
{
    t_Fm                        *p_Fm = (t_Fm*)h_Fm;

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        p_Fm->h_IpcSessions[0])
    {
        t_FmIpcMacMaxFrameParams    macMaxFrameLengthParams;
        t_Error                     err;
        t_FmIpcMsg                  msg;

        memset(&msg, 0, sizeof(msg));
        macMaxFrameLengthParams.macParams.id = macId;
        macMaxFrameLengthParams.macParams.enumType = (uint32_t)type;
        macMaxFrameLengthParams.maxFrameLength = (uint16_t)mtu;
        msg.msgId = FM_SET_MAC_MAX_FRAME;
        memcpy(msg.msgBody,  &macMaxFrameLengthParams, sizeof(macMaxFrameLengthParams));
        err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                (uint8_t*)&msg,
                                sizeof(msg.msgId)+sizeof(macMaxFrameLengthParams),
                                NULL,
                                NULL,
                                NULL,
                                NULL);
        if (err != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
        return E_OK;
    }
    else if (p_Fm->guestId != NCSW_MASTER_ID)
        RETURN_ERROR(MINOR, E_NOT_SUPPORTED,
                     ("running in guest-mode without IPC!"));

    /* if port is already initialized, check that MaxFrameLength is smaller
     * or equal to the port's max */
#if (defined(FM_MAX_NUM_OF_10G_MACS) && (FM_MAX_NUM_OF_10G_MACS))
    if (type == e_FM_MAC_10G)
    {
        if ((!p_Fm->p_FmStateStruct->portMaxFrameLengths10G[macId])
           || (p_Fm->p_FmStateStruct->portMaxFrameLengths10G[macId] &&
              (mtu <= p_Fm->p_FmStateStruct->portMaxFrameLengths10G[macId])))
               p_Fm->p_FmStateStruct->macMaxFrameLengths10G[macId] = mtu;
        else
            RETURN_ERROR(MINOR, E_INVALID_VALUE, ("MAC maxFrameLength is larger than Port maxFrameLength"));

    }
    else
#else
    UNUSED(type);
#endif /* (defined(FM_MAX_NUM_OF_10G_MACS) && ... */
    if ((!p_Fm->p_FmStateStruct->portMaxFrameLengths1G[macId])
       || (p_Fm->p_FmStateStruct->portMaxFrameLengths1G[macId] &&
          (mtu <= p_Fm->p_FmStateStruct->portMaxFrameLengths1G[macId])))
        p_Fm->p_FmStateStruct->macMaxFrameLengths1G[macId] = mtu;
    else
        RETURN_ERROR(MINOR, E_INVALID_VALUE, ("MAC maxFrameLength is larger than Port maxFrameLength"));

    return E_OK;
}

uint16_t FmGetClockFreq(t_Handle h_Fm)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    /* for multicore environment: this depends on the
     * fact that fmClkFreq was properly initialized at "init". */
    return p_Fm->p_FmStateStruct->fmClkFreq;
}

uint32_t FmGetTimeStampScale(t_Handle h_Fm)
{
    t_Fm                *p_Fm = (t_Fm*)h_Fm;

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        !p_Fm->baseAddr &&
        p_Fm->h_IpcSessions[0])
    {
        t_Error             err;
        t_FmIpcMsg          msg;
        t_FmIpcReply        reply;
        uint32_t            replyLength, timeStamp;

        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        msg.msgId = FM_GET_TIMESTAMP_SCALE;
        replyLength = sizeof(uint32_t) + sizeof(uint32_t);
        if ((err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                     (uint8_t*)&msg,
                                     sizeof(msg.msgId),
                                     (uint8_t*)&reply,
                                     &replyLength,
                                     NULL,
                                     NULL)) != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
        if (replyLength != (sizeof(uint32_t) + sizeof(uint32_t)))
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("IPC reply length mismatch"));

        memcpy((uint8_t*)&timeStamp, reply.replyBody, sizeof(uint32_t));
        return timeStamp;
    }
    else if ((p_Fm->guestId != NCSW_MASTER_ID) &&
             p_Fm->baseAddr)
    {
        if (!(GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_tsc1) & FPM_TS_CTL_EN))
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("timestamp is not enabled!"));

        return p_Fm->p_FmStateStruct->count1MicroBit;
    }
    else if (p_Fm->guestId != NCSW_MASTER_ID)
        DBG(WARNING, ("No IPC - can't validate FM if timestamp enabled."));

    return p_Fm->p_FmStateStruct->count1MicroBit;
}

t_Error FmEnableRamsEcc(t_Handle h_Fm)
{
    t_Fm        *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);

    p_Fm->p_FmStateStruct->ramsEccOwners++;
    p_Fm->p_FmStateStruct->internalCall = TRUE;

    return FM_EnableRamsEcc(p_Fm);
}

t_Error FmDisableRamsEcc(t_Handle h_Fm)
{
    t_Fm        *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);

    ASSERT_COND(p_Fm->p_FmStateStruct->ramsEccOwners);
    p_Fm->p_FmStateStruct->ramsEccOwners--;

    if (p_Fm->p_FmStateStruct->ramsEccOwners==0)
    {
        p_Fm->p_FmStateStruct->internalCall = TRUE;
        return FM_DisableRamsEcc(p_Fm);
    }

    return E_OK;
}

uint8_t FmGetGuestId(t_Handle h_Fm)
{
    t_Fm     *p_Fm = (t_Fm*)h_Fm;

    return p_Fm->guestId;
}

bool FmIsMaster(t_Handle h_Fm)
{
    t_Fm     *p_Fm = (t_Fm*)h_Fm;

    return (p_Fm->guestId == NCSW_MASTER_ID);
}

#ifdef FM_NO_GUARANTEED_RESET_VALUES
t_Error FmSetSizeOfFifo(t_Handle    h_Fm,
                        uint8_t     hardwarePortId,
                        uint32_t    *p_SizeOfFifo,
                        uint32_t    *p_ExtraSizeOfFifo,
                        bool        initialConfig)
{
    t_Fm                    *p_Fm = (t_Fm*)h_Fm;
    t_FmIpcPortRsrcParams   rsrcParams;
    t_Error                 err;
    uint32_t                tmpReg = 0, sizeOfFifo = *p_SizeOfFifo, extraSizeOfFifo = *p_ExtraSizeOfFifo;
    uint16_t                oldVal = 0;

    ASSERT_COND(IN_RANGE(1, hardwarePortId, 63));

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        !p_Fm->baseAddr &&
        p_Fm->h_IpcSessions[0])
    {
        t_FmIpcMsg          msg;
        t_FmIpcReply        reply;
        uint32_t            replyLength;

        rsrcParams.hardwarePortId = hardwarePortId;
        rsrcParams.val = sizeOfFifo;
        rsrcParams.extra = extraSizeOfFifo;
        rsrcParams.boolInitialConfig = (uint8_t)initialConfig;

        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        msg.msgId = FM_SET_SIZE_OF_FIFO;
        memcpy(msg.msgBody, &rsrcParams, sizeof(rsrcParams));
        replyLength = sizeof(uint32_t);
        if ((err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                     (uint8_t*)&msg,
                                     sizeof(msg.msgId) + sizeof(rsrcParams),
                                     (uint8_t*)&reply,
                                     &replyLength,
                                     NULL,
                                     NULL)) != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
        if (replyLength != sizeof(uint32_t))
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("IPC reply length mismatch"));
        return (t_Error)(reply.error);
    }
    else if ((p_Fm->guestId != NCSW_MASTER_ID) &&
             p_Fm->baseAddr)
    {
        DBG(WARNING, ("No IPC - can't validate FM total-fifo size."));

        tmpReg = (uint32_t)((sizeOfFifo/BMI_FIFO_UNITS - 1) |
                            ((extraSizeOfFifo/BMI_FIFO_UNITS) << BMI_EXTRA_FIFO_SIZE_SHIFT));
        WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_pfs[hardwarePortId-1], tmpReg);
    }
    else if (p_Fm->guestId != NCSW_MASTER_ID)
        RETURN_ERROR(MAJOR, E_NOT_SUPPORTED,
                     ("running in guest-mode without neither IPC nor mapped register!"));

    if (!initialConfig)
    {
        tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_pfs[hardwarePortId-1]);
        /* read into oldVal the current extra fifo size */
        oldVal = (uint16_t)((((tmpReg & BMI_EXTRA_FIFO_SIZE_MASK) + 1) * BMI_FIFO_UNITS) >> BMI_EXTRA_FIFO_SIZE_SHIFT);
    }

    if (extraSizeOfFifo > oldVal)
    {
        if (extraSizeOfFifo && !p_Fm->p_FmStateStruct->extraFifoPoolSize)
            /* if this is the first time a port requires extraFifoPoolSize, the total extraFifoPoolSize
             * must be initialized to 1 buffer per port
             */
            p_Fm->p_FmStateStruct->extraFifoPoolSize = FM_MAX_NUM_OF_RX_PORTS*BMI_FIFO_UNITS;

        p_Fm->p_FmStateStruct->extraFifoPoolSize = MAX(p_Fm->p_FmStateStruct->extraFifoPoolSize, extraSizeOfFifo);
    }

    if (!initialConfig)
        /* read into oldVal the current num of tasks */
        oldVal = (uint16_t)(((tmpReg & BMI_FIFO_SIZE_MASK) + 1) * BMI_FIFO_UNITS);

    /* check that there are enough uncommitted fifo size */
    if ((p_Fm->p_FmStateStruct->accumulatedFifoSize - oldVal + sizeOfFifo) >
        (p_Fm->p_FmStateStruct->totalFifoSize - p_Fm->p_FmStateStruct->extraFifoPoolSize))
        RETURN_ERROR(MAJOR, E_NOT_AVAILABLE, ("Requested fifo size and extra size exceed total FIFO size."));
    else
    {
        /* update accumulated */
        ASSERT_COND(p_Fm->p_FmStateStruct->accumulatedFifoSize >= oldVal);
        p_Fm->p_FmStateStruct->accumulatedFifoSize -= oldVal;
        p_Fm->p_FmStateStruct->accumulatedFifoSize += sizeOfFifo;
        /* calculate reg */
        tmpReg = (uint32_t)((sizeOfFifo/BMI_FIFO_UNITS - 1) |
                            ((extraSizeOfFifo/BMI_FIFO_UNITS) << BMI_EXTRA_FIFO_SIZE_SHIFT));
        WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_pfs[hardwarePortId-1], tmpReg);
    }

    return E_OK;
}

#else /*FM_NO_GUARANTEED_RESET_VALUES*/
t_Error FmSetSizeOfFifo(t_Handle    h_Fm,
                        uint8_t     hardwarePortId,
                        uint32_t    *p_SizeOfFifo,
                        uint32_t    *p_ExtraSizeOfFifo,
                        bool        initialConfig)
{
    t_Fm                    *p_Fm = (t_Fm*)h_Fm;
    t_FmIpcPortRsrcParams   rsrcParams;
    t_Error                 err;
    uint32_t                tmpReg = 0, sizeOfFifo = *p_SizeOfFifo, extraSizeOfFifo = *p_ExtraSizeOfFifo;
    uint16_t                currentVal, currentExtraVal;

    ASSERT_COND(IN_RANGE(1, hardwarePortId, 63));
    /* it's illegal to be in a state where this is not the first set and no value is specified */
    ASSERT_COND(initialConfig || sizeOfFifo);

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        !p_Fm->baseAddr &&
        p_Fm->h_IpcSessions[0])
    {
        t_FmIpcMsg          msg;
        t_FmIpcReply        reply;
        uint32_t            replyLength;

        rsrcParams.hardwarePortId = hardwarePortId;
        rsrcParams.val = sizeOfFifo;
        rsrcParams.extra = extraSizeOfFifo;
        rsrcParams.boolInitialConfig = (uint8_t)initialConfig;

        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        msg.msgId = FM_SET_SIZE_OF_FIFO;
        memcpy(msg.msgBody, &rsrcParams, sizeof(rsrcParams));
        replyLength = sizeof(uint32_t);
        if ((err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                     (uint8_t*)&msg,
                                     sizeof(msg.msgId) + sizeof(rsrcParams),
                                     (uint8_t*)&reply,
                                     &replyLength,
                                     NULL,
                                     NULL)) != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
        if (replyLength != sizeof(uint32_t))
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("IPC reply length mismatch"));
        return (t_Error)(reply.error);
    }
    else if ((p_Fm->guestId != NCSW_MASTER_ID) &&
             p_Fm->baseAddr)
    {
        DBG(WARNING, ("No IPC - can't validate FM total-fifo size."));

        if (sizeOfFifo)
        {
            /* whether it is the first time with explicit value, or runtime "set" - write register */
            tmpReg = (uint32_t)((sizeOfFifo/BMI_FIFO_UNITS - 1) |
                                ((extraSizeOfFifo/BMI_FIFO_UNITS) << BMI_EXTRA_FIFO_SIZE_SHIFT));
            WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_pfs[hardwarePortId-1], tmpReg);
        }
        else /* first config without explicit value: Do Nothing - reset value shouldn't be
                changed, read register for port save */
        {
            tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_pfs[hardwarePortId-1]);
            /* read into oldVal the current extra fifo size */
            *p_ExtraSizeOfFifo = (uint16_t)((((tmpReg & BMI_EXTRA_FIFO_SIZE_MASK) + 1) * BMI_FIFO_UNITS) >> BMI_EXTRA_FIFO_SIZE_SHIFT);
            *p_SizeOfFifo = (uint16_t)(((tmpReg & BMI_FIFO_SIZE_MASK) + 1) * BMI_FIFO_UNITS);
        }
    }
    else if (p_Fm->guestId != NCSW_MASTER_ID)
        RETURN_ERROR(MAJOR, E_NOT_SUPPORTED,
                     ("running in guest-mode without neither IPC nor mapped register!"));

    if (!initialConfig || !sizeOfFifo)
    {
        /* !initialConfig - runtime change of existing value.
         * !numOfTasks - first configuration according to values in regs.
         * In both cases: read the current FIFO size */
        tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_pfs[hardwarePortId-1]);
        /* read into oldVal the current extra fifo size */
        currentExtraVal = (uint16_t)((((tmpReg & BMI_EXTRA_FIFO_SIZE_MASK) + 1) * BMI_FIFO_UNITS) >> BMI_EXTRA_FIFO_SIZE_SHIFT);
        currentVal = (uint16_t)(((tmpReg & BMI_FIFO_SIZE_MASK) + 1) * BMI_FIFO_UNITS);
    }
    else /* first time and sizeOfFifo explicitly specified */
        currentVal = currentExtraVal = 0;

    if (!sizeOfFifo)
    {
        /* This is the first configuration and user did not specify value (!numOfTasks),
         * reset values will be used and we just save these values for resource management */
        if (currentExtraVal)
        {
            if (!p_Fm->p_FmStateStruct->extraFifoPoolSize)
                /* if this is the first time a port requires extraFifoPoolSize, the total extraFifoPoolSize
                 * must be initialized to 1 buffer per port
                 */
                p_Fm->p_FmStateStruct->extraFifoPoolSize = FM_MAX_NUM_OF_RX_PORTS*BMI_FIFO_UNITS;

            p_Fm->p_FmStateStruct->extraFifoPoolSize = MAX(p_Fm->p_FmStateStruct->extraFifoPoolSize, extraSizeOfFifo);
        }
        if ((p_Fm->p_FmStateStruct->accumulatedFifoSize + currentVal) >
            (p_Fm->p_FmStateStruct->totalFifoSize - p_Fm->p_FmStateStruct->extraFifoPoolSize))
            RETURN_ERROR(MAJOR, E_NOT_AVAILABLE, ("Total port's fifo size and extra size exceed total available FIFO size."));

        p_Fm->p_FmStateStruct->accumulatedFifoSize += currentVal;

        *p_SizeOfFifo = currentVal;
        *p_ExtraSizeOfFifo = currentExtraVal;

    }
    else
    {
        /* user requires a specific value.
         * If this is the first configuration call, (numOfTasks != 0) currentVal & currentExtraVal are set to "0",
         * otherwise they hold the value written in the register.
         */
        if (extraSizeOfFifo > currentExtraVal)
        {
            if (extraSizeOfFifo && !p_Fm->p_FmStateStruct->extraFifoPoolSize)
                /* if this is the first time a port requires extraFifoPoolSize, the total extraFifoPoolSize
                 * must be initialized to 1 buffer per port
                 */
                p_Fm->p_FmStateStruct->extraFifoPoolSize = FM_MAX_NUM_OF_RX_PORTS*BMI_FIFO_UNITS;

            p_Fm->p_FmStateStruct->extraFifoPoolSize = MAX(p_Fm->p_FmStateStruct->extraFifoPoolSize, extraSizeOfFifo);
        }

        /* check that there are enough uncommitted fifo size */
        if ((p_Fm->p_FmStateStruct->accumulatedFifoSize - currentVal + sizeOfFifo) >
            (p_Fm->p_FmStateStruct->totalFifoSize - p_Fm->p_FmStateStruct->extraFifoPoolSize))
            RETURN_ERROR(MAJOR, E_NOT_AVAILABLE, ("Requested fifo size and extra size exceed total FIFO size."));
        else
        {
            /* update accumulated */
            ASSERT_COND(p_Fm->p_FmStateStruct->accumulatedFifoSize >= currentVal);
            p_Fm->p_FmStateStruct->accumulatedFifoSize -= currentVal;
            p_Fm->p_FmStateStruct->accumulatedFifoSize += sizeOfFifo;
            /* calculate reg */
            tmpReg = (uint32_t)((sizeOfFifo/BMI_FIFO_UNITS - 1) |
                                ((extraSizeOfFifo/BMI_FIFO_UNITS) << BMI_EXTRA_FIFO_SIZE_SHIFT));
            WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_pfs[hardwarePortId-1], tmpReg);
        }
        *p_SizeOfFifo = sizeOfFifo;
        *p_ExtraSizeOfFifo = extraSizeOfFifo;

    }

    return E_OK;
}
#endif /* FM_NO_GUARANTEED_RESET_VALUES */

#ifdef FM_NO_GUARANTEED_RESET_VALUES
t_Error FmSetNumOfTasks(t_Handle    h_Fm,
                        uint8_t     hardwarePortId,
                        uint8_t     *p_NumOfTasks,
                        uint8_t     *p_NumOfExtraTasks,
                        bool        initialConfig)
{
    t_Fm                    *p_Fm = (t_Fm *)h_Fm;
    t_Error                 err;
    uint32_t                tmpReg = 0;
    uint8_t                 oldVal = 0, numOfTasks = *p_NumOfTasks, numOfExtraTasks = *p_NumOfExtraTasks;

    ASSERT_COND(IN_RANGE(1, hardwarePortId, 63));

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        !p_Fm->baseAddr &&
        p_Fm->h_IpcSessions[0])
    {
        t_FmIpcPortRsrcParams   rsrcParams;
        t_FmIpcMsg              msg;
        t_FmIpcReply            reply;
        uint32_t                replyLength;

        rsrcParams.hardwarePortId = hardwarePortId;
        rsrcParams.val = numOfTasks;
        rsrcParams.extra = numOfExtraTasks;
        rsrcParams.boolInitialConfig = (uint8_t)initialConfig;

        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        msg.msgId = FM_SET_NUM_OF_TASKS;
        memcpy(msg.msgBody, &rsrcParams, sizeof(rsrcParams));
        replyLength = sizeof(uint32_t);
        if ((err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                     (uint8_t*)&msg,
                                     sizeof(msg.msgId) + sizeof(rsrcParams),
                                     (uint8_t*)&reply,
                                     &replyLength,
                                     NULL,
                                     NULL)) != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
        if (replyLength != sizeof(uint32_t))
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("IPC reply length mismatch"));
        return (t_Error)(reply.error);
    }
    else if ((p_Fm->guestId != NCSW_MASTER_ID) &&
             p_Fm->baseAddr)
    {
        DBG(WARNING, ("No IPC - can't validate FM total-num-of-tasks."));

        /* calculate reg */
        tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1]) & ~(BMI_NUM_OF_TASKS_MASK | BMI_NUM_OF_EXTRA_TASKS_MASK);
        tmpReg |= (uint32_t)(((numOfTasks-1) << BMI_NUM_OF_TASKS_SHIFT) |
                    (numOfExtraTasks << BMI_EXTRA_NUM_OF_TASKS_SHIFT));
        WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1],tmpReg);
    }
    else if (p_Fm->guestId != NCSW_MASTER_ID)
        RETURN_ERROR(MAJOR, E_NOT_SUPPORTED,
                     ("running in guest-mode without neither IPC nor mapped register!"));

    if (!initialConfig)
    {
        tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1]);
        /* read into oldVal the current extra tasks */
        oldVal = (uint8_t)((tmpReg & BMI_NUM_OF_EXTRA_TASKS_MASK) >> BMI_EXTRA_NUM_OF_TASKS_SHIFT);
    }

    if (numOfExtraTasks > oldVal)
        p_Fm->p_FmStateStruct->extraTasksPoolSize =
            (uint8_t)MAX(p_Fm->p_FmStateStruct->extraTasksPoolSize, numOfExtraTasks);

    if (!initialConfig)
        /* read into oldVal the current num of tasks */
        oldVal = (uint8_t)(((tmpReg & BMI_NUM_OF_TASKS_MASK) >> BMI_NUM_OF_TASKS_SHIFT) + 1);

    /* check that there are enough uncommitted tasks */
    if ((p_Fm->p_FmStateStruct->accumulatedNumOfTasks - oldVal + numOfTasks) >
       (p_Fm->p_FmStateStruct->totalNumOfTasks - p_Fm->p_FmStateStruct->extraTasksPoolSize))
        RETURN_ERROR(MAJOR, E_NOT_AVAILABLE,
                     ("Requested numOfTasks and extra tasks pool for fm%d exceed total numOfTasks.",
                      p_Fm->p_FmStateStruct->fmId));
    else
    {
        ASSERT_COND(p_Fm->p_FmStateStruct->accumulatedNumOfTasks >= oldVal);
        /* update accumulated */
        p_Fm->p_FmStateStruct->accumulatedNumOfTasks -= oldVal;
        p_Fm->p_FmStateStruct->accumulatedNumOfTasks += numOfTasks;
        /* calculate reg */
        tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1]) & ~(BMI_NUM_OF_TASKS_MASK | BMI_NUM_OF_EXTRA_TASKS_MASK);
        tmpReg |= (uint32_t)(((numOfTasks-1) << BMI_NUM_OF_TASKS_SHIFT) |
                    (numOfExtraTasks << BMI_EXTRA_NUM_OF_TASKS_SHIFT));
        WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1],tmpReg);
    }

    return E_OK;
}

#else /*FM_NO_GUARANTEED_RESET_VALUES*/
t_Error FmSetNumOfTasks(t_Handle    h_Fm,
                        uint8_t     hardwarePortId,
                        uint8_t     *p_NumOfTasks,
                        uint8_t     *p_NumOfExtraTasks,
                        bool        initialConfig)
{
    t_Fm                    *p_Fm = (t_Fm *)h_Fm;
    t_Error                 err;
    uint32_t                tmpReg = 0;
    uint8_t                 currentVal, currentExtraVal,numOfTasks = *p_NumOfTasks, numOfExtraTasks = *p_NumOfExtraTasks;

    ASSERT_COND(IN_RANGE(1, hardwarePortId, 63));
    /* it's illegal to be in a state where this is not the first set and no value is specified */
    ASSERT_COND(initialConfig || numOfTasks);

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        !p_Fm->baseAddr &&
        p_Fm->h_IpcSessions[0])
    {
        t_FmIpcPortRsrcParams   rsrcParams;
        t_FmIpcMsg              msg;
        t_FmIpcReply            reply;
        uint32_t                replyLength;

        rsrcParams.hardwarePortId = hardwarePortId;
        rsrcParams.val = numOfTasks;
        rsrcParams.extra = numOfExtraTasks;
        rsrcParams.boolInitialConfig = (uint8_t)initialConfig;

        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        msg.msgId = FM_SET_NUM_OF_TASKS;
        memcpy(msg.msgBody, &rsrcParams, sizeof(rsrcParams));
        replyLength = sizeof(uint32_t);
        if ((err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                     (uint8_t*)&msg,
                                     sizeof(msg.msgId) + sizeof(rsrcParams),
                                     (uint8_t*)&reply,
                                     &replyLength,
                                     NULL,
                                     NULL)) != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
        if (replyLength != sizeof(uint32_t))
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("IPC reply length mismatch"));
        return (t_Error)(reply.error);
    }
    else if ((p_Fm->guestId != NCSW_MASTER_ID) &&
             p_Fm->baseAddr)
    {
        DBG(WARNING, ("No Ipc - can't validate FM total-num-of-tasks."));

        if (numOfTasks)
        {
            /* whether it is the first time with explicit value, or runtime "set" - write register */
            tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1]) & ~(BMI_NUM_OF_TASKS_MASK | BMI_NUM_OF_EXTRA_TASKS_MASK);
            tmpReg |= (uint32_t)(((numOfTasks-1) << BMI_NUM_OF_TASKS_SHIFT) |
                        (numOfExtraTasks << BMI_EXTRA_NUM_OF_TASKS_SHIFT));
            WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1],tmpReg);
        }
        else /* first config without explicit value: Do Nothing - reset value shouldn't be
                 changed, read register for port save */
         {
            tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1]);
            *p_NumOfTasks = (uint8_t)(((tmpReg & BMI_NUM_OF_TASKS_MASK) >> BMI_NUM_OF_TASKS_SHIFT) + 1);
            *p_NumOfExtraTasks = (uint8_t)((tmpReg & BMI_NUM_OF_EXTRA_TASKS_MASK) >> BMI_EXTRA_NUM_OF_TASKS_SHIFT);
         }

    }
    else if (p_Fm->guestId != NCSW_MASTER_ID)
        RETURN_ERROR(MAJOR, E_NOT_SUPPORTED,
                     ("running in guest-mode without neither IPC nor mapped register!"));

    if (!initialConfig || !numOfTasks)
    {
        /* !initialConfig - runtime change of existing value.
         * !numOfTasks - first configuration according to values in regs.
         * In both cases: read the current number of tasks */
        tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1]);
        currentVal = (uint8_t)(((tmpReg & BMI_NUM_OF_TASKS_MASK) >> BMI_NUM_OF_TASKS_SHIFT) + 1);
        currentExtraVal = (uint8_t)((tmpReg & BMI_NUM_OF_EXTRA_TASKS_MASK) >> BMI_EXTRA_NUM_OF_TASKS_SHIFT);
    }
    else /* first time and numOfTasks explicitly specified */
        currentVal = currentExtraVal = 0;

    if (!numOfTasks)
    {
        /* This is the first configuration and user did not specify value (!numOfTasks),
         * reset values will be used and we just save these values for resource management */
         p_Fm->p_FmStateStruct->extraTasksPoolSize =
                    (uint8_t)MAX(p_Fm->p_FmStateStruct->extraTasksPoolSize, currentExtraVal);
         if ((p_Fm->p_FmStateStruct->accumulatedNumOfTasks + currentVal) >
                 (p_Fm->p_FmStateStruct->totalNumOfTasks - p_Fm->p_FmStateStruct->extraTasksPoolSize))
             RETURN_ERROR(MAJOR, E_NOT_AVAILABLE,
                                      ("Total ports' numOfTasks and extra tasks pool for fm%d exceed total available numOfTasks.",
                                       p_Fm->p_FmStateStruct->fmId));
        p_Fm->p_FmStateStruct->accumulatedNumOfTasks += currentVal;
        *p_NumOfTasks = currentVal;
        *p_NumOfExtraTasks = currentExtraVal;
    }
    else
    {
        /* user requires a specific value.
         * If this is the first configuration call, (numOfTasks != 0) currentVal & currentExtraVal are set to "0",
         * otherwise they hold the value written in the register.
         */
        if (numOfExtraTasks > currentExtraVal)
             p_Fm->p_FmStateStruct->extraTasksPoolSize =
                 (uint8_t)MAX(p_Fm->p_FmStateStruct->extraTasksPoolSize, numOfExtraTasks);

        /* check that there are enough uncommitted tasks */
        if ((p_Fm->p_FmStateStruct->accumulatedNumOfTasks - currentVal + numOfTasks) >
           (p_Fm->p_FmStateStruct->totalNumOfTasks - p_Fm->p_FmStateStruct->extraTasksPoolSize))
            RETURN_ERROR(MAJOR, E_NOT_AVAILABLE,
                         ("Requested numOfTasks and extra tasks pool for fm%d exceed total numOfTasks.",
                          p_Fm->p_FmStateStruct->fmId));
        else
        {
            ASSERT_COND(p_Fm->p_FmStateStruct->accumulatedNumOfTasks >= currentVal);
            /* update acummulated */
            p_Fm->p_FmStateStruct->accumulatedNumOfTasks -= currentVal;
            p_Fm->p_FmStateStruct->accumulatedNumOfTasks += numOfTasks;
            /* calculate reg */
            tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1]) & ~(BMI_NUM_OF_TASKS_MASK | BMI_NUM_OF_EXTRA_TASKS_MASK);
            tmpReg |= (uint32_t)(((numOfTasks-1) << BMI_NUM_OF_TASKS_SHIFT) |
                        (numOfExtraTasks << BMI_EXTRA_NUM_OF_TASKS_SHIFT));
            WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1],tmpReg);
        }
        *p_NumOfTasks = numOfTasks;
        *p_NumOfExtraTasks = numOfExtraTasks;
    }

    return E_OK;
}
#endif /* FM_NO_GUARANTEED_RESET_VALUES */

#ifdef FM_NO_GUARANTEED_RESET_VALUES
t_Error FmSetNumOfOpenDmas(t_Handle h_Fm,
                            uint8_t hardwarePortId,
                            uint8_t *p_NumOfOpenDmas,
                            uint8_t *p_NumOfExtraOpenDmas,
                            bool    initialConfig)

{
    t_Fm                    *p_Fm = (t_Fm *)h_Fm;
    uint8_t                 oldVal = 0, numOfOpenDmas = *p_NumOfOpenDmas, numOfExtraOpenDmas = *p_NumOfExtraOpenDmas;
    uint32_t                tmpReg = 0;
    t_Error                 err;

    ASSERT_COND(IN_RANGE(1, hardwarePortId, 63));

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        !p_Fm->baseAddr &&
        p_Fm->h_IpcSessions[0])
    {
        t_FmIpcPortRsrcParams   rsrcParams;
        t_FmIpcMsg              msg;
        t_FmIpcReply            reply;
        uint32_t                replyLength;

        rsrcParams.hardwarePortId = hardwarePortId;
        rsrcParams.val = numOfOpenDmas;
        rsrcParams.extra = numOfExtraOpenDmas;
        rsrcParams.boolInitialConfig = (uint8_t)initialConfig;

        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        msg.msgId = FM_SET_NUM_OF_OPEN_DMAS;
        memcpy(msg.msgBody, &rsrcParams, sizeof(rsrcParams));
        replyLength = sizeof(uint32_t);
        if ((err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                     (uint8_t*)&msg,
                                     sizeof(msg.msgId) + sizeof(rsrcParams),
                                     (uint8_t*)&reply,
                                     &replyLength,
                                     NULL,
                                     NULL)) != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
        if (replyLength != sizeof(uint32_t))
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("IPC reply length mismatch"));
        return (t_Error)(reply.error);
    }
#ifdef FM_HAS_TOTAL_DMAS
    else if (p_Fm->guestId != NCSW_MASTER_ID)
        RETURN_ERROR(MAJOR, E_NOT_SUPPORTED,
                     ("running in guest-mode without IPC!"));
#else
    else if ((p_Fm->guestId != NCSW_MASTER_ID) &&
             p_Fm->baseAddr &&
             (p_Fm->p_FmStateStruct->revInfo.majorRev >= 6))
    {
        /*DBG(WARNING, ("No IPC - can't validate FM total-num-of-dmas."));*/

        /* calculate reg */
        tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1]) & ~(BMI_NUM_OF_DMAS_MASK | BMI_NUM_OF_EXTRA_DMAS_MASK);
        tmpReg |= (uint32_t)(((numOfOpenDmas-1) << BMI_NUM_OF_DMAS_SHIFT) |
                    (numOfExtraOpenDmas << BMI_EXTRA_NUM_OF_DMAS_SHIFT));
        WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1], tmpReg);
    }
    else if (p_Fm->guestId != NCSW_MASTER_ID)
        RETURN_ERROR(MAJOR, E_NOT_SUPPORTED,
                     ("running in guest-mode without neither IPC nor mapped register!"));
#endif /* FM_HAS_TOTAL_DMAS */

    if (!initialConfig)
    {
        tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1]);
        /* read into oldVal the current extra tasks */
        oldVal = (uint8_t)((tmpReg & BMI_NUM_OF_EXTRA_DMAS_MASK) >> BMI_EXTRA_NUM_OF_DMAS_SHIFT);
    }

    if (numOfExtraOpenDmas > oldVal)
        p_Fm->p_FmStateStruct->extraOpenDmasPoolSize =
            (uint8_t)MAX(p_Fm->p_FmStateStruct->extraOpenDmasPoolSize, numOfExtraOpenDmas);

    if (!initialConfig)
        /* read into oldVal the current num of tasks */
        oldVal = (uint8_t)(((tmpReg & BMI_NUM_OF_DMAS_MASK) >> BMI_NUM_OF_DMAS_SHIFT) + 1);

    /* check that there are enough uncommitted open DMA's */
    ASSERT_COND(p_Fm->p_FmStateStruct->accumulatedNumOfOpenDmas >= oldVal);
#ifdef FM_HAS_TOTAL_DMAS
    if ((p_Fm->p_FmStateStruct->revInfo.majorRev < 6) &&
        (p_Fm->p_FmStateStruct->accumulatedNumOfOpenDmas - oldVal + numOfOpenDmas >
            p_Fm->p_FmStateStruct->maxNumOfOpenDmas))
            RETURN_ERROR(MAJOR, E_NOT_AVAILABLE,
                         ("Requested numOfOpenDmas for fm%d exceeds total numOfOpenDmas.",
                         p_Fm->p_FmStateStruct->fmId));
#else
    if ((p_Fm->p_FmStateStruct->revInfo.majorRev >= 6) &&
        (p_Fm->p_FmStateStruct->accumulatedNumOfOpenDmas - oldVal + numOfOpenDmas > DMA_THRESH_MAX_COMMQ + 1))
        RETURN_ERROR(MAJOR, E_NOT_AVAILABLE,
                     ("Requested numOfOpenDmas for fm%d exceeds DMA Command queue (%d)",
                      p_Fm->p_FmStateStruct->fmId, DMA_THRESH_MAX_COMMQ+1));
#endif /* FM_HAS_TOTAL_DMAS */
    else
    {
        /* update acummulated */
        p_Fm->p_FmStateStruct->accumulatedNumOfOpenDmas -= oldVal;
        p_Fm->p_FmStateStruct->accumulatedNumOfOpenDmas += numOfOpenDmas;

        /* calculate reg */
        tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1]) & ~(BMI_NUM_OF_DMAS_MASK | BMI_NUM_OF_EXTRA_DMAS_MASK);
        tmpReg |= (uint32_t)(((numOfOpenDmas-1) << BMI_NUM_OF_DMAS_SHIFT) |
                    (numOfExtraOpenDmas << BMI_EXTRA_NUM_OF_DMAS_SHIFT));
        WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1], tmpReg);

#ifdef FM_HAS_TOTAL_DMAS
        if (p_Fm->p_FmStateStruct->revInfo.majorRev < 6)
        {
            /* update total num of DMA's with committed number of open DMAS, and max uncommitted pool. */
            tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_cfg2) & ~BMI_CFG2_DMAS_MASK;
            tmpReg |= (uint32_t)(p_Fm->p_FmStateStruct->accumulatedNumOfOpenDmas + p_Fm->p_FmStateStruct->extraOpenDmasPoolSize - 1) << BMI_CFG2_DMAS_SHIFT;
            WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_cfg2, tmpReg);
        }
#endif /* FM_HAS_TOTAL_DMAS */
    }


    return E_OK;
}

#else /* FM_NO_GUARANTEED_RESET_VALUES */
t_Error FmSetNumOfOpenDmas(t_Handle h_Fm,
                            uint8_t hardwarePortId,
                            uint8_t *p_NumOfOpenDmas,
                            uint8_t *p_NumOfExtraOpenDmas,
                            bool    initialConfig)

{
    t_Fm                    *p_Fm = (t_Fm *)h_Fm;
    uint32_t                tmpReg = 0;
    t_Error                 err;
    uint8_t                 currentVal, currentExtraVal, numOfOpenDmas = *p_NumOfOpenDmas, numOfExtraOpenDmas = *p_NumOfExtraOpenDmas;

    ASSERT_COND(IN_RANGE(1, hardwarePortId, 63));

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        !p_Fm->baseAddr &&
        p_Fm->h_IpcSessions[0])
    {
        t_FmIpcPortRsrcParams   rsrcParams;
        t_FmIpcMsg              msg;
        t_FmIpcReply            reply;
        uint32_t                replyLength;

        rsrcParams.hardwarePortId = hardwarePortId;
        rsrcParams.val = numOfOpenDmas;
        rsrcParams.extra = numOfExtraOpenDmas;
        rsrcParams.boolInitialConfig = (uint8_t)initialConfig;

        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        msg.msgId = FM_SET_NUM_OF_OPEN_DMAS;
        memcpy(msg.msgBody, &rsrcParams, sizeof(rsrcParams));
        replyLength = sizeof(uint32_t);
        if ((err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                     (uint8_t*)&msg,
                                     sizeof(msg.msgId) + sizeof(rsrcParams),
                                     (uint8_t*)&reply,
                                     &replyLength,
                                     NULL,
                                     NULL)) != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
        if (replyLength != sizeof(uint32_t))
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("IPC reply length mismatch"));
        return (t_Error)(reply.error);
    }
#ifdef FM_HAS_TOTAL_DMAS
    else if (p_Fm->guestId != NCSW_MASTER_ID)
        RETURN_ERROR(MAJOR, E_NOT_SUPPORTED,
                     ("running in guest-mode without IPC!"));
#else
    else if ((p_Fm->guestId != NCSW_MASTER_ID) &&
             p_Fm->baseAddr &&
             (p_Fm->p_FmStateStruct->revInfo.majorRev >= 6))
    {
        /*DBG(WARNING, ("No IPC - can't validate FM total-num-of-dmas."));*/

        if (numOfOpenDmas)
        {
            /* whether it is the first time with explicit value, or runtime "set" - write register */
            tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1]) & ~(BMI_NUM_OF_DMAS_MASK | BMI_NUM_OF_EXTRA_DMAS_MASK);
            tmpReg |= (uint32_t)(((numOfOpenDmas-1) << BMI_NUM_OF_DMAS_SHIFT) |
                        (numOfExtraOpenDmas << BMI_EXTRA_NUM_OF_DMAS_SHIFT));
            WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1], tmpReg);
        }
        else /* first config without explicit value: Do Nothing - reset value shouldn't be
                changed, read register for port save */
        {
            tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1]);
            /* read into oldVal the current extra tasks */
            *p_NumOfOpenDmas = (uint8_t)((tmpReg & BMI_NUM_OF_EXTRA_DMAS_MASK) >> BMI_EXTRA_NUM_OF_DMAS_SHIFT);
            *p_NumOfExtraOpenDmas = (uint8_t)(((tmpReg & BMI_NUM_OF_DMAS_MASK) >> BMI_NUM_OF_DMAS_SHIFT) + 1);
        }
    }
    else if (p_Fm->guestId != NCSW_MASTER_ID)
        RETURN_ERROR(MAJOR, E_NOT_SUPPORTED,
                     ("running in guest-mode without neither IPC nor mapped register!"));
#endif /* FM_HAS_TOTAL_DMAS */

    /* it's illegal to be in a state where this is not the first set and no value is specified */
    ASSERT_COND(initialConfig || numOfOpenDmas);

    if (!initialConfig || !numOfOpenDmas)
    {
        /* !initialConfig - runtime change of existing value.
         * !numOfTasks - first configuration according to values in regs.
         * In both cases: read the current number of open Dma's */
        tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1]);
        /* read into oldVal the current extra tasks */
        currentExtraVal = (uint8_t)((tmpReg & BMI_NUM_OF_EXTRA_DMAS_MASK) >> BMI_EXTRA_NUM_OF_DMAS_SHIFT);
        currentVal = (uint8_t)(((tmpReg & BMI_NUM_OF_DMAS_MASK) >> BMI_NUM_OF_DMAS_SHIFT) + 1);
    }
    else /* first time and numOfTasks explicitly specified */
        currentVal = currentExtraVal = 0;

    if (!numOfOpenDmas)
    {
        /* This is the first configuration and user did not specify value (!numOfOpenDmas),
         * reset values will be used and we just save these values for resource management */
        p_Fm->p_FmStateStruct->extraOpenDmasPoolSize =
                    (uint8_t)MAX(p_Fm->p_FmStateStruct->extraOpenDmasPoolSize, currentExtraVal);
        p_Fm->p_FmStateStruct->accumulatedNumOfOpenDmas += currentVal;
        *p_NumOfOpenDmas = currentVal;
        *p_NumOfExtraOpenDmas = currentExtraVal;
    }
    else
    {
        /* user requires a specific value.
         * If this is the first configuration call, (numOfTasks != 0) currentVal & currentExtraVal are set to "0",
         * otherwise they hold the value written in the register.
         */
        if (numOfExtraOpenDmas > currentExtraVal)
             p_Fm->p_FmStateStruct->extraOpenDmasPoolSize =
                 (uint8_t)MAX(p_Fm->p_FmStateStruct->extraOpenDmasPoolSize, numOfExtraOpenDmas);


        /* read into oldVal the current num of tasks */
#ifdef FM_HAS_TOTAL_DMAS
        if ((p_Fm->p_FmStateStruct->revInfo.majorRev < 6) &&
            (p_Fm->p_FmStateStruct->accumulatedNumOfOpenDmas - currentVal + numOfOpenDmas >
                p_Fm->p_FmStateStruct->maxNumOfOpenDmas))
                RETURN_ERROR(MAJOR, E_NOT_AVAILABLE,
                             ("Requested numOfOpenDmas for fm%d exceeds total numOfOpenDmas.",
                             p_Fm->p_FmStateStruct->fmId));
#else
        if ((p_Fm->p_FmStateStruct->revInfo.majorRev >= 6) &&
            (p_Fm->p_FmStateStruct->accumulatedNumOfOpenDmas - currentVal + numOfOpenDmas > DMA_THRESH_MAX_COMMQ + 1))
            RETURN_ERROR(MAJOR, E_NOT_AVAILABLE,
                         ("Requested numOfOpenDmas for fm%d exceeds DMA Command queue (%d)",
                          p_Fm->p_FmStateStruct->fmId, DMA_THRESH_MAX_COMMQ+1));
#endif /* FM_HAS_TOTAL_DMAS */
        else
        {
            ASSERT_COND(p_Fm->p_FmStateStruct->accumulatedNumOfOpenDmas >= currentVal);
            /* update acummulated */
            p_Fm->p_FmStateStruct->accumulatedNumOfOpenDmas -= currentVal;
            p_Fm->p_FmStateStruct->accumulatedNumOfOpenDmas += numOfOpenDmas;

            /* calculate reg */
            tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1]) & ~(BMI_NUM_OF_DMAS_MASK | BMI_NUM_OF_EXTRA_DMAS_MASK);
            tmpReg |= (uint32_t)(((numOfOpenDmas-1) << BMI_NUM_OF_DMAS_SHIFT) |
                        (numOfExtraOpenDmas << BMI_EXTRA_NUM_OF_DMAS_SHIFT));
            WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1], tmpReg);
#ifdef FM_HAS_TOTAL_DMAS
            if (p_Fm->p_FmStateStruct->revInfo.majorRev < 6)
            {
                /* update total num of DMA's with committed number of open DMAS, and max uncommitted pool. */
                tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_cfg2) & ~BMI_CFG2_DMAS_MASK;
                tmpReg |= (uint32_t)(p_Fm->p_FmStateStruct->accumulatedNumOfOpenDmas + p_Fm->p_FmStateStruct->extraOpenDmasPoolSize - 1) << BMI_CFG2_DMAS_SHIFT;
                WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_cfg2, tmpReg);
            }
#endif /* FM_HAS_TOTAL_DMAS */

        }
        *p_NumOfOpenDmas = numOfOpenDmas;
        *p_NumOfExtraOpenDmas = numOfExtraOpenDmas;
    }


    return E_OK;
}
#endif /* FM_NO_GUARANTEED_RESET_VALUES */

#if (DPAA_VERSION >= 11)
t_Error FmVSPCheckRelativeProfile(t_Handle        h_Fm,
                                  e_FmPortType    portType,
                                  uint8_t         portId,
                                  uint16_t        relativeProfile)
{
    t_Fm         *p_Fm;
    t_FmSp      *p_FmPcdSp;
    uint8_t     swPortIndex=0, hardwarePortId;

    ASSERT_COND(h_Fm);
    p_Fm = (t_Fm*)h_Fm;

    SW_PORT_ID_TO_HW_PORT_ID(hardwarePortId, portType, portId)
    ASSERT_COND(hardwarePortId);
    HW_PORT_ID_TO_SW_PORT_INDX(swPortIndex, hardwarePortId);

    p_FmPcdSp = p_Fm->p_FmSp;
    ASSERT_COND(p_FmPcdSp);

    if (!p_FmPcdSp->portsMapping[swPortIndex].numOfProfiles)
        RETURN_ERROR(MAJOR, E_INVALID_STATE , ("Port has no allocated profiles"));
    if (relativeProfile >= p_FmPcdSp->portsMapping[swPortIndex].numOfProfiles)
        RETURN_ERROR(MAJOR, E_NOT_IN_RANGE , ("Profile id is out of range"));

    return E_OK;
}

t_Error FmVSPGetAbsoluteProfileId(t_Handle        h_Fm,
                                  e_FmPortType    portType,
                                  uint8_t         portId,
                                  uint16_t        relativeProfile,
                                  uint16_t        *p_AbsoluteId)
{
    t_Fm         *p_Fm;
    t_FmSp      *p_FmPcdSp;
    uint8_t     swPortIndex=0, hardwarePortId;
    t_Error     err;

    ASSERT_COND(h_Fm);
    p_Fm = (t_Fm*)h_Fm;

    err = FmVSPCheckRelativeProfile(h_Fm, portType, portId, relativeProfile);
    if (err != E_OK)
        return err;

    SW_PORT_ID_TO_HW_PORT_ID(hardwarePortId, portType, portId)
    ASSERT_COND(hardwarePortId);
    HW_PORT_ID_TO_SW_PORT_INDX(swPortIndex, hardwarePortId);

    p_FmPcdSp = p_Fm->p_FmSp;
    ASSERT_COND(p_FmPcdSp);

    *p_AbsoluteId = (uint16_t)(p_FmPcdSp->portsMapping[swPortIndex].profilesBase + relativeProfile);

    return E_OK;
}
#endif /* (DPAA_VERSION >= 11) */

static t_Error InitFmDma(t_Fm *p_Fm)
{
    t_FmDriverParam         *p_FmDriverParam = NULL;
    uint32_t                tmpReg;

    ASSERT_COND(p_Fm);
    ASSERT_COND(p_Fm->p_FmDriverParam);

    p_FmDriverParam = p_Fm->p_FmDriverParam;

    /* clear status reg events */
#if (DPAA_VERSION >= 11)
        tmpReg = DMA_STATUS_FM_SPDAT_ECC;
#else
        tmpReg = DMA_STATUS_FM_ECC;
#endif /* DPAA_VERSION >= 11 */
    WRITE_UINT32(p_Fm->p_FmDmaRegs->fmdmsr, GET_UINT32(p_Fm->p_FmDmaRegs->fmdmsr) | tmpReg);

    /* configure mode register */
    tmpReg = 0;
    tmpReg |= p_FmDriverParam->dmaCacheOverride << DMA_MODE_CACHE_OR_SHIFT;
    if (p_FmDriverParam->dmaAidOverride)
        tmpReg |= DMA_MODE_AID_OR;
    if (p_Fm->p_FmStateStruct->exceptions & FM_EX_DMA_BUS_ERROR)
        tmpReg |= DMA_MODE_BER;
    if (p_FmDriverParam->dmaEnEmergency)
    {
        tmpReg |= p_FmDriverParam->dmaEmergency.emergencyBusSelect;
        tmpReg |= p_FmDriverParam->dmaEmergency.emergencyLevel << DMA_MODE_EMERGENCY_LEVEL_SHIFT;
        if (p_FmDriverParam->dmaEnEmergencySmoother)
            WRITE_UINT32(p_Fm->p_FmDmaRegs->fmdmemsr, p_FmDriverParam->dmaEmergencySwitchCounter);
    }
    tmpReg |= ((p_FmDriverParam->dmaCamNumOfEntries/DMA_CAM_UNITS) - 1) << DMA_MODE_CEN_SHIFT;
    tmpReg |= p_FmDriverParam->dmaDbgCntMode << DMA_MODE_DBG_SHIFT;
    tmpReg |= DMA_MODE_SECURE_PROT;
    tmpReg |= p_FmDriverParam->dmaAidMode << DMA_MODE_AID_MODE_SHIFT;

#if (DPAA_VERSION >= 11)
    if (p_Fm->p_FmStateStruct->exceptions & FM_EX_DMA_SINGLE_PORT_ECC)
        tmpReg |= DMA_MODE_ECC;
#else
    if ((p_Fm->p_FmStateStruct->exceptions & FM_EX_DMA_SYSTEM_WRITE_ECC) | (p_Fm->p_FmStateStruct->exceptions & FM_EX_DMA_READ_ECC) | (p_Fm->p_FmStateStruct->exceptions & FM_EX_DMA_FM_WRITE_ECC))
        tmpReg |= DMA_MODE_ECC;
    if (p_FmDriverParam->dmaStopOnBusError)
        tmpReg |= DMA_MODE_SBER;
    tmpReg |= (uint32_t)(p_FmDriverParam->dmaAxiDbgNumOfBeats - 1) << DMA_MODE_AXI_DBG_SHIFT;
#ifdef FM_PEDANTIC_DMA
    tmpReg |= DMA_MODE_EMERGENCY_READ;
#endif /* FM_PEDANTIC_DMA */
#endif /* DPAA_VERSION >= 11 */

    WRITE_UINT32(p_Fm->p_FmDmaRegs->fmdmmr, tmpReg);

    /* configure thresholds register */
    tmpReg = GET_UINT32(p_Fm->p_FmDmaRegs->fmdmtr);
    tmpReg &= ~DMA_THRESH_COMMQ_MASK;
    tmpReg |= ((uint32_t)p_FmDriverParam->dmaCommQThresholds.assertEmergency << DMA_THRESH_COMMQ_SHIFT);
#if (DPAA_VERSION < 11)
    tmpReg &= ~(DMA_THRESH_READ_INT_BUF_MASK | DMA_THRESH_WRITE_INT_BUF_MASK);
    tmpReg |= ((uint32_t)p_FmDriverParam->dmaReadBufThresholds.assertEmergency << DMA_THRESH_READ_INT_BUF_SHIFT) |
               ((uint32_t)p_FmDriverParam->dmaWriteBufThresholds.assertEmergency);
#endif /* (DPAA_VERSION < 11) */
    WRITE_UINT32(p_Fm->p_FmDmaRegs->fmdmtr, tmpReg);

    /* configure hysteresis register */
    tmpReg = ((uint32_t)p_FmDriverParam->dmaCommQThresholds.clearEmergency << DMA_THRESH_COMMQ_SHIFT);
#if (DPAA_VERSION < 11)
    tmpReg |= ((uint32_t)p_FmDriverParam->dmaReadBufThresholds.clearEmergency << DMA_THRESH_READ_INT_BUF_SHIFT) |
              ((uint32_t)p_FmDriverParam->dmaWriteBufThresholds.clearEmergency);
#endif /* (DPAA_VERSION < 11) */
    WRITE_UINT32(p_Fm->p_FmDmaRegs->fmdmhy, tmpReg);

    /* configure emergency threshold */
    WRITE_UINT32(p_Fm->p_FmDmaRegs->fmdmsetr, p_FmDriverParam->dmaSosEmergency);

    /* configure Watchdog */
    WRITE_UINT32(p_Fm->p_FmDmaRegs->fmdmwcr, USEC_TO_CLK(p_FmDriverParam->dmaWatchdog, p_Fm->p_FmStateStruct->fmClkFreq));

    /* Allocate MURAM for CAM */
    p_Fm->camBaseAddr = PTR_TO_UINT(FM_MURAM_AllocMem(p_Fm->h_FmMuram,
                                                      (uint32_t)(p_FmDriverParam->dmaCamNumOfEntries*DMA_CAM_SIZEOF_ENTRY),
                                                      DMA_CAM_ALIGN));
    if (!p_Fm->camBaseAddr)
        RETURN_ERROR(MAJOR, E_NO_MEMORY, ("MURAM alloc for DMA CAM failed"));

    WRITE_BLOCK(UINT_TO_PTR(p_Fm->camBaseAddr),
                0,
                (uint32_t)(p_FmDriverParam->dmaCamNumOfEntries*DMA_CAM_SIZEOF_ENTRY));

    if (p_Fm->p_FmStateStruct->revInfo.majorRev == 2)
    {
        FM_MURAM_FreeMem(p_Fm->h_FmMuram, UINT_TO_PTR(p_Fm->camBaseAddr));

        p_Fm->camBaseAddr = PTR_TO_UINT(FM_MURAM_AllocMem(p_Fm->h_FmMuram,
                                                          (uint32_t)(p_FmDriverParam->dmaCamNumOfEntries*72 + 128),
                                                          64));
        if (!p_Fm->camBaseAddr)
            RETURN_ERROR(MAJOR, E_NO_MEMORY, ("MURAM alloc for DMA CAM failed"));

        WRITE_BLOCK(UINT_TO_PTR(p_Fm->camBaseAddr),
                   0,
                   (uint32_t)(p_FmDriverParam->dmaCamNumOfEntries*72 + 128));

        switch (p_FmDriverParam->dmaCamNumOfEntries)
        {
            case (8):
                WRITE_UINT32(*(uint32_t*)p_Fm->camBaseAddr, 0xff000000);
                break;
            case (16):
                WRITE_UINT32(*(uint32_t*)p_Fm->camBaseAddr, 0xffff0000);
                break;
            case (24):
                WRITE_UINT32(*(uint32_t*)p_Fm->camBaseAddr, 0xffffff00);
                break;
            case (32):
                WRITE_UINT32(*(uint32_t*)p_Fm->camBaseAddr, 0xffffffff);
                break;
            default:
                RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("wrong dmaCamNumOfEntries"));
        }
    }

    /* VirtToPhys */
    WRITE_UINT32(p_Fm->p_FmDmaRegs->fmdmebcr,
                 (uint32_t)(XX_VirtToPhys(UINT_TO_PTR(p_Fm->camBaseAddr)) - p_Fm->fmMuramPhysBaseAddr));

    return E_OK;
}

static t_Error InitFmFpm(t_Fm *p_Fm)
{
    t_FmDriverParam         *p_FmDriverParam = NULL;
    uint32_t                tmpReg;
    int                     i;

    ASSERT_COND(p_Fm);
    ASSERT_COND(p_Fm->p_FmDriverParam);

    p_FmDriverParam = p_Fm->p_FmDriverParam;

    tmpReg = (uint32_t)(p_FmDriverParam->thresholds.dispLimit << FPM_DISP_LIMIT_SHIFT);
    WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_mxd, tmpReg);

    tmpReg =   (((uint32_t)p_FmDriverParam->thresholds.prsDispTh  << FPM_THR1_PRS_SHIFT) |
                ((uint32_t)p_FmDriverParam->thresholds.kgDispTh  << FPM_THR1_KG_SHIFT) |
                ((uint32_t)p_FmDriverParam->thresholds.plcrDispTh  << FPM_THR1_PLCR_SHIFT) |
                ((uint32_t)p_FmDriverParam->thresholds.bmiDispTh  << FPM_THR1_BMI_SHIFT));
    WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_dis1, tmpReg);

    tmpReg =   (((uint32_t)p_FmDriverParam->thresholds.qmiEnqDispTh  << FPM_THR2_QMI_ENQ_SHIFT) |
                ((uint32_t)p_FmDriverParam->thresholds.qmiDeqDispTh  << FPM_THR2_QMI_DEQ_SHIFT) |
                ((uint32_t)p_FmDriverParam->thresholds.fmCtl1DispTh  << FPM_THR2_FM_CTL1_SHIFT) |
                ((uint32_t)p_FmDriverParam->thresholds.fmCtl2DispTh  << FPM_THR2_FM_CTL2_SHIFT));
    WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_dis2, tmpReg);

    /* define exceptions and error behavior */
    tmpReg = 0;
    /* Clear events */
    tmpReg |= (FPM_EV_MASK_STALL | FPM_EV_MASK_DOUBLE_ECC | FPM_EV_MASK_SINGLE_ECC);
    /* enable interrupts */
    if (p_Fm->p_FmStateStruct->exceptions & FM_EX_FPM_STALL_ON_TASKS)
        tmpReg |= FPM_EV_MASK_STALL_EN;
    if (p_Fm->p_FmStateStruct->exceptions & FM_EX_FPM_SINGLE_ECC)
        tmpReg |= FPM_EV_MASK_SINGLE_ECC_EN;
    if (p_Fm->p_FmStateStruct->exceptions & FM_EX_FPM_DOUBLE_ECC)
        tmpReg |= FPM_EV_MASK_DOUBLE_ECC_EN;
    tmpReg |= (p_Fm->p_FmDriverParam->catastrophicErr  << FPM_EV_MASK_CAT_ERR_SHIFT);
    tmpReg |= (p_Fm->p_FmDriverParam->dmaErr << FPM_EV_MASK_DMA_ERR_SHIFT);
    if (!p_Fm->p_FmDriverParam->haltOnExternalActivation)
        tmpReg |= FPM_EV_MASK_EXTERNAL_HALT;
    if (!p_Fm->p_FmDriverParam->haltOnUnrecoverableEccError)
        tmpReg |= FPM_EV_MASK_ECC_ERR_HALT;
    WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_ee, tmpReg);

    /* clear all fmCtls event registers */
    for (i=0;i<FM_NUM_OF_FMAN_CTRL_EVENT_REGS;i++)
        WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_cev[i], 0xFFFFFFFF);

    /* RAM ECC -  enable and clear events */
    /* first we need to clear all parser memory, as it is uninitialized and
       may cause ECC errors
     */
    tmpReg = 0;
    /* event bits */
    tmpReg = (FPM_RAM_CTL_MURAM_ECC | FPM_RAM_CTL_IRAM_ECC);
    /* Rams enable is not effected by the RCR bit, but by a COP configuration */
    if (p_Fm->p_FmDriverParam->externalEccRamsEnable)
        tmpReg |= FPM_RAM_CTL_RAMS_ECC_EN_SRC_SEL;

    /* enable test mode */
    if (p_FmDriverParam->enMuramTestMode)
        tmpReg |= FPM_RAM_CTL_MURAM_TEST_ECC;
    if (p_FmDriverParam->enIramTestMode)
        tmpReg |= FPM_RAM_CTL_IRAM_TEST_ECC;
    WRITE_UINT32(p_Fm->p_FmFpmRegs->fm_rcr, tmpReg);

    tmpReg = 0;
    if (p_Fm->p_FmStateStruct->exceptions & FM_EX_IRAM_ECC)
    {
        tmpReg |= FPM_IRAM_ECC_ERR_EX_EN;
        FmEnableRamsEcc(p_Fm);
    }
    if (p_Fm->p_FmStateStruct->exceptions & FM_EX_MURAM_ECC)
    {
        tmpReg |= FPM_MURAM_ECC_ERR_EX_EN;
        FmEnableRamsEcc(p_Fm);
    }
    WRITE_UINT32(p_Fm->p_FmFpmRegs->fm_rie, tmpReg);

    return E_OK;
}

static t_Error InitFmBmi(t_Fm *p_Fm)
{
    uint32_t                tmpReg;

    ASSERT_COND(p_Fm);
    ASSERT_COND(p_Fm->p_FmDriverParam);

    tmpReg = (uint32_t)(XX_VirtToPhys(UINT_TO_PTR(p_Fm->fifoBaseAddr)) - p_Fm->fmMuramPhysBaseAddr);
    tmpReg = tmpReg / BMI_FIFO_ALIGN;

    tmpReg |= ((p_Fm->p_FmStateStruct->totalFifoSize/BMI_FIFO_UNITS - 1) << BMI_CFG1_FIFO_SIZE_SHIFT);
    WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_cfg1, tmpReg);

    tmpReg =  ((uint32_t)(p_Fm->p_FmStateStruct->totalNumOfTasks - 1) << BMI_CFG2_TASKS_SHIFT);
#ifdef FM_HAS_TOTAL_DMAS
    if (p_Fm->p_FmStateStruct->revInfo.majorRev < 6)
        tmpReg |= (uint32_t)(p_Fm->p_FmStateStruct->maxNumOfOpenDmas - 1) << BMI_CFG2_DMAS_SHIFT;
#endif /* FM_HAS_TOTAL_DMAS */
    WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_cfg2, tmpReg);

    /* define unmaskable exceptions, enable and clear events */
    tmpReg = 0;
    WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_ievr, (BMI_ERR_INTR_EN_LIST_RAM_ECC |
                                                BMI_ERR_INTR_EN_STORAGE_PROFILE_ECC |
                                                BMI_ERR_INTR_EN_STATISTICS_RAM_ECC |
                                                BMI_ERR_INTR_EN_DISPATCH_RAM_ECC));
    if (p_Fm->p_FmStateStruct->exceptions & FM_EX_BMI_LIST_RAM_ECC)
        tmpReg |= BMI_ERR_INTR_EN_LIST_RAM_ECC;
    if (p_Fm->p_FmStateStruct->exceptions & FM_EX_BMI_STORAGE_PROFILE_ECC)
        tmpReg |= BMI_ERR_INTR_EN_STORAGE_PROFILE_ECC;
    if (p_Fm->p_FmStateStruct->exceptions & FM_EX_BMI_STATISTICS_RAM_ECC)
        tmpReg |= BMI_ERR_INTR_EN_STATISTICS_RAM_ECC;
    if (p_Fm->p_FmStateStruct->exceptions & FM_EX_BMI_DISPATCH_RAM_ECC)
        tmpReg |= BMI_ERR_INTR_EN_DISPATCH_RAM_ECC;
    WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_ier, tmpReg);

    return E_OK;
}

static t_Error InitFmQmi(t_Fm *p_Fm)
{
    uint32_t                tmpReg;

    ASSERT_COND(p_Fm);
    ASSERT_COND(p_Fm->p_FmDriverParam);

     /* Clear error interrupt events */
    WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_eie, (QMI_ERR_INTR_EN_DOUBLE_ECC | QMI_ERR_INTR_EN_DEQ_FROM_DEF));
    tmpReg = 0;
    if (p_Fm->p_FmStateStruct->exceptions & FM_EX_QMI_DEQ_FROM_UNKNOWN_PORTID)
        tmpReg |= QMI_ERR_INTR_EN_DEQ_FROM_DEF;
    if (p_Fm->p_FmStateStruct->exceptions & FM_EX_QMI_DOUBLE_ECC)
        tmpReg |= QMI_ERR_INTR_EN_DOUBLE_ECC;
    /* enable events */
    WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_eien, tmpReg);

    if (p_Fm->tnumAgingPeriod)
    {
        uint16_t                periodInFmClocks;
        uint8_t                 remainder;

        /* tnumAgingPeriod is in units of microseconds, p_FmClockFreq is in Mhz */
        periodInFmClocks = (uint16_t)(p_Fm->tnumAgingPeriod * p_Fm->p_FmStateStruct->fmClkFreq);
        /* periodInFmClocks must be a 64 multiply */
        remainder = (uint8_t)(periodInFmClocks % 64);
        if (remainder > 64)
            tmpReg = (uint32_t)((periodInFmClocks/64) + 1);
        else
        {
            tmpReg = (uint32_t)(periodInFmClocks/64);
            if (!tmpReg)
                tmpReg = 1;
        }
        tmpReg <<= QMI_TAPC_TAP;
        WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_tapc, tmpReg);

    }

    tmpReg = 0;
    /* Clear interrupt events */
    if ((p_Fm->p_FmStateStruct->revInfo.majorRev != 4) && (p_Fm->p_FmStateStruct->revInfo.majorRev < 6))
    {
        WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_ie, QMI_INTR_EN_SINGLE_ECC);
        if (p_Fm->p_FmStateStruct->exceptions & FM_EX_QMI_SINGLE_ECC)
            tmpReg |= QMI_INTR_EN_SINGLE_ECC;
        /* enable events */
        WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_ien, tmpReg);
    }

    return E_OK;
}

static t_Error InitGuestMode(t_Fm *p_Fm)
{
    t_Error                 err = E_OK;
    int                     i;
    t_FmIpcMsg              msg;
    t_FmIpcReply            reply;
    uint32_t                replyLength;

    ASSERT_COND(p_Fm);
    ASSERT_COND(p_Fm->guestId != NCSW_MASTER_ID);

    /* build the FM guest partition IPC address */
    if (Sprint (p_Fm->fmModuleName, "FM_%d_%d",p_Fm->p_FmStateStruct->fmId, p_Fm->guestId) != (p_Fm->guestId<10 ? 6:7))
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Sprint failed"));

    /* build the FM master partition IPC address */
    memset(p_Fm->fmIpcHandlerModuleName, 0, (sizeof(char)) * MODULE_NAME_SIZE);
    if (Sprint (p_Fm->fmIpcHandlerModuleName[0], "FM_%d_%d",p_Fm->p_FmStateStruct->fmId, NCSW_MASTER_ID) != 6)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Sprint failed"));

    for (i=0;i<e_FM_EV_DUMMY_LAST;i++)
        p_Fm->intrMng[i].f_Isr = UnimplementedIsr;

    p_Fm->h_IpcSessions[0] = XX_IpcInitSession(p_Fm->fmIpcHandlerModuleName[0], p_Fm->fmModuleName);
    if (p_Fm->h_IpcSessions[0])
    {
        uint8_t                 isMasterAlive;
        t_FmIpcParams           ipcParams;

        err = XX_IpcRegisterMsgHandler(p_Fm->fmModuleName, FmGuestHandleIpcMsgCB, p_Fm, FM_IPC_MAX_REPLY_SIZE);
        if (err)
            RETURN_ERROR(MAJOR, err, NO_MSG);

        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        msg.msgId = FM_MASTER_IS_ALIVE;
        msg.msgBody[0] = p_Fm->guestId;
        replyLength = sizeof(uint32_t) + sizeof(uint8_t);
        do
        {
            blockingFlag = TRUE;
            if ((err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                         (uint8_t*)&msg,
                                         sizeof(msg.msgId)+sizeof(p_Fm->guestId),
                                         (uint8_t*)&reply,
                                         &replyLength,
                                         IpcMsgCompletionCB,
                                         p_Fm)) != E_OK)
                REPORT_ERROR(MINOR, err, NO_MSG);
            while (blockingFlag) ;
            if (replyLength != (sizeof(uint32_t) + sizeof(uint8_t)))
                REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("IPC reply length mismatch"));
            isMasterAlive = *(uint8_t*)(reply.replyBody);
        } while (!isMasterAlive);

        /* read FM parameters and save */
        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        msg.msgId = FM_GET_PARAMS;
        replyLength = sizeof(uint32_t) + sizeof(t_FmIpcParams);
        if ((err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                     (uint8_t*)&msg,
                                     sizeof(msg.msgId),
                                     (uint8_t*)&reply,
                                     &replyLength,
                                     NULL,
                                     NULL)) != E_OK)
            RETURN_ERROR(MAJOR, err, NO_MSG);
        if (replyLength != (sizeof(uint32_t) + sizeof(t_FmIpcParams)))
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("IPC reply length mismatch"));
        memcpy((uint8_t*)&ipcParams, reply.replyBody, sizeof(t_FmIpcParams));

        p_Fm->p_FmStateStruct->fmClkFreq = ipcParams.fmClkFreq;
        p_Fm->p_FmStateStruct->revInfo.majorRev = ipcParams.majorRev;
        p_Fm->p_FmStateStruct->revInfo.minorRev = ipcParams.minorRev;
    }
    else
    {
        uint32_t    tmpReg;

        DBG(WARNING, ("FM Guest mode - without IPC"));
        if (!p_Fm->p_FmStateStruct->fmClkFreq)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("No fmClkFreq configured for guest without IPC"));
        if (p_Fm->baseAddr)
        {
            /* read revision register 1 */
            tmpReg = GET_UINT32(p_Fm->p_FmFpmRegs->fm_ip_rev_1);
            p_Fm->p_FmStateStruct->revInfo.majorRev = (uint8_t)((tmpReg & FPM_REV1_MAJOR_MASK) >> FPM_REV1_MAJOR_SHIFT);
            p_Fm->p_FmStateStruct->revInfo.minorRev = (uint8_t)((tmpReg & FPM_REV1_MINOR_MASK) >> FPM_REV1_MINOR_SHIFT);
        }
    }

#if (DPAA_VERSION >= 11)
    p_Fm->partVSPBase = AllocVSPsForPartition(p_Fm, p_Fm->partVSPBase, p_Fm->partNumOfVSPs, p_Fm->guestId);
    if (p_Fm->partVSPBase == ILLEGAL_BASE)
        DBG(WARNING, ("partition VSPs allocation is FAILED"));
#endif /* (DPAA_VERSION >= 11) */

    /* General FM driver initialization */
    if (p_Fm->baseAddr)
        p_Fm->fmMuramPhysBaseAddr =
            (uint64_t)(XX_VirtToPhys(UINT_TO_PTR(p_Fm->baseAddr + FM_MM_MURAM)));

    XX_Free(p_Fm->p_FmDriverParam);
    p_Fm->p_FmDriverParam = NULL;

    if ((p_Fm->guestId == NCSW_MASTER_ID) ||
        (p_Fm->h_IpcSessions[0]))
    {
        FM_DisableRamsEcc(p_Fm);
        FmMuramClear(p_Fm->h_FmMuram);
        FM_EnableRamsEcc(p_Fm);
    }

    return E_OK;
}

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
t_Error FmDumpPortRegs (t_Handle h_Fm, uint8_t hardwarePortId)
{
    t_Fm            *p_Fm = (t_Fm *)h_Fm;

    DECLARE_DUMP;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(((p_Fm->guestId == NCSW_MASTER_ID) ||
                               p_Fm->baseAddr), E_INVALID_OPERATION);

    DUMP_TITLE(&p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1], ("fmbm_pp for port %u", (hardwarePortId)));
    DUMP_MEMORY(&p_Fm->p_FmBmiRegs->fmbm_pp[hardwarePortId-1], sizeof(uint32_t));

    DUMP_TITLE(&p_Fm->p_FmBmiRegs->fmbm_pfs[hardwarePortId-1], ("fmbm_pfs for port %u", (hardwarePortId )));
    DUMP_MEMORY(&p_Fm->p_FmBmiRegs->fmbm_pfs[hardwarePortId-1], sizeof(uint32_t));

    DUMP_TITLE(&p_Fm->p_FmBmiRegs->fmbm_spliodn[hardwarePortId-1], ("fmbm_spliodn for port %u", (hardwarePortId)));
    DUMP_MEMORY(&p_Fm->p_FmBmiRegs->fmbm_spliodn[hardwarePortId-1], sizeof(uint32_t));

    DUMP_TITLE(&p_Fm->p_FmFpmRegs->fmfp_ps[hardwarePortId], ("fmfp_ps for port %u", (hardwarePortId)));
    DUMP_MEMORY(&p_Fm->p_FmFpmRegs->fmfp_ps[hardwarePortId], sizeof(uint32_t));

    DUMP_TITLE(&p_Fm->p_FmDmaRegs->fmdmplr[hardwarePortId/2], ("fmdmplr for port %u", (hardwarePortId)));
    DUMP_MEMORY(&p_Fm->p_FmDmaRegs->fmdmplr[hardwarePortId/2], sizeof(uint32_t));

    return E_OK;
}
#endif /* (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0)) */


/*****************************************************************************/
/*                      API Init unit functions                              */
/*****************************************************************************/
t_Handle FM_Config(t_FmParams *p_FmParam)
{
    t_Fm                *p_Fm;
    uint8_t             i;
    uintptr_t           baseAddr;
    uint32_t            tmpReg;

    SANITY_CHECK_RETURN_VALUE(p_FmParam, E_NULL_POINTER, NULL);
    SANITY_CHECK_RETURN_VALUE(((p_FmParam->firmware.p_Code && p_FmParam->firmware.size) ||
                               (!p_FmParam->firmware.p_Code && !p_FmParam->firmware.size)),
                              E_INVALID_VALUE, NULL);

    baseAddr = p_FmParam->baseAddr;

    /* Allocate FM structure */
    p_Fm = (t_Fm *) XX_Malloc(sizeof(t_Fm));
    if (!p_Fm)
    {
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("FM driver structure"));
        return NULL;
    }
    memset(p_Fm, 0, sizeof(t_Fm));

    p_Fm->p_FmStateStruct = (t_FmStateStruct *) XX_Malloc(sizeof(t_FmStateStruct));
    if (!p_Fm->p_FmStateStruct)
    {
        XX_Free(p_Fm);
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("FM Status structure"));
        return NULL;
    }
    memset(p_Fm->p_FmStateStruct, 0, sizeof(t_FmStateStruct));

    /* Initialize FM parameters which will be kept by the driver */
    p_Fm->p_FmStateStruct->fmId = p_FmParam->fmId;
    p_Fm->guestId               = p_FmParam->guestId;

    for (i=0; i<FM_MAX_NUM_OF_HW_PORT_IDS; i++)
        p_Fm->p_FmStateStruct->portsTypes[i] = e_FM_PORT_TYPE_DUMMY;

    /* Allocate the FM driver's parameters structure */
    p_Fm->p_FmDriverParam = (t_FmDriverParam *)XX_Malloc(sizeof(t_FmDriverParam));
    if (!p_Fm->p_FmDriverParam)
    {
        XX_Free(p_Fm->p_FmStateStruct);
        XX_Free(p_Fm);
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("FM driver parameters"));
        return NULL;
    }
    memset(p_Fm->p_FmDriverParam, 0, sizeof(t_FmDriverParam));

#if (DPAA_VERSION >= 11)
    p_Fm->p_FmSp = (t_FmSp *)XX_Malloc(sizeof(t_FmSp));
    if (!p_Fm->p_FmSp)
    {
        XX_Free(p_Fm->p_FmDriverParam);
        XX_Free(p_Fm->p_FmStateStruct);
        XX_Free(p_Fm);
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("allocation for internal data structure failed"));
        return NULL;
    }
    memset(p_Fm->p_FmSp, 0, sizeof(t_FmSp));

    for (i=0; i<FM_VSP_MAX_NUM_OF_ENTRIES; i++)
        p_Fm->p_FmSp->profiles[i].profilesMng.ownerId = (uint8_t)ILLEGAL_BASE;
#endif /* (DPAA_VERSION >= 11) */

    /* Initialize FM parameters which will be kept by the driver */
    p_Fm->p_FmStateStruct->fmId                 = p_FmParam->fmId;
    p_Fm->h_FmMuram                             = p_FmParam->h_FmMuram;
    p_Fm->h_App                                 = p_FmParam->h_App;
    p_Fm->p_FmStateStruct->fmClkFreq            = p_FmParam->fmClkFreq;
    p_Fm->f_Exception                           = p_FmParam->f_Exception;
    p_Fm->f_BusError                            = p_FmParam->f_BusError;
    p_Fm->p_FmFpmRegs                           = (t_FmFpmRegs *)UINT_TO_PTR(baseAddr + FM_MM_FPM);
    p_Fm->p_FmBmiRegs                           = (t_FmBmiRegs *)UINT_TO_PTR(baseAddr + FM_MM_BMI);
    p_Fm->p_FmQmiRegs                           = (t_FmQmiRegs *)UINT_TO_PTR(baseAddr + FM_MM_QMI);
    p_Fm->p_FmDmaRegs                           = (t_FmDmaRegs *)UINT_TO_PTR(baseAddr + FM_MM_DMA);
    p_Fm->baseAddr                              = baseAddr;
    p_Fm->p_FmStateStruct->irq                  = p_FmParam->irq;
    p_Fm->p_FmStateStruct->errIrq               = p_FmParam->errIrq;
    p_Fm->hcPortInitialized                     = FALSE;
    p_Fm->independentMode                       = FALSE;

    p_Fm->h_Spinlock = XX_InitSpinlock();
    if (!p_Fm->h_Spinlock)
    {
        XX_Free(p_Fm->p_FmDriverParam);
        XX_Free(p_Fm->p_FmStateStruct);
        XX_Free(p_Fm);
        REPORT_ERROR(MAJOR, E_INVALID_STATE, ("cant allocate spinlock!"));
        return NULL;
    }

#if (DPAA_VERSION >= 11)
    p_Fm->partVSPBase   = p_FmParam->partVSPBase;
    p_Fm->partNumOfVSPs = p_FmParam->partNumOfVSPs;
    p_Fm->vspBaseAddr = p_FmParam->vspBaseAddr;
#endif /* (DPAA_VERSION >= 11) */



    p_Fm->p_FmStateStruct->ramsEccEnable        = FALSE;
    p_Fm->p_FmStateStruct->extraFifoPoolSize    = 0;
    p_Fm->p_FmStateStruct->exceptions           = DEFAULT_exceptions;
    /*p_Fm->p_FmDriverParam->numOfPartitions                    = p_FmParam->numOfPartitions;    */
    p_Fm->p_FmDriverParam->resetOnInit                          = DEFAULT_resetOnInit;

    p_Fm->p_FmDriverParam->catastrophicErr                      = DEFAULT_catastrophicErr;
    p_Fm->p_FmDriverParam->dmaErr                               = DEFAULT_dmaErr;
    p_Fm->p_FmDriverParam->haltOnExternalActivation             = DEFAULT_haltOnExternalActivation;
    p_Fm->p_FmDriverParam->haltOnUnrecoverableEccError          = DEFAULT_haltOnUnrecoverableEccError;
    p_Fm->p_FmDriverParam->enIramTestMode                       = FALSE;
    p_Fm->p_FmDriverParam->enMuramTestMode                      = FALSE;
    p_Fm->p_FmDriverParam->externalEccRamsEnable                = DEFAULT_externalEccRamsEnable;

    p_Fm->p_FmDriverParam->fwVerify                             = DEFAULT_VerifyUcode;
    p_Fm->p_FmDriverParam->firmware.size                        = p_FmParam->firmware.size;
    if (p_Fm->p_FmDriverParam->firmware.size)
    {
        p_Fm->p_FmDriverParam->firmware.p_Code = (uint32_t *)XX_Malloc(p_Fm->p_FmDriverParam->firmware.size);
        if (!p_Fm->p_FmDriverParam->firmware.p_Code)
        {
            XX_FreeSpinlock(p_Fm->h_Spinlock);
            XX_Free(p_Fm->p_FmStateStruct);
            XX_Free(p_Fm->p_FmDriverParam);
            XX_Free(p_Fm);
            REPORT_ERROR(MAJOR, E_NO_MEMORY, ("FM firmware code"));
            return NULL;
        }
        memcpy(p_Fm->p_FmDriverParam->firmware.p_Code, p_FmParam->firmware.p_Code ,p_Fm->p_FmDriverParam->firmware.size);
    }

    if (p_Fm->guestId != NCSW_MASTER_ID)
        return p_Fm;

    /* read revision register 1 */
    tmpReg = GET_UINT32(p_Fm->p_FmFpmRegs->fm_ip_rev_1);
    p_Fm->p_FmStateStruct->revInfo.majorRev = (uint8_t)((tmpReg & FPM_REV1_MAJOR_MASK) >> FPM_REV1_MAJOR_SHIFT);
    p_Fm->p_FmStateStruct->revInfo.minorRev = (uint8_t)((tmpReg & FPM_REV1_MINOR_MASK) >> FPM_REV1_MINOR_SHIFT);
    /* Chip dependent, will be configured in Init */

    p_Fm->tnumAgingPeriod                                       = DEFAULT_tnumAgingPeriod;
    p_Fm->p_FmDriverParam->dmaAidOverride                       = DEFAULT_aidOverride;
    p_Fm->p_FmDriverParam->dmaAidMode                           = DEFAULT_aidMode;
#ifdef FM_AID_MODE_NO_TNUM_SW005
    if (p_Fm->p_FmStateStruct->revInfo.majorRev >= 6)
        p_Fm->p_FmDriverParam->dmaAidMode                       = e_FM_DMA_AID_OUT_PORT_ID;
#endif /* FM_AID_MODE_NO_TNUM_SW005 */
#ifdef FM_NO_GUARANTEED_RESET_VALUES
    if (1)//p_Fm->p_FmStateStruct->revInfo.majorRev < 6)
    {
        p_Fm->p_FmStateStruct->totalFifoSize        = 0;
        p_Fm->p_FmStateStruct->totalNumOfTasks      = DEFAULT_totalNumOfTasks;
#ifdef FM_HAS_TOTAL_DMAS
        p_Fm->p_FmStateStruct->maxNumOfOpenDmas     = BMI_MAX_NUM_OF_DMAS;
#endif
        p_Fm->p_FmDriverParam->dmaCommQThresholds.clearEmergency        = DEFAULT_dmaCommQLow;
        p_Fm->p_FmDriverParam->dmaCommQThresholds.assertEmergency       = DEFAULT_dmaCommQHigh;
#if (DPAA_VERSION < 11)
        p_Fm->p_FmDriverParam->dmaReadBufThresholds.clearEmergency      = DEFAULT_dmaReadIntBufLow;
        p_Fm->p_FmDriverParam->dmaReadBufThresholds.assertEmergency     = DEFAULT_dmaReadIntBufHigh;
        p_Fm->p_FmDriverParam->dmaWriteBufThresholds.clearEmergency     = DEFAULT_dmaWriteIntBufLow;
        p_Fm->p_FmDriverParam->dmaWriteBufThresholds.assertEmergency    = DEFAULT_dmaWriteIntBufHigh;
        p_Fm->p_FmDriverParam->dmaAxiDbgNumOfBeats                      = DEFAULT_axiDbgNumOfBeats;
#endif /* (DPAA_VERSION < 11) */
        p_Fm->p_FmDriverParam->dmaCacheOverride                     = DEFAULT_cacheOverride;
        p_Fm->p_FmDriverParam->dmaCamNumOfEntries                   = DEFAULT_dmaCamNumOfEntries;
        p_Fm->p_FmDriverParam->dmaDbgCntMode                        = DEFAULT_dmaDbgCntMode;
        p_Fm->p_FmDriverParam->dmaEnEmergency                       = DEFAULT_dmaEnEmergency;
        p_Fm->p_FmDriverParam->dmaSosEmergency                      = DEFAULT_dmaSosEmergency;
        p_Fm->p_FmDriverParam->dmaWatchdog                          = DEFAULT_dmaWatchdog;
        p_Fm->p_FmDriverParam->dmaEnEmergencySmoother               = DEFAULT_dmaEnEmergencySmoother;
        p_Fm->p_FmDriverParam->dmaEmergencySwitchCounter            = DEFAULT_dmaEmergencySwitchCounter;
        p_Fm->p_FmDriverParam->thresholds.dispLimit                 = DEFAULT_dispLimit;
        p_Fm->p_FmDriverParam->thresholds.prsDispTh                 = DEFAULT_prsDispTh;
        p_Fm->p_FmDriverParam->thresholds.plcrDispTh                = DEFAULT_plcrDispTh;
        p_Fm->p_FmDriverParam->thresholds.kgDispTh                  = DEFAULT_kgDispTh;
        p_Fm->p_FmDriverParam->thresholds.bmiDispTh                 = DEFAULT_bmiDispTh;
        p_Fm->p_FmDriverParam->thresholds.qmiEnqDispTh              = DEFAULT_qmiEnqDispTh;
        p_Fm->p_FmDriverParam->thresholds.qmiDeqDispTh              = DEFAULT_qmiDeqDispTh;
        p_Fm->p_FmDriverParam->thresholds.fmCtl1DispTh              = DEFAULT_fmCtl1DispTh;
        p_Fm->p_FmDriverParam->thresholds.fmCtl2DispTh              = DEFAULT_fmCtl2DispTh;
    }
    else
#endif /* FM_NO_GUARANTEED_RESET_VALUES */
    {
        /* read the values from the registers as they are initialized by the HW with
         * the required values.
         */
        tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_cfg1);
        p_Fm->p_FmStateStruct->totalFifoSize =
            (((tmpReg & BMI_TOTAL_FIFO_SIZE_MASK) >> BMI_CFG1_FIFO_SIZE_SHIFT) + 1) * BMI_FIFO_UNITS;

#ifdef FM_WRONG_RESET_VALUES_ERRATA_FMAN_A005127
        tmpReg = 0x007B0000;
        WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_cfg2, tmpReg);
#endif /* FM_WRONG_RESET_VALUES_ERRATA_FMAN_A005127 */

        tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_cfg2);
        p_Fm->p_FmStateStruct->totalNumOfTasks =
            (uint8_t)(((tmpReg & BMI_TOTAL_NUM_OF_TASKS_MASK) >> BMI_CFG2_TASKS_SHIFT) + 1);

        tmpReg = GET_UINT32(p_Fm->p_FmDmaRegs->fmdmtr);
        p_Fm->p_FmDriverParam->dmaCommQThresholds.assertEmergency =
            (uint8_t)(tmpReg >> DMA_THRESH_COMMQ_SHIFT);

        tmpReg = GET_UINT32(p_Fm->p_FmDmaRegs->fmdmhy);
        p_Fm->p_FmDriverParam->dmaCommQThresholds.clearEmergency =
            (uint8_t)(tmpReg >> DMA_THRESH_COMMQ_SHIFT);

        tmpReg = GET_UINT32(p_Fm->p_FmDmaRegs->fmdmmr);
        p_Fm->p_FmDriverParam->dmaCacheOverride                     = (e_FmDmaCacheOverride)((tmpReg & DMA_MODE_CACHE_OR_MASK) >> DMA_MODE_CACHE_OR_SHIFT);
        p_Fm->p_FmDriverParam->dmaCamNumOfEntries                   = (uint8_t)((((tmpReg & DMA_MODE_CEN_MASK) >> DMA_MODE_CEN_SHIFT) +1)*DMA_CAM_UNITS);
        p_Fm->p_FmDriverParam->dmaDbgCntMode                        = (e_FmDmaDbgCntMode)((tmpReg & DMA_MODE_DBG_MASK) >> DMA_MODE_DBG_SHIFT);
        p_Fm->p_FmDriverParam->dmaEnEmergency                       = (bool)((tmpReg & DMA_MODE_EB)? TRUE : FALSE);

        tmpReg = GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_mxd);
        p_Fm->p_FmDriverParam->thresholds.dispLimit                 = (uint8_t)((tmpReg & FPM_DISP_LIMIT_MASK) << FPM_DISP_LIMIT_SHIFT);

        tmpReg = GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_dis1);
        p_Fm->p_FmDriverParam->thresholds.prsDispTh                 = (uint8_t)((tmpReg & FPM_THR1_PRS_MASK ) >> FPM_THR1_PRS_SHIFT);
        p_Fm->p_FmDriverParam->thresholds.plcrDispTh                = (uint8_t)((tmpReg & FPM_THR1_KG_MASK ) >> FPM_THR1_KG_SHIFT);
        p_Fm->p_FmDriverParam->thresholds.kgDispTh                  = (uint8_t)((tmpReg & FPM_THR1_PLCR_MASK ) >> FPM_THR1_PLCR_SHIFT);
        p_Fm->p_FmDriverParam->thresholds.bmiDispTh                 = (uint8_t)((tmpReg & FPM_THR1_BMI_MASK ) >> FPM_THR1_BMI_SHIFT);

        tmpReg = GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_dis2);
        p_Fm->p_FmDriverParam->thresholds.qmiEnqDispTh              = (uint8_t)((tmpReg & FPM_THR2_QMI_ENQ_MASK ) >> FPM_THR2_QMI_ENQ_SHIFT);
        p_Fm->p_FmDriverParam->thresholds.qmiDeqDispTh              = (uint8_t)((tmpReg & FPM_THR2_QMI_DEQ_MASK ) >> FPM_THR2_QMI_DEQ_SHIFT);
        p_Fm->p_FmDriverParam->thresholds.fmCtl1DispTh              = (uint8_t)((tmpReg & FPM_THR2_FM_CTL1_MASK ) >> FPM_THR2_FM_CTL1_SHIFT);
        p_Fm->p_FmDriverParam->thresholds.fmCtl2DispTh              = (uint8_t)((tmpReg & FPM_THR2_FM_CTL2_MASK ) >> FPM_THR2_FM_CTL2_SHIFT);

        tmpReg = GET_UINT32(p_Fm->p_FmDmaRegs->fmdmsetr);
        p_Fm->p_FmDriverParam->dmaSosEmergency                        = tmpReg;

        tmpReg = GET_UINT32(p_Fm->p_FmDmaRegs->fmdmwcr);
        p_Fm->p_FmDriverParam->dmaWatchdog                          = tmpReg/p_Fm->p_FmStateStruct->fmClkFreq;

        tmpReg = GET_UINT32(p_Fm->p_FmDmaRegs->fmdmemsr);
        p_Fm->p_FmDriverParam->dmaEnEmergencySmoother               = (bool)((tmpReg & DMA_EMSR_EMSTR_MASK)? TRUE : FALSE);
        p_Fm->p_FmDriverParam->dmaEmergencySwitchCounter            = (tmpReg & DMA_EMSR_EMSTR_MASK);
    }

   return p_Fm;
}

/**************************************************************************//**
 @Function      FM_Init

 @Description   Initializes the FM module

 @Param[in]     h_Fm - FM module descriptor

 @Return        E_OK on success; Error code otherwise.
*//***************************************************************************/
t_Error FM_Init(t_Handle h_Fm)
{
    t_Fm                    *p_Fm = (t_Fm*)h_Fm;
    t_FmDriverParam         *p_FmDriverParam = NULL;
    t_Error                 err = E_OK;
    uint32_t                cfgReg = 0;
    int                     i;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);

    p_Fm->p_FmStateStruct->count1MicroBit = FM_TIMESTAMP_1_USEC_BIT;

    if (p_Fm->guestId != NCSW_MASTER_ID)
        return InitGuestMode(p_Fm);

#ifdef FM_NO_GUARANTEED_RESET_VALUES
    if (1)//p_Fm->p_FmStateStruct->revInfo.majorRev < 6)
        /* if user didn't configured totalFifoSize - (totalFifoSize=0) we configure default
         * according to chip. otherwise, we use user's configuration.
         */
        if (p_Fm->p_FmStateStruct->totalFifoSize == 0)
            p_Fm->p_FmStateStruct->totalFifoSize = DEFAULT_totalFifoSize(p_Fm->p_FmStateStruct->revInfo.majorRev);
#endif  /* FM_NO_GUARANTEED_RESET_VALUES */

    CHECK_INIT_PARAMETERS(p_Fm, CheckFmParameters);

    p_FmDriverParam = p_Fm->p_FmDriverParam;

    /* clear revision-dependent non existing exception */
#ifdef FM_NO_DISPATCH_RAM_ECC
    if ((p_Fm->p_FmStateStruct->revInfo.majorRev != 4) &&
        (p_Fm->p_FmStateStruct->revInfo.majorRev < 6))
        p_Fm->p_FmStateStruct->exceptions &= ~FM_EX_BMI_DISPATCH_RAM_ECC;
#endif /* FM_NO_DISPATCH_RAM_ECC */

#ifdef FM_QMI_NO_ECC_EXCEPTIONS
    if (p_Fm->p_FmStateStruct->revInfo.majorRev == 4)
        p_Fm->p_FmStateStruct->exceptions &= ~(FM_EX_QMI_SINGLE_ECC | FM_EX_QMI_DOUBLE_ECC);
#endif /* FM_QMI_NO_ECC_EXCEPTIONS */

#ifdef FM_QMI_NO_SINGLE_ECC_EXCEPTION
    if (p_Fm->p_FmStateStruct->revInfo.majorRev >= 6)
       p_Fm->p_FmStateStruct->exceptions &= ~FM_EX_QMI_SINGLE_ECC;
#endif /* FM_QMI_NO_SINGLE_ECC_EXCEPTION */

    FmMuramClear(p_Fm->h_FmMuram);

    /* clear CPG */
    IOMemSet32(UINT_TO_PTR(p_Fm->baseAddr + FM_MM_CGP), 0, FM_PORT_NUM_OF_CONGESTION_GRPS);

    /* add to the default exceptions the user's definitions */
    p_Fm->p_FmStateStruct->exceptions |= p_FmDriverParam->userSetExceptions;

#ifdef FM_UCODE_NOT_RESET_ERRATA_BUGZILLA6173
    if (p_FmDriverParam->resetOnInit)
    {
        if ((err = FwNotResetErratumBugzilla6173WA(p_Fm)) != E_OK)
            RETURN_ERROR(MAJOR, err, NO_MSG);
    }
    else
    {
#endif /* FM_UCODE_NOT_RESET_ERRATA_BUGZILLA6173 */

    /* Reset the FM if required. */
    if (p_FmDriverParam->resetOnInit)
    {
        WRITE_UINT32(p_Fm->p_FmFpmRegs->fm_rstc, FPM_RSTC_FM_RESET);
        CORE_MemoryBarrier();
        XX_UDelay(100);

        if (GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_gs) & QMI_GS_HALT_NOT_BUSY)
        {
            uint32_t tmpReg = GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_ee);
            /* clear tmpReg event bits in order not to clear standing events */
            tmpReg &= ~(FPM_EV_MASK_DOUBLE_ECC | FPM_EV_MASK_STALL | FPM_EV_MASK_SINGLE_ECC);
            WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_ee, tmpReg | FPM_EV_MASK_RELEASE_FM);
            CORE_MemoryBarrier();
            XX_UDelay(100);
        }
    }

    /*************************************/
    /* Load FMan-Controller code to IRAM */
    /*************************************/
    if (ClearIRam(p_Fm) != E_OK)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);
    if (p_Fm->p_FmDriverParam->firmware.p_Code &&
        (LoadFmanCtrlCode(p_Fm) != E_OK))
        RETURN_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);
#ifdef FM_UCODE_NOT_RESET_ERRATA_BUGZILLA6173
    }
#endif /* FM_UCODE_NOT_RESET_ERRATA_BUGZILLA6173 */

#ifdef FM_CAPWAP_SUPPORT
    /* save first 256 byte in MURAM */
    p_Fm->resAddr = PTR_TO_UINT(FM_MURAM_AllocMem(p_Fm->h_FmMuram, 256, 0));
    if (!p_Fm->resAddr)
        RETURN_ERROR(MAJOR, E_NO_MEMORY, ("MURAM alloc for reserved Area failed"));

    WRITE_BLOCK(UINT_TO_PTR(p_Fm->resAddr), 0, 256);
#endif /* FM_CAPWAP_SUPPORT */

#if (DPAA_VERSION >= 11)
    p_Fm->partVSPBase = AllocVSPsForPartition(h_Fm, p_Fm->partVSPBase, p_Fm->partNumOfVSPs, p_Fm->guestId);
    if (p_Fm->partVSPBase == ILLEGAL_BASE)
        DBG(WARNING, ("partition VSPs allocation is FAILED"));
#endif /* (DPAA_VERSION >= 11) */

    /* General FM driver initialization */
    p_Fm->fmMuramPhysBaseAddr =
        (uint64_t)(XX_VirtToPhys(UINT_TO_PTR(p_Fm->baseAddr + FM_MM_MURAM)));

    for (i=0;i<e_FM_EV_DUMMY_LAST;i++)
        p_Fm->intrMng[i].f_Isr = UnimplementedIsr;
    for (i=0;i<FM_NUM_OF_FMAN_CTRL_EVENT_REGS;i++)
        p_Fm->fmanCtrlIntr[i].f_Isr = UnimplementedFmanCtrlIsr;

    /**********************/
    /* Init DMA Registers */
    /**********************/
    err = InitFmDma(p_Fm);
    if (err != E_OK)
    {
        FreeInitResources(p_Fm);
        RETURN_ERROR(MAJOR, err, NO_MSG);
    }

    /**********************/
    /* Init FPM Registers */
    /**********************/
    err = InitFmFpm(p_Fm);
    if (err != E_OK)
    {
        FreeInitResources(p_Fm);
        RETURN_ERROR(MAJOR, err, NO_MSG);
    }

    /* define common resources */
    /* allocate MURAM for FIFO according to total size */
    p_Fm->fifoBaseAddr = PTR_TO_UINT(FM_MURAM_AllocMem(p_Fm->h_FmMuram,
                                                       p_Fm->p_FmStateStruct->totalFifoSize,
                                                       BMI_FIFO_ALIGN));
    if (!p_Fm->fifoBaseAddr)
    {
        FreeInitResources(p_Fm);
        RETURN_ERROR(MAJOR, E_NO_MEMORY, ("MURAM alloc for BMI FIFO failed"));
    }

    /**********************/
    /* Init BMI Registers */
    /**********************/
    err = InitFmBmi(p_Fm);
    if (err != E_OK)
    {
        FreeInitResources(p_Fm);
        RETURN_ERROR(MAJOR, err, NO_MSG);
    }

    /**********************/
    /* Init QMI Registers */
    /**********************/
    err = InitFmQmi(p_Fm);
    if (err != E_OK)
    {
        FreeInitResources(p_Fm);
        RETURN_ERROR(MAJOR, err, NO_MSG);
    }

    /* build the FM master partition IPC address */
    if (Sprint (p_Fm->fmModuleName, "FM_%d_%d",p_Fm->p_FmStateStruct->fmId, NCSW_MASTER_ID) != 6)
    {
        FreeInitResources(p_Fm);
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Sprint failed"));
    }

    err = XX_IpcRegisterMsgHandler(p_Fm->fmModuleName, FmHandleIpcMsgCB, p_Fm, FM_IPC_MAX_REPLY_SIZE);
    if (err)
    {
        FreeInitResources(p_Fm);
        RETURN_ERROR(MAJOR, err, NO_MSG);
    }

    /* Register the FM interrupts handlers */
    if (p_Fm->p_FmStateStruct->irq != NO_IRQ)
    {
        XX_SetIntr(p_Fm->p_FmStateStruct->irq, FM_EventIsr, p_Fm);
        XX_EnableIntr(p_Fm->p_FmStateStruct->irq);
    }

    if (p_Fm->p_FmStateStruct->errIrq != NO_IRQ)
    {
        XX_SetIntr(p_Fm->p_FmStateStruct->errIrq, (void (*) (t_Handle))FM_ErrorIsr, p_Fm);
        XX_EnableIntr(p_Fm->p_FmStateStruct->errIrq);
    }

    /**********************/
    /* Enable all modules */
    /**********************/
    /* clear & enable global counters  - calculate reg and save for later,
       because it's the same reg for QMI enable */
    cfgReg = QMI_CFG_EN_COUNTERS;
#ifndef FM_QMI_NO_DEQ_OPTIONS_SUPPORT
    if (p_Fm->p_FmStateStruct->revInfo.majorRev != 4)
        cfgReg |= (uint32_t)(((QMI_DEF_TNUMS_THRESH) << 8) | (uint32_t)QMI_DEF_TNUMS_THRESH);
#endif /* FM_QMI_NO_DEQ_OPTIONS_SUPPORT */

    WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_init, BMI_INIT_START);
    WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_gc, cfgReg | QMI_CFG_ENQ_EN | QMI_CFG_DEQ_EN);

    EnableTimeStamp(p_Fm);

    if (p_Fm->p_FmDriverParam->firmware.p_Code)
    {
        XX_Free(p_Fm->p_FmDriverParam->firmware.p_Code);
        p_Fm->p_FmDriverParam->firmware.p_Code = NULL;
    }

    XX_Free(p_Fm->p_FmDriverParam);
    p_Fm->p_FmDriverParam = NULL;

    return E_OK;
}

/**************************************************************************//**
 @Function      FM_Free

 @Description   Frees all resources that were assigned to FM module.

                Calling this routine invalidates the descriptor.

 @Param[in]     h_Fm - FM module descriptor

 @Return        E_OK on success; Error code otherwise.
*//***************************************************************************/
t_Error FM_Free(t_Handle h_Fm)
{
    t_Fm    *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);

    if (p_Fm->guestId != NCSW_MASTER_ID)
    {
#if (DPAA_VERSION >= 11)
        FreeVSPsForPartition(h_Fm, p_Fm->partVSPBase, p_Fm->partNumOfVSPs, p_Fm->guestId);

        if (p_Fm->p_FmSp)
        {
            XX_Free(p_Fm->p_FmSp);
            p_Fm->p_FmSp = NULL;
        }
#endif /* (DPAA_VERSION >= 11) */

        if (p_Fm->fmModuleName)
            XX_IpcUnregisterMsgHandler(p_Fm->fmModuleName);

        if (!p_Fm->recoveryMode)
            XX_Free(p_Fm->p_FmStateStruct);

        XX_Free(p_Fm);

        return E_OK;
    }

    /* disable BMI and QMI */
    WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_init, 0);
    WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_gc, 0);

    if ((p_Fm->guestId == NCSW_MASTER_ID) && (p_Fm->fmModuleName[0] != 0))
        XX_IpcUnregisterMsgHandler(p_Fm->fmModuleName);

    if (p_Fm->p_FmStateStruct)
    {
        if (p_Fm->p_FmStateStruct->irq != NO_IRQ)
        {
            XX_DisableIntr(p_Fm->p_FmStateStruct->irq);
            XX_FreeIntr(p_Fm->p_FmStateStruct->irq);
        }
        if (p_Fm->p_FmStateStruct->errIrq != NO_IRQ)
        {
            XX_DisableIntr(p_Fm->p_FmStateStruct->errIrq);
            XX_FreeIntr(p_Fm->p_FmStateStruct->errIrq);
        }
    }

#if (DPAA_VERSION >= 11)
    FreeVSPsForPartition(h_Fm, p_Fm->partVSPBase, p_Fm->partNumOfVSPs, p_Fm->guestId);

    if (p_Fm->p_FmSp)
    {
        XX_Free(p_Fm->p_FmSp);
        p_Fm->p_FmSp = NULL;
    }
#endif /* (DPAA_VERSION >= 11) */

    if (p_Fm->h_Spinlock)
        XX_FreeSpinlock(p_Fm->h_Spinlock);

    if (p_Fm->p_FmDriverParam)
    {
        if (p_Fm->p_FmDriverParam->firmware.p_Code)
            XX_Free(p_Fm->p_FmDriverParam->firmware.p_Code);
        XX_Free(p_Fm->p_FmDriverParam);
        p_Fm->p_FmDriverParam = NULL;
    }

    FreeInitResources(p_Fm);

    if (!p_Fm->recoveryMode && p_Fm->p_FmStateStruct)
        XX_Free(p_Fm->p_FmStateStruct);

    XX_Free(p_Fm);

    return E_OK;
}

/*************************************************/
/*       API Advanced Init unit functions        */
/*************************************************/

t_Error FM_ConfigResetOnInit(t_Handle h_Fm, bool enable)
{

    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    p_Fm->p_FmDriverParam->resetOnInit = enable;

    return E_OK;
}

t_Error FM_ConfigTotalFifoSize(t_Handle h_Fm, uint32_t totalFifoSize)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    p_Fm->p_FmStateStruct->totalFifoSize = totalFifoSize;

    return E_OK;
}

t_Error FM_ConfigDmaCacheOverride(t_Handle h_Fm, e_FmDmaCacheOverride cacheOverride)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    p_Fm->p_FmDriverParam->dmaCacheOverride = cacheOverride;

    return E_OK;
}

t_Error FM_ConfigDmaAidOverride(t_Handle h_Fm, bool aidOverride)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    p_Fm->p_FmDriverParam->dmaAidOverride = aidOverride;

    return E_OK;
}

t_Error FM_ConfigDmaAidMode(t_Handle h_Fm, e_FmDmaAidMode aidMode)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    p_Fm->p_FmDriverParam->dmaAidMode = aidMode;

    return E_OK;
}

t_Error FM_ConfigDmaAxiDbgNumOfBeats(t_Handle h_Fm, uint8_t axiDbgNumOfBeats)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

#if (DPAA_VERSION >= 11)
    RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("Not available for this FM revision!"));
#endif
    p_Fm->p_FmDriverParam->dmaAxiDbgNumOfBeats = axiDbgNumOfBeats;

    return E_OK;
}

t_Error FM_ConfigDmaCamNumOfEntries(t_Handle h_Fm, uint8_t numOfEntries)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    p_Fm->p_FmDriverParam->dmaCamNumOfEntries = numOfEntries;

    return E_OK;
}

t_Error FM_ConfigDmaDbgCounter(t_Handle h_Fm, e_FmDmaDbgCntMode fmDmaDbgCntMode)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    p_Fm->p_FmDriverParam->dmaDbgCntMode = fmDmaDbgCntMode;

    return E_OK;
}

t_Error FM_ConfigDmaStopOnBusErr(t_Handle h_Fm, bool stop)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    p_Fm->p_FmDriverParam->dmaStopOnBusError = stop;

    return E_OK;
}

t_Error FM_ConfigDmaEmergency(t_Handle h_Fm, t_FmDmaEmergency *p_Emergency)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    p_Fm->p_FmDriverParam->dmaEnEmergency = TRUE;
    memcpy(&p_Fm->p_FmDriverParam->dmaEmergency, p_Emergency, sizeof(t_FmDmaEmergency));

    return E_OK;
}

t_Error FM_ConfigDmaEmergencySmoother(t_Handle h_Fm, uint32_t emergencyCnt)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    p_Fm->p_FmDriverParam->dmaEnEmergencySmoother = TRUE;
    p_Fm->p_FmDriverParam->dmaEmergencySwitchCounter = emergencyCnt;

    return E_OK;
}

t_Error FM_ConfigDmaErr(t_Handle h_Fm, e_FmDmaErr dmaErr)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    p_Fm->p_FmDriverParam->dmaErr = dmaErr;

    return E_OK;
}

t_Error FM_ConfigCatastrophicErr(t_Handle h_Fm, e_FmCatastrophicErr catastrophicErr)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    p_Fm->p_FmDriverParam->catastrophicErr = catastrophicErr;

    return E_OK;
}

t_Error FM_ConfigEnableMuramTestMode(t_Handle h_Fm)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    if (p_Fm->p_FmStateStruct->revInfo.majorRev >= 6)
        RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("Not available for this FM revision!"));

    p_Fm->p_FmDriverParam->enMuramTestMode = TRUE;

    return E_OK;
}

t_Error FM_ConfigEnableIramTestMode(t_Handle h_Fm)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE );
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    if (p_Fm->p_FmStateStruct->revInfo.majorRev >= 6)
        RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("Not available for this FM revision!"));

    p_Fm->p_FmDriverParam->enIramTestMode = TRUE;

    return E_OK;
}

t_Error FM_ConfigHaltOnExternalActivation(t_Handle h_Fm, bool enable)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    p_Fm->p_FmDriverParam->haltOnExternalActivation = enable;

    return E_OK;
}

t_Error FM_ConfigHaltOnUnrecoverableEccError(t_Handle h_Fm, bool enable)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    if (p_Fm->p_FmStateStruct->revInfo.majorRev >= 6)
        RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("Not available for this FM revision!"));

    p_Fm->p_FmDriverParam->haltOnUnrecoverableEccError = enable;
    return E_OK;
}

t_Error FM_ConfigException(t_Handle h_Fm, e_FmExceptions exception, bool enable)
{
    t_Fm                *p_Fm = (t_Fm*)h_Fm;
    uint32_t            bitMask = 0;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    GET_EXCEPTION_FLAG(bitMask, exception);
    if (bitMask)
    {
        if (enable)
            p_Fm->p_FmDriverParam->userSetExceptions |= bitMask;
        else
            p_Fm->p_FmStateStruct->exceptions &= ~bitMask;
   }
    else
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Undefined exception"));

    return E_OK;
}

t_Error FM_ConfigExternalEccRamsEnable(t_Handle h_Fm, bool enable)
{
    t_Fm        *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    p_Fm->p_FmDriverParam->externalEccRamsEnable = enable;

    return E_OK;
}

t_Error FM_ConfigTnumAgingPeriod(t_Handle h_Fm, uint16_t tnumAgingPeriod)
{
    t_Fm             *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    p_Fm->tnumAgingPeriod = tnumAgingPeriod;

    return E_OK;
}

/****************************************************/
/*       Hidden-DEBUG Only API                      */
/****************************************************/

t_Error FM_ConfigThresholds(t_Handle h_Fm, t_FmThresholds *p_FmThresholds)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    memcpy(&p_Fm->p_FmDriverParam->thresholds, p_FmThresholds, sizeof(t_FmThresholds));

    return E_OK;
}

t_Error FM_ConfigDmaSosEmergencyThreshold(t_Handle h_Fm, uint32_t dmaSosEmergency)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    p_Fm->p_FmDriverParam->dmaSosEmergency = dmaSosEmergency;

    return E_OK;
}

t_Error FM_ConfigDmaWriteBufThresholds(t_Handle h_Fm, t_FmDmaThresholds *p_FmDmaThresholds)

{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

#if (DPAA_VERSION >= 11)
    RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("Not available for this FM revision!"));
#endif
    memcpy(&p_Fm->p_FmDriverParam->dmaWriteBufThresholds, p_FmDmaThresholds, sizeof(t_FmDmaThresholds));

    return E_OK;
}

t_Error FM_ConfigDmaCommQThresholds(t_Handle h_Fm, t_FmDmaThresholds *p_FmDmaThresholds)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    memcpy(&p_Fm->p_FmDriverParam->dmaCommQThresholds, p_FmDmaThresholds, sizeof(t_FmDmaThresholds));

    return E_OK;
}

t_Error FM_ConfigDmaReadBufThresholds(t_Handle h_Fm, t_FmDmaThresholds *p_FmDmaThresholds)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

#if (DPAA_VERSION >= 11)
    RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("Not available for this FM revision!"));
#endif
    memcpy(&p_Fm->p_FmDriverParam->dmaReadBufThresholds, p_FmDmaThresholds, sizeof(t_FmDmaThresholds));

    return E_OK;
}

t_Error FM_ConfigDmaWatchdog(t_Handle h_Fm, uint32_t watchdogValue)
{
    t_Fm                *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    p_Fm->p_FmDriverParam->dmaWatchdog = watchdogValue;

    return E_OK;
}

t_Error FM_ConfigEnableCounters(t_Handle h_Fm)
{
    t_Fm                *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
UNUSED(p_Fm);

    return E_OK;
}

/****************************************************/
/*       API Run-time Control uint functions        */
/****************************************************/
void FM_EventIsr(t_Handle h_Fm)
{
#define FM_M_CALL_1G_MAC_ISR(_id)    \
    {                                \
        if (p_Fm->guestId != p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_1G_MAC0+_id)].guestId)    \
            SendIpcIsr(p_Fm, (e_FmInterModuleEvent)(e_FM_EV_1G_MAC0+_id), pending);                 \
        else                                                                                        \
            p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_1G_MAC0+_id)].f_Isr(p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_1G_MAC0+_id)].h_SrcHandle);\
    }
#define FM_M_CALL_10G_MAC_ISR(_id)   \
    {                                \
        if (p_Fm->guestId != p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_10G_MAC0+_id)].guestId)    \
            SendIpcIsr(p_Fm, (e_FmInterModuleEvent)(e_FM_EV_10G_MAC0+_id), pending);                 \
        else                                                                                         \
            p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_10G_MAC0+_id)].f_Isr(p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_10G_MAC0+_id)].h_SrcHandle);\
    }
    t_Fm                    *p_Fm = (t_Fm*)h_Fm;
    uint32_t                pending, event;

    SANITY_CHECK_RETURN(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN(!p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    /* normal interrupts */
    pending = GET_UINT32(p_Fm->p_FmFpmRegs->fm_npi);
    if (!pending)
        return;

    if (pending & INTR_EN_QMI)
        QmiEvent(p_Fm);
    if (pending & INTR_EN_PRS)
        p_Fm->intrMng[e_FM_EV_PRS].f_Isr(p_Fm->intrMng[e_FM_EV_PRS].h_SrcHandle);
    if (pending & INTR_EN_PLCR)
        p_Fm->intrMng[e_FM_EV_PLCR].f_Isr(p_Fm->intrMng[e_FM_EV_PLCR].h_SrcHandle);
    if (pending & INTR_EN_TMR)
            p_Fm->intrMng[e_FM_EV_TMR].f_Isr(p_Fm->intrMng[e_FM_EV_TMR].h_SrcHandle);

    /* MAC events may belong to different partitions */
    if (pending & INTR_EN_1G_MAC0)
        FM_M_CALL_1G_MAC_ISR(0);
    if (pending & INTR_EN_1G_MAC1)
        FM_M_CALL_1G_MAC_ISR(1);
    if (pending & INTR_EN_1G_MAC2)
        FM_M_CALL_1G_MAC_ISR(2);
    if (pending & INTR_EN_1G_MAC3)
        FM_M_CALL_1G_MAC_ISR(3);
    if (pending & INTR_EN_1G_MAC4)
        FM_M_CALL_1G_MAC_ISR(4);
    if (pending & INTR_EN_1G_MAC5)
        FM_M_CALL_1G_MAC_ISR(5);
    if (pending & INTR_EN_1G_MAC6)
        FM_M_CALL_1G_MAC_ISR(6);
    if (pending & INTR_EN_1G_MAC7)
        FM_M_CALL_1G_MAC_ISR(7);
    if (pending & INTR_EN_10G_MAC0)
        FM_M_CALL_10G_MAC_ISR(0);
    if (pending & INTR_EN_10G_MAC1)
        FM_M_CALL_10G_MAC_ISR(1);

    /* IM port events may belong to different partitions */
    if (pending & INTR_EN_REV0)
    {
        event = GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_fcev[0]) & GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_cee[0]);
        WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_cev[0], event);
        if (p_Fm->guestId != p_Fm->intrMng[e_FM_EV_FMAN_CTRL_0].guestId)
            /*TODO IPC ISR For Fman Ctrl */
            ASSERT_COND(0);
            /* SendIpcIsr(p_Fm, e_FM_EV_FMAN_CTRL_0, pending); */
        else
            p_Fm->fmanCtrlIntr[0].f_Isr(p_Fm->fmanCtrlIntr[0].h_SrcHandle, event);

    }
    if (pending & INTR_EN_REV1)
    {
        event = GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_fcev[1]) & GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_cee[1]);
        WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_cev[1], event);
        if (p_Fm->guestId != p_Fm->intrMng[e_FM_EV_FMAN_CTRL_1].guestId)
            /*TODO IPC ISR For Fman Ctrl */
            ASSERT_COND(0);
            /* SendIpcIsr(p_Fm, e_FM_EV_FMAN_CTRL_1, pending); */
        else
            p_Fm->fmanCtrlIntr[1].f_Isr(p_Fm->fmanCtrlIntr[1].h_SrcHandle, event);
    }
    if (pending & INTR_EN_REV2)
    {
        event = GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_fcev[2]) & GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_cee[2]);
        WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_cev[2], event);
        if (p_Fm->guestId != p_Fm->intrMng[e_FM_EV_FMAN_CTRL_2].guestId)
            /*TODO IPC ISR For Fman Ctrl */
            ASSERT_COND(0);
            /* SendIpcIsr(p_Fm, e_FM_EV_FMAN_CTRL_2, pending); */
        else
           p_Fm->fmanCtrlIntr[2].f_Isr(p_Fm->fmanCtrlIntr[2].h_SrcHandle, event);
    }
    if (pending & INTR_EN_REV3)
    {
        event = GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_fcev[3]) & GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_cee[3]);
        WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_cev[3], event);
        if (p_Fm->guestId != p_Fm->intrMng[e_FM_EV_FMAN_CTRL_3].guestId)
            /*TODO IPC ISR For Fman Ctrl */
            ASSERT_COND(0);
            /* SendIpcIsr(p_Fm, e_FM_EV_FMAN_CTRL_2, pendin3); */
        else
            p_Fm->fmanCtrlIntr[3].f_Isr(p_Fm->fmanCtrlIntr[3].h_SrcHandle, event);
    }
#ifdef FM_MACSEC_SUPPORT
    if (pending & INTR_EN_MACSEC_MAC0)
    {
       if (p_Fm->guestId != p_Fm->intrMng[e_FM_EV_MACSEC_MAC0].guestId)
            SendIpcIsr(p_Fm, e_FM_EV_MACSEC_MAC0, pending);
        else
            p_Fm->intrMng[e_FM_EV_MACSEC_MAC0].f_Isr(p_Fm->intrMng[e_FM_EV_MACSEC_MAC0].h_SrcHandle);
    }
#endif /* FM_MACSEC_SUPPORT */
}

t_Error FM_ErrorIsr(t_Handle h_Fm)
{
#define FM_M_CALL_1G_MAC_ERR_ISR(_id)   \
    {                                   \
       if (p_Fm->guestId != p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_ERR_1G_MAC0+_id)].guestId) \
            SendIpcIsr(p_Fm, (e_FmInterModuleEvent)(e_FM_EV_ERR_1G_MAC0+_id), pending);             \
       else                                                                                         \
            p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_ERR_1G_MAC0+_id)].f_Isr(p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_ERR_1G_MAC0+_id)].h_SrcHandle);\
    }
#define FM_M_CALL_10G_MAC_ERR_ISR(_id)   \
    {                                    \
       if (p_Fm->guestId != p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_ERR_10G_MAC0+_id)].guestId) \
            SendIpcIsr(p_Fm, (e_FmInterModuleEvent)(e_FM_EV_ERR_10G_MAC0+_id), pending);             \
       else                                                                                          \
            p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_ERR_10G_MAC0+_id)].f_Isr(p_Fm->intrMng[(e_FmInterModuleEvent)(e_FM_EV_ERR_10G_MAC0+_id)].h_SrcHandle);\
    }
    t_Fm                    *p_Fm = (t_Fm*)h_Fm;
    uint32_t                pending;

    SANITY_CHECK_RETURN_ERROR(h_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Fm->p_FmDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    /* error interrupts */
    pending = GET_UINT32(p_Fm->p_FmFpmRegs->fm_epi);
    if (!pending)
        return ERROR_CODE(E_EMPTY);

    if (pending & ERR_INTR_EN_BMI)
        BmiErrEvent(p_Fm);
    if (pending & ERR_INTR_EN_QMI)
        QmiErrEvent(p_Fm);
    if (pending & ERR_INTR_EN_FPM)
        FpmErrEvent(p_Fm);
    if (pending & ERR_INTR_EN_DMA)
        DmaErrEvent(p_Fm);
    if (pending & ERR_INTR_EN_IRAM)
        IramErrIntr(p_Fm);
    if (pending & ERR_INTR_EN_MURAM)
        MuramErrIntr(p_Fm);
    if (pending & ERR_INTR_EN_PRS)
        p_Fm->intrMng[e_FM_EV_ERR_PRS].f_Isr(p_Fm->intrMng[e_FM_EV_ERR_PRS].h_SrcHandle);
    if (pending & ERR_INTR_EN_PLCR)
        p_Fm->intrMng[e_FM_EV_ERR_PLCR].f_Isr(p_Fm->intrMng[e_FM_EV_ERR_PLCR].h_SrcHandle);
    if (pending & ERR_INTR_EN_KG)
        p_Fm->intrMng[e_FM_EV_ERR_KG].f_Isr(p_Fm->intrMng[e_FM_EV_ERR_KG].h_SrcHandle);

    /* MAC events may belong to different partitions */
    if (pending & ERR_INTR_EN_1G_MAC0)
        FM_M_CALL_1G_MAC_ERR_ISR(0);
    if (pending & ERR_INTR_EN_1G_MAC1)
        FM_M_CALL_1G_MAC_ERR_ISR(1);
    if (pending & ERR_INTR_EN_1G_MAC2)
        FM_M_CALL_1G_MAC_ERR_ISR(2);
    if (pending & ERR_INTR_EN_1G_MAC3)
        FM_M_CALL_1G_MAC_ERR_ISR(3);
    if (pending & ERR_INTR_EN_1G_MAC4)
        FM_M_CALL_1G_MAC_ERR_ISR(4);
    if (pending & ERR_INTR_EN_1G_MAC5)
        FM_M_CALL_1G_MAC_ERR_ISR(5);
    if (pending & ERR_INTR_EN_1G_MAC6)
        FM_M_CALL_1G_MAC_ERR_ISR(6);
    if (pending & ERR_INTR_EN_1G_MAC7)
        FM_M_CALL_1G_MAC_ERR_ISR(7);
    if (pending & ERR_INTR_EN_10G_MAC0)
        FM_M_CALL_10G_MAC_ERR_ISR(0);
    if (pending & ERR_INTR_EN_10G_MAC1)
        FM_M_CALL_10G_MAC_ERR_ISR(1);

#ifdef FM_MACSEC_SUPPORT
    if (pending & ERR_INTR_EN_MACSEC_MAC0)
    {
       if (p_Fm->guestId != p_Fm->intrMng[e_FM_EV_ERR_MACSEC_MAC0].guestId)
            SendIpcIsr(p_Fm, e_FM_EV_ERR_MACSEC_MAC0, pending);
        else
            p_Fm->intrMng[e_FM_EV_ERR_MACSEC_MAC0].f_Isr(p_Fm->intrMng[e_FM_EV_ERR_MACSEC_MAC0].h_SrcHandle);
    }
#endif /* FM_MACSEC_SUPPORT */

    return E_OK;
}

t_Error FM_SetPortsBandwidth(t_Handle h_Fm, t_FmPortsBandwidthParams *p_PortsBandwidth)
{
    t_Fm        *p_Fm = (t_Fm*)h_Fm;
    int         i;
    uint8_t     sum;
    uint8_t     hardwarePortId;
    uint32_t    tmpRegs[8] = {0,0,0,0,0,0,0,0};
    uint8_t     relativePortId, shift, weight, maxPercent = 0;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Fm->p_FmDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    /* check that all ports add up to 100% */
    sum = 0;
    for (i=0;i<p_PortsBandwidth->numOfPorts;i++)
        sum +=p_PortsBandwidth->portsBandwidths[i].bandwidth;
    if (sum != 100)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Sum of ports bandwidth differ from 100%"));

    /* find highest percent */
    for (i=0;i<p_PortsBandwidth->numOfPorts;i++)
    {
        if (p_PortsBandwidth->portsBandwidths[i].bandwidth > maxPercent)
            maxPercent = p_PortsBandwidth->portsBandwidths[i].bandwidth;
    }

    /* calculate weight for each port */
    for (i=0;i<p_PortsBandwidth->numOfPorts;i++)
    {
        weight = (uint8_t)((p_PortsBandwidth->portsBandwidths[i].bandwidth * PORT_MAX_WEIGHT )/maxPercent);
        /* we want even division between 1-to-PORT_MAX_WEIGHT. so if exect division
           is not reached, we round up so that:
           0 until maxPercent/PORT_MAX_WEIGHT get "1"
           maxPercent/PORT_MAX_WEIGHT+1 until (maxPercent/PORT_MAX_WEIGHT)*2 get "2"
           ...
           maxPercent - maxPercent/PORT_MAX_WEIGHT until maxPercent get "PORT_MAX_WEIGHT: */
        if ((uint8_t)((p_PortsBandwidth->portsBandwidths[i].bandwidth * PORT_MAX_WEIGHT ) % maxPercent))
            weight++;

        /* find the location of this port within the register */
        SW_PORT_ID_TO_HW_PORT_ID(hardwarePortId,
                                 p_PortsBandwidth->portsBandwidths[i].type,
                                 p_PortsBandwidth->portsBandwidths[i].relativePortId);
        relativePortId = (uint8_t)(hardwarePortId % 8);
        shift = (uint8_t)(32-4*(relativePortId+1));


        if (weight > 1)
            /* Add this port to tmpReg */
            /* (each 8 ports result in one register)*/
            tmpRegs[hardwarePortId/8] |= ((weight-1) << shift);
    }

    for (i=0;i<8;i++)
        if (tmpRegs[i])
            WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_arb[i], tmpRegs[i]);

    return E_OK;
}

t_Error FM_EnableRamsEcc(t_Handle h_Fm)
{
    t_Fm        *p_Fm = (t_Fm*)h_Fm;
    uint32_t    tmpReg;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    /*SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);*/

    if (p_Fm->guestId != NCSW_MASTER_ID)
    {
        t_FmIpcMsg      msg;
        t_Error         err;

        memset(&msg, 0, sizeof(msg));
        msg.msgId = FM_ENABLE_RAM_ECC;
        err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                (uint8_t*)&msg,
                                sizeof(msg.msgId),
                                NULL,
                                NULL,
                                NULL,
                                NULL);
        if (err != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
        return E_OK;
    }

    if (!p_Fm->p_FmStateStruct->internalCall)
        p_Fm->p_FmStateStruct->explicitEnable = TRUE;
    p_Fm->p_FmStateStruct->internalCall = FALSE;

    if (p_Fm->p_FmStateStruct->ramsEccEnable)
        return E_OK;
    else
    {
        tmpReg = GET_UINT32(p_Fm->p_FmFpmRegs->fm_rcr);
        if (tmpReg & FPM_RAM_CTL_RAMS_ECC_EN_SRC_SEL)
            WRITE_UINT32(p_Fm->p_FmFpmRegs->fm_rcr, tmpReg | FPM_RAM_CTL_IRAM_ECC_EN);
        else
            WRITE_UINT32(p_Fm->p_FmFpmRegs->fm_rcr, tmpReg | (FPM_RAM_CTL_RAMS_ECC_EN | FPM_RAM_CTL_IRAM_ECC_EN));
        p_Fm->p_FmStateStruct->ramsEccEnable = TRUE;
    }

    return E_OK;
}

t_Error FM_DisableRamsEcc(t_Handle h_Fm)
{
    t_Fm        *p_Fm = (t_Fm*)h_Fm;
    uint32_t    tmpReg;
    bool        explicitDisable = FALSE;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Fm->p_FmDriverParam, E_INVALID_HANDLE);
    /*SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);*/

    if (p_Fm->guestId != NCSW_MASTER_ID)
    {
        t_Error             err;
        t_FmIpcMsg          msg;

        memset(&msg, 0, sizeof(msg));
        msg.msgId = FM_DISABLE_RAM_ECC;
        if ((err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                     (uint8_t*)&msg,
                                     sizeof(msg.msgId),
                                     NULL,
                                     NULL,
                                     NULL,
                                     NULL)) != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
        return E_OK;
    }

    if (!p_Fm->p_FmStateStruct->internalCall)
        explicitDisable = TRUE;
    p_Fm->p_FmStateStruct->internalCall = FALSE;

    /* if rams are already disabled, or if rams were explicitly enabled and are
       currently called indirectly (not explicitly), ignore this call. */
    if (!p_Fm->p_FmStateStruct->ramsEccEnable ||
        (p_Fm->p_FmStateStruct->explicitEnable && !explicitDisable))
        return E_OK;
    else
    {
        if (p_Fm->p_FmStateStruct->explicitEnable)
            /* This is the case were both explicit are TRUE.
               Turn off this flag for cases were following ramsEnable
               routines are called */
            p_Fm->p_FmStateStruct->explicitEnable = FALSE;

        tmpReg = GET_UINT32(p_Fm->p_FmFpmRegs->fm_rcr);
        if (tmpReg & FPM_RAM_CTL_RAMS_ECC_EN_SRC_SEL)
        {
            DBG(WARNING, ("Rams ECC is configured to be controlled through JTAG"));
            WRITE_UINT32(p_Fm->p_FmFpmRegs->fm_rcr, tmpReg & ~FPM_RAM_CTL_IRAM_ECC_EN);
        }
        else
            WRITE_UINT32(p_Fm->p_FmFpmRegs->fm_rcr, tmpReg & ~(FPM_RAM_CTL_RAMS_ECC_EN | FPM_RAM_CTL_IRAM_ECC_EN));
        p_Fm->p_FmStateStruct->ramsEccEnable = FALSE;
    }

    return E_OK;
}

t_Error FM_SetException(t_Handle h_Fm, e_FmExceptions exception, bool enable)
{
    t_Fm                *p_Fm = (t_Fm*)h_Fm;
    uint32_t            bitMask = 0;
    uint32_t            tmpReg;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Fm->p_FmDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    GET_EXCEPTION_FLAG(bitMask, exception);
    if (bitMask)
    {
        if (enable)
            p_Fm->p_FmStateStruct->exceptions |= bitMask;
        else
            p_Fm->p_FmStateStruct->exceptions &= ~bitMask;

        switch (exception)
        {
             case (e_FM_EX_DMA_BUS_ERROR):
                tmpReg = GET_UINT32(p_Fm->p_FmDmaRegs->fmdmmr);
                if (enable)
                    tmpReg |= DMA_MODE_BER;
                else
                    tmpReg &= ~DMA_MODE_BER;
                /* disable bus error */
                WRITE_UINT32(p_Fm->p_FmDmaRegs->fmdmmr, tmpReg);
                break;
             case (e_FM_EX_DMA_READ_ECC):
             case (e_FM_EX_DMA_SYSTEM_WRITE_ECC):
             case (e_FM_EX_DMA_FM_WRITE_ECC):
                tmpReg = GET_UINT32(p_Fm->p_FmDmaRegs->fmdmmr);
                if (enable)
                    tmpReg |= DMA_MODE_ECC;
                else
                    tmpReg &= ~DMA_MODE_ECC;
                WRITE_UINT32(p_Fm->p_FmDmaRegs->fmdmmr, tmpReg);
                break;
             case (e_FM_EX_FPM_STALL_ON_TASKS):
                tmpReg = GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_ee);
                if (enable)
                    tmpReg |= FPM_EV_MASK_STALL_EN;
                else
                    tmpReg &= ~FPM_EV_MASK_STALL_EN;
                WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_ee, tmpReg);
                break;
             case (e_FM_EX_FPM_SINGLE_ECC):
                tmpReg = GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_ee);
                if (enable)
                    tmpReg |= FPM_EV_MASK_SINGLE_ECC_EN;
                else
                    tmpReg &= ~FPM_EV_MASK_SINGLE_ECC_EN;
                WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_ee, tmpReg);
                break;
            case ( e_FM_EX_FPM_DOUBLE_ECC):
                tmpReg = GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_ee);
                if (enable)
                    tmpReg |= FPM_EV_MASK_DOUBLE_ECC_EN;
                else
                    tmpReg &= ~FPM_EV_MASK_DOUBLE_ECC_EN;
                WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_ee, tmpReg);
                break;
            case ( e_FM_EX_QMI_SINGLE_ECC):
#if defined(FM_QMI_NO_ECC_EXCEPTIONS) || defined(FM_QMI_NO_SINGLE_ECC_EXCEPTION)
               if ((p_Fm->p_FmStateStruct->revInfo.majorRev == 4) ||
                   (p_Fm->p_FmStateStruct->revInfo.majorRev >= 6))
                {
                   REPORT_ERROR(MINOR, E_NOT_SUPPORTED, ("e_FM_EX_QMI_SINGLE_ECC"));
                   return E_OK;
                }
#endif   /* FM_QMI_NO_ECC_EXCEPTIONS */
                tmpReg = GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_ien);
                if (enable)
                    tmpReg |= QMI_INTR_EN_SINGLE_ECC;
                else
                    tmpReg &= ~QMI_INTR_EN_SINGLE_ECC;
                WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_ien, tmpReg);
                break;
             case (e_FM_EX_QMI_DOUBLE_ECC):
#ifdef FM_QMI_NO_ECC_EXCEPTIONS
                if (p_Fm->p_FmStateStruct->revInfo.majorRev == 4)
                {
                   REPORT_ERROR(MINOR, E_NOT_SUPPORTED, ("e_FM_EX_QMI_DOUBLE_ECC"));
                   return E_OK;
                }
#endif   /* FM_QMI_NO_ECC_EXCEPTIONS */
                tmpReg = GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_eien);
                if (enable)
                    tmpReg |= QMI_ERR_INTR_EN_DOUBLE_ECC;
                else
                    tmpReg &= ~QMI_ERR_INTR_EN_DOUBLE_ECC;
                WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_eien, tmpReg);
                break;
             case (e_FM_EX_QMI_DEQ_FROM_UNKNOWN_PORTID):
                tmpReg = GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_eien);
                if (enable)
                    tmpReg |= QMI_ERR_INTR_EN_DEQ_FROM_DEF;
                else
                    tmpReg &= ~QMI_ERR_INTR_EN_DEQ_FROM_DEF;
                WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_eien, tmpReg);
                break;
             case (e_FM_EX_BMI_LIST_RAM_ECC):
                tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_ier);
                if (enable)
                    tmpReg |= BMI_ERR_INTR_EN_LIST_RAM_ECC;
                else
                    tmpReg &= ~BMI_ERR_INTR_EN_LIST_RAM_ECC;
                WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_ier, tmpReg);
                break;
             case (e_FM_EX_BMI_STORAGE_PROFILE_ECC):
                tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_ier);
                if (enable)
                    tmpReg |= BMI_ERR_INTR_EN_STORAGE_PROFILE_ECC;
                else
                    tmpReg &= ~BMI_ERR_INTR_EN_STORAGE_PROFILE_ECC;
                WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_ier, tmpReg);
                break;
             case (e_FM_EX_BMI_STATISTICS_RAM_ECC):
                tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_ier);
                if (enable)
                    tmpReg |= BMI_ERR_INTR_EN_STATISTICS_RAM_ECC;
                else
                    tmpReg &= ~BMI_ERR_INTR_EN_STATISTICS_RAM_ECC;
                WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_ier, tmpReg);
                break;
             case (e_FM_EX_BMI_DISPATCH_RAM_ECC):
               tmpReg = GET_UINT32(p_Fm->p_FmBmiRegs->fmbm_ier);
               if (enable)
               {
#ifdef FM_NO_DISPATCH_RAM_ECC
                   if (p_Fm->p_FmStateStruct->revInfo.majorRev != 4)
                   {
                       REPORT_ERROR(MINOR, E_NOT_SUPPORTED, ("e_FM_EX_BMI_DISPATCH_RAM_ECC"));
                       return E_OK;
                   }
#endif /* FM_NO_DISPATCH_RAM_ECC */
                   tmpReg |= BMI_ERR_INTR_EN_DISPATCH_RAM_ECC;
               }
               else
                   tmpReg &= ~BMI_ERR_INTR_EN_DISPATCH_RAM_ECC;
               WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_ier, tmpReg);
               break;
             case (e_FM_EX_IRAM_ECC):
                 tmpReg = GET_UINT32(p_Fm->p_FmFpmRegs->fm_rie);
                if (enable)
                {
                    /* enable ECC if not enabled */
                    FmEnableRamsEcc(p_Fm);
                    /* enable ECC interrupts */
                    tmpReg |= FPM_IRAM_ECC_ERR_EX_EN;
                }
                else
                {
                    /* ECC mechanism may be disabled, depending on driver status  */
                    FmDisableRamsEcc(p_Fm);
                    tmpReg &= ~FPM_IRAM_ECC_ERR_EX_EN;
                }
                WRITE_UINT32(p_Fm->p_FmFpmRegs->fm_rie, tmpReg);
                break;

             case (e_FM_EX_MURAM_ECC):
                tmpReg = GET_UINT32(p_Fm->p_FmFpmRegs->fm_rie);
                if (enable)
                {
                    /* enable ECC if not enabled */
                    FmEnableRamsEcc(p_Fm);
                    /* enable ECC interrupts */
                    tmpReg |= FPM_MURAM_ECC_ERR_EX_EN;
                }
                else
                {
                    /* ECC mechanism may be disabled, depending on driver status  */
                    FmDisableRamsEcc(p_Fm);
                    tmpReg &= ~FPM_MURAM_ECC_ERR_EX_EN;
                }

                WRITE_UINT32(p_Fm->p_FmFpmRegs->fm_rie, tmpReg);
                break;
            default:
                RETURN_ERROR(MINOR, E_INVALID_SELECTION, NO_MSG);
        }
    }
    else
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Undefined exception"));

    return E_OK;
}

t_Error FM_GetRevision(t_Handle h_Fm, t_FmRevisionInfo *p_FmRevisionInfo)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    p_FmRevisionInfo->majorRev = p_Fm->p_FmStateStruct->revInfo.majorRev;
    p_FmRevisionInfo->minorRev = p_Fm->p_FmStateStruct->revInfo.minorRev;

    return E_OK;
}

t_Error FM_GetFmanCtrlCodeRevision(t_Handle h_Fm, t_FmCtrlCodeRevisionInfo *p_RevisionInfo)
{
    t_Fm                            *p_Fm = (t_Fm*)h_Fm;
    t_FMIramRegs                    *p_Iram;
    uint32_t                        revInfo;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_RevisionInfo, E_NULL_POINTER);

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        p_Fm->h_IpcSessions[0])
    {
        t_Error                         err;
        t_FmIpcMsg                      msg;
        t_FmIpcReply                    reply;
        uint32_t                        replyLength;
        t_FmIpcFmanCtrlCodeRevisionInfo ipcRevInfo;

        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        msg.msgId = FM_GET_FMAN_CTRL_CODE_REV;
        replyLength = sizeof(uint32_t) + sizeof(t_FmCtrlCodeRevisionInfo);
        if ((err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                     (uint8_t*)&msg,
                                     sizeof(msg.msgId),
                                     (uint8_t*)&reply,
                                     &replyLength,
                                     NULL,
                                     NULL)) != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
        if (replyLength != (sizeof(uint32_t) + sizeof(t_FmCtrlCodeRevisionInfo)))
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("IPC reply length mismatch"));
        memcpy((uint8_t*)&ipcRevInfo, reply.replyBody, sizeof(t_FmCtrlCodeRevisionInfo));
        p_RevisionInfo->packageRev = ipcRevInfo.packageRev;
        p_RevisionInfo->majorRev = ipcRevInfo.majorRev;
        p_RevisionInfo->minorRev = ipcRevInfo.minorRev;
        return (t_Error)(reply.error);
    }
    else if (p_Fm->guestId != NCSW_MASTER_ID)
        RETURN_ERROR(MINOR, E_NOT_SUPPORTED,
                     ("running in guest-mode without IPC!"));

    p_Iram = (t_FMIramRegs *)UINT_TO_PTR(p_Fm->baseAddr + FM_MM_IMEM);
    WRITE_UINT32(p_Iram->iadd, 0x4);
    while (GET_UINT32(p_Iram->iadd) != 0x4) ;
    revInfo = GET_UINT32(p_Iram->idata);
    p_RevisionInfo->packageRev = (uint16_t)((revInfo & 0xFFFF0000) >> 16);
    p_RevisionInfo->majorRev = (uint8_t)((revInfo & 0x0000FF00) >> 8);
    p_RevisionInfo->minorRev = (uint8_t)(revInfo & 0x000000FF);

    return E_OK;
}

uint32_t FM_GetCounter(t_Handle h_Fm, e_FmCounters counter)
{
    t_Fm        *p_Fm = (t_Fm*)h_Fm;
    t_Error     err;
    uint32_t    counterValue;

    SANITY_CHECK_RETURN_VALUE(p_Fm, E_INVALID_HANDLE, 0);
    SANITY_CHECK_RETURN_VALUE(!p_Fm->p_FmDriverParam, E_INVALID_STATE, 0);

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        !p_Fm->baseAddr &&
        p_Fm->h_IpcSessions[0])
    {
        t_FmIpcMsg          msg;
        t_FmIpcReply        reply;
        uint32_t            replyLength, outCounter;

        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        msg.msgId = FM_GET_COUNTER;
        memcpy(msg.msgBody, (uint8_t *)&counter, sizeof(uint32_t));
        replyLength = sizeof(uint32_t) + sizeof(uint32_t);
        err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                     (uint8_t*)&msg,
                                     sizeof(msg.msgId) +sizeof(counterValue),
                                     (uint8_t*)&reply,
                                     &replyLength,
                                     NULL,
                                     NULL);
        if (err != E_OK)
        {
            REPORT_ERROR(MAJOR, err, NO_MSG);
            return 0;
        }
        if (replyLength != (sizeof(uint32_t) + sizeof(uint32_t)))
        {
            REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("IPC reply length mismatch"));
            return 0;
        }

        memcpy((uint8_t*)&outCounter, reply.replyBody, sizeof(uint32_t));
        return outCounter;
    }
    else if (!p_Fm->baseAddr)
    {
        REPORT_ERROR(MAJOR, E_NOT_SUPPORTED, ("Either IPC or 'baseAddress' is required!"));
        return 0;
    }

    /* When applicable (when there is an 'enable counters' bit,
    check that counters are enabled */
    switch (counter)
    {
        case (e_FM_COUNTERS_DEQ_1):
        case (e_FM_COUNTERS_DEQ_2):
        case (e_FM_COUNTERS_DEQ_3):
            if (p_Fm->p_FmStateStruct->revInfo.majorRev >= 6)
            {
                REPORT_ERROR(MAJOR, E_NOT_SUPPORTED, ("Requested counter not supported"));
                return 0;
            }
        case (e_FM_COUNTERS_ENQ_TOTAL_FRAME):
        case (e_FM_COUNTERS_DEQ_TOTAL_FRAME):
        case (e_FM_COUNTERS_DEQ_0):
        case (e_FM_COUNTERS_DEQ_FROM_DEFAULT):
        case (e_FM_COUNTERS_DEQ_FROM_CONTEXT):
        case (e_FM_COUNTERS_DEQ_FROM_FD):
        case (e_FM_COUNTERS_DEQ_CONFIRM):
            if (!(GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_gc) & QMI_CFG_EN_COUNTERS))
            {
                REPORT_ERROR(MAJOR, E_INVALID_STATE, ("Requested counter was not enabled"));
                return 0;
            }
            break;
        default:
            break;
    }

    switch (counter)
    {
        case (e_FM_COUNTERS_ENQ_TOTAL_FRAME):
            return GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_etfc);
        case (e_FM_COUNTERS_DEQ_TOTAL_FRAME):
            return GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_dtfc);
        case (e_FM_COUNTERS_DEQ_0):
            return GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_dc0);
        case (e_FM_COUNTERS_DEQ_1):
            return GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_dc1);
        case (e_FM_COUNTERS_DEQ_2):
            return GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_dc2);
        case (e_FM_COUNTERS_DEQ_3):
            return GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_dc3);
        case (e_FM_COUNTERS_DEQ_FROM_DEFAULT):
            return GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_dfdc);
        case (e_FM_COUNTERS_DEQ_FROM_CONTEXT):
            return GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_dfcc);
        case (e_FM_COUNTERS_DEQ_FROM_FD):
            return GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_dffc);
        case (e_FM_COUNTERS_DEQ_CONFIRM):
            return GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_dcc);
            break;
    }
    /* should never get here */
    ASSERT_COND(FALSE);
    return 0;
}

t_Error FM_ModifyCounter(t_Handle h_Fm, e_FmCounters counter, uint32_t val)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Fm->p_FmDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    /* When applicable (when there is an 'enable counters' bit,
    check that counters are enabled */
    switch (counter)
    {
        case (e_FM_COUNTERS_DEQ_1):
        case (e_FM_COUNTERS_DEQ_2):
        case (e_FM_COUNTERS_DEQ_3):
             if (p_Fm->p_FmStateStruct->revInfo.majorRev >= 6)
                 RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("Requested counter not supported"));
        case (e_FM_COUNTERS_ENQ_TOTAL_FRAME):
        case (e_FM_COUNTERS_DEQ_TOTAL_FRAME):
        case (e_FM_COUNTERS_DEQ_0):
        case (e_FM_COUNTERS_DEQ_FROM_DEFAULT):
        case (e_FM_COUNTERS_DEQ_FROM_CONTEXT):
        case (e_FM_COUNTERS_DEQ_FROM_FD):
        case (e_FM_COUNTERS_DEQ_CONFIRM):
            if (!(GET_UINT32(p_Fm->p_FmQmiRegs->fmqm_gc) & QMI_CFG_EN_COUNTERS))
                RETURN_ERROR(MINOR, E_INVALID_STATE, ("Requested counter was not enabled"));
            break;
        default:
            break;
    }

    /* Set counter */
    switch (counter)
    {
        case (e_FM_COUNTERS_ENQ_TOTAL_FRAME):
            WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_etfc, val);
            break;
        case (e_FM_COUNTERS_DEQ_TOTAL_FRAME):
            WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_dtfc, val);
            break;
        case (e_FM_COUNTERS_DEQ_0):
            WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_dc0, val);
            break;
        case (e_FM_COUNTERS_DEQ_1):
            WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_dc1, val);
            break;
        case (e_FM_COUNTERS_DEQ_2):
            WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_dc2, val);
            break;
        case (e_FM_COUNTERS_DEQ_3):
            WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_dc3, val);
            break;
        case (e_FM_COUNTERS_DEQ_FROM_DEFAULT):
            WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_dfdc, val);
            break;
        case (e_FM_COUNTERS_DEQ_FROM_CONTEXT):
            WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_dfcc, val);
            break;
        case (e_FM_COUNTERS_DEQ_FROM_FD):
            WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_dffc, val);
            break;
        case (e_FM_COUNTERS_DEQ_CONFIRM):
            WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_dcc, val);
            break;
        default:
            break;
    }

    return E_OK;
}

void FM_SetDmaEmergency(t_Handle h_Fm, e_FmDmaMuramPort muramPort, bool enable)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;
    uint32_t    bitMask;

    SANITY_CHECK_RETURN(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN(!p_Fm->p_FmDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    bitMask = (uint32_t)((muramPort==e_FM_DMA_MURAM_PORT_WRITE) ? DMA_MODE_EMERGENCY_WRITE : DMA_MODE_EMERGENCY_READ);

    if (enable)
        WRITE_UINT32(p_Fm->p_FmDmaRegs->fmdmmr, GET_UINT32(p_Fm->p_FmDmaRegs->fmdmmr) | bitMask);
    else /* disable */
        WRITE_UINT32(p_Fm->p_FmDmaRegs->fmdmmr, GET_UINT32(p_Fm->p_FmDmaRegs->fmdmmr) & ~bitMask);

    return;
}

void FM_SetDmaExtBusPri(t_Handle h_Fm, e_FmDmaExtBusPri pri)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN(!p_Fm->p_FmDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    WRITE_UINT32(p_Fm->p_FmDmaRegs->fmdmmr, GET_UINT32(p_Fm->p_FmDmaRegs->fmdmmr) | ((uint32_t)pri << DMA_MODE_BUS_PRI_SHIFT) );

    return;
}

void FM_GetDmaStatus(t_Handle h_Fm, t_FmDmaStatus *p_FmDmaStatus)
{
    t_Fm                *p_Fm = (t_Fm*)h_Fm;
    uint32_t            tmpReg;

    SANITY_CHECK_RETURN(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN(!p_Fm->p_FmDriverParam, E_INVALID_STATE);

    if ((p_Fm->guestId != NCSW_MASTER_ID) &&
        !p_Fm->baseAddr &&
        p_Fm->h_IpcSessions[0])
    {
        t_FmIpcDmaStatus    ipcDmaStatus;
        t_FmIpcMsg          msg;
        t_FmIpcReply        reply;
        t_Error             err;
        uint32_t            replyLength;

        memset(&msg, 0, sizeof(msg));
        memset(&reply, 0, sizeof(reply));
        msg.msgId = FM_DMA_STAT;
        replyLength = sizeof(uint32_t) + sizeof(t_FmIpcDmaStatus);
        err = XX_IpcSendMessage(p_Fm->h_IpcSessions[0],
                                (uint8_t*)&msg,
                                sizeof(msg.msgId),
                                (uint8_t*)&reply,
                                &replyLength,
                                NULL,
                                NULL);
        if (err != E_OK)
        {
            REPORT_ERROR(MINOR, err, NO_MSG);
            return;
        }
        if (replyLength != (sizeof(uint32_t) + sizeof(t_FmIpcDmaStatus)))
        {
            REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("IPC reply length mismatch"));
            return;
        }
        memcpy((uint8_t*)&ipcDmaStatus, reply.replyBody, sizeof(t_FmIpcDmaStatus));

        p_FmDmaStatus->cmqNotEmpty = (bool)ipcDmaStatus.boolCmqNotEmpty;            /**< Command queue is not empty */
        p_FmDmaStatus->busError = (bool)ipcDmaStatus.boolBusError;                  /**< Bus error occurred */
        p_FmDmaStatus->readBufEccError = (bool)ipcDmaStatus.boolReadBufEccError;        /**< Double ECC error on buffer Read */
        p_FmDmaStatus->writeBufEccSysError =(bool)ipcDmaStatus.boolWriteBufEccSysError;    /**< Double ECC error on buffer write from system side */
        p_FmDmaStatus->writeBufEccFmError = (bool)ipcDmaStatus.boolWriteBufEccFmError;     /**< Double ECC error on buffer write from FM side */
        p_FmDmaStatus->singlePortEccError = (bool)ipcDmaStatus.boolSinglePortEccError;     /**< Double ECC error on buffer write from FM side */
        return;
    }
    else if (!p_Fm->baseAddr)
    {
        REPORT_ERROR(MINOR, E_NOT_SUPPORTED,
                     ("Either IPC or 'baseAddress' is required!"));
        return;
    }

    tmpReg = GET_UINT32(p_Fm->p_FmDmaRegs->fmdmsr);

    p_FmDmaStatus->cmqNotEmpty = (bool)(tmpReg & DMA_STATUS_CMD_QUEUE_NOT_EMPTY);
    p_FmDmaStatus->busError = (bool)(tmpReg & DMA_STATUS_BUS_ERR);
    if (p_Fm->p_FmStateStruct->revInfo.majorRev >= 6)
        p_FmDmaStatus->singlePortEccError = (bool)(tmpReg & DMA_STATUS_FM_SPDAT_ECC);
    else
    {
        p_FmDmaStatus->readBufEccError = (bool)(tmpReg & DMA_STATUS_READ_ECC);
        p_FmDmaStatus->writeBufEccSysError = (bool)(tmpReg & DMA_STATUS_SYSTEM_WRITE_ECC);
        p_FmDmaStatus->writeBufEccFmError = (bool)(tmpReg & DMA_STATUS_FM_WRITE_ECC);
    }
}

void FM_Resume(t_Handle h_Fm)
{
    t_Fm            *p_Fm = (t_Fm*)h_Fm;
    uint32_t        tmpReg;

    SANITY_CHECK_RETURN(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN(!p_Fm->p_FmDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    tmpReg  = GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_ee);
    /* clear tmpReg event bits in order not to clear standing events */
    tmpReg &= ~(FPM_EV_MASK_DOUBLE_ECC | FPM_EV_MASK_STALL | FPM_EV_MASK_SINGLE_ECC);
    WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_ee, tmpReg | FPM_EV_MASK_RELEASE_FM);
}

t_Error FM_GetSpecialOperationCoding(t_Handle               h_Fm,
                                     fmSpecialOperations_t  spOper,
                                     uint8_t                *p_SpOperCoding)
{
    t_Fm                        *p_Fm = (t_Fm*)h_Fm;
    t_FmCtrlCodeRevisionInfo    revInfo;
    t_Error                     err;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Fm->p_FmDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(p_SpOperCoding, E_NULL_POINTER);

    if (!spOper)
    {
        *p_SpOperCoding = 0;
        return E_OK;
    }

    if ((err = FM_GetFmanCtrlCodeRevision(p_Fm, &revInfo)) != E_OK)
    {
        DBG(WARNING, ("FM in guest-mode without IPC, can't validate firmware revision."));
        revInfo.packageRev = IP_OFFLOAD_PACKAGE_NUMBER;
    }
    else if (revInfo.packageRev != IP_OFFLOAD_PACKAGE_NUMBER)
        RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("Fman ctrl code package"));

    if (revInfo.packageRev == IP_OFFLOAD_PACKAGE_NUMBER)
    {
        switch (spOper)
        {
            case (FM_SP_OP_IPSEC|FM_SP_OP_IPSEC_UPDATE_UDP_LEN|FM_SP_OP_IPSEC_MANIP):
            case (FM_SP_OP_IPSEC|FM_SP_OP_IPSEC_UPDATE_UDP_LEN|FM_SP_OP_IPSEC_MANIP|FM_SP_OP_RPD):
                    *p_SpOperCoding = 5;
                    break;
            case (FM_SP_OP_IPSEC|FM_SP_OP_IPSEC_MANIP):
            case (FM_SP_OP_IPSEC|FM_SP_OP_IPSEC_MANIP|FM_SP_OP_RPD):
                    *p_SpOperCoding = 6;
                    break;
            case (FM_SP_OP_IPSEC|FM_SP_OP_IPSEC_UPDATE_UDP_LEN|FM_SP_OP_RPD):
                    *p_SpOperCoding = 3;
                    break;
            case (FM_SP_OP_IPSEC|FM_SP_OP_IPSEC_UPDATE_UDP_LEN):
                    *p_SpOperCoding = 1;
                    break;
            case (FM_SP_OP_IPSEC|FM_SP_OP_RPD):
                    *p_SpOperCoding = 4;
                    break;
            case (FM_SP_OP_IPSEC):
                    *p_SpOperCoding = 2;
                    break;
            case (FM_SP_OP_DCL4C):
                    *p_SpOperCoding = 7;
                    break;
            case (FM_SP_OP_CLEAR_RPD):
                    *p_SpOperCoding = 8;
                    break;
            default:
                RETURN_ERROR(MINOR, E_INVALID_VALUE, NO_MSG);
        }
    }
    return E_OK;
}

t_Error FM_CtrlMonStart(t_Handle h_Fm)
{
    t_Fm            *p_Fm = (t_Fm *)h_Fm;
    t_FmTrbRegs     *p_MonRegs;
    uint8_t         i;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Fm->p_FmDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_brkc,
                 GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_brkc) | FPM_BRKC_RDBG);

    for (i = 0; i < FM_NUM_OF_CTRL; i++)
    {
        p_MonRegs = (t_FmTrbRegs *)UINT_TO_PTR(p_Fm->baseAddr + FM_MM_TRB(i));

        /* Reset control registers */
        WRITE_UINT32(p_MonRegs->tcrh, TRB_TCRH_RESET);
        WRITE_UINT32(p_MonRegs->tcrl, TRB_TCRL_RESET);

        /* Configure: counter #1 counts all stalls in risc - ldsched stall
                      counter #2 counts all stalls in risc - other stall*/
        WRITE_UINT32(p_MonRegs->tcrl, TRB_TCRL_RESET | TRB_TCRL_UTIL);

        /* Enable monitoring */
        WRITE_UINT32(p_MonRegs->tcrh, TRB_TCRH_ENABLE_COUNTERS);
    }

    return E_OK;
}

t_Error FM_CtrlMonStop(t_Handle h_Fm)
{
    t_Fm            *p_Fm = (t_Fm *)h_Fm;
    t_FmTrbRegs     *p_MonRegs;
    uint8_t         i;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Fm->p_FmDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);

    for (i = 0; i < FM_NUM_OF_CTRL; i++)
    {
        p_MonRegs = (t_FmTrbRegs *)UINT_TO_PTR(p_Fm->baseAddr + FM_MM_TRB(i));
        WRITE_UINT32(p_MonRegs->tcrh, TRB_TCRH_DISABLE_COUNTERS);
    }

    WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_brkc,
                 GET_UINT32(p_Fm->p_FmFpmRegs->fmfp_brkc) & ~FPM_BRKC_RDBG);

    return E_OK;
}

t_Error FM_CtrlMonGetCounters(t_Handle h_Fm, uint8_t fmCtrlIndex, t_FmCtrlMon *p_Mon)
{
    t_Fm            *p_Fm = (t_Fm *)h_Fm;
    t_FmTrbRegs     *p_MonRegs;
    uint64_t        clkCnt, utilValue, effValue;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Fm->p_FmDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR((p_Fm->guestId == NCSW_MASTER_ID), E_NOT_SUPPORTED);
    SANITY_CHECK_RETURN_ERROR(p_Mon, E_NULL_POINTER);

    if (fmCtrlIndex >= FM_NUM_OF_CTRL)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("FM Controller index"));

    p_MonRegs = (t_FmTrbRegs *)UINT_TO_PTR(p_Fm->baseAddr + FM_MM_TRB(fmCtrlIndex));

    clkCnt = (uint64_t)
            ((uint64_t)GET_UINT32(p_MonRegs->tpcch) << 32 | GET_UINT32(p_MonRegs->tpccl));

    utilValue = (uint64_t)
            ((uint64_t)GET_UINT32(p_MonRegs->tpc1h) << 32 | GET_UINT32(p_MonRegs->tpc1l));

    effValue = (uint64_t)
            ((uint64_t)GET_UINT32(p_MonRegs->tpc2h) << 32 | GET_UINT32(p_MonRegs->tpc2l));

    p_Mon->percentCnt[0] = (uint8_t)((clkCnt - utilValue) * 100 / clkCnt);
    if (clkCnt != utilValue)
        p_Mon->percentCnt[1] = (uint8_t)(((clkCnt - utilValue) - effValue) * 100 / (clkCnt - utilValue));
    else
        p_Mon->percentCnt[1] = 0;

    return E_OK;
}

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
t_Error FM_DumpRegs(t_Handle h_Fm)
{
    t_Fm            *p_Fm = (t_Fm *)h_Fm;
    uint8_t         i,j = 0;

    DECLARE_DUMP;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Fm->p_FmDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(((p_Fm->guestId == NCSW_MASTER_ID) ||
                               p_Fm->baseAddr), E_INVALID_OPERATION);

    DUMP_SUBTITLE(("\n"));

    DUMP_TITLE(p_Fm->p_FmFpmRegs, ("FM-FPM Regs"));

    DUMP_VAR(p_Fm->p_FmFpmRegs,fmfp_tnc);
    DUMP_VAR(p_Fm->p_FmFpmRegs,fmfp_prc);
    DUMP_VAR(p_Fm->p_FmFpmRegs,fmfp_brkc);
    DUMP_VAR(p_Fm->p_FmFpmRegs,fmfp_mxd);
    DUMP_VAR(p_Fm->p_FmFpmRegs,fmfp_dis1);
    DUMP_VAR(p_Fm->p_FmFpmRegs,fmfp_dis2);
    DUMP_VAR(p_Fm->p_FmFpmRegs,fm_epi);
    DUMP_VAR(p_Fm->p_FmFpmRegs,fm_rie);

    DUMP_TITLE(&p_Fm->p_FmFpmRegs->fmfp_fcev, ("fmfp_fcev"));
    DUMP_SUBSTRUCT_ARRAY(i, 4)
    {
        DUMP_MEMORY(&p_Fm->p_FmFpmRegs->fmfp_fcev[i], sizeof(uint32_t));
    }

    DUMP_TITLE(&p_Fm->p_FmFpmRegs->fmfp_cee, ("fmfp_cee"));
    DUMP_SUBSTRUCT_ARRAY(i, 4)
    {
        DUMP_MEMORY(&p_Fm->p_FmFpmRegs->fmfp_cee[i], sizeof(uint32_t));
    }

    DUMP_SUBTITLE(("\n"));
    DUMP_VAR(p_Fm->p_FmFpmRegs,fmfp_tsc1);
    DUMP_VAR(p_Fm->p_FmFpmRegs,fmfp_tsc2);
    DUMP_VAR(p_Fm->p_FmFpmRegs,fmfp_tsp);
    DUMP_VAR(p_Fm->p_FmFpmRegs,fmfp_tsf);
    DUMP_VAR(p_Fm->p_FmFpmRegs,fm_rcr);
    DUMP_VAR(p_Fm->p_FmFpmRegs,fmfp_extc);
    DUMP_VAR(p_Fm->p_FmFpmRegs,fmfp_ext1);
    DUMP_VAR(p_Fm->p_FmFpmRegs,fmfp_ext2);

    DUMP_SUBTITLE(("\n"));
    WRITE_UINT32(p_Fm->p_FmFpmRegs->fmfp_dra, 0);
    CORE_MemoryBarrier();
    for (j=0; j<128; j++)
    {
        DUMP_TITLE(j, ("fmfp_dra"));
        DUMP_SUBSTRUCT_ARRAY(i, 4)
        {
            DUMP_MEMORY(&p_Fm->p_FmFpmRegs->fmfp_drd[i], sizeof(uint32_t));
        }
        DUMP_TITLE(j, ("fmfp_ts"));
        DUMP_MEMORY(&p_Fm->p_FmFpmRegs->fmfp_ts[j], sizeof(uint32_t));
    }

    DUMP_SUBTITLE(("\n"));
    DUMP_VAR(p_Fm->p_FmFpmRegs,fm_ip_rev_1);
    DUMP_VAR(p_Fm->p_FmFpmRegs,fm_ip_rev_2);
    DUMP_VAR(p_Fm->p_FmFpmRegs,fm_rstc);
    DUMP_VAR(p_Fm->p_FmFpmRegs,fm_cld);
    DUMP_VAR(p_Fm->p_FmFpmRegs,fm_npi);
    DUMP_VAR(p_Fm->p_FmFpmRegs,fmfp_ee);

    DUMP_TITLE(&p_Fm->p_FmFpmRegs->fmfp_cev, ("fmfp_cev"));
    DUMP_SUBSTRUCT_ARRAY(i, 4)
    {
        DUMP_MEMORY(&p_Fm->p_FmFpmRegs->fmfp_cev[i], sizeof(uint32_t));
    }

    DUMP_TITLE(&p_Fm->p_FmFpmRegs->fmfp_ps, ("fmfp_ps"));
    DUMP_SUBSTRUCT_ARRAY(i, 64)
    {
        DUMP_MEMORY(&p_Fm->p_FmFpmRegs->fmfp_ps[i], sizeof(uint32_t));
    }

    DUMP_TITLE(p_Fm->p_FmDmaRegs, ("FM-DMA Regs"));
    DUMP_VAR(p_Fm->p_FmDmaRegs,fmdmsr);
    DUMP_VAR(p_Fm->p_FmDmaRegs,fmdmemsr);
    DUMP_VAR(p_Fm->p_FmDmaRegs,fmdmmr);
    DUMP_VAR(p_Fm->p_FmDmaRegs,fmdmtr);
    DUMP_VAR(p_Fm->p_FmDmaRegs,fmdmhy);
    DUMP_VAR(p_Fm->p_FmDmaRegs,fmdmsetr);
    DUMP_VAR(p_Fm->p_FmDmaRegs,fmdmtah);
    DUMP_VAR(p_Fm->p_FmDmaRegs,fmdmtal);
    DUMP_VAR(p_Fm->p_FmDmaRegs,fmdmtcid);
    DUMP_VAR(p_Fm->p_FmDmaRegs,fmdmra);
    DUMP_VAR(p_Fm->p_FmDmaRegs,fmdmrd);
    DUMP_VAR(p_Fm->p_FmDmaRegs,fmdmwcr);
    DUMP_VAR(p_Fm->p_FmDmaRegs,fmdmebcr);
    DUMP_VAR(p_Fm->p_FmDmaRegs,fmdmdcr);

    DUMP_TITLE(&p_Fm->p_FmDmaRegs->fmdmplr, ("fmdmplr"));

    DUMP_SUBSTRUCT_ARRAY(i, FM_MAX_NUM_OF_HW_PORT_IDS/2)
    {
        DUMP_MEMORY(&p_Fm->p_FmDmaRegs->fmdmplr[i], sizeof(uint32_t));
    }

    DUMP_TITLE(p_Fm->p_FmBmiRegs, ("FM-BMI COMMON Regs"));
    DUMP_VAR(p_Fm->p_FmBmiRegs,fmbm_init);
    DUMP_VAR(p_Fm->p_FmBmiRegs,fmbm_cfg1);
    DUMP_VAR(p_Fm->p_FmBmiRegs,fmbm_cfg2);
    DUMP_VAR(p_Fm->p_FmBmiRegs,fmbm_ievr);
    DUMP_VAR(p_Fm->p_FmBmiRegs,fmbm_ier);

    DUMP_TITLE(&p_Fm->p_FmBmiRegs->fmbm_arb, ("fmbm_arb"));
    DUMP_SUBSTRUCT_ARRAY(i, 8)
    {
        DUMP_MEMORY(&p_Fm->p_FmBmiRegs->fmbm_arb[i], sizeof(uint32_t));
    }

    DUMP_TITLE(p_Fm->p_FmQmiRegs, ("FM-QMI COMMON Regs"));
    DUMP_VAR(p_Fm->p_FmQmiRegs,fmqm_gc);
    DUMP_VAR(p_Fm->p_FmQmiRegs,fmqm_eie);
    DUMP_VAR(p_Fm->p_FmQmiRegs,fmqm_eien);
    DUMP_VAR(p_Fm->p_FmQmiRegs,fmqm_eif);
    DUMP_VAR(p_Fm->p_FmQmiRegs,fmqm_ie);
    DUMP_VAR(p_Fm->p_FmQmiRegs,fmqm_ien);
    DUMP_VAR(p_Fm->p_FmQmiRegs,fmqm_if);
    DUMP_VAR(p_Fm->p_FmQmiRegs,fmqm_gs);
    DUMP_VAR(p_Fm->p_FmQmiRegs,fmqm_etfc);

    return E_OK;
}
#endif /* (defined(DEBUG_ERRORS) && ... */



/****************************************************/
/*       Hidden-DEBUG Only API                      */
/****************************************************/

t_Error FM_ForceIntr (t_Handle h_Fm, e_FmExceptions exception)
{
    t_Fm *p_Fm = (t_Fm*)h_Fm;

    SANITY_CHECK_RETURN_ERROR(p_Fm, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Fm->p_FmDriverParam, E_INVALID_STATE);

    switch (exception)
    {
        case e_FM_EX_QMI_DEQ_FROM_UNKNOWN_PORTID:
            if (!(p_Fm->p_FmStateStruct->exceptions & FM_EX_QMI_DEQ_FROM_UNKNOWN_PORTID))
                RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("The selected exception is masked"));
            WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_eif, QMI_ERR_INTR_EN_DEQ_FROM_DEF);
            break;
        case e_FM_EX_QMI_SINGLE_ECC:
            if (p_Fm->p_FmStateStruct->revInfo.majorRev >= 6)
                 RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("e_FM_EX_QMI_SINGLE_ECC not supported on this integration."));

            if (!(p_Fm->p_FmStateStruct->exceptions & FM_EX_QMI_SINGLE_ECC))
                RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("The selected exception is masked"));
            WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_if, QMI_INTR_EN_SINGLE_ECC);
            break;
        case e_FM_EX_QMI_DOUBLE_ECC:
            if (!(p_Fm->p_FmStateStruct->exceptions & FM_EX_QMI_DOUBLE_ECC))
                RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("The selected exception is masked"));
            WRITE_UINT32(p_Fm->p_FmQmiRegs->fmqm_eif, QMI_ERR_INTR_EN_DOUBLE_ECC);
            break;
        case e_FM_EX_BMI_LIST_RAM_ECC:
            if (!(p_Fm->p_FmStateStruct->exceptions & FM_EX_BMI_LIST_RAM_ECC))
                RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("The selected exception is masked"));
            WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_ifr, BMI_ERR_INTR_EN_LIST_RAM_ECC);
            break;
        case e_FM_EX_BMI_STORAGE_PROFILE_ECC:
            if (!(p_Fm->p_FmStateStruct->exceptions & FM_EX_BMI_STORAGE_PROFILE_ECC))
                RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("The selected exception is masked"));
            WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_ifr, BMI_ERR_INTR_EN_STORAGE_PROFILE_ECC);
            break;
        case e_FM_EX_BMI_STATISTICS_RAM_ECC:
            if (!(p_Fm->p_FmStateStruct->exceptions & FM_EX_BMI_STATISTICS_RAM_ECC))
                RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("The selected exception is masked"));
            WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_ifr, BMI_ERR_INTR_EN_STATISTICS_RAM_ECC);
            break;
        case e_FM_EX_BMI_DISPATCH_RAM_ECC:
            if (!(p_Fm->p_FmStateStruct->exceptions & FM_EX_BMI_DISPATCH_RAM_ECC))
                RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("The selected exception is masked"));
            WRITE_UINT32(p_Fm->p_FmBmiRegs->fmbm_ifr, BMI_ERR_INTR_EN_DISPATCH_RAM_ECC);
            break;
        default:
            RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("The selected exception may not be forced"));
    }

    return E_OK;
}
