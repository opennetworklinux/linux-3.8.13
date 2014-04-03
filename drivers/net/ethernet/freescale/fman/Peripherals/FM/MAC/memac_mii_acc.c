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


#include "error_ext.h"
#include "std_ext.h"
#include "fm_mac.h"
#include "memac.h"
#include "xx_ext.h"

#include "fm_common.h"


static void WritePhyReg10G(t_MemacMiiAccessMemMap   *p_MiiAccess,
                           uint8_t                  phyAddr,
                           uint8_t                  reg,
                           uint16_t                 data)
{
    uint32_t                tmpReg;

    tmpReg = GET_UINT32(p_MiiAccess->mdio_cfg);
    /* Leave only MDIO_CLK_DIV bits set on */
    tmpReg &= MDIO_CFG_CLK_DIV_MASK;
    /* Set maximum MDIO_HOLD value to allow phy to see
       change of data signal */
    tmpReg |= MDIO_CFG_HOLD_MASK;
    /* Add 10G interface mode */
    tmpReg |= MDIO_CFG_ENC45;
    WRITE_UINT32(p_MiiAccess->mdio_cfg, tmpReg);

    /* Wait for command completion */
    while ((GET_UINT32(p_MiiAccess->mdio_cfg)) & MDIO_CFG_BSY)
        XX_UDelay(1);

    /* Specify phy and register to be accessed */
    WRITE_UINT32(p_MiiAccess->mdio_ctrl, phyAddr);
    WRITE_UINT32(p_MiiAccess->mdio_addr, reg);
    CORE_MemoryBarrier();

    while ((GET_UINT32(p_MiiAccess->mdio_cfg)) & MDIO_CFG_BSY)
        XX_UDelay(1);

    /* Write data */
    WRITE_UINT32(p_MiiAccess->mdio_data, data);
    CORE_MemoryBarrier();

    /* Wait for write transaction end */
    while ((GET_UINT32(p_MiiAccess->mdio_data)) & MDIO_DATA_BSY)
        XX_UDelay(1);
}

static uint32_t ReadPhyReg10G(t_MemacMiiAccessMemMap *p_MiiAccess,
                              uint8_t                phyAddr,
                              uint8_t                reg,
                              uint16_t               *p_Data)
{
    uint32_t                tmpReg;

    tmpReg = GET_UINT32(p_MiiAccess->mdio_cfg);
    /* Leave only MDIO_CLK_DIV bits set on */
    tmpReg &= MDIO_CFG_CLK_DIV_MASK;
    /* Set maximum MDIO_HOLD value to allow phy to see
       change of data signal */
    tmpReg |= MDIO_CFG_HOLD_MASK;
    /* Add 10G interface mode */
    tmpReg |= MDIO_CFG_ENC45;
    WRITE_UINT32(p_MiiAccess->mdio_cfg, tmpReg);

    /* Wait for command completion */
    while ((GET_UINT32(p_MiiAccess->mdio_cfg)) & MDIO_CFG_BSY)
        XX_UDelay(1);

    /* Specify phy and register to be accessed */
    WRITE_UINT32(p_MiiAccess->mdio_ctrl, phyAddr);
    WRITE_UINT32(p_MiiAccess->mdio_addr, reg);
    CORE_MemoryBarrier();

    while ((GET_UINT32(p_MiiAccess->mdio_cfg)) & MDIO_CFG_BSY)
        XX_UDelay(1);

    /* Read cycle */
    tmpReg = phyAddr;
    tmpReg |= MDIO_CTL_READ;
    WRITE_UINT32(p_MiiAccess->mdio_ctrl, tmpReg);
    CORE_MemoryBarrier();

    /* Wait for data to be available */
    while ((GET_UINT32(p_MiiAccess->mdio_data)) & MDIO_DATA_BSY)
        XX_UDelay(1);

    *p_Data =  (uint16_t)GET_UINT32(p_MiiAccess->mdio_data);

    /* Check if there was an error */
    return GET_UINT32(p_MiiAccess->mdio_cfg);
}

static void WritePhyReg1G(t_MemacMiiAccessMemMap    *p_MiiAccess,
                          uint8_t                   phyAddr,
                          uint8_t                   reg,
                          uint16_t                  data)
{
    uint32_t                tmpReg;

    /* Leave only MDIO_CLK_DIV and MDIO_HOLD bits set on */
    tmpReg = GET_UINT32(p_MiiAccess->mdio_cfg);
    tmpReg &= (MDIO_CFG_CLK_DIV_MASK | MDIO_CFG_HOLD_MASK);
    WRITE_UINT32(p_MiiAccess->mdio_cfg, tmpReg);

    /* Wait for command completion */
    while ((GET_UINT32(p_MiiAccess->mdio_cfg)) & MDIO_CFG_BSY)
        XX_UDelay(1);

    /* Write transaction */
    tmpReg = (phyAddr << MDIO_CTL_PHY_ADDR_SHIFT);
    tmpReg |= reg;
    WRITE_UINT32(p_MiiAccess->mdio_ctrl, tmpReg);

    while ((GET_UINT32(p_MiiAccess->mdio_cfg)) & MDIO_CFG_BSY)
        XX_UDelay(1);

    WRITE_UINT32(p_MiiAccess->mdio_data, data);
    CORE_MemoryBarrier();

    /* Wait for write transaction to end */
    while ((GET_UINT32(p_MiiAccess->mdio_data)) & MDIO_DATA_BSY)
        XX_UDelay(1);
}

static uint32_t ReadPhyReg1G(t_MemacMiiAccessMemMap  *p_MiiAccess,
                             uint8_t                 phyAddr,
                             uint8_t                 reg,
                             uint16_t                *p_Data)
{
    uint32_t                tmpReg;

    /* Leave only MDIO_CLK_DIV and MDIO_HOLD bits set on */
    tmpReg = GET_UINT32(p_MiiAccess->mdio_cfg);
    tmpReg &= (MDIO_CFG_CLK_DIV_MASK | MDIO_CFG_HOLD_MASK);
    WRITE_UINT32(p_MiiAccess->mdio_cfg, tmpReg);

    /* Wait for command completion */
    while ((GET_UINT32(p_MiiAccess->mdio_cfg)) & MDIO_CFG_BSY)
        XX_UDelay(1);

    /* Read transaction */
    tmpReg = (phyAddr << MDIO_CTL_PHY_ADDR_SHIFT);
    tmpReg |= reg;
    tmpReg |= MDIO_CTL_READ;
    WRITE_UINT32(p_MiiAccess->mdio_ctrl, tmpReg);

    while ((GET_UINT32(p_MiiAccess->mdio_cfg)) & MDIO_CFG_BSY)
        XX_UDelay(1);

    /* Wait for data to be available */
    while ((GET_UINT32(p_MiiAccess->mdio_data)) & MDIO_DATA_BSY)
        XX_UDelay(1);

    *p_Data =  (uint16_t)GET_UINT32(p_MiiAccess->mdio_data);

    /* Check error */
    return GET_UINT32(p_MiiAccess->mdio_cfg);
}

/*****************************************************************************/
t_Error MEMAC_MII_WritePhyReg(t_Handle  h_Memac,
                             uint8_t    phyAddr,
                             uint8_t    reg,
                             uint16_t   data)
{
    t_Memac                 *p_Memac = (t_Memac *)h_Memac;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MiiMemMap, E_INVALID_HANDLE);

    /* Figure out interface type - 10G vs 1G.
       In 10G interface both phyAddr and devAddr present. */
    if (ENET_SPEED_FROM_MODE(p_Memac->enetMode) == e_ENET_SPEED_10000)
        WritePhyReg10G(p_Memac->p_MiiMemMap, phyAddr, reg, data);
    else
        WritePhyReg1G(p_Memac->p_MiiMemMap, phyAddr, reg, data);

    return E_OK;
}

/*****************************************************************************/
t_Error MEMAC_MII_ReadPhyReg(t_Handle h_Memac,
                            uint8_t   phyAddr,
                            uint8_t   reg,
                            uint16_t  *p_Data)
{
    t_Memac                 *p_Memac = (t_Memac *)h_Memac;
    uint32_t                ans;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MiiMemMap, E_INVALID_HANDLE);

    /* Figure out interface type - 10G vs 1G.
       In 10G interface both phyAddr and devAddr present. */
    if (ENET_SPEED_FROM_MODE(p_Memac->enetMode) == e_ENET_SPEED_10000)
        ans = ReadPhyReg10G(p_Memac->p_MiiMemMap, phyAddr, reg, p_Data);
    else
        ans = ReadPhyReg1G(p_Memac->p_MiiMemMap, phyAddr, reg, p_Data);

    if (ans & MDIO_CFG_READ_ERR)
        RETURN_ERROR(MINOR, E_INVALID_VALUE,
                     ("Read Error: phyAddr 0x%x, dev 0x%x, reg 0x%x, cfgReg 0x%x",
                      ((phyAddr & 0xe0) >> 5), (phyAddr & 0x1f), reg, ans));

    return E_OK;
}
