#ifndef _REG_MODEMHP_H_
#define _REG_MODEMHP_H_

#include <stdint.h>
#include "_reg_modemhp.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_MODEMHP_COUNT 57

#define REG_MODEMHP_DECODING_MASK 0x000000FF

/**
 * @brief MDM_VERSION register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16                  REL   0x1
 *  15:08                  UPG   0x3
 *  07:00                BUILD   0x2
 * </pre>
 */
#define MDM_VERSION_MDMHP_MDM_VERSION_ADDR   0x21020000
#define MDM_VERSION_MDMHP_MDM_VERSION_OFFSET 0x00000000
#define MDM_VERSION_MDMHP_MDM_VERSION_INDEX  0x00000000
#define MDM_VERSION_MDMHP_MDM_VERSION_RESET  0x00010302

__INLINE uint32_t mdmhp_mdm_version_get(void)
{
    return REG_IP_RD(MDM_VERSION_MDMHP_MDM_VERSION_ADDR);
}

// field definitions
#define MDM_VERSION_MDMHP_REL_MASK     ((uint32_t)0x00FF0000)
#define MDM_VERSION_MDMHP_REL_LSB      16
#define MDM_VERSION_MDMHP_REL_WIDTH    ((uint32_t)0x00000008)
#define MDM_VERSION_MDMHP_UPG_MASK     ((uint32_t)0x0000FF00)
#define MDM_VERSION_MDMHP_UPG_LSB      8
#define MDM_VERSION_MDMHP_UPG_WIDTH    ((uint32_t)0x00000008)
#define MDM_VERSION_MDMHP_BUILD_MASK   ((uint32_t)0x000000FF)
#define MDM_VERSION_MDMHP_BUILD_LSB    0
#define MDM_VERSION_MDMHP_BUILD_WIDTH  ((uint32_t)0x00000008)

#define MDM_VERSION_MDMHP_REL_RST      0x1
#define MDM_VERSION_MDMHP_UPG_RST      0x3
#define MDM_VERSION_MDMHP_BUILD_RST    0x2

__INLINE void mdmhp_mdm_version_unpack(uint8_t* rel, uint8_t* upg, uint8_t* build)
{
    uint32_t localVal = REG_IP_RD(MDM_VERSION_MDMHP_MDM_VERSION_ADDR);

    *rel = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *upg = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *build = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t mdmhp_mdm_version_rel_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_VERSION_MDMHP_MDM_VERSION_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE uint8_t mdmhp_mdm_version_upg_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_VERSION_MDMHP_MDM_VERSION_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE uint8_t mdmhp_mdm_version_build_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_VERSION_MDMHP_MDM_VERSION_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

/**
 * @brief MDM_CONFIG register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     16                 BTDM   0
 *     01            OQPSK_154   0
 *     00               BLE_LR   0
 * </pre>
 */
#define MDM_CONFIG_MDMHP_MDM_CONFIG_ADDR   0x21020004
#define MDM_CONFIG_MDMHP_MDM_CONFIG_OFFSET 0x00000004
#define MDM_CONFIG_MDMHP_MDM_CONFIG_INDEX  0x00000001
#define MDM_CONFIG_MDMHP_MDM_CONFIG_RESET  0x00000000

__INLINE uint32_t mdmhp_mdm_config_get(void)
{
    return REG_IP_RD(MDM_CONFIG_MDMHP_MDM_CONFIG_ADDR);
}

// field definitions
#define MDM_CONFIG_MDMHP_BTDM_BIT         ((uint32_t)0x00010000)
#define MDM_CONFIG_MDMHP_BTDM_POS         16
#define MDM_CONFIG_MDMHP_OQPSK_154_BIT    ((uint32_t)0x00000002)
#define MDM_CONFIG_MDMHP_OQPSK_154_POS    1
#define MDM_CONFIG_MDMHP_BLE_LR_BIT       ((uint32_t)0x00000001)
#define MDM_CONFIG_MDMHP_BLE_LR_POS       0

#define MDM_CONFIG_MDMHP_BTDM_RST         0x0
#define MDM_CONFIG_MDMHP_OQPSK_154_RST    0x0
#define MDM_CONFIG_MDMHP_BLE_LR_RST       0x0

__INLINE void mdmhp_mdm_config_unpack(uint8_t* btdm, uint8_t* oqpsk154, uint8_t* blelr)
{
    uint32_t localVal = REG_IP_RD(MDM_CONFIG_MDMHP_MDM_CONFIG_ADDR);

    *btdm = (localVal & ((uint32_t)0x00010000)) >> 16;
    *oqpsk154 = (localVal & ((uint32_t)0x00000002)) >> 1;
    *blelr = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t mdmhp_mdm_config_btdm_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_CONFIG_MDMHP_MDM_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE uint8_t mdmhp_mdm_config_oqpsk_154_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_CONFIG_MDMHP_MDM_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t mdmhp_mdm_config_ble_lr_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_CONFIG_MDMHP_MDM_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief STAT_RXTSI0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:16        STAT_TSI_STOP   0x0
 *  13:00       STAT_TSI_START   0x0
 * </pre>
 */
#define STAT_RXTSI0_MDMHP_STAT_RXTSI0_ADDR   0x21020008
#define STAT_RXTSI0_MDMHP_STAT_RXTSI0_OFFSET 0x00000008
#define STAT_RXTSI0_MDMHP_STAT_RXTSI0_INDEX  0x00000002
#define STAT_RXTSI0_MDMHP_STAT_RXTSI0_RESET  0x00000000

__INLINE uint32_t mdmhp_stat_rxtsi0_get(void)
{
    return REG_IP_RD(STAT_RXTSI0_MDMHP_STAT_RXTSI0_ADDR);
}

// field definitions
#define STAT_RXTSI0_MDMHP_STAT_TSI_STOP_MASK    ((uint32_t)0x3FFF0000)
#define STAT_RXTSI0_MDMHP_STAT_TSI_STOP_LSB     16
#define STAT_RXTSI0_MDMHP_STAT_TSI_STOP_WIDTH   ((uint32_t)0x0000000E)
#define STAT_RXTSI0_MDMHP_STAT_TSI_START_MASK   ((uint32_t)0x00003FFF)
#define STAT_RXTSI0_MDMHP_STAT_TSI_START_LSB    0
#define STAT_RXTSI0_MDMHP_STAT_TSI_START_WIDTH  ((uint32_t)0x0000000E)

#define STAT_RXTSI0_MDMHP_STAT_TSI_STOP_RST     0x0
#define STAT_RXTSI0_MDMHP_STAT_TSI_START_RST    0x0

__INLINE void mdmhp_stat_rxtsi0_unpack(uint16_t* stattsistop, uint16_t* stattsistart)
{
    uint32_t localVal = REG_IP_RD(STAT_RXTSI0_MDMHP_STAT_RXTSI0_ADDR);

    *stattsistop = (localVal & ((uint32_t)0x3FFF0000)) >> 16;
    *stattsistart = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint16_t mdmhp_stat_rxtsi0_stat_tsi_stop_getf(void)
{
    uint32_t localVal = REG_IP_RD(STAT_RXTSI0_MDMHP_STAT_RXTSI0_ADDR);
    return ((localVal & ((uint32_t)0x3FFF0000)) >> 16);
}

__INLINE uint16_t mdmhp_stat_rxtsi0_stat_tsi_start_getf(void)
{
    uint32_t localVal = REG_IP_RD(STAT_RXTSI0_MDMHP_STAT_RXTSI0_ADDR);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

/**
 * @brief STAT_RXTSI1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  17:16   STAT_PEAK_CORR_TAU   0x0
 *  13:00   STAT_PEAK_CORR_INDEX   0x0
 * </pre>
 */
#define STAT_RXTSI1_MDMHP_STAT_RXTSI1_ADDR   0x2102000C
#define STAT_RXTSI1_MDMHP_STAT_RXTSI1_OFFSET 0x0000000C
#define STAT_RXTSI1_MDMHP_STAT_RXTSI1_INDEX  0x00000003
#define STAT_RXTSI1_MDMHP_STAT_RXTSI1_RESET  0x00000000

__INLINE uint32_t mdmhp_stat_rxtsi1_get(void)
{
    return REG_IP_RD(STAT_RXTSI1_MDMHP_STAT_RXTSI1_ADDR);
}

// field definitions
#define STAT_RXTSI1_MDMHP_STAT_PEAK_CORR_TAU_MASK     ((uint32_t)0x00030000)
#define STAT_RXTSI1_MDMHP_STAT_PEAK_CORR_TAU_LSB      16
#define STAT_RXTSI1_MDMHP_STAT_PEAK_CORR_TAU_WIDTH    ((uint32_t)0x00000002)
#define STAT_RXTSI1_MDMHP_STAT_PEAK_CORR_INDEX_MASK   ((uint32_t)0x00003FFF)
#define STAT_RXTSI1_MDMHP_STAT_PEAK_CORR_INDEX_LSB    0
#define STAT_RXTSI1_MDMHP_STAT_PEAK_CORR_INDEX_WIDTH  ((uint32_t)0x0000000E)

#define STAT_RXTSI1_MDMHP_STAT_PEAK_CORR_TAU_RST      0x0
#define STAT_RXTSI1_MDMHP_STAT_PEAK_CORR_INDEX_RST    0x0

__INLINE void mdmhp_stat_rxtsi1_unpack(uint8_t* statpeakcorrtau, uint16_t* statpeakcorrindex)
{
    uint32_t localVal = REG_IP_RD(STAT_RXTSI1_MDMHP_STAT_RXTSI1_ADDR);

    *statpeakcorrtau = (localVal & ((uint32_t)0x00030000)) >> 16;
    *statpeakcorrindex = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint8_t mdmhp_stat_rxtsi1_stat_peak_corr_tau_getf(void)
{
    uint32_t localVal = REG_IP_RD(STAT_RXTSI1_MDMHP_STAT_RXTSI1_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE uint16_t mdmhp_stat_rxtsi1_stat_peak_corr_index_getf(void)
{
    uint32_t localVal = REG_IP_RD(STAT_RXTSI1_MDMHP_STAT_RXTSI1_ADDR);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

/**
 * @brief STAT_RXSYNC0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  28:20          STAT_CFOEST   0x0
 *     16         STAT_SYNC_OK   0
 *  07:00     STAT_NB_SYNC_ERR   0x0
 * </pre>
 */
#define STAT_RXSYNC0_MDMHP_STAT_RXSYNC0_ADDR   0x21020010
#define STAT_RXSYNC0_MDMHP_STAT_RXSYNC0_OFFSET 0x00000010
#define STAT_RXSYNC0_MDMHP_STAT_RXSYNC0_INDEX  0x00000004
#define STAT_RXSYNC0_MDMHP_STAT_RXSYNC0_RESET  0x00000000

__INLINE uint32_t mdmhp_stat_rxsync0_get(void)
{
    return REG_IP_RD(STAT_RXSYNC0_MDMHP_STAT_RXSYNC0_ADDR);
}

// field definitions
#define STAT_RXSYNC0_MDMHP_STAT_CFOEST_MASK        ((uint32_t)0x1FF00000)
#define STAT_RXSYNC0_MDMHP_STAT_CFOEST_LSB         20
#define STAT_RXSYNC0_MDMHP_STAT_CFOEST_WIDTH       ((uint32_t)0x00000009)
#define STAT_RXSYNC0_MDMHP_STAT_SYNC_OK_BIT        ((uint32_t)0x00010000)
#define STAT_RXSYNC0_MDMHP_STAT_SYNC_OK_POS        16
#define STAT_RXSYNC0_MDMHP_STAT_NB_SYNC_ERR_MASK   ((uint32_t)0x000000FF)
#define STAT_RXSYNC0_MDMHP_STAT_NB_SYNC_ERR_LSB    0
#define STAT_RXSYNC0_MDMHP_STAT_NB_SYNC_ERR_WIDTH  ((uint32_t)0x00000008)

#define STAT_RXSYNC0_MDMHP_STAT_CFOEST_RST         0x0
#define STAT_RXSYNC0_MDMHP_STAT_SYNC_OK_RST        0x0
#define STAT_RXSYNC0_MDMHP_STAT_NB_SYNC_ERR_RST    0x0

__INLINE void mdmhp_stat_rxsync0_unpack(uint16_t* statcfoest, uint8_t* statsyncok, uint8_t* statnbsyncerr)
{
    uint32_t localVal = REG_IP_RD(STAT_RXSYNC0_MDMHP_STAT_RXSYNC0_ADDR);

    *statcfoest = (localVal & ((uint32_t)0x1FF00000)) >> 20;
    *statsyncok = (localVal & ((uint32_t)0x00010000)) >> 16;
    *statnbsyncerr = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint16_t mdmhp_stat_rxsync0_stat_cfoest_getf(void)
{
    uint32_t localVal = REG_IP_RD(STAT_RXSYNC0_MDMHP_STAT_RXSYNC0_ADDR);
    return ((localVal & ((uint32_t)0x1FF00000)) >> 20);
}

__INLINE uint8_t mdmhp_stat_rxsync0_stat_sync_ok_getf(void)
{
    uint32_t localVal = REG_IP_RD(STAT_RXSYNC0_MDMHP_STAT_RXSYNC0_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE uint8_t mdmhp_stat_rxsync0_stat_nb_sync_err_getf(void)
{
    uint32_t localVal = REG_IP_RD(STAT_RXSYNC0_MDMHP_STAT_RXSYNC0_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

/**
 * @brief STAT_RXSYNC1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31      STAT_PSI_DL_SEL   0
 *     30        STAT_ZCIND_OK   0
 *  29:00           STAT_ZCIND   0x0
 * </pre>
 */
#define STAT_RXSYNC1_MDMHP_STAT_RXSYNC1_ADDR   0x21020014
#define STAT_RXSYNC1_MDMHP_STAT_RXSYNC1_OFFSET 0x00000014
#define STAT_RXSYNC1_MDMHP_STAT_RXSYNC1_INDEX  0x00000005
#define STAT_RXSYNC1_MDMHP_STAT_RXSYNC1_RESET  0x00000000

__INLINE uint32_t mdmhp_stat_rxsync1_get(void)
{
    return REG_IP_RD(STAT_RXSYNC1_MDMHP_STAT_RXSYNC1_ADDR);
}

// field definitions
#define STAT_RXSYNC1_MDMHP_STAT_PSI_DL_SEL_BIT    ((uint32_t)0x80000000)
#define STAT_RXSYNC1_MDMHP_STAT_PSI_DL_SEL_POS    31
#define STAT_RXSYNC1_MDMHP_STAT_ZCIND_OK_BIT      ((uint32_t)0x40000000)
#define STAT_RXSYNC1_MDMHP_STAT_ZCIND_OK_POS      30
#define STAT_RXSYNC1_MDMHP_STAT_ZCIND_MASK        ((uint32_t)0x3FFFFFFF)
#define STAT_RXSYNC1_MDMHP_STAT_ZCIND_LSB         0
#define STAT_RXSYNC1_MDMHP_STAT_ZCIND_WIDTH       ((uint32_t)0x0000001E)

#define STAT_RXSYNC1_MDMHP_STAT_PSI_DL_SEL_RST    0x0
#define STAT_RXSYNC1_MDMHP_STAT_ZCIND_OK_RST      0x0
#define STAT_RXSYNC1_MDMHP_STAT_ZCIND_RST         0x0

__INLINE void mdmhp_stat_rxsync1_unpack(uint8_t* statpsidlsel, uint8_t* statzcindok, uint32_t* statzcind)
{
    uint32_t localVal = REG_IP_RD(STAT_RXSYNC1_MDMHP_STAT_RXSYNC1_ADDR);

    *statpsidlsel = (localVal & ((uint32_t)0x80000000)) >> 31;
    *statzcindok = (localVal & ((uint32_t)0x40000000)) >> 30;
    *statzcind = (localVal & ((uint32_t)0x3FFFFFFF)) >> 0;
}

__INLINE uint8_t mdmhp_stat_rxsync1_stat_psi_dl_sel_getf(void)
{
    uint32_t localVal = REG_IP_RD(STAT_RXSYNC1_MDMHP_STAT_RXSYNC1_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE uint8_t mdmhp_stat_rxsync1_stat_zcind_ok_getf(void)
{
    uint32_t localVal = REG_IP_RD(STAT_RXSYNC1_MDMHP_STAT_RXSYNC1_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE uint32_t mdmhp_stat_rxsync1_stat_zcind_getf(void)
{
    uint32_t localVal = REG_IP_RD(STAT_RXSYNC1_MDMHP_STAT_RXSYNC1_ADDR);
    return ((localVal & ((uint32_t)0x3FFFFFFF)) >> 0);
}

/**
 * @brief STAT_RXSYNC2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00   STAT_SYNC_WORD_LSB   0x0
 * </pre>
 */
#define STAT_RXSYNC2_MDMHP_STAT_RXSYNC2_ADDR   0x21020018
#define STAT_RXSYNC2_MDMHP_STAT_RXSYNC2_OFFSET 0x00000018
#define STAT_RXSYNC2_MDMHP_STAT_RXSYNC2_INDEX  0x00000006
#define STAT_RXSYNC2_MDMHP_STAT_RXSYNC2_RESET  0x00000000

__INLINE uint32_t mdmhp_stat_rxsync2_get(void)
{
    return REG_IP_RD(STAT_RXSYNC2_MDMHP_STAT_RXSYNC2_ADDR);
}

// field definitions
#define STAT_RXSYNC2_MDMHP_STAT_SYNC_WORD_LSB_MASK   ((uint32_t)0xFFFFFFFF)
#define STAT_RXSYNC2_MDMHP_STAT_SYNC_WORD_LSB_LSB    0
#define STAT_RXSYNC2_MDMHP_STAT_SYNC_WORD_LSB_WIDTH  ((uint32_t)0x00000020)

#define STAT_RXSYNC2_MDMHP_STAT_SYNC_WORD_LSB_RST    0x0

__INLINE uint32_t mdmhp_stat_rxsync2_stat_sync_word_lsb_getf(void)
{
    uint32_t localVal = REG_IP_RD(STAT_RXSYNC2_MDMHP_STAT_RXSYNC2_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief STAT_RXRSSI register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:12    STAT_RSSI_EST_LIN   0x0
 *     08   STAT_RX_NHP_LP_SEL   0
 *  07:00    STAT_RSSI_EST_DBV   0x0
 * </pre>
 */
#define STAT_RXRSSI_MDMHP_STAT_RXRSSI_ADDR   0x2102001C
#define STAT_RXRSSI_MDMHP_STAT_RXRSSI_OFFSET 0x0000001C
#define STAT_RXRSSI_MDMHP_STAT_RXRSSI_INDEX  0x00000007
#define STAT_RXRSSI_MDMHP_STAT_RXRSSI_RESET  0x00000000

__INLINE uint32_t mdmhp_stat_rxrssi_get(void)
{
    return REG_IP_RD(STAT_RXRSSI_MDMHP_STAT_RXRSSI_ADDR);
}

// field definitions
#define STAT_RXRSSI_MDMHP_STAT_RSSI_EST_LIN_MASK    ((uint32_t)0x3FFFF000)
#define STAT_RXRSSI_MDMHP_STAT_RSSI_EST_LIN_LSB     12
#define STAT_RXRSSI_MDMHP_STAT_RSSI_EST_LIN_WIDTH   ((uint32_t)0x00000012)
#define STAT_RXRSSI_MDMHP_STAT_RX_NHP_LP_SEL_BIT    ((uint32_t)0x00000100)
#define STAT_RXRSSI_MDMHP_STAT_RX_NHP_LP_SEL_POS    8
#define STAT_RXRSSI_MDMHP_STAT_RSSI_EST_DBV_MASK    ((uint32_t)0x000000FF)
#define STAT_RXRSSI_MDMHP_STAT_RSSI_EST_DBV_LSB     0
#define STAT_RXRSSI_MDMHP_STAT_RSSI_EST_DBV_WIDTH   ((uint32_t)0x00000008)

#define STAT_RXRSSI_MDMHP_STAT_RSSI_EST_LIN_RST     0x0
#define STAT_RXRSSI_MDMHP_STAT_RX_NHP_LP_SEL_RST    0x0
#define STAT_RXRSSI_MDMHP_STAT_RSSI_EST_DBV_RST     0x0

__INLINE void mdmhp_stat_rxrssi_unpack(uint32_t* statrssiestlin, uint8_t* statrxnhplpsel, uint8_t* statrssiestdbv)
{
    uint32_t localVal = REG_IP_RD(STAT_RXRSSI_MDMHP_STAT_RXRSSI_ADDR);

    *statrssiestlin = (localVal & ((uint32_t)0x3FFFF000)) >> 12;
    *statrxnhplpsel = (localVal & ((uint32_t)0x00000100)) >> 8;
    *statrssiestdbv = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint32_t mdmhp_stat_rxrssi_stat_rssi_est_lin_getf(void)
{
    uint32_t localVal = REG_IP_RD(STAT_RXRSSI_MDMHP_STAT_RXRSSI_ADDR);
    return ((localVal & ((uint32_t)0x3FFFF000)) >> 12);
}

__INLINE uint8_t mdmhp_stat_rxrssi_stat_rx_nhp_lp_sel_getf(void)
{
    uint32_t localVal = REG_IP_RD(STAT_RXRSSI_MDMHP_STAT_RXRSSI_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE uint8_t mdmhp_stat_rxrssi_stat_rssi_est_dbv_getf(void)
{
    uint32_t localVal = REG_IP_RD(STAT_RXRSSI_MDMHP_STAT_RXRSSI_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

/**
 * @brief STAT_RXSTO register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24      STAT_STO_INDEX0   0x0
 *  23:20     STAT_STO_SCALING   0x0
 *  19:00         STAT_STO_EST   0x0
 * </pre>
 */
#define STAT_RXSTO_MDMHP_STAT_RXSTO_ADDR   0x21020020
#define STAT_RXSTO_MDMHP_STAT_RXSTO_OFFSET 0x00000020
#define STAT_RXSTO_MDMHP_STAT_RXSTO_INDEX  0x00000008
#define STAT_RXSTO_MDMHP_STAT_RXSTO_RESET  0x00000000

__INLINE uint32_t mdmhp_stat_rxsto_get(void)
{
    return REG_IP_RD(STAT_RXSTO_MDMHP_STAT_RXSTO_ADDR);
}

// field definitions
#define STAT_RXSTO_MDMHP_STAT_STO_INDEX0_MASK    ((uint32_t)0xFF000000)
#define STAT_RXSTO_MDMHP_STAT_STO_INDEX0_LSB     24
#define STAT_RXSTO_MDMHP_STAT_STO_INDEX0_WIDTH   ((uint32_t)0x00000008)
#define STAT_RXSTO_MDMHP_STAT_STO_SCALING_MASK   ((uint32_t)0x00F00000)
#define STAT_RXSTO_MDMHP_STAT_STO_SCALING_LSB    20
#define STAT_RXSTO_MDMHP_STAT_STO_SCALING_WIDTH  ((uint32_t)0x00000004)
#define STAT_RXSTO_MDMHP_STAT_STO_EST_MASK       ((uint32_t)0x000FFFFF)
#define STAT_RXSTO_MDMHP_STAT_STO_EST_LSB        0
#define STAT_RXSTO_MDMHP_STAT_STO_EST_WIDTH      ((uint32_t)0x00000014)

#define STAT_RXSTO_MDMHP_STAT_STO_INDEX0_RST     0x0
#define STAT_RXSTO_MDMHP_STAT_STO_SCALING_RST    0x0
#define STAT_RXSTO_MDMHP_STAT_STO_EST_RST        0x0

__INLINE void mdmhp_stat_rxsto_unpack(uint8_t* statstoindex0, uint8_t* statstoscaling, uint32_t* statstoest)
{
    uint32_t localVal = REG_IP_RD(STAT_RXSTO_MDMHP_STAT_RXSTO_ADDR);

    *statstoindex0 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *statstoscaling = (localVal & ((uint32_t)0x00F00000)) >> 20;
    *statstoest = (localVal & ((uint32_t)0x000FFFFF)) >> 0;
}

__INLINE uint8_t mdmhp_stat_rxsto_stat_sto_index0_getf(void)
{
    uint32_t localVal = REG_IP_RD(STAT_RXSTO_MDMHP_STAT_RXSTO_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE uint8_t mdmhp_stat_rxsto_stat_sto_scaling_getf(void)
{
    uint32_t localVal = REG_IP_RD(STAT_RXSTO_MDMHP_STAT_RXSTO_ADDR);
    return ((localVal & ((uint32_t)0x00F00000)) >> 20);
}

__INLINE uint32_t mdmhp_stat_rxsto_stat_sto_est_getf(void)
{
    uint32_t localVal = REG_IP_RD(STAT_RXSTO_MDMHP_STAT_RXSTO_ADDR);
    return ((localVal & ((uint32_t)0x000FFFFF)) >> 0);
}

/**
 * @brief STAT_RXLR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  17:16           STAT_LR_CI   0x0
 *  03:00     STAT_LR_SYNC_IND   0x0
 * </pre>
 */
#define STAT_RXLR_MDMHP_STAT_RXLR_ADDR   0x21020024
#define STAT_RXLR_MDMHP_STAT_RXLR_OFFSET 0x00000024
#define STAT_RXLR_MDMHP_STAT_RXLR_INDEX  0x00000009
#define STAT_RXLR_MDMHP_STAT_RXLR_RESET  0x00000000

__INLINE uint32_t mdmhp_stat_rxlr_get(void)
{
    return REG_IP_RD(STAT_RXLR_MDMHP_STAT_RXLR_ADDR);
}

// field definitions
#define STAT_RXLR_MDMHP_STAT_LR_CI_MASK         ((uint32_t)0x00030000)
#define STAT_RXLR_MDMHP_STAT_LR_CI_LSB          16
#define STAT_RXLR_MDMHP_STAT_LR_CI_WIDTH        ((uint32_t)0x00000002)
#define STAT_RXLR_MDMHP_STAT_LR_SYNC_IND_MASK   ((uint32_t)0x0000000F)
#define STAT_RXLR_MDMHP_STAT_LR_SYNC_IND_LSB    0
#define STAT_RXLR_MDMHP_STAT_LR_SYNC_IND_WIDTH  ((uint32_t)0x00000004)

#define STAT_RXLR_MDMHP_STAT_LR_CI_RST          0x0
#define STAT_RXLR_MDMHP_STAT_LR_SYNC_IND_RST    0x0

__INLINE void mdmhp_stat_rxlr_unpack(uint8_t* statlrci, uint8_t* statlrsyncind)
{
    uint32_t localVal = REG_IP_RD(STAT_RXLR_MDMHP_STAT_RXLR_ADDR);

    *statlrci = (localVal & ((uint32_t)0x00030000)) >> 16;
    *statlrsyncind = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t mdmhp_stat_rxlr_stat_lr_ci_getf(void)
{
    uint32_t localVal = REG_IP_RD(STAT_RXLR_MDMHP_STAT_RXLR_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE uint8_t mdmhp_stat_rxlr_stat_lr_sync_ind_getf(void)
{
    uint32_t localVal = REG_IP_RD(STAT_RXLR_MDMHP_STAT_RXLR_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

/**
 * @brief MDM_CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24        TX_STARTUPDEL   0x2A
 *  23:16        RX_STARTUPDEL   0x21
 *  15:12        CTES_SAMP_DLY   0x0
 *     10        CTES_FORCE_EN   0
 *     09          CTES_SOURCE   0
 *     08            RXCTES_EN   1
 *     07        RX_VALID_MODE   1
 *     06           HPLP_BB_IF   0
 *  05:04            HPLP_MODE   0x3
 *     02        TX_VALID_MODE   1
 *     01             FMTX2PEN   0
 *     00               FMTXEN   1
 * </pre>
 */
#define MDM_CNTL_MDMHP_MDM_CNTL_ADDR   0x21020030
#define MDM_CNTL_MDMHP_MDM_CNTL_OFFSET 0x00000030
#define MDM_CNTL_MDMHP_MDM_CNTL_INDEX  0x0000000C
#define MDM_CNTL_MDMHP_MDM_CNTL_RESET  0x2A2101B5

__INLINE uint32_t mdmhp_mdm_cntl_get(void)
{
    return REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR);
}

__INLINE void mdmhp_mdm_cntl_set(uint32_t value)
{
    REG_IP_WR(MDM_CNTL_MDMHP_MDM_CNTL_ADDR, value);
}

// field definitions
#define MDM_CNTL_MDMHP_TX_STARTUPDEL_MASK   ((uint32_t)0xFF000000)
#define MDM_CNTL_MDMHP_TX_STARTUPDEL_LSB    24
#define MDM_CNTL_MDMHP_TX_STARTUPDEL_WIDTH  ((uint32_t)0x00000008)
#define MDM_CNTL_MDMHP_RX_STARTUPDEL_MASK   ((uint32_t)0x00FF0000)
#define MDM_CNTL_MDMHP_RX_STARTUPDEL_LSB    16
#define MDM_CNTL_MDMHP_RX_STARTUPDEL_WIDTH  ((uint32_t)0x00000008)
#define MDM_CNTL_MDMHP_CTES_SAMP_DLY_MASK   ((uint32_t)0x0000F000)
#define MDM_CNTL_MDMHP_CTES_SAMP_DLY_LSB    12
#define MDM_CNTL_MDMHP_CTES_SAMP_DLY_WIDTH  ((uint32_t)0x00000004)
#define MDM_CNTL_MDMHP_CTES_FORCE_EN_BIT    ((uint32_t)0x00000400)
#define MDM_CNTL_MDMHP_CTES_FORCE_EN_POS    10
#define MDM_CNTL_MDMHP_CTES_SOURCE_BIT      ((uint32_t)0x00000200)
#define MDM_CNTL_MDMHP_CTES_SOURCE_POS      9
#define MDM_CNTL_MDMHP_RXCTES_EN_BIT        ((uint32_t)0x00000100)
#define MDM_CNTL_MDMHP_RXCTES_EN_POS        8
#define MDM_CNTL_MDMHP_RX_VALID_MODE_BIT    ((uint32_t)0x00000080)
#define MDM_CNTL_MDMHP_RX_VALID_MODE_POS    7
#define MDM_CNTL_MDMHP_HPLP_BB_IF_BIT       ((uint32_t)0x00000040)
#define MDM_CNTL_MDMHP_HPLP_BB_IF_POS       6
#define MDM_CNTL_MDMHP_HPLP_MODE_MASK       ((uint32_t)0x00000030)
#define MDM_CNTL_MDMHP_HPLP_MODE_LSB        4
#define MDM_CNTL_MDMHP_HPLP_MODE_WIDTH      ((uint32_t)0x00000002)
#define MDM_CNTL_MDMHP_TX_VALID_MODE_BIT    ((uint32_t)0x00000004)
#define MDM_CNTL_MDMHP_TX_VALID_MODE_POS    2
#define MDM_CNTL_MDMHP_FMTX2PEN_BIT         ((uint32_t)0x00000002)
#define MDM_CNTL_MDMHP_FMTX2PEN_POS         1
#define MDM_CNTL_MDMHP_FMTXEN_BIT           ((uint32_t)0x00000001)
#define MDM_CNTL_MDMHP_FMTXEN_POS           0

#define MDM_CNTL_MDMHP_TX_STARTUPDEL_RST    0x2A
#define MDM_CNTL_MDMHP_RX_STARTUPDEL_RST    0x21
#define MDM_CNTL_MDMHP_CTES_SAMP_DLY_RST    0x0
#define MDM_CNTL_MDMHP_CTES_FORCE_EN_RST    0x0
#define MDM_CNTL_MDMHP_CTES_SOURCE_RST      0x0
#define MDM_CNTL_MDMHP_RXCTES_EN_RST        0x1
#define MDM_CNTL_MDMHP_RX_VALID_MODE_RST    0x1
#define MDM_CNTL_MDMHP_HPLP_BB_IF_RST       0x0
#define MDM_CNTL_MDMHP_HPLP_MODE_RST        0x3
#define MDM_CNTL_MDMHP_TX_VALID_MODE_RST    0x1
#define MDM_CNTL_MDMHP_FMTX2PEN_RST         0x0
#define MDM_CNTL_MDMHP_FMTXEN_RST           0x1

__INLINE void mdmhp_mdm_cntl_pack(uint8_t txstartupdel, uint8_t rxstartupdel, uint8_t ctessampdly, uint8_t ctesforceen, uint8_t ctessource, uint8_t rxctesen, uint8_t rxvalidmode, uint8_t hplpbbif, uint8_t hplpmode, uint8_t txvalidmode, uint8_t fmtx2pen, uint8_t fmtxen)
{
    ASSERT_ERR((((uint32_t)txstartupdel << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)rxstartupdel << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)ctessampdly << 12) & ~((uint32_t)0x0000F000)) == 0);
    ASSERT_ERR((((uint32_t)ctesforceen << 10) & ~((uint32_t)0x00000400)) == 0);
    ASSERT_ERR((((uint32_t)ctessource << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)rxctesen << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)rxvalidmode << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)hplpbbif << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)hplpmode << 4) & ~((uint32_t)0x00000030)) == 0);
    ASSERT_ERR((((uint32_t)txvalidmode << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)fmtx2pen << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)fmtxen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_IP_WR(MDM_CNTL_MDMHP_MDM_CNTL_ADDR,  ((uint32_t)txstartupdel << 24) | ((uint32_t)rxstartupdel << 16) | ((uint32_t)ctessampdly << 12) | ((uint32_t)ctesforceen << 10) | ((uint32_t)ctessource << 9) | ((uint32_t)rxctesen << 8) | ((uint32_t)rxvalidmode << 7) | ((uint32_t)hplpbbif << 6) | ((uint32_t)hplpmode << 4) | ((uint32_t)txvalidmode << 2) | ((uint32_t)fmtx2pen << 1) | ((uint32_t)fmtxen << 0));
}

__INLINE void mdmhp_mdm_cntl_unpack(uint8_t* txstartupdel, uint8_t* rxstartupdel, uint8_t* ctessampdly, uint8_t* ctesforceen, uint8_t* ctessource, uint8_t* rxctesen, uint8_t* rxvalidmode, uint8_t* hplpbbif, uint8_t* hplpmode, uint8_t* txvalidmode, uint8_t* fmtx2pen, uint8_t* fmtxen)
{
    uint32_t localVal = REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR);

    *txstartupdel = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *rxstartupdel = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *ctessampdly = (localVal & ((uint32_t)0x0000F000)) >> 12;
    *ctesforceen = (localVal & ((uint32_t)0x00000400)) >> 10;
    *ctessource = (localVal & ((uint32_t)0x00000200)) >> 9;
    *rxctesen = (localVal & ((uint32_t)0x00000100)) >> 8;
    *rxvalidmode = (localVal & ((uint32_t)0x00000080)) >> 7;
    *hplpbbif = (localVal & ((uint32_t)0x00000040)) >> 6;
    *hplpmode = (localVal & ((uint32_t)0x00000030)) >> 4;
    *txvalidmode = (localVal & ((uint32_t)0x00000004)) >> 2;
    *fmtx2pen = (localVal & ((uint32_t)0x00000002)) >> 1;
    *fmtxen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t mdmhp_mdm_cntl_tx_startupdel_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void mdmhp_mdm_cntl_tx_startupdel_setf(uint8_t txstartupdel)
{
    ASSERT_ERR((((uint32_t)txstartupdel << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_IP_WR(MDM_CNTL_MDMHP_MDM_CNTL_ADDR, (REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)txstartupdel << 24));
}

__INLINE uint8_t mdmhp_mdm_cntl_rx_startupdel_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void mdmhp_mdm_cntl_rx_startupdel_setf(uint8_t rxstartupdel)
{
    ASSERT_ERR((((uint32_t)rxstartupdel << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_IP_WR(MDM_CNTL_MDMHP_MDM_CNTL_ADDR, (REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)rxstartupdel << 16));
}

__INLINE uint8_t mdmhp_mdm_cntl_ctes_samp_dly_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000F000)) >> 12);
}

__INLINE void mdmhp_mdm_cntl_ctes_samp_dly_setf(uint8_t ctessampdly)
{
    ASSERT_ERR((((uint32_t)ctessampdly << 12) & ~((uint32_t)0x0000F000)) == 0);
    REG_IP_WR(MDM_CNTL_MDMHP_MDM_CNTL_ADDR, (REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR) & ~((uint32_t)0x0000F000)) | ((uint32_t)ctessampdly << 12));
}

__INLINE uint8_t mdmhp_mdm_cntl_ctes_force_en_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void mdmhp_mdm_cntl_ctes_force_en_setf(uint8_t ctesforceen)
{
    ASSERT_ERR((((uint32_t)ctesforceen << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_IP_WR(MDM_CNTL_MDMHP_MDM_CNTL_ADDR, (REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)ctesforceen << 10));
}

__INLINE uint8_t mdmhp_mdm_cntl_ctes_source_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void mdmhp_mdm_cntl_ctes_source_setf(uint8_t ctessource)
{
    ASSERT_ERR((((uint32_t)ctessource << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_IP_WR(MDM_CNTL_MDMHP_MDM_CNTL_ADDR, (REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)ctessource << 9));
}

__INLINE uint8_t mdmhp_mdm_cntl_rxctes_en_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void mdmhp_mdm_cntl_rxctes_en_setf(uint8_t rxctesen)
{
    ASSERT_ERR((((uint32_t)rxctesen << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_IP_WR(MDM_CNTL_MDMHP_MDM_CNTL_ADDR, (REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)rxctesen << 8));
}

__INLINE uint8_t mdmhp_mdm_cntl_rx_valid_mode_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void mdmhp_mdm_cntl_rx_valid_mode_setf(uint8_t rxvalidmode)
{
    ASSERT_ERR((((uint32_t)rxvalidmode << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_IP_WR(MDM_CNTL_MDMHP_MDM_CNTL_ADDR, (REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)rxvalidmode << 7));
}

__INLINE uint8_t mdmhp_mdm_cntl_hplp_bb_if_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void mdmhp_mdm_cntl_hplp_bb_if_setf(uint8_t hplpbbif)
{
    ASSERT_ERR((((uint32_t)hplpbbif << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_IP_WR(MDM_CNTL_MDMHP_MDM_CNTL_ADDR, (REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)hplpbbif << 6));
}

__INLINE uint8_t mdmhp_mdm_cntl_hplp_mode_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000030)) >> 4);
}

__INLINE void mdmhp_mdm_cntl_hplp_mode_setf(uint8_t hplpmode)
{
    ASSERT_ERR((((uint32_t)hplpmode << 4) & ~((uint32_t)0x00000030)) == 0);
    REG_IP_WR(MDM_CNTL_MDMHP_MDM_CNTL_ADDR, (REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR) & ~((uint32_t)0x00000030)) | ((uint32_t)hplpmode << 4));
}

__INLINE uint8_t mdmhp_mdm_cntl_tx_valid_mode_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void mdmhp_mdm_cntl_tx_valid_mode_setf(uint8_t txvalidmode)
{
    ASSERT_ERR((((uint32_t)txvalidmode << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_IP_WR(MDM_CNTL_MDMHP_MDM_CNTL_ADDR, (REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)txvalidmode << 2));
}

__INLINE uint8_t mdmhp_mdm_cntl_fmtx2pen_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void mdmhp_mdm_cntl_fmtx2pen_setf(uint8_t fmtx2pen)
{
    ASSERT_ERR((((uint32_t)fmtx2pen << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_IP_WR(MDM_CNTL_MDMHP_MDM_CNTL_ADDR, (REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)fmtx2pen << 1));
}

__INLINE uint8_t mdmhp_mdm_cntl_fmtxen_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void mdmhp_mdm_cntl_fmtxen_setf(uint8_t fmtxen)
{
    ASSERT_ERR((((uint32_t)fmtxen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_IP_WR(MDM_CNTL_MDMHP_MDM_CNTL_ADDR, (REG_IP_RD(MDM_CNTL_MDMHP_MDM_CNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)fmtxen << 0));
}

/**
 * @brief MDM_DIAGCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24            DIAGCNTL3   0x7
 *  23:16            DIAGCNTL2   0x6
 *  15:08            DIAGCNTL1   0x37
 *  07:00            DIAGCNTL0   0x0
 * </pre>
 */
#define MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_ADDR   0x21020034
#define MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_OFFSET 0x00000034
#define MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_INDEX  0x0000000D
#define MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_RESET  0x07063700

__INLINE uint32_t mdmhp_mdm_diagcntl_get(void)
{
    return REG_IP_RD(MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_ADDR);
}

__INLINE void mdmhp_mdm_diagcntl_set(uint32_t value)
{
    REG_IP_WR(MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_ADDR, value);
}

// field definitions
#define MDM_DIAGCNTL_MDMHP_DIAGCNTL3_MASK   ((uint32_t)0xFF000000)
#define MDM_DIAGCNTL_MDMHP_DIAGCNTL3_LSB    24
#define MDM_DIAGCNTL_MDMHP_DIAGCNTL3_WIDTH  ((uint32_t)0x00000008)
#define MDM_DIAGCNTL_MDMHP_DIAGCNTL2_MASK   ((uint32_t)0x00FF0000)
#define MDM_DIAGCNTL_MDMHP_DIAGCNTL2_LSB    16
#define MDM_DIAGCNTL_MDMHP_DIAGCNTL2_WIDTH  ((uint32_t)0x00000008)
#define MDM_DIAGCNTL_MDMHP_DIAGCNTL1_MASK   ((uint32_t)0x0000FF00)
#define MDM_DIAGCNTL_MDMHP_DIAGCNTL1_LSB    8
#define MDM_DIAGCNTL_MDMHP_DIAGCNTL1_WIDTH  ((uint32_t)0x00000008)
#define MDM_DIAGCNTL_MDMHP_DIAGCNTL0_MASK   ((uint32_t)0x000000FF)
#define MDM_DIAGCNTL_MDMHP_DIAGCNTL0_LSB    0
#define MDM_DIAGCNTL_MDMHP_DIAGCNTL0_WIDTH  ((uint32_t)0x00000008)

#define MDM_DIAGCNTL_MDMHP_DIAGCNTL3_RST    0x7
#define MDM_DIAGCNTL_MDMHP_DIAGCNTL2_RST    0x6
#define MDM_DIAGCNTL_MDMHP_DIAGCNTL1_RST    0x37
#define MDM_DIAGCNTL_MDMHP_DIAGCNTL0_RST    0x0

__INLINE void mdmhp_mdm_diagcntl_pack(uint8_t diagcntl3, uint8_t diagcntl2, uint8_t diagcntl1, uint8_t diagcntl0)
{
    ASSERT_ERR((((uint32_t)diagcntl3 << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)diagcntl2 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)diagcntl1 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)diagcntl0 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_ADDR,  ((uint32_t)diagcntl3 << 24) | ((uint32_t)diagcntl2 << 16) | ((uint32_t)diagcntl1 << 8) | ((uint32_t)diagcntl0 << 0));
}

__INLINE void mdmhp_mdm_diagcntl_unpack(uint8_t* diagcntl3, uint8_t* diagcntl2, uint8_t* diagcntl1, uint8_t* diagcntl0)
{
    uint32_t localVal = REG_IP_RD(MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_ADDR);

    *diagcntl3 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *diagcntl2 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *diagcntl1 = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *diagcntl0 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t mdmhp_mdm_diagcntl_diagcntl3_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void mdmhp_mdm_diagcntl_diagcntl3_setf(uint8_t diagcntl3)
{
    ASSERT_ERR((((uint32_t)diagcntl3 << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_IP_WR(MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_ADDR, (REG_IP_RD(MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)diagcntl3 << 24));
}

__INLINE uint8_t mdmhp_mdm_diagcntl_diagcntl2_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void mdmhp_mdm_diagcntl_diagcntl2_setf(uint8_t diagcntl2)
{
    ASSERT_ERR((((uint32_t)diagcntl2 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_IP_WR(MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_ADDR, (REG_IP_RD(MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)diagcntl2 << 16));
}

__INLINE uint8_t mdmhp_mdm_diagcntl_diagcntl1_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void mdmhp_mdm_diagcntl_diagcntl1_setf(uint8_t diagcntl1)
{
    ASSERT_ERR((((uint32_t)diagcntl1 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_IP_WR(MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_ADDR, (REG_IP_RD(MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)diagcntl1 << 8));
}

__INLINE uint8_t mdmhp_mdm_diagcntl_diagcntl0_getf(void)
{
    uint32_t localVal = REG_IP_RD(MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void mdmhp_mdm_diagcntl_diagcntl0_setf(uint8_t diagcntl0)
{
    ASSERT_ERR((((uint32_t)diagcntl0 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_ADDR, (REG_IP_RD(MDM_DIAGCNTL_MDMHP_MDM_DIAGCNTL_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)diagcntl0 << 0));
}

/**
 * @brief GSG_CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:24     GSG_DPHI_NOM_BLE   0x1
 *  19:16     GSG_DPHI_DEN_BLE   0x0
 *  15:08              GSG_NOM   0x1
 *  02:00              GSG_DEN   0x0
 * </pre>
 */
#define GSG_CNTL_MDMHP_GSG_CNTL_ADDR   0x21020038
#define GSG_CNTL_MDMHP_GSG_CNTL_OFFSET 0x00000038
#define GSG_CNTL_MDMHP_GSG_CNTL_INDEX  0x0000000E
#define GSG_CNTL_MDMHP_GSG_CNTL_RESET  0x01000100

__INLINE uint32_t mdmhp_gsg_cntl_get(void)
{
    return REG_IP_RD(GSG_CNTL_MDMHP_GSG_CNTL_ADDR);
}

__INLINE void mdmhp_gsg_cntl_set(uint32_t value)
{
    REG_IP_WR(GSG_CNTL_MDMHP_GSG_CNTL_ADDR, value);
}

// field definitions
#define GSG_CNTL_MDMHP_GSG_DPHI_NOM_BLE_MASK   ((uint32_t)0x7F000000)
#define GSG_CNTL_MDMHP_GSG_DPHI_NOM_BLE_LSB    24
#define GSG_CNTL_MDMHP_GSG_DPHI_NOM_BLE_WIDTH  ((uint32_t)0x00000007)
#define GSG_CNTL_MDMHP_GSG_DPHI_DEN_BLE_MASK   ((uint32_t)0x000F0000)
#define GSG_CNTL_MDMHP_GSG_DPHI_DEN_BLE_LSB    16
#define GSG_CNTL_MDMHP_GSG_DPHI_DEN_BLE_WIDTH  ((uint32_t)0x00000004)
#define GSG_CNTL_MDMHP_GSG_NOM_MASK            ((uint32_t)0x0000FF00)
#define GSG_CNTL_MDMHP_GSG_NOM_LSB             8
#define GSG_CNTL_MDMHP_GSG_NOM_WIDTH           ((uint32_t)0x00000008)
#define GSG_CNTL_MDMHP_GSG_DEN_MASK            ((uint32_t)0x00000007)
#define GSG_CNTL_MDMHP_GSG_DEN_LSB             0
#define GSG_CNTL_MDMHP_GSG_DEN_WIDTH           ((uint32_t)0x00000003)

#define GSG_CNTL_MDMHP_GSG_DPHI_NOM_BLE_RST    0x1
#define GSG_CNTL_MDMHP_GSG_DPHI_DEN_BLE_RST    0x0
#define GSG_CNTL_MDMHP_GSG_NOM_RST             0x1
#define GSG_CNTL_MDMHP_GSG_DEN_RST             0x0

__INLINE void mdmhp_gsg_cntl_pack(uint8_t gsgdphinomble, uint8_t gsgdphidenble, uint8_t gsgnom, uint8_t gsgden)
{
    ASSERT_ERR((((uint32_t)gsgdphinomble << 24) & ~((uint32_t)0x7F000000)) == 0);
    ASSERT_ERR((((uint32_t)gsgdphidenble << 16) & ~((uint32_t)0x000F0000)) == 0);
    ASSERT_ERR((((uint32_t)gsgnom << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)gsgden << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_IP_WR(GSG_CNTL_MDMHP_GSG_CNTL_ADDR,  ((uint32_t)gsgdphinomble << 24) | ((uint32_t)gsgdphidenble << 16) | ((uint32_t)gsgnom << 8) | ((uint32_t)gsgden << 0));
}

__INLINE void mdmhp_gsg_cntl_unpack(uint8_t* gsgdphinomble, uint8_t* gsgdphidenble, uint8_t* gsgnom, uint8_t* gsgden)
{
    uint32_t localVal = REG_IP_RD(GSG_CNTL_MDMHP_GSG_CNTL_ADDR);

    *gsgdphinomble = (localVal & ((uint32_t)0x7F000000)) >> 24;
    *gsgdphidenble = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *gsgnom = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *gsgden = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t mdmhp_gsg_cntl_gsg_dphi_nom_ble_getf(void)
{
    uint32_t localVal = REG_IP_RD(GSG_CNTL_MDMHP_GSG_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x7F000000)) >> 24);
}

__INLINE void mdmhp_gsg_cntl_gsg_dphi_nom_ble_setf(uint8_t gsgdphinomble)
{
    ASSERT_ERR((((uint32_t)gsgdphinomble << 24) & ~((uint32_t)0x7F000000)) == 0);
    REG_IP_WR(GSG_CNTL_MDMHP_GSG_CNTL_ADDR, (REG_IP_RD(GSG_CNTL_MDMHP_GSG_CNTL_ADDR) & ~((uint32_t)0x7F000000)) | ((uint32_t)gsgdphinomble << 24));
}

__INLINE uint8_t mdmhp_gsg_cntl_gsg_dphi_den_ble_getf(void)
{
    uint32_t localVal = REG_IP_RD(GSG_CNTL_MDMHP_GSG_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void mdmhp_gsg_cntl_gsg_dphi_den_ble_setf(uint8_t gsgdphidenble)
{
    ASSERT_ERR((((uint32_t)gsgdphidenble << 16) & ~((uint32_t)0x000F0000)) == 0);
    REG_IP_WR(GSG_CNTL_MDMHP_GSG_CNTL_ADDR, (REG_IP_RD(GSG_CNTL_MDMHP_GSG_CNTL_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)gsgdphidenble << 16));
}

__INLINE uint8_t mdmhp_gsg_cntl_gsg_nom_getf(void)
{
    uint32_t localVal = REG_IP_RD(GSG_CNTL_MDMHP_GSG_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void mdmhp_gsg_cntl_gsg_nom_setf(uint8_t gsgnom)
{
    ASSERT_ERR((((uint32_t)gsgnom << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_IP_WR(GSG_CNTL_MDMHP_GSG_CNTL_ADDR, (REG_IP_RD(GSG_CNTL_MDMHP_GSG_CNTL_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)gsgnom << 8));
}

__INLINE uint8_t mdmhp_gsg_cntl_gsg_den_getf(void)
{
    uint32_t localVal = REG_IP_RD(GSG_CNTL_MDMHP_GSG_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void mdmhp_gsg_cntl_gsg_den_setf(uint8_t gsgden)
{
    ASSERT_ERR((((uint32_t)gsgden << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_IP_WR(GSG_CNTL_MDMHP_GSG_CNTL_ADDR, (REG_IP_RD(GSG_CNTL_MDMHP_GSG_CNTL_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)gsgden << 0));
}

/**
 * @brief GSG_VCO_BLE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24    GSG_VCO_NOM_BLE2M   0xBA
 *  20:16    GSG_VCO_DEN_BLE2M   0xA
 *  15:08    GSG_VCO_NOM_BLE1M   0xB9
 *  04:00    GSG_VCO_DEN_BLE1M   0x9
 * </pre>
 */
#define GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_ADDR   0x2102003C
#define GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_OFFSET 0x0000003C
#define GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_INDEX  0x0000000F
#define GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_RESET  0xBA0AB909

__INLINE uint32_t mdmhp_gsg_vco_ble_get(void)
{
    return REG_IP_RD(GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_ADDR);
}

__INLINE void mdmhp_gsg_vco_ble_set(uint32_t value)
{
    REG_IP_WR(GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_ADDR, value);
}

// field definitions
#define GSG_VCO_BLE_MDMHP_GSG_VCO_NOM_BLE2M_MASK   ((uint32_t)0xFF000000)
#define GSG_VCO_BLE_MDMHP_GSG_VCO_NOM_BLE2M_LSB    24
#define GSG_VCO_BLE_MDMHP_GSG_VCO_NOM_BLE2M_WIDTH  ((uint32_t)0x00000008)
#define GSG_VCO_BLE_MDMHP_GSG_VCO_DEN_BLE2M_MASK   ((uint32_t)0x001F0000)
#define GSG_VCO_BLE_MDMHP_GSG_VCO_DEN_BLE2M_LSB    16
#define GSG_VCO_BLE_MDMHP_GSG_VCO_DEN_BLE2M_WIDTH  ((uint32_t)0x00000005)
#define GSG_VCO_BLE_MDMHP_GSG_VCO_NOM_BLE1M_MASK   ((uint32_t)0x0000FF00)
#define GSG_VCO_BLE_MDMHP_GSG_VCO_NOM_BLE1M_LSB    8
#define GSG_VCO_BLE_MDMHP_GSG_VCO_NOM_BLE1M_WIDTH  ((uint32_t)0x00000008)
#define GSG_VCO_BLE_MDMHP_GSG_VCO_DEN_BLE1M_MASK   ((uint32_t)0x0000001F)
#define GSG_VCO_BLE_MDMHP_GSG_VCO_DEN_BLE1M_LSB    0
#define GSG_VCO_BLE_MDMHP_GSG_VCO_DEN_BLE1M_WIDTH  ((uint32_t)0x00000005)

#define GSG_VCO_BLE_MDMHP_GSG_VCO_NOM_BLE2M_RST    0xBA
#define GSG_VCO_BLE_MDMHP_GSG_VCO_DEN_BLE2M_RST    0xA
#define GSG_VCO_BLE_MDMHP_GSG_VCO_NOM_BLE1M_RST    0xB9
#define GSG_VCO_BLE_MDMHP_GSG_VCO_DEN_BLE1M_RST    0x9

__INLINE void mdmhp_gsg_vco_ble_pack(uint8_t gsgvconomble2m, uint8_t gsgvcodenble2m, uint8_t gsgvconomble1m, uint8_t gsgvcodenble1m)
{
    ASSERT_ERR((((uint32_t)gsgvconomble2m << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)gsgvcodenble2m << 16) & ~((uint32_t)0x001F0000)) == 0);
    ASSERT_ERR((((uint32_t)gsgvconomble1m << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)gsgvcodenble1m << 0) & ~((uint32_t)0x0000001F)) == 0);
    REG_IP_WR(GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_ADDR,  ((uint32_t)gsgvconomble2m << 24) | ((uint32_t)gsgvcodenble2m << 16) | ((uint32_t)gsgvconomble1m << 8) | ((uint32_t)gsgvcodenble1m << 0));
}

__INLINE void mdmhp_gsg_vco_ble_unpack(uint8_t* gsgvconomble2m, uint8_t* gsgvcodenble2m, uint8_t* gsgvconomble1m, uint8_t* gsgvcodenble1m)
{
    uint32_t localVal = REG_IP_RD(GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_ADDR);

    *gsgvconomble2m = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *gsgvcodenble2m = (localVal & ((uint32_t)0x001F0000)) >> 16;
    *gsgvconomble1m = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *gsgvcodenble1m = (localVal & ((uint32_t)0x0000001F)) >> 0;
}

__INLINE uint8_t mdmhp_gsg_vco_ble_gsg_vco_nom_ble2m_getf(void)
{
    uint32_t localVal = REG_IP_RD(GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void mdmhp_gsg_vco_ble_gsg_vco_nom_ble2m_setf(uint8_t gsgvconomble2m)
{
    ASSERT_ERR((((uint32_t)gsgvconomble2m << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_IP_WR(GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_ADDR, (REG_IP_RD(GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)gsgvconomble2m << 24));
}

__INLINE uint8_t mdmhp_gsg_vco_ble_gsg_vco_den_ble2m_getf(void)
{
    uint32_t localVal = REG_IP_RD(GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_ADDR);
    return ((localVal & ((uint32_t)0x001F0000)) >> 16);
}

__INLINE void mdmhp_gsg_vco_ble_gsg_vco_den_ble2m_setf(uint8_t gsgvcodenble2m)
{
    ASSERT_ERR((((uint32_t)gsgvcodenble2m << 16) & ~((uint32_t)0x001F0000)) == 0);
    REG_IP_WR(GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_ADDR, (REG_IP_RD(GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_ADDR) & ~((uint32_t)0x001F0000)) | ((uint32_t)gsgvcodenble2m << 16));
}

__INLINE uint8_t mdmhp_gsg_vco_ble_gsg_vco_nom_ble1m_getf(void)
{
    uint32_t localVal = REG_IP_RD(GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void mdmhp_gsg_vco_ble_gsg_vco_nom_ble1m_setf(uint8_t gsgvconomble1m)
{
    ASSERT_ERR((((uint32_t)gsgvconomble1m << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_IP_WR(GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_ADDR, (REG_IP_RD(GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)gsgvconomble1m << 8));
}

__INLINE uint8_t mdmhp_gsg_vco_ble_gsg_vco_den_ble1m_getf(void)
{
    uint32_t localVal = REG_IP_RD(GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_ADDR);
    return ((localVal & ((uint32_t)0x0000001F)) >> 0);
}

__INLINE void mdmhp_gsg_vco_ble_gsg_vco_den_ble1m_setf(uint8_t gsgvcodenble1m)
{
    ASSERT_ERR((((uint32_t)gsgvcodenble1m << 0) & ~((uint32_t)0x0000001F)) == 0);
    REG_IP_WR(GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_ADDR, (REG_IP_RD(GSG_VCO_BLE_MDMHP_GSG_VCO_BLE_ADDR) & ~((uint32_t)0x0000001F)) | ((uint32_t)gsgvcodenble1m << 0));
}

/**
 * @brief FM2P_SWLAT_BLE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  20:16     FM2P_SWLAT_BLE2M   0x3
 *  04:00     FM2P_SWLAT_BLE1M   0x6
 * </pre>
 */
#define FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE_ADDR   0x21020040
#define FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE_OFFSET 0x00000040
#define FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE_INDEX  0x00000010
#define FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE_RESET  0x00030006

__INLINE uint32_t mdmhp_fm2p_swlat_ble_get(void)
{
    return REG_IP_RD(FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE_ADDR);
}

__INLINE void mdmhp_fm2p_swlat_ble_set(uint32_t value)
{
    REG_IP_WR(FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE_ADDR, value);
}

// field definitions
#define FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE2M_MASK   ((uint32_t)0x001F0000)
#define FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE2M_LSB    16
#define FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE2M_WIDTH  ((uint32_t)0x00000005)
#define FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE1M_MASK   ((uint32_t)0x0000001F)
#define FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE1M_LSB    0
#define FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE1M_WIDTH  ((uint32_t)0x00000005)

#define FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE2M_RST    0x3
#define FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE1M_RST    0x6

__INLINE void mdmhp_fm2p_swlat_ble_pack(uint8_t fm2pswlatble2m, uint8_t fm2pswlatble1m)
{
    ASSERT_ERR((((uint32_t)fm2pswlatble2m << 16) & ~((uint32_t)0x001F0000)) == 0);
    ASSERT_ERR((((uint32_t)fm2pswlatble1m << 0) & ~((uint32_t)0x0000001F)) == 0);
    REG_IP_WR(FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE_ADDR,  ((uint32_t)fm2pswlatble2m << 16) | ((uint32_t)fm2pswlatble1m << 0));
}

__INLINE void mdmhp_fm2p_swlat_ble_unpack(uint8_t* fm2pswlatble2m, uint8_t* fm2pswlatble1m)
{
    uint32_t localVal = REG_IP_RD(FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE_ADDR);

    *fm2pswlatble2m = (localVal & ((uint32_t)0x001F0000)) >> 16;
    *fm2pswlatble1m = (localVal & ((uint32_t)0x0000001F)) >> 0;
}

__INLINE uint8_t mdmhp_fm2p_swlat_ble_fm2p_swlat_ble2m_getf(void)
{
    uint32_t localVal = REG_IP_RD(FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE_ADDR);
    return ((localVal & ((uint32_t)0x001F0000)) >> 16);
}

__INLINE void mdmhp_fm2p_swlat_ble_fm2p_swlat_ble2m_setf(uint8_t fm2pswlatble2m)
{
    ASSERT_ERR((((uint32_t)fm2pswlatble2m << 16) & ~((uint32_t)0x001F0000)) == 0);
    REG_IP_WR(FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE_ADDR, (REG_IP_RD(FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE_ADDR) & ~((uint32_t)0x001F0000)) | ((uint32_t)fm2pswlatble2m << 16));
}

__INLINE uint8_t mdmhp_fm2p_swlat_ble_fm2p_swlat_ble1m_getf(void)
{
    uint32_t localVal = REG_IP_RD(FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE_ADDR);
    return ((localVal & ((uint32_t)0x0000001F)) >> 0);
}

__INLINE void mdmhp_fm2p_swlat_ble_fm2p_swlat_ble1m_setf(uint8_t fm2pswlatble1m)
{
    ASSERT_ERR((((uint32_t)fm2pswlatble1m << 0) & ~((uint32_t)0x0000001F)) == 0);
    REG_IP_WR(FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE_ADDR, (REG_IP_RD(FM2P_SWLAT_BLE_MDMHP_FM2P_SWLAT_BLE_ADDR) & ~((uint32_t)0x0000001F)) | ((uint32_t)fm2pswlatble1m << 0));
}

/**
 * @brief RXRSSI_THR_CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16       RSSI_SELTHR_BT   0xCC
 *  15:08     RSSI_SELTHR_BLE2   0xD0
 *  07:00     RSSI_SELTHR_BLE1   0xCE
 * </pre>
 */
#define RXRSSI_THR_CNTL_MDMHP_RXRSSI_THR_CNTL_ADDR   0x21020050
#define RXRSSI_THR_CNTL_MDMHP_RXRSSI_THR_CNTL_OFFSET 0x00000050
#define RXRSSI_THR_CNTL_MDMHP_RXRSSI_THR_CNTL_INDEX  0x00000014
#define RXRSSI_THR_CNTL_MDMHP_RXRSSI_THR_CNTL_RESET  0x00CCD0CE

__INLINE uint32_t mdmhp_rxrssi_thr_cntl_get(void)
{
    return REG_IP_RD(RXRSSI_THR_CNTL_MDMHP_RXRSSI_THR_CNTL_ADDR);
}

__INLINE void mdmhp_rxrssi_thr_cntl_set(uint32_t value)
{
    REG_IP_WR(RXRSSI_THR_CNTL_MDMHP_RXRSSI_THR_CNTL_ADDR, value);
}

// field definitions
#define RXRSSI_THR_CNTL_MDMHP_RSSI_SELTHR_BT_MASK     ((uint32_t)0x00FF0000)
#define RXRSSI_THR_CNTL_MDMHP_RSSI_SELTHR_BT_LSB      16
#define RXRSSI_THR_CNTL_MDMHP_RSSI_SELTHR_BT_WIDTH    ((uint32_t)0x00000008)
#define RXRSSI_THR_CNTL_MDMHP_RSSI_SELTHR_BLE2_MASK   ((uint32_t)0x0000FF00)
#define RXRSSI_THR_CNTL_MDMHP_RSSI_SELTHR_BLE2_LSB    8
#define RXRSSI_THR_CNTL_MDMHP_RSSI_SELTHR_BLE2_WIDTH  ((uint32_t)0x00000008)
#define RXRSSI_THR_CNTL_MDMHP_RSSI_SELTHR_BLE1_MASK   ((uint32_t)0x000000FF)
#define RXRSSI_THR_CNTL_MDMHP_RSSI_SELTHR_BLE1_LSB    0
#define RXRSSI_THR_CNTL_MDMHP_RSSI_SELTHR_BLE1_WIDTH  ((uint32_t)0x00000008)

#define RXRSSI_THR_CNTL_MDMHP_RSSI_SELTHR_BT_RST      0xCC
#define RXRSSI_THR_CNTL_MDMHP_RSSI_SELTHR_BLE2_RST    0xD0
#define RXRSSI_THR_CNTL_MDMHP_RSSI_SELTHR_BLE1_RST    0xCE

__INLINE void mdmhp_rxrssi_thr_cntl_pack(uint8_t rssiselthrbt, uint8_t rssiselthrble2, uint8_t rssiselthrble1)
{
    ASSERT_ERR((((uint32_t)rssiselthrbt << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)rssiselthrble2 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)rssiselthrble1 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(RXRSSI_THR_CNTL_MDMHP_RXRSSI_THR_CNTL_ADDR,  ((uint32_t)rssiselthrbt << 16) | ((uint32_t)rssiselthrble2 << 8) | ((uint32_t)rssiselthrble1 << 0));
}

__INLINE void mdmhp_rxrssi_thr_cntl_unpack(uint8_t* rssiselthrbt, uint8_t* rssiselthrble2, uint8_t* rssiselthrble1)
{
    uint32_t localVal = REG_IP_RD(RXRSSI_THR_CNTL_MDMHP_RXRSSI_THR_CNTL_ADDR);

    *rssiselthrbt = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *rssiselthrble2 = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *rssiselthrble1 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t mdmhp_rxrssi_thr_cntl_rssi_selthr_bt_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXRSSI_THR_CNTL_MDMHP_RXRSSI_THR_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void mdmhp_rxrssi_thr_cntl_rssi_selthr_bt_setf(uint8_t rssiselthrbt)
{
    ASSERT_ERR((((uint32_t)rssiselthrbt << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_IP_WR(RXRSSI_THR_CNTL_MDMHP_RXRSSI_THR_CNTL_ADDR, (REG_IP_RD(RXRSSI_THR_CNTL_MDMHP_RXRSSI_THR_CNTL_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)rssiselthrbt << 16));
}

__INLINE uint8_t mdmhp_rxrssi_thr_cntl_rssi_selthr_ble2_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXRSSI_THR_CNTL_MDMHP_RXRSSI_THR_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void mdmhp_rxrssi_thr_cntl_rssi_selthr_ble2_setf(uint8_t rssiselthrble2)
{
    ASSERT_ERR((((uint32_t)rssiselthrble2 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_IP_WR(RXRSSI_THR_CNTL_MDMHP_RXRSSI_THR_CNTL_ADDR, (REG_IP_RD(RXRSSI_THR_CNTL_MDMHP_RXRSSI_THR_CNTL_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)rssiselthrble2 << 8));
}

__INLINE uint8_t mdmhp_rxrssi_thr_cntl_rssi_selthr_ble1_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXRSSI_THR_CNTL_MDMHP_RXRSSI_THR_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void mdmhp_rxrssi_thr_cntl_rssi_selthr_ble1_setf(uint8_t rssiselthrble1)
{
    ASSERT_ERR((((uint32_t)rssiselthrble1 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(RXRSSI_THR_CNTL_MDMHP_RXRSSI_THR_CNTL_ADDR, (REG_IP_RD(RXRSSI_THR_CNTL_MDMHP_RXRSSI_THR_CNTL_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)rssiselthrble1 << 0));
}

/**
 * @brief RXRSSI_SCALING_CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24      RSSI_SCALING_LR   0x0
 *  23:16      RSSI_SCALING_BT   0x0
 *  15:08    RSSI_SCALING_BLE2   0x0
 *  07:00    RSSI_SCALING_BLE1   0x0
 * </pre>
 */
#define RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_ADDR   0x21020054
#define RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_OFFSET 0x00000054
#define RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_INDEX  0x00000015
#define RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_RESET  0x00000000

__INLINE uint32_t mdmhp_rxrssi_scaling_cntl_get(void)
{
    return REG_IP_RD(RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_ADDR);
}

__INLINE void mdmhp_rxrssi_scaling_cntl_set(uint32_t value)
{
    REG_IP_WR(RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_ADDR, value);
}

// field definitions
#define RXRSSI_SCALING_CNTL_MDMHP_RSSI_SCALING_LR_MASK     ((uint32_t)0xFF000000)
#define RXRSSI_SCALING_CNTL_MDMHP_RSSI_SCALING_LR_LSB      24
#define RXRSSI_SCALING_CNTL_MDMHP_RSSI_SCALING_LR_WIDTH    ((uint32_t)0x00000008)
#define RXRSSI_SCALING_CNTL_MDMHP_RSSI_SCALING_BT_MASK     ((uint32_t)0x00FF0000)
#define RXRSSI_SCALING_CNTL_MDMHP_RSSI_SCALING_BT_LSB      16
#define RXRSSI_SCALING_CNTL_MDMHP_RSSI_SCALING_BT_WIDTH    ((uint32_t)0x00000008)
#define RXRSSI_SCALING_CNTL_MDMHP_RSSI_SCALING_BLE2_MASK   ((uint32_t)0x0000FF00)
#define RXRSSI_SCALING_CNTL_MDMHP_RSSI_SCALING_BLE2_LSB    8
#define RXRSSI_SCALING_CNTL_MDMHP_RSSI_SCALING_BLE2_WIDTH  ((uint32_t)0x00000008)
#define RXRSSI_SCALING_CNTL_MDMHP_RSSI_SCALING_BLE1_MASK   ((uint32_t)0x000000FF)
#define RXRSSI_SCALING_CNTL_MDMHP_RSSI_SCALING_BLE1_LSB    0
#define RXRSSI_SCALING_CNTL_MDMHP_RSSI_SCALING_BLE1_WIDTH  ((uint32_t)0x00000008)

#define RXRSSI_SCALING_CNTL_MDMHP_RSSI_SCALING_LR_RST      0x0
#define RXRSSI_SCALING_CNTL_MDMHP_RSSI_SCALING_BT_RST      0x0
#define RXRSSI_SCALING_CNTL_MDMHP_RSSI_SCALING_BLE2_RST    0x0
#define RXRSSI_SCALING_CNTL_MDMHP_RSSI_SCALING_BLE1_RST    0x0

__INLINE void mdmhp_rxrssi_scaling_cntl_pack(uint8_t rssiscalinglr, uint8_t rssiscalingbt, uint8_t rssiscalingble2, uint8_t rssiscalingble1)
{
    ASSERT_ERR((((uint32_t)rssiscalinglr << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)rssiscalingbt << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)rssiscalingble2 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)rssiscalingble1 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_ADDR,  ((uint32_t)rssiscalinglr << 24) | ((uint32_t)rssiscalingbt << 16) | ((uint32_t)rssiscalingble2 << 8) | ((uint32_t)rssiscalingble1 << 0));
}

__INLINE void mdmhp_rxrssi_scaling_cntl_unpack(uint8_t* rssiscalinglr, uint8_t* rssiscalingbt, uint8_t* rssiscalingble2, uint8_t* rssiscalingble1)
{
    uint32_t localVal = REG_IP_RD(RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_ADDR);

    *rssiscalinglr = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *rssiscalingbt = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *rssiscalingble2 = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *rssiscalingble1 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t mdmhp_rxrssi_scaling_cntl_rssi_scaling_lr_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void mdmhp_rxrssi_scaling_cntl_rssi_scaling_lr_setf(uint8_t rssiscalinglr)
{
    ASSERT_ERR((((uint32_t)rssiscalinglr << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_IP_WR(RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_ADDR, (REG_IP_RD(RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)rssiscalinglr << 24));
}

__INLINE uint8_t mdmhp_rxrssi_scaling_cntl_rssi_scaling_bt_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void mdmhp_rxrssi_scaling_cntl_rssi_scaling_bt_setf(uint8_t rssiscalingbt)
{
    ASSERT_ERR((((uint32_t)rssiscalingbt << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_IP_WR(RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_ADDR, (REG_IP_RD(RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)rssiscalingbt << 16));
}

__INLINE uint8_t mdmhp_rxrssi_scaling_cntl_rssi_scaling_ble2_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void mdmhp_rxrssi_scaling_cntl_rssi_scaling_ble2_setf(uint8_t rssiscalingble2)
{
    ASSERT_ERR((((uint32_t)rssiscalingble2 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_IP_WR(RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_ADDR, (REG_IP_RD(RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)rssiscalingble2 << 8));
}

__INLINE uint8_t mdmhp_rxrssi_scaling_cntl_rssi_scaling_ble1_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void mdmhp_rxrssi_scaling_cntl_rssi_scaling_ble1_setf(uint8_t rssiscalingble1)
{
    ASSERT_ERR((((uint32_t)rssiscalingble1 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_ADDR, (REG_IP_RD(RXRSSI_SCALING_CNTL_MDMHP_RXRSSI_SCALING_CNTL_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)rssiscalingble1 << 0));
}

/**
 * @brief RXRSSI_SOP_CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24            SOP_KAL_R   0xA
 *  22:16            SOP_KAL_Q   0x6
 *  13:08              SOP_THR   0x4
 * </pre>
 */
#define RXRSSI_SOP_CNTL_MDMHP_RXRSSI_SOP_CNTL_ADDR   0x21020058
#define RXRSSI_SOP_CNTL_MDMHP_RXRSSI_SOP_CNTL_OFFSET 0x00000058
#define RXRSSI_SOP_CNTL_MDMHP_RXRSSI_SOP_CNTL_INDEX  0x00000016
#define RXRSSI_SOP_CNTL_MDMHP_RXRSSI_SOP_CNTL_RESET  0x0A060400

__INLINE uint32_t mdmhp_rxrssi_sop_cntl_get(void)
{
    return REG_IP_RD(RXRSSI_SOP_CNTL_MDMHP_RXRSSI_SOP_CNTL_ADDR);
}

__INLINE void mdmhp_rxrssi_sop_cntl_set(uint32_t value)
{
    REG_IP_WR(RXRSSI_SOP_CNTL_MDMHP_RXRSSI_SOP_CNTL_ADDR, value);
}

// field definitions
#define RXRSSI_SOP_CNTL_MDMHP_SOP_KAL_R_MASK   ((uint32_t)0xFF000000)
#define RXRSSI_SOP_CNTL_MDMHP_SOP_KAL_R_LSB    24
#define RXRSSI_SOP_CNTL_MDMHP_SOP_KAL_R_WIDTH  ((uint32_t)0x00000008)
#define RXRSSI_SOP_CNTL_MDMHP_SOP_KAL_Q_MASK   ((uint32_t)0x007F0000)
#define RXRSSI_SOP_CNTL_MDMHP_SOP_KAL_Q_LSB    16
#define RXRSSI_SOP_CNTL_MDMHP_SOP_KAL_Q_WIDTH  ((uint32_t)0x00000007)
#define RXRSSI_SOP_CNTL_MDMHP_SOP_THR_MASK     ((uint32_t)0x00003F00)
#define RXRSSI_SOP_CNTL_MDMHP_SOP_THR_LSB      8
#define RXRSSI_SOP_CNTL_MDMHP_SOP_THR_WIDTH    ((uint32_t)0x00000006)

#define RXRSSI_SOP_CNTL_MDMHP_SOP_KAL_R_RST    0xA
#define RXRSSI_SOP_CNTL_MDMHP_SOP_KAL_Q_RST    0x6
#define RXRSSI_SOP_CNTL_MDMHP_SOP_THR_RST      0x4

__INLINE void mdmhp_rxrssi_sop_cntl_pack(uint8_t sopkalr, uint8_t sopkalq, uint8_t sopthr)
{
    ASSERT_ERR((((uint32_t)sopkalr << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)sopkalq << 16) & ~((uint32_t)0x007F0000)) == 0);
    ASSERT_ERR((((uint32_t)sopthr << 8) & ~((uint32_t)0x00003F00)) == 0);
    REG_IP_WR(RXRSSI_SOP_CNTL_MDMHP_RXRSSI_SOP_CNTL_ADDR,  ((uint32_t)sopkalr << 24) | ((uint32_t)sopkalq << 16) | ((uint32_t)sopthr << 8));
}

__INLINE void mdmhp_rxrssi_sop_cntl_unpack(uint8_t* sopkalr, uint8_t* sopkalq, uint8_t* sopthr)
{
    uint32_t localVal = REG_IP_RD(RXRSSI_SOP_CNTL_MDMHP_RXRSSI_SOP_CNTL_ADDR);

    *sopkalr = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *sopkalq = (localVal & ((uint32_t)0x007F0000)) >> 16;
    *sopthr = (localVal & ((uint32_t)0x00003F00)) >> 8;
}

__INLINE uint8_t mdmhp_rxrssi_sop_cntl_sop_kal_r_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXRSSI_SOP_CNTL_MDMHP_RXRSSI_SOP_CNTL_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void mdmhp_rxrssi_sop_cntl_sop_kal_r_setf(uint8_t sopkalr)
{
    ASSERT_ERR((((uint32_t)sopkalr << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_IP_WR(RXRSSI_SOP_CNTL_MDMHP_RXRSSI_SOP_CNTL_ADDR, (REG_IP_RD(RXRSSI_SOP_CNTL_MDMHP_RXRSSI_SOP_CNTL_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)sopkalr << 24));
}

__INLINE uint8_t mdmhp_rxrssi_sop_cntl_sop_kal_q_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXRSSI_SOP_CNTL_MDMHP_RXRSSI_SOP_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x007F0000)) >> 16);
}

__INLINE void mdmhp_rxrssi_sop_cntl_sop_kal_q_setf(uint8_t sopkalq)
{
    ASSERT_ERR((((uint32_t)sopkalq << 16) & ~((uint32_t)0x007F0000)) == 0);
    REG_IP_WR(RXRSSI_SOP_CNTL_MDMHP_RXRSSI_SOP_CNTL_ADDR, (REG_IP_RD(RXRSSI_SOP_CNTL_MDMHP_RXRSSI_SOP_CNTL_ADDR) & ~((uint32_t)0x007F0000)) | ((uint32_t)sopkalq << 16));
}

__INLINE uint8_t mdmhp_rxrssi_sop_cntl_sop_thr_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXRSSI_SOP_CNTL_MDMHP_RXRSSI_SOP_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00003F00)) >> 8);
}

__INLINE void mdmhp_rxrssi_sop_cntl_sop_thr_setf(uint8_t sopthr)
{
    ASSERT_ERR((((uint32_t)sopthr << 8) & ~((uint32_t)0x00003F00)) == 0);
    REG_IP_WR(RXRSSI_SOP_CNTL_MDMHP_RXRSSI_SOP_CNTL_ADDR, (REG_IP_RD(RXRSSI_SOP_CNTL_MDMHP_RXRSSI_SOP_CNTL_ADDR) & ~((uint32_t)0x00003F00)) | ((uint32_t)sopthr << 8));
}

/**
 * @brief RXSOP_CNTL_BLE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     29   SOP_HIGHTHR_EN_BLE2   0
 *     28        SOP_MODE_BLE2   0
 *  23:16         SOP_DEL_BLE2   0x13
 *     13   SOP_HIGHTHR_EN_BLE1   0
 *     12        SOP_MODE_BLE1   0
 *  07:00         SOP_DEL_BLE1   0x13
 * </pre>
 */
#define RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR   0x2102005C
#define RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_OFFSET 0x0000005C
#define RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_INDEX  0x00000017
#define RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_RESET  0x00130013

__INLINE uint32_t mdmhp_rxsop_cntl_ble_get(void)
{
    return REG_IP_RD(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR);
}

__INLINE void mdmhp_rxsop_cntl_ble_set(uint32_t value)
{
    REG_IP_WR(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR, value);
}

// field definitions
#define RXSOP_CNTL_BLE_MDMHP_SOP_HIGHTHR_EN_BLE2_BIT    ((uint32_t)0x20000000)
#define RXSOP_CNTL_BLE_MDMHP_SOP_HIGHTHR_EN_BLE2_POS    29
#define RXSOP_CNTL_BLE_MDMHP_SOP_MODE_BLE2_BIT          ((uint32_t)0x10000000)
#define RXSOP_CNTL_BLE_MDMHP_SOP_MODE_BLE2_POS          28
#define RXSOP_CNTL_BLE_MDMHP_SOP_DEL_BLE2_MASK          ((uint32_t)0x00FF0000)
#define RXSOP_CNTL_BLE_MDMHP_SOP_DEL_BLE2_LSB           16
#define RXSOP_CNTL_BLE_MDMHP_SOP_DEL_BLE2_WIDTH         ((uint32_t)0x00000008)
#define RXSOP_CNTL_BLE_MDMHP_SOP_HIGHTHR_EN_BLE1_BIT    ((uint32_t)0x00002000)
#define RXSOP_CNTL_BLE_MDMHP_SOP_HIGHTHR_EN_BLE1_POS    13
#define RXSOP_CNTL_BLE_MDMHP_SOP_MODE_BLE1_BIT          ((uint32_t)0x00001000)
#define RXSOP_CNTL_BLE_MDMHP_SOP_MODE_BLE1_POS          12
#define RXSOP_CNTL_BLE_MDMHP_SOP_DEL_BLE1_MASK          ((uint32_t)0x000000FF)
#define RXSOP_CNTL_BLE_MDMHP_SOP_DEL_BLE1_LSB           0
#define RXSOP_CNTL_BLE_MDMHP_SOP_DEL_BLE1_WIDTH         ((uint32_t)0x00000008)

#define RXSOP_CNTL_BLE_MDMHP_SOP_HIGHTHR_EN_BLE2_RST    0x0
#define RXSOP_CNTL_BLE_MDMHP_SOP_MODE_BLE2_RST          0x0
#define RXSOP_CNTL_BLE_MDMHP_SOP_DEL_BLE2_RST           0x13
#define RXSOP_CNTL_BLE_MDMHP_SOP_HIGHTHR_EN_BLE1_RST    0x0
#define RXSOP_CNTL_BLE_MDMHP_SOP_MODE_BLE1_RST          0x0
#define RXSOP_CNTL_BLE_MDMHP_SOP_DEL_BLE1_RST           0x13

__INLINE void mdmhp_rxsop_cntl_ble_pack(uint8_t sophighthrenble2, uint8_t sopmodeble2, uint8_t sopdelble2, uint8_t sophighthrenble1, uint8_t sopmodeble1, uint8_t sopdelble1)
{
    ASSERT_ERR((((uint32_t)sophighthrenble2 << 29) & ~((uint32_t)0x20000000)) == 0);
    ASSERT_ERR((((uint32_t)sopmodeble2 << 28) & ~((uint32_t)0x10000000)) == 0);
    ASSERT_ERR((((uint32_t)sopdelble2 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)sophighthrenble1 << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)sopmodeble1 << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)sopdelble1 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR,  ((uint32_t)sophighthrenble2 << 29) | ((uint32_t)sopmodeble2 << 28) | ((uint32_t)sopdelble2 << 16) | ((uint32_t)sophighthrenble1 << 13) | ((uint32_t)sopmodeble1 << 12) | ((uint32_t)sopdelble1 << 0));
}

__INLINE void mdmhp_rxsop_cntl_ble_unpack(uint8_t* sophighthrenble2, uint8_t* sopmodeble2, uint8_t* sopdelble2, uint8_t* sophighthrenble1, uint8_t* sopmodeble1, uint8_t* sopdelble1)
{
    uint32_t localVal = REG_IP_RD(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR);

    *sophighthrenble2 = (localVal & ((uint32_t)0x20000000)) >> 29;
    *sopmodeble2 = (localVal & ((uint32_t)0x10000000)) >> 28;
    *sopdelble2 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *sophighthrenble1 = (localVal & ((uint32_t)0x00002000)) >> 13;
    *sopmodeble1 = (localVal & ((uint32_t)0x00001000)) >> 12;
    *sopdelble1 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t mdmhp_rxsop_cntl_ble_sop_highthr_en_ble2_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR);
    return ((localVal & ((uint32_t)0x20000000)) >> 29);
}

__INLINE void mdmhp_rxsop_cntl_ble_sop_highthr_en_ble2_setf(uint8_t sophighthrenble2)
{
    ASSERT_ERR((((uint32_t)sophighthrenble2 << 29) & ~((uint32_t)0x20000000)) == 0);
    REG_IP_WR(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR, (REG_IP_RD(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR) & ~((uint32_t)0x20000000)) | ((uint32_t)sophighthrenble2 << 29));
}

__INLINE uint8_t mdmhp_rxsop_cntl_ble_sop_mode_ble2_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void mdmhp_rxsop_cntl_ble_sop_mode_ble2_setf(uint8_t sopmodeble2)
{
    ASSERT_ERR((((uint32_t)sopmodeble2 << 28) & ~((uint32_t)0x10000000)) == 0);
    REG_IP_WR(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR, (REG_IP_RD(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)sopmodeble2 << 28));
}

__INLINE uint8_t mdmhp_rxsop_cntl_ble_sop_del_ble2_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void mdmhp_rxsop_cntl_ble_sop_del_ble2_setf(uint8_t sopdelble2)
{
    ASSERT_ERR((((uint32_t)sopdelble2 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_IP_WR(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR, (REG_IP_RD(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)sopdelble2 << 16));
}

__INLINE uint8_t mdmhp_rxsop_cntl_ble_sop_highthr_en_ble1_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void mdmhp_rxsop_cntl_ble_sop_highthr_en_ble1_setf(uint8_t sophighthrenble1)
{
    ASSERT_ERR((((uint32_t)sophighthrenble1 << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_IP_WR(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR, (REG_IP_RD(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)sophighthrenble1 << 13));
}

__INLINE uint8_t mdmhp_rxsop_cntl_ble_sop_mode_ble1_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void mdmhp_rxsop_cntl_ble_sop_mode_ble1_setf(uint8_t sopmodeble1)
{
    ASSERT_ERR((((uint32_t)sopmodeble1 << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_IP_WR(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR, (REG_IP_RD(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)sopmodeble1 << 12));
}

__INLINE uint8_t mdmhp_rxsop_cntl_ble_sop_del_ble1_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void mdmhp_rxsop_cntl_ble_sop_del_ble1_setf(uint8_t sopdelble1)
{
    ASSERT_ERR((((uint32_t)sopdelble1 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR, (REG_IP_RD(RXSOP_CNTL_BLE_MDMHP_RXSOP_CNTL_BLE_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)sopdelble1 << 0));
}

/**
 * @brief RXSOP_CNTL_LR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     13    SOP_HIGHTHR_EN_LR   1
 *     12          SOP_MODE_LR   1
 *  07:00           SOP_DEL_LR   0x13
 * </pre>
 */
#define RXSOP_CNTL_LR_MDMHP_RXSOP_CNTL_LR_ADDR   0x21020060
#define RXSOP_CNTL_LR_MDMHP_RXSOP_CNTL_LR_OFFSET 0x00000060
#define RXSOP_CNTL_LR_MDMHP_RXSOP_CNTL_LR_INDEX  0x00000018
#define RXSOP_CNTL_LR_MDMHP_RXSOP_CNTL_LR_RESET  0x00003013

__INLINE uint32_t mdmhp_rxsop_cntl_lr_get(void)
{
    return REG_IP_RD(RXSOP_CNTL_LR_MDMHP_RXSOP_CNTL_LR_ADDR);
}

__INLINE void mdmhp_rxsop_cntl_lr_set(uint32_t value)
{
    REG_IP_WR(RXSOP_CNTL_LR_MDMHP_RXSOP_CNTL_LR_ADDR, value);
}

// field definitions
#define RXSOP_CNTL_LR_MDMHP_SOP_HIGHTHR_EN_LR_BIT    ((uint32_t)0x00002000)
#define RXSOP_CNTL_LR_MDMHP_SOP_HIGHTHR_EN_LR_POS    13
#define RXSOP_CNTL_LR_MDMHP_SOP_MODE_LR_BIT          ((uint32_t)0x00001000)
#define RXSOP_CNTL_LR_MDMHP_SOP_MODE_LR_POS          12
#define RXSOP_CNTL_LR_MDMHP_SOP_DEL_LR_MASK          ((uint32_t)0x000000FF)
#define RXSOP_CNTL_LR_MDMHP_SOP_DEL_LR_LSB           0
#define RXSOP_CNTL_LR_MDMHP_SOP_DEL_LR_WIDTH         ((uint32_t)0x00000008)

#define RXSOP_CNTL_LR_MDMHP_SOP_HIGHTHR_EN_LR_RST    0x1
#define RXSOP_CNTL_LR_MDMHP_SOP_MODE_LR_RST          0x1
#define RXSOP_CNTL_LR_MDMHP_SOP_DEL_LR_RST           0x13

__INLINE void mdmhp_rxsop_cntl_lr_pack(uint8_t sophighthrenlr, uint8_t sopmodelr, uint8_t sopdellr)
{
    ASSERT_ERR((((uint32_t)sophighthrenlr << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)sopmodelr << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)sopdellr << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(RXSOP_CNTL_LR_MDMHP_RXSOP_CNTL_LR_ADDR,  ((uint32_t)sophighthrenlr << 13) | ((uint32_t)sopmodelr << 12) | ((uint32_t)sopdellr << 0));
}

__INLINE void mdmhp_rxsop_cntl_lr_unpack(uint8_t* sophighthrenlr, uint8_t* sopmodelr, uint8_t* sopdellr)
{
    uint32_t localVal = REG_IP_RD(RXSOP_CNTL_LR_MDMHP_RXSOP_CNTL_LR_ADDR);

    *sophighthrenlr = (localVal & ((uint32_t)0x00002000)) >> 13;
    *sopmodelr = (localVal & ((uint32_t)0x00001000)) >> 12;
    *sopdellr = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t mdmhp_rxsop_cntl_lr_sop_highthr_en_lr_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSOP_CNTL_LR_MDMHP_RXSOP_CNTL_LR_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void mdmhp_rxsop_cntl_lr_sop_highthr_en_lr_setf(uint8_t sophighthrenlr)
{
    ASSERT_ERR((((uint32_t)sophighthrenlr << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_IP_WR(RXSOP_CNTL_LR_MDMHP_RXSOP_CNTL_LR_ADDR, (REG_IP_RD(RXSOP_CNTL_LR_MDMHP_RXSOP_CNTL_LR_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)sophighthrenlr << 13));
}

__INLINE uint8_t mdmhp_rxsop_cntl_lr_sop_mode_lr_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSOP_CNTL_LR_MDMHP_RXSOP_CNTL_LR_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void mdmhp_rxsop_cntl_lr_sop_mode_lr_setf(uint8_t sopmodelr)
{
    ASSERT_ERR((((uint32_t)sopmodelr << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_IP_WR(RXSOP_CNTL_LR_MDMHP_RXSOP_CNTL_LR_ADDR, (REG_IP_RD(RXSOP_CNTL_LR_MDMHP_RXSOP_CNTL_LR_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)sopmodelr << 12));
}

__INLINE uint8_t mdmhp_rxsop_cntl_lr_sop_del_lr_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSOP_CNTL_LR_MDMHP_RXSOP_CNTL_LR_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void mdmhp_rxsop_cntl_lr_sop_del_lr_setf(uint8_t sopdellr)
{
    ASSERT_ERR((((uint32_t)sopdellr << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(RXSOP_CNTL_LR_MDMHP_RXSOP_CNTL_LR_ADDR, (REG_IP_RD(RXSOP_CNTL_LR_MDMHP_RXSOP_CNTL_LR_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)sopdellr << 0));
}

/**
 * @brief RXSOP_CNTL_BT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     13    SOP_HIGHTHR_EN_BT   0
 *     12          SOP_MODE_BT   0
 *  07:00           SOP_DEL_BT   0x13
 * </pre>
 */
#define RXSOP_CNTL_BT_MDMHP_RXSOP_CNTL_BT_ADDR   0x21020064
#define RXSOP_CNTL_BT_MDMHP_RXSOP_CNTL_BT_OFFSET 0x00000064
#define RXSOP_CNTL_BT_MDMHP_RXSOP_CNTL_BT_INDEX  0x00000019
#define RXSOP_CNTL_BT_MDMHP_RXSOP_CNTL_BT_RESET  0x00000013

__INLINE uint32_t mdmhp_rxsop_cntl_bt_get(void)
{
    return REG_IP_RD(RXSOP_CNTL_BT_MDMHP_RXSOP_CNTL_BT_ADDR);
}

__INLINE void mdmhp_rxsop_cntl_bt_set(uint32_t value)
{
    REG_IP_WR(RXSOP_CNTL_BT_MDMHP_RXSOP_CNTL_BT_ADDR, value);
}

// field definitions
#define RXSOP_CNTL_BT_MDMHP_SOP_HIGHTHR_EN_BT_BIT    ((uint32_t)0x00002000)
#define RXSOP_CNTL_BT_MDMHP_SOP_HIGHTHR_EN_BT_POS    13
#define RXSOP_CNTL_BT_MDMHP_SOP_MODE_BT_BIT          ((uint32_t)0x00001000)
#define RXSOP_CNTL_BT_MDMHP_SOP_MODE_BT_POS          12
#define RXSOP_CNTL_BT_MDMHP_SOP_DEL_BT_MASK          ((uint32_t)0x000000FF)
#define RXSOP_CNTL_BT_MDMHP_SOP_DEL_BT_LSB           0
#define RXSOP_CNTL_BT_MDMHP_SOP_DEL_BT_WIDTH         ((uint32_t)0x00000008)

#define RXSOP_CNTL_BT_MDMHP_SOP_HIGHTHR_EN_BT_RST    0x0
#define RXSOP_CNTL_BT_MDMHP_SOP_MODE_BT_RST          0x0
#define RXSOP_CNTL_BT_MDMHP_SOP_DEL_BT_RST           0x13

__INLINE void mdmhp_rxsop_cntl_bt_pack(uint8_t sophighthrenbt, uint8_t sopmodebt, uint8_t sopdelbt)
{
    ASSERT_ERR((((uint32_t)sophighthrenbt << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)sopmodebt << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)sopdelbt << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(RXSOP_CNTL_BT_MDMHP_RXSOP_CNTL_BT_ADDR,  ((uint32_t)sophighthrenbt << 13) | ((uint32_t)sopmodebt << 12) | ((uint32_t)sopdelbt << 0));
}

__INLINE void mdmhp_rxsop_cntl_bt_unpack(uint8_t* sophighthrenbt, uint8_t* sopmodebt, uint8_t* sopdelbt)
{
    uint32_t localVal = REG_IP_RD(RXSOP_CNTL_BT_MDMHP_RXSOP_CNTL_BT_ADDR);

    *sophighthrenbt = (localVal & ((uint32_t)0x00002000)) >> 13;
    *sopmodebt = (localVal & ((uint32_t)0x00001000)) >> 12;
    *sopdelbt = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t mdmhp_rxsop_cntl_bt_sop_highthr_en_bt_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSOP_CNTL_BT_MDMHP_RXSOP_CNTL_BT_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void mdmhp_rxsop_cntl_bt_sop_highthr_en_bt_setf(uint8_t sophighthrenbt)
{
    ASSERT_ERR((((uint32_t)sophighthrenbt << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_IP_WR(RXSOP_CNTL_BT_MDMHP_RXSOP_CNTL_BT_ADDR, (REG_IP_RD(RXSOP_CNTL_BT_MDMHP_RXSOP_CNTL_BT_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)sophighthrenbt << 13));
}

__INLINE uint8_t mdmhp_rxsop_cntl_bt_sop_mode_bt_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSOP_CNTL_BT_MDMHP_RXSOP_CNTL_BT_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void mdmhp_rxsop_cntl_bt_sop_mode_bt_setf(uint8_t sopmodebt)
{
    ASSERT_ERR((((uint32_t)sopmodebt << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_IP_WR(RXSOP_CNTL_BT_MDMHP_RXSOP_CNTL_BT_ADDR, (REG_IP_RD(RXSOP_CNTL_BT_MDMHP_RXSOP_CNTL_BT_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)sopmodebt << 12));
}

__INLINE uint8_t mdmhp_rxsop_cntl_bt_sop_del_bt_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSOP_CNTL_BT_MDMHP_RXSOP_CNTL_BT_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void mdmhp_rxsop_cntl_bt_sop_del_bt_setf(uint8_t sopdelbt)
{
    ASSERT_ERR((((uint32_t)sopdelbt << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(RXSOP_CNTL_BT_MDMHP_RXSOP_CNTL_BT_ADDR, (REG_IP_RD(RXSOP_CNTL_BT_MDMHP_RXSOP_CNTL_BT_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)sopdelbt << 0));
}

/**
 * @brief RXTSI_COEFF_LUT_CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31   LUT_RXTSI_COEFF_WEN   0
 *  28:24   LUT_RXTSI_COEFF_ADD   0x0
 *  13:00   LUT_RXTSI_COEFF_IN   0x0
 * </pre>
 */
#define RXTSI_COEFF_LUT_CNTL_MDMHP_RXTSI_COEFF_LUT_CNTL_ADDR   0x21020068
#define RXTSI_COEFF_LUT_CNTL_MDMHP_RXTSI_COEFF_LUT_CNTL_OFFSET 0x00000068
#define RXTSI_COEFF_LUT_CNTL_MDMHP_RXTSI_COEFF_LUT_CNTL_INDEX  0x0000001A
#define RXTSI_COEFF_LUT_CNTL_MDMHP_RXTSI_COEFF_LUT_CNTL_RESET  0x00000000

__INLINE uint32_t mdmhp_rxtsi_coeff_lut_cntl_get(void)
{
    return REG_IP_RD(RXTSI_COEFF_LUT_CNTL_MDMHP_RXTSI_COEFF_LUT_CNTL_ADDR);
}

__INLINE void mdmhp_rxtsi_coeff_lut_cntl_set(uint32_t value)
{
    REG_IP_WR(RXTSI_COEFF_LUT_CNTL_MDMHP_RXTSI_COEFF_LUT_CNTL_ADDR, value);
}

// field definitions
#define RXTSI_COEFF_LUT_CNTL_MDMHP_LUT_RXTSI_COEFF_WEN_BIT    ((uint32_t)0x80000000)
#define RXTSI_COEFF_LUT_CNTL_MDMHP_LUT_RXTSI_COEFF_WEN_POS    31
#define RXTSI_COEFF_LUT_CNTL_MDMHP_LUT_RXTSI_COEFF_ADD_MASK   ((uint32_t)0x1F000000)
#define RXTSI_COEFF_LUT_CNTL_MDMHP_LUT_RXTSI_COEFF_ADD_LSB    24
#define RXTSI_COEFF_LUT_CNTL_MDMHP_LUT_RXTSI_COEFF_ADD_WIDTH  ((uint32_t)0x00000005)
#define RXTSI_COEFF_LUT_CNTL_MDMHP_LUT_RXTSI_COEFF_IN_MASK    ((uint32_t)0x00003FFF)
#define RXTSI_COEFF_LUT_CNTL_MDMHP_LUT_RXTSI_COEFF_IN_LSB     0
#define RXTSI_COEFF_LUT_CNTL_MDMHP_LUT_RXTSI_COEFF_IN_WIDTH   ((uint32_t)0x0000000E)

#define RXTSI_COEFF_LUT_CNTL_MDMHP_LUT_RXTSI_COEFF_WEN_RST    0x0
#define RXTSI_COEFF_LUT_CNTL_MDMHP_LUT_RXTSI_COEFF_ADD_RST    0x0
#define RXTSI_COEFF_LUT_CNTL_MDMHP_LUT_RXTSI_COEFF_IN_RST     0x0

__INLINE void mdmhp_rxtsi_coeff_lut_cntl_pack(uint8_t lutrxtsicoeffwen, uint8_t lutrxtsicoeffadd, uint16_t lutrxtsicoeffin)
{
    ASSERT_ERR((((uint32_t)lutrxtsicoeffwen << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)lutrxtsicoeffadd << 24) & ~((uint32_t)0x1F000000)) == 0);
    ASSERT_ERR((((uint32_t)lutrxtsicoeffin << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_IP_WR(RXTSI_COEFF_LUT_CNTL_MDMHP_RXTSI_COEFF_LUT_CNTL_ADDR,  ((uint32_t)lutrxtsicoeffwen << 31) | ((uint32_t)lutrxtsicoeffadd << 24) | ((uint32_t)lutrxtsicoeffin << 0));
}

__INLINE void mdmhp_rxtsi_coeff_lut_cntl_unpack(uint8_t* lutrxtsicoeffwen, uint8_t* lutrxtsicoeffadd, uint16_t* lutrxtsicoeffin)
{
    uint32_t localVal = REG_IP_RD(RXTSI_COEFF_LUT_CNTL_MDMHP_RXTSI_COEFF_LUT_CNTL_ADDR);

    *lutrxtsicoeffwen = (localVal & ((uint32_t)0x80000000)) >> 31;
    *lutrxtsicoeffadd = (localVal & ((uint32_t)0x1F000000)) >> 24;
    *lutrxtsicoeffin = (localVal & ((uint32_t)0x00003FFF)) >> 0;
}

__INLINE uint8_t mdmhp_rxtsi_coeff_lut_cntl_lut_rxtsi_coeff_wen_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXTSI_COEFF_LUT_CNTL_MDMHP_RXTSI_COEFF_LUT_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void mdmhp_rxtsi_coeff_lut_cntl_lut_rxtsi_coeff_wen_setf(uint8_t lutrxtsicoeffwen)
{
    ASSERT_ERR((((uint32_t)lutrxtsicoeffwen << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_IP_WR(RXTSI_COEFF_LUT_CNTL_MDMHP_RXTSI_COEFF_LUT_CNTL_ADDR, (REG_IP_RD(RXTSI_COEFF_LUT_CNTL_MDMHP_RXTSI_COEFF_LUT_CNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)lutrxtsicoeffwen << 31));
}

__INLINE uint8_t mdmhp_rxtsi_coeff_lut_cntl_lut_rxtsi_coeff_add_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXTSI_COEFF_LUT_CNTL_MDMHP_RXTSI_COEFF_LUT_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x1F000000)) >> 24);
}

__INLINE void mdmhp_rxtsi_coeff_lut_cntl_lut_rxtsi_coeff_add_setf(uint8_t lutrxtsicoeffadd)
{
    ASSERT_ERR((((uint32_t)lutrxtsicoeffadd << 24) & ~((uint32_t)0x1F000000)) == 0);
    REG_IP_WR(RXTSI_COEFF_LUT_CNTL_MDMHP_RXTSI_COEFF_LUT_CNTL_ADDR, (REG_IP_RD(RXTSI_COEFF_LUT_CNTL_MDMHP_RXTSI_COEFF_LUT_CNTL_ADDR) & ~((uint32_t)0x1F000000)) | ((uint32_t)lutrxtsicoeffadd << 24));
}

__INLINE uint16_t mdmhp_rxtsi_coeff_lut_cntl_lut_rxtsi_coeff_in_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXTSI_COEFF_LUT_CNTL_MDMHP_RXTSI_COEFF_LUT_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00003FFF)) >> 0);
}

__INLINE void mdmhp_rxtsi_coeff_lut_cntl_lut_rxtsi_coeff_in_setf(uint16_t lutrxtsicoeffin)
{
    ASSERT_ERR((((uint32_t)lutrxtsicoeffin << 0) & ~((uint32_t)0x00003FFF)) == 0);
    REG_IP_WR(RXTSI_COEFF_LUT_CNTL_MDMHP_RXTSI_COEFF_LUT_CNTL_ADDR, (REG_IP_RD(RXTSI_COEFF_LUT_CNTL_MDMHP_RXTSI_COEFF_LUT_CNTL_ADDR) & ~((uint32_t)0x00003FFF)) | ((uint32_t)lutrxtsicoeffin << 0));
}

/**
 * @brief RXTSI_COEFF_LUT_VAL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  13:00   LUT_RXTSI_COEFF_VAL   0x0
 * </pre>
 */
#define RXTSI_COEFF_LUT_VAL_MDMHP_RXTSI_COEFF_LUT_VAL_ADDR   0x2102006C
#define RXTSI_COEFF_LUT_VAL_MDMHP_RXTSI_COEFF_LUT_VAL_OFFSET 0x0000006C
#define RXTSI_COEFF_LUT_VAL_MDMHP_RXTSI_COEFF_LUT_VAL_INDEX  0x0000001B
#define RXTSI_COEFF_LUT_VAL_MDMHP_RXTSI_COEFF_LUT_VAL_RESET  0x00000000

__INLINE uint32_t mdmhp_rxtsi_coeff_lut_val_get(void)
{
    return REG_IP_RD(RXTSI_COEFF_LUT_VAL_MDMHP_RXTSI_COEFF_LUT_VAL_ADDR);
}

// field definitions
#define RXTSI_COEFF_LUT_VAL_MDMHP_LUT_RXTSI_COEFF_VAL_MASK   ((uint32_t)0x00003FFF)
#define RXTSI_COEFF_LUT_VAL_MDMHP_LUT_RXTSI_COEFF_VAL_LSB    0
#define RXTSI_COEFF_LUT_VAL_MDMHP_LUT_RXTSI_COEFF_VAL_WIDTH  ((uint32_t)0x0000000E)

#define RXTSI_COEFF_LUT_VAL_MDMHP_LUT_RXTSI_COEFF_VAL_RST    0x0

__INLINE uint16_t mdmhp_rxtsi_coeff_lut_val_lut_rxtsi_coeff_val_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXTSI_COEFF_LUT_VAL_MDMHP_RXTSI_COEFF_LUT_VAL_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x00003FFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief RXSWC_COEFF_TAU0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:24   SWC_COEFF_tau3_D11   0x15
 *  22:16   SWC_COEFF_TAU0_D21   0x1F
 *  14:08   SWC_COEFF_TAU0_D12   0x8
 *  06:00   SWC_COEFF_TAU0_D11   0x6
 * </pre>
 */
#define RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_ADDR   0x210200A0
#define RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_OFFSET 0x000000A0
#define RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_INDEX  0x00000028
#define RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_RESET  0x151F0806

__INLINE uint32_t mdmhp_rxswc_coeff_tau0_get(void)
{
    return REG_IP_RD(RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_ADDR);
}

__INLINE void mdmhp_rxswc_coeff_tau0_set(uint32_t value)
{
    REG_IP_WR(RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_ADDR, value);
}

// field definitions
#define RXSWC_COEFF_TAU0_MDMHP_SWC_COEFF_TAU_3_D_11_MASK   ((uint32_t)0x7F000000)
#define RXSWC_COEFF_TAU0_MDMHP_SWC_COEFF_TAU_3_D_11_LSB    24
#define RXSWC_COEFF_TAU0_MDMHP_SWC_COEFF_TAU_3_D_11_WIDTH  ((uint32_t)0x00000007)
#define RXSWC_COEFF_TAU0_MDMHP_SWC_COEFF_TAU0_D21_MASK     ((uint32_t)0x007F0000)
#define RXSWC_COEFF_TAU0_MDMHP_SWC_COEFF_TAU0_D21_LSB      16
#define RXSWC_COEFF_TAU0_MDMHP_SWC_COEFF_TAU0_D21_WIDTH    ((uint32_t)0x00000007)
#define RXSWC_COEFF_TAU0_MDMHP_SWC_COEFF_TAU0_D12_MASK     ((uint32_t)0x00007F00)
#define RXSWC_COEFF_TAU0_MDMHP_SWC_COEFF_TAU0_D12_LSB      8
#define RXSWC_COEFF_TAU0_MDMHP_SWC_COEFF_TAU0_D12_WIDTH    ((uint32_t)0x00000007)
#define RXSWC_COEFF_TAU0_MDMHP_SWC_COEFF_TAU0_D11_MASK     ((uint32_t)0x0000007F)
#define RXSWC_COEFF_TAU0_MDMHP_SWC_COEFF_TAU0_D11_LSB      0
#define RXSWC_COEFF_TAU0_MDMHP_SWC_COEFF_TAU0_D11_WIDTH    ((uint32_t)0x00000007)

#define RXSWC_COEFF_TAU0_MDMHP_SWC_COEFF_TAU_3_D_11_RST    0x15
#define RXSWC_COEFF_TAU0_MDMHP_SWC_COEFF_TAU0_D21_RST      0x1F
#define RXSWC_COEFF_TAU0_MDMHP_SWC_COEFF_TAU0_D12_RST      0x8
#define RXSWC_COEFF_TAU0_MDMHP_SWC_COEFF_TAU0_D11_RST      0x6

__INLINE void mdmhp_rxswc_coeff_tau0_pack(uint8_t swccoefftau3d11, uint8_t swccoefftau0d21, uint8_t swccoefftau0d12, uint8_t swccoefftau0d11)
{
    ASSERT_ERR((((uint32_t)swccoefftau3d11 << 24) & ~((uint32_t)0x7F000000)) == 0);
    ASSERT_ERR((((uint32_t)swccoefftau0d21 << 16) & ~((uint32_t)0x007F0000)) == 0);
    ASSERT_ERR((((uint32_t)swccoefftau0d12 << 8) & ~((uint32_t)0x00007F00)) == 0);
    ASSERT_ERR((((uint32_t)swccoefftau0d11 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_IP_WR(RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_ADDR,  ((uint32_t)swccoefftau3d11 << 24) | ((uint32_t)swccoefftau0d21 << 16) | ((uint32_t)swccoefftau0d12 << 8) | ((uint32_t)swccoefftau0d11 << 0));
}

__INLINE void mdmhp_rxswc_coeff_tau0_unpack(uint8_t* swccoefftau3d11, uint8_t* swccoefftau0d21, uint8_t* swccoefftau0d12, uint8_t* swccoefftau0d11)
{
    uint32_t localVal = REG_IP_RD(RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_ADDR);

    *swccoefftau3d11 = (localVal & ((uint32_t)0x7F000000)) >> 24;
    *swccoefftau0d21 = (localVal & ((uint32_t)0x007F0000)) >> 16;
    *swccoefftau0d12 = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *swccoefftau0d11 = (localVal & ((uint32_t)0x0000007F)) >> 0;
}

__INLINE uint8_t mdmhp_rxswc_coeff_tau0_swc_coeff_tau_3_d_11_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_ADDR);
    return ((localVal & ((uint32_t)0x7F000000)) >> 24);
}

__INLINE void mdmhp_rxswc_coeff_tau0_swc_coeff_tau_3_d_11_setf(uint8_t swccoefftau3d11)
{
    ASSERT_ERR((((uint32_t)swccoefftau3d11 << 24) & ~((uint32_t)0x7F000000)) == 0);
    REG_IP_WR(RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_ADDR, (REG_IP_RD(RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_ADDR) & ~((uint32_t)0x7F000000)) | ((uint32_t)swccoefftau3d11 << 24));
}

__INLINE uint8_t mdmhp_rxswc_coeff_tau0_swc_coeff_tau0_d21_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_ADDR);
    return ((localVal & ((uint32_t)0x007F0000)) >> 16);
}

__INLINE void mdmhp_rxswc_coeff_tau0_swc_coeff_tau0_d21_setf(uint8_t swccoefftau0d21)
{
    ASSERT_ERR((((uint32_t)swccoefftau0d21 << 16) & ~((uint32_t)0x007F0000)) == 0);
    REG_IP_WR(RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_ADDR, (REG_IP_RD(RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_ADDR) & ~((uint32_t)0x007F0000)) | ((uint32_t)swccoefftau0d21 << 16));
}

__INLINE uint8_t mdmhp_rxswc_coeff_tau0_swc_coeff_tau0_d12_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void mdmhp_rxswc_coeff_tau0_swc_coeff_tau0_d12_setf(uint8_t swccoefftau0d12)
{
    ASSERT_ERR((((uint32_t)swccoefftau0d12 << 8) & ~((uint32_t)0x00007F00)) == 0);
    REG_IP_WR(RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_ADDR, (REG_IP_RD(RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)swccoefftau0d12 << 8));
}

__INLINE uint8_t mdmhp_rxswc_coeff_tau0_swc_coeff_tau0_d11_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_ADDR);
    return ((localVal & ((uint32_t)0x0000007F)) >> 0);
}

__INLINE void mdmhp_rxswc_coeff_tau0_swc_coeff_tau0_d11_setf(uint8_t swccoefftau0d11)
{
    ASSERT_ERR((((uint32_t)swccoefftau0d11 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_IP_WR(RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_ADDR, (REG_IP_RD(RXSWC_COEFF_TAU0_MDMHP_RXSWC_COEFF_TAU0_ADDR) & ~((uint32_t)0x0000007F)) | ((uint32_t)swccoefftau0d11 << 0));
}

/**
 * @brief RXSWC_COEFF_TAU1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:24   SWC_COEFF_tau3_D12   0x1
 *  22:16   SWC_COEFF_TAU1_D21   0x24
 *  14:08   SWC_COEFF_TAU1_D12   0x4
 *  06:00   SWC_COEFF_TAU1_D11   0xB
 * </pre>
 */
#define RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_ADDR   0x210200A4
#define RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_OFFSET 0x000000A4
#define RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_INDEX  0x00000029
#define RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_RESET  0x0124040B

__INLINE uint32_t mdmhp_rxswc_coeff_tau1_get(void)
{
    return REG_IP_RD(RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_ADDR);
}

__INLINE void mdmhp_rxswc_coeff_tau1_set(uint32_t value)
{
    REG_IP_WR(RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_ADDR, value);
}

// field definitions
#define RXSWC_COEFF_TAU1_MDMHP_SWC_COEFF_TAU_3_D_12_MASK   ((uint32_t)0x7F000000)
#define RXSWC_COEFF_TAU1_MDMHP_SWC_COEFF_TAU_3_D_12_LSB    24
#define RXSWC_COEFF_TAU1_MDMHP_SWC_COEFF_TAU_3_D_12_WIDTH  ((uint32_t)0x00000007)
#define RXSWC_COEFF_TAU1_MDMHP_SWC_COEFF_TAU1_D21_MASK     ((uint32_t)0x007F0000)
#define RXSWC_COEFF_TAU1_MDMHP_SWC_COEFF_TAU1_D21_LSB      16
#define RXSWC_COEFF_TAU1_MDMHP_SWC_COEFF_TAU1_D21_WIDTH    ((uint32_t)0x00000007)
#define RXSWC_COEFF_TAU1_MDMHP_SWC_COEFF_TAU1_D12_MASK     ((uint32_t)0x00007F00)
#define RXSWC_COEFF_TAU1_MDMHP_SWC_COEFF_TAU1_D12_LSB      8
#define RXSWC_COEFF_TAU1_MDMHP_SWC_COEFF_TAU1_D12_WIDTH    ((uint32_t)0x00000007)
#define RXSWC_COEFF_TAU1_MDMHP_SWC_COEFF_TAU1_D11_MASK     ((uint32_t)0x0000007F)
#define RXSWC_COEFF_TAU1_MDMHP_SWC_COEFF_TAU1_D11_LSB      0
#define RXSWC_COEFF_TAU1_MDMHP_SWC_COEFF_TAU1_D11_WIDTH    ((uint32_t)0x00000007)

#define RXSWC_COEFF_TAU1_MDMHP_SWC_COEFF_TAU_3_D_12_RST    0x1
#define RXSWC_COEFF_TAU1_MDMHP_SWC_COEFF_TAU1_D21_RST      0x24
#define RXSWC_COEFF_TAU1_MDMHP_SWC_COEFF_TAU1_D12_RST      0x4
#define RXSWC_COEFF_TAU1_MDMHP_SWC_COEFF_TAU1_D11_RST      0xB

__INLINE void mdmhp_rxswc_coeff_tau1_pack(uint8_t swccoefftau3d12, uint8_t swccoefftau1d21, uint8_t swccoefftau1d12, uint8_t swccoefftau1d11)
{
    ASSERT_ERR((((uint32_t)swccoefftau3d12 << 24) & ~((uint32_t)0x7F000000)) == 0);
    ASSERT_ERR((((uint32_t)swccoefftau1d21 << 16) & ~((uint32_t)0x007F0000)) == 0);
    ASSERT_ERR((((uint32_t)swccoefftau1d12 << 8) & ~((uint32_t)0x00007F00)) == 0);
    ASSERT_ERR((((uint32_t)swccoefftau1d11 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_IP_WR(RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_ADDR,  ((uint32_t)swccoefftau3d12 << 24) | ((uint32_t)swccoefftau1d21 << 16) | ((uint32_t)swccoefftau1d12 << 8) | ((uint32_t)swccoefftau1d11 << 0));
}

__INLINE void mdmhp_rxswc_coeff_tau1_unpack(uint8_t* swccoefftau3d12, uint8_t* swccoefftau1d21, uint8_t* swccoefftau1d12, uint8_t* swccoefftau1d11)
{
    uint32_t localVal = REG_IP_RD(RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_ADDR);

    *swccoefftau3d12 = (localVal & ((uint32_t)0x7F000000)) >> 24;
    *swccoefftau1d21 = (localVal & ((uint32_t)0x007F0000)) >> 16;
    *swccoefftau1d12 = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *swccoefftau1d11 = (localVal & ((uint32_t)0x0000007F)) >> 0;
}

__INLINE uint8_t mdmhp_rxswc_coeff_tau1_swc_coeff_tau_3_d_12_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_ADDR);
    return ((localVal & ((uint32_t)0x7F000000)) >> 24);
}

__INLINE void mdmhp_rxswc_coeff_tau1_swc_coeff_tau_3_d_12_setf(uint8_t swccoefftau3d12)
{
    ASSERT_ERR((((uint32_t)swccoefftau3d12 << 24) & ~((uint32_t)0x7F000000)) == 0);
    REG_IP_WR(RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_ADDR, (REG_IP_RD(RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_ADDR) & ~((uint32_t)0x7F000000)) | ((uint32_t)swccoefftau3d12 << 24));
}

__INLINE uint8_t mdmhp_rxswc_coeff_tau1_swc_coeff_tau1_d21_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_ADDR);
    return ((localVal & ((uint32_t)0x007F0000)) >> 16);
}

__INLINE void mdmhp_rxswc_coeff_tau1_swc_coeff_tau1_d21_setf(uint8_t swccoefftau1d21)
{
    ASSERT_ERR((((uint32_t)swccoefftau1d21 << 16) & ~((uint32_t)0x007F0000)) == 0);
    REG_IP_WR(RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_ADDR, (REG_IP_RD(RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_ADDR) & ~((uint32_t)0x007F0000)) | ((uint32_t)swccoefftau1d21 << 16));
}

__INLINE uint8_t mdmhp_rxswc_coeff_tau1_swc_coeff_tau1_d12_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void mdmhp_rxswc_coeff_tau1_swc_coeff_tau1_d12_setf(uint8_t swccoefftau1d12)
{
    ASSERT_ERR((((uint32_t)swccoefftau1d12 << 8) & ~((uint32_t)0x00007F00)) == 0);
    REG_IP_WR(RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_ADDR, (REG_IP_RD(RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)swccoefftau1d12 << 8));
}

__INLINE uint8_t mdmhp_rxswc_coeff_tau1_swc_coeff_tau1_d11_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_ADDR);
    return ((localVal & ((uint32_t)0x0000007F)) >> 0);
}

__INLINE void mdmhp_rxswc_coeff_tau1_swc_coeff_tau1_d11_setf(uint8_t swccoefftau1d11)
{
    ASSERT_ERR((((uint32_t)swccoefftau1d11 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_IP_WR(RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_ADDR, (REG_IP_RD(RXSWC_COEFF_TAU1_MDMHP_RXSWC_COEFF_TAU1_ADDR) & ~((uint32_t)0x0000007F)) | ((uint32_t)swccoefftau1d11 << 0));
}

/**
 * @brief RXSWC_COEFF_TAU2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:24   SWC_COEFF_tau3_D21   0x2F
 *  22:16   SWC_COEFF_TAU2_D21   0x2A
 *  14:08   SWC_COEFF_TAU2_D12   0x2
 *  06:00   SWC_COEFF_TAU2_D11   0x10
 * </pre>
 */
#define RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_ADDR   0x210200A8
#define RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_OFFSET 0x000000A8
#define RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_INDEX  0x0000002A
#define RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_RESET  0x2F2A0210

__INLINE uint32_t mdmhp_rxswc_coeff_tau2_get(void)
{
    return REG_IP_RD(RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_ADDR);
}

__INLINE void mdmhp_rxswc_coeff_tau2_set(uint32_t value)
{
    REG_IP_WR(RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_ADDR, value);
}

// field definitions
#define RXSWC_COEFF_TAU2_MDMHP_SWC_COEFF_TAU_3_D_21_MASK   ((uint32_t)0x7F000000)
#define RXSWC_COEFF_TAU2_MDMHP_SWC_COEFF_TAU_3_D_21_LSB    24
#define RXSWC_COEFF_TAU2_MDMHP_SWC_COEFF_TAU_3_D_21_WIDTH  ((uint32_t)0x00000007)
#define RXSWC_COEFF_TAU2_MDMHP_SWC_COEFF_TAU2_D21_MASK     ((uint32_t)0x007F0000)
#define RXSWC_COEFF_TAU2_MDMHP_SWC_COEFF_TAU2_D21_LSB      16
#define RXSWC_COEFF_TAU2_MDMHP_SWC_COEFF_TAU2_D21_WIDTH    ((uint32_t)0x00000007)
#define RXSWC_COEFF_TAU2_MDMHP_SWC_COEFF_TAU2_D12_MASK     ((uint32_t)0x00007F00)
#define RXSWC_COEFF_TAU2_MDMHP_SWC_COEFF_TAU2_D12_LSB      8
#define RXSWC_COEFF_TAU2_MDMHP_SWC_COEFF_TAU2_D12_WIDTH    ((uint32_t)0x00000007)
#define RXSWC_COEFF_TAU2_MDMHP_SWC_COEFF_TAU2_D11_MASK     ((uint32_t)0x0000007F)
#define RXSWC_COEFF_TAU2_MDMHP_SWC_COEFF_TAU2_D11_LSB      0
#define RXSWC_COEFF_TAU2_MDMHP_SWC_COEFF_TAU2_D11_WIDTH    ((uint32_t)0x00000007)

#define RXSWC_COEFF_TAU2_MDMHP_SWC_COEFF_TAU_3_D_21_RST    0x2F
#define RXSWC_COEFF_TAU2_MDMHP_SWC_COEFF_TAU2_D21_RST      0x2A
#define RXSWC_COEFF_TAU2_MDMHP_SWC_COEFF_TAU2_D12_RST      0x2
#define RXSWC_COEFF_TAU2_MDMHP_SWC_COEFF_TAU2_D11_RST      0x10

__INLINE void mdmhp_rxswc_coeff_tau2_pack(uint8_t swccoefftau3d21, uint8_t swccoefftau2d21, uint8_t swccoefftau2d12, uint8_t swccoefftau2d11)
{
    ASSERT_ERR((((uint32_t)swccoefftau3d21 << 24) & ~((uint32_t)0x7F000000)) == 0);
    ASSERT_ERR((((uint32_t)swccoefftau2d21 << 16) & ~((uint32_t)0x007F0000)) == 0);
    ASSERT_ERR((((uint32_t)swccoefftau2d12 << 8) & ~((uint32_t)0x00007F00)) == 0);
    ASSERT_ERR((((uint32_t)swccoefftau2d11 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_IP_WR(RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_ADDR,  ((uint32_t)swccoefftau3d21 << 24) | ((uint32_t)swccoefftau2d21 << 16) | ((uint32_t)swccoefftau2d12 << 8) | ((uint32_t)swccoefftau2d11 << 0));
}

__INLINE void mdmhp_rxswc_coeff_tau2_unpack(uint8_t* swccoefftau3d21, uint8_t* swccoefftau2d21, uint8_t* swccoefftau2d12, uint8_t* swccoefftau2d11)
{
    uint32_t localVal = REG_IP_RD(RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_ADDR);

    *swccoefftau3d21 = (localVal & ((uint32_t)0x7F000000)) >> 24;
    *swccoefftau2d21 = (localVal & ((uint32_t)0x007F0000)) >> 16;
    *swccoefftau2d12 = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *swccoefftau2d11 = (localVal & ((uint32_t)0x0000007F)) >> 0;
}

__INLINE uint8_t mdmhp_rxswc_coeff_tau2_swc_coeff_tau_3_d_21_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_ADDR);
    return ((localVal & ((uint32_t)0x7F000000)) >> 24);
}

__INLINE void mdmhp_rxswc_coeff_tau2_swc_coeff_tau_3_d_21_setf(uint8_t swccoefftau3d21)
{
    ASSERT_ERR((((uint32_t)swccoefftau3d21 << 24) & ~((uint32_t)0x7F000000)) == 0);
    REG_IP_WR(RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_ADDR, (REG_IP_RD(RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_ADDR) & ~((uint32_t)0x7F000000)) | ((uint32_t)swccoefftau3d21 << 24));
}

__INLINE uint8_t mdmhp_rxswc_coeff_tau2_swc_coeff_tau2_d21_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_ADDR);
    return ((localVal & ((uint32_t)0x007F0000)) >> 16);
}

__INLINE void mdmhp_rxswc_coeff_tau2_swc_coeff_tau2_d21_setf(uint8_t swccoefftau2d21)
{
    ASSERT_ERR((((uint32_t)swccoefftau2d21 << 16) & ~((uint32_t)0x007F0000)) == 0);
    REG_IP_WR(RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_ADDR, (REG_IP_RD(RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_ADDR) & ~((uint32_t)0x007F0000)) | ((uint32_t)swccoefftau2d21 << 16));
}

__INLINE uint8_t mdmhp_rxswc_coeff_tau2_swc_coeff_tau2_d12_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void mdmhp_rxswc_coeff_tau2_swc_coeff_tau2_d12_setf(uint8_t swccoefftau2d12)
{
    ASSERT_ERR((((uint32_t)swccoefftau2d12 << 8) & ~((uint32_t)0x00007F00)) == 0);
    REG_IP_WR(RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_ADDR, (REG_IP_RD(RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)swccoefftau2d12 << 8));
}

__INLINE uint8_t mdmhp_rxswc_coeff_tau2_swc_coeff_tau2_d11_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_ADDR);
    return ((localVal & ((uint32_t)0x0000007F)) >> 0);
}

__INLINE void mdmhp_rxswc_coeff_tau2_swc_coeff_tau2_d11_setf(uint8_t swccoefftau2d11)
{
    ASSERT_ERR((((uint32_t)swccoefftau2d11 << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_IP_WR(RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_ADDR, (REG_IP_RD(RXSWC_COEFF_TAU2_MDMHP_RXSWC_COEFF_TAU2_ADDR) & ~((uint32_t)0x0000007F)) | ((uint32_t)swccoefftau2d11 << 0));
}

/**
 * @brief RXSYNC_CNTL_BLE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31          SYNC_MIN_EN   0
 *  28:16   SYNC_TSI_THRES_BLE   0x800
 *  15:12   SYNC_NB_ERR_MAX_BLE   0x0
 *  09:00   SYNC_LLHOOD_THRES_BLE   0x11A
 * </pre>
 */
#define RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_ADDR   0x210200AC
#define RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_OFFSET 0x000000AC
#define RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_INDEX  0x0000002B
#define RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_RESET  0x0800011A

__INLINE uint32_t mdmhp_rxsync_cntl_ble_get(void)
{
    return REG_IP_RD(RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_ADDR);
}

__INLINE void mdmhp_rxsync_cntl_ble_set(uint32_t value)
{
    REG_IP_WR(RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_ADDR, value);
}

// field definitions
#define RXSYNC_CNTL_BLE_MDMHP_SYNC_MIN_EN_BIT              ((uint32_t)0x80000000)
#define RXSYNC_CNTL_BLE_MDMHP_SYNC_MIN_EN_POS              31
#define RXSYNC_CNTL_BLE_MDMHP_SYNC_TSI_THRES_BLE_MASK      ((uint32_t)0x1FFF0000)
#define RXSYNC_CNTL_BLE_MDMHP_SYNC_TSI_THRES_BLE_LSB       16
#define RXSYNC_CNTL_BLE_MDMHP_SYNC_TSI_THRES_BLE_WIDTH     ((uint32_t)0x0000000D)
#define RXSYNC_CNTL_BLE_MDMHP_SYNC_NB_ERR_MAX_BLE_MASK     ((uint32_t)0x0000F000)
#define RXSYNC_CNTL_BLE_MDMHP_SYNC_NB_ERR_MAX_BLE_LSB      12
#define RXSYNC_CNTL_BLE_MDMHP_SYNC_NB_ERR_MAX_BLE_WIDTH    ((uint32_t)0x00000004)
#define RXSYNC_CNTL_BLE_MDMHP_SYNC_LLHOOD_THRES_BLE_MASK   ((uint32_t)0x000003FF)
#define RXSYNC_CNTL_BLE_MDMHP_SYNC_LLHOOD_THRES_BLE_LSB    0
#define RXSYNC_CNTL_BLE_MDMHP_SYNC_LLHOOD_THRES_BLE_WIDTH  ((uint32_t)0x0000000A)

#define RXSYNC_CNTL_BLE_MDMHP_SYNC_MIN_EN_RST              0x0
#define RXSYNC_CNTL_BLE_MDMHP_SYNC_TSI_THRES_BLE_RST       0x800
#define RXSYNC_CNTL_BLE_MDMHP_SYNC_NB_ERR_MAX_BLE_RST      0x0
#define RXSYNC_CNTL_BLE_MDMHP_SYNC_LLHOOD_THRES_BLE_RST    0x11A

__INLINE void mdmhp_rxsync_cntl_ble_pack(uint8_t syncminen, uint16_t synctsithresble, uint8_t syncnberrmaxble, uint16_t syncllhoodthresble)
{
    ASSERT_ERR((((uint32_t)syncminen << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)synctsithresble << 16) & ~((uint32_t)0x1FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)syncnberrmaxble << 12) & ~((uint32_t)0x0000F000)) == 0);
    ASSERT_ERR((((uint32_t)syncllhoodthresble << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_IP_WR(RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_ADDR,  ((uint32_t)syncminen << 31) | ((uint32_t)synctsithresble << 16) | ((uint32_t)syncnberrmaxble << 12) | ((uint32_t)syncllhoodthresble << 0));
}

__INLINE void mdmhp_rxsync_cntl_ble_unpack(uint8_t* syncminen, uint16_t* synctsithresble, uint8_t* syncnberrmaxble, uint16_t* syncllhoodthresble)
{
    uint32_t localVal = REG_IP_RD(RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_ADDR);

    *syncminen = (localVal & ((uint32_t)0x80000000)) >> 31;
    *synctsithresble = (localVal & ((uint32_t)0x1FFF0000)) >> 16;
    *syncnberrmaxble = (localVal & ((uint32_t)0x0000F000)) >> 12;
    *syncllhoodthresble = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint8_t mdmhp_rxsync_cntl_ble_sync_min_en_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void mdmhp_rxsync_cntl_ble_sync_min_en_setf(uint8_t syncminen)
{
    ASSERT_ERR((((uint32_t)syncminen << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_IP_WR(RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_ADDR, (REG_IP_RD(RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)syncminen << 31));
}

__INLINE uint16_t mdmhp_rxsync_cntl_ble_sync_tsi_thres_ble_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_ADDR);
    return ((localVal & ((uint32_t)0x1FFF0000)) >> 16);
}

__INLINE void mdmhp_rxsync_cntl_ble_sync_tsi_thres_ble_setf(uint16_t synctsithresble)
{
    ASSERT_ERR((((uint32_t)synctsithresble << 16) & ~((uint32_t)0x1FFF0000)) == 0);
    REG_IP_WR(RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_ADDR, (REG_IP_RD(RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_ADDR) & ~((uint32_t)0x1FFF0000)) | ((uint32_t)synctsithresble << 16));
}

__INLINE uint8_t mdmhp_rxsync_cntl_ble_sync_nb_err_max_ble_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_ADDR);
    return ((localVal & ((uint32_t)0x0000F000)) >> 12);
}

__INLINE void mdmhp_rxsync_cntl_ble_sync_nb_err_max_ble_setf(uint8_t syncnberrmaxble)
{
    ASSERT_ERR((((uint32_t)syncnberrmaxble << 12) & ~((uint32_t)0x0000F000)) == 0);
    REG_IP_WR(RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_ADDR, (REG_IP_RD(RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_ADDR) & ~((uint32_t)0x0000F000)) | ((uint32_t)syncnberrmaxble << 12));
}

__INLINE uint16_t mdmhp_rxsync_cntl_ble_sync_llhood_thres_ble_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void mdmhp_rxsync_cntl_ble_sync_llhood_thres_ble_setf(uint16_t syncllhoodthresble)
{
    ASSERT_ERR((((uint32_t)syncllhoodthresble << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_IP_WR(RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_ADDR, (REG_IP_RD(RXSYNC_CNTL_BLE_MDMHP_RXSYNC_CNTL_BLE_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)syncllhoodthresble << 0));
}

/**
 * @brief RXSYNC_CNTL_BT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  28:16    SYNC_TSI_THRES_BT   0x5A1
 *  15:12   SYNC_NB_ERR_MAX_BT   0x3
 *  09:00   SYNC_LLHOOD_THRES_BT   0xC8
 * </pre>
 */
#define RXSYNC_CNTL_BT_MDMHP_RXSYNC_CNTL_BT_ADDR   0x210200B0
#define RXSYNC_CNTL_BT_MDMHP_RXSYNC_CNTL_BT_OFFSET 0x000000B0
#define RXSYNC_CNTL_BT_MDMHP_RXSYNC_CNTL_BT_INDEX  0x0000002C
#define RXSYNC_CNTL_BT_MDMHP_RXSYNC_CNTL_BT_RESET  0x05A130C8

__INLINE uint32_t mdmhp_rxsync_cntl_bt_get(void)
{
    return REG_IP_RD(RXSYNC_CNTL_BT_MDMHP_RXSYNC_CNTL_BT_ADDR);
}

__INLINE void mdmhp_rxsync_cntl_bt_set(uint32_t value)
{
    REG_IP_WR(RXSYNC_CNTL_BT_MDMHP_RXSYNC_CNTL_BT_ADDR, value);
}

// field definitions
#define RXSYNC_CNTL_BT_MDMHP_SYNC_TSI_THRES_BT_MASK      ((uint32_t)0x1FFF0000)
#define RXSYNC_CNTL_BT_MDMHP_SYNC_TSI_THRES_BT_LSB       16
#define RXSYNC_CNTL_BT_MDMHP_SYNC_TSI_THRES_BT_WIDTH     ((uint32_t)0x0000000D)
#define RXSYNC_CNTL_BT_MDMHP_SYNC_NB_ERR_MAX_BT_MASK     ((uint32_t)0x0000F000)
#define RXSYNC_CNTL_BT_MDMHP_SYNC_NB_ERR_MAX_BT_LSB      12
#define RXSYNC_CNTL_BT_MDMHP_SYNC_NB_ERR_MAX_BT_WIDTH    ((uint32_t)0x00000004)
#define RXSYNC_CNTL_BT_MDMHP_SYNC_LLHOOD_THRES_BT_MASK   ((uint32_t)0x000003FF)
#define RXSYNC_CNTL_BT_MDMHP_SYNC_LLHOOD_THRES_BT_LSB    0
#define RXSYNC_CNTL_BT_MDMHP_SYNC_LLHOOD_THRES_BT_WIDTH  ((uint32_t)0x0000000A)

#define RXSYNC_CNTL_BT_MDMHP_SYNC_TSI_THRES_BT_RST       0x5A1
#define RXSYNC_CNTL_BT_MDMHP_SYNC_NB_ERR_MAX_BT_RST      0x3
#define RXSYNC_CNTL_BT_MDMHP_SYNC_LLHOOD_THRES_BT_RST    0xC8

__INLINE void mdmhp_rxsync_cntl_bt_pack(uint16_t synctsithresbt, uint8_t syncnberrmaxbt, uint16_t syncllhoodthresbt)
{
    ASSERT_ERR((((uint32_t)synctsithresbt << 16) & ~((uint32_t)0x1FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)syncnberrmaxbt << 12) & ~((uint32_t)0x0000F000)) == 0);
    ASSERT_ERR((((uint32_t)syncllhoodthresbt << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_IP_WR(RXSYNC_CNTL_BT_MDMHP_RXSYNC_CNTL_BT_ADDR,  ((uint32_t)synctsithresbt << 16) | ((uint32_t)syncnberrmaxbt << 12) | ((uint32_t)syncllhoodthresbt << 0));
}

__INLINE void mdmhp_rxsync_cntl_bt_unpack(uint16_t* synctsithresbt, uint8_t* syncnberrmaxbt, uint16_t* syncllhoodthresbt)
{
    uint32_t localVal = REG_IP_RD(RXSYNC_CNTL_BT_MDMHP_RXSYNC_CNTL_BT_ADDR);

    *synctsithresbt = (localVal & ((uint32_t)0x1FFF0000)) >> 16;
    *syncnberrmaxbt = (localVal & ((uint32_t)0x0000F000)) >> 12;
    *syncllhoodthresbt = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint16_t mdmhp_rxsync_cntl_bt_sync_tsi_thres_bt_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSYNC_CNTL_BT_MDMHP_RXSYNC_CNTL_BT_ADDR);
    return ((localVal & ((uint32_t)0x1FFF0000)) >> 16);
}

__INLINE void mdmhp_rxsync_cntl_bt_sync_tsi_thres_bt_setf(uint16_t synctsithresbt)
{
    ASSERT_ERR((((uint32_t)synctsithresbt << 16) & ~((uint32_t)0x1FFF0000)) == 0);
    REG_IP_WR(RXSYNC_CNTL_BT_MDMHP_RXSYNC_CNTL_BT_ADDR, (REG_IP_RD(RXSYNC_CNTL_BT_MDMHP_RXSYNC_CNTL_BT_ADDR) & ~((uint32_t)0x1FFF0000)) | ((uint32_t)synctsithresbt << 16));
}

__INLINE uint8_t mdmhp_rxsync_cntl_bt_sync_nb_err_max_bt_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSYNC_CNTL_BT_MDMHP_RXSYNC_CNTL_BT_ADDR);
    return ((localVal & ((uint32_t)0x0000F000)) >> 12);
}

__INLINE void mdmhp_rxsync_cntl_bt_sync_nb_err_max_bt_setf(uint8_t syncnberrmaxbt)
{
    ASSERT_ERR((((uint32_t)syncnberrmaxbt << 12) & ~((uint32_t)0x0000F000)) == 0);
    REG_IP_WR(RXSYNC_CNTL_BT_MDMHP_RXSYNC_CNTL_BT_ADDR, (REG_IP_RD(RXSYNC_CNTL_BT_MDMHP_RXSYNC_CNTL_BT_ADDR) & ~((uint32_t)0x0000F000)) | ((uint32_t)syncnberrmaxbt << 12));
}

__INLINE uint16_t mdmhp_rxsync_cntl_bt_sync_llhood_thres_bt_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSYNC_CNTL_BT_MDMHP_RXSYNC_CNTL_BT_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void mdmhp_rxsync_cntl_bt_sync_llhood_thres_bt_setf(uint16_t syncllhoodthresbt)
{
    ASSERT_ERR((((uint32_t)syncllhoodthresbt << 0) & ~((uint32_t)0x000003FF)) == 0);
    REG_IP_WR(RXSYNC_CNTL_BT_MDMHP_RXSYNC_CNTL_BT_ADDR, (REG_IP_RD(RXSYNC_CNTL_BT_MDMHP_RXSYNC_CNTL_BT_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)syncllhoodthresbt << 0));
}

/**
 * @brief RXSYNC_CNTL_LR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  28:16    SYNC_TSI_THRES_LR   0x99A
 *  07:00   SYNC_WIN_MARGIN_US_LR   0x8
 * </pre>
 */
#define RXSYNC_CNTL_LR_MDMHP_RXSYNC_CNTL_LR_ADDR   0x210200B4
#define RXSYNC_CNTL_LR_MDMHP_RXSYNC_CNTL_LR_OFFSET 0x000000B4
#define RXSYNC_CNTL_LR_MDMHP_RXSYNC_CNTL_LR_INDEX  0x0000002D
#define RXSYNC_CNTL_LR_MDMHP_RXSYNC_CNTL_LR_RESET  0x099A0008

__INLINE uint32_t mdmhp_rxsync_cntl_lr_get(void)
{
    return REG_IP_RD(RXSYNC_CNTL_LR_MDMHP_RXSYNC_CNTL_LR_ADDR);
}

__INLINE void mdmhp_rxsync_cntl_lr_set(uint32_t value)
{
    REG_IP_WR(RXSYNC_CNTL_LR_MDMHP_RXSYNC_CNTL_LR_ADDR, value);
}

// field definitions
#define RXSYNC_CNTL_LR_MDMHP_SYNC_TSI_THRES_LR_MASK       ((uint32_t)0x1FFF0000)
#define RXSYNC_CNTL_LR_MDMHP_SYNC_TSI_THRES_LR_LSB        16
#define RXSYNC_CNTL_LR_MDMHP_SYNC_TSI_THRES_LR_WIDTH      ((uint32_t)0x0000000D)
#define RXSYNC_CNTL_LR_MDMHP_SYNC_WIN_MARGIN_US_LR_MASK   ((uint32_t)0x000000FF)
#define RXSYNC_CNTL_LR_MDMHP_SYNC_WIN_MARGIN_US_LR_LSB    0
#define RXSYNC_CNTL_LR_MDMHP_SYNC_WIN_MARGIN_US_LR_WIDTH  ((uint32_t)0x00000008)

#define RXSYNC_CNTL_LR_MDMHP_SYNC_TSI_THRES_LR_RST        0x99A
#define RXSYNC_CNTL_LR_MDMHP_SYNC_WIN_MARGIN_US_LR_RST    0x8

__INLINE void mdmhp_rxsync_cntl_lr_pack(uint16_t synctsithreslr, uint8_t syncwinmarginuslr)
{
    ASSERT_ERR((((uint32_t)synctsithreslr << 16) & ~((uint32_t)0x1FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)syncwinmarginuslr << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(RXSYNC_CNTL_LR_MDMHP_RXSYNC_CNTL_LR_ADDR,  ((uint32_t)synctsithreslr << 16) | ((uint32_t)syncwinmarginuslr << 0));
}

__INLINE void mdmhp_rxsync_cntl_lr_unpack(uint16_t* synctsithreslr, uint8_t* syncwinmarginuslr)
{
    uint32_t localVal = REG_IP_RD(RXSYNC_CNTL_LR_MDMHP_RXSYNC_CNTL_LR_ADDR);

    *synctsithreslr = (localVal & ((uint32_t)0x1FFF0000)) >> 16;
    *syncwinmarginuslr = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint16_t mdmhp_rxsync_cntl_lr_sync_tsi_thres_lr_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSYNC_CNTL_LR_MDMHP_RXSYNC_CNTL_LR_ADDR);
    return ((localVal & ((uint32_t)0x1FFF0000)) >> 16);
}

__INLINE void mdmhp_rxsync_cntl_lr_sync_tsi_thres_lr_setf(uint16_t synctsithreslr)
{
    ASSERT_ERR((((uint32_t)synctsithreslr << 16) & ~((uint32_t)0x1FFF0000)) == 0);
    REG_IP_WR(RXSYNC_CNTL_LR_MDMHP_RXSYNC_CNTL_LR_ADDR, (REG_IP_RD(RXSYNC_CNTL_LR_MDMHP_RXSYNC_CNTL_LR_ADDR) & ~((uint32_t)0x1FFF0000)) | ((uint32_t)synctsithreslr << 16));
}

__INLINE uint8_t mdmhp_rxsync_cntl_lr_sync_win_margin_us_lr_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSYNC_CNTL_LR_MDMHP_RXSYNC_CNTL_LR_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void mdmhp_rxsync_cntl_lr_sync_win_margin_us_lr_setf(uint8_t syncwinmarginuslr)
{
    ASSERT_ERR((((uint32_t)syncwinmarginuslr << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(RXSYNC_CNTL_LR_MDMHP_RXSYNC_CNTL_LR_ADDR, (REG_IP_RD(RXSYNC_CNTL_LR_MDMHP_RXSYNC_CNTL_LR_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)syncwinmarginuslr << 0));
}

/**
 * @brief RXHPDEMOD_CFO_CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     16         HP_CFO_FORCE   0
 *  06:00           HP_CFO_VAL   0x0
 * </pre>
 */
#define RXHPDEMOD_CFO_CNTL_MDMHP_RXHPDEMOD_CFO_CNTL_ADDR   0x210200B8
#define RXHPDEMOD_CFO_CNTL_MDMHP_RXHPDEMOD_CFO_CNTL_OFFSET 0x000000B8
#define RXHPDEMOD_CFO_CNTL_MDMHP_RXHPDEMOD_CFO_CNTL_INDEX  0x0000002E
#define RXHPDEMOD_CFO_CNTL_MDMHP_RXHPDEMOD_CFO_CNTL_RESET  0x00000000

__INLINE uint32_t mdmhp_rxhpdemod_cfo_cntl_get(void)
{
    return REG_IP_RD(RXHPDEMOD_CFO_CNTL_MDMHP_RXHPDEMOD_CFO_CNTL_ADDR);
}

__INLINE void mdmhp_rxhpdemod_cfo_cntl_set(uint32_t value)
{
    REG_IP_WR(RXHPDEMOD_CFO_CNTL_MDMHP_RXHPDEMOD_CFO_CNTL_ADDR, value);
}

// field definitions
#define RXHPDEMOD_CFO_CNTL_MDMHP_HP_CFO_FORCE_BIT    ((uint32_t)0x00010000)
#define RXHPDEMOD_CFO_CNTL_MDMHP_HP_CFO_FORCE_POS    16
#define RXHPDEMOD_CFO_CNTL_MDMHP_HP_CFO_VAL_MASK     ((uint32_t)0x0000007F)
#define RXHPDEMOD_CFO_CNTL_MDMHP_HP_CFO_VAL_LSB      0
#define RXHPDEMOD_CFO_CNTL_MDMHP_HP_CFO_VAL_WIDTH    ((uint32_t)0x00000007)

#define RXHPDEMOD_CFO_CNTL_MDMHP_HP_CFO_FORCE_RST    0x0
#define RXHPDEMOD_CFO_CNTL_MDMHP_HP_CFO_VAL_RST      0x0

__INLINE void mdmhp_rxhpdemod_cfo_cntl_pack(uint8_t hpcfoforce, uint8_t hpcfoval)
{
    ASSERT_ERR((((uint32_t)hpcfoforce << 16) & ~((uint32_t)0x00010000)) == 0);
    ASSERT_ERR((((uint32_t)hpcfoval << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_IP_WR(RXHPDEMOD_CFO_CNTL_MDMHP_RXHPDEMOD_CFO_CNTL_ADDR,  ((uint32_t)hpcfoforce << 16) | ((uint32_t)hpcfoval << 0));
}

__INLINE void mdmhp_rxhpdemod_cfo_cntl_unpack(uint8_t* hpcfoforce, uint8_t* hpcfoval)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_CFO_CNTL_MDMHP_RXHPDEMOD_CFO_CNTL_ADDR);

    *hpcfoforce = (localVal & ((uint32_t)0x00010000)) >> 16;
    *hpcfoval = (localVal & ((uint32_t)0x0000007F)) >> 0;
}

__INLINE uint8_t mdmhp_rxhpdemod_cfo_cntl_hp_cfo_force_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_CFO_CNTL_MDMHP_RXHPDEMOD_CFO_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void mdmhp_rxhpdemod_cfo_cntl_hp_cfo_force_setf(uint8_t hpcfoforce)
{
    ASSERT_ERR((((uint32_t)hpcfoforce << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_IP_WR(RXHPDEMOD_CFO_CNTL_MDMHP_RXHPDEMOD_CFO_CNTL_ADDR, (REG_IP_RD(RXHPDEMOD_CFO_CNTL_MDMHP_RXHPDEMOD_CFO_CNTL_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)hpcfoforce << 16));
}

__INLINE uint8_t mdmhp_rxhpdemod_cfo_cntl_hp_cfo_val_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_CFO_CNTL_MDMHP_RXHPDEMOD_CFO_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000007F)) >> 0);
}

__INLINE void mdmhp_rxhpdemod_cfo_cntl_hp_cfo_val_setf(uint8_t hpcfoval)
{
    ASSERT_ERR((((uint32_t)hpcfoval << 0) & ~((uint32_t)0x0000007F)) == 0);
    REG_IP_WR(RXHPDEMOD_CFO_CNTL_MDMHP_RXHPDEMOD_CFO_CNTL_ADDR, (REG_IP_RD(RXHPDEMOD_CFO_CNTL_MDMHP_RXHPDEMOD_CFO_CNTL_ADDR) & ~((uint32_t)0x0000007F)) | ((uint32_t)hpcfoval << 0));
}

/**
 * @brief RXSTOEST_CNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31        STOEST_EN_154   1
 *     30         STOEST_EN_LR   1
 *     29         STOEST_EN_BT   0
 *     28        STOEST_EN_BLE   0
 *  27:16         STO_STOP_DEL   0xF9F
 *     11   STO_SCALING_CAP_154   0
 *     10   STO_SCALING_CAP_LR   0
 *     09   STO_SCALING_CAP_BT   0
 *     08   STO_SCALING_CAP_BLE   0
 *  07:04          STO_SCALING   0xC
 *     02    STO_SCALING_FORCE   0
 *     01       STO_PREPROC_EN   1
 * </pre>
 */
#define RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR   0x210200CC
#define RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_OFFSET 0x000000CC
#define RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_INDEX  0x00000033
#define RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_RESET  0xCF9F00C2

__INLINE uint32_t mdmhp_rxstoest_cntl0_get(void)
{
    return REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR);
}

__INLINE void mdmhp_rxstoest_cntl0_set(uint32_t value)
{
    REG_IP_WR(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR, value);
}

// field definitions
#define RXSTOEST_CNTL0_MDMHP_STOEST_EN_154_BIT          ((uint32_t)0x80000000)
#define RXSTOEST_CNTL0_MDMHP_STOEST_EN_154_POS          31
#define RXSTOEST_CNTL0_MDMHP_STOEST_EN_LR_BIT           ((uint32_t)0x40000000)
#define RXSTOEST_CNTL0_MDMHP_STOEST_EN_LR_POS           30
#define RXSTOEST_CNTL0_MDMHP_STOEST_EN_BT_BIT           ((uint32_t)0x20000000)
#define RXSTOEST_CNTL0_MDMHP_STOEST_EN_BT_POS           29
#define RXSTOEST_CNTL0_MDMHP_STOEST_EN_BLE_BIT          ((uint32_t)0x10000000)
#define RXSTOEST_CNTL0_MDMHP_STOEST_EN_BLE_POS          28
#define RXSTOEST_CNTL0_MDMHP_STO_STOP_DEL_MASK          ((uint32_t)0x0FFF0000)
#define RXSTOEST_CNTL0_MDMHP_STO_STOP_DEL_LSB           16
#define RXSTOEST_CNTL0_MDMHP_STO_STOP_DEL_WIDTH         ((uint32_t)0x0000000C)
#define RXSTOEST_CNTL0_MDMHP_STO_SCALING_CAP_154_BIT    ((uint32_t)0x00000800)
#define RXSTOEST_CNTL0_MDMHP_STO_SCALING_CAP_154_POS    11
#define RXSTOEST_CNTL0_MDMHP_STO_SCALING_CAP_LR_BIT     ((uint32_t)0x00000400)
#define RXSTOEST_CNTL0_MDMHP_STO_SCALING_CAP_LR_POS     10
#define RXSTOEST_CNTL0_MDMHP_STO_SCALING_CAP_BT_BIT     ((uint32_t)0x00000200)
#define RXSTOEST_CNTL0_MDMHP_STO_SCALING_CAP_BT_POS     9
#define RXSTOEST_CNTL0_MDMHP_STO_SCALING_CAP_BLE_BIT    ((uint32_t)0x00000100)
#define RXSTOEST_CNTL0_MDMHP_STO_SCALING_CAP_BLE_POS    8
#define RXSTOEST_CNTL0_MDMHP_STO_SCALING_MASK           ((uint32_t)0x000000F0)
#define RXSTOEST_CNTL0_MDMHP_STO_SCALING_LSB            4
#define RXSTOEST_CNTL0_MDMHP_STO_SCALING_WIDTH          ((uint32_t)0x00000004)
#define RXSTOEST_CNTL0_MDMHP_STO_SCALING_FORCE_BIT      ((uint32_t)0x00000004)
#define RXSTOEST_CNTL0_MDMHP_STO_SCALING_FORCE_POS      2
#define RXSTOEST_CNTL0_MDMHP_STO_PREPROC_EN_BIT         ((uint32_t)0x00000002)
#define RXSTOEST_CNTL0_MDMHP_STO_PREPROC_EN_POS         1

#define RXSTOEST_CNTL0_MDMHP_STOEST_EN_154_RST          0x1
#define RXSTOEST_CNTL0_MDMHP_STOEST_EN_LR_RST           0x1
#define RXSTOEST_CNTL0_MDMHP_STOEST_EN_BT_RST           0x0
#define RXSTOEST_CNTL0_MDMHP_STOEST_EN_BLE_RST          0x0
#define RXSTOEST_CNTL0_MDMHP_STO_STOP_DEL_RST           0xF9F
#define RXSTOEST_CNTL0_MDMHP_STO_SCALING_CAP_154_RST    0x0
#define RXSTOEST_CNTL0_MDMHP_STO_SCALING_CAP_LR_RST     0x0
#define RXSTOEST_CNTL0_MDMHP_STO_SCALING_CAP_BT_RST     0x0
#define RXSTOEST_CNTL0_MDMHP_STO_SCALING_CAP_BLE_RST    0x0
#define RXSTOEST_CNTL0_MDMHP_STO_SCALING_RST            0xC
#define RXSTOEST_CNTL0_MDMHP_STO_SCALING_FORCE_RST      0x0
#define RXSTOEST_CNTL0_MDMHP_STO_PREPROC_EN_RST         0x1

__INLINE void mdmhp_rxstoest_cntl0_pack(uint8_t stoesten154, uint8_t stoestenlr, uint8_t stoestenbt, uint8_t stoestenble, uint16_t stostopdel, uint8_t stoscalingcap154, uint8_t stoscalingcaplr, uint8_t stoscalingcapbt, uint8_t stoscalingcapble, uint8_t stoscaling, uint8_t stoscalingforce, uint8_t stopreprocen)
{
    ASSERT_ERR((((uint32_t)stoesten154 << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)stoestenlr << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)stoestenbt << 29) & ~((uint32_t)0x20000000)) == 0);
    ASSERT_ERR((((uint32_t)stoestenble << 28) & ~((uint32_t)0x10000000)) == 0);
    ASSERT_ERR((((uint32_t)stostopdel << 16) & ~((uint32_t)0x0FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)stoscalingcap154 << 11) & ~((uint32_t)0x00000800)) == 0);
    ASSERT_ERR((((uint32_t)stoscalingcaplr << 10) & ~((uint32_t)0x00000400)) == 0);
    ASSERT_ERR((((uint32_t)stoscalingcapbt << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)stoscalingcapble << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)stoscaling << 4) & ~((uint32_t)0x000000F0)) == 0);
    ASSERT_ERR((((uint32_t)stoscalingforce << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)stopreprocen << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_IP_WR(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR,  ((uint32_t)stoesten154 << 31) | ((uint32_t)stoestenlr << 30) | ((uint32_t)stoestenbt << 29) | ((uint32_t)stoestenble << 28) | ((uint32_t)stostopdel << 16) | ((uint32_t)stoscalingcap154 << 11) | ((uint32_t)stoscalingcaplr << 10) | ((uint32_t)stoscalingcapbt << 9) | ((uint32_t)stoscalingcapble << 8) | ((uint32_t)stoscaling << 4) | ((uint32_t)stoscalingforce << 2) | ((uint32_t)stopreprocen << 1));
}

__INLINE void mdmhp_rxstoest_cntl0_unpack(uint8_t* stoesten154, uint8_t* stoestenlr, uint8_t* stoestenbt, uint8_t* stoestenble, uint16_t* stostopdel, uint8_t* stoscalingcap154, uint8_t* stoscalingcaplr, uint8_t* stoscalingcapbt, uint8_t* stoscalingcapble, uint8_t* stoscaling, uint8_t* stoscalingforce, uint8_t* stopreprocen)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR);

    *stoesten154 = (localVal & ((uint32_t)0x80000000)) >> 31;
    *stoestenlr = (localVal & ((uint32_t)0x40000000)) >> 30;
    *stoestenbt = (localVal & ((uint32_t)0x20000000)) >> 29;
    *stoestenble = (localVal & ((uint32_t)0x10000000)) >> 28;
    *stostopdel = (localVal & ((uint32_t)0x0FFF0000)) >> 16;
    *stoscalingcap154 = (localVal & ((uint32_t)0x00000800)) >> 11;
    *stoscalingcaplr = (localVal & ((uint32_t)0x00000400)) >> 10;
    *stoscalingcapbt = (localVal & ((uint32_t)0x00000200)) >> 9;
    *stoscalingcapble = (localVal & ((uint32_t)0x00000100)) >> 8;
    *stoscaling = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *stoscalingforce = (localVal & ((uint32_t)0x00000004)) >> 2;
    *stopreprocen = (localVal & ((uint32_t)0x00000002)) >> 1;
}

__INLINE uint8_t mdmhp_rxstoest_cntl0_stoest_en_154_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void mdmhp_rxstoest_cntl0_stoest_en_154_setf(uint8_t stoesten154)
{
    ASSERT_ERR((((uint32_t)stoesten154 << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_IP_WR(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR, (REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)stoesten154 << 31));
}

__INLINE uint8_t mdmhp_rxstoest_cntl0_stoest_en_lr_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void mdmhp_rxstoest_cntl0_stoest_en_lr_setf(uint8_t stoestenlr)
{
    ASSERT_ERR((((uint32_t)stoestenlr << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_IP_WR(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR, (REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)stoestenlr << 30));
}

__INLINE uint8_t mdmhp_rxstoest_cntl0_stoest_en_bt_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x20000000)) >> 29);
}

__INLINE void mdmhp_rxstoest_cntl0_stoest_en_bt_setf(uint8_t stoestenbt)
{
    ASSERT_ERR((((uint32_t)stoestenbt << 29) & ~((uint32_t)0x20000000)) == 0);
    REG_IP_WR(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR, (REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR) & ~((uint32_t)0x20000000)) | ((uint32_t)stoestenbt << 29));
}

__INLINE uint8_t mdmhp_rxstoest_cntl0_stoest_en_ble_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void mdmhp_rxstoest_cntl0_stoest_en_ble_setf(uint8_t stoestenble)
{
    ASSERT_ERR((((uint32_t)stoestenble << 28) & ~((uint32_t)0x10000000)) == 0);
    REG_IP_WR(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR, (REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)stoestenble << 28));
}

__INLINE uint16_t mdmhp_rxstoest_cntl0_sto_stop_del_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x0FFF0000)) >> 16);
}

__INLINE void mdmhp_rxstoest_cntl0_sto_stop_del_setf(uint16_t stostopdel)
{
    ASSERT_ERR((((uint32_t)stostopdel << 16) & ~((uint32_t)0x0FFF0000)) == 0);
    REG_IP_WR(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR, (REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR) & ~((uint32_t)0x0FFF0000)) | ((uint32_t)stostopdel << 16));
}

__INLINE uint8_t mdmhp_rxstoest_cntl0_sto_scaling_cap_154_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void mdmhp_rxstoest_cntl0_sto_scaling_cap_154_setf(uint8_t stoscalingcap154)
{
    ASSERT_ERR((((uint32_t)stoscalingcap154 << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_IP_WR(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR, (REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)stoscalingcap154 << 11));
}

__INLINE uint8_t mdmhp_rxstoest_cntl0_sto_scaling_cap_lr_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void mdmhp_rxstoest_cntl0_sto_scaling_cap_lr_setf(uint8_t stoscalingcaplr)
{
    ASSERT_ERR((((uint32_t)stoscalingcaplr << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_IP_WR(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR, (REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)stoscalingcaplr << 10));
}

__INLINE uint8_t mdmhp_rxstoest_cntl0_sto_scaling_cap_bt_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void mdmhp_rxstoest_cntl0_sto_scaling_cap_bt_setf(uint8_t stoscalingcapbt)
{
    ASSERT_ERR((((uint32_t)stoscalingcapbt << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_IP_WR(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR, (REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)stoscalingcapbt << 9));
}

__INLINE uint8_t mdmhp_rxstoest_cntl0_sto_scaling_cap_ble_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void mdmhp_rxstoest_cntl0_sto_scaling_cap_ble_setf(uint8_t stoscalingcapble)
{
    ASSERT_ERR((((uint32_t)stoscalingcapble << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_IP_WR(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR, (REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)stoscalingcapble << 8));
}

__INLINE uint8_t mdmhp_rxstoest_cntl0_sto_scaling_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void mdmhp_rxstoest_cntl0_sto_scaling_setf(uint8_t stoscaling)
{
    ASSERT_ERR((((uint32_t)stoscaling << 4) & ~((uint32_t)0x000000F0)) == 0);
    REG_IP_WR(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR, (REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)stoscaling << 4));
}

__INLINE uint8_t mdmhp_rxstoest_cntl0_sto_scaling_force_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void mdmhp_rxstoest_cntl0_sto_scaling_force_setf(uint8_t stoscalingforce)
{
    ASSERT_ERR((((uint32_t)stoscalingforce << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_IP_WR(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR, (REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)stoscalingforce << 2));
}

__INLINE uint8_t mdmhp_rxstoest_cntl0_sto_preproc_en_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void mdmhp_rxstoest_cntl0_sto_preproc_en_setf(uint8_t stopreprocen)
{
    ASSERT_ERR((((uint32_t)stopreprocen << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_IP_WR(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR, (REG_IP_RD(RXSTOEST_CNTL0_MDMHP_RXSTOEST_CNTL0_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)stopreprocen << 1));
}

/**
 * @brief RXSTOEST_CNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:16   STO_START_DEL_2MHZ   0x1F3
 *  11:00   STO_START_DEL_1MHZ   0x3E7
 * </pre>
 */
#define RXSTOEST_CNTL1_MDMHP_RXSTOEST_CNTL1_ADDR   0x210200D0
#define RXSTOEST_CNTL1_MDMHP_RXSTOEST_CNTL1_OFFSET 0x000000D0
#define RXSTOEST_CNTL1_MDMHP_RXSTOEST_CNTL1_INDEX  0x00000034
#define RXSTOEST_CNTL1_MDMHP_RXSTOEST_CNTL1_RESET  0x01F303E7

__INLINE uint32_t mdmhp_rxstoest_cntl1_get(void)
{
    return REG_IP_RD(RXSTOEST_CNTL1_MDMHP_RXSTOEST_CNTL1_ADDR);
}

__INLINE void mdmhp_rxstoest_cntl1_set(uint32_t value)
{
    REG_IP_WR(RXSTOEST_CNTL1_MDMHP_RXSTOEST_CNTL1_ADDR, value);
}

// field definitions
#define RXSTOEST_CNTL1_MDMHP_STO_START_DEL_2MHZ_MASK   ((uint32_t)0x0FFF0000)
#define RXSTOEST_CNTL1_MDMHP_STO_START_DEL_2MHZ_LSB    16
#define RXSTOEST_CNTL1_MDMHP_STO_START_DEL_2MHZ_WIDTH  ((uint32_t)0x0000000C)
#define RXSTOEST_CNTL1_MDMHP_STO_START_DEL_1MHZ_MASK   ((uint32_t)0x00000FFF)
#define RXSTOEST_CNTL1_MDMHP_STO_START_DEL_1MHZ_LSB    0
#define RXSTOEST_CNTL1_MDMHP_STO_START_DEL_1MHZ_WIDTH  ((uint32_t)0x0000000C)

#define RXSTOEST_CNTL1_MDMHP_STO_START_DEL_2MHZ_RST    0x1F3
#define RXSTOEST_CNTL1_MDMHP_STO_START_DEL_1MHZ_RST    0x3E7

__INLINE void mdmhp_rxstoest_cntl1_pack(uint16_t stostartdel2mhz, uint16_t stostartdel1mhz)
{
    ASSERT_ERR((((uint32_t)stostartdel2mhz << 16) & ~((uint32_t)0x0FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)stostartdel1mhz << 0) & ~((uint32_t)0x00000FFF)) == 0);
    REG_IP_WR(RXSTOEST_CNTL1_MDMHP_RXSTOEST_CNTL1_ADDR,  ((uint32_t)stostartdel2mhz << 16) | ((uint32_t)stostartdel1mhz << 0));
}

__INLINE void mdmhp_rxstoest_cntl1_unpack(uint16_t* stostartdel2mhz, uint16_t* stostartdel1mhz)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_CNTL1_MDMHP_RXSTOEST_CNTL1_ADDR);

    *stostartdel2mhz = (localVal & ((uint32_t)0x0FFF0000)) >> 16;
    *stostartdel1mhz = (localVal & ((uint32_t)0x00000FFF)) >> 0;
}

__INLINE uint16_t mdmhp_rxstoest_cntl1_sto_start_del_2mhz_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_CNTL1_MDMHP_RXSTOEST_CNTL1_ADDR);
    return ((localVal & ((uint32_t)0x0FFF0000)) >> 16);
}

__INLINE void mdmhp_rxstoest_cntl1_sto_start_del_2mhz_setf(uint16_t stostartdel2mhz)
{
    ASSERT_ERR((((uint32_t)stostartdel2mhz << 16) & ~((uint32_t)0x0FFF0000)) == 0);
    REG_IP_WR(RXSTOEST_CNTL1_MDMHP_RXSTOEST_CNTL1_ADDR, (REG_IP_RD(RXSTOEST_CNTL1_MDMHP_RXSTOEST_CNTL1_ADDR) & ~((uint32_t)0x0FFF0000)) | ((uint32_t)stostartdel2mhz << 16));
}

__INLINE uint16_t mdmhp_rxstoest_cntl1_sto_start_del_1mhz_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_CNTL1_MDMHP_RXSTOEST_CNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000FFF)) >> 0);
}

__INLINE void mdmhp_rxstoest_cntl1_sto_start_del_1mhz_setf(uint16_t stostartdel1mhz)
{
    ASSERT_ERR((((uint32_t)stostartdel1mhz << 0) & ~((uint32_t)0x00000FFF)) == 0);
    REG_IP_WR(RXSTOEST_CNTL1_MDMHP_RXSTOEST_CNTL1_ADDR, (REG_IP_RD(RXSTOEST_CNTL1_MDMHP_RXSTOEST_CNTL1_ADDR) & ~((uint32_t)0x00000FFF)) | ((uint32_t)stostartdel1mhz << 0));
}

/**
 * @brief RXSTOEST_KALMAN_CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:28        STO_KALMAN_QR   0x3
 *  27:16   STO_KALMAN_DEL_2MHZ   0xF9
 *  11:00   STO_KALMAN_DEL_1MHZ   0x1F3
 * </pre>
 */
#define RXSTOEST_KALMAN_CNTL_MDMHP_RXSTOEST_KALMAN_CNTL_ADDR   0x210200D4
#define RXSTOEST_KALMAN_CNTL_MDMHP_RXSTOEST_KALMAN_CNTL_OFFSET 0x000000D4
#define RXSTOEST_KALMAN_CNTL_MDMHP_RXSTOEST_KALMAN_CNTL_INDEX  0x00000035
#define RXSTOEST_KALMAN_CNTL_MDMHP_RXSTOEST_KALMAN_CNTL_RESET  0x30F901F3

__INLINE uint32_t mdmhp_rxstoest_kalman_cntl_get(void)
{
    return REG_IP_RD(RXSTOEST_KALMAN_CNTL_MDMHP_RXSTOEST_KALMAN_CNTL_ADDR);
}

__INLINE void mdmhp_rxstoest_kalman_cntl_set(uint32_t value)
{
    REG_IP_WR(RXSTOEST_KALMAN_CNTL_MDMHP_RXSTOEST_KALMAN_CNTL_ADDR, value);
}

// field definitions
#define RXSTOEST_KALMAN_CNTL_MDMHP_STO_KALMAN_QR_MASK         ((uint32_t)0x70000000)
#define RXSTOEST_KALMAN_CNTL_MDMHP_STO_KALMAN_QR_LSB          28
#define RXSTOEST_KALMAN_CNTL_MDMHP_STO_KALMAN_QR_WIDTH        ((uint32_t)0x00000003)
#define RXSTOEST_KALMAN_CNTL_MDMHP_STO_KALMAN_DEL_2MHZ_MASK   ((uint32_t)0x0FFF0000)
#define RXSTOEST_KALMAN_CNTL_MDMHP_STO_KALMAN_DEL_2MHZ_LSB    16
#define RXSTOEST_KALMAN_CNTL_MDMHP_STO_KALMAN_DEL_2MHZ_WIDTH  ((uint32_t)0x0000000C)
#define RXSTOEST_KALMAN_CNTL_MDMHP_STO_KALMAN_DEL_1MHZ_MASK   ((uint32_t)0x00000FFF)
#define RXSTOEST_KALMAN_CNTL_MDMHP_STO_KALMAN_DEL_1MHZ_LSB    0
#define RXSTOEST_KALMAN_CNTL_MDMHP_STO_KALMAN_DEL_1MHZ_WIDTH  ((uint32_t)0x0000000C)

#define RXSTOEST_KALMAN_CNTL_MDMHP_STO_KALMAN_QR_RST          0x3
#define RXSTOEST_KALMAN_CNTL_MDMHP_STO_KALMAN_DEL_2MHZ_RST    0xF9
#define RXSTOEST_KALMAN_CNTL_MDMHP_STO_KALMAN_DEL_1MHZ_RST    0x1F3

__INLINE void mdmhp_rxstoest_kalman_cntl_pack(uint8_t stokalmanqr, uint16_t stokalmandel2mhz, uint16_t stokalmandel1mhz)
{
    ASSERT_ERR((((uint32_t)stokalmanqr << 28) & ~((uint32_t)0x70000000)) == 0);
    ASSERT_ERR((((uint32_t)stokalmandel2mhz << 16) & ~((uint32_t)0x0FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)stokalmandel1mhz << 0) & ~((uint32_t)0x00000FFF)) == 0);
    REG_IP_WR(RXSTOEST_KALMAN_CNTL_MDMHP_RXSTOEST_KALMAN_CNTL_ADDR,  ((uint32_t)stokalmanqr << 28) | ((uint32_t)stokalmandel2mhz << 16) | ((uint32_t)stokalmandel1mhz << 0));
}

__INLINE void mdmhp_rxstoest_kalman_cntl_unpack(uint8_t* stokalmanqr, uint16_t* stokalmandel2mhz, uint16_t* stokalmandel1mhz)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_KALMAN_CNTL_MDMHP_RXSTOEST_KALMAN_CNTL_ADDR);

    *stokalmanqr = (localVal & ((uint32_t)0x70000000)) >> 28;
    *stokalmandel2mhz = (localVal & ((uint32_t)0x0FFF0000)) >> 16;
    *stokalmandel1mhz = (localVal & ((uint32_t)0x00000FFF)) >> 0;
}

__INLINE uint8_t mdmhp_rxstoest_kalman_cntl_sto_kalman_qr_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_KALMAN_CNTL_MDMHP_RXSTOEST_KALMAN_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x70000000)) >> 28);
}

__INLINE void mdmhp_rxstoest_kalman_cntl_sto_kalman_qr_setf(uint8_t stokalmanqr)
{
    ASSERT_ERR((((uint32_t)stokalmanqr << 28) & ~((uint32_t)0x70000000)) == 0);
    REG_IP_WR(RXSTOEST_KALMAN_CNTL_MDMHP_RXSTOEST_KALMAN_CNTL_ADDR, (REG_IP_RD(RXSTOEST_KALMAN_CNTL_MDMHP_RXSTOEST_KALMAN_CNTL_ADDR) & ~((uint32_t)0x70000000)) | ((uint32_t)stokalmanqr << 28));
}

__INLINE uint16_t mdmhp_rxstoest_kalman_cntl_sto_kalman_del_2mhz_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_KALMAN_CNTL_MDMHP_RXSTOEST_KALMAN_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x0FFF0000)) >> 16);
}

__INLINE void mdmhp_rxstoest_kalman_cntl_sto_kalman_del_2mhz_setf(uint16_t stokalmandel2mhz)
{
    ASSERT_ERR((((uint32_t)stokalmandel2mhz << 16) & ~((uint32_t)0x0FFF0000)) == 0);
    REG_IP_WR(RXSTOEST_KALMAN_CNTL_MDMHP_RXSTOEST_KALMAN_CNTL_ADDR, (REG_IP_RD(RXSTOEST_KALMAN_CNTL_MDMHP_RXSTOEST_KALMAN_CNTL_ADDR) & ~((uint32_t)0x0FFF0000)) | ((uint32_t)stokalmandel2mhz << 16));
}

__INLINE uint16_t mdmhp_rxstoest_kalman_cntl_sto_kalman_del_1mhz_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOEST_KALMAN_CNTL_MDMHP_RXSTOEST_KALMAN_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000FFF)) >> 0);
}

__INLINE void mdmhp_rxstoest_kalman_cntl_sto_kalman_del_1mhz_setf(uint16_t stokalmandel1mhz)
{
    ASSERT_ERR((((uint32_t)stokalmandel1mhz << 0) & ~((uint32_t)0x00000FFF)) == 0);
    REG_IP_WR(RXSTOEST_KALMAN_CNTL_MDMHP_RXSTOEST_KALMAN_CNTL_ADDR, (REG_IP_RD(RXSTOEST_KALMAN_CNTL_MDMHP_RXSTOEST_KALMAN_CNTL_ADDR) & ~((uint32_t)0x00000FFF)) | ((uint32_t)stokalmandel1mhz << 0));
}

/**
 * @brief RXSTOCOMP_CNTL_BTBLE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     28    STO_COMP_FORCE_BT   1
 *  26:16      STO_COMP_VAL_BT   0x0
 *     12   STO_COMP_FORCE_BLE   1
 *  10:00     STO_COMP_VAL_BLE   0x0
 * </pre>
 */
#define RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_ADDR   0x210200D8
#define RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_OFFSET 0x000000D8
#define RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_INDEX  0x00000036
#define RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_RESET  0x10001000

__INLINE uint32_t mdmhp_rxstocomp_cntl_btble_get(void)
{
    return REG_IP_RD(RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_ADDR);
}

__INLINE void mdmhp_rxstocomp_cntl_btble_set(uint32_t value)
{
    REG_IP_WR(RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_ADDR, value);
}

// field definitions
#define RXSTOCOMP_CNTL_BTBLE_MDMHP_STO_COMP_FORCE_BT_BIT     ((uint32_t)0x10000000)
#define RXSTOCOMP_CNTL_BTBLE_MDMHP_STO_COMP_FORCE_BT_POS     28
#define RXSTOCOMP_CNTL_BTBLE_MDMHP_STO_COMP_VAL_BT_MASK      ((uint32_t)0x07FF0000)
#define RXSTOCOMP_CNTL_BTBLE_MDMHP_STO_COMP_VAL_BT_LSB       16
#define RXSTOCOMP_CNTL_BTBLE_MDMHP_STO_COMP_VAL_BT_WIDTH     ((uint32_t)0x0000000B)
#define RXSTOCOMP_CNTL_BTBLE_MDMHP_STO_COMP_FORCE_BLE_BIT    ((uint32_t)0x00001000)
#define RXSTOCOMP_CNTL_BTBLE_MDMHP_STO_COMP_FORCE_BLE_POS    12
#define RXSTOCOMP_CNTL_BTBLE_MDMHP_STO_COMP_VAL_BLE_MASK     ((uint32_t)0x000007FF)
#define RXSTOCOMP_CNTL_BTBLE_MDMHP_STO_COMP_VAL_BLE_LSB      0
#define RXSTOCOMP_CNTL_BTBLE_MDMHP_STO_COMP_VAL_BLE_WIDTH    ((uint32_t)0x0000000B)

#define RXSTOCOMP_CNTL_BTBLE_MDMHP_STO_COMP_FORCE_BT_RST     0x1
#define RXSTOCOMP_CNTL_BTBLE_MDMHP_STO_COMP_VAL_BT_RST       0x0
#define RXSTOCOMP_CNTL_BTBLE_MDMHP_STO_COMP_FORCE_BLE_RST    0x1
#define RXSTOCOMP_CNTL_BTBLE_MDMHP_STO_COMP_VAL_BLE_RST      0x0

__INLINE void mdmhp_rxstocomp_cntl_btble_pack(uint8_t stocompforcebt, uint16_t stocompvalbt, uint8_t stocompforceble, uint16_t stocompvalble)
{
    ASSERT_ERR((((uint32_t)stocompforcebt << 28) & ~((uint32_t)0x10000000)) == 0);
    ASSERT_ERR((((uint32_t)stocompvalbt << 16) & ~((uint32_t)0x07FF0000)) == 0);
    ASSERT_ERR((((uint32_t)stocompforceble << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)stocompvalble << 0) & ~((uint32_t)0x000007FF)) == 0);
    REG_IP_WR(RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_ADDR,  ((uint32_t)stocompforcebt << 28) | ((uint32_t)stocompvalbt << 16) | ((uint32_t)stocompforceble << 12) | ((uint32_t)stocompvalble << 0));
}

__INLINE void mdmhp_rxstocomp_cntl_btble_unpack(uint8_t* stocompforcebt, uint16_t* stocompvalbt, uint8_t* stocompforceble, uint16_t* stocompvalble)
{
    uint32_t localVal = REG_IP_RD(RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_ADDR);

    *stocompforcebt = (localVal & ((uint32_t)0x10000000)) >> 28;
    *stocompvalbt = (localVal & ((uint32_t)0x07FF0000)) >> 16;
    *stocompforceble = (localVal & ((uint32_t)0x00001000)) >> 12;
    *stocompvalble = (localVal & ((uint32_t)0x000007FF)) >> 0;
}

__INLINE uint8_t mdmhp_rxstocomp_cntl_btble_sto_comp_force_bt_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void mdmhp_rxstocomp_cntl_btble_sto_comp_force_bt_setf(uint8_t stocompforcebt)
{
    ASSERT_ERR((((uint32_t)stocompforcebt << 28) & ~((uint32_t)0x10000000)) == 0);
    REG_IP_WR(RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_ADDR, (REG_IP_RD(RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)stocompforcebt << 28));
}

__INLINE uint16_t mdmhp_rxstocomp_cntl_btble_sto_comp_val_bt_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_ADDR);
    return ((localVal & ((uint32_t)0x07FF0000)) >> 16);
}

__INLINE void mdmhp_rxstocomp_cntl_btble_sto_comp_val_bt_setf(uint16_t stocompvalbt)
{
    ASSERT_ERR((((uint32_t)stocompvalbt << 16) & ~((uint32_t)0x07FF0000)) == 0);
    REG_IP_WR(RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_ADDR, (REG_IP_RD(RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_ADDR) & ~((uint32_t)0x07FF0000)) | ((uint32_t)stocompvalbt << 16));
}

__INLINE uint8_t mdmhp_rxstocomp_cntl_btble_sto_comp_force_ble_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void mdmhp_rxstocomp_cntl_btble_sto_comp_force_ble_setf(uint8_t stocompforceble)
{
    ASSERT_ERR((((uint32_t)stocompforceble << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_IP_WR(RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_ADDR, (REG_IP_RD(RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)stocompforceble << 12));
}

__INLINE uint16_t mdmhp_rxstocomp_cntl_btble_sto_comp_val_ble_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_ADDR);
    return ((localVal & ((uint32_t)0x000007FF)) >> 0);
}

__INLINE void mdmhp_rxstocomp_cntl_btble_sto_comp_val_ble_setf(uint16_t stocompvalble)
{
    ASSERT_ERR((((uint32_t)stocompvalble << 0) & ~((uint32_t)0x000007FF)) == 0);
    REG_IP_WR(RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_ADDR, (REG_IP_RD(RXSTOCOMP_CNTL_BTBLE_MDMHP_RXSTOCOMP_CNTL_BTBLE_ADDR) & ~((uint32_t)0x000007FF)) | ((uint32_t)stocompvalble << 0));
}

/**
 * @brief RX_CNTL_LR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     30          LR_CI_FORCE   0
 *  29:28            LR_CI_VAL   0x0
 *     24         LR_CFO_FORCE   0
 *  22:16           LR_CFO_VAL   0x0
 *     12    STO_COMP_FORCE_LR   0
 *  10:00      STO_COMP_VAL_LR   0x0
 * </pre>
 */
#define RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR   0x210200DC
#define RX_CNTL_LR_MDMHP_RX_CNTL_LR_OFFSET 0x000000DC
#define RX_CNTL_LR_MDMHP_RX_CNTL_LR_INDEX  0x00000037
#define RX_CNTL_LR_MDMHP_RX_CNTL_LR_RESET  0x00000000

__INLINE uint32_t mdmhp_rx_cntl_lr_get(void)
{
    return REG_IP_RD(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR);
}

__INLINE void mdmhp_rx_cntl_lr_set(uint32_t value)
{
    REG_IP_WR(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR, value);
}

// field definitions
#define RX_CNTL_LR_MDMHP_LR_CI_FORCE_BIT          ((uint32_t)0x40000000)
#define RX_CNTL_LR_MDMHP_LR_CI_FORCE_POS          30
#define RX_CNTL_LR_MDMHP_LR_CI_VAL_MASK           ((uint32_t)0x30000000)
#define RX_CNTL_LR_MDMHP_LR_CI_VAL_LSB            28
#define RX_CNTL_LR_MDMHP_LR_CI_VAL_WIDTH          ((uint32_t)0x00000002)
#define RX_CNTL_LR_MDMHP_LR_CFO_FORCE_BIT         ((uint32_t)0x01000000)
#define RX_CNTL_LR_MDMHP_LR_CFO_FORCE_POS         24
#define RX_CNTL_LR_MDMHP_LR_CFO_VAL_MASK          ((uint32_t)0x007F0000)
#define RX_CNTL_LR_MDMHP_LR_CFO_VAL_LSB           16
#define RX_CNTL_LR_MDMHP_LR_CFO_VAL_WIDTH         ((uint32_t)0x00000007)
#define RX_CNTL_LR_MDMHP_STO_COMP_FORCE_LR_BIT    ((uint32_t)0x00001000)
#define RX_CNTL_LR_MDMHP_STO_COMP_FORCE_LR_POS    12
#define RX_CNTL_LR_MDMHP_STO_COMP_VAL_LR_MASK     ((uint32_t)0x000007FF)
#define RX_CNTL_LR_MDMHP_STO_COMP_VAL_LR_LSB      0
#define RX_CNTL_LR_MDMHP_STO_COMP_VAL_LR_WIDTH    ((uint32_t)0x0000000B)

#define RX_CNTL_LR_MDMHP_LR_CI_FORCE_RST          0x0
#define RX_CNTL_LR_MDMHP_LR_CI_VAL_RST            0x0
#define RX_CNTL_LR_MDMHP_LR_CFO_FORCE_RST         0x0
#define RX_CNTL_LR_MDMHP_LR_CFO_VAL_RST           0x0
#define RX_CNTL_LR_MDMHP_STO_COMP_FORCE_LR_RST    0x0
#define RX_CNTL_LR_MDMHP_STO_COMP_VAL_LR_RST      0x0

__INLINE void mdmhp_rx_cntl_lr_pack(uint8_t lrciforce, uint8_t lrcival, uint8_t lrcfoforce, uint8_t lrcfoval, uint8_t stocompforcelr, uint16_t stocompvallr)
{
    ASSERT_ERR((((uint32_t)lrciforce << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)lrcival << 28) & ~((uint32_t)0x30000000)) == 0);
    ASSERT_ERR((((uint32_t)lrcfoforce << 24) & ~((uint32_t)0x01000000)) == 0);
    ASSERT_ERR((((uint32_t)lrcfoval << 16) & ~((uint32_t)0x007F0000)) == 0);
    ASSERT_ERR((((uint32_t)stocompforcelr << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)stocompvallr << 0) & ~((uint32_t)0x000007FF)) == 0);
    REG_IP_WR(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR,  ((uint32_t)lrciforce << 30) | ((uint32_t)lrcival << 28) | ((uint32_t)lrcfoforce << 24) | ((uint32_t)lrcfoval << 16) | ((uint32_t)stocompforcelr << 12) | ((uint32_t)stocompvallr << 0));
}

__INLINE void mdmhp_rx_cntl_lr_unpack(uint8_t* lrciforce, uint8_t* lrcival, uint8_t* lrcfoforce, uint8_t* lrcfoval, uint8_t* stocompforcelr, uint16_t* stocompvallr)
{
    uint32_t localVal = REG_IP_RD(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR);

    *lrciforce = (localVal & ((uint32_t)0x40000000)) >> 30;
    *lrcival = (localVal & ((uint32_t)0x30000000)) >> 28;
    *lrcfoforce = (localVal & ((uint32_t)0x01000000)) >> 24;
    *lrcfoval = (localVal & ((uint32_t)0x007F0000)) >> 16;
    *stocompforcelr = (localVal & ((uint32_t)0x00001000)) >> 12;
    *stocompvallr = (localVal & ((uint32_t)0x000007FF)) >> 0;
}

__INLINE uint8_t mdmhp_rx_cntl_lr_lr_ci_force_getf(void)
{
    uint32_t localVal = REG_IP_RD(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void mdmhp_rx_cntl_lr_lr_ci_force_setf(uint8_t lrciforce)
{
    ASSERT_ERR((((uint32_t)lrciforce << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_IP_WR(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR, (REG_IP_RD(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)lrciforce << 30));
}

__INLINE uint8_t mdmhp_rx_cntl_lr_lr_ci_val_getf(void)
{
    uint32_t localVal = REG_IP_RD(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR);
    return ((localVal & ((uint32_t)0x30000000)) >> 28);
}

__INLINE void mdmhp_rx_cntl_lr_lr_ci_val_setf(uint8_t lrcival)
{
    ASSERT_ERR((((uint32_t)lrcival << 28) & ~((uint32_t)0x30000000)) == 0);
    REG_IP_WR(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR, (REG_IP_RD(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR) & ~((uint32_t)0x30000000)) | ((uint32_t)lrcival << 28));
}

__INLINE uint8_t mdmhp_rx_cntl_lr_lr_cfo_force_getf(void)
{
    uint32_t localVal = REG_IP_RD(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR);
    return ((localVal & ((uint32_t)0x01000000)) >> 24);
}

__INLINE void mdmhp_rx_cntl_lr_lr_cfo_force_setf(uint8_t lrcfoforce)
{
    ASSERT_ERR((((uint32_t)lrcfoforce << 24) & ~((uint32_t)0x01000000)) == 0);
    REG_IP_WR(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR, (REG_IP_RD(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR) & ~((uint32_t)0x01000000)) | ((uint32_t)lrcfoforce << 24));
}

__INLINE uint8_t mdmhp_rx_cntl_lr_lr_cfo_val_getf(void)
{
    uint32_t localVal = REG_IP_RD(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR);
    return ((localVal & ((uint32_t)0x007F0000)) >> 16);
}

__INLINE void mdmhp_rx_cntl_lr_lr_cfo_val_setf(uint8_t lrcfoval)
{
    ASSERT_ERR((((uint32_t)lrcfoval << 16) & ~((uint32_t)0x007F0000)) == 0);
    REG_IP_WR(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR, (REG_IP_RD(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR) & ~((uint32_t)0x007F0000)) | ((uint32_t)lrcfoval << 16));
}

__INLINE uint8_t mdmhp_rx_cntl_lr_sto_comp_force_lr_getf(void)
{
    uint32_t localVal = REG_IP_RD(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void mdmhp_rx_cntl_lr_sto_comp_force_lr_setf(uint8_t stocompforcelr)
{
    ASSERT_ERR((((uint32_t)stocompforcelr << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_IP_WR(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR, (REG_IP_RD(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)stocompforcelr << 12));
}

__INLINE uint16_t mdmhp_rx_cntl_lr_sto_comp_val_lr_getf(void)
{
    uint32_t localVal = REG_IP_RD(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR);
    return ((localVal & ((uint32_t)0x000007FF)) >> 0);
}

__INLINE void mdmhp_rx_cntl_lr_sto_comp_val_lr_setf(uint16_t stocompvallr)
{
    ASSERT_ERR((((uint32_t)stocompvallr << 0) & ~((uint32_t)0x000007FF)) == 0);
    REG_IP_WR(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR, (REG_IP_RD(RX_CNTL_LR_MDMHP_RX_CNTL_LR_ADDR) & ~((uint32_t)0x000007FF)) | ((uint32_t)stocompvallr << 0));
}

/**
 * @brief RXLPDEMOD_CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  26:24         LP_CFD_GAMMA   0x4
 *     20       LP_CFD_COMP_EN   1
 *     16        LP_DFE_BYPASS   0
 *     12         LP_CFO_FORCE   0
 *  07:00           LP_CFO_VAL   0x0
 * </pre>
 */
#define RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR   0x210200E0
#define RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_OFFSET 0x000000E0
#define RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_INDEX  0x00000038
#define RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_RESET  0x04100000

__INLINE uint32_t mdmhp_rxlpdemod_cntl_get(void)
{
    return REG_IP_RD(RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR);
}

__INLINE void mdmhp_rxlpdemod_cntl_set(uint32_t value)
{
    REG_IP_WR(RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR, value);
}

// field definitions
#define RXLPDEMOD_CNTL_MDMHP_LP_CFD_GAMMA_MASK     ((uint32_t)0x07000000)
#define RXLPDEMOD_CNTL_MDMHP_LP_CFD_GAMMA_LSB      24
#define RXLPDEMOD_CNTL_MDMHP_LP_CFD_GAMMA_WIDTH    ((uint32_t)0x00000003)
#define RXLPDEMOD_CNTL_MDMHP_LP_CFD_COMP_EN_BIT    ((uint32_t)0x00100000)
#define RXLPDEMOD_CNTL_MDMHP_LP_CFD_COMP_EN_POS    20
#define RXLPDEMOD_CNTL_MDMHP_LP_DFE_BYPASS_BIT     ((uint32_t)0x00010000)
#define RXLPDEMOD_CNTL_MDMHP_LP_DFE_BYPASS_POS     16
#define RXLPDEMOD_CNTL_MDMHP_LP_CFO_FORCE_BIT      ((uint32_t)0x00001000)
#define RXLPDEMOD_CNTL_MDMHP_LP_CFO_FORCE_POS      12
#define RXLPDEMOD_CNTL_MDMHP_LP_CFO_VAL_MASK       ((uint32_t)0x000000FF)
#define RXLPDEMOD_CNTL_MDMHP_LP_CFO_VAL_LSB        0
#define RXLPDEMOD_CNTL_MDMHP_LP_CFO_VAL_WIDTH      ((uint32_t)0x00000008)

#define RXLPDEMOD_CNTL_MDMHP_LP_CFD_GAMMA_RST      0x4
#define RXLPDEMOD_CNTL_MDMHP_LP_CFD_COMP_EN_RST    0x1
#define RXLPDEMOD_CNTL_MDMHP_LP_DFE_BYPASS_RST     0x0
#define RXLPDEMOD_CNTL_MDMHP_LP_CFO_FORCE_RST      0x0
#define RXLPDEMOD_CNTL_MDMHP_LP_CFO_VAL_RST        0x0

__INLINE void mdmhp_rxlpdemod_cntl_pack(uint8_t lpcfdgamma, uint8_t lpcfdcompen, uint8_t lpdfebypass, uint8_t lpcfoforce, uint8_t lpcfoval)
{
    ASSERT_ERR((((uint32_t)lpcfdgamma << 24) & ~((uint32_t)0x07000000)) == 0);
    ASSERT_ERR((((uint32_t)lpcfdcompen << 20) & ~((uint32_t)0x00100000)) == 0);
    ASSERT_ERR((((uint32_t)lpdfebypass << 16) & ~((uint32_t)0x00010000)) == 0);
    ASSERT_ERR((((uint32_t)lpcfoforce << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)lpcfoval << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR,  ((uint32_t)lpcfdgamma << 24) | ((uint32_t)lpcfdcompen << 20) | ((uint32_t)lpdfebypass << 16) | ((uint32_t)lpcfoforce << 12) | ((uint32_t)lpcfoval << 0));
}

__INLINE void mdmhp_rxlpdemod_cntl_unpack(uint8_t* lpcfdgamma, uint8_t* lpcfdcompen, uint8_t* lpdfebypass, uint8_t* lpcfoforce, uint8_t* lpcfoval)
{
    uint32_t localVal = REG_IP_RD(RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR);

    *lpcfdgamma = (localVal & ((uint32_t)0x07000000)) >> 24;
    *lpcfdcompen = (localVal & ((uint32_t)0x00100000)) >> 20;
    *lpdfebypass = (localVal & ((uint32_t)0x00010000)) >> 16;
    *lpcfoforce = (localVal & ((uint32_t)0x00001000)) >> 12;
    *lpcfoval = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t mdmhp_rxlpdemod_cntl_lp_cfd_gamma_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x07000000)) >> 24);
}

__INLINE void mdmhp_rxlpdemod_cntl_lp_cfd_gamma_setf(uint8_t lpcfdgamma)
{
    ASSERT_ERR((((uint32_t)lpcfdgamma << 24) & ~((uint32_t)0x07000000)) == 0);
    REG_IP_WR(RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR, (REG_IP_RD(RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR) & ~((uint32_t)0x07000000)) | ((uint32_t)lpcfdgamma << 24));
}

__INLINE uint8_t mdmhp_rxlpdemod_cntl_lp_cfd_comp_en_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE void mdmhp_rxlpdemod_cntl_lp_cfd_comp_en_setf(uint8_t lpcfdcompen)
{
    ASSERT_ERR((((uint32_t)lpcfdcompen << 20) & ~((uint32_t)0x00100000)) == 0);
    REG_IP_WR(RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR, (REG_IP_RD(RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR) & ~((uint32_t)0x00100000)) | ((uint32_t)lpcfdcompen << 20));
}

__INLINE uint8_t mdmhp_rxlpdemod_cntl_lp_dfe_bypass_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void mdmhp_rxlpdemod_cntl_lp_dfe_bypass_setf(uint8_t lpdfebypass)
{
    ASSERT_ERR((((uint32_t)lpdfebypass << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_IP_WR(RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR, (REG_IP_RD(RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)lpdfebypass << 16));
}

__INLINE uint8_t mdmhp_rxlpdemod_cntl_lp_cfo_force_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void mdmhp_rxlpdemod_cntl_lp_cfo_force_setf(uint8_t lpcfoforce)
{
    ASSERT_ERR((((uint32_t)lpcfoforce << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_IP_WR(RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR, (REG_IP_RD(RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)lpcfoforce << 12));
}

__INLINE uint8_t mdmhp_rxlpdemod_cntl_lp_cfo_val_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void mdmhp_rxlpdemod_cntl_lp_cfo_val_setf(uint8_t lpcfoval)
{
    ASSERT_ERR((((uint32_t)lpcfoval << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_IP_WR(RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR, (REG_IP_RD(RXLPDEMOD_CNTL_MDMHP_RXLPDEMOD_CNTL_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)lpcfoval << 0));
}

/**
 * @brief RXHPDEMOD_QCOEFF_BLE1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:24         HP_Q1_2_BLE1   0x1
 *  23:20         HP_Q1_1_BLE1   0x0
 *  19:16         HP_Q1_0_BLE1   0x0
 *  11:08         HP_Q0_2_BLE1   0x4
 *  07:04         HP_Q0_1_BLE1   0x3
 *  03:00         HP_Q0_0_BLE1   0x2
 * </pre>
 */
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR   0x210200BC
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_OFFSET 0x000000BC
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_INDEX  0x0000002F
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_RESET  0x01000432

__INLINE uint32_t mdmhp_rxhpdemod_qcoeff_ble1_get(void)
{
    return REG_IP_RD(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_ble1_set(uint32_t value)
{
    REG_IP_WR(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR, value);
}

// field definitions
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q1_2_BLE1_MASK   ((uint32_t)0x0F000000)
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q1_2_BLE1_LSB    24
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q1_2_BLE1_WIDTH  ((uint32_t)0x00000004)
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q1_1_BLE1_MASK   ((uint32_t)0x00F00000)
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q1_1_BLE1_LSB    20
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q1_1_BLE1_WIDTH  ((uint32_t)0x00000004)
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q1_0_BLE1_MASK   ((uint32_t)0x000F0000)
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q1_0_BLE1_LSB    16
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q1_0_BLE1_WIDTH  ((uint32_t)0x00000004)
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q0_2_BLE1_MASK   ((uint32_t)0x00000F00)
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q0_2_BLE1_LSB    8
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q0_2_BLE1_WIDTH  ((uint32_t)0x00000004)
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q0_1_BLE1_MASK   ((uint32_t)0x000000F0)
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q0_1_BLE1_LSB    4
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q0_1_BLE1_WIDTH  ((uint32_t)0x00000004)
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q0_0_BLE1_MASK   ((uint32_t)0x0000000F)
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q0_0_BLE1_LSB    0
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q0_0_BLE1_WIDTH  ((uint32_t)0x00000004)

#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q1_2_BLE1_RST    0x1
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q1_1_BLE1_RST    0x0
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q1_0_BLE1_RST    0x0
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q0_2_BLE1_RST    0x4
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q0_1_BLE1_RST    0x3
#define RXHPDEMOD_QCOEFF_BLE1_MDMHP_HP_Q0_0_BLE1_RST    0x2

__INLINE void mdmhp_rxhpdemod_qcoeff_ble1_pack(uint8_t hpq12ble1, uint8_t hpq11ble1, uint8_t hpq10ble1, uint8_t hpq02ble1, uint8_t hpq01ble1, uint8_t hpq00ble1)
{
    ASSERT_ERR((((uint32_t)hpq12ble1 << 24) & ~((uint32_t)0x0F000000)) == 0);
    ASSERT_ERR((((uint32_t)hpq11ble1 << 20) & ~((uint32_t)0x00F00000)) == 0);
    ASSERT_ERR((((uint32_t)hpq10ble1 << 16) & ~((uint32_t)0x000F0000)) == 0);
    ASSERT_ERR((((uint32_t)hpq02ble1 << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)hpq01ble1 << 4) & ~((uint32_t)0x000000F0)) == 0);
    ASSERT_ERR((((uint32_t)hpq00ble1 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR,  ((uint32_t)hpq12ble1 << 24) | ((uint32_t)hpq11ble1 << 20) | ((uint32_t)hpq10ble1 << 16) | ((uint32_t)hpq02ble1 << 8) | ((uint32_t)hpq01ble1 << 4) | ((uint32_t)hpq00ble1 << 0));
}

__INLINE void mdmhp_rxhpdemod_qcoeff_ble1_unpack(uint8_t* hpq12ble1, uint8_t* hpq11ble1, uint8_t* hpq10ble1, uint8_t* hpq02ble1, uint8_t* hpq01ble1, uint8_t* hpq00ble1)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR);

    *hpq12ble1 = (localVal & ((uint32_t)0x0F000000)) >> 24;
    *hpq11ble1 = (localVal & ((uint32_t)0x00F00000)) >> 20;
    *hpq10ble1 = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *hpq02ble1 = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *hpq01ble1 = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *hpq00ble1 = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t mdmhp_rxhpdemod_qcoeff_ble1_hp_q1_2_ble1_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR);
    return ((localVal & ((uint32_t)0x0F000000)) >> 24);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_ble1_hp_q1_2_ble1_setf(uint8_t hpq12ble1)
{
    ASSERT_ERR((((uint32_t)hpq12ble1 << 24) & ~((uint32_t)0x0F000000)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR, (REG_IP_RD(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR) & ~((uint32_t)0x0F000000)) | ((uint32_t)hpq12ble1 << 24));
}

__INLINE uint8_t mdmhp_rxhpdemod_qcoeff_ble1_hp_q1_1_ble1_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR);
    return ((localVal & ((uint32_t)0x00F00000)) >> 20);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_ble1_hp_q1_1_ble1_setf(uint8_t hpq11ble1)
{
    ASSERT_ERR((((uint32_t)hpq11ble1 << 20) & ~((uint32_t)0x00F00000)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR, (REG_IP_RD(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR) & ~((uint32_t)0x00F00000)) | ((uint32_t)hpq11ble1 << 20));
}

__INLINE uint8_t mdmhp_rxhpdemod_qcoeff_ble1_hp_q1_0_ble1_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_ble1_hp_q1_0_ble1_setf(uint8_t hpq10ble1)
{
    ASSERT_ERR((((uint32_t)hpq10ble1 << 16) & ~((uint32_t)0x000F0000)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR, (REG_IP_RD(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)hpq10ble1 << 16));
}

__INLINE uint8_t mdmhp_rxhpdemod_qcoeff_ble1_hp_q0_2_ble1_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_ble1_hp_q0_2_ble1_setf(uint8_t hpq02ble1)
{
    ASSERT_ERR((((uint32_t)hpq02ble1 << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR, (REG_IP_RD(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)hpq02ble1 << 8));
}

__INLINE uint8_t mdmhp_rxhpdemod_qcoeff_ble1_hp_q0_1_ble1_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_ble1_hp_q0_1_ble1_setf(uint8_t hpq01ble1)
{
    ASSERT_ERR((((uint32_t)hpq01ble1 << 4) & ~((uint32_t)0x000000F0)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR, (REG_IP_RD(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)hpq01ble1 << 4));
}

__INLINE uint8_t mdmhp_rxhpdemod_qcoeff_ble1_hp_q0_0_ble1_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_ble1_hp_q0_0_ble1_setf(uint8_t hpq00ble1)
{
    ASSERT_ERR((((uint32_t)hpq00ble1 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR, (REG_IP_RD(RXHPDEMOD_QCOEFF_BLE1_MDMHP_RXHPDEMOD_QCOEFF_BLE1_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)hpq00ble1 << 0));
}

/**
 * @brief RXHPDEMOD_QCOEFF_BLE2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:24         HP_Q1_2_BLE2   0x0
 *  23:20         HP_Q1_1_BLE2   0x0
 *  19:16         HP_Q1_0_BLE2   0x0
 *  11:08         HP_Q0_2_BLE2   0x4
 *  07:04         HP_Q0_1_BLE2   0x3
 *  03:00         HP_Q0_0_BLE2   0x2
 * </pre>
 */
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR   0x210200C0
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_OFFSET 0x000000C0
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_INDEX  0x00000030
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_RESET  0x00000432

__INLINE uint32_t mdmhp_rxhpdemod_qcoeff_ble2_get(void)
{
    return REG_IP_RD(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_ble2_set(uint32_t value)
{
    REG_IP_WR(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR, value);
}

// field definitions
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q1_2_BLE2_MASK   ((uint32_t)0x0F000000)
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q1_2_BLE2_LSB    24
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q1_2_BLE2_WIDTH  ((uint32_t)0x00000004)
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q1_1_BLE2_MASK   ((uint32_t)0x00F00000)
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q1_1_BLE2_LSB    20
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q1_1_BLE2_WIDTH  ((uint32_t)0x00000004)
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q1_0_BLE2_MASK   ((uint32_t)0x000F0000)
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q1_0_BLE2_LSB    16
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q1_0_BLE2_WIDTH  ((uint32_t)0x00000004)
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q0_2_BLE2_MASK   ((uint32_t)0x00000F00)
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q0_2_BLE2_LSB    8
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q0_2_BLE2_WIDTH  ((uint32_t)0x00000004)
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q0_1_BLE2_MASK   ((uint32_t)0x000000F0)
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q0_1_BLE2_LSB    4
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q0_1_BLE2_WIDTH  ((uint32_t)0x00000004)
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q0_0_BLE2_MASK   ((uint32_t)0x0000000F)
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q0_0_BLE2_LSB    0
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q0_0_BLE2_WIDTH  ((uint32_t)0x00000004)

#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q1_2_BLE2_RST    0x0
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q1_1_BLE2_RST    0x0
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q1_0_BLE2_RST    0x0
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q0_2_BLE2_RST    0x4
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q0_1_BLE2_RST    0x3
#define RXHPDEMOD_QCOEFF_BLE2_MDMHP_HP_Q0_0_BLE2_RST    0x2

__INLINE void mdmhp_rxhpdemod_qcoeff_ble2_pack(uint8_t hpq12ble2, uint8_t hpq11ble2, uint8_t hpq10ble2, uint8_t hpq02ble2, uint8_t hpq01ble2, uint8_t hpq00ble2)
{
    ASSERT_ERR((((uint32_t)hpq12ble2 << 24) & ~((uint32_t)0x0F000000)) == 0);
    ASSERT_ERR((((uint32_t)hpq11ble2 << 20) & ~((uint32_t)0x00F00000)) == 0);
    ASSERT_ERR((((uint32_t)hpq10ble2 << 16) & ~((uint32_t)0x000F0000)) == 0);
    ASSERT_ERR((((uint32_t)hpq02ble2 << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)hpq01ble2 << 4) & ~((uint32_t)0x000000F0)) == 0);
    ASSERT_ERR((((uint32_t)hpq00ble2 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR,  ((uint32_t)hpq12ble2 << 24) | ((uint32_t)hpq11ble2 << 20) | ((uint32_t)hpq10ble2 << 16) | ((uint32_t)hpq02ble2 << 8) | ((uint32_t)hpq01ble2 << 4) | ((uint32_t)hpq00ble2 << 0));
}

__INLINE void mdmhp_rxhpdemod_qcoeff_ble2_unpack(uint8_t* hpq12ble2, uint8_t* hpq11ble2, uint8_t* hpq10ble2, uint8_t* hpq02ble2, uint8_t* hpq01ble2, uint8_t* hpq00ble2)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR);

    *hpq12ble2 = (localVal & ((uint32_t)0x0F000000)) >> 24;
    *hpq11ble2 = (localVal & ((uint32_t)0x00F00000)) >> 20;
    *hpq10ble2 = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *hpq02ble2 = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *hpq01ble2 = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *hpq00ble2 = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t mdmhp_rxhpdemod_qcoeff_ble2_hp_q1_2_ble2_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR);
    return ((localVal & ((uint32_t)0x0F000000)) >> 24);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_ble2_hp_q1_2_ble2_setf(uint8_t hpq12ble2)
{
    ASSERT_ERR((((uint32_t)hpq12ble2 << 24) & ~((uint32_t)0x0F000000)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR, (REG_IP_RD(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR) & ~((uint32_t)0x0F000000)) | ((uint32_t)hpq12ble2 << 24));
}

__INLINE uint8_t mdmhp_rxhpdemod_qcoeff_ble2_hp_q1_1_ble2_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR);
    return ((localVal & ((uint32_t)0x00F00000)) >> 20);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_ble2_hp_q1_1_ble2_setf(uint8_t hpq11ble2)
{
    ASSERT_ERR((((uint32_t)hpq11ble2 << 20) & ~((uint32_t)0x00F00000)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR, (REG_IP_RD(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR) & ~((uint32_t)0x00F00000)) | ((uint32_t)hpq11ble2 << 20));
}

__INLINE uint8_t mdmhp_rxhpdemod_qcoeff_ble2_hp_q1_0_ble2_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_ble2_hp_q1_0_ble2_setf(uint8_t hpq10ble2)
{
    ASSERT_ERR((((uint32_t)hpq10ble2 << 16) & ~((uint32_t)0x000F0000)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR, (REG_IP_RD(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)hpq10ble2 << 16));
}

__INLINE uint8_t mdmhp_rxhpdemod_qcoeff_ble2_hp_q0_2_ble2_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_ble2_hp_q0_2_ble2_setf(uint8_t hpq02ble2)
{
    ASSERT_ERR((((uint32_t)hpq02ble2 << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR, (REG_IP_RD(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)hpq02ble2 << 8));
}

__INLINE uint8_t mdmhp_rxhpdemod_qcoeff_ble2_hp_q0_1_ble2_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_ble2_hp_q0_1_ble2_setf(uint8_t hpq01ble2)
{
    ASSERT_ERR((((uint32_t)hpq01ble2 << 4) & ~((uint32_t)0x000000F0)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR, (REG_IP_RD(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)hpq01ble2 << 4));
}

__INLINE uint8_t mdmhp_rxhpdemod_qcoeff_ble2_hp_q0_0_ble2_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_ble2_hp_q0_0_ble2_setf(uint8_t hpq00ble2)
{
    ASSERT_ERR((((uint32_t)hpq00ble2 << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR, (REG_IP_RD(RXHPDEMOD_QCOEFF_BLE2_MDMHP_RXHPDEMOD_QCOEFF_BLE2_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)hpq00ble2 << 0));
}

/**
 * @brief RXHPDEMOD_QCOEFF_BT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:24           HP_Q1_2_BT   0x1
 *  23:20           HP_Q1_1_BT   0x0
 *  19:16           HP_Q1_0_BT   0x0
 *  11:08           HP_Q0_2_BT   0x4
 *  07:04           HP_Q0_1_BT   0x3
 *  03:00           HP_Q0_0_BT   0x2
 * </pre>
 */
#define RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR   0x210200C4
#define RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_OFFSET 0x000000C4
#define RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_INDEX  0x00000031
#define RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_RESET  0x01000432

__INLINE uint32_t mdmhp_rxhpdemod_qcoeff_bt_get(void)
{
    return REG_IP_RD(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_bt_set(uint32_t value)
{
    REG_IP_WR(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR, value);
}

// field definitions
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q1_2_BT_MASK   ((uint32_t)0x0F000000)
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q1_2_BT_LSB    24
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q1_2_BT_WIDTH  ((uint32_t)0x00000004)
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q1_1_BT_MASK   ((uint32_t)0x00F00000)
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q1_1_BT_LSB    20
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q1_1_BT_WIDTH  ((uint32_t)0x00000004)
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q1_0_BT_MASK   ((uint32_t)0x000F0000)
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q1_0_BT_LSB    16
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q1_0_BT_WIDTH  ((uint32_t)0x00000004)
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q0_2_BT_MASK   ((uint32_t)0x00000F00)
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q0_2_BT_LSB    8
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q0_2_BT_WIDTH  ((uint32_t)0x00000004)
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q0_1_BT_MASK   ((uint32_t)0x000000F0)
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q0_1_BT_LSB    4
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q0_1_BT_WIDTH  ((uint32_t)0x00000004)
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q0_0_BT_MASK   ((uint32_t)0x0000000F)
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q0_0_BT_LSB    0
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q0_0_BT_WIDTH  ((uint32_t)0x00000004)

#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q1_2_BT_RST    0x1
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q1_1_BT_RST    0x0
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q1_0_BT_RST    0x0
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q0_2_BT_RST    0x4
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q0_1_BT_RST    0x3
#define RXHPDEMOD_QCOEFF_BT_MDMHP_HP_Q0_0_BT_RST    0x2

__INLINE void mdmhp_rxhpdemod_qcoeff_bt_pack(uint8_t hpq12bt, uint8_t hpq11bt, uint8_t hpq10bt, uint8_t hpq02bt, uint8_t hpq01bt, uint8_t hpq00bt)
{
    ASSERT_ERR((((uint32_t)hpq12bt << 24) & ~((uint32_t)0x0F000000)) == 0);
    ASSERT_ERR((((uint32_t)hpq11bt << 20) & ~((uint32_t)0x00F00000)) == 0);
    ASSERT_ERR((((uint32_t)hpq10bt << 16) & ~((uint32_t)0x000F0000)) == 0);
    ASSERT_ERR((((uint32_t)hpq02bt << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)hpq01bt << 4) & ~((uint32_t)0x000000F0)) == 0);
    ASSERT_ERR((((uint32_t)hpq00bt << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR,  ((uint32_t)hpq12bt << 24) | ((uint32_t)hpq11bt << 20) | ((uint32_t)hpq10bt << 16) | ((uint32_t)hpq02bt << 8) | ((uint32_t)hpq01bt << 4) | ((uint32_t)hpq00bt << 0));
}

__INLINE void mdmhp_rxhpdemod_qcoeff_bt_unpack(uint8_t* hpq12bt, uint8_t* hpq11bt, uint8_t* hpq10bt, uint8_t* hpq02bt, uint8_t* hpq01bt, uint8_t* hpq00bt)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR);

    *hpq12bt = (localVal & ((uint32_t)0x0F000000)) >> 24;
    *hpq11bt = (localVal & ((uint32_t)0x00F00000)) >> 20;
    *hpq10bt = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *hpq02bt = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *hpq01bt = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *hpq00bt = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t mdmhp_rxhpdemod_qcoeff_bt_hp_q1_2_bt_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR);
    return ((localVal & ((uint32_t)0x0F000000)) >> 24);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_bt_hp_q1_2_bt_setf(uint8_t hpq12bt)
{
    ASSERT_ERR((((uint32_t)hpq12bt << 24) & ~((uint32_t)0x0F000000)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR, (REG_IP_RD(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR) & ~((uint32_t)0x0F000000)) | ((uint32_t)hpq12bt << 24));
}

__INLINE uint8_t mdmhp_rxhpdemod_qcoeff_bt_hp_q1_1_bt_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR);
    return ((localVal & ((uint32_t)0x00F00000)) >> 20);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_bt_hp_q1_1_bt_setf(uint8_t hpq11bt)
{
    ASSERT_ERR((((uint32_t)hpq11bt << 20) & ~((uint32_t)0x00F00000)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR, (REG_IP_RD(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR) & ~((uint32_t)0x00F00000)) | ((uint32_t)hpq11bt << 20));
}

__INLINE uint8_t mdmhp_rxhpdemod_qcoeff_bt_hp_q1_0_bt_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_bt_hp_q1_0_bt_setf(uint8_t hpq10bt)
{
    ASSERT_ERR((((uint32_t)hpq10bt << 16) & ~((uint32_t)0x000F0000)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR, (REG_IP_RD(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)hpq10bt << 16));
}

__INLINE uint8_t mdmhp_rxhpdemod_qcoeff_bt_hp_q0_2_bt_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_bt_hp_q0_2_bt_setf(uint8_t hpq02bt)
{
    ASSERT_ERR((((uint32_t)hpq02bt << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR, (REG_IP_RD(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)hpq02bt << 8));
}

__INLINE uint8_t mdmhp_rxhpdemod_qcoeff_bt_hp_q0_1_bt_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_bt_hp_q0_1_bt_setf(uint8_t hpq01bt)
{
    ASSERT_ERR((((uint32_t)hpq01bt << 4) & ~((uint32_t)0x000000F0)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR, (REG_IP_RD(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)hpq01bt << 4));
}

__INLINE uint8_t mdmhp_rxhpdemod_qcoeff_bt_hp_q0_0_bt_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void mdmhp_rxhpdemod_qcoeff_bt_hp_q0_0_bt_setf(uint8_t hpq00bt)
{
    ASSERT_ERR((((uint32_t)hpq00bt << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_IP_WR(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR, (REG_IP_RD(RXHPDEMOD_QCOEFF_BT_MDMHP_RXHPDEMOD_QCOEFF_BT_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)hpq00bt << 0));
}

/**
 * @brief RXHPDEMOD_RHO register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:23            HP_RHO_BT   0x0
 *  20:12          HP_RHO_BLE2   0x0
 *  08:00          HP_RHO_BLE1   0x0
 * </pre>
 */
#define RXHPDEMOD_RHO_MDMHP_RXHPDEMOD_RHO_ADDR   0x210200C8
#define RXHPDEMOD_RHO_MDMHP_RXHPDEMOD_RHO_OFFSET 0x000000C8
#define RXHPDEMOD_RHO_MDMHP_RXHPDEMOD_RHO_INDEX  0x00000032
#define RXHPDEMOD_RHO_MDMHP_RXHPDEMOD_RHO_RESET  0x00000000

__INLINE uint32_t mdmhp_rxhpdemod_rho_get(void)
{
    return REG_IP_RD(RXHPDEMOD_RHO_MDMHP_RXHPDEMOD_RHO_ADDR);
}

__INLINE void mdmhp_rxhpdemod_rho_set(uint32_t value)
{
    REG_IP_WR(RXHPDEMOD_RHO_MDMHP_RXHPDEMOD_RHO_ADDR, value);
}

// field definitions
#define RXHPDEMOD_RHO_MDMHP_HP_RHO_BT_MASK     ((uint32_t)0xFF800000)
#define RXHPDEMOD_RHO_MDMHP_HP_RHO_BT_LSB      23
#define RXHPDEMOD_RHO_MDMHP_HP_RHO_BT_WIDTH    ((uint32_t)0x00000009)
#define RXHPDEMOD_RHO_MDMHP_HP_RHO_BLE2_MASK   ((uint32_t)0x001FF000)
#define RXHPDEMOD_RHO_MDMHP_HP_RHO_BLE2_LSB    12
#define RXHPDEMOD_RHO_MDMHP_HP_RHO_BLE2_WIDTH  ((uint32_t)0x00000009)
#define RXHPDEMOD_RHO_MDMHP_HP_RHO_BLE1_MASK   ((uint32_t)0x000001FF)
#define RXHPDEMOD_RHO_MDMHP_HP_RHO_BLE1_LSB    0
#define RXHPDEMOD_RHO_MDMHP_HP_RHO_BLE1_WIDTH  ((uint32_t)0x00000009)

#define RXHPDEMOD_RHO_MDMHP_HP_RHO_BT_RST      0x0
#define RXHPDEMOD_RHO_MDMHP_HP_RHO_BLE2_RST    0x0
#define RXHPDEMOD_RHO_MDMHP_HP_RHO_BLE1_RST    0x0

__INLINE void mdmhp_rxhpdemod_rho_pack(uint16_t hprhobt, uint16_t hprhoble2, uint16_t hprhoble1)
{
    ASSERT_ERR((((uint32_t)hprhobt << 23) & ~((uint32_t)0xFF800000)) == 0);
    ASSERT_ERR((((uint32_t)hprhoble2 << 12) & ~((uint32_t)0x001FF000)) == 0);
    ASSERT_ERR((((uint32_t)hprhoble1 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_IP_WR(RXHPDEMOD_RHO_MDMHP_RXHPDEMOD_RHO_ADDR,  ((uint32_t)hprhobt << 23) | ((uint32_t)hprhoble2 << 12) | ((uint32_t)hprhoble1 << 0));
}

__INLINE void mdmhp_rxhpdemod_rho_unpack(uint16_t* hprhobt, uint16_t* hprhoble2, uint16_t* hprhoble1)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_RHO_MDMHP_RXHPDEMOD_RHO_ADDR);

    *hprhobt = (localVal & ((uint32_t)0xFF800000)) >> 23;
    *hprhoble2 = (localVal & ((uint32_t)0x001FF000)) >> 12;
    *hprhoble1 = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint16_t mdmhp_rxhpdemod_rho_hp_rho_bt_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_RHO_MDMHP_RXHPDEMOD_RHO_ADDR);
    return ((localVal & ((uint32_t)0xFF800000)) >> 23);
}

__INLINE void mdmhp_rxhpdemod_rho_hp_rho_bt_setf(uint16_t hprhobt)
{
    ASSERT_ERR((((uint32_t)hprhobt << 23) & ~((uint32_t)0xFF800000)) == 0);
    REG_IP_WR(RXHPDEMOD_RHO_MDMHP_RXHPDEMOD_RHO_ADDR, (REG_IP_RD(RXHPDEMOD_RHO_MDMHP_RXHPDEMOD_RHO_ADDR) & ~((uint32_t)0xFF800000)) | ((uint32_t)hprhobt << 23));
}

__INLINE uint16_t mdmhp_rxhpdemod_rho_hp_rho_ble2_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_RHO_MDMHP_RXHPDEMOD_RHO_ADDR);
    return ((localVal & ((uint32_t)0x001FF000)) >> 12);
}

__INLINE void mdmhp_rxhpdemod_rho_hp_rho_ble2_setf(uint16_t hprhoble2)
{
    ASSERT_ERR((((uint32_t)hprhoble2 << 12) & ~((uint32_t)0x001FF000)) == 0);
    REG_IP_WR(RXHPDEMOD_RHO_MDMHP_RXHPDEMOD_RHO_ADDR, (REG_IP_RD(RXHPDEMOD_RHO_MDMHP_RXHPDEMOD_RHO_ADDR) & ~((uint32_t)0x001FF000)) | ((uint32_t)hprhoble2 << 12));
}

__INLINE uint16_t mdmhp_rxhpdemod_rho_hp_rho_ble1_getf(void)
{
    uint32_t localVal = REG_IP_RD(RXHPDEMOD_RHO_MDMHP_RXHPDEMOD_RHO_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void mdmhp_rxhpdemod_rho_hp_rho_ble1_setf(uint16_t hprhoble1)
{
    ASSERT_ERR((((uint32_t)hprhoble1 << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_IP_WR(RXHPDEMOD_RHO_MDMHP_RXHPDEMOD_RHO_ADDR, (REG_IP_RD(RXHPDEMOD_RHO_MDMHP_RXHPDEMOD_RHO_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)hprhoble1 << 0));
}


#endif // _REG_MODEMHP_H_

