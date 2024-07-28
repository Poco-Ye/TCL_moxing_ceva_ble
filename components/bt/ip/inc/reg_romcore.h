#ifndef _REG_ROMCORE_H_
#define _REG_ROMCORE_H_

#include <stdint.h>
#include "_reg_romcore.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_ROMCORE_COUNT 48

#define REG_ROMCORE_DECODING_MASK 0x000000FF

/**
 * @brief VERSION register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16                  TYP   0x0
 *  15:08                  REL   0x0
 *  07:00                  UPG   0x0
 * </pre>
 */
#define ROM_VERSION_ADDR   0x00320004
#define ROM_VERSION_OFFSET 0x00000004
#define ROM_VERSION_INDEX  0x00000001
#define ROM_VERSION_RESET  0x00000000

__INLINE uint32_t rom_version_get(void)
{
    return REG_BT_RD(ROM_VERSION_ADDR);
}

// field definitions
#define ROM_TYP_MASK   ((uint32_t)0xFFFF0000)
#define ROM_TYP_LSB    16
#define ROM_TYP_WIDTH  ((uint32_t)0x00000010)
#define ROM_REL_MASK   ((uint32_t)0x0000FF00)
#define ROM_REL_LSB    8
#define ROM_REL_WIDTH  ((uint32_t)0x00000008)
#define ROM_UPG_MASK   ((uint32_t)0x000000FF)
#define ROM_UPG_LSB    0
#define ROM_UPG_WIDTH  ((uint32_t)0x00000008)

#define ROM_TYP_RST    0x0
#define ROM_REL_RST    0x0
#define ROM_UPG_RST    0x0

__INLINE void rom_version_unpack(uint16_t* typ, uint8_t* rel, uint8_t* upg)
{
    uint32_t localVal = REG_BT_RD(ROM_VERSION_ADDR);

    *typ = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *rel = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *upg = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint16_t rom_typ_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_VERSION_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE uint8_t rom_rel_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_VERSION_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE uint8_t rom_upg_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_VERSION_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

/**
 * @brief RADIOCNTL0_DC register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     25   SYNC_PULSE_MODE_DC   0
 *     24         DPCORR_EN_DC   0
 *  19:16              SKEW_DC   0x0
 *  13:12           RXLOWIF_DC   0x2
 *     09          BIDIRCLK_DC   0
 *     08         BIDIRDATA_DC   1
 *     07            SPIPOL_DC   0
 *  06:05           SPIFREQ_DC   0x0
 *     04           SPIMODE_DC   0
 *     01           SPICOMP_DC   1
 *     00             SPIGO_DC   0
 * </pre>
 */
#define ROM_RADIOCNTL0_DC_ADDR   0x00320048
#define ROM_RADIOCNTL0_DC_OFFSET 0x00000048
#define ROM_RADIOCNTL0_DC_INDEX  0x00000012
#define ROM_RADIOCNTL0_DC_RESET  0x00002102

__INLINE uint32_t rom_radiocntl0_dc_get(void)
{
    return REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR);
}

__INLINE void rom_radiocntl0_dc_set(uint32_t value)
{
    REG_BT_WR(ROM_RADIOCNTL0_DC_ADDR, value);
}

// field definitions
#define ROM_SYNC_PULSE_MODE_DC_BIT    ((uint32_t)0x02000000)
#define ROM_SYNC_PULSE_MODE_DC_POS    25
#define ROM_DPCORR_EN_DC_BIT          ((uint32_t)0x01000000)
#define ROM_DPCORR_EN_DC_POS          24
#define ROM_SKEW_DC_MASK              ((uint32_t)0x000F0000)
#define ROM_SKEW_DC_LSB               16
#define ROM_SKEW_DC_WIDTH             ((uint32_t)0x00000004)
#define ROM_RXLOWIF_DC_MASK           ((uint32_t)0x00003000)
#define ROM_RXLOWIF_DC_LSB            12
#define ROM_RXLOWIF_DC_WIDTH          ((uint32_t)0x00000002)
#define ROM_BIDIRCLK_DC_BIT           ((uint32_t)0x00000200)
#define ROM_BIDIRCLK_DC_POS           9
#define ROM_BIDIRDATA_DC_BIT          ((uint32_t)0x00000100)
#define ROM_BIDIRDATA_DC_POS          8
#define ROM_SPIPOL_DC_BIT             ((uint32_t)0x00000080)
#define ROM_SPIPOL_DC_POS             7
#define ROM_SPIFREQ_DC_MASK           ((uint32_t)0x00000060)
#define ROM_SPIFREQ_DC_LSB            5
#define ROM_SPIFREQ_DC_WIDTH          ((uint32_t)0x00000002)
#define ROM_SPIMODE_DC_BIT            ((uint32_t)0x00000010)
#define ROM_SPIMODE_DC_POS            4
#define ROM_SPICOMP_DC_BIT            ((uint32_t)0x00000002)
#define ROM_SPICOMP_DC_POS            1
#define ROM_SPIGO_DC_BIT              ((uint32_t)0x00000001)
#define ROM_SPIGO_DC_POS              0

#define ROM_SYNC_PULSE_MODE_DC_RST    0x0
#define ROM_DPCORR_EN_DC_RST          0x0
#define ROM_SKEW_DC_RST               0x0
#define ROM_RXLOWIF_DC_RST            0x2
#define ROM_BIDIRCLK_DC_RST           0x0
#define ROM_BIDIRDATA_DC_RST          0x1
#define ROM_SPIPOL_DC_RST             0x0
#define ROM_SPIFREQ_DC_RST            0x0
#define ROM_SPIMODE_DC_RST            0x0
#define ROM_SPICOMP_DC_RST            0x1
#define ROM_SPIGO_DC_RST              0x0

__INLINE void rom_radiocntl0_dc_pack(uint8_t syncpulsemodedc, uint8_t dpcorrendc, uint8_t skewdc, uint8_t rxlowifdc, uint8_t bidirclkdc, uint8_t bidirdatadc, uint8_t spipoldc, uint8_t spifreqdc, uint8_t spimodedc, uint8_t spigodc)
{
    ASSERT_ERR((((uint32_t)syncpulsemodedc << 25) & ~((uint32_t)0x02000000)) == 0);
    ASSERT_ERR((((uint32_t)dpcorrendc << 24) & ~((uint32_t)0x01000000)) == 0);
    ASSERT_ERR((((uint32_t)skewdc << 16) & ~((uint32_t)0x000F0000)) == 0);
    ASSERT_ERR((((uint32_t)rxlowifdc << 12) & ~((uint32_t)0x00003000)) == 0);
    ASSERT_ERR((((uint32_t)bidirclkdc << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)bidirdatadc << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)spipoldc << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)spifreqdc << 5) & ~((uint32_t)0x00000060)) == 0);
    ASSERT_ERR((((uint32_t)spimodedc << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)spigodc << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DC_ADDR,  ((uint32_t)syncpulsemodedc << 25) | ((uint32_t)dpcorrendc << 24) | ((uint32_t)skewdc << 16) | ((uint32_t)rxlowifdc << 12) | ((uint32_t)bidirclkdc << 9) | ((uint32_t)bidirdatadc << 8) | ((uint32_t)spipoldc << 7) | ((uint32_t)spifreqdc << 5) | ((uint32_t)spimodedc << 4) | ((uint32_t)spigodc << 0));
}

__INLINE void rom_radiocntl0_dc_unpack(uint8_t* syncpulsemodedc, uint8_t* dpcorrendc, uint8_t* skewdc, uint8_t* rxlowifdc, uint8_t* bidirclkdc, uint8_t* bidirdatadc, uint8_t* spipoldc, uint8_t* spifreqdc, uint8_t* spimodedc, uint8_t* spicompdc, uint8_t* spigodc)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR);

    *syncpulsemodedc = (localVal & ((uint32_t)0x02000000)) >> 25;
    *dpcorrendc = (localVal & ((uint32_t)0x01000000)) >> 24;
    *skewdc = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *rxlowifdc = (localVal & ((uint32_t)0x00003000)) >> 12;
    *bidirclkdc = (localVal & ((uint32_t)0x00000200)) >> 9;
    *bidirdatadc = (localVal & ((uint32_t)0x00000100)) >> 8;
    *spipoldc = (localVal & ((uint32_t)0x00000080)) >> 7;
    *spifreqdc = (localVal & ((uint32_t)0x00000060)) >> 5;
    *spimodedc = (localVal & ((uint32_t)0x00000010)) >> 4;
    *spicompdc = (localVal & ((uint32_t)0x00000002)) >> 1;
    *spigodc = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t rom_sync_pulse_mode_dc_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR);
    return ((localVal & ((uint32_t)0x02000000)) >> 25);
}

__INLINE void rom_sync_pulse_mode_dc_setf(uint8_t syncpulsemodedc)
{
    ASSERT_ERR((((uint32_t)syncpulsemodedc << 25) & ~((uint32_t)0x02000000)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DC_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR) & ~((uint32_t)0x02000000)) | ((uint32_t)syncpulsemodedc << 25));
}

__INLINE uint8_t rom_dpcorr_en_dc_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR);
    return ((localVal & ((uint32_t)0x01000000)) >> 24);
}

__INLINE void rom_dpcorr_en_dc_setf(uint8_t dpcorrendc)
{
    ASSERT_ERR((((uint32_t)dpcorrendc << 24) & ~((uint32_t)0x01000000)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DC_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR) & ~((uint32_t)0x01000000)) | ((uint32_t)dpcorrendc << 24));
}

__INLINE uint8_t rom_skew_dc_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void rom_skew_dc_setf(uint8_t skewdc)
{
    ASSERT_ERR((((uint32_t)skewdc << 16) & ~((uint32_t)0x000F0000)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DC_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)skewdc << 16));
}

__INLINE uint8_t rom_rxlowif_dc_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR);
    return ((localVal & ((uint32_t)0x00003000)) >> 12);
}

__INLINE void rom_rxlowif_dc_setf(uint8_t rxlowifdc)
{
    ASSERT_ERR((((uint32_t)rxlowifdc << 12) & ~((uint32_t)0x00003000)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DC_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR) & ~((uint32_t)0x00003000)) | ((uint32_t)rxlowifdc << 12));
}

__INLINE uint8_t rom_bidirclk_dc_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void rom_bidirclk_dc_setf(uint8_t bidirclkdc)
{
    ASSERT_ERR((((uint32_t)bidirclkdc << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DC_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)bidirclkdc << 9));
}

__INLINE uint8_t rom_bidirdata_dc_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void rom_bidirdata_dc_setf(uint8_t bidirdatadc)
{
    ASSERT_ERR((((uint32_t)bidirdatadc << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DC_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)bidirdatadc << 8));
}

__INLINE uint8_t rom_spipol_dc_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void rom_spipol_dc_setf(uint8_t spipoldc)
{
    ASSERT_ERR((((uint32_t)spipoldc << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DC_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)spipoldc << 7));
}

__INLINE uint8_t rom_spifreq_dc_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR);
    return ((localVal & ((uint32_t)0x00000060)) >> 5);
}

__INLINE void rom_spifreq_dc_setf(uint8_t spifreqdc)
{
    ASSERT_ERR((((uint32_t)spifreqdc << 5) & ~((uint32_t)0x00000060)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DC_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR) & ~((uint32_t)0x00000060)) | ((uint32_t)spifreqdc << 5));
}

__INLINE uint8_t rom_spimode_dc_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void rom_spimode_dc_setf(uint8_t spimodedc)
{
    ASSERT_ERR((((uint32_t)spimodedc << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DC_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)spimodedc << 4));
}

__INLINE uint8_t rom_spicomp_dc_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t rom_spigo_dc_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void rom_spigo_dc_setf(uint8_t spigodc)
{
    ASSERT_ERR((((uint32_t)spigodc << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DC_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DC_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)spigodc << 0));
}

/**
 * @brief RADIOCNTL1_DC register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  20:16            XRFSEL_DC   0x0
 *  15:00            SPIPTR_DC   0x0
 * </pre>
 */
#define ROM_RADIOCNTL1_DC_ADDR   0x0032004C
#define ROM_RADIOCNTL1_DC_OFFSET 0x0000004C
#define ROM_RADIOCNTL1_DC_INDEX  0x00000013
#define ROM_RADIOCNTL1_DC_RESET  0x00000000

__INLINE uint32_t rom_radiocntl1_dc_get(void)
{
    return REG_BT_RD(ROM_RADIOCNTL1_DC_ADDR);
}

__INLINE void rom_radiocntl1_dc_set(uint32_t value)
{
    REG_BT_WR(ROM_RADIOCNTL1_DC_ADDR, value);
}

// field definitions
#define ROM_XRFSEL_DC_MASK   ((uint32_t)0x001F0000)
#define ROM_XRFSEL_DC_LSB    16
#define ROM_XRFSEL_DC_WIDTH  ((uint32_t)0x00000005)
#define ROM_SPIPTR_DC_MASK   ((uint32_t)0x0000FFFF)
#define ROM_SPIPTR_DC_LSB    0
#define ROM_SPIPTR_DC_WIDTH  ((uint32_t)0x00000010)

#define ROM_XRFSEL_DC_RST    0x0
#define ROM_SPIPTR_DC_RST    0x0

__INLINE void rom_radiocntl1_dc_pack(uint8_t xrfseldc, uint16_t spiptrdc)
{
    ASSERT_ERR((((uint32_t)xrfseldc << 16) & ~((uint32_t)0x001F0000)) == 0);
    ASSERT_ERR((((uint32_t)spiptrdc << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(ROM_RADIOCNTL1_DC_ADDR,  ((uint32_t)xrfseldc << 16) | ((uint32_t)spiptrdc << 0));
}

__INLINE void rom_radiocntl1_dc_unpack(uint8_t* xrfseldc, uint16_t* spiptrdc)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL1_DC_ADDR);

    *xrfseldc = (localVal & ((uint32_t)0x001F0000)) >> 16;
    *spiptrdc = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint8_t rom_xrfsel_dc_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL1_DC_ADDR);
    return ((localVal & ((uint32_t)0x001F0000)) >> 16);
}

__INLINE void rom_xrfsel_dc_setf(uint8_t xrfseldc)
{
    ASSERT_ERR((((uint32_t)xrfseldc << 16) & ~((uint32_t)0x001F0000)) == 0);
    REG_BT_WR(ROM_RADIOCNTL1_DC_ADDR, (REG_BT_RD(ROM_RADIOCNTL1_DC_ADDR) & ~((uint32_t)0x001F0000)) | ((uint32_t)xrfseldc << 16));
}

__INLINE uint16_t rom_spiptr_dc_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL1_DC_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void rom_spiptr_dc_setf(uint16_t spiptrdc)
{
    ASSERT_ERR((((uint32_t)spiptrdc << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(ROM_RADIOCNTL1_DC_ADDR, (REG_BT_RD(ROM_RADIOCNTL1_DC_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)spiptrdc << 0));
}

/**
 * @brief RADIOPWRUPDN_DC register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16         RXPWRUPCT_DC   0xD2
 *  11:08         TXPWRDNCT_DC   0x3
 *  07:00         TXPWRUPCT_DC   0xD2
 * </pre>
 */
#define ROM_RADIOPWRUPDN_DC_ADDR   0x003200BC
#define ROM_RADIOPWRUPDN_DC_OFFSET 0x000000BC
#define ROM_RADIOPWRUPDN_DC_INDEX  0x0000002F
#define ROM_RADIOPWRUPDN_DC_RESET  0x00D203D2

__INLINE uint32_t rom_radiopwrupdn_dc_get(void)
{
    return REG_BT_RD(ROM_RADIOPWRUPDN_DC_ADDR);
}

__INLINE void rom_radiopwrupdn_dc_set(uint32_t value)
{
    REG_BT_WR(ROM_RADIOPWRUPDN_DC_ADDR, value);
}

// field definitions
#define ROM_RXPWRUPCT_DC_MASK   ((uint32_t)0x00FF0000)
#define ROM_RXPWRUPCT_DC_LSB    16
#define ROM_RXPWRUPCT_DC_WIDTH  ((uint32_t)0x00000008)
#define ROM_TXPWRDNCT_DC_MASK   ((uint32_t)0x00000F00)
#define ROM_TXPWRDNCT_DC_LSB    8
#define ROM_TXPWRDNCT_DC_WIDTH  ((uint32_t)0x00000004)
#define ROM_TXPWRUPCT_DC_MASK   ((uint32_t)0x000000FF)
#define ROM_TXPWRUPCT_DC_LSB    0
#define ROM_TXPWRUPCT_DC_WIDTH  ((uint32_t)0x00000008)

#define ROM_RXPWRUPCT_DC_RST    0xD2
#define ROM_TXPWRDNCT_DC_RST    0x3
#define ROM_TXPWRUPCT_DC_RST    0xD2

__INLINE void rom_radiopwrupdn_dc_pack(uint8_t rxpwrupctdc, uint8_t txpwrdnctdc, uint8_t txpwrupctdc)
{
    ASSERT_ERR((((uint32_t)rxpwrupctdc << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)txpwrdnctdc << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)txpwrupctdc << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BT_WR(ROM_RADIOPWRUPDN_DC_ADDR,  ((uint32_t)rxpwrupctdc << 16) | ((uint32_t)txpwrdnctdc << 8) | ((uint32_t)txpwrupctdc << 0));
}

__INLINE void rom_radiopwrupdn_dc_unpack(uint8_t* rxpwrupctdc, uint8_t* txpwrdnctdc, uint8_t* txpwrupctdc)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOPWRUPDN_DC_ADDR);

    *rxpwrupctdc = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *txpwrdnctdc = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *txpwrupctdc = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t rom_rxpwrupct_dc_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOPWRUPDN_DC_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void rom_rxpwrupct_dc_setf(uint8_t rxpwrupctdc)
{
    ASSERT_ERR((((uint32_t)rxpwrupctdc << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BT_WR(ROM_RADIOPWRUPDN_DC_ADDR, (REG_BT_RD(ROM_RADIOPWRUPDN_DC_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)rxpwrupctdc << 16));
}

__INLINE uint8_t rom_txpwrdnct_dc_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOPWRUPDN_DC_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void rom_txpwrdnct_dc_setf(uint8_t txpwrdnctdc)
{
    ASSERT_ERR((((uint32_t)txpwrdnctdc << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_BT_WR(ROM_RADIOPWRUPDN_DC_ADDR, (REG_BT_RD(ROM_RADIOPWRUPDN_DC_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)txpwrdnctdc << 8));
}

__INLINE uint8_t rom_txpwrupct_dc_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOPWRUPDN_DC_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void rom_txpwrupct_dc_setf(uint8_t txpwrupctdc)
{
    ASSERT_ERR((((uint32_t)txpwrupctdc << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BT_WR(ROM_RADIOPWRUPDN_DC_ADDR, (REG_BT_RD(ROM_RADIOPWRUPDN_DC_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)txpwrupctdc << 0));
}

/**
 * @brief RADIOCNTL0_DM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:26            VGACNT_DM   0x0
 *  25:24            LNACNT_DM   0x0
 *     23     AGCFORCE_MODE_DM   0
 *     22         DPCORR_EN_DM   0
 *     21     SPISWACCDBGEN_DM   0
 *  13:12           RXLOWIF_DM   0x2
 *     09          BIDIRCLK_DM   0
 *     08         BIDIRDATA_DM   1
 *     07            SPIPOL_DM   0
 *  06:05           SPIFREQ_DM   0x0
 *     04           SPIMODE_DM   0
 *     01           SPICOMP_DM   1
 *     00             SPIGO_DM   0
 * </pre>
 */
#define ROM_RADIOCNTL0_DM_ADDR   0x00320070
#define ROM_RADIOCNTL0_DM_OFFSET 0x00000070
#define ROM_RADIOCNTL0_DM_INDEX  0x0000001C
#define ROM_RADIOCNTL0_DM_RESET  0x00002102

__INLINE uint32_t rom_radiocntl0_dm_get(void)
{
    return REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR);
}

__INLINE void rom_radiocntl0_dm_set(uint32_t value)
{
    REG_BT_WR(ROM_RADIOCNTL0_DM_ADDR, value);
}

// field definitions
#define ROM_VGACNT_DM_MASK          ((uint32_t)0xFC000000)
#define ROM_VGACNT_DM_LSB           26
#define ROM_VGACNT_DM_WIDTH         ((uint32_t)0x00000006)
#define ROM_LNACNT_DM_MASK          ((uint32_t)0x03000000)
#define ROM_LNACNT_DM_LSB           24
#define ROM_LNACNT_DM_WIDTH         ((uint32_t)0x00000002)
#define ROM_AGCFORCE_MODE_DM_BIT    ((uint32_t)0x00800000)
#define ROM_AGCFORCE_MODE_DM_POS    23
#define ROM_DPCORR_EN_DM_BIT        ((uint32_t)0x00400000)
#define ROM_DPCORR_EN_DM_POS        22
#define ROM_SPISWACCDBGEN_DM_BIT    ((uint32_t)0x00200000)
#define ROM_SPISWACCDBGEN_DM_POS    21
#define ROM_RXLOWIF_DM_MASK         ((uint32_t)0x00003000)
#define ROM_RXLOWIF_DM_LSB          12
#define ROM_RXLOWIF_DM_WIDTH        ((uint32_t)0x00000002)
#define ROM_BIDIRCLK_DM_BIT         ((uint32_t)0x00000200)
#define ROM_BIDIRCLK_DM_POS         9
#define ROM_BIDIRDATA_DM_BIT        ((uint32_t)0x00000100)
#define ROM_BIDIRDATA_DM_POS        8
#define ROM_SPIPOL_DM_BIT           ((uint32_t)0x00000080)
#define ROM_SPIPOL_DM_POS           7
#define ROM_SPIFREQ_DM_MASK         ((uint32_t)0x00000060)
#define ROM_SPIFREQ_DM_LSB          5
#define ROM_SPIFREQ_DM_WIDTH        ((uint32_t)0x00000002)
#define ROM_SPIMODE_DM_BIT          ((uint32_t)0x00000010)
#define ROM_SPIMODE_DM_POS          4
#define ROM_SPICOMP_DM_BIT          ((uint32_t)0x00000002)
#define ROM_SPICOMP_DM_POS          1
#define ROM_SPIGO_DM_BIT            ((uint32_t)0x00000001)
#define ROM_SPIGO_DM_POS            0

#define ROM_VGACNT_DM_RST           0x0
#define ROM_LNACNT_DM_RST           0x0
#define ROM_AGCFORCE_MODE_DM_RST    0x0
#define ROM_DPCORR_EN_DM_RST        0x0
#define ROM_SPISWACCDBGEN_DM_RST    0x0
#define ROM_RXLOWIF_DM_RST          0x2
#define ROM_BIDIRCLK_DM_RST         0x0
#define ROM_BIDIRDATA_DM_RST        0x1
#define ROM_SPIPOL_DM_RST           0x0
#define ROM_SPIFREQ_DM_RST          0x0
#define ROM_SPIMODE_DM_RST          0x0
#define ROM_SPICOMP_DM_RST          0x1
#define ROM_SPIGO_DM_RST            0x0

__INLINE void rom_radiocntl0_dm_pack(uint8_t vgacntdm, uint8_t lnacntdm, uint8_t agcforcemodedm, uint8_t dpcorrendm, uint8_t spiswaccdbgendm, uint8_t rxlowifdm, uint8_t bidirclkdm, uint8_t bidirdatadm, uint8_t spipoldm, uint8_t spifreqdm, uint8_t spimodedm, uint8_t spigodm)
{
    ASSERT_ERR((((uint32_t)vgacntdm << 26) & ~((uint32_t)0xFC000000)) == 0);
    ASSERT_ERR((((uint32_t)lnacntdm << 24) & ~((uint32_t)0x03000000)) == 0);
    ASSERT_ERR((((uint32_t)agcforcemodedm << 23) & ~((uint32_t)0x00800000)) == 0);
    ASSERT_ERR((((uint32_t)dpcorrendm << 22) & ~((uint32_t)0x00400000)) == 0);
    ASSERT_ERR((((uint32_t)spiswaccdbgendm << 21) & ~((uint32_t)0x00200000)) == 0);
    ASSERT_ERR((((uint32_t)rxlowifdm << 12) & ~((uint32_t)0x00003000)) == 0);
    ASSERT_ERR((((uint32_t)bidirclkdm << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)bidirdatadm << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)spipoldm << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)spifreqdm << 5) & ~((uint32_t)0x00000060)) == 0);
    ASSERT_ERR((((uint32_t)spimodedm << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)spigodm << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DM_ADDR,  ((uint32_t)vgacntdm << 26) | ((uint32_t)lnacntdm << 24) | ((uint32_t)agcforcemodedm << 23) | ((uint32_t)dpcorrendm << 22) | ((uint32_t)spiswaccdbgendm << 21) | ((uint32_t)rxlowifdm << 12) | ((uint32_t)bidirclkdm << 9) | ((uint32_t)bidirdatadm << 8) | ((uint32_t)spipoldm << 7) | ((uint32_t)spifreqdm << 5) | ((uint32_t)spimodedm << 4) | ((uint32_t)spigodm << 0));
}

__INLINE void rom_radiocntl0_dm_unpack(uint8_t* vgacntdm, uint8_t* lnacntdm, uint8_t* agcforcemodedm, uint8_t* dpcorrendm, uint8_t* spiswaccdbgendm, uint8_t* rxlowifdm, uint8_t* bidirclkdm, uint8_t* bidirdatadm, uint8_t* spipoldm, uint8_t* spifreqdm, uint8_t* spimodedm, uint8_t* spicompdm, uint8_t* spigodm)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR);

    *vgacntdm = (localVal & ((uint32_t)0xFC000000)) >> 26;
    *lnacntdm = (localVal & ((uint32_t)0x03000000)) >> 24;
    *agcforcemodedm = (localVal & ((uint32_t)0x00800000)) >> 23;
    *dpcorrendm = (localVal & ((uint32_t)0x00400000)) >> 22;
    *spiswaccdbgendm = (localVal & ((uint32_t)0x00200000)) >> 21;
    *rxlowifdm = (localVal & ((uint32_t)0x00003000)) >> 12;
    *bidirclkdm = (localVal & ((uint32_t)0x00000200)) >> 9;
    *bidirdatadm = (localVal & ((uint32_t)0x00000100)) >> 8;
    *spipoldm = (localVal & ((uint32_t)0x00000080)) >> 7;
    *spifreqdm = (localVal & ((uint32_t)0x00000060)) >> 5;
    *spimodedm = (localVal & ((uint32_t)0x00000010)) >> 4;
    *spicompdm = (localVal & ((uint32_t)0x00000002)) >> 1;
    *spigodm = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t rom_vgacnt_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR);
    return ((localVal & ((uint32_t)0xFC000000)) >> 26);
}

__INLINE void rom_vgacnt_dm_setf(uint8_t vgacntdm)
{
    ASSERT_ERR((((uint32_t)vgacntdm << 26) & ~((uint32_t)0xFC000000)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DM_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR) & ~((uint32_t)0xFC000000)) | ((uint32_t)vgacntdm << 26));
}

__INLINE uint8_t rom_lnacnt_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR);
    return ((localVal & ((uint32_t)0x03000000)) >> 24);
}

__INLINE void rom_lnacnt_dm_setf(uint8_t lnacntdm)
{
    ASSERT_ERR((((uint32_t)lnacntdm << 24) & ~((uint32_t)0x03000000)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DM_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR) & ~((uint32_t)0x03000000)) | ((uint32_t)lnacntdm << 24));
}

__INLINE uint8_t rom_agcforce_mode_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE void rom_agcforce_mode_dm_setf(uint8_t agcforcemodedm)
{
    ASSERT_ERR((((uint32_t)agcforcemodedm << 23) & ~((uint32_t)0x00800000)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DM_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR) & ~((uint32_t)0x00800000)) | ((uint32_t)agcforcemodedm << 23));
}

__INLINE uint8_t rom_dpcorr_en_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR);
    return ((localVal & ((uint32_t)0x00400000)) >> 22);
}

__INLINE void rom_dpcorr_en_dm_setf(uint8_t dpcorrendm)
{
    ASSERT_ERR((((uint32_t)dpcorrendm << 22) & ~((uint32_t)0x00400000)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DM_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR) & ~((uint32_t)0x00400000)) | ((uint32_t)dpcorrendm << 22));
}

__INLINE uint8_t rom_spiswaccdbgen_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE void rom_spiswaccdbgen_dm_setf(uint8_t spiswaccdbgendm)
{
    ASSERT_ERR((((uint32_t)spiswaccdbgendm << 21) & ~((uint32_t)0x00200000)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DM_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR) & ~((uint32_t)0x00200000)) | ((uint32_t)spiswaccdbgendm << 21));
}

__INLINE uint8_t rom_rxlowif_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR);
    return ((localVal & ((uint32_t)0x00003000)) >> 12);
}

__INLINE void rom_rxlowif_dm_setf(uint8_t rxlowifdm)
{
    ASSERT_ERR((((uint32_t)rxlowifdm << 12) & ~((uint32_t)0x00003000)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DM_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR) & ~((uint32_t)0x00003000)) | ((uint32_t)rxlowifdm << 12));
}

__INLINE uint8_t rom_bidirclk_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void rom_bidirclk_dm_setf(uint8_t bidirclkdm)
{
    ASSERT_ERR((((uint32_t)bidirclkdm << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DM_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)bidirclkdm << 9));
}

__INLINE uint8_t rom_bidirdata_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void rom_bidirdata_dm_setf(uint8_t bidirdatadm)
{
    ASSERT_ERR((((uint32_t)bidirdatadm << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DM_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)bidirdatadm << 8));
}

__INLINE uint8_t rom_spipol_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void rom_spipol_dm_setf(uint8_t spipoldm)
{
    ASSERT_ERR((((uint32_t)spipoldm << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DM_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)spipoldm << 7));
}

__INLINE uint8_t rom_spifreq_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR);
    return ((localVal & ((uint32_t)0x00000060)) >> 5);
}

__INLINE void rom_spifreq_dm_setf(uint8_t spifreqdm)
{
    ASSERT_ERR((((uint32_t)spifreqdm << 5) & ~((uint32_t)0x00000060)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DM_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR) & ~((uint32_t)0x00000060)) | ((uint32_t)spifreqdm << 5));
}

__INLINE uint8_t rom_spimode_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void rom_spimode_dm_setf(uint8_t spimodedm)
{
    ASSERT_ERR((((uint32_t)spimodedm << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DM_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)spimodedm << 4));
}

__INLINE uint8_t rom_spicomp_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t rom_spigo_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void rom_spigo_dm_setf(uint8_t spigodm)
{
    ASSERT_ERR((((uint32_t)spigodm << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BT_WR(ROM_RADIOCNTL0_DM_ADDR, (REG_BT_RD(ROM_RADIOCNTL0_DM_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)spigodm << 0));
}

/**
 * @brief RADIOCNTL1_DM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     28         RAMP_TYPE_DM   0
 *  27:24          RAMPSTEP_DM   0x0
 *  20:16            XRFSEL_DM   0x0
 *  15:00            SPIPTR_DM   0x0
 * </pre>
 */
#define ROM_RADIOCNTL1_DM_ADDR   0x00320074
#define ROM_RADIOCNTL1_DM_OFFSET 0x00000074
#define ROM_RADIOCNTL1_DM_INDEX  0x0000001D
#define ROM_RADIOCNTL1_DM_RESET  0x00000000

__INLINE uint32_t rom_radiocntl1_dm_get(void)
{
    return REG_BT_RD(ROM_RADIOCNTL1_DM_ADDR);
}

__INLINE void rom_radiocntl1_dm_set(uint32_t value)
{
    REG_BT_WR(ROM_RADIOCNTL1_DM_ADDR, value);
}

// field definitions
#define ROM_RAMP_TYPE_DM_BIT    ((uint32_t)0x10000000)
#define ROM_RAMP_TYPE_DM_POS    28
#define ROM_RAMPSTEP_DM_MASK    ((uint32_t)0x0F000000)
#define ROM_RAMPSTEP_DM_LSB     24
#define ROM_RAMPSTEP_DM_WIDTH   ((uint32_t)0x00000004)
#define ROM_XRFSEL_DM_MASK      ((uint32_t)0x001F0000)
#define ROM_XRFSEL_DM_LSB       16
#define ROM_XRFSEL_DM_WIDTH     ((uint32_t)0x00000005)
#define ROM_SPIPTR_DM_MASK      ((uint32_t)0x0000FFFF)
#define ROM_SPIPTR_DM_LSB       0
#define ROM_SPIPTR_DM_WIDTH     ((uint32_t)0x00000010)

#define ROM_RAMP_TYPE_DM_RST    0x0
#define ROM_RAMPSTEP_DM_RST     0x0
#define ROM_XRFSEL_DM_RST       0x0
#define ROM_SPIPTR_DM_RST       0x0

__INLINE void rom_radiocntl1_dm_pack(uint8_t ramptypedm, uint8_t rampstepdm, uint8_t xrfseldm, uint16_t spiptrdm)
{
    ASSERT_ERR((((uint32_t)ramptypedm << 28) & ~((uint32_t)0x10000000)) == 0);
    ASSERT_ERR((((uint32_t)rampstepdm << 24) & ~((uint32_t)0x0F000000)) == 0);
    ASSERT_ERR((((uint32_t)xrfseldm << 16) & ~((uint32_t)0x001F0000)) == 0);
    ASSERT_ERR((((uint32_t)spiptrdm << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(ROM_RADIOCNTL1_DM_ADDR,  ((uint32_t)ramptypedm << 28) | ((uint32_t)rampstepdm << 24) | ((uint32_t)xrfseldm << 16) | ((uint32_t)spiptrdm << 0));
}

__INLINE void rom_radiocntl1_dm_unpack(uint8_t* ramptypedm, uint8_t* rampstepdm, uint8_t* xrfseldm, uint16_t* spiptrdm)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL1_DM_ADDR);

    *ramptypedm = (localVal & ((uint32_t)0x10000000)) >> 28;
    *rampstepdm = (localVal & ((uint32_t)0x0F000000)) >> 24;
    *xrfseldm = (localVal & ((uint32_t)0x001F0000)) >> 16;
    *spiptrdm = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint8_t rom_ramp_type_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL1_DM_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void rom_ramp_type_dm_setf(uint8_t ramptypedm)
{
    ASSERT_ERR((((uint32_t)ramptypedm << 28) & ~((uint32_t)0x10000000)) == 0);
    REG_BT_WR(ROM_RADIOCNTL1_DM_ADDR, (REG_BT_RD(ROM_RADIOCNTL1_DM_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)ramptypedm << 28));
}

__INLINE uint8_t rom_rampstep_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL1_DM_ADDR);
    return ((localVal & ((uint32_t)0x0F000000)) >> 24);
}

__INLINE void rom_rampstep_dm_setf(uint8_t rampstepdm)
{
    ASSERT_ERR((((uint32_t)rampstepdm << 24) & ~((uint32_t)0x0F000000)) == 0);
    REG_BT_WR(ROM_RADIOCNTL1_DM_ADDR, (REG_BT_RD(ROM_RADIOCNTL1_DM_ADDR) & ~((uint32_t)0x0F000000)) | ((uint32_t)rampstepdm << 24));
}

__INLINE uint8_t rom_xrfsel_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL1_DM_ADDR);
    return ((localVal & ((uint32_t)0x001F0000)) >> 16);
}

__INLINE void rom_xrfsel_dm_setf(uint8_t xrfseldm)
{
    ASSERT_ERR((((uint32_t)xrfseldm << 16) & ~((uint32_t)0x001F0000)) == 0);
    REG_BT_WR(ROM_RADIOCNTL1_DM_ADDR, (REG_BT_RD(ROM_RADIOCNTL1_DM_ADDR) & ~((uint32_t)0x001F0000)) | ((uint32_t)xrfseldm << 16));
}

__INLINE uint16_t rom_spiptr_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOCNTL1_DM_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void rom_spiptr_dm_setf(uint16_t spiptrdm)
{
    ASSERT_ERR((((uint32_t)spiptrdm << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_BT_WR(ROM_RADIOCNTL1_DM_ADDR, (REG_BT_RD(ROM_RADIOCNTL1_DM_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)spiptrdm << 0));
}

/**
 * @brief RADIOPWRUPDN_DM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:28       RTRIP_DELAY_DM   0x0
 *  23:16           RXPWRUP_DM   0xD2
 *  11:08           TXPWRDN_DM   0x3
 *  07:00           TXPWRUP_DM   0xD2
 * </pre>
 */
#define ROM_RADIOPWRUPDN_DM_ADDR   0x00320080
#define ROM_RADIOPWRUPDN_DM_OFFSET 0x00000080
#define ROM_RADIOPWRUPDN_DM_INDEX  0x00000020
#define ROM_RADIOPWRUPDN_DM_RESET  0x00D203D2

__INLINE uint32_t rom_radiopwrupdn_dm_get(void)
{
    return REG_BT_RD(ROM_RADIOPWRUPDN_DM_ADDR);
}

__INLINE void rom_radiopwrupdn_dm_set(uint32_t value)
{
    REG_BT_WR(ROM_RADIOPWRUPDN_DM_ADDR, value);
}

// field definitions
#define ROM_RTRIP_DELAY_DM_MASK   ((uint32_t)0xF0000000)
#define ROM_RTRIP_DELAY_DM_LSB    28
#define ROM_RTRIP_DELAY_DM_WIDTH  ((uint32_t)0x00000004)
#define ROM_RXPWRUP_DM_MASK       ((uint32_t)0x00FF0000)
#define ROM_RXPWRUP_DM_LSB        16
#define ROM_RXPWRUP_DM_WIDTH      ((uint32_t)0x00000008)
#define ROM_TXPWRDN_DM_MASK       ((uint32_t)0x00000F00)
#define ROM_TXPWRDN_DM_LSB        8
#define ROM_TXPWRDN_DM_WIDTH      ((uint32_t)0x00000004)
#define ROM_TXPWRUP_DM_MASK       ((uint32_t)0x000000FF)
#define ROM_TXPWRUP_DM_LSB        0
#define ROM_TXPWRUP_DM_WIDTH      ((uint32_t)0x00000008)

#define ROM_RTRIP_DELAY_DM_RST    0x0
#define ROM_RXPWRUP_DM_RST        0xD2
#define ROM_TXPWRDN_DM_RST        0x3
#define ROM_TXPWRUP_DM_RST        0xD2

__INLINE void rom_radiopwrupdn_dm_pack(uint8_t rtripdelaydm, uint8_t rxpwrupdm, uint8_t txpwrdndm, uint8_t txpwrupdm)
{
    ASSERT_ERR((((uint32_t)rtripdelaydm << 28) & ~((uint32_t)0xF0000000)) == 0);
    ASSERT_ERR((((uint32_t)rxpwrupdm << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)txpwrdndm << 8) & ~((uint32_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint32_t)txpwrupdm << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BT_WR(ROM_RADIOPWRUPDN_DM_ADDR,  ((uint32_t)rtripdelaydm << 28) | ((uint32_t)rxpwrupdm << 16) | ((uint32_t)txpwrdndm << 8) | ((uint32_t)txpwrupdm << 0));
}

__INLINE void rom_radiopwrupdn_dm_unpack(uint8_t* rtripdelaydm, uint8_t* rxpwrupdm, uint8_t* txpwrdndm, uint8_t* txpwrupdm)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOPWRUPDN_DM_ADDR);

    *rtripdelaydm = (localVal & ((uint32_t)0xF0000000)) >> 28;
    *rxpwrupdm = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *txpwrdndm = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *txpwrupdm = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t rom_rtrip_delay_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOPWRUPDN_DM_ADDR);
    return ((localVal & ((uint32_t)0xF0000000)) >> 28);
}

__INLINE void rom_rtrip_delay_dm_setf(uint8_t rtripdelaydm)
{
    ASSERT_ERR((((uint32_t)rtripdelaydm << 28) & ~((uint32_t)0xF0000000)) == 0);
    REG_BT_WR(ROM_RADIOPWRUPDN_DM_ADDR, (REG_BT_RD(ROM_RADIOPWRUPDN_DM_ADDR) & ~((uint32_t)0xF0000000)) | ((uint32_t)rtripdelaydm << 28));
}

__INLINE uint8_t rom_rxpwrup_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOPWRUPDN_DM_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void rom_rxpwrup_dm_setf(uint8_t rxpwrupdm)
{
    ASSERT_ERR((((uint32_t)rxpwrupdm << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_BT_WR(ROM_RADIOPWRUPDN_DM_ADDR, (REG_BT_RD(ROM_RADIOPWRUPDN_DM_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)rxpwrupdm << 16));
}

__INLINE uint8_t rom_txpwrdn_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOPWRUPDN_DM_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void rom_txpwrdn_dm_setf(uint8_t txpwrdndm)
{
    ASSERT_ERR((((uint32_t)txpwrdndm << 8) & ~((uint32_t)0x00000F00)) == 0);
    REG_BT_WR(ROM_RADIOPWRUPDN_DM_ADDR, (REG_BT_RD(ROM_RADIOPWRUPDN_DM_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)txpwrdndm << 8));
}

__INLINE uint8_t rom_txpwrup_dm_getf(void)
{
    uint32_t localVal = REG_BT_RD(ROM_RADIOPWRUPDN_DM_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void rom_txpwrup_dm_setf(uint8_t txpwrupdm)
{
    ASSERT_ERR((((uint32_t)txpwrupdm << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BT_WR(ROM_RADIOPWRUPDN_DM_ADDR, (REG_BT_RD(ROM_RADIOPWRUPDN_DM_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)txpwrupdm << 0));
}


#endif // _REG_ROMCORE_H_

