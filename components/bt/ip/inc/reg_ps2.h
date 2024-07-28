#ifndef _REG_PS2_H_
#define _REG_PS2_H_

#include <stdint.h>
#include "_reg_ps2.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_PS2_COUNT 8

#define REG_PS2_DECODING_MASK 0x0000001F

/**
 * @brief PS2CONF register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  27:16         RXTIMEOUTCNT   0x0
 *  15:00                FRACT   0x0
 * </pre>
 */
#define PS2_PS2CONF_ADDR   0x00403000
#define PS2_PS2CONF_OFFSET 0x00000000
#define PS2_PS2CONF_INDEX  0x00000000
#define PS2_PS2CONF_RESET  0x00000000

__INLINE uint32_t ps2_ps2conf_get(void)
{
    return REG_PL_RD(PS2_PS2CONF_ADDR);
}

__INLINE void ps2_ps2conf_set(uint32_t value)
{
    REG_PL_WR(PS2_PS2CONF_ADDR, value);
}

// field definitions
#define PS2_RXTIMEOUTCNT_MASK   ((uint32_t)0x0FFF0000)
#define PS2_RXTIMEOUTCNT_LSB    16
#define PS2_RXTIMEOUTCNT_WIDTH  ((uint32_t)0x0000000C)
#define PS2_FRACT_MASK          ((uint32_t)0x0000FFFF)
#define PS2_FRACT_LSB           0
#define PS2_FRACT_WIDTH         ((uint32_t)0x00000010)

#define PS2_RXTIMEOUTCNT_RST    0x0
#define PS2_FRACT_RST           0x0

__INLINE void ps2_ps2conf_pack(uint16_t rxtimeoutcnt, uint16_t fract)
{
    ASSERT_ERR((((uint32_t)rxtimeoutcnt << 16) & ~((uint32_t)0x0FFF0000)) == 0);
    ASSERT_ERR((((uint32_t)fract << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_PL_WR(PS2_PS2CONF_ADDR,  ((uint32_t)rxtimeoutcnt << 16) | ((uint32_t)fract << 0));
}

__INLINE void ps2_ps2conf_unpack(uint16_t* rxtimeoutcnt, uint16_t* fract)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2CONF_ADDR);

    *rxtimeoutcnt = (localVal & ((uint32_t)0x0FFF0000)) >> 16;
    *fract = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t ps2_ps2conf_rxtimeoutcnt_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2CONF_ADDR);
    return ((localVal & ((uint32_t)0x0FFF0000)) >> 16);
}

__INLINE void ps2_ps2conf_rxtimeoutcnt_setf(uint16_t rxtimeoutcnt)
{
    ASSERT_ERR((((uint32_t)rxtimeoutcnt << 16) & ~((uint32_t)0x0FFF0000)) == 0);
    REG_PL_WR(PS2_PS2CONF_ADDR, (REG_PL_RD(PS2_PS2CONF_ADDR) & ~((uint32_t)0x0FFF0000)) | ((uint32_t)rxtimeoutcnt << 16));
}

__INLINE uint16_t ps2_ps2conf_fract_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2CONF_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ps2_ps2conf_fract_setf(uint16_t fract)
{
    ASSERT_ERR((((uint32_t)fract << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_PL_WR(PS2_PS2CONF_ADDR, (REG_PL_RD(PS2_PS2CONF_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)fract << 0));
}

/**
 * @brief PS2STAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  06:04             RXNUMBER   0x0
 *     03            TXSTOPERR   0
 *     02             TXACKERR   0
 *     01          RXPARITYERR   0
 *     00            RXSTOPERR   0
 * </pre>
 */
#define PS2_PS2STAT_ADDR   0x00403004
#define PS2_PS2STAT_OFFSET 0x00000004
#define PS2_PS2STAT_INDEX  0x00000001
#define PS2_PS2STAT_RESET  0x00000000

__INLINE uint32_t ps2_ps2stat_get(void)
{
    return REG_PL_RD(PS2_PS2STAT_ADDR);
}

// field definitions
#define PS2_RXNUMBER_MASK      ((uint32_t)0x00000070)
#define PS2_RXNUMBER_LSB       4
#define PS2_RXNUMBER_WIDTH     ((uint32_t)0x00000003)
#define PS2_TXSTOPERR_BIT      ((uint32_t)0x00000008)
#define PS2_TXSTOPERR_POS      3
#define PS2_TXACKERR_BIT       ((uint32_t)0x00000004)
#define PS2_TXACKERR_POS       2
#define PS2_RXPARITYERR_BIT    ((uint32_t)0x00000002)
#define PS2_RXPARITYERR_POS    1
#define PS2_RXSTOPERR_BIT      ((uint32_t)0x00000001)
#define PS2_RXSTOPERR_POS      0

#define PS2_RXNUMBER_RST       0x0
#define PS2_TXSTOPERR_RST      0x0
#define PS2_TXACKERR_RST       0x0
#define PS2_RXPARITYERR_RST    0x0
#define PS2_RXSTOPERR_RST      0x0

__INLINE void ps2_ps2stat_unpack(uint8_t* rxnumber, uint8_t* txstoperr, uint8_t* txackerr, uint8_t* rxparityerr, uint8_t* rxstoperr)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2STAT_ADDR);

    *rxnumber = (localVal & ((uint32_t)0x00000070)) >> 4;
    *txstoperr = (localVal & ((uint32_t)0x00000008)) >> 3;
    *txackerr = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rxparityerr = (localVal & ((uint32_t)0x00000002)) >> 1;
    *rxstoperr = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ps2_ps2stat_rxnumber_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000070)) >> 4);
}

__INLINE uint8_t ps2_ps2stat_txstoperr_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t ps2_ps2stat_txackerr_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t ps2_ps2stat_rxparityerr_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t ps2_ps2stat_rxstoperr_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2STAT_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief PS2INTCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     02            ERRINTMSK   0
 *     01             RXINTMSK   0
 *     00             TXINTMSK   0
 * </pre>
 */
#define PS2_PS2INTCNTL_ADDR   0x00403008
#define PS2_PS2INTCNTL_OFFSET 0x00000008
#define PS2_PS2INTCNTL_INDEX  0x00000002
#define PS2_PS2INTCNTL_RESET  0x00000000

__INLINE uint32_t ps2_ps2intcntl_get(void)
{
    return REG_PL_RD(PS2_PS2INTCNTL_ADDR);
}

__INLINE void ps2_ps2intcntl_set(uint32_t value)
{
    REG_PL_WR(PS2_PS2INTCNTL_ADDR, value);
}

// field definitions
#define PS2_ERRINTMSK_BIT    ((uint32_t)0x00000004)
#define PS2_ERRINTMSK_POS    2
#define PS2_RXINTMSK_BIT     ((uint32_t)0x00000002)
#define PS2_RXINTMSK_POS     1
#define PS2_TXINTMSK_BIT     ((uint32_t)0x00000001)
#define PS2_TXINTMSK_POS     0

#define PS2_ERRINTMSK_RST    0x0
#define PS2_RXINTMSK_RST     0x0
#define PS2_TXINTMSK_RST     0x0

__INLINE void ps2_ps2intcntl_pack(uint8_t errintmsk, uint8_t rxintmsk, uint8_t txintmsk)
{
    ASSERT_ERR((((uint32_t)errintmsk << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)rxintmsk << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)txintmsk << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(PS2_PS2INTCNTL_ADDR,  ((uint32_t)errintmsk << 2) | ((uint32_t)rxintmsk << 1) | ((uint32_t)txintmsk << 0));
}

__INLINE void ps2_ps2intcntl_unpack(uint8_t* errintmsk, uint8_t* rxintmsk, uint8_t* txintmsk)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2INTCNTL_ADDR);

    *errintmsk = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rxintmsk = (localVal & ((uint32_t)0x00000002)) >> 1;
    *txintmsk = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ps2_ps2intcntl_errintmsk_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void ps2_ps2intcntl_errintmsk_setf(uint8_t errintmsk)
{
    ASSERT_ERR((((uint32_t)errintmsk << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_PL_WR(PS2_PS2INTCNTL_ADDR, (REG_PL_RD(PS2_PS2INTCNTL_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)errintmsk << 2));
}

__INLINE uint8_t ps2_ps2intcntl_rxintmsk_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void ps2_ps2intcntl_rxintmsk_setf(uint8_t rxintmsk)
{
    ASSERT_ERR((((uint32_t)rxintmsk << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_PL_WR(PS2_PS2INTCNTL_ADDR, (REG_PL_RD(PS2_PS2INTCNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)rxintmsk << 1));
}

__INLINE uint8_t ps2_ps2intcntl_txintmsk_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2INTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ps2_ps2intcntl_txintmsk_setf(uint8_t txintmsk)
{
    ASSERT_ERR((((uint32_t)txintmsk << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(PS2_PS2INTCNTL_ADDR, (REG_PL_RD(PS2_PS2INTCNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)txintmsk << 0));
}

/**
 * @brief PS2INTSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     02           ERRINTSTAT   0
 *     01            RXINTSTAT   0
 *     00            TXINTSTAT   0
 * </pre>
 */
#define PS2_PS2INTSTAT_ADDR   0x0040300C
#define PS2_PS2INTSTAT_OFFSET 0x0000000C
#define PS2_PS2INTSTAT_INDEX  0x00000003
#define PS2_PS2INTSTAT_RESET  0x00000000

__INLINE uint32_t ps2_ps2intstat_get(void)
{
    return REG_PL_RD(PS2_PS2INTSTAT_ADDR);
}

// field definitions
#define PS2_ERRINTSTAT_BIT    ((uint32_t)0x00000004)
#define PS2_ERRINTSTAT_POS    2
#define PS2_RXINTSTAT_BIT     ((uint32_t)0x00000002)
#define PS2_RXINTSTAT_POS     1
#define PS2_TXINTSTAT_BIT     ((uint32_t)0x00000001)
#define PS2_TXINTSTAT_POS     0

#define PS2_ERRINTSTAT_RST    0x0
#define PS2_RXINTSTAT_RST     0x0
#define PS2_TXINTSTAT_RST     0x0

__INLINE void ps2_ps2intstat_unpack(uint8_t* errintstat, uint8_t* rxintstat, uint8_t* txintstat)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2INTSTAT_ADDR);

    *errintstat = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rxintstat = (localVal & ((uint32_t)0x00000002)) >> 1;
    *txintstat = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ps2_ps2intstat_errintstat_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t ps2_ps2intstat_rxintstat_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t ps2_ps2intstat_txintstat_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2INTSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief PS2INTRAWSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     02        ERRINTRAWSTAT   0
 *     01         RXINTRAWSTAT   0
 *     00         TXINTRAWSTAT   0
 * </pre>
 */
#define PS2_PS2INTRAWSTAT_ADDR   0x00403010
#define PS2_PS2INTRAWSTAT_OFFSET 0x00000010
#define PS2_PS2INTRAWSTAT_INDEX  0x00000004
#define PS2_PS2INTRAWSTAT_RESET  0x00000000

__INLINE uint32_t ps2_ps2intrawstat_get(void)
{
    return REG_PL_RD(PS2_PS2INTRAWSTAT_ADDR);
}

// field definitions
#define PS2_ERRINTRAWSTAT_BIT    ((uint32_t)0x00000004)
#define PS2_ERRINTRAWSTAT_POS    2
#define PS2_RXINTRAWSTAT_BIT     ((uint32_t)0x00000002)
#define PS2_RXINTRAWSTAT_POS     1
#define PS2_TXINTRAWSTAT_BIT     ((uint32_t)0x00000001)
#define PS2_TXINTRAWSTAT_POS     0

#define PS2_ERRINTRAWSTAT_RST    0x0
#define PS2_RXINTRAWSTAT_RST     0x0
#define PS2_TXINTRAWSTAT_RST     0x0

__INLINE void ps2_ps2intrawstat_unpack(uint8_t* errintrawstat, uint8_t* rxintrawstat, uint8_t* txintrawstat)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2INTRAWSTAT_ADDR);

    *errintrawstat = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rxintrawstat = (localVal & ((uint32_t)0x00000002)) >> 1;
    *txintrawstat = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ps2_ps2intrawstat_errintrawstat_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t ps2_ps2intrawstat_rxintrawstat_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t ps2_ps2intrawstat_txintrawstat_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2INTRAWSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief PS2INTACK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     02            ERRINTACK   0
 *     01             RXINTACK   0
 *     00             TXINTACK   0
 * </pre>
 */
#define PS2_PS2INTACK_ADDR   0x00403014
#define PS2_PS2INTACK_OFFSET 0x00000014
#define PS2_PS2INTACK_INDEX  0x00000005
#define PS2_PS2INTACK_RESET  0x00000000

__INLINE uint32_t ps2_ps2intack_get(void)
{
    return REG_PL_RD(PS2_PS2INTACK_ADDR);
}

__INLINE void ps2_ps2intack_clear(uint32_t value)
{
    REG_PL_WR(PS2_PS2INTACK_ADDR, value);
}

// field definitions
#define PS2_ERRINTACK_BIT    ((uint32_t)0x00000004)
#define PS2_ERRINTACK_POS    2
#define PS2_RXINTACK_BIT     ((uint32_t)0x00000002)
#define PS2_RXINTACK_POS     1
#define PS2_TXINTACK_BIT     ((uint32_t)0x00000001)
#define PS2_TXINTACK_POS     0

#define PS2_ERRINTACK_RST    0x0
#define PS2_RXINTACK_RST     0x0
#define PS2_TXINTACK_RST     0x0

__INLINE void ps2_ps2intack_pack(uint8_t errintack, uint8_t rxintack, uint8_t txintack)
{
    ASSERT_ERR((((uint32_t)errintack << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)rxintack << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)txintack << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(PS2_PS2INTACK_ADDR,  ((uint32_t)errintack << 2) | ((uint32_t)rxintack << 1) | ((uint32_t)txintack << 0));
}

__INLINE void ps2_ps2intack_unpack(uint8_t* errintack, uint8_t* rxintack, uint8_t* txintack)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2INTACK_ADDR);

    *errintack = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rxintack = (localVal & ((uint32_t)0x00000002)) >> 1;
    *txintack = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ps2_ps2intack_errintack_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void ps2_ps2intack_errintack_clearf(uint8_t errintack)
{
    ASSERT_ERR((((uint32_t)errintack << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_PL_WR(PS2_PS2INTACK_ADDR, (uint32_t)errintack << 2);
}

__INLINE uint8_t ps2_ps2intack_rxintack_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void ps2_ps2intack_rxintack_clearf(uint8_t rxintack)
{
    ASSERT_ERR((((uint32_t)rxintack << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_PL_WR(PS2_PS2INTACK_ADDR, (uint32_t)rxintack << 1);
}

__INLINE uint8_t ps2_ps2intack_txintack_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2INTACK_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ps2_ps2intack_txintack_clearf(uint8_t txintack)
{
    ASSERT_ERR((((uint32_t)txintack << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(PS2_PS2INTACK_ADDR, (uint32_t)txintack << 0);
}

/**
 * @brief PS2TXDATA register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15                 SEND   0
 *  07:00               TXDATA   0x0
 * </pre>
 */
#define PS2_PS2TXDATA_ADDR   0x00403018
#define PS2_PS2TXDATA_OFFSET 0x00000018
#define PS2_PS2TXDATA_INDEX  0x00000006
#define PS2_PS2TXDATA_RESET  0x00000000

__INLINE void ps2_ps2txdata_set(uint32_t value)
{
    REG_PL_WR(PS2_PS2TXDATA_ADDR, value);
}

// field definitions
#define PS2_SEND_BIT      ((uint32_t)0x00008000)
#define PS2_SEND_POS      15
#define PS2_TXDATA_MASK   ((uint32_t)0x000000FF)
#define PS2_TXDATA_LSB    0
#define PS2_TXDATA_WIDTH  ((uint32_t)0x00000008)

#define PS2_SEND_RST      0x0
#define PS2_TXDATA_RST    0x0

__INLINE void ps2_ps2txdata_pack(uint8_t send, uint8_t txdata)
{
    ASSERT_ERR((((uint32_t)send << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)txdata << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(PS2_PS2TXDATA_ADDR,  ((uint32_t)send << 15) | ((uint32_t)txdata << 0));
}

__INLINE void ps2_ps2txdata_send_setf(uint8_t send)
{
    ASSERT_ERR((((uint32_t)send << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_PL_WR(PS2_PS2TXDATA_ADDR, (REG_PL_RD(PS2_PS2TXDATA_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)send << 15));
}

__INLINE void ps2_ps2txdata_txdata_setf(uint8_t txdata)
{
    ASSERT_ERR((((uint32_t)txdata << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(PS2_PS2TXDATA_ADDR, (REG_PL_RD(PS2_PS2TXDATA_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)txdata << 0));
}

/**
 * @brief PS2RXDATA register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00               RXDATA   0x0
 * </pre>
 */
#define PS2_PS2RXDATA_ADDR   0x0040301C
#define PS2_PS2RXDATA_OFFSET 0x0000001C
#define PS2_PS2RXDATA_INDEX  0x00000007
#define PS2_PS2RXDATA_RESET  0x00000000

__INLINE uint32_t ps2_ps2rxdata_get(void)
{
    return REG_PL_RD(PS2_PS2RXDATA_ADDR);
}

// field definitions
#define PS2_RXDATA_MASK   ((uint32_t)0xFFFFFFFF)
#define PS2_RXDATA_LSB    0
#define PS2_RXDATA_WIDTH  ((uint32_t)0x00000020)

#define PS2_RXDATA_RST    0x0

__INLINE uint32_t ps2_ps2rxdata_rxdata_getf(void)
{
    uint32_t localVal = REG_PL_RD(PS2_PS2RXDATA_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}


#endif // _REG_PS2_H_

