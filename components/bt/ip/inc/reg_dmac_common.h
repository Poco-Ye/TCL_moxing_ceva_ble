#ifndef _REG_DMAC_COMMON_H_
#define _REG_DMAC_COMMON_H_

#include <stdint.h>
#include "_reg_dmac_common.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_DMAC_COMMON_COUNT 4

#define REG_DMAC_COMMON_DECODING_MASK 0x0000000F

/**
 * @brief FFSR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15              FFFULL7   0
 *     14             FFEMPTY7   0
 *     13              FFFULL6   0
 *     12             FFEMPTY6   0
 *     11              FFFULL5   0
 *     10             FFEMPTY5   0
 *     09              FFFULL4   0
 *     08             FFEMPTY4   0
 *     07              FFFULL3   0
 *     06             FFEMPTY3   0
 *     05              FFFULL2   0
 *     04             FFEMPTY2   0
 *     03              FFFULL1   0
 *     02             FFEMPTY1   0
 *     01              FFFULL0   0
 *     00             FFEMPTY0   0
 * </pre>
 */
#define DMAC_FFSR_ADDR   0x00408000
#define DMAC_FFSR_OFFSET 0x00000000
#define DMAC_FFSR_INDEX  0x00000000
#define DMAC_FFSR_RESET  0x00000000

__INLINE uint32_t dmac_ffsr_get(void)
{
    return REG_PL_RD(DMAC_FFSR_ADDR);
}

// field definitions
#define DMAC_FFFULL7_BIT     ((uint32_t)0x00008000)
#define DMAC_FFFULL7_POS     15
#define DMAC_FFEMPTY7_BIT    ((uint32_t)0x00004000)
#define DMAC_FFEMPTY7_POS    14
#define DMAC_FFFULL6_BIT     ((uint32_t)0x00002000)
#define DMAC_FFFULL6_POS     13
#define DMAC_FFEMPTY6_BIT    ((uint32_t)0x00001000)
#define DMAC_FFEMPTY6_POS    12
#define DMAC_FFFULL5_BIT     ((uint32_t)0x00000800)
#define DMAC_FFFULL5_POS     11
#define DMAC_FFEMPTY5_BIT    ((uint32_t)0x00000400)
#define DMAC_FFEMPTY5_POS    10
#define DMAC_FFFULL4_BIT     ((uint32_t)0x00000200)
#define DMAC_FFFULL4_POS     9
#define DMAC_FFEMPTY4_BIT    ((uint32_t)0x00000100)
#define DMAC_FFEMPTY4_POS    8
#define DMAC_FFFULL3_BIT     ((uint32_t)0x00000080)
#define DMAC_FFFULL3_POS     7
#define DMAC_FFEMPTY3_BIT    ((uint32_t)0x00000040)
#define DMAC_FFEMPTY3_POS    6
#define DMAC_FFFULL2_BIT     ((uint32_t)0x00000020)
#define DMAC_FFFULL2_POS     5
#define DMAC_FFEMPTY2_BIT    ((uint32_t)0x00000010)
#define DMAC_FFEMPTY2_POS    4
#define DMAC_FFFULL1_BIT     ((uint32_t)0x00000008)
#define DMAC_FFFULL1_POS     3
#define DMAC_FFEMPTY1_BIT    ((uint32_t)0x00000004)
#define DMAC_FFEMPTY1_POS    2
#define DMAC_FFFULL0_BIT     ((uint32_t)0x00000002)
#define DMAC_FFFULL0_POS     1
#define DMAC_FFEMPTY0_BIT    ((uint32_t)0x00000001)
#define DMAC_FFEMPTY0_POS    0

#define DMAC_FFFULL7_RST     0x0
#define DMAC_FFEMPTY7_RST    0x0
#define DMAC_FFFULL6_RST     0x0
#define DMAC_FFEMPTY6_RST    0x0
#define DMAC_FFFULL5_RST     0x0
#define DMAC_FFEMPTY5_RST    0x0
#define DMAC_FFFULL4_RST     0x0
#define DMAC_FFEMPTY4_RST    0x0
#define DMAC_FFFULL3_RST     0x0
#define DMAC_FFEMPTY3_RST    0x0
#define DMAC_FFFULL2_RST     0x0
#define DMAC_FFEMPTY2_RST    0x0
#define DMAC_FFFULL1_RST     0x0
#define DMAC_FFEMPTY1_RST    0x0
#define DMAC_FFFULL0_RST     0x0
#define DMAC_FFEMPTY0_RST    0x0

__INLINE void dmac_ffsr_unpack(uint8_t* fffull7, uint8_t* ffempty7, uint8_t* fffull6, uint8_t* ffempty6, uint8_t* fffull5, uint8_t* ffempty5, uint8_t* fffull4, uint8_t* ffempty4, uint8_t* fffull3, uint8_t* ffempty3, uint8_t* fffull2, uint8_t* ffempty2, uint8_t* fffull1, uint8_t* ffempty1, uint8_t* fffull0, uint8_t* ffempty0)
{
    uint32_t localVal = REG_PL_RD(DMAC_FFSR_ADDR);

    *fffull7 = (localVal & ((uint32_t)0x00008000)) >> 15;
    *ffempty7 = (localVal & ((uint32_t)0x00004000)) >> 14;
    *fffull6 = (localVal & ((uint32_t)0x00002000)) >> 13;
    *ffempty6 = (localVal & ((uint32_t)0x00001000)) >> 12;
    *fffull5 = (localVal & ((uint32_t)0x00000800)) >> 11;
    *ffempty5 = (localVal & ((uint32_t)0x00000400)) >> 10;
    *fffull4 = (localVal & ((uint32_t)0x00000200)) >> 9;
    *ffempty4 = (localVal & ((uint32_t)0x00000100)) >> 8;
    *fffull3 = (localVal & ((uint32_t)0x00000080)) >> 7;
    *ffempty3 = (localVal & ((uint32_t)0x00000040)) >> 6;
    *fffull2 = (localVal & ((uint32_t)0x00000020)) >> 5;
    *ffempty2 = (localVal & ((uint32_t)0x00000010)) >> 4;
    *fffull1 = (localVal & ((uint32_t)0x00000008)) >> 3;
    *ffempty1 = (localVal & ((uint32_t)0x00000004)) >> 2;
    *fffull0 = (localVal & ((uint32_t)0x00000002)) >> 1;
    *ffempty0 = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t dmac_fffull7_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_FFSR_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE uint8_t dmac_ffempty7_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_FFSR_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE uint8_t dmac_fffull6_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_FFSR_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE uint8_t dmac_ffempty6_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_FFSR_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE uint8_t dmac_fffull5_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_FFSR_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE uint8_t dmac_ffempty5_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_FFSR_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE uint8_t dmac_fffull4_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_FFSR_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE uint8_t dmac_ffempty4_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_FFSR_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE uint8_t dmac_fffull3_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_FFSR_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE uint8_t dmac_ffempty3_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_FFSR_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t dmac_fffull2_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_FFSR_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t dmac_ffempty2_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_FFSR_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t dmac_fffull1_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_FFSR_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t dmac_ffempty1_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_FFSR_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t dmac_fffull0_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_FFSR_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t dmac_ffempty0_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_FFSR_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief ISR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24             DEINTLLI   0x0
 *  23:16             DEINTSRC   0x0
 *  15:08             DEINTDST   0x0
 *  07:00                TCINT   0x0
 * </pre>
 */
#define DMAC_ISR_ADDR   0x00408004
#define DMAC_ISR_OFFSET 0x00000004
#define DMAC_ISR_INDEX  0x00000001
#define DMAC_ISR_RESET  0x00000000

__INLINE uint32_t dmac_isr_get(void)
{
    return REG_PL_RD(DMAC_ISR_ADDR);
}

// field definitions
#define DMAC_DEINTLLI_MASK   ((uint32_t)0xFF000000)
#define DMAC_DEINTLLI_LSB    24
#define DMAC_DEINTLLI_WIDTH  ((uint32_t)0x00000008)
#define DMAC_DEINTSRC_MASK   ((uint32_t)0x00FF0000)
#define DMAC_DEINTSRC_LSB    16
#define DMAC_DEINTSRC_WIDTH  ((uint32_t)0x00000008)
#define DMAC_DEINTDST_MASK   ((uint32_t)0x0000FF00)
#define DMAC_DEINTDST_LSB    8
#define DMAC_DEINTDST_WIDTH  ((uint32_t)0x00000008)
#define DMAC_TCINT_MASK      ((uint32_t)0x000000FF)
#define DMAC_TCINT_LSB       0
#define DMAC_TCINT_WIDTH     ((uint32_t)0x00000008)

#define DMAC_DEINTLLI_RST    0x0
#define DMAC_DEINTSRC_RST    0x0
#define DMAC_DEINTDST_RST    0x0
#define DMAC_TCINT_RST       0x0

__INLINE void dmac_isr_unpack(uint8_t* deintlli, uint8_t* deintsrc, uint8_t* deintdst, uint8_t* tcint)
{
    uint32_t localVal = REG_PL_RD(DMAC_ISR_ADDR);

    *deintlli = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *deintsrc = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *deintdst = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *tcint = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t dmac_deintlli_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_ISR_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE uint8_t dmac_deintsrc_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_ISR_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE uint8_t dmac_deintdst_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_ISR_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE uint8_t dmac_tcint_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_ISR_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

/**
 * @brief RISR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24          RAWDEINTLLI   0x0
 *  23:16          RAWDEINTSRC   0x0
 *  15:08          RAWDEINTDST   0x0
 *  07:00             RAWTCINT   0x0
 * </pre>
 */
#define DMAC_RISR_ADDR   0x00408008
#define DMAC_RISR_OFFSET 0x00000008
#define DMAC_RISR_INDEX  0x00000002
#define DMAC_RISR_RESET  0x00000000

__INLINE uint32_t dmac_risr_get(void)
{
    return REG_PL_RD(DMAC_RISR_ADDR);
}

__INLINE void dmac_risr_set(uint32_t value)
{
    REG_PL_WR(DMAC_RISR_ADDR, value);
}

// field definitions
#define DMAC_RAWDEINTLLI_MASK   ((uint32_t)0xFF000000)
#define DMAC_RAWDEINTLLI_LSB    24
#define DMAC_RAWDEINTLLI_WIDTH  ((uint32_t)0x00000008)
#define DMAC_RAWDEINTSRC_MASK   ((uint32_t)0x00FF0000)
#define DMAC_RAWDEINTSRC_LSB    16
#define DMAC_RAWDEINTSRC_WIDTH  ((uint32_t)0x00000008)
#define DMAC_RAWDEINTDST_MASK   ((uint32_t)0x0000FF00)
#define DMAC_RAWDEINTDST_LSB    8
#define DMAC_RAWDEINTDST_WIDTH  ((uint32_t)0x00000008)
#define DMAC_RAWTCINT_MASK      ((uint32_t)0x000000FF)
#define DMAC_RAWTCINT_LSB       0
#define DMAC_RAWTCINT_WIDTH     ((uint32_t)0x00000008)

#define DMAC_RAWDEINTLLI_RST    0x0
#define DMAC_RAWDEINTSRC_RST    0x0
#define DMAC_RAWDEINTDST_RST    0x0
#define DMAC_RAWTCINT_RST       0x0

__INLINE void dmac_risr_pack(uint8_t rawdeintlli, uint8_t rawdeintsrc, uint8_t rawdeintdst, uint8_t rawtcint)
{
    ASSERT_ERR((((uint32_t)rawdeintlli << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)rawdeintsrc << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)rawdeintdst << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)rawtcint << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(DMAC_RISR_ADDR,  ((uint32_t)rawdeintlli << 24) | ((uint32_t)rawdeintsrc << 16) | ((uint32_t)rawdeintdst << 8) | ((uint32_t)rawtcint << 0));
}

__INLINE void dmac_risr_unpack(uint8_t* rawdeintlli, uint8_t* rawdeintsrc, uint8_t* rawdeintdst, uint8_t* rawtcint)
{
    uint32_t localVal = REG_PL_RD(DMAC_RISR_ADDR);

    *rawdeintlli = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *rawdeintsrc = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *rawdeintdst = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *rawtcint = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t dmac_rawdeintlli_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_RISR_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void dmac_rawdeintlli_setf(uint8_t rawdeintlli)
{
    ASSERT_ERR((((uint32_t)rawdeintlli << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_PL_WR(DMAC_RISR_ADDR, (REG_PL_RD(DMAC_RISR_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)rawdeintlli << 24));
}

__INLINE uint8_t dmac_rawdeintsrc_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_RISR_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void dmac_rawdeintsrc_setf(uint8_t rawdeintsrc)
{
    ASSERT_ERR((((uint32_t)rawdeintsrc << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_PL_WR(DMAC_RISR_ADDR, (REG_PL_RD(DMAC_RISR_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)rawdeintsrc << 16));
}

__INLINE uint8_t dmac_rawdeintdst_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_RISR_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void dmac_rawdeintdst_setf(uint8_t rawdeintdst)
{
    ASSERT_ERR((((uint32_t)rawdeintdst << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_PL_WR(DMAC_RISR_ADDR, (REG_PL_RD(DMAC_RISR_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)rawdeintdst << 8));
}

__INLINE uint8_t dmac_rawtcint_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_RISR_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void dmac_rawtcint_setf(uint8_t rawtcint)
{
    ASSERT_ERR((((uint32_t)rawtcint << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(DMAC_RISR_ADDR, (REG_PL_RD(DMAC_RISR_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)rawtcint << 0));
}

/**
 * @brief ICLR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24          CLRDEINTLLI   0x0
 *  23:16          CLRDEINTSRC   0x0
 *  15:08          CLRDEINTDST   0x0
 *  07:00             CLRTCINT   0x0
 * </pre>
 */
#define DMAC_ICLR_ADDR   0x0040800C
#define DMAC_ICLR_OFFSET 0x0000000C
#define DMAC_ICLR_INDEX  0x00000003
#define DMAC_ICLR_RESET  0x00000000

__INLINE uint32_t dmac_iclr_get(void)
{
    return REG_PL_RD(DMAC_ICLR_ADDR);
}

__INLINE void dmac_iclr_set(uint32_t value)
{
    REG_PL_WR(DMAC_ICLR_ADDR, value);
}

// field definitions
#define DMAC_CLRDEINTLLI_MASK   ((uint32_t)0xFF000000)
#define DMAC_CLRDEINTLLI_LSB    24
#define DMAC_CLRDEINTLLI_WIDTH  ((uint32_t)0x00000008)
#define DMAC_CLRDEINTSRC_MASK   ((uint32_t)0x00FF0000)
#define DMAC_CLRDEINTSRC_LSB    16
#define DMAC_CLRDEINTSRC_WIDTH  ((uint32_t)0x00000008)
#define DMAC_CLRDEINTDST_MASK   ((uint32_t)0x0000FF00)
#define DMAC_CLRDEINTDST_LSB    8
#define DMAC_CLRDEINTDST_WIDTH  ((uint32_t)0x00000008)
#define DMAC_CLRTCINT_MASK      ((uint32_t)0x000000FF)
#define DMAC_CLRTCINT_LSB       0
#define DMAC_CLRTCINT_WIDTH     ((uint32_t)0x00000008)

#define DMAC_CLRDEINTLLI_RST    0x0
#define DMAC_CLRDEINTSRC_RST    0x0
#define DMAC_CLRDEINTDST_RST    0x0
#define DMAC_CLRTCINT_RST       0x0

__INLINE void dmac_iclr_pack(uint8_t clrdeintlli, uint8_t clrdeintsrc, uint8_t clrdeintdst, uint8_t clrtcint)
{
    ASSERT_ERR((((uint32_t)clrdeintlli << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)clrdeintsrc << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)clrdeintdst << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)clrtcint << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(DMAC_ICLR_ADDR,  ((uint32_t)clrdeintlli << 24) | ((uint32_t)clrdeintsrc << 16) | ((uint32_t)clrdeintdst << 8) | ((uint32_t)clrtcint << 0));
}

__INLINE void dmac_iclr_unpack(uint8_t* clrdeintlli, uint8_t* clrdeintsrc, uint8_t* clrdeintdst, uint8_t* clrtcint)
{
    uint32_t localVal = REG_PL_RD(DMAC_ICLR_ADDR);

    *clrdeintlli = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *clrdeintsrc = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *clrdeintdst = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *clrtcint = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t dmac_clrdeintlli_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_ICLR_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void dmac_clrdeintlli_setf(uint8_t clrdeintlli)
{
    ASSERT_ERR((((uint32_t)clrdeintlli << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_PL_WR(DMAC_ICLR_ADDR, (REG_PL_RD(DMAC_ICLR_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)clrdeintlli << 24));
}

__INLINE uint8_t dmac_clrdeintsrc_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_ICLR_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void dmac_clrdeintsrc_setf(uint8_t clrdeintsrc)
{
    ASSERT_ERR((((uint32_t)clrdeintsrc << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_PL_WR(DMAC_ICLR_ADDR, (REG_PL_RD(DMAC_ICLR_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)clrdeintsrc << 16));
}

__INLINE uint8_t dmac_clrdeintdst_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_ICLR_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void dmac_clrdeintdst_setf(uint8_t clrdeintdst)
{
    ASSERT_ERR((((uint32_t)clrdeintdst << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_PL_WR(DMAC_ICLR_ADDR, (REG_PL_RD(DMAC_ICLR_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)clrdeintdst << 8));
}

__INLINE uint8_t dmac_clrtcint_getf(void)
{
    uint32_t localVal = REG_PL_RD(DMAC_ICLR_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void dmac_clrtcint_setf(uint8_t clrtcint)
{
    ASSERT_ERR((((uint32_t)clrtcint << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(DMAC_ICLR_ADDR, (REG_PL_RD(DMAC_ICLR_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)clrtcint << 0));
}


#endif // _REG_DMAC_COMMON_H_

