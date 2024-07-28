#ifndef _REG_DMAC_CHANNEL_H_
#define _REG_DMAC_CHANNEL_H_

#include <stdint.h>
#include "_reg_dmac_channel.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_DMAC_CHANNEL_COUNT 8

#define REG_DMAC_CHANNEL_DECODING_MASK 0x0000001F

/**
 * @brief SAR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00                  SAR   0x0
 * </pre>
 */
#define DMAC_SAR_ADDR   0x00408100
#define DMAC_SAR_OFFSET 0x00000000
#define DMAC_SAR_INDEX  0x00000000
#define DMAC_SAR_RESET  0x00000000

__INLINE uint32_t dmac_sar_get(int elt_idx)
{
    return REG_PL_RD(DMAC_SAR_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
}

__INLINE void dmac_sar_set(int elt_idx, uint32_t value)
{
    REG_PL_WR(DMAC_SAR_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, value);
}

// field definitions
#define DMAC_SAR_MASK   ((uint32_t)0xFFFFFFFF)
#define DMAC_SAR_LSB    0
#define DMAC_SAR_WIDTH  ((uint32_t)0x00000020)

#define DMAC_SAR_RST    0x0

__INLINE uint32_t dmac_sar_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_SAR_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief DAR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00                  DAR   0x0
 * </pre>
 */
#define DMAC_DAR_ADDR   0x00408104
#define DMAC_DAR_OFFSET 0x00000004
#define DMAC_DAR_INDEX  0x00000001
#define DMAC_DAR_RESET  0x00000000

__INLINE void dmac_dar_set(int elt_idx, uint32_t value)
{
    REG_PL_WR(DMAC_DAR_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, value);
}

// field definitions
#define DMAC_DAR_MASK   ((uint32_t)0xFFFFFFFF)
#define DMAC_DAR_LSB    0
#define DMAC_DAR_WIDTH  ((uint32_t)0x00000020)

#define DMAC_DAR_RST    0x0

__INLINE void dmac_dar_setf(int elt_idx, uint32_t dar)
{
    ASSERT_ERR((((uint32_t)dar << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(DMAC_DAR_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (uint32_t)dar << 0);
}

/**
 * @brief CCFGR1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:19          BLKXFERSIZE   0x0
 *     18            BUSLCKSRC   0
 *     17            BUSLCKDST   0
 *     16                CHLCK   0
 *  15:13          BRSTSIZESRC   0x0
 *  12:10          BRSTSIZEDST   0x0
 *  09:08         SRCXFERWIDTH   0x0
 *  07:06         DSTXFERWIDTH   0x0
 *     05           MSTRSELSRC   0
 *     04           MSTRSELDST   0
 *     03            ADDRRISRC   0
 *     02            ADDRRIDST   0
 *     01           ADDRINCSRC   0
 *     00           ADDRINCDST   0
 * </pre>
 */
#define DMAC_CCFGR1_ADDR   0x00408108
#define DMAC_CCFGR1_OFFSET 0x00000008
#define DMAC_CCFGR1_INDEX  0x00000002
#define DMAC_CCFGR1_RESET  0x00000000

__INLINE uint32_t dmac_ccfgr1_get(int elt_idx)
{
    return REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
}

__INLINE void dmac_ccfgr1_set(int elt_idx, uint32_t value)
{
    REG_PL_WR(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, value);
}

// field definitions
#define DMAC_BLKXFERSIZE_MASK    ((uint32_t)0xFFF80000)
#define DMAC_BLKXFERSIZE_LSB     19
#define DMAC_BLKXFERSIZE_WIDTH   ((uint32_t)0x0000000D)
#define DMAC_BUSLCKSRC_BIT       ((uint32_t)0x00040000)
#define DMAC_BUSLCKSRC_POS       18
#define DMAC_BUSLCKDST_BIT       ((uint32_t)0x00020000)
#define DMAC_BUSLCKDST_POS       17
#define DMAC_CHLCK_BIT           ((uint32_t)0x00010000)
#define DMAC_CHLCK_POS           16
#define DMAC_BRSTSIZESRC_MASK    ((uint32_t)0x0000E000)
#define DMAC_BRSTSIZESRC_LSB     13
#define DMAC_BRSTSIZESRC_WIDTH   ((uint32_t)0x00000003)
#define DMAC_BRSTSIZEDST_MASK    ((uint32_t)0x00001C00)
#define DMAC_BRSTSIZEDST_LSB     10
#define DMAC_BRSTSIZEDST_WIDTH   ((uint32_t)0x00000003)
#define DMAC_SRCXFERWIDTH_MASK   ((uint32_t)0x00000300)
#define DMAC_SRCXFERWIDTH_LSB    8
#define DMAC_SRCXFERWIDTH_WIDTH  ((uint32_t)0x00000002)
#define DMAC_DSTXFERWIDTH_MASK   ((uint32_t)0x000000C0)
#define DMAC_DSTXFERWIDTH_LSB    6
#define DMAC_DSTXFERWIDTH_WIDTH  ((uint32_t)0x00000002)
#define DMAC_MSTRSELSRC_BIT      ((uint32_t)0x00000020)
#define DMAC_MSTRSELSRC_POS      5
#define DMAC_MSTRSELDST_BIT      ((uint32_t)0x00000010)
#define DMAC_MSTRSELDST_POS      4
#define DMAC_ADDRRISRC_BIT       ((uint32_t)0x00000008)
#define DMAC_ADDRRISRC_POS       3
#define DMAC_ADDRRIDST_BIT       ((uint32_t)0x00000004)
#define DMAC_ADDRRIDST_POS       2
#define DMAC_ADDRINCSRC_BIT      ((uint32_t)0x00000002)
#define DMAC_ADDRINCSRC_POS      1
#define DMAC_ADDRINCDST_BIT      ((uint32_t)0x00000001)
#define DMAC_ADDRINCDST_POS      0

#define DMAC_BLKXFERSIZE_RST     0x0
#define DMAC_BUSLCKSRC_RST       0x0
#define DMAC_BUSLCKDST_RST       0x0
#define DMAC_CHLCK_RST           0x0
#define DMAC_BRSTSIZESRC_RST     0x0
#define DMAC_BRSTSIZEDST_RST     0x0
#define DMAC_SRCXFERWIDTH_RST    0x0
#define DMAC_DSTXFERWIDTH_RST    0x0
#define DMAC_MSTRSELSRC_RST      0x0
#define DMAC_MSTRSELDST_RST      0x0
#define DMAC_ADDRRISRC_RST       0x0
#define DMAC_ADDRRIDST_RST       0x0
#define DMAC_ADDRINCSRC_RST      0x0
#define DMAC_ADDRINCDST_RST      0x0

__INLINE void dmac_ccfgr1_pack(int elt_idx, uint16_t blkxfersize, uint8_t buslcksrc, uint8_t buslckdst, uint8_t chlck, uint8_t brstsizesrc, uint8_t brstsizedst, uint8_t srcxferwidth, uint8_t dstxferwidth, uint8_t mstrselsrc, uint8_t mstrseldst, uint8_t addrrisrc, uint8_t addrridst, uint8_t addrincsrc, uint8_t addrincdst)
{
    ASSERT_ERR((((uint32_t)blkxfersize << 19) & ~((uint32_t)0xFFF80000)) == 0);
    ASSERT_ERR((((uint32_t)buslcksrc << 18) & ~((uint32_t)0x00040000)) == 0);
    ASSERT_ERR((((uint32_t)buslckdst << 17) & ~((uint32_t)0x00020000)) == 0);
    ASSERT_ERR((((uint32_t)chlck << 16) & ~((uint32_t)0x00010000)) == 0);
    ASSERT_ERR((((uint32_t)brstsizesrc << 13) & ~((uint32_t)0x0000E000)) == 0);
    ASSERT_ERR((((uint32_t)brstsizedst << 10) & ~((uint32_t)0x00001C00)) == 0);
    ASSERT_ERR((((uint32_t)srcxferwidth << 8) & ~((uint32_t)0x00000300)) == 0);
    ASSERT_ERR((((uint32_t)dstxferwidth << 6) & ~((uint32_t)0x000000C0)) == 0);
    ASSERT_ERR((((uint32_t)mstrselsrc << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)mstrseldst << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)addrrisrc << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)addrridst << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)addrincsrc << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)addrincdst << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE,  ((uint32_t)blkxfersize << 19) | ((uint32_t)buslcksrc << 18) | ((uint32_t)buslckdst << 17) | ((uint32_t)chlck << 16) | ((uint32_t)brstsizesrc << 13) | ((uint32_t)brstsizedst << 10) | ((uint32_t)srcxferwidth << 8) | ((uint32_t)dstxferwidth << 6) | ((uint32_t)mstrselsrc << 5) | ((uint32_t)mstrseldst << 4) | ((uint32_t)addrrisrc << 3) | ((uint32_t)addrridst << 2) | ((uint32_t)addrincsrc << 1) | ((uint32_t)addrincdst << 0));
}

__INLINE void dmac_ccfgr1_unpack(int elt_idx, uint16_t* blkxfersize, uint8_t* buslcksrc, uint8_t* buslckdst, uint8_t* chlck, uint8_t* brstsizesrc, uint8_t* brstsizedst, uint8_t* srcxferwidth, uint8_t* dstxferwidth, uint8_t* mstrselsrc, uint8_t* mstrseldst, uint8_t* addrrisrc, uint8_t* addrridst, uint8_t* addrincsrc, uint8_t* addrincdst)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);

    *blkxfersize = (localVal & ((uint32_t)0xFFF80000)) >> 19;
    *buslcksrc = (localVal & ((uint32_t)0x00040000)) >> 18;
    *buslckdst = (localVal & ((uint32_t)0x00020000)) >> 17;
    *chlck = (localVal & ((uint32_t)0x00010000)) >> 16;
    *brstsizesrc = (localVal & ((uint32_t)0x0000E000)) >> 13;
    *brstsizedst = (localVal & ((uint32_t)0x00001C00)) >> 10;
    *srcxferwidth = (localVal & ((uint32_t)0x00000300)) >> 8;
    *dstxferwidth = (localVal & ((uint32_t)0x000000C0)) >> 6;
    *mstrselsrc = (localVal & ((uint32_t)0x00000020)) >> 5;
    *mstrseldst = (localVal & ((uint32_t)0x00000010)) >> 4;
    *addrrisrc = (localVal & ((uint32_t)0x00000008)) >> 3;
    *addrridst = (localVal & ((uint32_t)0x00000004)) >> 2;
    *addrincsrc = (localVal & ((uint32_t)0x00000002)) >> 1;
    *addrincdst = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint16_t dmac_blkxfersize_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0xFFF80000)) >> 19);
}

__INLINE void dmac_blkxfersize_setf(int elt_idx, uint16_t blkxfersize)
{
    ASSERT_ERR((((uint32_t)blkxfersize << 19) & ~((uint32_t)0xFFF80000)) == 0);
    REG_PL_WR(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0xFFF80000)) | ((uint32_t)blkxfersize << 19));
}

__INLINE uint8_t dmac_buslcksrc_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void dmac_buslcksrc_setf(int elt_idx, uint8_t buslcksrc)
{
    ASSERT_ERR((((uint32_t)buslcksrc << 18) & ~((uint32_t)0x00040000)) == 0);
    REG_PL_WR(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00040000)) | ((uint32_t)buslcksrc << 18));
}

__INLINE uint8_t dmac_buslckdst_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE void dmac_buslckdst_setf(int elt_idx, uint8_t buslckdst)
{
    ASSERT_ERR((((uint32_t)buslckdst << 17) & ~((uint32_t)0x00020000)) == 0);
    REG_PL_WR(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00020000)) | ((uint32_t)buslckdst << 17));
}

__INLINE uint8_t dmac_chlck_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void dmac_chlck_setf(int elt_idx, uint8_t chlck)
{
    ASSERT_ERR((((uint32_t)chlck << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_PL_WR(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00010000)) | ((uint32_t)chlck << 16));
}

__INLINE uint8_t dmac_brstsizesrc_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x0000E000)) >> 13);
}

__INLINE void dmac_brstsizesrc_setf(int elt_idx, uint8_t brstsizesrc)
{
    ASSERT_ERR((((uint32_t)brstsizesrc << 13) & ~((uint32_t)0x0000E000)) == 0);
    REG_PL_WR(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x0000E000)) | ((uint32_t)brstsizesrc << 13));
}

__INLINE uint8_t dmac_brstsizedst_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00001C00)) >> 10);
}

__INLINE void dmac_brstsizedst_setf(int elt_idx, uint8_t brstsizedst)
{
    ASSERT_ERR((((uint32_t)brstsizedst << 10) & ~((uint32_t)0x00001C00)) == 0);
    REG_PL_WR(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00001C00)) | ((uint32_t)brstsizedst << 10));
}

__INLINE uint8_t dmac_srcxferwidth_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00000300)) >> 8);
}

__INLINE void dmac_srcxferwidth_setf(int elt_idx, uint8_t srcxferwidth)
{
    ASSERT_ERR((((uint32_t)srcxferwidth << 8) & ~((uint32_t)0x00000300)) == 0);
    REG_PL_WR(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00000300)) | ((uint32_t)srcxferwidth << 8));
}

__INLINE uint8_t dmac_dstxferwidth_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x000000C0)) >> 6);
}

__INLINE void dmac_dstxferwidth_setf(int elt_idx, uint8_t dstxferwidth)
{
    ASSERT_ERR((((uint32_t)dstxferwidth << 6) & ~((uint32_t)0x000000C0)) == 0);
    REG_PL_WR(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x000000C0)) | ((uint32_t)dstxferwidth << 6));
}

__INLINE uint8_t dmac_mstrselsrc_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void dmac_mstrselsrc_setf(int elt_idx, uint8_t mstrselsrc)
{
    ASSERT_ERR((((uint32_t)mstrselsrc << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_PL_WR(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00000020)) | ((uint32_t)mstrselsrc << 5));
}

__INLINE uint8_t dmac_mstrseldst_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void dmac_mstrseldst_setf(int elt_idx, uint8_t mstrseldst)
{
    ASSERT_ERR((((uint32_t)mstrseldst << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_PL_WR(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00000010)) | ((uint32_t)mstrseldst << 4));
}

__INLINE uint8_t dmac_addrrisrc_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void dmac_addrrisrc_setf(int elt_idx, uint8_t addrrisrc)
{
    ASSERT_ERR((((uint32_t)addrrisrc << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_PL_WR(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00000008)) | ((uint32_t)addrrisrc << 3));
}

__INLINE uint8_t dmac_addrridst_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void dmac_addrridst_setf(int elt_idx, uint8_t addrridst)
{
    ASSERT_ERR((((uint32_t)addrridst << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_PL_WR(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00000004)) | ((uint32_t)addrridst << 2));
}

__INLINE uint8_t dmac_addrincsrc_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void dmac_addrincsrc_setf(int elt_idx, uint8_t addrincsrc)
{
    ASSERT_ERR((((uint32_t)addrincsrc << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_PL_WR(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00000002)) | ((uint32_t)addrincsrc << 1));
}

__INLINE uint8_t dmac_addrincdst_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void dmac_addrincdst_setf(int elt_idx, uint8_t addrincdst)
{
    ASSERT_ERR((((uint32_t)addrincdst << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR1_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00000001)) | ((uint32_t)addrincdst << 0));
}

/**
 * @brief CCFGR2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     09           MSTSEL_LLI   0
 *     08               MEMSET   0
 *     07               TCMASK   0
 *     06               DEMASK   0
 *     05         SWBRSTREQSRC   0
 *     04         SWBRSTREQDST   0
 *     03          SWSGLREQSRC   0
 *     02          SWSGLREQDST   0
 *     01           HWREQSRCEN   0
 *     00           HWREQDSTEN   0
 * </pre>
 */
#define DMAC_CCFGR2_ADDR   0x0040810C
#define DMAC_CCFGR2_OFFSET 0x0000000C
#define DMAC_CCFGR2_INDEX  0x00000003
#define DMAC_CCFGR2_RESET  0x00000000

__INLINE uint32_t dmac_ccfgr2_get(int elt_idx)
{
    return REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
}

__INLINE void dmac_ccfgr2_set(int elt_idx, uint32_t value)
{
    REG_PL_WR(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, value);
}

// field definitions
#define DMAC_MSTSEL_LLI_BIT      ((uint32_t)0x00000200)
#define DMAC_MSTSEL_LLI_POS      9
#define DMAC_MEMSET_BIT          ((uint32_t)0x00000100)
#define DMAC_MEMSET_POS          8
#define DMAC_TCMASK_BIT          ((uint32_t)0x00000080)
#define DMAC_TCMASK_POS          7
#define DMAC_DEMASK_BIT          ((uint32_t)0x00000040)
#define DMAC_DEMASK_POS          6
#define DMAC_SWBRSTREQSRC_BIT    ((uint32_t)0x00000020)
#define DMAC_SWBRSTREQSRC_POS    5
#define DMAC_SWBRSTREQDST_BIT    ((uint32_t)0x00000010)
#define DMAC_SWBRSTREQDST_POS    4
#define DMAC_SWSGLREQSRC_BIT     ((uint32_t)0x00000008)
#define DMAC_SWSGLREQSRC_POS     3
#define DMAC_SWSGLREQDST_BIT     ((uint32_t)0x00000004)
#define DMAC_SWSGLREQDST_POS     2
#define DMAC_HWREQSRCEN_BIT      ((uint32_t)0x00000002)
#define DMAC_HWREQSRCEN_POS      1
#define DMAC_HWREQDSTEN_BIT      ((uint32_t)0x00000001)
#define DMAC_HWREQDSTEN_POS      0

#define DMAC_MSTSEL_LLI_RST      0x0
#define DMAC_MEMSET_RST          0x0
#define DMAC_TCMASK_RST          0x0
#define DMAC_DEMASK_RST          0x0
#define DMAC_SWBRSTREQSRC_RST    0x0
#define DMAC_SWBRSTREQDST_RST    0x0
#define DMAC_SWSGLREQSRC_RST     0x0
#define DMAC_SWSGLREQDST_RST     0x0
#define DMAC_HWREQSRCEN_RST      0x0
#define DMAC_HWREQDSTEN_RST      0x0

__INLINE void dmac_ccfgr2_pack(int elt_idx, uint8_t mstsellli, uint8_t memset, uint8_t tcmask, uint8_t demask, uint8_t swbrstreqsrc, uint8_t swbrstreqdst, uint8_t swsglreqsrc, uint8_t swsglreqdst, uint8_t hwreqsrcen, uint8_t hwreqdsten)
{
    ASSERT_ERR((((uint32_t)mstsellli << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)memset << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)tcmask << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)demask << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)swbrstreqsrc << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)swbrstreqdst << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)swsglreqsrc << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)swsglreqdst << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)hwreqsrcen << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)hwreqdsten << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE,  ((uint32_t)mstsellli << 9) | ((uint32_t)memset << 8) | ((uint32_t)tcmask << 7) | ((uint32_t)demask << 6) | ((uint32_t)swbrstreqsrc << 5) | ((uint32_t)swbrstreqdst << 4) | ((uint32_t)swsglreqsrc << 3) | ((uint32_t)swsglreqdst << 2) | ((uint32_t)hwreqsrcen << 1) | ((uint32_t)hwreqdsten << 0));
}

__INLINE void dmac_ccfgr2_unpack(int elt_idx, uint8_t* mstsellli, uint8_t* memset, uint8_t* tcmask, uint8_t* demask, uint8_t* swbrstreqsrc, uint8_t* swbrstreqdst, uint8_t* swsglreqsrc, uint8_t* swsglreqdst, uint8_t* hwreqsrcen, uint8_t* hwreqdsten)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);

    *mstsellli = (localVal & ((uint32_t)0x00000200)) >> 9;
    *memset = (localVal & ((uint32_t)0x00000100)) >> 8;
    *tcmask = (localVal & ((uint32_t)0x00000080)) >> 7;
    *demask = (localVal & ((uint32_t)0x00000040)) >> 6;
    *swbrstreqsrc = (localVal & ((uint32_t)0x00000020)) >> 5;
    *swbrstreqdst = (localVal & ((uint32_t)0x00000010)) >> 4;
    *swsglreqsrc = (localVal & ((uint32_t)0x00000008)) >> 3;
    *swsglreqdst = (localVal & ((uint32_t)0x00000004)) >> 2;
    *hwreqsrcen = (localVal & ((uint32_t)0x00000002)) >> 1;
    *hwreqdsten = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t dmac_mstsel_lli_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void dmac_mstsel_lli_setf(int elt_idx, uint8_t mstsellli)
{
    ASSERT_ERR((((uint32_t)mstsellli << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_PL_WR(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00000200)) | ((uint32_t)mstsellli << 9));
}

__INLINE uint8_t dmac_memset_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void dmac_memset_setf(int elt_idx, uint8_t memset)
{
    ASSERT_ERR((((uint32_t)memset << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_PL_WR(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00000100)) | ((uint32_t)memset << 8));
}

__INLINE uint8_t dmac_tcmask_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void dmac_tcmask_setf(int elt_idx, uint8_t tcmask)
{
    ASSERT_ERR((((uint32_t)tcmask << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_PL_WR(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00000080)) | ((uint32_t)tcmask << 7));
}

__INLINE uint8_t dmac_demask_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void dmac_demask_setf(int elt_idx, uint8_t demask)
{
    ASSERT_ERR((((uint32_t)demask << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_PL_WR(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00000040)) | ((uint32_t)demask << 6));
}

__INLINE uint8_t dmac_swbrstreqsrc_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void dmac_swbrstreqsrc_setf(int elt_idx, uint8_t swbrstreqsrc)
{
    ASSERT_ERR((((uint32_t)swbrstreqsrc << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_PL_WR(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00000020)) | ((uint32_t)swbrstreqsrc << 5));
}

__INLINE uint8_t dmac_swbrstreqdst_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void dmac_swbrstreqdst_setf(int elt_idx, uint8_t swbrstreqdst)
{
    ASSERT_ERR((((uint32_t)swbrstreqdst << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_PL_WR(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00000010)) | ((uint32_t)swbrstreqdst << 4));
}

__INLINE uint8_t dmac_swsglreqsrc_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void dmac_swsglreqsrc_setf(int elt_idx, uint8_t swsglreqsrc)
{
    ASSERT_ERR((((uint32_t)swsglreqsrc << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_PL_WR(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00000008)) | ((uint32_t)swsglreqsrc << 3));
}

__INLINE uint8_t dmac_swsglreqdst_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void dmac_swsglreqdst_setf(int elt_idx, uint8_t swsglreqdst)
{
    ASSERT_ERR((((uint32_t)swsglreqdst << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_PL_WR(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00000004)) | ((uint32_t)swsglreqdst << 2));
}

__INLINE uint8_t dmac_hwreqsrcen_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void dmac_hwreqsrcen_setf(int elt_idx, uint8_t hwreqsrcen)
{
    ASSERT_ERR((((uint32_t)hwreqsrcen << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_PL_WR(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00000002)) | ((uint32_t)hwreqsrcen << 1));
}

__INLINE uint8_t dmac_hwreqdsten_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void dmac_hwreqdsten_setf(int elt_idx, uint8_t hwreqdsten)
{
    ASSERT_ERR((((uint32_t)hwreqdsten << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (REG_PL_RD(DMAC_CCFGR2_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE) & ~((uint32_t)0x00000001)) | ((uint32_t)hwreqdsten << 0));
}

/**
 * @brief LLPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00                LLPTR   0x0
 * </pre>
 */
#define DMAC_LLPTR_ADDR   0x00408110
#define DMAC_LLPTR_OFFSET 0x00000010
#define DMAC_LLPTR_INDEX  0x00000004
#define DMAC_LLPTR_RESET  0x00000000

__INLINE uint32_t dmac_llptr_get(int elt_idx)
{
    return REG_PL_RD(DMAC_LLPTR_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
}

__INLINE void dmac_llptr_set(int elt_idx, uint32_t value)
{
    REG_PL_WR(DMAC_LLPTR_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, value);
}

// field definitions
#define DMAC_LLPTR_MASK   ((uint32_t)0xFFFFFFFF)
#define DMAC_LLPTR_LSB    0
#define DMAC_LLPTR_WIDTH  ((uint32_t)0x00000020)

#define DMAC_LLPTR_RST    0x0

__INLINE uint32_t dmac_llptr_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_LLPTR_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void dmac_llptr_setf(int elt_idx, uint32_t llptr)
{
    ASSERT_ERR((((uint32_t)llptr << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(DMAC_LLPTR_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (uint32_t)llptr << 0);
}

/**
 * @brief FFLVLR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00                FFLVL   0x0
 * </pre>
 */
#define DMAC_FFLVLR_ADDR   0x00408114
#define DMAC_FFLVLR_OFFSET 0x00000014
#define DMAC_FFLVLR_INDEX  0x00000005
#define DMAC_FFLVLR_RESET  0x00000000

__INLINE uint32_t dmac_fflvlr_get(int elt_idx)
{
    return REG_PL_RD(DMAC_FFLVLR_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
}

__INLINE void dmac_fflvlr_set(int elt_idx, uint32_t value)
{
    REG_PL_WR(DMAC_FFLVLR_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, value);
}

// field definitions
#define DMAC_FFLVL_MASK   ((uint32_t)0x0000FFFF)
#define DMAC_FFLVL_LSB    0
#define DMAC_FFLVL_WIDTH  ((uint32_t)0x00000010)

#define DMAC_FFLVL_RST    0x0

__INLINE uint16_t dmac_fflvl_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_FFLVLR_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void dmac_fflvl_setf(int elt_idx, uint16_t fflvl)
{
    ASSERT_ERR((((uint32_t)fflvl << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_PL_WR(DMAC_FFLVLR_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (uint32_t)fflvl << 0);
}

/**
 * @brief XFRCNTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  12:00               XFRCNT   0x0
 * </pre>
 */
#define DMAC_XFRCNTR_ADDR   0x00408118
#define DMAC_XFRCNTR_OFFSET 0x00000018
#define DMAC_XFRCNTR_INDEX  0x00000006
#define DMAC_XFRCNTR_RESET  0x00000000

__INLINE uint32_t dmac_xfrcntr_get(int elt_idx)
{
    return REG_PL_RD(DMAC_XFRCNTR_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
}

__INLINE void dmac_xfrcntr_set(int elt_idx, uint32_t value)
{
    REG_PL_WR(DMAC_XFRCNTR_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, value);
}

// field definitions
#define DMAC_XFRCNT_MASK   ((uint32_t)0x00001FFF)
#define DMAC_XFRCNT_LSB    0
#define DMAC_XFRCNT_WIDTH  ((uint32_t)0x0000000D)

#define DMAC_XFRCNT_RST    0x0

__INLINE uint16_t dmac_xfrcnt_getf(int elt_idx)
{
    uint32_t localVal = REG_PL_RD(DMAC_XFRCNTR_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE);
    ASSERT_ERR((localVal & ~((uint32_t)0x00001FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void dmac_xfrcnt_setf(int elt_idx, uint16_t xfrcnt)
{
    ASSERT_ERR((((uint32_t)xfrcnt << 0) & ~((uint32_t)0x00001FFF)) == 0);
    REG_PL_WR(DMAC_XFRCNTR_ADDR + elt_idx * REG_DMAC_CHANNEL_SIZE, (uint32_t)xfrcnt << 0);
}


#endif // _REG_DMAC_CHANNEL_H_

