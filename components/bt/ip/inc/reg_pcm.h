#ifndef _REG_PCM_H_
#define _REG_PCM_H_

#include <stdint.h>
#include "_reg_pcm.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_PCM_COUNT 17

#define REG_PCM_DECODING_MASK 0x0000007F

/**
 * @brief LEAUDIOPATHCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31           LEAUDIORST   0
 *     00           LEAUDIO_EN   0
 * </pre>
 */
#define PCM_LEAUDIOPATHCNTL_ADDR   0x0040C000
#define PCM_LEAUDIOPATHCNTL_OFFSET 0x00000000
#define PCM_LEAUDIOPATHCNTL_INDEX  0x00000000
#define PCM_LEAUDIOPATHCNTL_RESET  0x00000000

__INLINE uint32_t pcm_leaudiopathcntl_get(void)
{
    return REG_PL_RD(PCM_LEAUDIOPATHCNTL_ADDR);
}

__INLINE void pcm_leaudiopathcntl_set(uint32_t value)
{
    REG_PL_WR(PCM_LEAUDIOPATHCNTL_ADDR, value);
}

// field definitions
#define PCM_LEAUDIORST_BIT    ((uint32_t)0x80000000)
#define PCM_LEAUDIORST_POS    31
#define PCM_LEAUDIO_EN_BIT    ((uint32_t)0x00000001)
#define PCM_LEAUDIO_EN_POS    0

#define PCM_LEAUDIORST_RST    0x0
#define PCM_LEAUDIO_EN_RST    0x0

__INLINE void pcm_leaudiopathcntl_pack(uint8_t leaudiorst, uint8_t leaudioen)
{
    ASSERT_ERR((((uint32_t)leaudiorst << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)leaudioen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(PCM_LEAUDIOPATHCNTL_ADDR,  ((uint32_t)leaudiorst << 31) | ((uint32_t)leaudioen << 0));
}

__INLINE void pcm_leaudiopathcntl_unpack(uint8_t* leaudiorst, uint8_t* leaudioen)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPATHCNTL_ADDR);

    *leaudiorst = (localVal & ((uint32_t)0x80000000)) >> 31;
    *leaudioen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcm_leaudiorst_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPATHCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void pcm_leaudiorst_setf(uint8_t leaudiorst)
{
    ASSERT_ERR((((uint32_t)leaudiorst << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_PL_WR(PCM_LEAUDIOPATHCNTL_ADDR, (REG_PL_RD(PCM_LEAUDIOPATHCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)leaudiorst << 31));
}

__INLINE uint8_t pcm_leaudio_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPATHCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pcm_leaudio_en_setf(uint8_t leaudioen)
{
    ASSERT_ERR((((uint32_t)leaudioen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(PCM_LEAUDIOPATHCNTL_ADDR, (REG_PL_RD(PCM_LEAUDIOPATHCNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)leaudioen << 0));
}

/**
 * @brief LEAUDIOSRCFIFOCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08         SRCFIFOAFTHR   0x0
 *  07:00         SRCFIFOAETHR   0x0
 * </pre>
 */
#define PCM_LEAUDIOSRCFIFOCNTL_ADDR   0x0040C004
#define PCM_LEAUDIOSRCFIFOCNTL_OFFSET 0x00000004
#define PCM_LEAUDIOSRCFIFOCNTL_INDEX  0x00000001
#define PCM_LEAUDIOSRCFIFOCNTL_RESET  0x00000000

__INLINE uint32_t pcm_leaudiosrcfifocntl_get(void)
{
    return REG_PL_RD(PCM_LEAUDIOSRCFIFOCNTL_ADDR);
}

__INLINE void pcm_leaudiosrcfifocntl_set(uint32_t value)
{
    REG_PL_WR(PCM_LEAUDIOSRCFIFOCNTL_ADDR, value);
}

// field definitions
#define PCM_SRCFIFOAFTHR_MASK   ((uint32_t)0x0000FF00)
#define PCM_SRCFIFOAFTHR_LSB    8
#define PCM_SRCFIFOAFTHR_WIDTH  ((uint32_t)0x00000008)
#define PCM_SRCFIFOAETHR_MASK   ((uint32_t)0x000000FF)
#define PCM_SRCFIFOAETHR_LSB    0
#define PCM_SRCFIFOAETHR_WIDTH  ((uint32_t)0x00000008)

#define PCM_SRCFIFOAFTHR_RST    0x0
#define PCM_SRCFIFOAETHR_RST    0x0

__INLINE void pcm_leaudiosrcfifocntl_pack(uint8_t srcfifoafthr, uint8_t srcfifoaethr)
{
    ASSERT_ERR((((uint32_t)srcfifoafthr << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)srcfifoaethr << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(PCM_LEAUDIOSRCFIFOCNTL_ADDR,  ((uint32_t)srcfifoafthr << 8) | ((uint32_t)srcfifoaethr << 0));
}

__INLINE void pcm_leaudiosrcfifocntl_unpack(uint8_t* srcfifoafthr, uint8_t* srcfifoaethr)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOSRCFIFOCNTL_ADDR);

    *srcfifoafthr = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *srcfifoaethr = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t pcm_srcfifoafthr_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOSRCFIFOCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void pcm_srcfifoafthr_setf(uint8_t srcfifoafthr)
{
    ASSERT_ERR((((uint32_t)srcfifoafthr << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_PL_WR(PCM_LEAUDIOSRCFIFOCNTL_ADDR, (REG_PL_RD(PCM_LEAUDIOSRCFIFOCNTL_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)srcfifoafthr << 8));
}

__INLINE uint8_t pcm_srcfifoaethr_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOSRCFIFOCNTL_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void pcm_srcfifoaethr_setf(uint8_t srcfifoaethr)
{
    ASSERT_ERR((((uint32_t)srcfifoaethr << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(PCM_LEAUDIOSRCFIFOCNTL_ADDR, (REG_PL_RD(PCM_LEAUDIOSRCFIFOCNTL_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)srcfifoaethr << 0));
}

/**
 * @brief LEAUDIOSRCFIFOSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     03    SRCFIFO_FULL_FLAG   0
 *     02   SRCFIFO_ALMOST_FULL_FLAG   0
 *     01   SRCFIFO_ALMOST_EMPTY_FLAG   0
 *     00   SRCFIFO_EMPTY_FLAG   1
 * </pre>
 */
#define PCM_LEAUDIOSRCFIFOSTAT_ADDR   0x0040C008
#define PCM_LEAUDIOSRCFIFOSTAT_OFFSET 0x00000008
#define PCM_LEAUDIOSRCFIFOSTAT_INDEX  0x00000002
#define PCM_LEAUDIOSRCFIFOSTAT_RESET  0x00000001

__INLINE uint32_t pcm_leaudiosrcfifostat_get(void)
{
    return REG_PL_RD(PCM_LEAUDIOSRCFIFOSTAT_ADDR);
}

// field definitions
#define PCM_SRCFIFO_FULL_FLAG_BIT            ((uint32_t)0x00000008)
#define PCM_SRCFIFO_FULL_FLAG_POS            3
#define PCM_SRCFIFO_ALMOST_FULL_FLAG_BIT     ((uint32_t)0x00000004)
#define PCM_SRCFIFO_ALMOST_FULL_FLAG_POS     2
#define PCM_SRCFIFO_ALMOST_EMPTY_FLAG_BIT    ((uint32_t)0x00000002)
#define PCM_SRCFIFO_ALMOST_EMPTY_FLAG_POS    1
#define PCM_SRCFIFO_EMPTY_FLAG_BIT           ((uint32_t)0x00000001)
#define PCM_SRCFIFO_EMPTY_FLAG_POS           0

#define PCM_SRCFIFO_FULL_FLAG_RST            0x0
#define PCM_SRCFIFO_ALMOST_FULL_FLAG_RST     0x0
#define PCM_SRCFIFO_ALMOST_EMPTY_FLAG_RST    0x0
#define PCM_SRCFIFO_EMPTY_FLAG_RST           0x1

__INLINE void pcm_leaudiosrcfifostat_unpack(uint8_t* srcfifofullflag, uint8_t* srcfifoalmostfullflag, uint8_t* srcfifoalmostemptyflag, uint8_t* srcfifoemptyflag)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOSRCFIFOSTAT_ADDR);

    *srcfifofullflag = (localVal & ((uint32_t)0x00000008)) >> 3;
    *srcfifoalmostfullflag = (localVal & ((uint32_t)0x00000004)) >> 2;
    *srcfifoalmostemptyflag = (localVal & ((uint32_t)0x00000002)) >> 1;
    *srcfifoemptyflag = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcm_srcfifo_full_flag_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOSRCFIFOSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t pcm_srcfifo_almost_full_flag_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOSRCFIFOSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t pcm_srcfifo_almost_empty_flag_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOSRCFIFOSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t pcm_srcfifo_empty_flag_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOSRCFIFOSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief LEAUDIOSNKFIFOCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08         SNKFIFOAFTHR   0x0
 *  07:00         SNKFIFOAETHR   0x0
 * </pre>
 */
#define PCM_LEAUDIOSNKFIFOCNTL_ADDR   0x0040C00C
#define PCM_LEAUDIOSNKFIFOCNTL_OFFSET 0x0000000C
#define PCM_LEAUDIOSNKFIFOCNTL_INDEX  0x00000003
#define PCM_LEAUDIOSNKFIFOCNTL_RESET  0x00000000

__INLINE uint32_t pcm_leaudiosnkfifocntl_get(void)
{
    return REG_PL_RD(PCM_LEAUDIOSNKFIFOCNTL_ADDR);
}

__INLINE void pcm_leaudiosnkfifocntl_set(uint32_t value)
{
    REG_PL_WR(PCM_LEAUDIOSNKFIFOCNTL_ADDR, value);
}

// field definitions
#define PCM_SNKFIFOAFTHR_MASK   ((uint32_t)0x0000FF00)
#define PCM_SNKFIFOAFTHR_LSB    8
#define PCM_SNKFIFOAFTHR_WIDTH  ((uint32_t)0x00000008)
#define PCM_SNKFIFOAETHR_MASK   ((uint32_t)0x000000FF)
#define PCM_SNKFIFOAETHR_LSB    0
#define PCM_SNKFIFOAETHR_WIDTH  ((uint32_t)0x00000008)

#define PCM_SNKFIFOAFTHR_RST    0x0
#define PCM_SNKFIFOAETHR_RST    0x0

__INLINE void pcm_leaudiosnkfifocntl_pack(uint8_t snkfifoafthr, uint8_t snkfifoaethr)
{
    ASSERT_ERR((((uint32_t)snkfifoafthr << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)snkfifoaethr << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(PCM_LEAUDIOSNKFIFOCNTL_ADDR,  ((uint32_t)snkfifoafthr << 8) | ((uint32_t)snkfifoaethr << 0));
}

__INLINE void pcm_leaudiosnkfifocntl_unpack(uint8_t* snkfifoafthr, uint8_t* snkfifoaethr)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOSNKFIFOCNTL_ADDR);

    *snkfifoafthr = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *snkfifoaethr = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t pcm_snkfifoafthr_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOSNKFIFOCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void pcm_snkfifoafthr_setf(uint8_t snkfifoafthr)
{
    ASSERT_ERR((((uint32_t)snkfifoafthr << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_PL_WR(PCM_LEAUDIOSNKFIFOCNTL_ADDR, (REG_PL_RD(PCM_LEAUDIOSNKFIFOCNTL_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)snkfifoafthr << 8));
}

__INLINE uint8_t pcm_snkfifoaethr_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOSNKFIFOCNTL_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void pcm_snkfifoaethr_setf(uint8_t snkfifoaethr)
{
    ASSERT_ERR((((uint32_t)snkfifoaethr << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(PCM_LEAUDIOSNKFIFOCNTL_ADDR, (REG_PL_RD(PCM_LEAUDIOSNKFIFOCNTL_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)snkfifoaethr << 0));
}

/**
 * @brief LEAUDIOSNKFIFOSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     03    SNKFIFO_FULL_FLAG   0
 *     02   SNKFIFO_ALMOST_FULL_FLAG   0
 *     01   SNKFIFO_ALMOST_EMPTY_FLAG   0
 *     00   SNKFIFO_EMPTY_FLAG   1
 * </pre>
 */
#define PCM_LEAUDIOSNKFIFOSTAT_ADDR   0x0040C010
#define PCM_LEAUDIOSNKFIFOSTAT_OFFSET 0x00000010
#define PCM_LEAUDIOSNKFIFOSTAT_INDEX  0x00000004
#define PCM_LEAUDIOSNKFIFOSTAT_RESET  0x00000001

__INLINE uint32_t pcm_leaudiosnkfifostat_get(void)
{
    return REG_PL_RD(PCM_LEAUDIOSNKFIFOSTAT_ADDR);
}

// field definitions
#define PCM_SNKFIFO_FULL_FLAG_BIT            ((uint32_t)0x00000008)
#define PCM_SNKFIFO_FULL_FLAG_POS            3
#define PCM_SNKFIFO_ALMOST_FULL_FLAG_BIT     ((uint32_t)0x00000004)
#define PCM_SNKFIFO_ALMOST_FULL_FLAG_POS     2
#define PCM_SNKFIFO_ALMOST_EMPTY_FLAG_BIT    ((uint32_t)0x00000002)
#define PCM_SNKFIFO_ALMOST_EMPTY_FLAG_POS    1
#define PCM_SNKFIFO_EMPTY_FLAG_BIT           ((uint32_t)0x00000001)
#define PCM_SNKFIFO_EMPTY_FLAG_POS           0

#define PCM_SNKFIFO_FULL_FLAG_RST            0x0
#define PCM_SNKFIFO_ALMOST_FULL_FLAG_RST     0x0
#define PCM_SNKFIFO_ALMOST_EMPTY_FLAG_RST    0x0
#define PCM_SNKFIFO_EMPTY_FLAG_RST           0x1

__INLINE void pcm_leaudiosnkfifostat_unpack(uint8_t* snkfifofullflag, uint8_t* snkfifoalmostfullflag, uint8_t* snkfifoalmostemptyflag, uint8_t* snkfifoemptyflag)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOSNKFIFOSTAT_ADDR);

    *snkfifofullflag = (localVal & ((uint32_t)0x00000008)) >> 3;
    *snkfifoalmostfullflag = (localVal & ((uint32_t)0x00000004)) >> 2;
    *snkfifoalmostemptyflag = (localVal & ((uint32_t)0x00000002)) >> 1;
    *snkfifoemptyflag = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcm_snkfifo_full_flag_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOSNKFIFOSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t pcm_snkfifo_almost_full_flag_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOSNKFIFOSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t pcm_snkfifo_almost_empty_flag_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOSNKFIFOSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t pcm_snkfifo_empty_flag_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOSNKFIFOSTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief LEAUDIOSRCDATA register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00              SRCDATA   0x0
 * </pre>
 */
#define PCM_LEAUDIOSRCDATA_ADDR   0x0040C014
#define PCM_LEAUDIOSRCDATA_OFFSET 0x00000014
#define PCM_LEAUDIOSRCDATA_INDEX  0x00000005
#define PCM_LEAUDIOSRCDATA_RESET  0x00000000

__INLINE uint32_t pcm_leaudiosrcdata_get(void)
{
    return REG_PL_RD(PCM_LEAUDIOSRCDATA_ADDR);
}

__INLINE void pcm_leaudiosrcdata_set(uint32_t value)
{
    REG_PL_WR(PCM_LEAUDIOSRCDATA_ADDR, value);
}

// field definitions
#define PCM_SRCDATA_MASK   ((uint32_t)0x0000FFFF)
#define PCM_SRCDATA_LSB    0
#define PCM_SRCDATA_WIDTH  ((uint32_t)0x00000010)

#define PCM_SRCDATA_RST    0x0

__INLINE uint16_t pcm_srcdata_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOSRCDATA_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void pcm_srcdata_setf(uint16_t srcdata)
{
    ASSERT_ERR((((uint32_t)srcdata << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_PL_WR(PCM_LEAUDIOSRCDATA_ADDR, (uint32_t)srcdata << 0);
}

/**
 * @brief LEAUDIOSNKDATA register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00              SNKDATA   0x0
 * </pre>
 */
#define PCM_LEAUDIOSNKDATA_ADDR   0x0040C018
#define PCM_LEAUDIOSNKDATA_OFFSET 0x00000018
#define PCM_LEAUDIOSNKDATA_INDEX  0x00000006
#define PCM_LEAUDIOSNKDATA_RESET  0x00000000

__INLINE uint32_t pcm_leaudiosnkdata_get(void)
{
    return REG_PL_RD(PCM_LEAUDIOSNKDATA_ADDR);
}

__INLINE void pcm_leaudiosnkdata_set(uint32_t value)
{
    REG_PL_WR(PCM_LEAUDIOSNKDATA_ADDR, value);
}

// field definitions
#define PCM_SNKDATA_MASK   ((uint32_t)0x0000FFFF)
#define PCM_SNKDATA_LSB    0
#define PCM_SNKDATA_WIDTH  ((uint32_t)0x00000010)

#define PCM_SNKDATA_RST    0x0

__INLINE uint16_t pcm_snkdata_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOSNKDATA_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void pcm_snkdata_setf(uint16_t snkdata)
{
    ASSERT_ERR((((uint32_t)snkdata << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_PL_WR(PCM_LEAUDIOSNKDATA_ADDR, (uint32_t)snkdata << 0);
}

/**
 * @brief LEAUDIORLCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15           PLFPATHSEL   0
 *  03:00         PLFAULAWCODE   0x0
 * </pre>
 */
#define PCM_LEAUDIORLCNTL_ADDR   0x0040C01C
#define PCM_LEAUDIORLCNTL_OFFSET 0x0000001C
#define PCM_LEAUDIORLCNTL_INDEX  0x00000007
#define PCM_LEAUDIORLCNTL_RESET  0x00000000

__INLINE uint32_t pcm_leaudiorlcntl_get(void)
{
    return REG_PL_RD(PCM_LEAUDIORLCNTL_ADDR);
}

__INLINE void pcm_leaudiorlcntl_set(uint32_t value)
{
    REG_PL_WR(PCM_LEAUDIORLCNTL_ADDR, value);
}

// field definitions
#define PCM_PLFPATHSEL_BIT      ((uint32_t)0x00008000)
#define PCM_PLFPATHSEL_POS      15
#define PCM_PLFAULAWCODE_MASK   ((uint32_t)0x0000000F)
#define PCM_PLFAULAWCODE_LSB    0
#define PCM_PLFAULAWCODE_WIDTH  ((uint32_t)0x00000004)

#define PCM_PLFPATHSEL_RST      0x0
#define PCM_PLFAULAWCODE_RST    0x0

__INLINE void pcm_leaudiorlcntl_pack(uint8_t plfpathsel, uint8_t plfaulawcode)
{
    ASSERT_ERR((((uint32_t)plfpathsel << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)plfaulawcode << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_PL_WR(PCM_LEAUDIORLCNTL_ADDR,  ((uint32_t)plfpathsel << 15) | ((uint32_t)plfaulawcode << 0));
}

__INLINE void pcm_leaudiorlcntl_unpack(uint8_t* plfpathsel, uint8_t* plfaulawcode)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIORLCNTL_ADDR);

    *plfpathsel = (localVal & ((uint32_t)0x00008000)) >> 15;
    *plfaulawcode = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t pcm_plfpathsel_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIORLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void pcm_plfpathsel_setf(uint8_t plfpathsel)
{
    ASSERT_ERR((((uint32_t)plfpathsel << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_PL_WR(PCM_LEAUDIORLCNTL_ADDR, (REG_PL_RD(PCM_LEAUDIORLCNTL_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)plfpathsel << 15));
}

__INLINE uint8_t pcm_plfaulawcode_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIORLCNTL_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void pcm_plfaulawcode_setf(uint8_t plfaulawcode)
{
    ASSERT_ERR((((uint32_t)plfaulawcode << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_PL_WR(PCM_LEAUDIORLCNTL_ADDR, (REG_PL_RD(PCM_LEAUDIORLCNTL_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)plfaulawcode << 0));
}

/**
 * @brief LEAUDIOPCMTIMINGREF register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  14:00         PCMTIMREFVAL   0x0
 * </pre>
 */
#define PCM_LEAUDIOPCMTIMINGREF_ADDR   0x0040C020
#define PCM_LEAUDIOPCMTIMINGREF_OFFSET 0x00000020
#define PCM_LEAUDIOPCMTIMINGREF_INDEX  0x00000008
#define PCM_LEAUDIOPCMTIMINGREF_RESET  0x00000000

__INLINE uint32_t pcm_leaudiopcmtimingref_get(void)
{
    return REG_PL_RD(PCM_LEAUDIOPCMTIMINGREF_ADDR);
}

__INLINE void pcm_leaudiopcmtimingref_set(uint32_t value)
{
    REG_PL_WR(PCM_LEAUDIOPCMTIMINGREF_ADDR, value);
}

// field definitions
#define PCM_PCMTIMREFVAL_MASK   ((uint32_t)0x00007FFF)
#define PCM_PCMTIMREFVAL_LSB    0
#define PCM_PCMTIMREFVAL_WIDTH  ((uint32_t)0x0000000F)

#define PCM_PCMTIMREFVAL_RST    0x0

__INLINE uint16_t pcm_pcmtimrefval_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMTIMINGREF_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x00007FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void pcm_pcmtimrefval_setf(uint16_t pcmtimrefval)
{
    ASSERT_ERR((((uint32_t)pcmtimrefval << 0) & ~((uint32_t)0x00007FFF)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMTIMINGREF_ADDR, (uint32_t)pcmtimrefval << 0);
}

/**
 * @brief LEAUDIOPCMGENCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     06          PCMLOOPBACK   0
 *     05        PLFMONO_LRSEL   0
 *     04       PLFMONO_STEREO   0
 *     03            PLFMSTSLV   0
 *     02          PLFBYTESWAP   0
 *     01            PLFLRSWAP   0
 *     00             PLFPCMEN   0
 * </pre>
 */
#define PCM_LEAUDIOPCMGENCNTL_ADDR   0x0040C024
#define PCM_LEAUDIOPCMGENCNTL_OFFSET 0x00000024
#define PCM_LEAUDIOPCMGENCNTL_INDEX  0x00000009
#define PCM_LEAUDIOPCMGENCNTL_RESET  0x00000000

__INLINE uint32_t pcm_leaudiopcmgencntl_get(void)
{
    return REG_PL_RD(PCM_LEAUDIOPCMGENCNTL_ADDR);
}

__INLINE void pcm_leaudiopcmgencntl_set(uint32_t value)
{
    REG_PL_WR(PCM_LEAUDIOPCMGENCNTL_ADDR, value);
}

// field definitions
#define PCM_PCMLOOPBACK_BIT       ((uint32_t)0x00000040)
#define PCM_PCMLOOPBACK_POS       6
#define PCM_PLFMONO_LRSEL_BIT     ((uint32_t)0x00000020)
#define PCM_PLFMONO_LRSEL_POS     5
#define PCM_PLFMONO_STEREO_BIT    ((uint32_t)0x00000010)
#define PCM_PLFMONO_STEREO_POS    4
#define PCM_PLFMSTSLV_BIT         ((uint32_t)0x00000008)
#define PCM_PLFMSTSLV_POS         3
#define PCM_PLFBYTESWAP_BIT       ((uint32_t)0x00000004)
#define PCM_PLFBYTESWAP_POS       2
#define PCM_PLFLRSWAP_BIT         ((uint32_t)0x00000002)
#define PCM_PLFLRSWAP_POS         1
#define PCM_PLFPCMEN_BIT          ((uint32_t)0x00000001)
#define PCM_PLFPCMEN_POS          0

#define PCM_PCMLOOPBACK_RST       0x0
#define PCM_PLFMONO_LRSEL_RST     0x0
#define PCM_PLFMONO_STEREO_RST    0x0
#define PCM_PLFMSTSLV_RST         0x0
#define PCM_PLFBYTESWAP_RST       0x0
#define PCM_PLFLRSWAP_RST         0x0
#define PCM_PLFPCMEN_RST          0x0

__INLINE void pcm_leaudiopcmgencntl_pack(uint8_t pcmloopback, uint8_t plfmonolrsel, uint8_t plfmonostereo, uint8_t plfmstslv, uint8_t plfbyteswap, uint8_t plflrswap, uint8_t plfpcmen)
{
    ASSERT_ERR((((uint32_t)pcmloopback << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)plfmonolrsel << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)plfmonostereo << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)plfmstslv << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)plfbyteswap << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)plflrswap << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)plfpcmen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMGENCNTL_ADDR,  ((uint32_t)pcmloopback << 6) | ((uint32_t)plfmonolrsel << 5) | ((uint32_t)plfmonostereo << 4) | ((uint32_t)plfmstslv << 3) | ((uint32_t)plfbyteswap << 2) | ((uint32_t)plflrswap << 1) | ((uint32_t)plfpcmen << 0));
}

__INLINE void pcm_leaudiopcmgencntl_unpack(uint8_t* pcmloopback, uint8_t* plfmonolrsel, uint8_t* plfmonostereo, uint8_t* plfmstslv, uint8_t* plfbyteswap, uint8_t* plflrswap, uint8_t* plfpcmen)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMGENCNTL_ADDR);

    *pcmloopback = (localVal & ((uint32_t)0x00000040)) >> 6;
    *plfmonolrsel = (localVal & ((uint32_t)0x00000020)) >> 5;
    *plfmonostereo = (localVal & ((uint32_t)0x00000010)) >> 4;
    *plfmstslv = (localVal & ((uint32_t)0x00000008)) >> 3;
    *plfbyteswap = (localVal & ((uint32_t)0x00000004)) >> 2;
    *plflrswap = (localVal & ((uint32_t)0x00000002)) >> 1;
    *plfpcmen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcm_pcmloopback_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void pcm_pcmloopback_setf(uint8_t pcmloopback)
{
    ASSERT_ERR((((uint32_t)pcmloopback << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMGENCNTL_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMGENCNTL_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)pcmloopback << 6));
}

__INLINE uint8_t pcm_plfmono_lrsel_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void pcm_plfmono_lrsel_setf(uint8_t plfmonolrsel)
{
    ASSERT_ERR((((uint32_t)plfmonolrsel << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMGENCNTL_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMGENCNTL_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)plfmonolrsel << 5));
}

__INLINE uint8_t pcm_plfmono_stereo_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void pcm_plfmono_stereo_setf(uint8_t plfmonostereo)
{
    ASSERT_ERR((((uint32_t)plfmonostereo << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMGENCNTL_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMGENCNTL_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)plfmonostereo << 4));
}

__INLINE uint8_t pcm_plfmstslv_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void pcm_plfmstslv_setf(uint8_t plfmstslv)
{
    ASSERT_ERR((((uint32_t)plfmstslv << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMGENCNTL_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMGENCNTL_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)plfmstslv << 3));
}

__INLINE uint8_t pcm_plfbyteswap_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void pcm_plfbyteswap_setf(uint8_t plfbyteswap)
{
    ASSERT_ERR((((uint32_t)plfbyteswap << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMGENCNTL_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMGENCNTL_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)plfbyteswap << 2));
}

__INLINE uint8_t pcm_plflrswap_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void pcm_plflrswap_setf(uint8_t plflrswap)
{
    ASSERT_ERR((((uint32_t)plflrswap << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMGENCNTL_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMGENCNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)plflrswap << 1));
}

__INLINE uint8_t pcm_plfpcmen_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pcm_plfpcmen_setf(uint8_t plfpcmen)
{
    ASSERT_ERR((((uint32_t)plfpcmen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMGENCNTL_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMGENCNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)plfpcmen << 0));
}

/**
 * @brief LEAUDIOPCMPHYSCNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  21:20      PLFFIRSTACTSLOT   0x0
 *  18:16            PLFSLOTNB   0x0
 *     13          PLFSAMPTYPE   0
 *     12            PLFSAMPSZ   0
 *     10            PLFLSB1ST   0
 *     09           PLFPCM_IOM   0
 *     08           PLFLRCHPOL   0
 *  05:04           PLFDOUTCFG   0x0
 *  02:00          PLFFSYNCSHP   0x0
 * </pre>
 */
#define PCM_LEAUDIOPCMPHYSCNTL0_ADDR   0x0040C028
#define PCM_LEAUDIOPCMPHYSCNTL0_OFFSET 0x00000028
#define PCM_LEAUDIOPCMPHYSCNTL0_INDEX  0x0000000A
#define PCM_LEAUDIOPCMPHYSCNTL0_RESET  0x00000000

__INLINE uint32_t pcm_leaudiopcmphyscntl0_get(void)
{
    return REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR);
}

__INLINE void pcm_leaudiopcmphyscntl0_set(uint32_t value)
{
    REG_PL_WR(PCM_LEAUDIOPCMPHYSCNTL0_ADDR, value);
}

// field definitions
#define PCM_PLFFIRSTACTSLOT_MASK   ((uint32_t)0x00300000)
#define PCM_PLFFIRSTACTSLOT_LSB    20
#define PCM_PLFFIRSTACTSLOT_WIDTH  ((uint32_t)0x00000002)
#define PCM_PLFSLOTNB_MASK         ((uint32_t)0x00070000)
#define PCM_PLFSLOTNB_LSB          16
#define PCM_PLFSLOTNB_WIDTH        ((uint32_t)0x00000003)
#define PCM_PLFSAMPTYPE_BIT        ((uint32_t)0x00002000)
#define PCM_PLFSAMPTYPE_POS        13
#define PCM_PLFSAMPSZ_BIT          ((uint32_t)0x00001000)
#define PCM_PLFSAMPSZ_POS          12
#define PCM_PLFLSB1ST_BIT          ((uint32_t)0x00000400)
#define PCM_PLFLSB1ST_POS          10
#define PCM_PLFPCM_IOM_BIT         ((uint32_t)0x00000200)
#define PCM_PLFPCM_IOM_POS         9
#define PCM_PLFLRCHPOL_BIT         ((uint32_t)0x00000100)
#define PCM_PLFLRCHPOL_POS         8
#define PCM_PLFDOUTCFG_MASK        ((uint32_t)0x00000030)
#define PCM_PLFDOUTCFG_LSB         4
#define PCM_PLFDOUTCFG_WIDTH       ((uint32_t)0x00000002)
#define PCM_PLFFSYNCSHP_MASK       ((uint32_t)0x00000007)
#define PCM_PLFFSYNCSHP_LSB        0
#define PCM_PLFFSYNCSHP_WIDTH      ((uint32_t)0x00000003)

#define PCM_PLFFIRSTACTSLOT_RST    0x0
#define PCM_PLFSLOTNB_RST          0x0
#define PCM_PLFSAMPTYPE_RST        0x0
#define PCM_PLFSAMPSZ_RST          0x0
#define PCM_PLFLSB1ST_RST          0x0
#define PCM_PLFPCM_IOM_RST         0x0
#define PCM_PLFLRCHPOL_RST         0x0
#define PCM_PLFDOUTCFG_RST         0x0
#define PCM_PLFFSYNCSHP_RST        0x0

__INLINE void pcm_leaudiopcmphyscntl0_pack(uint8_t plffirstactslot, uint8_t plfslotnb, uint8_t plfsamptype, uint8_t plfsampsz, uint8_t plflsb1st, uint8_t plfpcmiom, uint8_t plflrchpol, uint8_t plfdoutcfg, uint8_t plffsyncshp)
{
    ASSERT_ERR((((uint32_t)plffirstactslot << 20) & ~((uint32_t)0x00300000)) == 0);
    ASSERT_ERR((((uint32_t)plfslotnb << 16) & ~((uint32_t)0x00070000)) == 0);
    ASSERT_ERR((((uint32_t)plfsamptype << 13) & ~((uint32_t)0x00002000)) == 0);
    ASSERT_ERR((((uint32_t)plfsampsz << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)plflsb1st << 10) & ~((uint32_t)0x00000400)) == 0);
    ASSERT_ERR((((uint32_t)plfpcmiom << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)plflrchpol << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)plfdoutcfg << 4) & ~((uint32_t)0x00000030)) == 0);
    ASSERT_ERR((((uint32_t)plffsyncshp << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPHYSCNTL0_ADDR,  ((uint32_t)plffirstactslot << 20) | ((uint32_t)plfslotnb << 16) | ((uint32_t)plfsamptype << 13) | ((uint32_t)plfsampsz << 12) | ((uint32_t)plflsb1st << 10) | ((uint32_t)plfpcmiom << 9) | ((uint32_t)plflrchpol << 8) | ((uint32_t)plfdoutcfg << 4) | ((uint32_t)plffsyncshp << 0));
}

__INLINE void pcm_leaudiopcmphyscntl0_unpack(uint8_t* plffirstactslot, uint8_t* plfslotnb, uint8_t* plfsamptype, uint8_t* plfsampsz, uint8_t* plflsb1st, uint8_t* plfpcmiom, uint8_t* plflrchpol, uint8_t* plfdoutcfg, uint8_t* plffsyncshp)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR);

    *plffirstactslot = (localVal & ((uint32_t)0x00300000)) >> 20;
    *plfslotnb = (localVal & ((uint32_t)0x00070000)) >> 16;
    *plfsamptype = (localVal & ((uint32_t)0x00002000)) >> 13;
    *plfsampsz = (localVal & ((uint32_t)0x00001000)) >> 12;
    *plflsb1st = (localVal & ((uint32_t)0x00000400)) >> 10;
    *plfpcmiom = (localVal & ((uint32_t)0x00000200)) >> 9;
    *plflrchpol = (localVal & ((uint32_t)0x00000100)) >> 8;
    *plfdoutcfg = (localVal & ((uint32_t)0x00000030)) >> 4;
    *plffsyncshp = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t pcm_plffirstactslot_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00300000)) >> 20);
}

__INLINE void pcm_plffirstactslot_setf(uint8_t plffirstactslot)
{
    ASSERT_ERR((((uint32_t)plffirstactslot << 20) & ~((uint32_t)0x00300000)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPHYSCNTL0_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR) & ~((uint32_t)0x00300000)) | ((uint32_t)plffirstactslot << 20));
}

__INLINE uint8_t pcm_plfslotnb_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00070000)) >> 16);
}

__INLINE void pcm_plfslotnb_setf(uint8_t plfslotnb)
{
    ASSERT_ERR((((uint32_t)plfslotnb << 16) & ~((uint32_t)0x00070000)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPHYSCNTL0_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR) & ~((uint32_t)0x00070000)) | ((uint32_t)plfslotnb << 16));
}

__INLINE uint8_t pcm_plfsamptype_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void pcm_plfsamptype_setf(uint8_t plfsamptype)
{
    ASSERT_ERR((((uint32_t)plfsamptype << 13) & ~((uint32_t)0x00002000)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPHYSCNTL0_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)plfsamptype << 13));
}

__INLINE uint8_t pcm_plfsampsz_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void pcm_plfsampsz_setf(uint8_t plfsampsz)
{
    ASSERT_ERR((((uint32_t)plfsampsz << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPHYSCNTL0_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)plfsampsz << 12));
}

__INLINE uint8_t pcm_plflsb1st_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void pcm_plflsb1st_setf(uint8_t plflsb1st)
{
    ASSERT_ERR((((uint32_t)plflsb1st << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPHYSCNTL0_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)plflsb1st << 10));
}

__INLINE uint8_t pcm_plfpcm_iom_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void pcm_plfpcm_iom_setf(uint8_t plfpcmiom)
{
    ASSERT_ERR((((uint32_t)plfpcmiom << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPHYSCNTL0_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)plfpcmiom << 9));
}

__INLINE uint8_t pcm_plflrchpol_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void pcm_plflrchpol_setf(uint8_t plflrchpol)
{
    ASSERT_ERR((((uint32_t)plflrchpol << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPHYSCNTL0_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)plflrchpol << 8));
}

__INLINE uint8_t pcm_plfdoutcfg_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000030)) >> 4);
}

__INLINE void pcm_plfdoutcfg_setf(uint8_t plfdoutcfg)
{
    ASSERT_ERR((((uint32_t)plfdoutcfg << 4) & ~((uint32_t)0x00000030)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPHYSCNTL0_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR) & ~((uint32_t)0x00000030)) | ((uint32_t)plfdoutcfg << 4));
}

__INLINE uint8_t pcm_plffsyncshp_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void pcm_plffsyncshp_setf(uint8_t plffsyncshp)
{
    ASSERT_ERR((((uint32_t)plffsyncshp << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPHYSCNTL0_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL0_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)plffsyncshp << 0));
}

/**
 * @brief LEAUDIOPCMPHYSCNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31            PLFCLKINV   0
 *  23:16       PLFPCMCLKLIMIT   0x0
 *  08:00      PLFPCMCLKMAXVAL   0x0
 * </pre>
 */
#define PCM_LEAUDIOPCMPHYSCNTL1_ADDR   0x0040C02C
#define PCM_LEAUDIOPCMPHYSCNTL1_OFFSET 0x0000002C
#define PCM_LEAUDIOPCMPHYSCNTL1_INDEX  0x0000000B
#define PCM_LEAUDIOPCMPHYSCNTL1_RESET  0x00000000

__INLINE uint32_t pcm_leaudiopcmphyscntl1_get(void)
{
    return REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL1_ADDR);
}

__INLINE void pcm_leaudiopcmphyscntl1_set(uint32_t value)
{
    REG_PL_WR(PCM_LEAUDIOPCMPHYSCNTL1_ADDR, value);
}

// field definitions
#define PCM_PLFCLKINV_BIT          ((uint32_t)0x80000000)
#define PCM_PLFCLKINV_POS          31
#define PCM_PLFPCMCLKLIMIT_MASK    ((uint32_t)0x00FF0000)
#define PCM_PLFPCMCLKLIMIT_LSB     16
#define PCM_PLFPCMCLKLIMIT_WIDTH   ((uint32_t)0x00000008)
#define PCM_PLFPCMCLKMAXVAL_MASK   ((uint32_t)0x000001FF)
#define PCM_PLFPCMCLKMAXVAL_LSB    0
#define PCM_PLFPCMCLKMAXVAL_WIDTH  ((uint32_t)0x00000009)

#define PCM_PLFCLKINV_RST          0x0
#define PCM_PLFPCMCLKLIMIT_RST     0x0
#define PCM_PLFPCMCLKMAXVAL_RST    0x0

__INLINE void pcm_leaudiopcmphyscntl1_pack(uint8_t plfclkinv, uint8_t plfpcmclklimit, uint16_t plfpcmclkmaxval)
{
    ASSERT_ERR((((uint32_t)plfclkinv << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)plfpcmclklimit << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)plfpcmclkmaxval << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPHYSCNTL1_ADDR,  ((uint32_t)plfclkinv << 31) | ((uint32_t)plfpcmclklimit << 16) | ((uint32_t)plfpcmclkmaxval << 0));
}

__INLINE void pcm_leaudiopcmphyscntl1_unpack(uint8_t* plfclkinv, uint8_t* plfpcmclklimit, uint16_t* plfpcmclkmaxval)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL1_ADDR);

    *plfclkinv = (localVal & ((uint32_t)0x80000000)) >> 31;
    *plfpcmclklimit = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *plfpcmclkmaxval = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint8_t pcm_plfclkinv_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void pcm_plfclkinv_setf(uint8_t plfclkinv)
{
    ASSERT_ERR((((uint32_t)plfclkinv << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPHYSCNTL1_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL1_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)plfclkinv << 31));
}

__INLINE uint8_t pcm_plfpcmclklimit_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void pcm_plfpcmclklimit_setf(uint8_t plfpcmclklimit)
{
    ASSERT_ERR((((uint32_t)plfpcmclklimit << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPHYSCNTL1_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL1_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)plfpcmclklimit << 16));
}

__INLINE uint16_t pcm_plfpcmclkmaxval_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void pcm_plfpcmclkmaxval_setf(uint16_t plfpcmclkmaxval)
{
    ASSERT_ERR((((uint32_t)plfpcmclkmaxval << 0) & ~((uint32_t)0x000001FF)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPHYSCNTL1_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMPHYSCNTL1_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)plfpcmclkmaxval << 0));
}

/**
 * @brief LEAUDIOPCMPADDING register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16          PLFRSAMPPAD   0x0
 *  15:00          PLFLSAMPPAD   0x0
 * </pre>
 */
#define PCM_LEAUDIOPCMPADDING_ADDR   0x0040C030
#define PCM_LEAUDIOPCMPADDING_OFFSET 0x00000030
#define PCM_LEAUDIOPCMPADDING_INDEX  0x0000000C
#define PCM_LEAUDIOPCMPADDING_RESET  0x00000000

__INLINE uint32_t pcm_leaudiopcmpadding_get(void)
{
    return REG_PL_RD(PCM_LEAUDIOPCMPADDING_ADDR);
}

__INLINE void pcm_leaudiopcmpadding_set(uint32_t value)
{
    REG_PL_WR(PCM_LEAUDIOPCMPADDING_ADDR, value);
}

// field definitions
#define PCM_PLFRSAMPPAD_MASK   ((uint32_t)0xFFFF0000)
#define PCM_PLFRSAMPPAD_LSB    16
#define PCM_PLFRSAMPPAD_WIDTH  ((uint32_t)0x00000010)
#define PCM_PLFLSAMPPAD_MASK   ((uint32_t)0x0000FFFF)
#define PCM_PLFLSAMPPAD_LSB    0
#define PCM_PLFLSAMPPAD_WIDTH  ((uint32_t)0x00000010)

#define PCM_PLFRSAMPPAD_RST    0x0
#define PCM_PLFLSAMPPAD_RST    0x0

__INLINE void pcm_leaudiopcmpadding_pack(uint16_t plfrsamppad, uint16_t plflsamppad)
{
    ASSERT_ERR((((uint32_t)plfrsamppad << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)plflsamppad << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPADDING_ADDR,  ((uint32_t)plfrsamppad << 16) | ((uint32_t)plflsamppad << 0));
}

__INLINE void pcm_leaudiopcmpadding_unpack(uint16_t* plfrsamppad, uint16_t* plflsamppad)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPADDING_ADDR);

    *plfrsamppad = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *plflsamppad = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t pcm_plfrsamppad_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPADDING_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void pcm_plfrsamppad_setf(uint16_t plfrsamppad)
{
    ASSERT_ERR((((uint32_t)plfrsamppad << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPADDING_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMPADDING_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)plfrsamppad << 16));
}

__INLINE uint16_t pcm_plflsamppad_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPADDING_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void pcm_plflsamppad_setf(uint16_t plflsamppad)
{
    ASSERT_ERR((((uint32_t)plflsamppad << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPADDING_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMPADDING_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)plflsamppad << 0));
}

/**
 * @brief LEAUDIOPCMPLLCNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  19:00                PLFRF   0x0
 * </pre>
 */
#define PCM_LEAUDIOPCMPLLCNTL0_ADDR   0x0040C034
#define PCM_LEAUDIOPCMPLLCNTL0_OFFSET 0x00000034
#define PCM_LEAUDIOPCMPLLCNTL0_INDEX  0x0000000D
#define PCM_LEAUDIOPCMPLLCNTL0_RESET  0x00000000

__INLINE uint32_t pcm_leaudiopcmpllcntl0_get(void)
{
    return REG_PL_RD(PCM_LEAUDIOPCMPLLCNTL0_ADDR);
}

__INLINE void pcm_leaudiopcmpllcntl0_set(uint32_t value)
{
    REG_PL_WR(PCM_LEAUDIOPCMPLLCNTL0_ADDR, value);
}

// field definitions
#define PCM_PLFRF_MASK   ((uint32_t)0x000FFFFF)
#define PCM_PLFRF_LSB    0
#define PCM_PLFRF_WIDTH  ((uint32_t)0x00000014)

#define PCM_PLFRF_RST    0x0

__INLINE uint32_t pcm_plfrf_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPLLCNTL0_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000FFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void pcm_plfrf_setf(uint32_t plfrf)
{
    ASSERT_ERR((((uint32_t)plfrf << 0) & ~((uint32_t)0x000FFFFF)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPLLCNTL0_ADDR, (uint32_t)plfrf << 0);
}

/**
 * @brief LEAUDIOPCMPLLCNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:20               PLFOLC   0x0
 *  18:00                 PLFA   0x0
 * </pre>
 */
#define PCM_LEAUDIOPCMPLLCNTL1_ADDR   0x0040C038
#define PCM_LEAUDIOPCMPLLCNTL1_OFFSET 0x00000038
#define PCM_LEAUDIOPCMPLLCNTL1_INDEX  0x0000000E
#define PCM_LEAUDIOPCMPLLCNTL1_RESET  0x00000000

__INLINE uint32_t pcm_leaudiopcmpllcntl1_get(void)
{
    return REG_PL_RD(PCM_LEAUDIOPCMPLLCNTL1_ADDR);
}

__INLINE void pcm_leaudiopcmpllcntl1_set(uint32_t value)
{
    REG_PL_WR(PCM_LEAUDIOPCMPLLCNTL1_ADDR, value);
}

// field definitions
#define PCM_PLFOLC_MASK   ((uint32_t)0x7FF00000)
#define PCM_PLFOLC_LSB    20
#define PCM_PLFOLC_WIDTH  ((uint32_t)0x0000000B)
#define PCM_PLFA_MASK     ((uint32_t)0x0007FFFF)
#define PCM_PLFA_LSB      0
#define PCM_PLFA_WIDTH    ((uint32_t)0x00000013)

#define PCM_PLFOLC_RST    0x0
#define PCM_PLFA_RST      0x0

__INLINE void pcm_leaudiopcmpllcntl1_pack(uint16_t plfolc, uint32_t plfa)
{
    ASSERT_ERR((((uint32_t)plfolc << 20) & ~((uint32_t)0x7FF00000)) == 0);
    ASSERT_ERR((((uint32_t)plfa << 0) & ~((uint32_t)0x0007FFFF)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPLLCNTL1_ADDR,  ((uint32_t)plfolc << 20) | ((uint32_t)plfa << 0));
}

__INLINE void pcm_leaudiopcmpllcntl1_unpack(uint16_t* plfolc, uint32_t* plfa)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPLLCNTL1_ADDR);

    *plfolc = (localVal & ((uint32_t)0x7FF00000)) >> 20;
    *plfa = (localVal & ((uint32_t)0x0007FFFF)) >> 0;
}

__INLINE uint16_t pcm_plfolc_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPLLCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x7FF00000)) >> 20);
}

__INLINE void pcm_plfolc_setf(uint16_t plfolc)
{
    ASSERT_ERR((((uint32_t)plfolc << 20) & ~((uint32_t)0x7FF00000)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPLLCNTL1_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMPLLCNTL1_ADDR) & ~((uint32_t)0x7FF00000)) | ((uint32_t)plfolc << 20));
}

__INLINE uint32_t pcm_plfa_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPLLCNTL1_ADDR);
    return ((localVal & ((uint32_t)0x0007FFFF)) >> 0);
}

__INLINE void pcm_plfa_setf(uint32_t plfa)
{
    ASSERT_ERR((((uint32_t)plfa << 0) & ~((uint32_t)0x0007FFFF)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPLLCNTL1_ADDR, (REG_PL_RD(PCM_LEAUDIOPCMPLLCNTL1_ADDR) & ~((uint32_t)0x0007FFFF)) | ((uint32_t)plfa << 0));
}

/**
 * @brief LEAUDIOPCMPLLCNTL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  18:00                 PLFW   0x0
 * </pre>
 */
#define PCM_LEAUDIOPCMPLLCNTL2_ADDR   0x0040C03C
#define PCM_LEAUDIOPCMPLLCNTL2_OFFSET 0x0000003C
#define PCM_LEAUDIOPCMPLLCNTL2_INDEX  0x0000000F
#define PCM_LEAUDIOPCMPLLCNTL2_RESET  0x00000000

__INLINE uint32_t pcm_leaudiopcmpllcntl2_get(void)
{
    return REG_PL_RD(PCM_LEAUDIOPCMPLLCNTL2_ADDR);
}

__INLINE void pcm_leaudiopcmpllcntl2_set(uint32_t value)
{
    REG_PL_WR(PCM_LEAUDIOPCMPLLCNTL2_ADDR, value);
}

// field definitions
#define PCM_PLFW_MASK   ((uint32_t)0x0007FFFF)
#define PCM_PLFW_LSB    0
#define PCM_PLFW_WIDTH  ((uint32_t)0x00000013)

#define PCM_PLFW_RST    0x0

__INLINE uint32_t pcm_plfw_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOPCMPLLCNTL2_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0007FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void pcm_plfw_setf(uint32_t plfw)
{
    ASSERT_ERR((((uint32_t)plfw << 0) & ~((uint32_t)0x0007FFFF)) == 0);
    REG_PL_WR(PCM_LEAUDIOPCMPLLCNTL2_ADDR, (uint32_t)plfw << 0);
}

/**
 * @brief LEAUDIOIDCCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  06:04            SPICLKCNT   0x0
 *  01:00              IDCMODE   0x0
 * </pre>
 */
#define PCM_LEAUDIOIDCCNTL_ADDR   0x0040C040
#define PCM_LEAUDIOIDCCNTL_OFFSET 0x00000040
#define PCM_LEAUDIOIDCCNTL_INDEX  0x00000010
#define PCM_LEAUDIOIDCCNTL_RESET  0x00000000

__INLINE uint32_t pcm_leaudioidccntl_get(void)
{
    return REG_PL_RD(PCM_LEAUDIOIDCCNTL_ADDR);
}

__INLINE void pcm_leaudioidccntl_set(uint32_t value)
{
    REG_PL_WR(PCM_LEAUDIOIDCCNTL_ADDR, value);
}

// field definitions
#define PCM_SPICLKCNT_MASK   ((uint32_t)0x00000070)
#define PCM_SPICLKCNT_LSB    4
#define PCM_SPICLKCNT_WIDTH  ((uint32_t)0x00000003)
#define PCM_IDCMODE_MASK     ((uint32_t)0x00000003)
#define PCM_IDCMODE_LSB      0
#define PCM_IDCMODE_WIDTH    ((uint32_t)0x00000002)

#define PCM_SPICLKCNT_RST    0x0
#define PCM_IDCMODE_RST      0x0

__INLINE void pcm_leaudioidccntl_pack(uint8_t spiclkcnt, uint8_t idcmode)
{
    ASSERT_ERR((((uint32_t)spiclkcnt << 4) & ~((uint32_t)0x00000070)) == 0);
    ASSERT_ERR((((uint32_t)idcmode << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_PL_WR(PCM_LEAUDIOIDCCNTL_ADDR,  ((uint32_t)spiclkcnt << 4) | ((uint32_t)idcmode << 0));
}

__INLINE void pcm_leaudioidccntl_unpack(uint8_t* spiclkcnt, uint8_t* idcmode)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOIDCCNTL_ADDR);

    *spiclkcnt = (localVal & ((uint32_t)0x00000070)) >> 4;
    *idcmode = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t pcm_spiclkcnt_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOIDCCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000070)) >> 4);
}

__INLINE void pcm_spiclkcnt_setf(uint8_t spiclkcnt)
{
    ASSERT_ERR((((uint32_t)spiclkcnt << 4) & ~((uint32_t)0x00000070)) == 0);
    REG_PL_WR(PCM_LEAUDIOIDCCNTL_ADDR, (REG_PL_RD(PCM_LEAUDIOIDCCNTL_ADDR) & ~((uint32_t)0x00000070)) | ((uint32_t)spiclkcnt << 4));
}

__INLINE uint8_t pcm_idcmode_getf(void)
{
    uint32_t localVal = REG_PL_RD(PCM_LEAUDIOIDCCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

__INLINE void pcm_idcmode_setf(uint8_t idcmode)
{
    ASSERT_ERR((((uint32_t)idcmode << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_PL_WR(PCM_LEAUDIOIDCCNTL_ADDR, (REG_PL_RD(PCM_LEAUDIOIDCCNTL_ADDR) & ~((uint32_t)0x00000003)) | ((uint32_t)idcmode << 0));
}


#endif // _REG_PCM_H_

