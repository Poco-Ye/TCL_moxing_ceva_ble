#ifndef _REG_PLF_H_
#define _REG_PLF_H_

#include <stdint.h>
#include "_reg_plf.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_PLF_COUNT 17

#define REG_PLF_DECODING_MASK 0x0000007F

/**
 * @brief VERSION register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24              IP_TYPE   0x0
 *  23:20             CPU_TYPE   0x0
 *  19:16             PLF_TYPE   0x0
 *  15:08              RF_TYPE   0x0
 *  07:00              VERSION   0x0
 * </pre>
 */
#define PLF_VERSION_ADDR   0x00500000
#define PLF_VERSION_OFFSET 0x00000000
#define PLF_VERSION_INDEX  0x00000000
#define PLF_VERSION_RESET  0x00000000

__INLINE uint32_t plf_version_get(void)
{
    return REG_PL_RD(PLF_VERSION_ADDR);
}

// field definitions
#define PLF_IP_TYPE_MASK    ((uint32_t)0xFF000000)
#define PLF_IP_TYPE_LSB     24
#define PLF_IP_TYPE_WIDTH   ((uint32_t)0x00000008)
#define PLF_CPU_TYPE_MASK   ((uint32_t)0x00F00000)
#define PLF_CPU_TYPE_LSB    20
#define PLF_CPU_TYPE_WIDTH  ((uint32_t)0x00000004)
#define PLF_PLF_TYPE_MASK   ((uint32_t)0x000F0000)
#define PLF_PLF_TYPE_LSB    16
#define PLF_PLF_TYPE_WIDTH  ((uint32_t)0x00000004)
#define PLF_RF_TYPE_MASK    ((uint32_t)0x0000FF00)
#define PLF_RF_TYPE_LSB     8
#define PLF_RF_TYPE_WIDTH   ((uint32_t)0x00000008)
#define PLF_VERSION_MASK    ((uint32_t)0x000000FF)
#define PLF_VERSION_LSB     0
#define PLF_VERSION_WIDTH   ((uint32_t)0x00000008)

#define PLF_IP_TYPE_RST     0x0
#define PLF_CPU_TYPE_RST    0x0
#define PLF_PLF_TYPE_RST    0x0
#define PLF_RF_TYPE_RST     0x0
#define PLF_VERSION_RST     0x0

__INLINE void plf_version_unpack(uint8_t* iptype, uint8_t* cputype, uint8_t* plftype, uint8_t* rftype, uint8_t* version)
{
    uint32_t localVal = REG_PL_RD(PLF_VERSION_ADDR);

    *iptype = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *cputype = (localVal & ((uint32_t)0x00F00000)) >> 20;
    *plftype = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *rftype = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *version = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t plf_version_ip_type_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_VERSION_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE uint8_t plf_version_cpu_type_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_VERSION_ADDR);
    return ((localVal & ((uint32_t)0x00F00000)) >> 20);
}

__INLINE uint8_t plf_version_plf_type_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_VERSION_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE uint8_t plf_version_rf_type_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_VERSION_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE uint8_t plf_version_version_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_VERSION_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

/**
 * @brief RELEASEDATE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:26                 YEAR   0x0
 *  25:22                MONTH   0x0
 *  21:17                  DAY   0x0
 *  16:12                 HOUR   0x0
 *  11:06                  MIN   0x0
 *  05:00                  SEC   0x0
 * </pre>
 */
#define PLF_RELEASEDATE_ADDR   0x00500004
#define PLF_RELEASEDATE_OFFSET 0x00000004
#define PLF_RELEASEDATE_INDEX  0x00000001
#define PLF_RELEASEDATE_RESET  0x00000000

__INLINE uint32_t plf_releasedate_get(void)
{
    return REG_PL_RD(PLF_RELEASEDATE_ADDR);
}

__INLINE void plf_releasedate_set(uint32_t value)
{
    REG_PL_WR(PLF_RELEASEDATE_ADDR, value);
}

// field definitions
#define PLF_YEAR_MASK    ((uint32_t)0x3C000000)
#define PLF_YEAR_LSB     26
#define PLF_YEAR_WIDTH   ((uint32_t)0x00000004)
#define PLF_MONTH_MASK   ((uint32_t)0x03C00000)
#define PLF_MONTH_LSB    22
#define PLF_MONTH_WIDTH  ((uint32_t)0x00000004)
#define PLF_DAY_MASK     ((uint32_t)0x003E0000)
#define PLF_DAY_LSB      17
#define PLF_DAY_WIDTH    ((uint32_t)0x00000005)
#define PLF_HOUR_MASK    ((uint32_t)0x0001F000)
#define PLF_HOUR_LSB     12
#define PLF_HOUR_WIDTH   ((uint32_t)0x00000005)
#define PLF_MIN_MASK     ((uint32_t)0x00000FC0)
#define PLF_MIN_LSB      6
#define PLF_MIN_WIDTH    ((uint32_t)0x00000006)
#define PLF_SEC_MASK     ((uint32_t)0x0000003F)
#define PLF_SEC_LSB      0
#define PLF_SEC_WIDTH    ((uint32_t)0x00000006)

#define PLF_YEAR_RST     0x0
#define PLF_MONTH_RST    0x0
#define PLF_DAY_RST      0x0
#define PLF_HOUR_RST     0x0
#define PLF_MIN_RST      0x0
#define PLF_SEC_RST      0x0

__INLINE void plf_releasedate_unpack(uint8_t* year, uint8_t* month, uint8_t* day, uint8_t* hour, uint8_t* min, uint8_t* sec)
{
    uint32_t localVal = REG_PL_RD(PLF_RELEASEDATE_ADDR);

    *year = (localVal & ((uint32_t)0x3C000000)) >> 26;
    *month = (localVal & ((uint32_t)0x03C00000)) >> 22;
    *day = (localVal & ((uint32_t)0x003E0000)) >> 17;
    *hour = (localVal & ((uint32_t)0x0001F000)) >> 12;
    *min = (localVal & ((uint32_t)0x00000FC0)) >> 6;
    *sec = (localVal & ((uint32_t)0x0000003F)) >> 0;
}

__INLINE uint8_t plf_releasedate_year_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_RELEASEDATE_ADDR);
    return ((localVal & ((uint32_t)0x3C000000)) >> 26);
}

__INLINE uint8_t plf_releasedate_month_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_RELEASEDATE_ADDR);
    return ((localVal & ((uint32_t)0x03C00000)) >> 22);
}

__INLINE uint8_t plf_releasedate_day_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_RELEASEDATE_ADDR);
    return ((localVal & ((uint32_t)0x003E0000)) >> 17);
}

__INLINE uint8_t plf_releasedate_hour_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_RELEASEDATE_ADDR);
    return ((localVal & ((uint32_t)0x0001F000)) >> 12);
}

__INLINE uint8_t plf_releasedate_min_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_RELEASEDATE_ADDR);
    return ((localVal & ((uint32_t)0x00000FC0)) >> 6);
}

__INLINE uint8_t plf_releasedate_sec_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_RELEASEDATE_ADDR);
    return ((localVal & ((uint32_t)0x0000003F)) >> 0);
}

/**
 * @brief DBG register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24            TEST_SEL3   0x0
 *  23:16            TEST_SEL2   0x0
 *  15:08            TEST_SEL1   0x0
 *  07:00            TEST_SEL0   0x0
 * </pre>
 */
#define PLF_DBG_ADDR   0x00500008
#define PLF_DBG_OFFSET 0x00000008
#define PLF_DBG_INDEX  0x00000002
#define PLF_DBG_RESET  0x00000000

__INLINE uint32_t plf_dbg_get(void)
{
    return REG_PL_RD(PLF_DBG_ADDR);
}

__INLINE void plf_dbg_set(uint32_t value)
{
    REG_PL_WR(PLF_DBG_ADDR, value);
}

// field definitions
#define PLF_TEST_SEL3_MASK   ((uint32_t)0xFF000000)
#define PLF_TEST_SEL3_LSB    24
#define PLF_TEST_SEL3_WIDTH  ((uint32_t)0x00000008)
#define PLF_TEST_SEL2_MASK   ((uint32_t)0x00FF0000)
#define PLF_TEST_SEL2_LSB    16
#define PLF_TEST_SEL2_WIDTH  ((uint32_t)0x00000008)
#define PLF_TEST_SEL1_MASK   ((uint32_t)0x0000FF00)
#define PLF_TEST_SEL1_LSB    8
#define PLF_TEST_SEL1_WIDTH  ((uint32_t)0x00000008)
#define PLF_TEST_SEL0_MASK   ((uint32_t)0x000000FF)
#define PLF_TEST_SEL0_LSB    0
#define PLF_TEST_SEL0_WIDTH  ((uint32_t)0x00000008)

#define PLF_TEST_SEL3_RST    0x0
#define PLF_TEST_SEL2_RST    0x0
#define PLF_TEST_SEL1_RST    0x0
#define PLF_TEST_SEL0_RST    0x0

__INLINE void plf_dbg_pack(uint8_t testsel3, uint8_t testsel2, uint8_t testsel1, uint8_t testsel0)
{
    ASSERT_ERR((((uint32_t)testsel3 << 24) & ~((uint32_t)0xFF000000)) == 0);
    ASSERT_ERR((((uint32_t)testsel2 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    ASSERT_ERR((((uint32_t)testsel1 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint32_t)testsel0 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(PLF_DBG_ADDR,  ((uint32_t)testsel3 << 24) | ((uint32_t)testsel2 << 16) | ((uint32_t)testsel1 << 8) | ((uint32_t)testsel0 << 0));
}

__INLINE void plf_dbg_unpack(uint8_t* testsel3, uint8_t* testsel2, uint8_t* testsel1, uint8_t* testsel0)
{
    uint32_t localVal = REG_PL_RD(PLF_DBG_ADDR);

    *testsel3 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *testsel2 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *testsel1 = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *testsel0 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t plf_dbg_test_sel3_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_DBG_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void plf_dbg_test_sel3_setf(uint8_t testsel3)
{
    ASSERT_ERR((((uint32_t)testsel3 << 24) & ~((uint32_t)0xFF000000)) == 0);
    REG_PL_WR(PLF_DBG_ADDR, (REG_PL_RD(PLF_DBG_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)testsel3 << 24));
}

__INLINE uint8_t plf_dbg_test_sel2_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_DBG_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void plf_dbg_test_sel2_setf(uint8_t testsel2)
{
    ASSERT_ERR((((uint32_t)testsel2 << 16) & ~((uint32_t)0x00FF0000)) == 0);
    REG_PL_WR(PLF_DBG_ADDR, (REG_PL_RD(PLF_DBG_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)testsel2 << 16));
}

__INLINE uint8_t plf_dbg_test_sel1_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_DBG_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void plf_dbg_test_sel1_setf(uint8_t testsel1)
{
    ASSERT_ERR((((uint32_t)testsel1 << 8) & ~((uint32_t)0x0000FF00)) == 0);
    REG_PL_WR(PLF_DBG_ADDR, (REG_PL_RD(PLF_DBG_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)testsel1 << 8));
}

__INLINE uint8_t plf_dbg_test_sel0_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_DBG_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void plf_dbg_test_sel0_setf(uint8_t testsel0)
{
    ASSERT_ERR((((uint32_t)testsel0 << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(PLF_DBG_ADDR, (REG_PL_RD(PLF_DBG_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)testsel0 << 0));
}

/**
 * @brief RF_DATA_GEN register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31               ENABLE   0
 *  14:00                DEPTH   0x0
 * </pre>
 */
#define PLF_RF_DATA_GEN_ADDR   0x0050000C
#define PLF_RF_DATA_GEN_OFFSET 0x0000000C
#define PLF_RF_DATA_GEN_INDEX  0x00000003
#define PLF_RF_DATA_GEN_RESET  0x00000000

__INLINE uint32_t plf_rf_data_gen_get(void)
{
    return REG_PL_RD(PLF_RF_DATA_GEN_ADDR);
}

__INLINE void plf_rf_data_gen_set(uint32_t value)
{
    REG_PL_WR(PLF_RF_DATA_GEN_ADDR, value);
}

// field definitions
#define PLF_ENABLE_BIT    ((uint32_t)0x80000000)
#define PLF_ENABLE_POS    31
#define PLF_DEPTH_MASK    ((uint32_t)0x00007FFF)
#define PLF_DEPTH_LSB     0
#define PLF_DEPTH_WIDTH   ((uint32_t)0x0000000F)

#define PLF_ENABLE_RST    0x0
#define PLF_DEPTH_RST     0x0

__INLINE void plf_rf_data_gen_pack(uint8_t enable, uint16_t depth)
{
    ASSERT_ERR((((uint32_t)enable << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)depth << 0) & ~((uint32_t)0x00007FFF)) == 0);
    REG_PL_WR(PLF_RF_DATA_GEN_ADDR,  ((uint32_t)enable << 31) | ((uint32_t)depth << 0));
}

__INLINE void plf_rf_data_gen_unpack(uint8_t* enable, uint16_t* depth)
{
    uint32_t localVal = REG_PL_RD(PLF_RF_DATA_GEN_ADDR);

    *enable = (localVal & ((uint32_t)0x80000000)) >> 31;
    *depth = (localVal & ((uint32_t)0x00007FFF)) >> 0;
}

__INLINE uint8_t plf_rf_data_gen_enable_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_RF_DATA_GEN_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void plf_rf_data_gen_enable_setf(uint8_t enable)
{
    ASSERT_ERR((((uint32_t)enable << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_PL_WR(PLF_RF_DATA_GEN_ADDR, (REG_PL_RD(PLF_RF_DATA_GEN_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)enable << 31));
}

__INLINE uint16_t plf_rf_data_gen_depth_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_RF_DATA_GEN_ADDR);
    return ((localVal & ((uint32_t)0x00007FFF)) >> 0);
}

__INLINE void plf_rf_data_gen_depth_setf(uint16_t depth)
{
    ASSERT_ERR((((uint32_t)depth << 0) & ~((uint32_t)0x00007FFF)) == 0);
    REG_PL_WR(PLF_RF_DATA_GEN_ADDR, (REG_PL_RD(PLF_RF_DATA_GEN_ADDR) & ~((uint32_t)0x00007FFF)) | ((uint32_t)depth << 0));
}

/**
 * @brief RF_ID register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  19:00                   ID   0x0
 * </pre>
 */
#define PLF_RF_ID_ADDR   0x00500010
#define PLF_RF_ID_OFFSET 0x00000010
#define PLF_RF_ID_INDEX  0x00000004
#define PLF_RF_ID_RESET  0x00000000

__INLINE uint32_t plf_rf_id_get(void)
{
    return REG_PL_RD(PLF_RF_ID_ADDR);
}

__INLINE void plf_rf_id_set(uint32_t value)
{
    REG_PL_WR(PLF_RF_ID_ADDR, value);
}

// field definitions
#define PLF_ID_MASK   ((uint32_t)0x000FFFFF)
#define PLF_ID_LSB    0
#define PLF_ID_WIDTH  ((uint32_t)0x00000014)

#define PLF_ID_RST    0x0

__INLINE uint32_t plf_rf_id_id_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_RF_ID_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000FFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief UART_SWAP register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00            UART_SWAP   0
 * </pre>
 */
#define PLF_UART_SWAP_ADDR   0x00500014
#define PLF_UART_SWAP_OFFSET 0x00000014
#define PLF_UART_SWAP_INDEX  0x00000005
#define PLF_UART_SWAP_RESET  0x00000000

__INLINE uint32_t plf_uart_swap_get(void)
{
    return REG_PL_RD(PLF_UART_SWAP_ADDR);
}

__INLINE void plf_uart_swap_set(uint32_t value)
{
    REG_PL_WR(PLF_UART_SWAP_ADDR, value);
}

// field definitions
#define PLF_UART_SWAP_BIT    ((uint32_t)0x00000001)
#define PLF_UART_SWAP_POS    0

#define PLF_UART_SWAP_RST    0x0

__INLINE uint8_t plf_uart_swap_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_UART_SWAP_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x00000001)) == 0);
    return (localVal >> 0);
}

__INLINE void plf_uart_swap_setf(uint8_t uartswap)
{
    ASSERT_ERR((((uint32_t)uartswap << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(PLF_UART_SWAP_ADDR, (uint32_t)uartswap << 0);
}

/**
 * @brief LA register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31         RX_CAPT_EDGE   0
 *     00            LA_ENABLE   0
 * </pre>
 */
#define PLF_LA_ADDR   0x00500018
#define PLF_LA_OFFSET 0x00000018
#define PLF_LA_INDEX  0x00000006
#define PLF_LA_RESET  0x00000000

__INLINE uint32_t plf_la_get(void)
{
    return REG_PL_RD(PLF_LA_ADDR);
}

__INLINE void plf_la_set(uint32_t value)
{
    REG_PL_WR(PLF_LA_ADDR, value);
}

// field definitions
#define PLF_RX_CAPT_EDGE_BIT    ((uint32_t)0x80000000)
#define PLF_RX_CAPT_EDGE_POS    31
#define PLF_LA_ENABLE_BIT       ((uint32_t)0x00000001)
#define PLF_LA_ENABLE_POS       0

#define PLF_RX_CAPT_EDGE_RST    0x0
#define PLF_LA_ENABLE_RST       0x0

__INLINE void plf_la_pack(uint8_t rxcaptedge, uint8_t laenable)
{
    ASSERT_ERR((((uint32_t)rxcaptedge << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)laenable << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(PLF_LA_ADDR,  ((uint32_t)rxcaptedge << 31) | ((uint32_t)laenable << 0));
}

__INLINE void plf_la_unpack(uint8_t* rxcaptedge, uint8_t* laenable)
{
    uint32_t localVal = REG_PL_RD(PLF_LA_ADDR);

    *rxcaptedge = (localVal & ((uint32_t)0x80000000)) >> 31;
    *laenable = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t plf_la_rx_capt_edge_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_LA_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void plf_la_rx_capt_edge_setf(uint8_t rxcaptedge)
{
    ASSERT_ERR((((uint32_t)rxcaptedge << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_PL_WR(PLF_LA_ADDR, (REG_PL_RD(PLF_LA_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)rxcaptedge << 31));
}

__INLINE uint8_t plf_la_la_enable_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_LA_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void plf_la_la_enable_setf(uint8_t laenable)
{
    ASSERT_ERR((((uint32_t)laenable << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(PLF_LA_ADDR, (REG_PL_RD(PLF_LA_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)laenable << 0));
}

/**
 * @brief IDCSEL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  03:00              IDC_SEL   0x0
 * </pre>
 */
#define PLF_IDCSEL_ADDR   0x0050001C
#define PLF_IDCSEL_OFFSET 0x0000001C
#define PLF_IDCSEL_INDEX  0x00000007
#define PLF_IDCSEL_RESET  0x00000000

__INLINE uint32_t plf_idcsel_get(void)
{
    return REG_PL_RD(PLF_IDCSEL_ADDR);
}

__INLINE void plf_idcsel_set(uint32_t value)
{
    REG_PL_WR(PLF_IDCSEL_ADDR, value);
}

// field definitions
#define PLF_IDC_SEL_MASK   ((uint32_t)0x0000000F)
#define PLF_IDC_SEL_LSB    0
#define PLF_IDC_SEL_WIDTH  ((uint32_t)0x00000004)

#define PLF_IDC_SEL_RST    0x0

__INLINE uint8_t plf_idcsel_idc_sel_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_IDCSEL_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000000F)) == 0);
    return (localVal >> 0);
}

__INLINE void plf_idcsel_idc_sel_setf(uint8_t idcsel)
{
    ASSERT_ERR((((uint32_t)idcsel << 0) & ~((uint32_t)0x0000000F)) == 0);
    REG_PL_WR(PLF_IDCSEL_ADDR, (uint32_t)idcsel << 0);
}

/**
 * @brief SWRESET register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00              swreset   0
 * </pre>
 */
#define PLF_SWRESET_ADDR   0x00500020
#define PLF_SWRESET_OFFSET 0x00000020
#define PLF_SWRESET_INDEX  0x00000008
#define PLF_SWRESET_RESET  0x00000000

__INLINE uint32_t plf_swreset_get(void)
{
    return REG_PL_RD(PLF_SWRESET_ADDR);
}

__INLINE void plf_swreset_set(uint32_t value)
{
    REG_PL_WR(PLF_SWRESET_ADDR, value);
}

// field definitions
#define PLF_SWRESET_BIT    ((uint32_t)0x00000001)
#define PLF_SWRESET_POS    0

#define PLF_SWRESET_RST    0x0

__INLINE uint8_t plf_swreset_swreset_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_SWRESET_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x00000001)) == 0);
    return (localVal >> 0);
}

__INLINE void plf_swreset_swreset_setf(uint8_t swreset)
{
    ASSERT_ERR((((uint32_t)swreset << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(PLF_SWRESET_ADDR, (uint32_t)swreset << 0);
}

/**
 * @brief SWTRIGGER register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00           sw_trigger   0
 * </pre>
 */
#define PLF_SWTRIGGER_ADDR   0x00500024
#define PLF_SWTRIGGER_OFFSET 0x00000024
#define PLF_SWTRIGGER_INDEX  0x00000009
#define PLF_SWTRIGGER_RESET  0x00000000

__INLINE uint32_t plf_swtrigger_get(void)
{
    return REG_PL_RD(PLF_SWTRIGGER_ADDR);
}

__INLINE void plf_swtrigger_set(uint32_t value)
{
    REG_PL_WR(PLF_SWTRIGGER_ADDR, value);
}

// field definitions
#define PLF_SW_TRIGGER_BIT    ((uint32_t)0x00000001)
#define PLF_SW_TRIGGER_POS    0

#define PLF_SW_TRIGGER_RST    0x0

__INLINE uint8_t plf_swtrigger_sw_trigger_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_SWTRIGGER_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x00000001)) == 0);
    return (localVal >> 0);
}

__INLINE void plf_swtrigger_sw_trigger_setf(uint8_t swtrigger)
{
    ASSERT_ERR((((uint32_t)swtrigger << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(PLF_SWTRIGGER_ADDR, (uint32_t)swtrigger << 0);
}

/**
 * @brief SP6_RECONFIG register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00         sp6_reconfig   0
 * </pre>
 */
#define PLF_SP6_RECONFIG_ADDR   0x00500028
#define PLF_SP6_RECONFIG_OFFSET 0x00000028
#define PLF_SP6_RECONFIG_INDEX  0x0000000A
#define PLF_SP6_RECONFIG_RESET  0x00000000

__INLINE uint32_t plf_sp6_reconfig_get(void)
{
    return REG_PL_RD(PLF_SP6_RECONFIG_ADDR);
}

__INLINE void plf_sp6_reconfig_set(uint32_t value)
{
    REG_PL_WR(PLF_SP6_RECONFIG_ADDR, value);
}

// field definitions
#define PLF_SP_6_RECONFIG_BIT    ((uint32_t)0x00000001)
#define PLF_SP_6_RECONFIG_POS    0

#define PLF_SP_6_RECONFIG_RST    0x0

__INLINE uint8_t plf_sp6_reconfig_sp_6_reconfig_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_SP6_RECONFIG_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x00000001)) == 0);
    return (localVal >> 0);
}

__INLINE void plf_sp6_reconfig_sp_6_reconfig_setf(uint8_t sp6reconfig)
{
    ASSERT_ERR((((uint32_t)sp6reconfig << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(PLF_SP6_RECONFIG_ADDR, (uint32_t)sp6reconfig << 0);
}

/**
 * @brief SP6_RECONFIG_ADD12 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00     SP6_RECONF_GNL12   0x0
 * </pre>
 */
#define PLF_SP6_RECONFIG_ADD12_ADDR   0x0050002C
#define PLF_SP6_RECONFIG_ADD12_OFFSET 0x0000002C
#define PLF_SP6_RECONFIG_ADD12_INDEX  0x0000000B
#define PLF_SP6_RECONFIG_ADD12_RESET  0x00000000

__INLINE uint32_t plf_sp6_reconfig_add12_get(void)
{
    return REG_PL_RD(PLF_SP6_RECONFIG_ADD12_ADDR);
}

__INLINE void plf_sp6_reconfig_add12_set(uint32_t value)
{
    REG_PL_WR(PLF_SP6_RECONFIG_ADD12_ADDR, value);
}

// field definitions
#define PLF_SP6_RECONF_GNL12_MASK   ((uint32_t)0xFFFFFFFF)
#define PLF_SP6_RECONF_GNL12_LSB    0
#define PLF_SP6_RECONF_GNL12_WIDTH  ((uint32_t)0x00000020)

#define PLF_SP6_RECONF_GNL12_RST    0x0

__INLINE uint32_t plf_sp6_reconfig_add12_sp6_reconf_gnl12_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_SP6_RECONFIG_ADD12_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void plf_sp6_reconfig_add12_sp6_reconf_gnl12_setf(uint32_t sp6reconfgnl12)
{
    ASSERT_ERR((((uint32_t)sp6reconfgnl12 << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(PLF_SP6_RECONFIG_ADD12_ADDR, (uint32_t)sp6reconfgnl12 << 0);
}

/**
 * @brief SP6_RECONFIG_ADD23 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00     SP6_RECONF_GNL23   0x0
 * </pre>
 */
#define PLF_SP6_RECONFIG_ADD23_ADDR   0x00500030
#define PLF_SP6_RECONFIG_ADD23_OFFSET 0x00000030
#define PLF_SP6_RECONFIG_ADD23_INDEX  0x0000000C
#define PLF_SP6_RECONFIG_ADD23_RESET  0x00000000

__INLINE uint32_t plf_sp6_reconfig_add23_get(void)
{
    return REG_PL_RD(PLF_SP6_RECONFIG_ADD23_ADDR);
}

__INLINE void plf_sp6_reconfig_add23_set(uint32_t value)
{
    REG_PL_WR(PLF_SP6_RECONFIG_ADD23_ADDR, value);
}

// field definitions
#define PLF_SP6_RECONF_GNL23_MASK   ((uint32_t)0xFFFFFFFF)
#define PLF_SP6_RECONF_GNL23_LSB    0
#define PLF_SP6_RECONF_GNL23_WIDTH  ((uint32_t)0x00000020)

#define PLF_SP6_RECONF_GNL23_RST    0x0

__INLINE uint32_t plf_sp6_reconfig_add23_sp6_reconf_gnl23_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_SP6_RECONFIG_ADD23_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void plf_sp6_reconfig_add23_sp6_reconf_gnl23_setf(uint32_t sp6reconfgnl23)
{
    ASSERT_ERR((((uint32_t)sp6reconfgnl23 << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(PLF_SP6_RECONFIG_ADD23_ADDR, (uint32_t)sp6reconfgnl23 << 0);
}

/**
 * @brief CODEC_CONF register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31             CODEC_GO   0
 *     30           CODEC_COMP   1
 *     23         CODECSRC_SEL   0
 *  15:00          CODEC_VALUE   0x0
 * </pre>
 */
#define PLF_CODEC_CONF_ADDR   0x00500034
#define PLF_CODEC_CONF_OFFSET 0x00000034
#define PLF_CODEC_CONF_INDEX  0x0000000D
#define PLF_CODEC_CONF_RESET  0x40000000

__INLINE uint32_t plf_codec_conf_get(void)
{
    return REG_PL_RD(PLF_CODEC_CONF_ADDR);
}

__INLINE void plf_codec_conf_set(uint32_t value)
{
    REG_PL_WR(PLF_CODEC_CONF_ADDR, value);
}

// field definitions
#define PLF_CODEC_GO_BIT        ((uint32_t)0x80000000)
#define PLF_CODEC_GO_POS        31
#define PLF_CODEC_COMP_BIT      ((uint32_t)0x40000000)
#define PLF_CODEC_COMP_POS      30
#define PLF_CODECSRC_SEL_BIT    ((uint32_t)0x00800000)
#define PLF_CODECSRC_SEL_POS    23
#define PLF_CODEC_VALUE_MASK    ((uint32_t)0x0000FFFF)
#define PLF_CODEC_VALUE_LSB     0
#define PLF_CODEC_VALUE_WIDTH   ((uint32_t)0x00000010)

#define PLF_CODEC_GO_RST        0x0
#define PLF_CODEC_COMP_RST      0x1
#define PLF_CODECSRC_SEL_RST    0x0
#define PLF_CODEC_VALUE_RST     0x0

__INLINE void plf_codec_conf_pack(uint8_t codecgo, uint8_t codecsrcsel, uint16_t codecvalue)
{
    ASSERT_ERR((((uint32_t)codecgo << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)codecsrcsel << 23) & ~((uint32_t)0x00800000)) == 0);
    ASSERT_ERR((((uint32_t)codecvalue << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_PL_WR(PLF_CODEC_CONF_ADDR,  ((uint32_t)codecgo << 31) | ((uint32_t)codecsrcsel << 23) | ((uint32_t)codecvalue << 0));
}

__INLINE void plf_codec_conf_unpack(uint8_t* codecgo, uint8_t* codeccomp, uint8_t* codecsrcsel, uint16_t* codecvalue)
{
    uint32_t localVal = REG_PL_RD(PLF_CODEC_CONF_ADDR);

    *codecgo = (localVal & ((uint32_t)0x80000000)) >> 31;
    *codeccomp = (localVal & ((uint32_t)0x40000000)) >> 30;
    *codecsrcsel = (localVal & ((uint32_t)0x00800000)) >> 23;
    *codecvalue = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint8_t plf_codec_conf_codec_go_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_CODEC_CONF_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void plf_codec_conf_codec_go_setf(uint8_t codecgo)
{
    ASSERT_ERR((((uint32_t)codecgo << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_PL_WR(PLF_CODEC_CONF_ADDR, (REG_PL_RD(PLF_CODEC_CONF_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)codecgo << 31));
}

__INLINE uint8_t plf_codec_conf_codec_comp_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_CODEC_CONF_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE uint8_t plf_codec_conf_codecsrc_sel_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_CODEC_CONF_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE void plf_codec_conf_codecsrc_sel_setf(uint8_t codecsrcsel)
{
    ASSERT_ERR((((uint32_t)codecsrcsel << 23) & ~((uint32_t)0x00800000)) == 0);
    REG_PL_WR(PLF_CODEC_CONF_ADDR, (REG_PL_RD(PLF_CODEC_CONF_ADDR) & ~((uint32_t)0x00800000)) | ((uint32_t)codecsrcsel << 23));
}

__INLINE uint16_t plf_codec_conf_codec_value_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_CODEC_CONF_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void plf_codec_conf_codec_value_setf(uint16_t codecvalue)
{
    ASSERT_ERR((((uint32_t)codecvalue << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_PL_WR(PLF_CODEC_CONF_ADDR, (REG_PL_RD(PLF_CODEC_CONF_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)codecvalue << 0));
}

/**
 * @brief RF_SEL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00       RF_VERSION_SEL   0
 * </pre>
 */
#define PLF_RF_SEL_ADDR   0x00500038
#define PLF_RF_SEL_OFFSET 0x00000038
#define PLF_RF_SEL_INDEX  0x0000000E
#define PLF_RF_SEL_RESET  0x00000000

__INLINE uint32_t plf_rf_sel_get(void)
{
    return REG_PL_RD(PLF_RF_SEL_ADDR);
}

__INLINE void plf_rf_sel_set(uint32_t value)
{
    REG_PL_WR(PLF_RF_SEL_ADDR, value);
}

// field definitions
#define PLF_RF_VERSION_SEL_BIT    ((uint32_t)0x00000001)
#define PLF_RF_VERSION_SEL_POS    0

#define PLF_RF_VERSION_SEL_RST    0x0

__INLINE uint8_t plf_rf_sel_rf_version_sel_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_RF_SEL_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x00000001)) == 0);
    return (localVal >> 0);
}

__INLINE void plf_rf_sel_rf_version_sel_setf(uint8_t rfversionsel)
{
    ASSERT_ERR((((uint32_t)rfversionsel << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(PLF_RF_SEL_ADDR, (uint32_t)rfversionsel << 0);
}

/**
 * @brief CODEC_SYSCLK_CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31            CODEC_RST   0
 *     30      CODEC_SYSCLK_EN   0
 *  07:00   CODEC_SYSCLK_PERIOD   0x0
 * </pre>
 */
#define PLF_CODEC_SYSCLK_CNTL_ADDR   0x0050003C
#define PLF_CODEC_SYSCLK_CNTL_OFFSET 0x0000003C
#define PLF_CODEC_SYSCLK_CNTL_INDEX  0x0000000F
#define PLF_CODEC_SYSCLK_CNTL_RESET  0x00000000

__INLINE uint32_t plf_codec_sysclk_cntl_get(void)
{
    return REG_PL_RD(PLF_CODEC_SYSCLK_CNTL_ADDR);
}

__INLINE void plf_codec_sysclk_cntl_set(uint32_t value)
{
    REG_PL_WR(PLF_CODEC_SYSCLK_CNTL_ADDR, value);
}

// field definitions
#define PLF_CODEC_RST_BIT              ((uint32_t)0x80000000)
#define PLF_CODEC_RST_POS              31
#define PLF_CODEC_SYSCLK_EN_BIT        ((uint32_t)0x40000000)
#define PLF_CODEC_SYSCLK_EN_POS        30
#define PLF_CODEC_SYSCLK_PERIOD_MASK   ((uint32_t)0x000000FF)
#define PLF_CODEC_SYSCLK_PERIOD_LSB    0
#define PLF_CODEC_SYSCLK_PERIOD_WIDTH  ((uint32_t)0x00000008)

#define PLF_CODEC_RST_RST              0x0
#define PLF_CODEC_SYSCLK_EN_RST        0x0
#define PLF_CODEC_SYSCLK_PERIOD_RST    0x0

__INLINE void plf_codec_sysclk_cntl_pack(uint8_t codecrst, uint8_t codecsysclken, uint8_t codecsysclkperiod)
{
    ASSERT_ERR((((uint32_t)codecrst << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)codecsysclken << 30) & ~((uint32_t)0x40000000)) == 0);
    ASSERT_ERR((((uint32_t)codecsysclkperiod << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(PLF_CODEC_SYSCLK_CNTL_ADDR,  ((uint32_t)codecrst << 31) | ((uint32_t)codecsysclken << 30) | ((uint32_t)codecsysclkperiod << 0));
}

__INLINE void plf_codec_sysclk_cntl_unpack(uint8_t* codecrst, uint8_t* codecsysclken, uint8_t* codecsysclkperiod)
{
    uint32_t localVal = REG_PL_RD(PLF_CODEC_SYSCLK_CNTL_ADDR);

    *codecrst = (localVal & ((uint32_t)0x80000000)) >> 31;
    *codecsysclken = (localVal & ((uint32_t)0x40000000)) >> 30;
    *codecsysclkperiod = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t plf_codec_sysclk_cntl_codec_rst_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_CODEC_SYSCLK_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void plf_codec_sysclk_cntl_codec_rst_setf(uint8_t codecrst)
{
    ASSERT_ERR((((uint32_t)codecrst << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_PL_WR(PLF_CODEC_SYSCLK_CNTL_ADDR, (REG_PL_RD(PLF_CODEC_SYSCLK_CNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)codecrst << 31));
}

__INLINE uint8_t plf_codec_sysclk_cntl_codec_sysclk_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_CODEC_SYSCLK_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void plf_codec_sysclk_cntl_codec_sysclk_en_setf(uint8_t codecsysclken)
{
    ASSERT_ERR((((uint32_t)codecsysclken << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_PL_WR(PLF_CODEC_SYSCLK_CNTL_ADDR, (REG_PL_RD(PLF_CODEC_SYSCLK_CNTL_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)codecsysclken << 30));
}

__INLINE uint8_t plf_codec_sysclk_cntl_codec_sysclk_period_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_CODEC_SYSCLK_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void plf_codec_sysclk_cntl_codec_sysclk_period_setf(uint8_t codecsysclkperiod)
{
    ASSERT_ERR((((uint32_t)codecsysclkperiod << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(PLF_CODEC_SYSCLK_CNTL_ADDR, (REG_PL_RD(PLF_CODEC_SYSCLK_CNTL_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)codecsysclkperiod << 0));
}

/**
 * @brief RF_SYSCLK_CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     16        RF_DCM_LOCKED   0
 *     08        RF_CLK_SWITCH   0
 *     00         RF_SYSCLK_EN   0
 * </pre>
 */
#define PLF_RF_SYSCLK_CNTL_ADDR   0x00500040
#define PLF_RF_SYSCLK_CNTL_OFFSET 0x00000040
#define PLF_RF_SYSCLK_CNTL_INDEX  0x00000010
#define PLF_RF_SYSCLK_CNTL_RESET  0x00000000

__INLINE uint32_t plf_rf_sysclk_cntl_get(void)
{
    return REG_PL_RD(PLF_RF_SYSCLK_CNTL_ADDR);
}

__INLINE void plf_rf_sysclk_cntl_set(uint32_t value)
{
    REG_PL_WR(PLF_RF_SYSCLK_CNTL_ADDR, value);
}

// field definitions
#define PLF_RF_DCM_LOCKED_BIT    ((uint32_t)0x00010000)
#define PLF_RF_DCM_LOCKED_POS    16
#define PLF_RF_CLK_SWITCH_BIT    ((uint32_t)0x00000100)
#define PLF_RF_CLK_SWITCH_POS    8
#define PLF_RF_SYSCLK_EN_BIT     ((uint32_t)0x00000001)
#define PLF_RF_SYSCLK_EN_POS     0

#define PLF_RF_DCM_LOCKED_RST    0x0
#define PLF_RF_CLK_SWITCH_RST    0x0
#define PLF_RF_SYSCLK_EN_RST     0x0

__INLINE void plf_rf_sysclk_cntl_pack(uint8_t rfclkswitch, uint8_t rfsysclken)
{
    ASSERT_ERR((((uint32_t)rfclkswitch << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)rfsysclken << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(PLF_RF_SYSCLK_CNTL_ADDR,  ((uint32_t)rfclkswitch << 8) | ((uint32_t)rfsysclken << 0));
}

__INLINE void plf_rf_sysclk_cntl_unpack(uint8_t* rfdcmlocked, uint8_t* rfclkswitch, uint8_t* rfsysclken)
{
    uint32_t localVal = REG_PL_RD(PLF_RF_SYSCLK_CNTL_ADDR);

    *rfdcmlocked = (localVal & ((uint32_t)0x00010000)) >> 16;
    *rfclkswitch = (localVal & ((uint32_t)0x00000100)) >> 8;
    *rfsysclken = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t plf_rf_sysclk_cntl_rf_dcm_locked_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_RF_SYSCLK_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE uint8_t plf_rf_sysclk_cntl_rf_clk_switch_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_RF_SYSCLK_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void plf_rf_sysclk_cntl_rf_clk_switch_setf(uint8_t rfclkswitch)
{
    ASSERT_ERR((((uint32_t)rfclkswitch << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_PL_WR(PLF_RF_SYSCLK_CNTL_ADDR, (REG_PL_RD(PLF_RF_SYSCLK_CNTL_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)rfclkswitch << 8));
}

__INLINE uint8_t plf_rf_sysclk_cntl_rf_sysclk_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(PLF_RF_SYSCLK_CNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void plf_rf_sysclk_cntl_rf_sysclk_en_setf(uint8_t rfsysclken)
{
    ASSERT_ERR((((uint32_t)rfsysclken << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(PLF_RF_SYSCLK_CNTL_ADDR, (REG_PL_RD(PLF_RF_SYSCLK_CNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)rfsysclken << 0));
}


#endif // _REG_PLF_H_

