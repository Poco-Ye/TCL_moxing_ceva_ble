#ifndef _REG_UART_H_
#define _REG_UART_H_

#include <stdint.h>
#include "_reg_uart.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_UART_COUNT 14

#define REG_UART_DECODING_MASK 0x0000003F

/**
 * @brief GENERAL_CONFIG register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     06          BAUD_CLK_EN   0
 *  05:04          PARITY_TYPE   0x0
 *     03            PARITY_EN   0
 *     02             STOP_LEN   0
 *  01:00             WORD_LEN   0x0
 * </pre>
 */
#define UART_GENERAL_CONFIG_ADDR   0x00401000
#define UART_GENERAL_CONFIG_OFFSET 0x00000000
#define UART_GENERAL_CONFIG_INDEX  0x00000000
#define UART_GENERAL_CONFIG_RESET  0x00000000

__INLINE uint32_t uart_general_config_get(void)
{
    return REG_PL_RD(UART_GENERAL_CONFIG_ADDR);
}

__INLINE void uart_general_config_set(uint32_t value)
{
    REG_PL_WR(UART_GENERAL_CONFIG_ADDR, value);
}

// field definitions
#define UART_BAUD_CLK_EN_BIT    ((uint32_t)0x00000040)
#define UART_BAUD_CLK_EN_POS    6
#define UART_PARITY_TYPE_MASK   ((uint32_t)0x00000030)
#define UART_PARITY_TYPE_LSB    4
#define UART_PARITY_TYPE_WIDTH  ((uint32_t)0x00000002)
#define UART_PARITY_EN_BIT      ((uint32_t)0x00000008)
#define UART_PARITY_EN_POS      3
#define UART_STOP_LEN_BIT       ((uint32_t)0x00000004)
#define UART_STOP_LEN_POS       2
#define UART_WORD_LEN_MASK      ((uint32_t)0x00000003)
#define UART_WORD_LEN_LSB       0
#define UART_WORD_LEN_WIDTH     ((uint32_t)0x00000002)

#define UART_BAUD_CLK_EN_RST    0x0
#define UART_PARITY_TYPE_RST    0x0
#define UART_PARITY_EN_RST      0x0
#define UART_STOP_LEN_RST       0x0
#define UART_WORD_LEN_RST       0x0

__INLINE void uart_general_config_pack(uint8_t baudclken, uint8_t paritytype, uint8_t parityen, uint8_t stoplen, uint8_t wordlen)
{
    ASSERT_ERR((((uint32_t)baudclken << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)paritytype << 4) & ~((uint32_t)0x00000030)) == 0);
    ASSERT_ERR((((uint32_t)parityen << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)stoplen << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)wordlen << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_PL_WR(UART_GENERAL_CONFIG_ADDR,  ((uint32_t)baudclken << 6) | ((uint32_t)paritytype << 4) | ((uint32_t)parityen << 3) | ((uint32_t)stoplen << 2) | ((uint32_t)wordlen << 0));
}

__INLINE void uart_general_config_unpack(uint8_t* baudclken, uint8_t* paritytype, uint8_t* parityen, uint8_t* stoplen, uint8_t* wordlen)
{
    uint32_t localVal = REG_PL_RD(UART_GENERAL_CONFIG_ADDR);

    *baudclken = (localVal & ((uint32_t)0x00000040)) >> 6;
    *paritytype = (localVal & ((uint32_t)0x00000030)) >> 4;
    *parityen = (localVal & ((uint32_t)0x00000008)) >> 3;
    *stoplen = (localVal & ((uint32_t)0x00000004)) >> 2;
    *wordlen = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t uart_general_config_baud_clk_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_GENERAL_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void uart_general_config_baud_clk_en_setf(uint8_t baudclken)
{
    ASSERT_ERR((((uint32_t)baudclken << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_PL_WR(UART_GENERAL_CONFIG_ADDR, (REG_PL_RD(UART_GENERAL_CONFIG_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)baudclken << 6));
}

__INLINE uint8_t uart_general_config_parity_type_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_GENERAL_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000030)) >> 4);
}

__INLINE void uart_general_config_parity_type_setf(uint8_t paritytype)
{
    ASSERT_ERR((((uint32_t)paritytype << 4) & ~((uint32_t)0x00000030)) == 0);
    REG_PL_WR(UART_GENERAL_CONFIG_ADDR, (REG_PL_RD(UART_GENERAL_CONFIG_ADDR) & ~((uint32_t)0x00000030)) | ((uint32_t)paritytype << 4));
}

__INLINE uint8_t uart_general_config_parity_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_GENERAL_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void uart_general_config_parity_en_setf(uint8_t parityen)
{
    ASSERT_ERR((((uint32_t)parityen << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_PL_WR(UART_GENERAL_CONFIG_ADDR, (REG_PL_RD(UART_GENERAL_CONFIG_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)parityen << 3));
}

__INLINE uint8_t uart_general_config_stop_len_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_GENERAL_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void uart_general_config_stop_len_setf(uint8_t stoplen)
{
    ASSERT_ERR((((uint32_t)stoplen << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_PL_WR(UART_GENERAL_CONFIG_ADDR, (REG_PL_RD(UART_GENERAL_CONFIG_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)stoplen << 2));
}

__INLINE uint8_t uart_general_config_word_len_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_GENERAL_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

__INLINE void uart_general_config_word_len_setf(uint8_t wordlen)
{
    ASSERT_ERR((((uint32_t)wordlen << 0) & ~((uint32_t)0x00000003)) == 0);
    REG_PL_WR(UART_GENERAL_CONFIG_ADDR, (REG_PL_RD(UART_GENERAL_CONFIG_ADDR) & ~((uint32_t)0x00000003)) | ((uint32_t)wordlen << 0));
}

/**
 * @brief INT_DIV_LSB register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00          INT_DIV_LSB   0x2
 * </pre>
 */
#define UART_INT_DIV_LSB_ADDR   0x00401004
#define UART_INT_DIV_LSB_OFFSET 0x00000004
#define UART_INT_DIV_LSB_INDEX  0x00000001
#define UART_INT_DIV_LSB_RESET  0x00000002

__INLINE uint32_t uart_int_div_lsb_get(void)
{
    return REG_PL_RD(UART_INT_DIV_LSB_ADDR);
}

__INLINE void uart_int_div_lsb_set(uint32_t value)
{
    REG_PL_WR(UART_INT_DIV_LSB_ADDR, value);
}

// field definitions
#define UART_INT_DIV_LSB_MASK   ((uint32_t)0x000000FF)
#define UART_INT_DIV_LSB_LSB    0
#define UART_INT_DIV_LSB_WIDTH  ((uint32_t)0x00000008)

#define UART_INT_DIV_LSB_RST    0x2

__INLINE uint8_t uart_int_div_lsb_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_INT_DIV_LSB_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

__INLINE void uart_int_div_lsb_setf(uint8_t intdivlsb)
{
    ASSERT_ERR((((uint32_t)intdivlsb << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(UART_INT_DIV_LSB_ADDR, (uint32_t)intdivlsb << 0);
}

/**
 * @brief INT_DIV_MSB register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  04:00          INT_DIV_MSB   0x0
 * </pre>
 */
#define UART_INT_DIV_MSB_ADDR   0x00401008
#define UART_INT_DIV_MSB_OFFSET 0x00000008
#define UART_INT_DIV_MSB_INDEX  0x00000002
#define UART_INT_DIV_MSB_RESET  0x00000000

__INLINE uint32_t uart_int_div_msb_get(void)
{
    return REG_PL_RD(UART_INT_DIV_MSB_ADDR);
}

__INLINE void uart_int_div_msb_set(uint32_t value)
{
    REG_PL_WR(UART_INT_DIV_MSB_ADDR, value);
}

// field definitions
#define UART_INT_DIV_MSB_MASK   ((uint32_t)0x0000001F)
#define UART_INT_DIV_MSB_LSB    0
#define UART_INT_DIV_MSB_WIDTH  ((uint32_t)0x00000005)

#define UART_INT_DIV_MSB_RST    0x0

__INLINE uint8_t uart_int_div_msb_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_INT_DIV_MSB_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x0000001F)) == 0);
    return (localVal >> 0);
}

__INLINE void uart_int_div_msb_setf(uint8_t intdivmsb)
{
    ASSERT_ERR((((uint32_t)intdivmsb << 0) & ~((uint32_t)0x0000001F)) == 0);
    REG_PL_WR(UART_INT_DIV_MSB_ADDR, (uint32_t)intdivmsb << 0);
}

/**
 * @brief FRACT_DIV register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  02:00            FRACT_DIV   0x0
 * </pre>
 */
#define UART_FRACT_DIV_ADDR   0x0040100C
#define UART_FRACT_DIV_OFFSET 0x0000000C
#define UART_FRACT_DIV_INDEX  0x00000003
#define UART_FRACT_DIV_RESET  0x00000000

__INLINE uint32_t uart_fract_div_get(void)
{
    return REG_PL_RD(UART_FRACT_DIV_ADDR);
}

__INLINE void uart_fract_div_set(uint32_t value)
{
    REG_PL_WR(UART_FRACT_DIV_ADDR, value);
}

// field definitions
#define UART_FRACT_DIV_MASK   ((uint32_t)0x00000007)
#define UART_FRACT_DIV_LSB    0
#define UART_FRACT_DIV_WIDTH  ((uint32_t)0x00000003)

#define UART_FRACT_DIV_RST    0x0

__INLINE uint8_t uart_fract_div_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_FRACT_DIV_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x00000007)) == 0);
    return (localVal >> 0);
}

__INLINE void uart_fract_div_setf(uint8_t fractdiv)
{
    ASSERT_ERR((((uint32_t)fractdiv << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_PL_WR(UART_FRACT_DIV_ADDR, (uint32_t)fractdiv << 0);
}

/**
 * @brief FLOW_CTRL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     04        RX_FIFO_FLUSH   0
 *     03        TX_FIFO_FLUSH   0
 *     02           RTS_FORCED   0
 *     01             AUTO_RTS   0
 *     00               CTS_EN   0
 * </pre>
 */
#define UART_FLOW_CTRL_ADDR   0x00401010
#define UART_FLOW_CTRL_OFFSET 0x00000010
#define UART_FLOW_CTRL_INDEX  0x00000004
#define UART_FLOW_CTRL_RESET  0x00000000

__INLINE uint32_t uart_flow_ctrl_get(void)
{
    return REG_PL_RD(UART_FLOW_CTRL_ADDR);
}

__INLINE void uart_flow_ctrl_set(uint32_t value)
{
    REG_PL_WR(UART_FLOW_CTRL_ADDR, value);
}

// field definitions
#define UART_RX_FIFO_FLUSH_BIT    ((uint32_t)0x00000010)
#define UART_RX_FIFO_FLUSH_POS    4
#define UART_TX_FIFO_FLUSH_BIT    ((uint32_t)0x00000008)
#define UART_TX_FIFO_FLUSH_POS    3
#define UART_RTS_FORCED_BIT       ((uint32_t)0x00000004)
#define UART_RTS_FORCED_POS       2
#define UART_AUTO_RTS_BIT         ((uint32_t)0x00000002)
#define UART_AUTO_RTS_POS         1
#define UART_CTS_EN_BIT           ((uint32_t)0x00000001)
#define UART_CTS_EN_POS           0

#define UART_RX_FIFO_FLUSH_RST    0x0
#define UART_TX_FIFO_FLUSH_RST    0x0
#define UART_RTS_FORCED_RST       0x0
#define UART_AUTO_RTS_RST         0x0
#define UART_CTS_EN_RST           0x0

__INLINE void uart_flow_ctrl_pack(uint8_t rxfifoflush, uint8_t txfifoflush, uint8_t rtsforced, uint8_t autorts, uint8_t ctsen)
{
    ASSERT_ERR((((uint32_t)rxfifoflush << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)txfifoflush << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)rtsforced << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)autorts << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)ctsen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(UART_FLOW_CTRL_ADDR,  ((uint32_t)rxfifoflush << 4) | ((uint32_t)txfifoflush << 3) | ((uint32_t)rtsforced << 2) | ((uint32_t)autorts << 1) | ((uint32_t)ctsen << 0));
}

__INLINE void uart_flow_ctrl_unpack(uint8_t* rxfifoflush, uint8_t* txfifoflush, uint8_t* rtsforced, uint8_t* autorts, uint8_t* ctsen)
{
    uint32_t localVal = REG_PL_RD(UART_FLOW_CTRL_ADDR);

    *rxfifoflush = (localVal & ((uint32_t)0x00000010)) >> 4;
    *txfifoflush = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rtsforced = (localVal & ((uint32_t)0x00000004)) >> 2;
    *autorts = (localVal & ((uint32_t)0x00000002)) >> 1;
    *ctsen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t uart_flow_ctrl_rx_fifo_flush_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_FLOW_CTRL_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void uart_flow_ctrl_rx_fifo_flush_setf(uint8_t rxfifoflush)
{
    ASSERT_ERR((((uint32_t)rxfifoflush << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_PL_WR(UART_FLOW_CTRL_ADDR, (REG_PL_RD(UART_FLOW_CTRL_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)rxfifoflush << 4));
}

__INLINE uint8_t uart_flow_ctrl_tx_fifo_flush_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_FLOW_CTRL_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void uart_flow_ctrl_tx_fifo_flush_setf(uint8_t txfifoflush)
{
    ASSERT_ERR((((uint32_t)txfifoflush << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_PL_WR(UART_FLOW_CTRL_ADDR, (REG_PL_RD(UART_FLOW_CTRL_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)txfifoflush << 3));
}

__INLINE uint8_t uart_flow_ctrl_rts_forced_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_FLOW_CTRL_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void uart_flow_ctrl_rts_forced_setf(uint8_t rtsforced)
{
    ASSERT_ERR((((uint32_t)rtsforced << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_PL_WR(UART_FLOW_CTRL_ADDR, (REG_PL_RD(UART_FLOW_CTRL_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)rtsforced << 2));
}

__INLINE uint8_t uart_flow_ctrl_auto_rts_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_FLOW_CTRL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void uart_flow_ctrl_auto_rts_setf(uint8_t autorts)
{
    ASSERT_ERR((((uint32_t)autorts << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_PL_WR(UART_FLOW_CTRL_ADDR, (REG_PL_RD(UART_FLOW_CTRL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)autorts << 1));
}

__INLINE uint8_t uart_flow_ctrl_cts_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_FLOW_CTRL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void uart_flow_ctrl_cts_en_setf(uint8_t ctsen)
{
    ASSERT_ERR((((uint32_t)ctsen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(UART_FLOW_CTRL_ADDR, (REG_PL_RD(UART_FLOW_CTRL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)ctsen << 0));
}

/**
 * @brief RX_FIFO_THRESHOLD register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00          RX_FIFO_THR   0x0
 * </pre>
 */
#define UART_RX_FIFO_THRESHOLD_ADDR   0x00401014
#define UART_RX_FIFO_THRESHOLD_OFFSET 0x00000014
#define UART_RX_FIFO_THRESHOLD_INDEX  0x00000005
#define UART_RX_FIFO_THRESHOLD_RESET  0x00000000

__INLINE uint32_t uart_rx_fifo_threshold_get(void)
{
    return REG_PL_RD(UART_RX_FIFO_THRESHOLD_ADDR);
}

__INLINE void uart_rx_fifo_threshold_set(uint32_t value)
{
    REG_PL_WR(UART_RX_FIFO_THRESHOLD_ADDR, value);
}

// field definitions
#define UART_RX_FIFO_THR_MASK   ((uint32_t)0x000000FF)
#define UART_RX_FIFO_THR_LSB    0
#define UART_RX_FIFO_THR_WIDTH  ((uint32_t)0x00000008)

#define UART_RX_FIFO_THR_RST    0x0

__INLINE uint8_t uart_rx_fifo_threshold_rx_fifo_thr_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_FIFO_THRESHOLD_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

__INLINE void uart_rx_fifo_threshold_rx_fifo_thr_setf(uint8_t rxfifothr)
{
    ASSERT_ERR((((uint32_t)rxfifothr << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(UART_RX_FIFO_THRESHOLD_ADDR, (uint32_t)rxfifothr << 0);
}

/**
 * @brief TX_FIFO_THRESHOLD register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00          TX_FIFO_THR   0x0
 * </pre>
 */
#define UART_TX_FIFO_THRESHOLD_ADDR   0x00401018
#define UART_TX_FIFO_THRESHOLD_OFFSET 0x00000018
#define UART_TX_FIFO_THRESHOLD_INDEX  0x00000006
#define UART_TX_FIFO_THRESHOLD_RESET  0x00000000

__INLINE uint32_t uart_tx_fifo_threshold_get(void)
{
    return REG_PL_RD(UART_TX_FIFO_THRESHOLD_ADDR);
}

__INLINE void uart_tx_fifo_threshold_set(uint32_t value)
{
    REG_PL_WR(UART_TX_FIFO_THRESHOLD_ADDR, value);
}

// field definitions
#define UART_TX_FIFO_THR_MASK   ((uint32_t)0x000000FF)
#define UART_TX_FIFO_THR_LSB    0
#define UART_TX_FIFO_THR_WIDTH  ((uint32_t)0x00000008)

#define UART_TX_FIFO_THR_RST    0x0

__INLINE uint8_t uart_tx_fifo_threshold_tx_fifo_thr_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_TX_FIFO_THRESHOLD_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

__INLINE void uart_tx_fifo_threshold_tx_fifo_thr_setf(uint8_t txfifothr)
{
    ASSERT_ERR((((uint32_t)txfifothr << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(UART_TX_FIFO_THRESHOLD_ADDR, (uint32_t)txfifothr << 0);
}

/**
 * @brief RX_TIMEOUT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00           RX_TIMEOUT   0x0
 * </pre>
 */
#define UART_RX_TIMEOUT_ADDR   0x0040101C
#define UART_RX_TIMEOUT_OFFSET 0x0000001C
#define UART_RX_TIMEOUT_INDEX  0x00000007
#define UART_RX_TIMEOUT_RESET  0x00000000

__INLINE uint32_t uart_rx_timeout_get(void)
{
    return REG_PL_RD(UART_RX_TIMEOUT_ADDR);
}

__INLINE void uart_rx_timeout_set(uint32_t value)
{
    REG_PL_WR(UART_RX_TIMEOUT_ADDR, value);
}

// field definitions
#define UART_RX_TIMEOUT_MASK   ((uint32_t)0x000000FF)
#define UART_RX_TIMEOUT_LSB    0
#define UART_RX_TIMEOUT_WIDTH  ((uint32_t)0x00000008)

#define UART_RX_TIMEOUT_RST    0x0

__INLINE uint8_t uart_rx_timeout_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_TIMEOUT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

__INLINE void uart_rx_timeout_setf(uint8_t rxtimeout)
{
    ASSERT_ERR((((uint32_t)rxtimeout << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(UART_RX_TIMEOUT_ADDR, (uint32_t)rxtimeout << 0);
}

/**
 * @brief RX_DATA register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00              RX_DATA   0x0
 * </pre>
 */
#define UART_RX_DATA_ADDR   0x00401020
#define UART_RX_DATA_OFFSET 0x00000020
#define UART_RX_DATA_INDEX  0x00000008
#define UART_RX_DATA_RESET  0x00000000

__INLINE uint32_t uart_rx_data_get(void)
{
    return REG_PL_RD(UART_RX_DATA_ADDR);
}

// field definitions
#define UART_RX_DATA_MASK   ((uint32_t)0x000000FF)
#define UART_RX_DATA_LSB    0
#define UART_RX_DATA_WIDTH  ((uint32_t)0x00000008)

#define UART_RX_DATA_RST    0x0

__INLINE uint8_t uart_rx_data_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_DATA_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief RX_STATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     07           RX_TIMEOUT   0
 *     06             RX_BREAK   0
 *     05         RX_FRAME_ERR   0
 *     04        RX_PARITY_ERR   0
 *     03      RX_FIFO_OVERRUN   0
 *     02         RX_FIFO_FULL   0
 *     01   RX_FIFO_ALMOST_FULL   0
 *     00    RX_FIFO_NOT_EMPTY   0
 * </pre>
 */
#define UART_RX_STATUS_ADDR   0x00401024
#define UART_RX_STATUS_OFFSET 0x00000024
#define UART_RX_STATUS_INDEX  0x00000009
#define UART_RX_STATUS_RESET  0x00000000

__INLINE uint32_t uart_rx_status_get(void)
{
    return REG_PL_RD(UART_RX_STATUS_ADDR);
}

// field definitions
#define UART_RX_TIMEOUT_BIT             ((uint32_t)0x00000080)
#define UART_RX_TIMEOUT_POS             7
#define UART_RX_BREAK_BIT               ((uint32_t)0x00000040)
#define UART_RX_BREAK_POS               6
#define UART_RX_FRAME_ERR_BIT           ((uint32_t)0x00000020)
#define UART_RX_FRAME_ERR_POS           5
#define UART_RX_PARITY_ERR_BIT          ((uint32_t)0x00000010)
#define UART_RX_PARITY_ERR_POS          4
#define UART_RX_FIFO_OVERRUN_BIT        ((uint32_t)0x00000008)
#define UART_RX_FIFO_OVERRUN_POS        3
#define UART_RX_FIFO_FULL_BIT           ((uint32_t)0x00000004)
#define UART_RX_FIFO_FULL_POS           2
#define UART_RX_FIFO_ALMOST_FULL_BIT    ((uint32_t)0x00000002)
#define UART_RX_FIFO_ALMOST_FULL_POS    1
#define UART_RX_FIFO_NOT_EMPTY_BIT      ((uint32_t)0x00000001)
#define UART_RX_FIFO_NOT_EMPTY_POS      0

#define UART_RX_TIMEOUT_RST             0x0
#define UART_RX_BREAK_RST               0x0
#define UART_RX_FRAME_ERR_RST           0x0
#define UART_RX_PARITY_ERR_RST          0x0
#define UART_RX_FIFO_OVERRUN_RST        0x0
#define UART_RX_FIFO_FULL_RST           0x0
#define UART_RX_FIFO_ALMOST_FULL_RST    0x0
#define UART_RX_FIFO_NOT_EMPTY_RST      0x0

__INLINE void uart_rx_status_unpack(uint8_t* rxtimeout, uint8_t* rxbreak, uint8_t* rxframeerr, uint8_t* rxparityerr, uint8_t* rxfifooverrun, uint8_t* rxfifofull, uint8_t* rxfifoalmostfull, uint8_t* rxfifonotempty)
{
    uint32_t localVal = REG_PL_RD(UART_RX_STATUS_ADDR);

    *rxtimeout = (localVal & ((uint32_t)0x00000080)) >> 7;
    *rxbreak = (localVal & ((uint32_t)0x00000040)) >> 6;
    *rxframeerr = (localVal & ((uint32_t)0x00000020)) >> 5;
    *rxparityerr = (localVal & ((uint32_t)0x00000010)) >> 4;
    *rxfifooverrun = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rxfifofull = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rxfifoalmostfull = (localVal & ((uint32_t)0x00000002)) >> 1;
    *rxfifonotempty = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t uart_rx_status_rx_timeout_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE uint8_t uart_rx_status_rx_break_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t uart_rx_status_rx_frame_err_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t uart_rx_status_rx_parity_err_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t uart_rx_status_rx_fifo_overrun_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t uart_rx_status_rx_fifo_full_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t uart_rx_status_rx_fifo_almost_full_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t uart_rx_status_rx_fifo_not_empty_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief RX_MASK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     07      RX_TIMEOUT_MASK   0
 *     06        RX_BREAK_MASK   0
 *     05    RX_FRAME_ERR_MASK   0
 *     04   RX_PARITY_ERR_MASK   0
 *     03   RX_FIFO_OVERRUN_MASK   0
 *     02    RX_FIFO_FULL_MASK   0
 *     01   RX_FIFO_THRESHOLD_MASK   0
 *     00   RX_FIFO_NOT_EMPTY_MASK   0
 * </pre>
 */
#define UART_RX_MASK_ADDR   0x00401028
#define UART_RX_MASK_OFFSET 0x00000028
#define UART_RX_MASK_INDEX  0x0000000A
#define UART_RX_MASK_RESET  0x00000000

__INLINE uint32_t uart_rx_mask_get(void)
{
    return REG_PL_RD(UART_RX_MASK_ADDR);
}

__INLINE void uart_rx_mask_set(uint32_t value)
{
    REG_PL_WR(UART_RX_MASK_ADDR, value);
}

// field definitions
#define UART_RX_TIMEOUT_MASK_BIT           ((uint32_t)0x00000080)
#define UART_RX_TIMEOUT_MASK_POS           7
#define UART_RX_BREAK_MASK_BIT             ((uint32_t)0x00000040)
#define UART_RX_BREAK_MASK_POS             6
#define UART_RX_FRAME_ERR_MASK_BIT         ((uint32_t)0x00000020)
#define UART_RX_FRAME_ERR_MASK_POS         5
#define UART_RX_PARITY_ERR_MASK_BIT        ((uint32_t)0x00000010)
#define UART_RX_PARITY_ERR_MASK_POS        4
#define UART_RX_FIFO_OVERRUN_MASK_BIT      ((uint32_t)0x00000008)
#define UART_RX_FIFO_OVERRUN_MASK_POS      3
#define UART_RX_FIFO_FULL_MASK_BIT         ((uint32_t)0x00000004)
#define UART_RX_FIFO_FULL_MASK_POS         2
#define UART_RX_FIFO_THRESHOLD_MASK_BIT    ((uint32_t)0x00000002)
#define UART_RX_FIFO_THRESHOLD_MASK_POS    1
#define UART_RX_FIFO_NOT_EMPTY_MASK_BIT    ((uint32_t)0x00000001)
#define UART_RX_FIFO_NOT_EMPTY_MASK_POS    0

#define UART_RX_TIMEOUT_MASK_RST           0x0
#define UART_RX_BREAK_MASK_RST             0x0
#define UART_RX_FRAME_ERR_MASK_RST         0x0
#define UART_RX_PARITY_ERR_MASK_RST        0x0
#define UART_RX_FIFO_OVERRUN_MASK_RST      0x0
#define UART_RX_FIFO_FULL_MASK_RST         0x0
#define UART_RX_FIFO_THRESHOLD_MASK_RST    0x0
#define UART_RX_FIFO_NOT_EMPTY_MASK_RST    0x0

__INLINE void uart_rx_mask_pack(uint8_t rxtimeoutmask, uint8_t rxbreakmask, uint8_t rxframeerrmask, uint8_t rxparityerrmask, uint8_t rxfifooverrunmask, uint8_t rxfifofullmask, uint8_t rxfifothresholdmask, uint8_t rxfifonotemptymask)
{
    ASSERT_ERR((((uint32_t)rxtimeoutmask << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)rxbreakmask << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)rxframeerrmask << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)rxparityerrmask << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)rxfifooverrunmask << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)rxfifofullmask << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)rxfifothresholdmask << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)rxfifonotemptymask << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(UART_RX_MASK_ADDR,  ((uint32_t)rxtimeoutmask << 7) | ((uint32_t)rxbreakmask << 6) | ((uint32_t)rxframeerrmask << 5) | ((uint32_t)rxparityerrmask << 4) | ((uint32_t)rxfifooverrunmask << 3) | ((uint32_t)rxfifofullmask << 2) | ((uint32_t)rxfifothresholdmask << 1) | ((uint32_t)rxfifonotemptymask << 0));
}

__INLINE void uart_rx_mask_unpack(uint8_t* rxtimeoutmask, uint8_t* rxbreakmask, uint8_t* rxframeerrmask, uint8_t* rxparityerrmask, uint8_t* rxfifooverrunmask, uint8_t* rxfifofullmask, uint8_t* rxfifothresholdmask, uint8_t* rxfifonotemptymask)
{
    uint32_t localVal = REG_PL_RD(UART_RX_MASK_ADDR);

    *rxtimeoutmask = (localVal & ((uint32_t)0x00000080)) >> 7;
    *rxbreakmask = (localVal & ((uint32_t)0x00000040)) >> 6;
    *rxframeerrmask = (localVal & ((uint32_t)0x00000020)) >> 5;
    *rxparityerrmask = (localVal & ((uint32_t)0x00000010)) >> 4;
    *rxfifooverrunmask = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rxfifofullmask = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rxfifothresholdmask = (localVal & ((uint32_t)0x00000002)) >> 1;
    *rxfifonotemptymask = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t uart_rx_mask_rx_timeout_mask_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_MASK_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void uart_rx_mask_rx_timeout_mask_setf(uint8_t rxtimeoutmask)
{
    ASSERT_ERR((((uint32_t)rxtimeoutmask << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_PL_WR(UART_RX_MASK_ADDR, (REG_PL_RD(UART_RX_MASK_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)rxtimeoutmask << 7));
}

__INLINE uint8_t uart_rx_mask_rx_break_mask_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_MASK_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void uart_rx_mask_rx_break_mask_setf(uint8_t rxbreakmask)
{
    ASSERT_ERR((((uint32_t)rxbreakmask << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_PL_WR(UART_RX_MASK_ADDR, (REG_PL_RD(UART_RX_MASK_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)rxbreakmask << 6));
}

__INLINE uint8_t uart_rx_mask_rx_frame_err_mask_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_MASK_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void uart_rx_mask_rx_frame_err_mask_setf(uint8_t rxframeerrmask)
{
    ASSERT_ERR((((uint32_t)rxframeerrmask << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_PL_WR(UART_RX_MASK_ADDR, (REG_PL_RD(UART_RX_MASK_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)rxframeerrmask << 5));
}

__INLINE uint8_t uart_rx_mask_rx_parity_err_mask_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_MASK_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void uart_rx_mask_rx_parity_err_mask_setf(uint8_t rxparityerrmask)
{
    ASSERT_ERR((((uint32_t)rxparityerrmask << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_PL_WR(UART_RX_MASK_ADDR, (REG_PL_RD(UART_RX_MASK_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)rxparityerrmask << 4));
}

__INLINE uint8_t uart_rx_mask_rx_fifo_overrun_mask_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_MASK_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void uart_rx_mask_rx_fifo_overrun_mask_setf(uint8_t rxfifooverrunmask)
{
    ASSERT_ERR((((uint32_t)rxfifooverrunmask << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_PL_WR(UART_RX_MASK_ADDR, (REG_PL_RD(UART_RX_MASK_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)rxfifooverrunmask << 3));
}

__INLINE uint8_t uart_rx_mask_rx_fifo_full_mask_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_MASK_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void uart_rx_mask_rx_fifo_full_mask_setf(uint8_t rxfifofullmask)
{
    ASSERT_ERR((((uint32_t)rxfifofullmask << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_PL_WR(UART_RX_MASK_ADDR, (REG_PL_RD(UART_RX_MASK_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)rxfifofullmask << 2));
}

__INLINE uint8_t uart_rx_mask_rx_fifo_threshold_mask_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_MASK_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void uart_rx_mask_rx_fifo_threshold_mask_setf(uint8_t rxfifothresholdmask)
{
    ASSERT_ERR((((uint32_t)rxfifothresholdmask << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_PL_WR(UART_RX_MASK_ADDR, (REG_PL_RD(UART_RX_MASK_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)rxfifothresholdmask << 1));
}

__INLINE uint8_t uart_rx_mask_rx_fifo_not_empty_mask_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_RX_MASK_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void uart_rx_mask_rx_fifo_not_empty_mask_setf(uint8_t rxfifonotemptymask)
{
    ASSERT_ERR((((uint32_t)rxfifonotemptymask << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(UART_RX_MASK_ADDR, (REG_PL_RD(UART_RX_MASK_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)rxfifonotemptymask << 0));
}

/**
 * @brief TX_DATA register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00              TX_DATA   0x0
 * </pre>
 */
#define UART_TX_DATA_ADDR   0x0040102C
#define UART_TX_DATA_OFFSET 0x0000002C
#define UART_TX_DATA_INDEX  0x0000000B
#define UART_TX_DATA_RESET  0x00000000

__INLINE void uart_tx_data_set(uint32_t value)
{
    REG_PL_WR(UART_TX_DATA_ADDR, value);
}

// field definitions
#define UART_TX_DATA_MASK   ((uint32_t)0x000000FF)
#define UART_TX_DATA_LSB    0
#define UART_TX_DATA_WIDTH  ((uint32_t)0x00000008)

#define UART_TX_DATA_RST    0x0

__INLINE void uart_tx_data_setf(uint8_t txdata)
{
    ASSERT_ERR((((uint32_t)txdata << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(UART_TX_DATA_ADDR, (uint32_t)txdata << 0);
}

/**
 * @brief TX_STATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     04                  CTS   0
 *     03          TX_SR_EMPTY   0
 *     02        TX_FIFO_EMPTY   0
 *     01   TX_FIO_ALMOST_EMPTY   0
 *     00     TX_FIFO_NOT_FULL   0
 * </pre>
 */
#define UART_TX_STATUS_ADDR   0x00401030
#define UART_TX_STATUS_OFFSET 0x00000030
#define UART_TX_STATUS_INDEX  0x0000000C
#define UART_TX_STATUS_RESET  0x00000000

__INLINE uint32_t uart_tx_status_get(void)
{
    return REG_PL_RD(UART_TX_STATUS_ADDR);
}

// field definitions
#define UART_CTS_BIT                    ((uint32_t)0x00000010)
#define UART_CTS_POS                    4
#define UART_TX_SR_EMPTY_BIT            ((uint32_t)0x00000008)
#define UART_TX_SR_EMPTY_POS            3
#define UART_TX_FIFO_EMPTY_BIT          ((uint32_t)0x00000004)
#define UART_TX_FIFO_EMPTY_POS          2
#define UART_TX_FIO_ALMOST_EMPTY_BIT    ((uint32_t)0x00000002)
#define UART_TX_FIO_ALMOST_EMPTY_POS    1
#define UART_TX_FIFO_NOT_FULL_BIT       ((uint32_t)0x00000001)
#define UART_TX_FIFO_NOT_FULL_POS       0

#define UART_CTS_RST                    0x0
#define UART_TX_SR_EMPTY_RST            0x0
#define UART_TX_FIFO_EMPTY_RST          0x0
#define UART_TX_FIO_ALMOST_EMPTY_RST    0x0
#define UART_TX_FIFO_NOT_FULL_RST       0x0

__INLINE void uart_tx_status_unpack(uint8_t* cts, uint8_t* txsrempty, uint8_t* txfifoempty, uint8_t* txfioalmostempty, uint8_t* txfifonotfull)
{
    uint32_t localVal = REG_PL_RD(UART_TX_STATUS_ADDR);

    *cts = (localVal & ((uint32_t)0x00000010)) >> 4;
    *txsrempty = (localVal & ((uint32_t)0x00000008)) >> 3;
    *txfifoempty = (localVal & ((uint32_t)0x00000004)) >> 2;
    *txfioalmostempty = (localVal & ((uint32_t)0x00000002)) >> 1;
    *txfifonotfull = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t uart_tx_status_cts_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_TX_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t uart_tx_status_tx_sr_empty_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_TX_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t uart_tx_status_tx_fifo_empty_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_TX_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t uart_tx_status_tx_fio_almost_empty_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_TX_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t uart_tx_status_tx_fifo_not_full_getf(void)
{
    uint32_t localVal = REG_PL_RD(UART_TX_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief TX_MASK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     04             CTS_MASK   0
 *     03     TX_SR_EMPTY_MASK   0
 *     02   TX_FIFO_EMPTY_MASK   0
 *     01   TX_FIO_ALMOST_EMPTY_MASK   0
 *     00   TX_FIFO_NOT_FULL_MASK   0
 * </pre>
 */
#define UART_TX_MASK_ADDR   0x00401034
#define UART_TX_MASK_OFFSET 0x00000034
#define UART_TX_MASK_INDEX  0x0000000D
#define UART_TX_MASK_RESET  0x00000000

__INLINE uint32_t uart_tx_mask_get(void)
{
    return REG_PL_RD(UART_TX_MASK_ADDR);
}

__INLINE void uart_tx_mask_set(uint32_t value)
{
    REG_PL_WR(UART_TX_MASK_ADDR, value);
}

// field definitions
#define UART_CTS_MASK_BIT                    ((uint32_t)0x00000010)
#define UART_CTS_MASK_POS                    4
#define UART_TX_SR_EMPTY_MASK_BIT            ((uint32_t)0x00000008)
#define UART_TX_SR_EMPTY_MASK_POS            3
#define UART_TX_FIFO_EMPTY_MASK_BIT          ((uint32_t)0x00000004)
#define UART_TX_FIFO_EMPTY_MASK_POS          2
#define UART_TX_FIO_ALMOST_EMPTY_MASK_BIT    ((uint32_t)0x00000002)
#define UART_TX_FIO_ALMOST_EMPTY_MASK_POS    1
#define UART_TX_FIFO_NOT_FULL_MASK_BIT       ((uint32_t)0x00000001)
#define UART_TX_FIFO_NOT_FULL_MASK_POS       0

#define UART_CTS_MASK_RST                    0x0
#define UART_TX_SR_EMPTY_MASK_RST            0x0
#define UART_TX_FIFO_EMPTY_MASK_RST          0x0
#define UART_TX_FIO_ALMOST_EMPTY_MASK_RST    0x0
#define UART_TX_FIFO_NOT_FULL_MASK_RST       0x0

__INLINE void uart_tx_mask_pack(uint8_t ctsmask, uint8_t txsremptymask, uint8_t txfifoemptymask, uint8_t txfioalmostemptymask, uint8_t txfifonotfullmask)
{
    ASSERT_ERR((((uint32_t)ctsmask << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)txsremptymask << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)txfifoemptymask << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)txfioalmostemptymask << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)txfifonotfullmask << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(UART_TX_MASK_ADDR,  ((uint32_t)ctsmask << 4) | ((uint32_t)txsremptymask << 3) | ((uint32_t)txfifoemptymask << 2) | ((uint32_t)txfioalmostemptymask << 1) | ((uint32_t)txfifonotfullmask << 0));
}

__INLINE void uart_tx_mask_unpack(uint8_t* ctsmask, uint8_t* txsremptymask, uint8_t* txfifoemptymask, uint8_t* txfioalmostemptymask, uint8_t* txfifonotfullmask)
{
    uint32_t localVal = REG_PL_RD(UART_TX_MASK_ADDR);

    *ctsmask = (localVal & ((uint32_t)0x00000010)) >> 4;
    *txsremptymask = (localVal & ((uint32_t)0x00000008)) >> 3;
    *txfifoemptymask = (localVal & ((uint32_t)0x00000004)) >> 2;
    *txfioalmostemptymask = (localVal & ((uint32_t)0x00000002)) >> 1;
    *txfifonotfullmask = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE void uart_tx_mask_cts_mask_setf(uint8_t ctsmask)
{
    ASSERT_ERR((((uint32_t)ctsmask << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_PL_WR(UART_TX_MASK_ADDR, (REG_PL_RD(UART_TX_MASK_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)ctsmask << 4));
}

__INLINE void uart_tx_mask_tx_sr_empty_mask_setf(uint8_t txsremptymask)
{
    ASSERT_ERR((((uint32_t)txsremptymask << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_PL_WR(UART_TX_MASK_ADDR, (REG_PL_RD(UART_TX_MASK_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)txsremptymask << 3));
}

__INLINE void uart_tx_mask_tx_fifo_empty_mask_setf(uint8_t txfifoemptymask)
{
    ASSERT_ERR((((uint32_t)txfifoemptymask << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_PL_WR(UART_TX_MASK_ADDR, (REG_PL_RD(UART_TX_MASK_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)txfifoemptymask << 2));
}

__INLINE void uart_tx_mask_tx_fio_almost_empty_mask_setf(uint8_t txfioalmostemptymask)
{
    ASSERT_ERR((((uint32_t)txfioalmostemptymask << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_PL_WR(UART_TX_MASK_ADDR, (REG_PL_RD(UART_TX_MASK_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)txfioalmostemptymask << 1));
}

__INLINE void uart_tx_mask_tx_fifo_not_full_mask_setf(uint8_t txfifonotfullmask)
{
    ASSERT_ERR((((uint32_t)txfifonotfullmask << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(UART_TX_MASK_ADDR, (REG_PL_RD(UART_TX_MASK_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)txfifonotfullmask << 0));
}


#endif // _REG_UART_H_

