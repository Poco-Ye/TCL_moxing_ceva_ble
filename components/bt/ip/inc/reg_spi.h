#ifndef _REG_SPI_H_
#define _REG_SPI_H_

#include <stdint.h>
#include "_reg_spi.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_SPI_COUNT 6

#define REG_SPI_DECODING_MASK 0x0000001F

/**
 * @brief SPI_CONFIG register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00           SPI_ENABLE   0
 * </pre>
 */
#define SPI_SPI_CONFIG_ADDR   0x00407000
#define SPI_SPI_CONFIG_OFFSET 0x00000000
#define SPI_SPI_CONFIG_INDEX  0x00000000
#define SPI_SPI_CONFIG_RESET  0x00000000

__INLINE uint32_t spi_spi_config_get(void)
{
    return REG_PL_RD(SPI_SPI_CONFIG_ADDR);
}

__INLINE void spi_spi_config_set(uint32_t value)
{
    REG_PL_WR(SPI_SPI_CONFIG_ADDR, value);
}

// field definitions
#define SPI_SPI_ENABLE_BIT    ((uint32_t)0x00000001)
#define SPI_SPI_ENABLE_POS    0

#define SPI_SPI_ENABLE_RST    0x0

__INLINE uint8_t spi_spi_config_spi_enable_getf(void)
{
    uint32_t localVal = REG_PL_RD(SPI_SPI_CONFIG_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x00000001)) == 0);
    return (localVal >> 0);
}

__INLINE void spi_spi_config_spi_enable_setf(uint8_t spienable)
{
    ASSERT_ERR((((uint32_t)spienable << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(SPI_SPI_CONFIG_ADDR, (uint32_t)spienable << 0);
}

/**
 * @brief SPI_CLK_DIV register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00              CLK_DIV   0x0
 * </pre>
 */
#define SPI_SPI_CLK_DIV_ADDR   0x00407004
#define SPI_SPI_CLK_DIV_OFFSET 0x00000004
#define SPI_SPI_CLK_DIV_INDEX  0x00000001
#define SPI_SPI_CLK_DIV_RESET  0x00000000

__INLINE uint32_t spi_spi_clk_div_get(void)
{
    return REG_PL_RD(SPI_SPI_CLK_DIV_ADDR);
}

__INLINE void spi_spi_clk_div_set(uint32_t value)
{
    REG_PL_WR(SPI_SPI_CLK_DIV_ADDR, value);
}

// field definitions
#define SPI_CLK_DIV_MASK   ((uint32_t)0x000000FF)
#define SPI_CLK_DIV_LSB    0
#define SPI_CLK_DIV_WIDTH  ((uint32_t)0x00000008)

#define SPI_CLK_DIV_RST    0x0

__INLINE uint8_t spi_spi_clk_div_clk_div_getf(void)
{
    uint32_t localVal = REG_PL_RD(SPI_SPI_CLK_DIV_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

__INLINE void spi_spi_clk_div_clk_div_setf(uint8_t clkdiv)
{
    ASSERT_ERR((((uint32_t)clkdiv << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(SPI_SPI_CLK_DIV_ADDR, (uint32_t)clkdiv << 0);
}

/**
 * @brief SPI_TX_DATA register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00              TX_DATA   0x0
 * </pre>
 */
#define SPI_SPI_TX_DATA_ADDR   0x00407008
#define SPI_SPI_TX_DATA_OFFSET 0x00000008
#define SPI_SPI_TX_DATA_INDEX  0x00000002
#define SPI_SPI_TX_DATA_RESET  0x00000000

__INLINE void spi_spi_tx_data_set(uint32_t value)
{
    REG_PL_WR(SPI_SPI_TX_DATA_ADDR, value);
}

// field definitions
#define SPI_TX_DATA_MASK   ((uint32_t)0x000000FF)
#define SPI_TX_DATA_LSB    0
#define SPI_TX_DATA_WIDTH  ((uint32_t)0x00000008)

#define SPI_TX_DATA_RST    0x0

__INLINE void spi_spi_tx_data_tx_data_setf(uint8_t txdata)
{
    ASSERT_ERR((((uint32_t)txdata << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(SPI_SPI_TX_DATA_ADDR, (uint32_t)txdata << 0);
}

/**
 * @brief SPI_RX_DATA register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00              RX_DATA   0x0
 * </pre>
 */
#define SPI_SPI_RX_DATA_ADDR   0x0040700C
#define SPI_SPI_RX_DATA_OFFSET 0x0000000C
#define SPI_SPI_RX_DATA_INDEX  0x00000003
#define SPI_SPI_RX_DATA_RESET  0x00000000

__INLINE uint32_t spi_spi_rx_data_get(void)
{
    return REG_PL_RD(SPI_SPI_RX_DATA_ADDR);
}

// field definitions
#define SPI_RX_DATA_MASK   ((uint32_t)0x000000FF)
#define SPI_RX_DATA_LSB    0
#define SPI_RX_DATA_WIDTH  ((uint32_t)0x00000008)

#define SPI_RX_DATA_RST    0x0

__INLINE uint8_t spi_spi_rx_data_rx_data_getf(void)
{
    uint32_t localVal = REG_PL_RD(SPI_SPI_RX_DATA_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief SPI_STATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     06      RX_FIFO_OVERRUN   0
 *     05    RX_FIFO_HALF_FULL   0
 *     04    RX_FIFO_NOT_EMPTY   0
 *     02        TX_FIFO_EMPTY   0
 *     01   TX_FIFO_HALF_EMPTY   0
 *     00     TX_FIFO_NOT_FULL   0
 * </pre>
 */
#define SPI_SPI_STATUS_ADDR   0x00407010
#define SPI_SPI_STATUS_OFFSET 0x00000010
#define SPI_SPI_STATUS_INDEX  0x00000004
#define SPI_SPI_STATUS_RESET  0x00000000

__INLINE uint32_t spi_spi_status_get(void)
{
    return REG_PL_RD(SPI_SPI_STATUS_ADDR);
}

// field definitions
#define SPI_RX_FIFO_OVERRUN_BIT       ((uint32_t)0x00000040)
#define SPI_RX_FIFO_OVERRUN_POS       6
#define SPI_RX_FIFO_HALF_FULL_BIT     ((uint32_t)0x00000020)
#define SPI_RX_FIFO_HALF_FULL_POS     5
#define SPI_RX_FIFO_NOT_EMPTY_BIT     ((uint32_t)0x00000010)
#define SPI_RX_FIFO_NOT_EMPTY_POS     4
#define SPI_TX_FIFO_EMPTY_BIT         ((uint32_t)0x00000004)
#define SPI_TX_FIFO_EMPTY_POS         2
#define SPI_TX_FIFO_HALF_EMPTY_BIT    ((uint32_t)0x00000002)
#define SPI_TX_FIFO_HALF_EMPTY_POS    1
#define SPI_TX_FIFO_NOT_FULL_BIT      ((uint32_t)0x00000001)
#define SPI_TX_FIFO_NOT_FULL_POS      0

#define SPI_RX_FIFO_OVERRUN_RST       0x0
#define SPI_RX_FIFO_HALF_FULL_RST     0x0
#define SPI_RX_FIFO_NOT_EMPTY_RST     0x0
#define SPI_TX_FIFO_EMPTY_RST         0x0
#define SPI_TX_FIFO_HALF_EMPTY_RST    0x0
#define SPI_TX_FIFO_NOT_FULL_RST      0x0

__INLINE void spi_spi_status_unpack(uint8_t* rxfifooverrun, uint8_t* rxfifohalffull, uint8_t* rxfifonotempty, uint8_t* txfifoempty, uint8_t* txfifohalfempty, uint8_t* txfifonotfull)
{
    uint32_t localVal = REG_PL_RD(SPI_SPI_STATUS_ADDR);

    *rxfifooverrun = (localVal & ((uint32_t)0x00000040)) >> 6;
    *rxfifohalffull = (localVal & ((uint32_t)0x00000020)) >> 5;
    *rxfifonotempty = (localVal & ((uint32_t)0x00000010)) >> 4;
    *txfifoempty = (localVal & ((uint32_t)0x00000004)) >> 2;
    *txfifohalfempty = (localVal & ((uint32_t)0x00000002)) >> 1;
    *txfifonotfull = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t spi_spi_status_rx_fifo_overrun_getf(void)
{
    uint32_t localVal = REG_PL_RD(SPI_SPI_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t spi_spi_status_rx_fifo_half_full_getf(void)
{
    uint32_t localVal = REG_PL_RD(SPI_SPI_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t spi_spi_status_rx_fifo_not_empty_getf(void)
{
    uint32_t localVal = REG_PL_RD(SPI_SPI_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t spi_spi_status_tx_fifo_empty_getf(void)
{
    uint32_t localVal = REG_PL_RD(SPI_SPI_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t spi_spi_status_tx_fifo_half_empty_getf(void)
{
    uint32_t localVal = REG_PL_RD(SPI_SPI_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t spi_spi_status_tx_fifo_not_full_getf(void)
{
    uint32_t localVal = REG_PL_RD(SPI_SPI_STATUS_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief SPI_MASK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     06   RX_FIFO_OVERRUN_MASK   0
 *     05   RX_FIFO_HALF_FULL_MASK   0
 *     04   RX_FIFO_NOT_EMPTY_MASK   0
 *     02   TX_FIFO_EMPTY_MASK   0
 *     01   TX_FIFO_HALF_EMPTY_MASK   0
 *     00   TX_FIFO_NOT_FULL_MASK   0
 * </pre>
 */
#define SPI_SPI_MASK_ADDR   0x00407014
#define SPI_SPI_MASK_OFFSET 0x00000014
#define SPI_SPI_MASK_INDEX  0x00000005
#define SPI_SPI_MASK_RESET  0x00000000

__INLINE uint32_t spi_spi_mask_get(void)
{
    return REG_PL_RD(SPI_SPI_MASK_ADDR);
}

__INLINE void spi_spi_mask_set(uint32_t value)
{
    REG_PL_WR(SPI_SPI_MASK_ADDR, value);
}

// field definitions
#define SPI_RX_FIFO_OVERRUN_MASK_BIT       ((uint32_t)0x00000040)
#define SPI_RX_FIFO_OVERRUN_MASK_POS       6
#define SPI_RX_FIFO_HALF_FULL_MASK_BIT     ((uint32_t)0x00000020)
#define SPI_RX_FIFO_HALF_FULL_MASK_POS     5
#define SPI_RX_FIFO_NOT_EMPTY_MASK_BIT     ((uint32_t)0x00000010)
#define SPI_RX_FIFO_NOT_EMPTY_MASK_POS     4
#define SPI_TX_FIFO_EMPTY_MASK_BIT         ((uint32_t)0x00000004)
#define SPI_TX_FIFO_EMPTY_MASK_POS         2
#define SPI_TX_FIFO_HALF_EMPTY_MASK_BIT    ((uint32_t)0x00000002)
#define SPI_TX_FIFO_HALF_EMPTY_MASK_POS    1
#define SPI_TX_FIFO_NOT_FULL_MASK_BIT      ((uint32_t)0x00000001)
#define SPI_TX_FIFO_NOT_FULL_MASK_POS      0

#define SPI_RX_FIFO_OVERRUN_MASK_RST       0x0
#define SPI_RX_FIFO_HALF_FULL_MASK_RST     0x0
#define SPI_RX_FIFO_NOT_EMPTY_MASK_RST     0x0
#define SPI_TX_FIFO_EMPTY_MASK_RST         0x0
#define SPI_TX_FIFO_HALF_EMPTY_MASK_RST    0x0
#define SPI_TX_FIFO_NOT_FULL_MASK_RST      0x0

__INLINE void spi_spi_mask_pack(uint8_t rxfifooverrunmask, uint8_t rxfifohalffullmask, uint8_t rxfifonotemptymask, uint8_t txfifoemptymask, uint8_t txfifohalfemptymask, uint8_t txfifonotfullmask)
{
    ASSERT_ERR((((uint32_t)rxfifooverrunmask << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)rxfifohalffullmask << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)rxfifonotemptymask << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)txfifoemptymask << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)txfifohalfemptymask << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)txfifonotfullmask << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(SPI_SPI_MASK_ADDR,  ((uint32_t)rxfifooverrunmask << 6) | ((uint32_t)rxfifohalffullmask << 5) | ((uint32_t)rxfifonotemptymask << 4) | ((uint32_t)txfifoemptymask << 2) | ((uint32_t)txfifohalfemptymask << 1) | ((uint32_t)txfifonotfullmask << 0));
}

__INLINE void spi_spi_mask_unpack(uint8_t* rxfifooverrunmask, uint8_t* rxfifohalffullmask, uint8_t* rxfifonotemptymask, uint8_t* txfifoemptymask, uint8_t* txfifohalfemptymask, uint8_t* txfifonotfullmask)
{
    uint32_t localVal = REG_PL_RD(SPI_SPI_MASK_ADDR);

    *rxfifooverrunmask = (localVal & ((uint32_t)0x00000040)) >> 6;
    *rxfifohalffullmask = (localVal & ((uint32_t)0x00000020)) >> 5;
    *rxfifonotemptymask = (localVal & ((uint32_t)0x00000010)) >> 4;
    *txfifoemptymask = (localVal & ((uint32_t)0x00000004)) >> 2;
    *txfifohalfemptymask = (localVal & ((uint32_t)0x00000002)) >> 1;
    *txfifonotfullmask = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t spi_spi_mask_rx_fifo_overrun_mask_getf(void)
{
    uint32_t localVal = REG_PL_RD(SPI_SPI_MASK_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void spi_spi_mask_rx_fifo_overrun_mask_setf(uint8_t rxfifooverrunmask)
{
    ASSERT_ERR((((uint32_t)rxfifooverrunmask << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_PL_WR(SPI_SPI_MASK_ADDR, (REG_PL_RD(SPI_SPI_MASK_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)rxfifooverrunmask << 6));
}

__INLINE uint8_t spi_spi_mask_rx_fifo_half_full_mask_getf(void)
{
    uint32_t localVal = REG_PL_RD(SPI_SPI_MASK_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void spi_spi_mask_rx_fifo_half_full_mask_setf(uint8_t rxfifohalffullmask)
{
    ASSERT_ERR((((uint32_t)rxfifohalffullmask << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_PL_WR(SPI_SPI_MASK_ADDR, (REG_PL_RD(SPI_SPI_MASK_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)rxfifohalffullmask << 5));
}

__INLINE uint8_t spi_spi_mask_rx_fifo_not_empty_mask_getf(void)
{
    uint32_t localVal = REG_PL_RD(SPI_SPI_MASK_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void spi_spi_mask_rx_fifo_not_empty_mask_setf(uint8_t rxfifonotemptymask)
{
    ASSERT_ERR((((uint32_t)rxfifonotemptymask << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_PL_WR(SPI_SPI_MASK_ADDR, (REG_PL_RD(SPI_SPI_MASK_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)rxfifonotemptymask << 4));
}

__INLINE uint8_t spi_spi_mask_tx_fifo_empty_mask_getf(void)
{
    uint32_t localVal = REG_PL_RD(SPI_SPI_MASK_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void spi_spi_mask_tx_fifo_empty_mask_setf(uint8_t txfifoemptymask)
{
    ASSERT_ERR((((uint32_t)txfifoemptymask << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_PL_WR(SPI_SPI_MASK_ADDR, (REG_PL_RD(SPI_SPI_MASK_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)txfifoemptymask << 2));
}

__INLINE uint8_t spi_spi_mask_tx_fifo_half_empty_mask_getf(void)
{
    uint32_t localVal = REG_PL_RD(SPI_SPI_MASK_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void spi_spi_mask_tx_fifo_half_empty_mask_setf(uint8_t txfifohalfemptymask)
{
    ASSERT_ERR((((uint32_t)txfifohalfemptymask << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_PL_WR(SPI_SPI_MASK_ADDR, (REG_PL_RD(SPI_SPI_MASK_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)txfifohalfemptymask << 1));
}

__INLINE uint8_t spi_spi_mask_tx_fifo_not_full_mask_getf(void)
{
    uint32_t localVal = REG_PL_RD(SPI_SPI_MASK_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void spi_spi_mask_tx_fifo_not_full_mask_setf(uint8_t txfifonotfullmask)
{
    ASSERT_ERR((((uint32_t)txfifonotfullmask << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(SPI_SPI_MASK_ADDR, (REG_PL_RD(SPI_SPI_MASK_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)txfifonotfullmask << 0));
}


#endif // _REG_SPI_H_

