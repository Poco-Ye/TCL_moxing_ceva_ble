#ifndef __MS_SPI_H
#define __MS_SPI_H

#ifdef __cplusplus
extern "C"{
#endif
#include <stdint.h>
#include "ms.h"
#include "system_ms.h"
#include "ms_spi_reg.h"
//#include "ms_sys_ctrl.h"

#define MS_SPI0_INDEX    0
#define MS_SPI1_INDEX    1
#define MS_SPI_NUM       2
#define SPI_UNVALID_ADDR  0xFFFFFFFF

#define SPI_ROLE_MASTER       (0x0)
#define SPI_ROLE_SLAVE        (0x4)
#define SPI_ROLE_MASTER_SLAVE  SPI_ROLE_MASTER

#define SPI_CPOL_CPHA_MODE 0
#define SPI_DMA_RX_CONFIG  DISABLE
#define SPI_DMA_TX_CONFIG  DISABLE

#define SPI_CLK               (SystemCoreClock)
#define SPI_FIFO_DEPTH        (8)

#define SPI_FRAME_FORMAT_SPI  (0x0)

#define SPI_SLAVE_OUTPUT_DISABLE  (1 << 3)

#define SPI_CLK_POLARITY_POS   (0x6)
#define SPI_CLK_POLARITY_LOW   (0x0)
#define SPI_CLK_POLARITY_HIGH  (0x1 << SPI_CLK_POLARITY_POS)

#define SPI_CLK_PHASE_POS      (0x7)
#define SPI_CLK_PHASE_1EDGE    (0x0)
#define SPI_CLK_PHASE_2EDGE    (0x1 << SPI_CLK_PHASE_POS)

#define SPI_DATA_SIZE_4BIT    (0x3)
#define SPI_DATA_SIZE_5BIT    (0x4)
#define SPI_DATA_SIZE_6BIT    (0x5)
#define SPI_DATA_SIZE_7BIT    (0x6)
#define SPI_DATA_SIZE_8BIT    (0x7)
#define SPI_DATA_SIZE_9BIT    (0x8)
#define SPI_DATA_SIZE_10BIT   (0x9)
#define SPI_DATA_SIZE_11BIT   (0xA)
#define SPI_DATA_SIZE_12BIT   (0xB)
#define SPI_DATA_SIZE_13BIT   (0xC)
#define SPI_DATA_SIZE_14BIT   (0xD)
#define SPI_DATA_SIZE_15BIT   (0xE)
#define SPI_DATA_SIZE_16BIT   (0xF)

/* SPI flags */
#define SPI_FLAG_TX_FIFO_EMPTY         (0x1)
#define SPI_FLAG_TX_FIFO_NOT_FULL      (1 << 1)
#define SPI_FLAG_RX_FIFO_NOT_EMPTY     (1 << 2)
#define SPI_FLAG_RX_FIFO_FULL          (1 << 3)
#define SPI_FLAG_BUSY                  (1 << 4)

/* SPI interrupts */
#define SPI_INTERRUPT_RX_FIFO_OVERRUN  (1 << 0)
#define SPI_INTERRUPT_RX_TIMEOUT       (1 << 1)
#define SPI_INTERRUPT_RX_FIFO_TRIGGER  (1 << 2)  //there are four or more entries in rx fifo
#define SPI_INTERRUPT_TX_FIFO_TRIGGER  (1 << 3)  //there are four or fewer entries in tx fifo.
#define SPI_INTERRUPT_ALL              (0xf)
#define SPI_DISABLE_INTERRUPT_ALL      (0x0)

#define SPI_DMA_TX_EN                  (1<<1)
#define SPI_DMA_RX_EN                  (1)


#define SPI0_CS_PIN   PAD3
#define SPI0_CLK_PIN  PAD2
#define SPI0_TXD_PIN  PAD4
#define SPI0_RXD_PIN  PAD5


typedef struct {
    uint32_t mode;        /* spi communication mode */
    uint32_t freq;        /* communication frequency Hz */
    uint32_t data_frame_size;
} ms_spi_config_t;

typedef struct {
    uint8_t      port;    /* spi port */
    ms_spi_config_t config;  /* spi config */
    void        *priv;    /* priv data */
} ms_spi_dev_t;

typedef void (*ms_spi_callback_func)(uint8_t);
extern ms_spi_callback_func g_ms_spi_callback_handler[MS_SPI_NUM];



void ms_spi_interrupt_config(uint32_t spi_base, uint8_t spi_interrupt, uint8_t new_state);
int32_t ms_spi_dma_config(ms_spi_dev_t * spi,uint8_t dma_tx_rx_sel,uint8_t new_state);
int32_t ms_spi_cpol_cpha_config(ms_spi_dev_t * spi,uint8_t mode);
void ms_spi_cmd(uint32_t spi_base, uint8_t new_state);
void ms_spi_struct_init(ms_spi_dev_t *init_struct);
void ms_spi_cs_enable(ms_spi_dev_t *rf_spi0);

void ms_spi_cs_disable(ms_spi_dev_t *rf_spi0);

void SPI0_master_config(ms_spi_dev_t * dev);

/**
 * Initialises the SPI interface for a given SPI device
 *
 * @param[in]  spi  the spi device
 *
 * @return  0 : on success, -1 : if the SPI device could not be initialised
 */
int32_t ms_spi_init(ms_spi_dev_t *spi);

/**
 * Spi send
 *
 * @param[in]  spi      the spi device
 * @param[in]  data     spi send data
 * @param[in]  size     spi send data size
 * @param[in]  timeout  timeout in ms
 *
 * @return  0 : on success, -1 : if the SPI device could not be initialised
 */
int32_t ms_spi_send(ms_spi_dev_t *spi, const uint32_t *data, uint16_t size, uint32_t timeout);

/**
 * spi_recv
 *
 * @param[in]   spi      the spi device
 * @param[out]  data     spi recv data
 * @param[in]   size     spi recv data size
 * @param[in]   timeout  timeout in ms
 *
 * @return  0 : on success, -1 : if the SPI device could not be initialised
 */
int32_t ms_spi_recv(ms_spi_dev_t *spi, uint32_t *data, uint16_t size, uint32_t timeout);

/**
 * De-initialises a SPI interface
 *
 *
 * @param[in]  spi the SPI device to be de-initialised
 *
 * @return  0 : on success, -1 : if an error occurred
 */
int32_t ms_spi_finalize(ms_spi_dev_t *spi);
void ms_spi_cs_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __MS_SPI_H */
