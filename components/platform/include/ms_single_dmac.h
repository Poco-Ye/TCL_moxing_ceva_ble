/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_single_dmac.h
 * @brief
 * @author bingrui.chen
 * @date 2021-12-30
 * @version 1.0
 * @Revision
 */
#ifndef MS_SINGLE_DMAC_H_
#define MS_SINGLE_DMAC_H_

#include "ms_dmac_hal.h"
#include <stdbool.h>



/**
 * @brief  DMA Callback ID Structure definition
 */
typedef enum
{
    DMAC_CB_ID_TFR,
    DMAC_CB_ID_BLK,
    DMAC_CB_ID_SRC_TFR,
    DMAC_CB_ID_DST_TFR,
    DMAC_CB_ID_ERR,
} DmacCallbackID_Type;


/**
 * @brief  DMA Channel Index Structure definition
 */
typedef enum
{
    DMAC_CHANNEL_0,
    DMAC_CHANNEL_1,
    DMAC_CHANNEL_2,
    DMAC_CHANNEL_3,
    DMAC_CHANNEL_4,
    DMAC_CHANNEL_5,
    DMAC_CHANNEL_MAX,
} DmacChannelIndex_Type;


/**
 * @brief  Peripheral DMA Supported Structure definition
 */
typedef enum DMAC_PERI_TYPE
{
    DMAC_MEM = -1,
    DMAC_I2S0_RX = 0,
    DMAC_I2S0_TX = 1,
    DMAC_I2S1_RX = 2,
    DMAC_I2S1_TX = 3,
    DMAC_UART0_RX = 4,
    DMAC_UART0_TX = 5,
    DMAC_UART1_RX = 6,
    DMAC_UART1_TX = 7,
    DMAC_SPI0_RX = 8,
    DMAC_SPI0_TX = 9,
    DMAC_QSPI_RX = 10,
    DMAC_QSPI_TX = 11,
    DMAC_I2C0_RX = 12,
    DMAC_I2C0_TX = 13,
    DMAC_AUDIO_ADC_RX = 14,
    DMAC_AUX_ADC_RX = 15,
    DMAC_PERI_INVALID
} DmacPeripheral_Type;

/**
 * @brief  DMA Configuration Structure definition
 */
typedef struct
{
    uint32_t src_peri_type;     /*!< DMAC Channel Source Peripheral Type @ref DmacPeripheral_Type*/
    uint32_t dst_peri_type;     /*!< DMAC Channel Destination Peripheral Type @ref DmacPeripheral_Type*/
    uint32_t src_addr_mode;     /*!< DMAC Channel Source Address Increment or Decrement @ref DMAC_ADDRESS_MODE*/
    uint32_t dst_addr_mode;     /*!< DMAC Channel Destination Address Increment or Decrement  @ref DMAC_ADDRESS_MODE*/
    uint32_t src_tr_width;      /*!< DMAC Channel Source Transfer Width  @ref DMAC_XFER_WIDTH*/
    uint32_t dst_tr_width;      /*!< DMAC Channel Destination Transfer Width  @ref DMAC_XFER_WIDTH*/
    uint32_t src_msize;         /*!< DMAC Channel Source Burst Transaction Length  @ref DMAC_MSIZE*/
    uint32_t dst_msize;         /*!< DMAC Channel Destination Burst Transaction Length  @ref DMAC_MSIZE*/
    uint8_t priority;          /*!< DMAC Channel Priority  @ref DMAC_CHANNEL_PRIOR*/
} DmacInit_Type;

/**
 * @brief  DMA Configuration Structure definition
 */
typedef struct __DmacHandle_Type
{
    DmacChannel_Type *dmac_ch_instance;                     /*!< pointer to DMA channel registers base address*/
    Dmac_Type *dma_instance;                                /*!< pointer to DMA controller registers base address*/
    void *parent;                                           /*!< pointer to peripheral handle*/
    DmacInit_Type init;                                     /*!< DMA initial struct*/
    uint16_t error_code;
    uint8_t channel_index;
    uint8_t state;                                          /*!< DMA channel allocation state*/
    void (*xfer_tfr_callback)(struct __DmacHandle_Type *hdmac); /*!< DMA transfer complete callback         */
    void (*xfer_error_callback)(struct __DmacHandle_Type *hdmac); /*!< DMA transfer error callback         */
    uint8_t busy[DMAC_CHANNEL_MAX];
} DmacHandle_Type;

/**
 * @brief  Initial DMAC Manag
 * @retval a Free DMAC Channel Handle
 */
extern void ms_dmac_mgmt_inti(void);

/**
 * @brief  Allocate a Free DMAC Channel
 * @retval a Free DMAC Channel Handle
 */
extern DmacHandle_Type* ms_dmac_mgmt_alloc(void);

/**
 * @brief  Free a Allocated DMAC Channel
 * @retval none
 */
extern void ms_dmac_mgmt_free(DmacHandle_Type *hdmac);

/**
 * @brief  Initial the DMAC Channel
 * @param  DMAC_Type *dmac: DMAC Instance
 * @retval STATUS_SUCCESS or STATUS_FAILURE
 */
extern int32_t ms_dmac_init(DmacHandle_Type *hdmac);

/**
 * @brief  De-initial the DMAC Channel
 * @param  DMAC_Type *dmac: DMAC Instance
 * @retval STATUS_SUCCESS or STATUS_FAILURE
 */
extern int32_t ms_dmac_deinit(DmacHandle_Type *hdmac);

/**
 * @brief  Start a DMAC transfer in block Mode
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t src_addr: dmac source address
 * @param  uint32_t dst_addr: dmac destination address
 * @param  uint16_t count: dmac transmit count
 * @retval STATUS_SUCCESS or STATUS_FAILURE
 */
extern int32_t ms_dmac_start_chx_xfer(DmacHandle_Type *hdmac, uint32_t src_addr, uint32_t dst_addr, uint16_t count);

/**
 * @brief  Get The DMAC Channel Transfer Complete state
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval true or false
 */
extern bool ms_dmac_is_chx_xfer_done(DmacHandle_Type *hdmac);

/**
 * @brief  Stop DMAC Transfer
 * @param[in]  DMAC_Type *dmac: DMAC Instance
 * @retval STATUS_SUCCESS or STATUS_FAILURE
 */
extern int32_t ms_dmac_stop_chx_xfer(DmacHandle_Type *hdmac);

/**
 * @brief  Start The DMAC Transfer in interrupt Mode
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t src_addr: dmac source address
 * @param  uint32_t dst_addr: dmac destination address
 * @param  uint16_t count: dmac transmit count
 * @retval STATUS_SUCCESS or STATUS_FAILURE
 */
extern int32_t ms_dmac_start_chx_xfer_it(DmacHandle_Type *hdmac, uint32_t src_addr, uint32_t dst_addr, uint16_t count);

/**
 * @brief  Stop The DMAC Transfer in interrupt Mode
 * @param[in]  DMAC_Type *dmac: DMAC Instance
 * @retval STATUS_SUCCESS or STATUS_FAILURE
 */
extern int32_t ms_dmac_stop_chx_xfer_it(DmacHandle_Type *hdmac);

/**
 * @brief  Disable DMA Channel x
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval none
 */
extern void ms_dmac_clear_all_int(DmacHandle_Type *hdmac);

/**
 * @brief  Register DMAC  Callback Handle by Callback ID
 * @param[in]  DMAC_Type *dmac: DMAC Instance
 * @param[in]  DmacCallbackID_Type callback_id: callback id
 * @param[in]   p_callback: callback handler
 * @retval STATUS_SUCCESS or STATUS_FAILURE
 */
extern int32_t ms_dmac_register_cb(DmacHandle_Type *hdmac, DmacCallbackID_Type callback_id,
        void (*p_callback)(DmacHandle_Type *_hdmac));

/**
 * @brief Unregister DMAC  Callback Handle by Callback ID
 * @param[in]  DMAC_Type *dmac: DMAC Instance
 * @param[in]  DmacCallbackID_Type callback_id: callback id
 * @retval none
 */
extern void ms_dmac_unregister_cb(DmacHandle_Type *hdmac, DmacCallbackID_Type callback_id);

/**
 * @brief  Get Error Code
 * @param[in]  DMAC_Type *dmac: DMAC Instance
 * @retval Error Code
 */
extern uint32_t dmac_get_error(DmacHandle_Type *hdmac);

#endif /* MS_SIGNLE_DMAC_H_ */
