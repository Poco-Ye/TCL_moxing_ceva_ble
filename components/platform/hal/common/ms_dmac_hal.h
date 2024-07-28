/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  ms_hal_dmac.h
 * @brief
 * @author bingrui.chen
 * @date 2022年1月5日
 * @version 1.0
 * @Revision:
 */
#ifndef BLE_SOC_XXX_MS_DMAC_HAL_H_
#define BLE_SOC_XXX_MS_DMAC_HAL_H_

#include <ms_dmac_ll.h>

/** @addtogroup DMAC_HAL_Functions
 * @{
 */
/** @defgroup DMAC_HAL DMAC
 * @{
 */

/**
 * @brief  Enable DW_ahb_dmac
 * @param  DMAC_Type *dmac: DMAC Instance
 * @retval None
 */
#define ms_dmac_hal_enable(dmac)  ms_dmac_ll_enable((dmac))

/**
 * @brief  Disable DW_ahb_dmac
 * @param  DMAC_Type *dmac: DMAC Instance
 * @retval None
 */
#define  ms_dmac_hal_disable(dmac) ms_dmac_ll_disable((dmac))

/**
 * @brief  Get the DW_ahb_dmac Enable Status
 * @param  DMAC_Type *dmac: DMAC Instance
 * @retval Status of  DW_ahb_dmac Enable( true or false)
 */
#define ms_dmac_hal_is_enable(dmac) ms_dmac_ll_is_enable((dmac))

/**
 * @brief  Enable DMA Channel x
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval none
 */
#define ms_dmac_hal_enable_channel(dmac,channel) ms_dmac_ll_enable_channel((dmac),(channel))

/**
 * @brief  Disable DMA Channel x
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval none
 */
#define ms_dmac_hal_disable_channel(dmac,channel) ms_dmac_ll_disable_channel((dmac),(channel))

/**
 * @brief  Get DMA Channel x Enable Status
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval enable or disable
 */
#define ms_dmac_hal_is_enable_channel(dmac,channel) ms_dmac_ll_is_enable_channel((dmac),(channel))

/**
 * @brief  Mask Channel x transfer complete Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
#define ms_dmac_hal_set_mask_tfr_int(dmac,channel,mask) ms_dmac_ll_set_mask_tfr_int((dmac),(channel),(mask))

/**
 * @brief  Mask Channel x block Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
#define ms_dmac_hal_set_mask_block_int(dmac,channel,mask) ms_dmac_ll_set_mask_block_int((dmac),(channel),(mask))

/**
 * @brief  Mask Channel x source transfer complete Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
#define ms_dmac_hal_set_mask_src_tfr_int(dmac,channel,mask) ms_dmac_ll_set_mask_src_tfr_int((dmac),(channel),(mask))

/**
 * @brief  Mask Channel x destination transfer complete Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
#define ms_dmac_hal_set_mask_dst_tfr_int(dmac,channel,mask) ms_dmac_ll_set_mask_dst_tfr_int((dmac),(channel),(mask))

/**
 * @brief  Mask Channel x error Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
#define ms_dmac_hal_set_mask_err_int(dmac,channel,mask) ms_dmac_ll_set_mask_err_int((dmac),(channel),(mask))

/**
 * @brief  Clear for transfer complete Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
#define ms_dmac_hal_clear_tfr_int(dmac,channel) ms_dmac_ll_clear_tfr_int((dmac),(channel))

/**
 * @brief  Clear for block Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
#define ms_dmac_hal_clear_block_int(dmac,channel) ms_dmac_ll_clear_block_int((dmac),(channel))

/**
 * @brief  Clear for transfer source complete Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
#define ms_dmac_hal_clear_src_tfr_int(dmac,channel) ms_dmac_ll_clear_src_tfr_int((dmac),(channel))

/**
 * @brief  Clear for transfer destination complete Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
#define ms_dmac_hal_clear_dst_tfr_int(dmac,channel) ms_dmac_ll_clear_dst_tfr_int((dmac),(channel))

/**
 * @brief   Clear for error Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
#define ms_dmac_hal_clear_err_it(dmac,channel) ms_dmac_ll_clear_err_it((dmac),(channel))

/**
 * @brief  Get DMA Channel Transfer complete interrupt status after masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status(true of false)
 */
#define ms_dmac_hal_ch_int_is_tfr_active(dmac,channel) ms_dmac_ll_ch_int_is_tfr_active((dmac),(channel))

/**
 * @brief  Get DMA Channel Transfer Block complete interrupt status after masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status
 */
#define ms_dmac_hal_ch_int_is_block_active(dmac,channel) ms_dmac_ll_ch_int_is_block_active((dmac),(channel))

/**
 * @brief  Get DMA Channel source Transfer complete interrupt status after masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status(true of false)
 */
#define ms_dmac_hal_ch_int_is_src_tfr_active(dmac,channel) ms_dmac_ll_ch_int_is_src_tfr_active((dmac),(channel))

/**
 * @brief  Get DMA Channel source Transfer complete interrupt status after masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status(true of false)
 */
#define ms_dmac_hal_ch_int_is_dst_tfr_active(dmac,channel) ms_dmac_ll_ch_int_is_dst_tfr_active((dmac),(channel))

/**
 * @brief  Get DMA Channel Error interrupt status after masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status
 */
#define ms_dmac_hal_ch_int_is_error_active(dmac,channel) ms_dmac_ll_ch_int_is_error_active((dmac),(channel))

/**
 * @brief  Get DMA Channel Transfer complete Combined interrupt status
 * @param  DMAC_Type *dmac: DMAC Instance
 * @retval Inactive or Active Interrupt Status(true of false)
 */
#define ms_dmac_hal_combi_is_tfr_active(dmac) ms_dmac_ll_combi_is_tfr_active((dmac))

/**
 * @brief  Get DMA Channel Transfer Block complete Combined interrupt status
 * @param  DMAC_Type *dmac: DMAC Instance
 * @retval Inactive or Active Interrupt Status
 */
#define ms_dmac_hal_combi_is_block_active(dmac) ms_dmac_ll_combi_is_block_active((dmac))

/**
 * @brief  Get DMA Channel X Error Combined interrupt Status
 * @param  DMAC_Type *dmac: DMAC Instance
 * @retval Inactive or Active Interrupt Status
 */
#define ms_dmac_hal_combi_is_src_active(dmac) ms_dmac_ll_combi_is_src_active((dmac))

/**
 * @brief  Get DMA Channel X Error Combined interrupt Status
 * @param  DMAC_Type *dmac: DMAC Instance
 * @retval Inactive or Active Interrupt Status
 */

#define ms_dmac_hal_combi_is_dst_active(dmac) ms_dmac_ll_combi_is_dst_active((dmac))

/**
 * @brief  Get DMA Channel X Error Combined interrupt Status
 * @param  DMAC_Type *dmac: DMAC Instance
 * @retval Inactive or Active Interrupt Status
 */
#define ms_dmac_hal_combi_is_error_active(dmac) ms_dmac_ll_combi_is_error_active((dmac))

/**
 * @brief  Get DMA Channel Transfer complete interrupt status before  masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status(true of false)
 */
#define ms_dmac_hal_raw_is_tfr_active(dmac,channel) ms_dmac_ll_raw_is_tfr_active((dmac),(channel))

/**
 * @brief  Get DMA Channel Transfer Block complete interrupt status before  masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status
 */
#define ms_dmac_hal_raw_is_block_active(dmac,channel) ms_dmac_ll_raw_is_block_active((dmac),(channel))

/**
 * @brief  Get DMA Channel X Error interrupt Status  before  masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status
 */
#define ms_dmac_hal_raw_is_src_active(dmac,channel) ms_dmac_ll_raw_is_block_active((dmac),(channel))

/**
 * @brief  Get DMA Channel X Error interrupt Status  before  masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status
 */
#define ms_dmac_hal_raw_is_dst_active(dmac,channel) ms_dmac_ll_raw_is_dst_active((dmac),(channel))

/**
 * @brief  Get DMA Channel X Error interrupt Status  before  masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status
 */
#define ms_dmac_hal_raw_is_error_active(dmac,channel) ms_dmac_ll_raw_is_error_active((dmac),(channel))

/**
 * @brief  Enable Source Software Transaction Request
 * @param  DMAC_Type *dmac:  DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
#define ms_dmac_hal_enable_src_soft_req(dmac,channel) ms_dmac_ll_enable_src_soft_req((dmac),(channel))

/**
 * @brief  Disable Source Software Transaction Request
 * @param  DMAC_Type *dmac:  DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
#define ms_dmac_hal_disable_src_soft_req(dmac,channel) ms_dmac_ll_disable_src_soft_req((dmac),(channel))

/**
 * @brief  Enable Destination Software Transaction Request
 * @param  DMAC_Type *dmac:  DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
#define ms_dmac_hal_enable_dst_soft_req(dmac,channel) ms_dmac_ll_enable_dst_soft_req((dmac),(channel))

/**
 * @brief  Disable Destination Software Transaction Request
 * @param  DMAC_Type *dmac:  DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
#define ms_dmac_hal_disable_dst_soft_req(dmac,channel) ms_dmac_ll_disable_dst_soft_req((dmac),(channel))

/**
 * @brief  Active Source Single Transaction Request
 * @param  DMAC_Type *dmac:  DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */

#define ms_dmac_hal_active_src_single_req(dmac,channel) ms_dmac_ll_active_src_single_req((dmac),(channel))

/**
 * @brief  Active Destination Single Transaction Request
 * @param  DMAC_Type *dmac:  DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
#define ms_dmac_hal_active_dst_single_req(dmac,channel) ms_dmac_ll_active_dst_single_req((dmac),(channel))

/**
 * @brief  Active Source Last Transaction Request register
 * @param  DMAC_Type *dmac:  DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
#define ms_dmac_hal_active_src_last_req(dmac,channel) ms_dmac_ll_active_src_last_req((dmac),(channel))

/**
 * @brief  Active Destination Last Transaction Request
 * @param  DMAC_Type *dmac:  DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
#define ms_dmac_hal_active_dst_last_req(dmac,channel) ms_dmac_ll_active_dst_last_req((dmac),(channel))

/**
 * @brief  Set DMAC Channel Transfer Type and Flow Control.
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t type: This parameter can be one of the following values:
 *         @arg @ref DMAC_XFER_TYPE_MEM_TO_MEM
 *         @arg @ref DMAC_XFER_TYPE_MEM_TO_PERI
 *         @arg @ref DMAC_XFER_TYPE_PERI_TO_MEM
 * @retval None
 */
#define ms_dmac_hal_channel_set_transfer_type(dmac,type) ms_dmac_ll_channel_set_transfer_type((dmac),(type))

/**
 * @brief  Set DMAC Channel Transfer Source Address
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t addr: Transfer Source Address
 * @retval None
 */
#define ms_dmac_hal_channel_set_src_addr(dmac,addr) ms_dmac_ll_channel_set_src_addr((dmac),(addr))

/**
 * @brief  Set DMAC Channel Transfer Destination Address
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t addr: Transfer Destination Address
 * @retval None
 */
#define ms_dmac_hal_channel_set_dst_addr(dmac,addr) ms_dmac_ll_channel_set_dst_addr((dmac),(addr))

/**
 * @brief  Set DMAC Channel Transfer Destination Address
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint64_t llp:
 * @retval None
 */
#define ms_dmac_hal_channel_set_llp(dmac,llp) ms_dmac_ll_channel_set_llp((dmac),(llp))

/**
 * @brief  Set DMAC Channel Source Transfer Width
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t width: This parameter can be one of the following values:
 *         @arg @ref DMAC_XFER_WIDTH_8BITS
 *         @arg @ref DMAC_XFER_WIDTH_16BITS
 *         @arg @ref DMAC_XFER_WIDTH_32BITS
 * @retval None
 */
#define ms_dmac_hal_channel_set_src_transfer_width(dmac,width) ms_dmac_ll_channel_set_src_transfer_width((dmac),(width))

/**
 * @brief  Set DMAC Channel Destination Transfer Width
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t addr_mode: This parameter can be one of the following values:
 *         @arg @ref DMAC_XFER_WIDTH_8BITS
 *         @arg @ref DMAC_XFER_WIDTH_16BITS
 *         @arg @ref DMAC_XFER_WIDTH_32BITS
 * @retval None
 */
#define ms_dmac_hal_channel_set_dst_transfer_width(dmac,width) ms_dmac_ll_channel_set_dst_transfer_width((dmac),(width))

/**
 * @brief  Set DMAC Channel Destination Address Increment
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t addr_mode: This parameter can be one of the following values:
 *         @arg @ref DMAC_ADDRESS_MODE_INC
 *         @arg @ref DMAC_ADDRESS_MODE_DEC
 *         @arg @ref DMAC_ADDRESS_MODE_NOC
 * @retval None
 */
#define ms_dmac_hal_channel_set_dst_addr_mode(dmac,addr_mode) ms_dmac_ll_channel_set_dst_addr_mode((dmac),(addr_mode))

/**
 * @brief  Set DMAC Channel Source Address Increment
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t addr_mode: This parameter can be one of the following values:
 *         @arg @ref DMAC_ADDRESS_MODE_INC
 *         @arg @ref DMAC_ADDRESS_MODE_DEC
 *         @arg @ref DMAC_ADDRESS_MODE_NOC
 * @retval None
 */

#define ms_dmac_hal_channel_set_src_addr_mode(dmac,addr_mode) ms_dmac_ll_channel_set_src_addr_mode((dmac),(addr_mode))

/**
 * @brief  Set DMAC Channel Priority
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t priority: This parameter can be one of the following values:
 *         @arg @ref DMAC_CHANNEL_PRIOR_0
 *         @arg @ref DMAC_CHANNEL_PRIOR_1
 *         @arg @ref DMAC_CHANNEL_PRIOR_2
 *         @arg @ref DMAC_CHANNEL_PRIOR_3
 *         @arg @ref DMAC_CHANNEL_PRIOR_4
 *         @arg @ref DMAC_CHANNEL_PRIOR_5
 *         @arg @ref DMAC_CHANNEL_PRIOR_6
 *         @arg @ref DMAC_CHANNEL_PRIOR_7
 * @retval None
 */
#define ms_dmac_hal_channel_set_priority(dmac,priority) ms_dmac_ll_channel_set_priority((dmac),(priority))

/**
 * @brief  Select DMAC Channel Source Software or Hardware Handshaking
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t hs_sel: This parameter can be one of the following values:
 *         @arg @ref DMAC_CHANNEL_HARDWARE_HS
 *         @arg @ref DMAC_CHANNEL_SOFTWARE_HS
 * @retval None
 */
#define ms_dmac_hal_channel_set_src_hs_sel(dmac,hs_sel) ms_dmac_ll_channel_set_src_hs_sel((dmac),(hs_sel))

/**
 * @brief  Select Destination Software or Hardware Handshaking
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t hs_sel: This parameter can be one of the following values:
 *         @arg @ref DMAC_CHANNEL_HARDWARE_HS
 *         @arg @ref DMAC_CHANNEL_SOFTWARE_HS
 * @retval None
 */
#define ms_dmac_hal_channel_set_dst_hs_sel(dmac,hs_sel) ms_dmac_ll_channel_set_dst_hs_sel((dmac),(hs_sel))

/**
 * @brief  Set DMAC Channel Source Handshaking Interface Polarity
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t polarity: This parameter can be one of the following values:
 *         @arg @ref DMAC_INTERFACE_POLARITY_ACTIVE_HIGH
 *         @arg @ref DMAC_INTERFACE_POLARITY_ACTIVE_LOW
 * @retval None
 */
#define ms_dmac_hal_channel_set_src_hs_polarity(dmac,polarity) ms_dmac_ll_channel_set_src_hs_polarity((dmac),(polarity))

/**
 * @brief  Set DMAC Channel Destination Handshaking Interface Polarity
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t polarity: This parameter can be one of the following values:
 *         @arg @ref DMAC_INTERFACE_POLARITY_ACTIVE_HIGH
 *         @arg @ref DMAC_INTERFACE_POLARITY_ACTIVE_LOW
 * @retval None
 */
#define ms_dmac_hal_channel_set_dst_hs_polarity(dmac,polarity) ms_dmac_ll_channel_set_dst_hs_polarity((dmac),(polarity))

/**
 * @brief  Set DMAC Channel Automatic Source Reload
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t reload: This parameter can be one of the following values:
 *         @arg @ref DMAC_AUTOMATIC_RELOAD_DISABLE
 *         @arg @ref DMAC_AUTOMATIC_RELOAD_ENABLE
 * @retval None
 */
#define ms_dmac_hal_channel_set_src_auto_reload(dmac,reload) ms_dmac_ll_channel_set_src_auto_reload((dmac),(reload))

/**
 * @brief  Set DMAC Channel Automatic Destination Reload
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t reload: This parameter can be one of the following values:
 *         @arg @ref DMAC_AUTOMATIC_RELOAD_DISABLE
 *         @arg @ref DMAC_AUTOMATIC_RELOAD_ENABLE
 * @retval None
 */
#define ms_dmac_hal_channel_set_dst_auto_reload(dmac,reload) ms_dmac_ll_channel_set_dst_auto_reload((dmac),(reload))

/**
 * @brief  Set DMAC Channel Flow Control Mode
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t fcmode: This parameter can be one of the following values:
 *         @arg @ref DMAC_FCMODE_PREFETCHING_ENABLE
 *         @arg @ref DMAC_FCMODE_PREFETCHING_DISABLE
 * @retval None
 */
#define ms_dmac_hal_channel_set_fcmode(dmac,fcmode) ms_dmac_ll_channel_set_fcmode((dmac),(fcmode))

/**
 * @brief  Set DMAC Channel FIFO Mode Select
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t fifo_mode: This parameter can be one of the following values:
 *         @arg @ref DMAC_FIFO_MODE_0
 *         @arg @ref DMAC_FIFO_MODE_1
 * @retval None
 */
#define ms_dmac_hal_channel_set_fifo_mode(dmac,fifo_mode) ms_dmac_ll_channel_set_fifo_mode((dmac),(fifo_mode))

/**
 * @brief  Set DMAC Channel Source Status Update Enable
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t enable: This parameter can be one of the following values:
 *         @arg @ref DMAC_STATUS_UPDATE_DISABLE
 *         @arg @ref DMAC_STATUS_UPDATE_ENABLE
 * @retval None
 */
#define ms_dmac_hal_channel_set_src_st_upd_en(dmac,enable) ms_dmac_ll_channel_set_src_st_upd_en((dmac),(enable))

/**
 * @brief  Set DMAC Channel Destination Status Update Enable
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t enable: This parameter can be one of the following values:
 *         @arg @ref DMAC_STATUS_UPDATE_DISABLE
 *         @arg @ref DMAC_STATUS_UPDATE_ENABLE
 * @retval None
 */
#define ms_dmac_hal_channel_set_dst_st_upd_en(dmac,enable) ms_dmac_ll_channel_set_dst_st_upd_en((dmac),(enable))

/**
 * @brief  Set DMAC Channel Protection Control
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t protect_ctrl:
 * @retval None
 */
#define ms_dmac_hal_channel_set_protect_ctrl(dmac,protect_ctrl) ms_dmac_ll_channel_set_protect_ctrl((dmac),(protect_ctrl))

/**
 * @brief  Set DMAC Channel Source Hardware Interface.
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t itf_index:
 * @retval None
 */
#define ms_dmac_hal_channel_set_src_itf_index(dmac,itf_index) ms_dmac_ll_channel_set_src_itf_index((dmac),(itf_index))

/**
 * @brief  Set DMAC Channel Destination hardware interface.
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t itf_index:
 * @retval None
 */
#define ms_dmac_hal_channel_set_dst_itf_index(dmac,itf_index) ms_dmac_ll_channel_set_dst_itf_index((dmac),(itf_index))

/**
 * @brief  Set DMAC Source Burst Transaction Length
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t msize: Number of data items. This parameter can be one of the following values:
 *         @arg @ref DMAC_MSIZE_1
 *         @arg @ref DMAC_MSIZE_4
 *         @arg @ref DMAC_MSIZE_8
 *         @arg @ref DMAC_MSIZE_16
 *         @arg @ref DMAC_MSIZE_32
 * @note   src_single_size_bytes =  CTLx.SRC_TR_WIDTH/8
 *         src_burst_size_bytes = CTLx.SRC_MSIZE * src_single_size_bytes
 * @retval None
 */
#define ms_dmac_hal_channel_set_src_msize(dmac,msize) ms_dmac_ll_channel_set_src_msize((dmac),(msize))

/**
 * @brief  Set DMAC Destination Burst Transaction Length
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t msize: Number of data items. This parameter can be one of the following values:
 *         @arg @ref DMAC_MSIZE_1
 *         @arg @ref DMAC_MSIZE_4
 *         @arg @ref DMAC_MSIZE_8
 *         @arg @ref DMAC_MSIZE_16
 *         @arg @ref DMAC_MSIZE_32
 * @note   dst_single_size_bytes = CTLx.DSTTR_WIDTH/8
 *         dst_burst_size_bytes = CTLx.DST_MSIZE * dst_single_size_bytes
 * @retval None
 */
#define ms_dmac_hal_channel_set_dst_msize(dmac,msize) ms_dmac_ll_channel_set_dst_msize((dmac),(msize))

/**
 * @brief  Set DMAC Block Transfer Size
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t block_size: the total number of single transactions to perform for every block transfer
 * @note   single_size_bytes =  transfer_width/8
 *         blk_size_bytes_dma = block_size * single_size_bytes
 * @retval None
 */
#define ms_dmac_hal_channel_set_block_size(dmac,block_size) ms_dmac_ll_channel_set_block_size((dmac),(block_size))

/**
 * @brief  Set DMAC Block Transfer Size
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t block_size: the total number of single transactions to perform for every block transfer
 * @note   single_size_bytes =  transfer_width/8
 *         blk_size_bytes_dma = block_size * single_size_bytes
 * @retval None
 */

#define ms_dmac_hal_channel_calc_block_size(xfer_count,tr_with) ms_dmac_ll_channel_calc_block_size((xfer_count),(tr_with))

/**
 * @brief  Decode DMAC Byte Number by tr_with Value
 * @retval Byte Number
 */
#define ms_dmac_hal_decode_byte_num(tr_with)   ms_dmac_ll_decode_byte_num((tr_with))

/**
 * @brief  Enable DMAC Channel Interrupt
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @retval None
 */
#define ms_dmac_hal_channel_enable_int(dmac) ms_dmac_ll_channel_enable_int((dmac))

/**
 * @brief  Disable DMAC Channel Interrupt
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @retval None
 */
#define ms_dmac_hal_channel_disable_int(dmac) ms_dmac_ll_channel_disable_int((dmac))

/**
 * @brief  Verify DMAC Channel Interrupt Is Enable
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @retval Returned value: Enable or Disable
 */
#define ms_dmac_hal_channel_is_enable_int(dmac) ms_dmac_ll_channel_is_enable_int((dmac))

/**
 * @}
 */

/**
 * @}
 */

#endif /* BLE_SOC_XXX_MS_DMAC_HAL_H_ */
