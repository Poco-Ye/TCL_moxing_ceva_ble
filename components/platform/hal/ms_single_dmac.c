/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_single_dmac.c
 * @brief
 * @author bingrui.chen
 * @date 2021-12-30
 * @version 1.0
 * @Revision
 */

#include <ms_clock_hal.h>
#include <ms_interrupt.h>
#include <ms_single_dmac.h>
#include <string.h>
#include "log.h"

/**
 * @brief  The DMAC Channel Handle List
 */
static DmacHandle_Type ms_dmac_handle_list[DMAC_CHANNEL_MAX];

/**
 * @brief  The DMAC Channel Register List
 */
static const DmacChannel_Type *dmac_ch_index_list[] =
{
    DMAC_CHANNLE0,
    DMAC_CHANNLE1,
    DMAC_CHANNLE2,
    DMAC_CHANNLE3,
    DMAC_CHANNLE4,
    DMAC_CHANNLE5
};

/**
 * @brief  Find The Dmac Channel Handler by Channel Index
 * @param[in]  uint32_t channel: 0~5
 * @retval Dmac Channel Handler
 */
static DmacHandle_Type* ms_dmac_find_handle(uint8_t ch)
{
    DmacHandle_Type *handle = NULL;
    if (ch < DMAC_CHANNEL_MAX)
    {
        handle = &ms_dmac_handle_list[ch];
    }
    return handle;
}

/**
 * @brief  Disable DMA Channel x
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval none
 */
void ms_dmac_clear_all_int(DmacHandle_Type *hdmac)
{
    CHECK_PTR_NULL(hdmac);
    ms_dmac_hal_clear_tfr_int(hdmac->dma_instance, hdmac->channel_index);
    ms_dmac_hal_clear_block_int(hdmac->dma_instance, hdmac->channel_index);
    ms_dmac_hal_clear_src_tfr_int(hdmac->dma_instance, hdmac->channel_index);
    ms_dmac_hal_clear_dst_tfr_int(hdmac->dma_instance, hdmac->channel_index);
    ms_dmac_hal_clear_err_it(hdmac->dma_instance, hdmac->channel_index);
}

/**
 * @brief  DMAC Transfer Complete Interrupt Request
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t max_ch_num:  the max channel number
 * @retval none
 */
static void ms_dmac_tfr_int_irq(Dmac_Type *dmac, uint32_t max_ch_num)
{
    CHECK_PTR_NULL(dmac);

    for (uint32_t ch = 0; ch < max_ch_num; ch++)
    {
        if (ms_dmac_hal_ch_int_is_tfr_active(DMAC, ch))
        {
            /*get dmac handler*/
            DmacHandle_Type *hdma = ms_dmac_find_handle(ch);

            if (hdma && hdma->xfer_tfr_callback)
            {
                hdma->busy[ch] = false;
                hdma->xfer_tfr_callback(hdma);
            }

            ms_dmac_hal_clear_tfr_int(dmac, ch);
            break;
        }
    }
}

/**
 * @brief  DMAC Block Transfer Complete Interrupt Request
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t max_ch_num:  the max channel number
 * @retval none
 */
static void ms_dmac_block_int_irq(Dmac_Type *dmac, uint32_t max_ch_num)
{
    CHECK_PTR_NULL(dmac);
    for (uint32_t ch = 0; ch < max_ch_num; ch++)
    {
        if (ms_dmac_hal_ch_int_is_block_active(DMAC, ch))
        {
            /*get dmac handler*/
            DmacHandle_Type *hdma = ms_dmac_find_handle(ch);
//            if (hdma && hdma->xfer_block_callback)
//            {
//                hdma->xfer_block_callback(hdma);
//            }

            ms_dmac_hal_clear_block_int(dmac, ch);
            break;
        }
    }
}

/**
 * @brief  DMAC Source Transfer Complete Interrupt Request
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t max_ch_num:  the max channel number
 * @retval none
 */
static void ms_dmac_src_int_irq(Dmac_Type *dmac, uint32_t max_ch_num)
{
    CHECK_PTR_NULL(dmac);
    for (uint32_t ch = 0; ch < max_ch_num; ch++)
    {
        if (ms_dmac_hal_ch_int_is_src_tfr_active(DMAC, ch))
        {
            ms_dmac_hal_clear_src_tfr_int(dmac, ch);
            break;
        }
    }
}

/**
 * @brief  DMAC Destination Transfer Complete Interrupt Request
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t max_ch_num:  the max channel number
 * @retval none
 */
static void ms_dmac_dst_int_irq(Dmac_Type *dmac, uint32_t max_ch_num)
{
    CHECK_PTR_NULL(dmac);
    for (uint32_t ch = 0; ch < max_ch_num; ch++)
    {
        if (ms_dmac_hal_ch_int_is_dst_tfr_active(DMAC, ch))
        {
            ms_dmac_hal_clear_dst_tfr_int(dmac, ch);
            break;
        }
    }
}

/**
 * @brief  DMAC Error Transfer Complete Interrupt Request
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t max_ch_num:  the max channel number
 * @retval none
 */
static void ms_dmac_error_int_irq(Dmac_Type *dmac, uint32_t max_ch_num)
{
    CHECK_PTR_NULL(dmac);
    for (uint32_t ch = 0; ch < max_ch_num; ch++)
    {
        if (ms_dmac_hal_ch_int_is_error_active(DMAC, ch))
        {
            /*get dmac handler*/
            DmacHandle_Type *hdma = ms_dmac_find_handle(ch);
            if (hdma && hdma->xfer_error_callback)
            {
                hdma->xfer_error_callback(hdma);
            }

            ms_dmac_hal_clear_err_it(dmac, ch);
            break;
        }
    }
}

void ms_dmac_mgmt_inti(void)
{
    for (uint32_t i = 0; i < DMAC_CHANNEL_MAX; i++)
    {
        memset(&ms_dmac_handle_list[i], 0, sizeof(DmacHandle_Type));
    }
}

DmacHandle_Type* ms_dmac_mgmt_alloc(void)
{
    DmacHandle_Type *handle = NULL;
    for (uint32_t i = 0; i < DMAC_CHANNEL_MAX; i++)
    {
        if (ms_dmac_handle_list[i].state == 0)
        {
            handle = &ms_dmac_handle_list[i];
            handle->channel_index = i;
            handle->dmac_ch_instance = (DmacChannel_Type*) dmac_ch_index_list[i];
            handle->dma_instance = DMAC;
            handle->state = 1;
            break;
        }
    }
    return handle;
}

void ms_dmac_mgmt_free(DmacHandle_Type *hdmac)
{
    CHECK_PTR_NULL(hdmac);
    hdmac->state = 0;
    ms_dmac_hal_disable_channel(hdmac->dma_instance, ((uint32_t )hdmac->channel_index));
}

int32_t ms_dmac_init(DmacHandle_Type *hdmac)
{
    CHECK_PTR_NULL_RET(hdmac, STATUS_ERROR);

    /*dmac clock enable*/
    MS_CLOCK_HAL_CLK_ENABLE_DMA();
    ms_dmac_hal_enable(hdmac->dma_instance);

    /*Clear any pending interrupts */
    ms_dmac_clear_all_int(hdmac);

    /*set default value for Single-block Transfer*/
    ms_dmac_hal_channel_set_llp(hdmac->dmac_ch_instance, 0x00);

    /*Set up the transfer type*/
    int32_t periph_cfg_val = 0;
    if (hdmac->init.src_peri_type == DMAC_MEM && hdmac->init.dst_peri_type == DMAC_MEM)
    {
        ms_dmac_hal_channel_set_transfer_type(hdmac->dmac_ch_instance, DMAC_XFER_TYPE_MEM_TO_MEM);
        periph_cfg_val = DMAC_MEM;
    }
    else if (hdmac->init.src_peri_type != DMAC_MEM && hdmac->init.dst_peri_type == DMAC_MEM)
    {
        ms_dmac_hal_channel_set_transfer_type(hdmac->dmac_ch_instance, DMAC_XFER_TYPE_PERI_TO_MEM);
        periph_cfg_val = hdmac->init.src_peri_type;
    }
    else if (hdmac->init.src_peri_type == DMAC_MEM && hdmac->init.dst_peri_type != DMAC_MEM)
    {
        ms_dmac_hal_channel_set_transfer_type(hdmac->dmac_ch_instance, DMAC_XFER_TYPE_MEM_TO_PERI);
        periph_cfg_val = hdmac->init.dst_peri_type;
    }

    /*Set up the Transfer width*/
    ms_dmac_hal_channel_set_src_transfer_width(hdmac->dmac_ch_instance, hdmac->init.src_tr_width);
    ms_dmac_hal_channel_set_dst_transfer_width(hdmac->dmac_ch_instance, hdmac->init.dst_tr_width);

    /*Set up the Incrementing/decrementing or fixed address*/
    ms_dmac_hal_channel_set_src_addr_mode(hdmac->dmac_ch_instance, hdmac->init.src_addr_mode);
    ms_dmac_hal_channel_set_dst_addr_mode(hdmac->dmac_ch_instance, hdmac->init.dst_addr_mode);

    /*Set up the the msize*/
    ms_dmac_hal_channel_set_src_msize(hdmac->dmac_ch_instance, hdmac->init.src_msize);
    ms_dmac_hal_channel_set_dst_msize(hdmac->dmac_ch_instance, hdmac->init.dst_msize);

    /*Write the channel configuration information into the CFGx register for channel x*/
    ms_dmac_hal_channel_set_fcmode(hdmac->dmac_ch_instance, DMAC_FCMODE_PREFETCHING_ENABLE);
    ms_dmac_hal_channel_set_fifo_mode(hdmac->dmac_ch_instance, DMAC_FIFO_MODE_0);
    ms_dmac_hal_channel_set_protect_ctrl(hdmac->dmac_ch_instance, 1);
    ms_dmac_hal_channel_set_priority(hdmac->dmac_ch_instance, hdmac->init.priority);

//    ms_dmac_hal_channel_set_src_st_upd_en(hdmac->dmac_ch_instance, DMAC_STATUS_UPDATE_ENABLE);
//    ms_dmac_hal_channel_set_dst_st_upd_en(hdmac->dmac_ch_instance, DMAC_STATUS_UPDATE_ENABLE);

     if (periph_cfg_val != DMAC_MEM)
    {
        ms_dmac_hal_channel_set_src_hs_polarity(hdmac->dmac_ch_instance, DMAC_INTERFACE_POLARITY_ACTIVE_HIGH);
        ms_dmac_hal_channel_set_dst_hs_polarity(hdmac->dmac_ch_instance, DMAC_INTERFACE_POLARITY_ACTIVE_HIGH);

        ms_dmac_hal_channel_set_src_hs_sel(hdmac->dmac_ch_instance, DMAC_CHANNEL_HARDWARE_HS);
        ms_dmac_hal_channel_set_dst_hs_sel(hdmac->dmac_ch_instance, DMAC_CHANNEL_HARDWARE_HS);

        ms_dmac_hal_channel_set_src_itf_index(hdmac->dmac_ch_instance, periph_cfg_val);
        ms_dmac_hal_channel_set_dst_itf_index(hdmac->dmac_ch_instance, periph_cfg_val);
    }

    ms_dmac_hal_channel_disable_int(hdmac->dmac_ch_instance);

    return STATUS_SUCCESS;
}

int32_t ms_dmac_deinit(DmacHandle_Type *hdmac)
{
    CHECK_PTR_NULL_RET(hdmac, STATUS_ERROR);

    return STATUS_SUCCESS;
}

int32_t ms_dmac_start_chx_xfer(DmacHandle_Type *hdmac, uint32_t src_addr, uint32_t dst_addr, uint16_t count)
{
    CHECK_PTR_NULL_RET(hdmac, STATUS_ERROR);
    INTERRUPT_DISABLE_IRQ(DMAC_IRQn);
    ms_dmac_hal_channel_disable_int(hdmac->dmac_ch_instance);

    /*Write the starting source address in the SARx register and  the DARx register */
    ms_dmac_hal_channel_set_src_addr(hdmac->dmac_ch_instance, src_addr);
    ms_dmac_hal_channel_set_dst_addr(hdmac->dmac_ch_instance, dst_addr);

    /*dmac_channel_set_block_size*/
    //uint32_t block_size = ms_dmac_hal_channel_calc_block_size(count, hdmac->init.src_tr_width);
    uint32_t block_size =count;
    ms_dmac_hal_channel_set_block_size(hdmac->dmac_ch_instance, block_size);

    /*Clear any pending interrupts */
    ms_dmac_clear_all_int(hdmac);

    /*Ensure that bit 0 of the DmaCfgReg register is enabled before writing to ChEnReg.*/
    // ms_dmac_hal_enable_src_soft_req(hdma->dma_instance, hdma->channel_index);
    /*Enable dma*/
    //ms_dmac_hal_enable(hdma->dma_instance);
    ms_dmac_hal_enable_channel(hdmac->dma_instance, hdmac->channel_index);

//    /*Wait foe xfer don */
//    while(!ms_dmac_is_chx_xfer_done(hdma));
//
//    /*Clear any pending interrupts */
//    ms_dmac_clear_all_int(hdma);

    return STATUS_SUCCESS;
}

bool ms_dmac_is_chx_xfer_done(DmacHandle_Type *hdmac)
{
    return ms_dmac_hal_raw_is_tfr_active(hdmac->dma_instance, hdmac->channel_index);
}

int32_t ms_dmac_stop_chx_xfer(DmacHandle_Type *hdmac)
{
    CHECK_PTR_NULL_RET(hdmac, STATUS_ERROR);

    ms_dmac_hal_disable_channel(hdmac->dma_instance, hdmac->channel_index);
    return STATUS_SUCCESS;
}

int32_t ms_dmac_start_chx_xfer_it(DmacHandle_Type *hdmac, uint32_t src_addr, uint32_t dst_addr, uint16_t count)
{
    CHECK_PTR_NULL_RET(hdmac, STATUS_ERROR);

    if(hdmac->busy[hdmac->channel_index] == true)
    {
        return STATUS_BUSY;
    }

    hdmac->busy[hdmac->channel_index] = true;

    /*Write the starting source address in the SARx register and  the DARx register */
    ms_dmac_hal_channel_set_src_addr(hdmac->dmac_ch_instance, src_addr);
    ms_dmac_hal_channel_set_dst_addr(hdmac->dmac_ch_instance, dst_addr);

    /*set dmac_channel_set_block_size*/
    uint32_t block_size = ms_dmac_hal_channel_calc_block_size(count, hdmac->init.src_tr_width);
    ms_dmac_hal_channel_set_block_size(hdmac->dmac_ch_instance, block_size);

    /*Clear any pending interrupts */
    ms_dmac_clear_all_int(hdmac);

    /*Enable the interrupts*/
    INTERRUPT_DISABLE_IRQ(DMAC_IRQn);
    ms_dmac_hal_channel_enable_int(hdmac->dmac_ch_instance);
    INTERRUPT_ENABLE_IRQ(DMAC_IRQn);

    ms_dmac_hal_set_mask_tfr_int(hdmac->dma_instance, hdmac->channel_index, DMAC_INT_UNMASK_VAL);
    //ms_dmac_hal_set_mask_block_int(hdma->dma_instance, hdma->channel_index, DMAC_INT_UNMASK_VAL);
    //ms_dmac_hal_set_mask_src_tfr_int(hdma->dma_instance, hdma->channel_index, DMAC_INT_UNMASK_VAL);
    // ms_dmac_hal_set_mask_dst_tfr_int(hdma->dma_instance, hdma->channel_index, DMAC_INT_UNMASK_VAL);
    ms_dmac_hal_set_mask_err_int(hdmac->dma_instance, hdmac->channel_index, DMAC_INT_UNMASK_VAL);

    /*Ensure that bit 0 of the DmaCfgReg register is enabled before writing to ChEnReg.*/
    ms_dmac_hal_enable(hdmac->dma_instance);
    ms_dmac_hal_enable_channel(hdmac->dma_instance, hdmac->channel_index);

    return STATUS_SUCCESS;
}

int32_t ms_dmac_stop_chx_xfer_it(DmacHandle_Type *hdmac)
{
    CHECK_PTR_NULL_RET(hdmac, STATUS_ERROR);

    ms_dmac_hal_disable_channel(hdmac->dma_instance, hdmac->channel_index);
    ms_dmac_hal_set_mask_tfr_int(hdmac->dma_instance, hdmac->channel_index, DMAC_INT_MASK_VAL);
    ms_dmac_hal_set_mask_block_int(hdmac->dma_instance, hdmac->channel_index, DMAC_INT_MASK_VAL);
    ms_dmac_hal_set_mask_src_tfr_int(hdmac->dma_instance, hdmac->channel_index, DMAC_INT_MASK_VAL);
    ms_dmac_hal_set_mask_dst_tfr_int(hdmac->dma_instance, hdmac->channel_index, DMAC_INT_MASK_VAL);
    ms_dmac_hal_set_mask_err_int(hdmac->dma_instance, hdmac->channel_index, DMAC_INT_MASK_VAL);

    return STATUS_SUCCESS;
}

int32_t ms_dmac_register_cb(DmacHandle_Type *hdmac, DmacCallbackID_Type callback_id,
        void (*p_callback)(DmacHandle_Type *_hdma))
{
    CHECK_PTR_NULL_RET(hdmac, STATUS_ERROR);
    CHECK_PTR_NULL_RET(p_callback, STATUS_ERROR);

    switch (callback_id)
    {
    case DMAC_CB_ID_TFR:
        hdmac->xfer_tfr_callback = p_callback;
        break;
    case DMAC_CB_ID_BLK:
//        hdma->xfer_block_callback = p_callback;
        break;
    case DMAC_CB_ID_SRC_TFR:
        /*todo : to be implement SRC_TFR*/
        break;
    case DMAC_CB_ID_DST_TFR:
        /*todo : to be implement DST_TFR*/
        break;
    case DMAC_CB_ID_ERR:
        hdmac->xfer_error_callback = p_callback;
        break;
    }
    return STATUS_SUCCESS;
}

void ms_dmac_unregister_cb(DmacHandle_Type *hdmac, DmacCallbackID_Type callback_id)
{
    CHECK_PTR_NULL(hdmac);

    switch (callback_id)
    {
    case DMAC_CB_ID_TFR:
        hdmac->xfer_tfr_callback = NULL;
        break;
    case DMAC_CB_ID_BLK:
//        hdma->xfer_block_callback = NULL;
        break;
    case DMAC_CB_ID_SRC_TFR:
        /*todo : to be implement SRC_TFR*/
        break;
    case DMAC_CB_ID_DST_TFR:
        /*todo : to be implement DST_TFR*/
        break;
    case DMAC_CB_ID_ERR:
        hdmac->xfer_error_callback = NULL;
        break;
    }
}

uint32_t ms_ms_dmac_hal_get_error(DmacHandle_Type *hdma)
{
    return hdma->error_code;
}

/**
 * @brief  DMAC Interrupt Request Handler
 */
void DMAC_IRQHandler(void)
{
    if (ms_dmac_hal_combi_is_tfr_active(DMAC))
    {
        ms_dmac_tfr_int_irq(DMAC, DMAC_CHANNEL_MAX);
    }

    if (ms_dmac_hal_combi_is_block_active(DMAC))
    {
        ms_dmac_block_int_irq(DMAC, DMAC_CHANNEL_MAX);
    }

    if (ms_dmac_hal_combi_is_src_active(DMAC))
    {
        ms_dmac_src_int_irq(DMAC, DMAC_CHANNEL_MAX);
    }

    if (ms_dmac_hal_combi_is_dst_active(DMAC))
    {
        ms_dmac_dst_int_irq(DMAC, DMAC_CHANNEL_MAX);
    }

    if (ms_dmac_hal_combi_is_error_active(DMAC))
    {
        ms_dmac_error_int_irq(DMAC, DMAC_CHANNEL_MAX);
    }
}
