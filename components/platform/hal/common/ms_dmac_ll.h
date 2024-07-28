/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file   ms_dmac_ll.h
 * @brief Header file of DMA  module.
 * @author bingrui.chen
 * @date   2021-12-21
 * @version 1.0
 * @Revision
 */

#ifndef MS_DMAC_LL_H_
#define MS_DMAC_LL_H_

#include <ms1008.h>
#include <stdint.h>
#include <stdbool.h>

/** @addtogroup MS_LL_Driver
 * @{
 */

/** @defgroup DMAC_LL DMAC
 * @{
 */

/** @defgroup DMAC_XFER_TYPE DMAC Channel Transfer Type definition
 * @{
 */
#define     DMAC_XFER_TYPE_MEM_TO_MEM               (0x00000000UL)
#define     DMAC_XFER_TYPE_MEM_TO_PERI              (0x00000001UL)
#define     DMAC_XFER_TYPE_PERI_TO_MEM              (0x00000002UL)
/**
 * @}
 */

/** @defgroup DMAC_MSIZE DMAC Channel Burst Transaction Length MSIZE definition
 * @{
 */
#define     DMAC_MSIZE_1                            (0x00000000UL)
#define     DMAC_MSIZE_4                            (0x00000001UL)
#define     DMAC_MSIZE_8                            (0x00000002UL)
#define     DMAC_MSIZE_16                          (0x00000003UL)
#define     DMAC_MSIZE_32                          (0x00000004UL)
#define     DMAC_MSIZE_64                          (0x00000005UL)
#define     DMAC_MSIZE_128                        (0x00000006UL)
#define     DMAC_MSIZE_256                        (0x00000007UL)


/**
 * @}
 */

/** @defgroup DMAC_ADDRESS_MODE DMAC Channel Address Increment or Decrement definition
 * @{
 */
#define     DMAC_ADDRESS_MODE_INC                   (0x00000000UL)
#define     DMAC_ADDRESS_MODE_DEC                   (0x00000001UL)
#define     DMAC_ADDRESS_MODE_NOC                   (0x00000002UL)
/**
 * @}
 */

/** @defgroup DMAC_XFER_WIDTH DMAC Channel Transfer Width definition
 * @{
 */
#define     DMAC_XFER_WIDTH_8BITS                   (0x00000000UL)
#define     DMAC_XFER_WIDTH_16BITS                  (0x00000001UL)
#define     DMAC_XFER_WIDTH_32BITS                  (0x00000002UL)
/**
 * @}
 */

/** @defgroup DMAC_CHANNEL_PRIOR DMAC Channel Priority definition
 * @{
 */
#define     DMAC_CHANNEL_PRIOR_0                    (0x00000000UL)
#define     DMAC_CHANNEL_PRIOR_1                    (0x00000001UL)
#define     DMAC_CHANNEL_PRIOR_2                    (0x00000002UL)
#define     DMAC_CHANNEL_PRIOR_3                    (0x00000003UL)
#define     DMAC_CHANNEL_PRIOR_4                    (0x00000004UL)
#define     DMAC_CHANNEL_PRIOR_5                    (0x00000005UL)
#define     DMAC_CHANNEL_PRIOR_6                    (0x00000006UL)
#define     DMAC_CHANNEL_PRIOR_7                    (0x00000007UL)
/**
 * @}
 */

/** @defgroup DMAC_INTERRUP_MASK_PRIOR DMAC Channel Interrupt Mask definition
 * @{
 */
#define     DMAC_INT_MASK_VAL                       (0x00000100UL)
#define     DMAC_INT_UNMASK_VAL                     (0x00000101UL)
/**
 * @}
 */

/** @defgroup DMAC_FCMODE_PREFETCHING DMAC Channel Pre-fetching definition
 * @{
 */
#define     DMAC_FCMODE_PREFETCHING_ENABLE          (0x00000000UL)
#define     DMAC_FCMODE_PREFETCHING_DISABLE         (0x00000001UL)
/**
 * @}
 */

/** @defgroup DMAC_FIFO_MODE DMAC Channel FIFO Mode definition
 * @{
 */
#define     DMAC_FIFO_MODE_0                        (0x00000000UL)
#define     DMAC_FIFO_MODE_1                        (0x00000001UL)
/**
 * @}
 */

/** @defgroup DMAC_STATUS_UPDATE DMAC Channel Status Update definition
 * @{
 */
#define     DMAC_STATUS_UPDATE_DISABLE               (0x00000000UL)
#define     DMAC_STATUS_UPDATE_ENABLE                (0x00000001UL)
/**
 * @}
 */
/** @defgroup DMAC_INTERFACE_POLARITY DMAC Channel Interface Priority definition
 * @{
 */
#define     DMAC_INTERFACE_POLARITY_ACTIVE_HIGH      (0x00000000UL)
#define     DMAC_INTERFACE_POLARITY_ACTIVE_LOW       (0x00000001UL)
/**
 * @}
 */
/** @defgroup DMAC_CHANNEL_HANDSHAKE DMAC Channel Interface Request Type definition
 * @{
 */
#define     DMAC_CHANNEL_HARDWARE_HS                 (0x00000000UL)
#define     DMAC_CHANNEL_SOFTWARE_HS                 (0x00000001UL)
/**
 * @}
 */

/**
 * @brief  Enable DW_ahb_dmac
 * @param  DMAC_Type *dmac: DMAC Instance
 * @retval None
 */
static inline void ms_dmac_ll_enable(Dmac_Type *dmac)
{
    SET_BIT(dmac->CFGREG_L, DMAC_CFGREG_L_DMA);
}

/**
 * @brief  Disable DW_ahb_dmac
 * @param  DMAC_Type *dmac: DMAC Instance
 * @retval None
 */
static inline void ms_dmac_ll_disable(Dmac_Type *dmac)
{
    CLEAR_BIT(dmac->CFGREG_L, DMAC_CFGREG_L_DMA);
}

/**
 * @brief  Get the DW_ahb_dmac Enable Status
 * @param  DMAC_Type *dmac: DMAC Instance
 * @retval Status of  DW_ahb_dmac Enable( true or false)
 */
static inline bool ms_dmac_ll_is_enable(Dmac_Type *dmac)
{
    return READ_BIT(dmac->CHENREG_L, DMAC_CFGREG_L_DMA) == DMAC_CFGREG_L_DMA;
}

/**
 * @brief  Enable DMA Channel x
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval none
 */
static inline void ms_dmac_ll_enable_channel(Dmac_Type *dmac, uint32_t channel)
{
    WRITE_REG(dmac->CHENREG_L, (0x0101UL << (DMAC_CHENREG_L_CH_EN_POS+channel)));
}

/**
 * @brief  Disable DMA Channel x
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval none
 */
static inline void ms_dmac_ll_disable_channel(Dmac_Type *dmac, uint32_t channel)
{
    WRITE_REG(dmac->CHENREG_L, (0x0100UL << (DMAC_CHENREG_L_CH_EN_POS+channel)));
}

/**
 * @brief  Get DMA Channel x Enable Status
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval enable or disable
 */
static inline bool ms_dmac_ll_is_enable_channel(Dmac_Type *dmac, uint32_t channel)
{
    return (READ_BIT(dmac->CHENREG_L, (0x01UL << (DMAC_CHENREG_L_CH_EN_POS+channel)))
            == (0x01UL << (DMAC_CHENREG_L_CH_EN_POS + channel)));
}

/**
 * @brief  Mask Channel x transfer complete Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
static inline void ms_dmac_ll_set_mask_tfr_int(Dmac_Type *dmac, uint32_t channel, uint32_t mask_val)
{
    uint32_t mask_tmp = mask_val << (channel);
    WRITE_REG(dmac->MASKTFR_L, mask_tmp);
}

/**
 * @brief  Mask Channel x block Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
static inline void ms_dmac_ll_set_mask_block_int(Dmac_Type *dmac, uint32_t channel, uint32_t mask_val)
{
    uint32_t mask_tmp = mask_val << (channel);
    WRITE_REG(dmac->MASKBLOCK_L, mask_tmp);
}

/**
 * @brief  Mask Channel x source transfer complete Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
static inline void ms_dmac_ll_set_mask_src_tfr_int(Dmac_Type *dmac, uint32_t channel, uint32_t mask_val)
{
    uint32_t mask_tmp = mask_val << (channel);
    WRITE_REG(dmac->MASKSRCTRAN_L, mask_tmp);
}
/**
 * @brief  Mask Channel x destination transfer complete Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
static inline void ms_dmac_ll_set_mask_dst_tfr_int(Dmac_Type *dmac, uint32_t channel, uint32_t mask_val)
{
    uint32_t mask_tmp = mask_val << (channel);
    WRITE_REG(dmac->MASKDSTTRAN_L, mask_tmp);
}

/**
 * @brief  Mask Channel x error Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
static inline void ms_dmac_ll_set_mask_err_int(Dmac_Type *dmac, uint32_t channel, uint32_t mask_val)
{
    uint32_t mask_tmp = mask_val << (channel);
    WRITE_REG(dmac->MASKERR_L, mask_tmp);

//    uint32_t mask_tmp = 0x0101UL << (channel);
//    MODIFY_REG(dmac->MASKERR_L, mask_tmp, ((mask_val << channel) & mask_tmp));
}

/**
 * @brief  Clear for transfer complete Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
static inline void ms_dmac_ll_clear_tfr_int(Dmac_Type *dmac, uint32_t channel)
{
    WRITE_REG(dmac->CLEARTFR_L, (0x01UL << (DMAC_CLEARTFR_L_CLEAR_POS+channel)));
}

/**
 * @brief  Clear for block Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
static inline void ms_dmac_ll_clear_block_int(Dmac_Type *dmac, uint32_t channel)
{
    WRITE_REG(dmac->CLEARBLOCK_L, (0x01UL << (DMAC_CLEARBLOCK_L_CLEAR_POS+channel)));
}

/**
 * @brief  Clear for transfer source complete Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
static inline void ms_dmac_ll_clear_src_tfr_int(Dmac_Type *dmac, uint32_t channel)
{
    WRITE_REG(dmac->CLEARSRCTRAN_L, (0x01UL << (DMAC_CLEARSRCTRAN_L_CLEAR_POS+channel)));
}

/**
 * @brief  Clear for transfer destination complete Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
static inline void ms_dmac_ll_clear_dst_tfr_int(Dmac_Type *dmac, uint32_t channel)
{
    WRITE_REG(dmac->CLEARDSTTRAN_L, (0x01UL << (DMAC_CLEARDSTTRAN_L_CLEAR_POS+channel)));
}

/**
 * @brief   Clear for error Interrupt
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
static inline void ms_dmac_ll_clear_err_it(Dmac_Type *dmac, uint32_t channel)
{
    WRITE_REG(dmac->CLEARERR_L, (0x01UL << (DMAC_CLEARERR_L_CLEAR_POS+channel)));
}

/**
 * @brief  Get DMA Channel Transfer complete interrupt status after masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status(true of false)
 */
static inline bool ms_dmac_ll_ch_int_is_tfr_active(Dmac_Type *dmac, uint32_t channel)
{
    return (READ_BIT(dmac->STATUSTFR_L, ((1UL)<<(DMAC_STATUSTFR_L_STATUS_POS+channel)))
            == ((1UL) << (DMAC_STATUSTFR_L_STATUS_POS + channel)));
}

/**
 * @brief  Get DMA Channel Transfer Block complete interrupt status after masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status
 */
static inline bool ms_dmac_ll_ch_int_is_block_active(Dmac_Type *dmac, uint32_t channel)
{
    return (READ_BIT(dmac->STATUSBLOCK_L, ((1UL)<<(DMAC_STATUSBLOCK_L_STATUS_POS+channel)))
            == ((1UL) << (DMAC_STATUSBLOCK_L_STATUS_POS + channel)));
}

/**
 * @brief  Get DMA Channel source Transfer complete interrupt status after masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status(true of false)
 */
static inline bool ms_dmac_ll_ch_int_is_src_tfr_active(Dmac_Type *dmac, uint32_t channel)
{
    return (READ_BIT(dmac->STATUSSRCTRAN_L, ((1UL)<<(DMAC_STATUSTFR_L_STATUS_POS+channel)))
            == ((1UL) << (DMAC_STATUSTFR_L_STATUS_POS + channel)));
}

/**
 * @brief  Get DMA Channel source Transfer complete interrupt status after masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status(true of false)
 */
static inline bool ms_dmac_ll_ch_int_is_dst_tfr_active(Dmac_Type *dmac, uint32_t channel)
{
    return (READ_BIT(dmac->STATUSDSTTRAN_L, ((1UL)<<(DMAC_STATUSDSTTRAN_L_STATUS_POS+channel)))
            == ((1UL) << (DMAC_STATUSDSTTRAN_L_STATUS_POS + channel)));
}

/**
 * @brief  Get DMA Channel Error interrupt status after masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status
 */
static inline bool ms_dmac_ll_ch_int_is_error_active(Dmac_Type *dmac, uint32_t channel)
{
    return (READ_BIT(dmac->STATUSERR_L, ((1UL)<<(DMAC_STATUSERR_L_STATUS_POS+channel)))
            == ((1UL) << (DMAC_STATUSERR_L_STATUS_POS + channel)));
}

/**
 * @brief  Get DMA Channel Transfer complete Combined interrupt status
 * @param  DMAC_Type *dmac: DMAC Instance
 * @retval Inactive or Active Interrupt Status(true of false)
 */
static inline bool ms_dmac_ll_combi_is_tfr_active(Dmac_Type *dmac)
{
    return (READ_BIT(dmac->STATUSINT_L,DMAC_STATUSINT_L_TRF) == DMAC_STATUSINT_L_TRF);
}

/**
 * @brief  Get DMA Channel Transfer Block complete Combined interrupt status
 * @param  DMAC_Type *dmac: DMAC Instance
 * @retval Inactive or Active Interrupt Status
 */
static inline bool ms_dmac_ll_combi_is_block_active(Dmac_Type *dmac)
{
    return (READ_BIT(dmac->STATUSINT_L,DMAC_STATUSINT_L_BLOCK) == DMAC_STATUSINT_L_BLOCK);
}

/**
 * @brief  Get DMA Channel X Error Combined interrupt Status
 * @param  DMAC_Type *dmac: DMAC Instance
 * @retval Inactive or Active Interrupt Status
 */
static inline bool ms_dmac_ll_combi_is_src_active(Dmac_Type *dmac)
{
    return (READ_BIT(dmac->STATUSINT_L,DMAC_STATUSINT_L_SRCT) == DMAC_STATUSINT_L_SRCT);
}

/**
 * @brief  Get DMA Channel X Error Combined interrupt Status
 * @param  DMAC_Type *dmac: DMAC Instance
 * @retval Inactive or Active Interrupt Status
 */
static inline bool ms_dmac_ll_combi_is_dst_active(Dmac_Type *dmac)
{
    return (READ_BIT(dmac->STATUSINT_L,DMAC_STATUSINT_L_DSTT) == DMAC_STATUSINT_L_DSTT);
}

/**
 * @brief  Get DMA Channel X Error Combined interrupt Status
 * @param  DMAC_Type *dmac: DMAC Instance
 * @retval Inactive or Active Interrupt Status
 */
static inline bool ms_dmac_ll_combi_is_error_active(Dmac_Type *dmac)
{
    return (READ_BIT(dmac->STATUSINT_L,DMAC_STATUSINT_L_ERR) == DMAC_STATUSINT_L_ERR);
}

/**
 * @brief  Get DMA Channel Transfer complete interrupt status before  masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status(true of false)
 */
static inline bool ms_dmac_ll_raw_is_tfr_active(Dmac_Type *dmac, uint32_t channel)
{
    return (READ_BIT(dmac->RAWTFR_L, ((1UL)<<(DMAC_RAWTFR_L_RAW_POS+channel)))
            == ((1UL) << (DMAC_RAWTFR_L_RAW_POS + channel)));
}

/**
 * @brief  Get DMA Channel Transfer Block complete interrupt status before  masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status
 */
static inline bool ms_dmac_ll_raw_is_block_active(Dmac_Type *dmac, uint32_t channel)
{
    return (READ_BIT(dmac->RAWBLOCK_L, ((1UL)<<(DMAC_RAWBLOCK_L_RAW_POS+channel)))
            == ((1UL) << (DMAC_RAWBLOCK_L_RAW_POS + channel)));
}
/**
 * @brief  Get DMA Channel X Error interrupt Status  before  masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status
 */
static inline bool ms_dmac_ll_raw_is_src_active(Dmac_Type *dmac, uint32_t channel)
{
    return (READ_BIT(dmac->RAWSRCTRAN_L, ((1UL)<<(DMAC_RAWSRCTRAN_L_RAW_POS+channel)))
            == ((1UL) << (DMAC_RAWSRCTRAN_L_RAW_POS + channel)));
}
/**
 * @brief  Get DMA Channel X Error interrupt Status  before  masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status
 */
static inline bool ms_dmac_ll_raw_is_dst_active(Dmac_Type *dmac, uint32_t channel)
{
    return (READ_BIT(dmac->RAWDSTTRAN_L, ((1UL)<<(DMAC_RAWDSTTRAN_L_RAW_POS+channel)))
            == ((1UL) << (DMAC_RAWDSTTRAN_L_RAW_POS + channel)));
}
/**
 * @brief  Get DMA Channel X Error interrupt Status  before  masking
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval Inactive or Active Interrupt Status
 */
static inline bool ms_dmac_ll_raw_is_error_active(Dmac_Type *dmac, uint32_t channel)
{
    return (READ_BIT(dmac->RAWERR_L, ((1UL)<<(DMAC_RAWERR_L_RAW_POS+channel)))
            == ((1UL) << (DMAC_RAWERR_L_RAW_POS + channel)));
}

/**
 * @brief  Enable Source Software Transaction Request
 * @param  DMAC_Type *dmac:  DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
static inline void ms_dmac_ll_enable_src_soft_req(Dmac_Type *dmac, uint32_t channel)
{
    WRITE_REG(dmac->REQSRCREG_L, (0x0101UL << channel));
}

/**
 * @brief  Disable Source Software Transaction Request
 * @param  DMAC_Type *dmac:  DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
static inline void ms_dmac_ll_disable_src_soft_req(Dmac_Type *dmac, uint32_t channel)
{
    WRITE_REG(dmac->REQSRCREG_L, (0x0100UL << channel));
}

/**
 * @brief  Enable Destination Software Transaction Request
 * @param  DMAC_Type *dmac:  DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
static inline void ms_dmac_ll_enable_dst_soft_req(Dmac_Type *dmac, uint32_t channel)
{
    WRITE_REG(dmac->REQDSTREG_L, (0x0101UL << channel));
}

/**
 * @brief  Disable Destination Software Transaction Request
 * @param  DMAC_Type *dmac:  DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
static inline void ms_dmac_ll_disable_dst_soft_req(Dmac_Type *dmac, uint32_t channel)
{
    WRITE_REG(dmac->REQDSTREG_L, (0x0100UL << channel));
}

/**
 * @brief  Active Source Single Transaction Request
 * @param  DMAC_Type *dmac:  DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
static inline void ms_dmac_ll_active_src_single_req(Dmac_Type *dmac, uint32_t channel)
{
    SET_BIT(dmac->SGLRQQSRCREG_L, (0x0101UL << channel));
}

/**
 * @brief  Active Destination Single Transaction Request
 * @param  DMAC_Type *dmac:  DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
static inline void ms_dmac_ll_active_dst_single_req(Dmac_Type *dmac, uint32_t channel)
{
    WRITE_REG(dmac->SGLRQQDSTREG_L, (0x0101UL << channel));
}

/**
 * @brief  Active Source Last Transaction Request register
 * @param  DMAC_Type *dmac:  DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
static inline void ms_dmac_ll_active_src_last_req(Dmac_Type *dmac, uint32_t channel)
{
    WRITE_REG(dmac->LSTSRCREG_L, (0x0101UL << channel));
}

/**
 * @brief  Active Destination Last Transaction Request
 * @param  DMAC_Type *dmac:  DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
static inline void ms_dmac_ll_active_dst_last_req(Dmac_Type *dmac, uint32_t channel)
{
    WRITE_REG(dmac->LSTDSTREG_L, (0x0101UL << channel));
}

/**
 * @brief  Set DMAC Channel Transfer Type and Flow Control.
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t type: This parameter can be one of the following values:
 *         @arg @ref DMAC_XFER_TYPE_MEM_TO_MEM
 *         @arg @ref DMAC_XFER_TYPE_MEM_TO_PERI
 *         @arg @ref DMAC_XFER_TYPE_PERI_TO_MEM
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_transfer_type(DmacChannel_Type *dmac, uint32_t type)
{
    MODIFY_REG(dmac->CTL_L, DMAC_CHNNL_CTL_L_TT_FC, (type << DMAC_CHNNL_CTL_L_TT_FC_POS) & DMAC_CHNNL_CTL_L_TT_FC_MASK);
}

/**
 * @brief  Set DMAC Channel Transfer Source Address
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t addr: Transfer Source Address
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_src_addr(DmacChannel_Type *dmac, uint32_t addr)
{
    WRITE_REG(dmac->SAR_L, addr);
}

/**
 * @brief  Set DMAC Channel Transfer Destination Address
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t addr: Transfer Destination Address
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_dst_addr(DmacChannel_Type *dmac, uint32_t addr)
{
    WRITE_REG(dmac->DAR_L, addr);
}

/**
 * @brief  Set DMAC Channel Transfer Destination Address
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t addr: Transfer Destination Address
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_llp(DmacChannel_Type *dmac, uint64_t llp)
{
    WRITE_REG(dmac->LLP_L, llp);
    WRITE_REG(dmac->LLP_H, llp >> 32);
}

/**
 * @brief  Set DMAC Channel Source Transfer Width
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t width: This parameter can be one of the following values:
 *         @arg @ref DMAC_XFER_WIDTH_8BITS
 *         @arg @ref DMAC_XFER_WIDTH_16BITS
 *         @arg @ref DMAC_XFER_WIDTH_32BITS
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_src_transfer_width(DmacChannel_Type *dmac, uint32_t width)
{
    MODIFY_REG(dmac->CTL_L, DMAC_CHNNL_CTL_L_SRC_TR_WIDTH,
            (width <<DMAC_CHNNL_CTL_L_SRC_TR_WIDTH_POS) & DMAC_CHNNL_CTL_L_SRC_TR_WIDTH_MASK);
}

/**
 * @brief  Set DMAC Channel Destination Transfer Width
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t addr_mode: This parameter can be one of the following values:
 *         @arg @ref DMAC_XFER_WIDTH_8BITS
 *         @arg @ref DMAC_XFER_WIDTH_16BITS
 *         @arg @ref DMAC_XFER_WIDTH_32BITS
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_dst_transfer_width(DmacChannel_Type *dmac, uint32_t width)
{
    MODIFY_REG(dmac->CTL_L, DMAC_CHNNL_CTL_L_DST_TR_WIDTH,
            (width <<DMAC_CHNNL_CTL_L_DST_TR_WIDTH_POS) & DMAC_CHNNL_CTL_L_DST_TR_WIDTH_MASK);
}

/**
 * @brief  Set DMAC Channel Destination Address Increment
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t addr_mode: This parameter can be one of the following values:
 *         @arg @ref DMAC_ADDRESS_MODE_INC
 *         @arg @ref DMAC_ADDRESS_MODE_DEC
 *         @arg @ref DMAC_ADDRESS_MODE_NOC
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_dst_addr_mode(DmacChannel_Type *dmac, uint32_t addr_mode)
{
    MODIFY_REG(dmac->CTL_L, DMAC_CHNNL_CTL_L_DINC,
            (addr_mode <<DMAC_CHNNL_CTL_L_DINC_POS) & DMAC_CHNNL_CTL_L_DINC_MASK);
}

/**
 * @brief  Set DMAC Channel Source Address Increment
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t addr_mode: This parameter can be one of the following values:
 *         @arg @ref DMAC_ADDRESS_MODE_INC
 *         @arg @ref DMAC_ADDRESS_MODE_DEC
 *         @arg @ref DMAC_ADDRESS_MODE_NOC
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_src_addr_mode(DmacChannel_Type *dmac, uint32_t addr_mode)
{
    MODIFY_REG(dmac->CTL_L, DMAC_CHNNL_CTL_L_SINC,
            (addr_mode <<DMAC_CHNNL_CTL_L_SINC_POS) & DMAC_CHNNL_CTL_L_SINC_MASK);
}

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
static inline void ms_dmac_ll_channel_set_priority(DmacChannel_Type *dmac, uint32_t priority)
{
    MODIFY_REG(dmac->CFG_L, DMAC_CHNNL_CFG_L_CH_PRIOR,
            (priority <<DMAC_CHNNL_CFG_L_CH_PRIOR_POS) & DMAC_CHNNL_CFG_L_CH_PRIOR_MASK);
}

/**
 * @brief  Select DMAC Channel Source Software or Hardware Handshaking
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t hs_sel: This parameter can be one of the following values:
 *         @arg @ref DMAC_CHANNEL_HARDWARE_HS
 *         @arg @ref DMAC_CHANNEL_SOFTWARE_HS
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_src_hs_sel(DmacChannel_Type *dmac, uint32_t hs_sel)
{
    MODIFY_REG(dmac->CFG_L, DMAC_CHNNL_CFG_L_HS_SEL_SRC,
            (hs_sel <<DMAC_CHNNL_CFG_L_HS_SEL_SRC_POS) & DMAC_CHNNL_CFG_L_HS_SEL_SRC_MASK);
}

/**
 * @brief  Select Destination Software or Hardware Handshaking
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t hs_sel: This parameter can be one of the following values:
 *         @arg @ref DMAC_CHANNEL_HARDWARE_HS
 *         @arg @ref DMAC_CHANNEL_SOFTWARE_HS
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_dst_hs_sel(DmacChannel_Type *dmac, uint32_t hs_sel)
{
    MODIFY_REG(dmac->CFG_L, DMAC_CHNNL_CFG_L_HS_SEL_DST,
            (hs_sel <<DMAC_CHNNL_CFG_L_HS_SEL_DST_POS) & DMAC_CHNNL_CFG_L_HS_SEL_DST_MASK);
}

/**
 * @brief  Set DMAC Channel Source Handshaking Interface Polarity
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t polarity: This parameter can be one of the following values:
 *         @arg @ref DMAC_INTERFACE_POLARITY_ACTIVE_HIGH
 *         @arg @ref DMAC_INTERFACE_POLARITY_ACTIVE_LOW
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_src_hs_polarity(DmacChannel_Type *dmac, uint32_t polarity)
{
    MODIFY_REG(dmac->CFG_L, DMAC_CHNNL_CFG_L_SRC_HS_POL,
            (polarity <<DMAC_CHNNL_CFG_L_SRC_HS_POL_POS) & DMAC_CHNNL_CFG_L_SRC_HS_POL_MASK);
}

/**
 * @brief  Set DMAC Channel Destination Handshaking Interface Polarity
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t polarity: This parameter can be one of the following values:
 *         @arg @ref DMAC_INTERFACE_POLARITY_ACTIVE_HIGH
 *         @arg @ref DMAC_INTERFACE_POLARITY_ACTIVE_LOW
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_dst_hs_polarity(DmacChannel_Type *dmac, uint32_t polarity)
{
    MODIFY_REG(dmac->CFG_L, DMAC_CHNNL_CFG_L_DST_HS_POL,
            (polarity << DMAC_CHNNL_CFG_L_DST_HS_POL_POS) & DMAC_CHNNL_CFG_L_DST_HS_POL_MASK);
}

/**
 * @brief  Set DMAC Channel Automatic Source Reload
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t reload: This parameter can be one of the following values:
 *         @arg @ref DMAC_AUTOMATIC_RELOAD_DISABLE
 *         @arg @ref DMAC_AUTOMATIC_RELOAD_ENABLE
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_src_auto_reload(DmacChannel_Type *dmac, uint32_t reload)
{
    MODIFY_REG(dmac->CFG_L, DMAC_CHNNL_CFG_L_RELOAD_SRC,
            (reload <<DMAC_CHNNL_CFG_L_RELOAD_SRC_POS) & DMAC_CHNNL_CFG_L_RELOAD_SRC_MASK);
}

/**
 * @brief  Set DMAC Channel Automatic Destination Reload
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t reload: This parameter can be one of the following values:
 *         @arg @ref DMAC_AUTOMATIC_RELOAD_DISABLE
 *         @arg @ref DMAC_AUTOMATIC_RELOAD_ENABLE
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_dst_auto_reload(DmacChannel_Type *dmac, uint32_t reload)
{
    MODIFY_REG(dmac->CFG_L, DMAC_CHNNL_CFG_L_RELOAD_DST,
            (reload << DMAC_CHNNL_CFG_L_RELOAD_DST_POS) & DMAC_CHNNL_CFG_L_RELOAD_DST_MASK);
}

/**
 * @brief  Set DMAC Channel Flow Control Mode
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t fcmode: This parameter can be one of the following values:
 *         @arg @ref DMAC_FCMODE_PREFETCHING_ENABLE
 *         @arg @ref DMAC_FCMODE_PREFETCHING_DISABLE
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_fcmode(DmacChannel_Type *dmac, uint32_t fcmode)
{
    MODIFY_REG(dmac->CFG_H, DMAC_CHNNL_CFG_H_FCMODE,
            (fcmode <<DMAC_CHNNL_CFG_H_FCMODE_POS) & DMAC_CHNNL_CFG_H_FCMODE_MASK);
}

/**
 * @brief  Set DMAC Channel FIFO Mode Select
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t fifo_mode: This parameter can be one of the following values:
 *         @arg @ref DMAC_FIFO_MODE_0
 *         @arg @ref DMAC_FIFO_MODE_1
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_fifo_mode(DmacChannel_Type *dmac, uint32_t fifo_mode)
{
    MODIFY_REG(dmac->CFG_H, DMAC_CHNNL_CFG_H_FIFO_MODE,
            (fifo_mode <<DMAC_CHNNL_CFG_H_FIFO_MODE_POS) & DMAC_CHNNL_CFG_H_FIFO_MODE_MASK);
}

/**
 * @brief  Set DMAC Channel Source Status Update Enable
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t enable: This parameter can be one of the following values:
 *         @arg @ref DMAC_STATUS_UPDATE_DISABLE
 *         @arg @ref DMAC_STATUS_UPDATE_ENABLE
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_src_st_upd_en(DmacChannel_Type *dmac, uint32_t enable)
{
    MODIFY_REG(dmac->CFG_H, DMAC_CHNNL_CFG_H_SS_UPD_EN,
            (enable <<DMAC_CHNNL_CFG_H_SS_UPD_EN_POS) & DMAC_CHNNL_CFG_H_SS_UPD_EN_MASK);
}

/**
 * @brief  Set DMAC Channel Destination Status Update Enable
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t enable: This parameter can be one of the following values:
 *         @arg @ref DMAC_STATUS_UPDATE_DISABLE
 *         @arg @ref DMAC_STATUS_UPDATE_ENABLE
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_dst_st_upd_en(DmacChannel_Type *dmac, uint32_t enable)
{
    MODIFY_REG(dmac->CFG_H, DMAC_CHNNL_CFG_H_DS_UPD_EN,
            (enable <<DMAC_CHNNL_CFG_H_DS_UPD_EN_POS) & DMAC_CHNNL_CFG_H_DS_UPD_EN_MASK);
}

/**
 * @brief  Set DMAC Channel Protection Control
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t protect_ctrl:
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_protect_ctrl(DmacChannel_Type *dmac, uint32_t protect_ctrl)
{
    MODIFY_REG(dmac->CFG_H, DMAC_CHNNL_CFG_H_PROTCTL,
            (protect_ctrl <<DMAC_CHNNL_CFG_H_PROTCTL_POS) & DMAC_CHNNL_CFG_H_PROTCTL_MASK);
}

/**
 * @brief  Set DMAC Channel Source Hardware Interface.
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t hs_sel:
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_src_itf_index(DmacChannel_Type *dmac, uint32_t hs_sel)
{
    MODIFY_REG(dmac->CFG_H, DMAC_CHNNL_CFG_H_SRC_PRE,
            (hs_sel << DMAC_CHNNL_CFG_H_SRC_PRE_POS) & DMAC_CHNNL_CFG_H_SRC_PRE_MASK);
}

/**
 * @brief  Set DMAC Channel Destination hardware interface.
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t hs_sel:
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_dst_itf_index(DmacChannel_Type *dmac, uint32_t hs_sel)
{
    MODIFY_REG(dmac->CFG_H, DMAC_CHNNL_CFG_H_DST_PRE,
            (hs_sel << DMAC_CHNNL_CFG_H_DST_PRE_POS) & DMAC_CHNNL_CFG_H_DST_PRE_MASK);
}

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
static inline void ms_dmac_ll_channel_set_src_msize(DmacChannel_Type *dmac, uint32_t msize)
{
    MODIFY_REG(dmac->CTL_L, DMAC_CHNNL_CTL_L_SRC_MSIZE,
            (msize <<DMAC_CHNNL_CTL_L_SRC_MSIZE_POS) & DMAC_CHNNL_CTL_L_SRC_MSIZE_MASK);
}

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
static inline void ms_dmac_ll_channel_set_dst_msize(DmacChannel_Type *dmac, uint32_t msize)
{
    MODIFY_REG(dmac->CTL_L, DMAC_CHNNL_CTL_L_DST_MSIZE,
            (msize <<DMAC_CHNNL_CTL_L_DST_MSIZE_POS) & DMAC_CHNNL_CTL_L_DST_MSIZE_MASK);
}

/**
 * @brief  Decode DMAC Byte Number by tr_with Value
 * @retval Byte Number
 */
static inline uint32_t ms_dmac_ll_decode_byte_num(uint32_t tr_with)
{
    return (tr_with == DMAC_XFER_WIDTH_8BITS) ? 1 : ((tr_with == DMAC_XFER_WIDTH_16BITS) ? 2 : 4);
}

/**
 * @brief  Set DMAC Block Transfer Size
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t block_size: the total number of single transactions to perform for every block transfer
 * @note   single_size_bytes =  transfer_width/8
 *         blk_size_bytes_dma = block_size * single_size_bytes
 * @retval None
 */
static inline void ms_dmac_ll_channel_set_block_size(DmacChannel_Type *dmac, uint32_t block_size)
{
    MODIFY_REG(dmac->CTL_H, DMAC_CHNNL_CTL_H_BLOCK_TS,
            (block_size <<DMAC_CHNNL_CTL_H_BLOCK_TS_POS) & DMAC_CHNNL_CTL_H_BLOCK_TS_MASK);
}

/**
 * @brief  Set DMAC Block Transfer Size
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @param  uint32_t block_size: the total number of single transactions to perform for every block transfer
 * @note   single_size_bytes =  transfer_width/8
 *         blk_size_bytes_dma = block_size * single_size_bytes
 * @retval None
 */
static inline uint32_t ms_dmac_ll_channel_calc_block_size(uint32_t xfer_count, uint32_t tr_with)
{
    return xfer_count / ms_dmac_ll_decode_byte_num(tr_with);
}

/**
 * @brief  Enable DMAC Channel Interrupt
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @retval None
 */
static inline void ms_dmac_ll_channel_enable_int(DmacChannel_Type *dmac)
{
    SET_BIT(dmac->CTL_L, DMAC_CHNNL_CTL_L_INT_EN);
}

/**
 * @brief  Disable DMAC Channel Interrupt
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @retval None
 */
static inline void ms_dmac_ll_channel_disable_int(DmacChannel_Type *dmac)
{
    CLEAR_BIT(dmac->CTL_L, DMAC_CHNNL_CTL_L_INT_EN);
}

/**
 * @brief  Verify DMAC Channel Interrupt Is Enable
 * @param  DmacChannel_Type *dmac: DMAC Channel Instance
 * @retval Returned value: Enable or Disable
 */
static inline bool ms_dmac_ll_channel_is_enable_int(DmacChannel_Type *dmac)
{
    return READ_BIT(dmac->CTL_L, DMAC_CHNNL_CTL_L_INT_EN) == DMAC_CHNNL_CTL_L_INT_EN;
}

/**
 * @}
 */

/**
 * @}
 */
#endif
