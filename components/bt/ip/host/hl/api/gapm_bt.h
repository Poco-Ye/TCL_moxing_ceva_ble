/**
 ****************************************************************************************
 *
 * @file gapm_bt.h
 *
 * @brief Generic Access Profile Manager - BT-Classic Activities
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */


#ifndef GAPM_BT_H_
#define GAPM_BT_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gapm.h"

/**
 ****************************************************************************************
 * @addtogroup GAPM_BT_API BT-Classic
 * @ingroup GAPM_API
 * @brief Set of functions and interfaces required to create and manage a BT-Classic activity.
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPM_BT_SEC_API Security toolbox
 * @ingroup GAPM_BT_API
 * @brief OOB data
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */
/// @addtogroup GAPM_ENUM_API
/// @{

/// Inquiry or Inquiry Scan type
enum gapm_inquiry_type
{
    /// General discoverable or discovery mode
    GAPM_INQUIRY_GENERAL,
    /// Limited discoverable or discovery mode
    GAPM_INQUIRY_LIMITED,
};

/// @} GAPM_ENUM_API

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * INTERFACES
 ****************************************************************************************
 */

/// @addtogroup GAPM_BT_SEC_API
/// @{

/**
 ***************************************************************************************
 * @brief Function executed when procedure execution is over.
 *
 * @param[in] dummy           Dummy parameter provided by upper layer application
 * @param[in] status          Procedure execution status  (see enum #hl_err)
 * @param[in] p_oob_192       Pointer to generated P-192 OOB data (NULL if status != GAP_ERR_NO_ERROR)
 * @param[in] p_oob_256       Pointer to generated P-256 OOB data (NULL if status != GAP_ERR_NO_ERROR)
 ***************************************************************************************
 */
typedef void (*gapm_bt_oob_cb)(uint32_t dummy, uint16_t status, const gap_oob_t* p_oob_192, const gap_oob_t* p_oob_256);
/// @} GAPM_BT_SEC_API

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/// @addtogroup GAPM_CONFIG_API
/// @{

/**
 ****************************************************************************************
 * @brief Insert into SDP a device identification record.
 *
 * @param[in] vendor_id_source  Designates which organization assigned the vendor_id attribute (see enum #gap_vendor_id_source)
 * @param[in] vendor_id         Uniquely identify the vendor of the device
 * @param[in] product_id        Distinguish between different products made by the vendor
 * @param[in] version           Numeric expression identifying the device release number in Binary-Coded Decimal
 *
 * @return Status of SDP record insertion (see enum #hl_err)
 ****************************************************************************************
 */
uint16_t gapm_set_sdp_device_identification_record(uint16_t vendor_id_source, uint16_t vendor_id, uint16_t product_id,
                                                   uint16_t version);
/// @}

/// @addtogroup GAPM_PROC_API
/// @{

/**
 ****************************************************************************************
 * @brief Get next available service record handle - shall be use only by BT Classic profiles
 *
 * @return Next available service record handle
 ****************************************************************************************
 */
uint32_t gapm_sdp_get_next_service_record_handle(void);

/// @} GAPM_PROC_API

/// @addtogroup GAPM_BT_SEC_API
/// @{

/**
 ***************************************************************************************
 * @brief Generate BT Classic OOB data using
 *
 *        OOB data must be conveyed to peer device through an out of band method.
 *
 * @param[in] dummy     Dummy parameter provided by upper layer application
 * @param[in] res_cb    Function called when procedure is over
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_bt_oob_cb callback execution
 ***************************************************************************************
 */
uint16_t gapm_generate_bt_oob_data(uint32_t dummy, gapm_bt_oob_cb res_cb);
/// @} GAPM_BT_SEC_API



#endif /* GAPM_BT_H_ */
