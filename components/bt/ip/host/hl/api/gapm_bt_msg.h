/**
 ****************************************************************************************
 *
 * @file gapm_bt_msg.h
 *
 * @brief Generic Access Profile Manager Message API. BT-Classic
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */


#ifndef _GAPM_BT_MSG_H_
#define _GAPM_BT_MSG_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gapm_msg.h"
#include "gapm_bt.h"
#include "gapm_bt_page.h"
#include "gapm_bt_page_scan.h"
#include "gapm_bt_inquiry.h"
#include "gapm_bt_inquiry_scan.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * ENUMERATIONS
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// @addtogroup GAPM_MSG_STRUCT_API Message Structures
/// @ingroup GAPM_MSG_API
/// @{

/// Parameters of the @ref GAPM_SET_SDP_DEVICE_IDENTIFICATION_RECORD_CMD message
/*@TRACE*/
struct gapm_set_sdp_device_identification_record_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_SET_SDP_DEVICE_IDENTIFICATION_RECORD
    uint8_t  operation;

    /// Designates which organization assigned the vendor_id attribute (see enum #gap_vendor_id_source)
    uint16_t vendor_id_source;
    /// Uniquely identify the vendor of the device
    uint16_t vendor_id;
    /// Distinguish between different products made by the vendor
    uint16_t product_id;
    /// Numeric expression identifying the device release number in Binary-Coded Decimal
    uint16_t version;
};


/// Parameters of the @ref GAPM_BT_OOB_DATA_IND message
/*@TRACE*/
struct gapm_bt_oob_data_ind
{
    /// Generated P-192 OOB data
    gap_oob_t oob_192;
    /// Generated P-256 OOB data
    gap_oob_t oob_256;
};

/// Inquiry scan start parameters
/*@TRACE*/
typedef struct gapm_inquiry_scan_start_param
{
    /// scan parameters
    gapm_inquiry_scan_param_t scan;
    /// Extended Inquiry Response data length
    uint8_t                   eir_length;
    /// Extended Inquiry Response data
    uint8_t                   eir_data[__ARRAY_EMPTY];
} gapm_inquiry_scan_start_param_t;


/// Parameter of GAPM_INQUIRY_REPORT_IND message
/*@TRACE*/
struct gapm_inquiry_report_ind
{
    /// Activity identifier
    uint8_t               actv_idx;
    /// Inquiry report information
    gapm_inquiry_report_t report;
    /// Length of received EIR data, 0 if nothing received
    uint8_t               eir_length;
    /// Extend inquiry response data
    uint8_t               eir_data[__ARRAY_EMPTY];
};

/// Activity parameters
typedef union gapm_bt_start_param
{
    /// Inquiry parameters
    //@trc_union @activity_map[$parent.actv_idx] == GAPM_ACTV_TYPE_INQUIRY
    gapm_inquiry_param_t            inquiry_param;
    /// Inquiry Scan parameters
    //@trc_union @activity_map[$parent.actv_idx] == GAPM_ACTV_TYPE_INQUIRY_SCAN
    gapm_inquiry_scan_start_param_t inquiry_scan_param;
    /// Page parameters
    //@trc_union @activity_map[$parent.actv_idx] == GAPM_ACTV_TYPE_PAGE
    gapm_page_param_t               page_param;
    /// Page Scan parameters
    //@trc_union @activity_map[$parent.actv_idx] == GAPM_ACTV_TYPE_PAGE_SCAN
    gapm_page_scan_param_t          page_scan_param;
} gapm_bt_start_param_u;

/// Start a given activity command
/*@TRACE*/
struct gapm_bt_activity_start_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_START_ACTIVITY: Start a given activity
    uint8_t operation;
    /// Activity identifier
    uint8_t actv_idx;
    /// Activity parameters
    gapm_bt_start_param_u u_param;
};


/// Parameters of #GAPM_BT_WRITE_LOOPBACK_MODE_CMD
/*@TRACE*/
struct gapm_bt_write_loopback_mode_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_BT_WRITE_LOOPBACK_MODE
    uint8_t operation;
    /// Loopback mode value (see #gapm_bt_loopback_mode)
    uint8_t loopback_mode;
};

/// Parameters of #GAPM_BT_ENABLE_DEVICE_UNDER_TEST_MODE_CMD
/*@TRACE*/
struct gapm_bt_enable_device_under_test_mode_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_BT_ENABLE_DEVICE_UNDER_TEST_MODE
    uint8_t operation;
};


/// Parameters of #GAPM_BT_WRITE_SIMPLE_PAIRING_DEBUG_MODE_CMD
/*@TRACE*/
struct gapm_bt_write_simple_pairing_debug_mode_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_BT_WRITE_SIMPLE_PAIRING_DEBUG_MODE
    uint8_t operation;
    /// True to enable debug mode, false otherwise.
    bool    enable_debug_mode;
};


/// Parameters of #GAPM_BT_WRITE_SECURE_CONNECTIONS_TEST_MODE_CMD
/*@TRACE*/
struct gapm_bt_write_secure_connections_test_mode_cmd
{
    /// Requested operation type (see enum #gapm_operation)
    ///  - #GAPM_BT_WRITE_SECURE_CONNECTIONS_TEST_MODE
    uint8_t operation;
    /// Connection index
    uint8_t conidx;
    /// Enables or disables the use of DM1 packets for transmitting ACL-U data.
    bool    enable_dm1_acl_u_mode;
    /// Enables and disables the loopback of received eSCO payloads.
    bool    enable_esco_loopback_mode;
};

/// Parameters of #GAPM_BT_LOOPBACK_IND
/*@TRACE*/
struct gapm_bt_loopback_ind
{
    /// Loopback mode value read (see #gapm_bt_loopback_mode)
    uint8_t loopback_mode;
};

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/// @} GAPM_MSG_STRUCT_API

#endif /* _GAPM_BT_MSG_H_ */
