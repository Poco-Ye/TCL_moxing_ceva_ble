/**
 ****************************************************************************************
 *
 * @file gapm_bt_page.h
 *
 * @brief Generic Access Profile Manager - BT-Classic Page Activities
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */


#ifndef GAPM_BT_PAGE_H_
#define GAPM_BT_PAGE_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gapm_bt.h"

/**
 ****************************************************************************************
 * @addtogroup GAPM_BT_PAGE_ACTV_API  BT-Classic Page
 *
 * @ingroup GAPM_ACTV_API
 *
 * @brief Create and control Page activity.
 *
 * Application can control only on page activity even if several can be created.
 *
 * Page is used by a BT-Classic device to initiate a connection establishment.
 *
 * The application must follow the #gapm_page_actv_cb_t callback interface to handle activities events:
 * \snippet{lineno} app_page.c APP_PAGE_ACTV_CB
 *
 * The application must also follow the #gapc_connection_req_cb_t callback interface to handle connection creation:
 * \snippet{lineno} app_connection.c APP_BT_CON_CB
 *
 * Application can then create an page activity using #gapm_page_create.
 * Once activity is created it can be immediately started using #gapm_page_start.
 *
 * When connection is established, the activity is automatically stopped.
 *
 * example:
 * \snippet{lineno} app_page.c APP_START_PAGE_ACTV
 *
 *
 * @note At least #GAP_ROLE_BT_CLASSIC role is required
 *
 * An application example is available in \ref app_test_bt_central.c
 *
 * @{
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/// Page scan repetition mode
enum gapm_page_scan_repetition_mode
{
    GAPM_SCAN_R0 = 0,
    GAPM_SCAN_R1,
    GAPM_SCAN_R2,
};

/// Packet type configuration bit field
enum gapm_packet_type_bf
{
    /// 2-DH1 shall not be used.
    GAPM_PKT_NO_USE_2_DH1_POS   = 1,
    GAPM_PKT_NO_USE_2_DH1_BIT   = (1 << 1),
    /// 3-DH1 shall not be used.
    GAPM_PKT_NO_USE_3_DH1_POS   = 2,
    GAPM_PKT_NO_USE_3_DH1_BIT   = (1 << 2),
    /// DH1 may be used.
    GAPM_PKT_MAY_USE_DH1_POS    = 4,
    GAPM_PKT_MAY_USE_DH1_BIT    = (1 << 4),
    /// 2-DH3 shall not be used.
    GAPM_PKT_NO_USE_2_DH3_POS   = 8,
    GAPM_PKT_NO_USE_2_DH3_BIT   = (1 << 8),
    /// 3-DH3 shall not be used.
    GAPM_PKT_NO_USE_3_DH3_POS   = 9,
    GAPM_PKT_NO_USE_3_DH3_BIT   = (1 << 9),
    /// DM3 may be used.
    GAPM_PKT_MAY_USE_DM3_POS    = 10,
    GAPM_PKT_MAY_USE_DM3_BIT    = (1 << 10),
    /// DH3 may be used.
    GAPM_PKT_MAY_USE_DH3_POS    = 11,
    GAPM_PKT_MAY_USE_DH3_BIT    = (1 << 11),
    /// 2-DH5 shall not be used.
    GAPM_PKT_NO_USE_2_DH5_POS   = 12,
    GAPM_PKT_NO_USE_2_DH5_BIT   = (1 << 12),
    /// 3-DH5 shall not be used.
    GAPM_PKT_NO_USE_3_DH5_POS   = 13,
    GAPM_PKT_NO_USE_3_DH5_BIT   = (1 << 13),
    /// DM5 may be used.
    GAPM_PKT_MAY_USE_DM5_POS    = 14,
    GAPM_PKT_MAY_USE_DM5_BIT    = (1 << 14),
    /// DH5 may be used.
    GAPM_PKT_MAY_USE_DH5_POS    = 15,
    GAPM_PKT_MAY_USE_DH5_BIT    = (1 << 15),
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Page parameters
/*@TRACE*/
typedef struct gapm_page_param
{
    /// Address of the device to connect
    gap_addr_t  peer_addr;
    /// Page timeout to consider connection establishment failed (in baseband slot: 0.625ms unit)
    /// if zero, keep default page timeout
    uint16_t    page_timeout;
    /// Bit field of supported and unsupported packet types (see enum #gapm_packet_type_bf)
    /// For an automatic selection use #GAPM_PAGE_AUTO_PKT_TYPE_SEL (0xFFFF)
    uint16_t    packet_type_bf;
    /// Peer device clock offset - shall be set to zero if unknown
    uint16_t    clock_offset;
    /// Page Scan repetition mode (see enum #gapm_page_scan_repetition_mode)
    uint8_t     page_scan_repetition;
    /// True to allow peer device to becomes master of the connection, False to stay master.
    bool        allow_role_switch;
} gapm_page_param_t;

/*
 * INTERFACES
 ****************************************************************************************
 */

/// Callback structure required to create a connectable activity
typedef gapm_actv_cb_t gapm_page_actv_cb_t;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Create Page activity - Create a Master BT Classic connection
 *
 * @param[in]  dummy            Dummy parameter provided by upper layer application
 * @param[in]  p_cbs            Activity Callback interface
 * @param[out] p_actv_idx       Pointer used to return allocated activity index
 *
 * @return Execution status (see enum #hl_err).
 ****************************************************************************************
 */
uint16_t gapm_page_create(uint32_t dummy, const gapm_page_actv_cb_t* p_cbs, uint8_t* p_actv_idx);

/**
 ****************************************************************************************
 * @brief Start Page activity
 *
 * @param[in] actv_idx       Activity local index
 * @param[in] p_param        Pointer to scan parameters
 *
 * @return Execution status (see enum #hl_err).
 *         If returns GAP_ERR_NO_ERROR, upper layer SW shall wait for #gapm_actv_cb_t.proc_cmp callback execution
 ****************************************************************************************
 */
uint16_t gapm_page_start(uint8_t actv_idx, const gapm_page_param_t* p_param);


/// @} GAPM_BT_PAGE_ACTV_API
#endif /* GAPM_BT_PAGE_H_ */
