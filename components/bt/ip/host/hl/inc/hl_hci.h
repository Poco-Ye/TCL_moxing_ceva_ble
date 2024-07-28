/**
 ****************************************************************************************
 *
 * @file hl_hci.h
 *
 * @brief Entry points for reception of Host Control Interface message from LL
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 *
 ****************************************************************************************
 */

#ifndef HL_HCI_H_
#define HL_HCI_H_

#include <stdint.h>
#include <stdbool.h>                // standard boolean definitions
#include "rwip_config.h"
/**
 ****************************************************************************************
 * @addtogroup HOST
 * @brief Entry points for reception of Host Control Interface message from LL
 *
 * @{
 ****************************************************************************************
 */

/*
 * MACROS
 ****************************************************************************************
 */

/// Allocate an HCI command message
#define HL_HCI_CMD_ALLOC(opcode, type)  \
    (struct type*) hl_hci_cmd_alloc((opcode), sizeof(struct type))
/// Allocate an HCI command message with variable size
#define HL_HCI_CMD_ALLOC_DYN(opcode, type, size) \
    (struct type*) hl_hci_cmd_alloc((opcode), (sizeof(struct type) + size))
/// Allocate HCI ACL Data message
#define HL_HCI_DATA_ALLOC(type) \
    (struct type*) hl_hci_data_alloc()

/// Send command to controller
#define HL_HCI_CMD_SEND_TO_CTRL(cmd, handle, cmp_evt_cb)  \
    hl_hci_send_cmd_to_ctrl((cmd), (handle), (hl_hci_cmd_evt_func_t) (cmp_evt_cb))

/// Send Basic HCI Command to controller
#define HL_HCI_BASIC_CMD_SEND(opcode, handle, cmp_evt_cb)\
    hl_hci_basic_cmd_send_to_ctrl((opcode), (handle), (hl_hci_cmd_evt_func_t) (cmp_evt_cb));

/// Send Basic HCI Command with conhdl to controller
#define HL_HCI_BASIC_CMD_SEND_WITH_CONHDL(opcode, conhdl, handle, cmp_evt_cb)\
    hl_hci_basic_cmd_send_with_conhdl_to_ctrl((opcode), (conhdl), (handle), (hl_hci_cmd_evt_func_t) (cmp_evt_cb));

/*
 * DEFINES
 ****************************************************************************************
 */

// When HCI command not supported by host
#define HL_HCI_INVALID_LID         (0xFF)

/// Type of supported events
enum hl_hci_evt_type
{
    /// Command complete event
    HL_HCI_CMD_CMP_EVT,
    /// Command Status event
    HL_HCI_CMD_STAT_EVT,
    /// HCI event
    HL_HCI_EVT,
    #if (BLE_HOST_PRESENT)
    /// HCI Low Energy event
    HL_HCI_LE_EVT,
    #endif // (BLE_HOST_PRESENT)
    /// HCI Debug event
    HL_HCI_DBG_EVT,
    /// HCI ACL data
    HL_HCI_ACL_DATA,
    /// Voice over HCI data
    HL_HCI_VOHCI_DATA,
    /// ISO over HCI data
    HL_HCI_ISO_DATA,
    /// HCI Command
    HL_HCI_CMD,

    /// unknown type
    HL_HCI_UNDEF,
};

#if (BLE_HOST_PRESENT)
/// Local index list of GAP LE event handler Local identifier
enum hl_hci_le_evt_handler_lid
{
    /* Scan procedure */
    HL_HCI_LE_EXT_ADV_REPORT_EVT,
    HL_HCI_LE_SCAN_TIMEOUT_EVT,

    /* Periodic synchronization procedure */
    HL_HCI_LE_PER_ADV_SYNC_EST_EVT,
    HL_HCI_LE_PER_ADV_REPORT_EVT,
    HL_HCI_LE_PER_ADV_SYNC_LOST_EVT,
    HL_HCI_LE_BIG_INFO_ADV_REPORT_EVT,
    HL_HCI_LE_PER_ADV_SYNC_TRANSF_REC_EVT,

    #if (BLE_AOD | BLE_AOA)
    HL_HCI_LE_CONLESS_IQ_REPORT_EVT,
    HL_HCI_LE_CTE_REQ_FAILED_EVT,
    #endif // (BLE_AOD | BLE_AOA)

    HL_HCI_LE_ENH_CON_CMP_EVT,
    HL_HCI_LE_RD_LOC_P256_PUB_KEY_CMP_EVT,
    HL_HCI_LE_GEN_DHKEY_CMP_EVT,

    /* Advertising procedure */
    HL_HCI_LE_ADV_SET_TERMINATED_EVT,
    HL_HCI_LE_SCAN_REQ_RCVD_EVT,

    HL_HCI_LE_RD_REM_FEATS_CMP_EVT,
    HL_HCI_LE_CON_UPDATE_CMP_EVT,
    HL_HCI_LE_REM_CON_PARAM_REQ_EVT,
    HL_HCI_LE_DATA_LEN_CHG_EVT,
    HL_HCI_LE_PHY_UPD_CMP_EVT,
    HL_HCI_LE_CH_SEL_ALGO_EVT,

    HL_HCI_LE_LTK_REQUEST_EVT,

    #if(BLE_AOA | BLE_AOD)
    HL_HCI_LE_CON_IQ_REPORT_EVT,
    #endif // (BLE_AOA | BLE_AOD)

    #if (BLE_PWR_CTRL)
    HL_HCI_LE_TX_POWER_REPORTING_EVT,
    HL_HCI_LE_PATH_LOSS_THRESHOLD_EVT,
    #endif // (BLE_PWR_CTRL)

    #if(BLE_CIS || BLE_BIS)
    HL_HCI_LE_CIS_ESTABLISHED_EVT,
    HL_HCI_LE_CIS_REQUEST_EVT,
    HL_HCI_LE_CREATE_BIG_CMP_EVT,
    HL_HCI_LE_TERMINATE_BIG_CMP_EVT,
    HL_HCI_LE_BIG_SYNC_ESTABLISHED_EVT,
    HL_HCI_LE_BIG_SYNC_LOST_EVT,
    #endif // (BLE_CIS || BLE_BIS)

    HL_HCI_LE_EVT_HANDLER_NB,
};
#endif // (BLE_HOST_PRESENT)

/// Local index list of GAP event handler Local identifier
enum hl_hci_evt_handler_lid
{
    HL_HCI_DISC_CMP_EVT,
    HL_HCI_RD_REM_VER_INFO_CMP_EVT,
    HL_HCI_AUTH_PAYL_TO_EXP_EVT,
    HL_HCI_ENC_CHG_EVT,
    HL_HCI_ENC_KEY_REFRESH_CMP_EVT,
    HL_HCI_NB_CMP_PKTS_EVT,

    #if (BT_HOST_PRESENT)
    HL_HCI_INQ_CMP_EVT,
    HL_HCI_INQ_RES_EVT,
    HL_HCI_INQ_RES_WITH_RSSI_EVT,
    HL_HCI_EXT_INQ_RES_EVT,
    #endif // (BT_HOST_PRESENT)

    HL_HCI_EVT_HANDLER_NB,
};

/// Local index list of GAP vendor specific event handler Local identifier
enum hl_hci_vs_evt_handler_lid
{
    HL_HCI_VS_EVT_HANDLER_NB,
};

/// Local index list of GAP vendor specific event handler Local identifier
enum hl_hci_acl_data_evt_handler_lid
{
    #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    HL_HCI_ACL_DATA_EVT,
    #endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)

    HL_HCI_ACL_DATA_EVT_HANDLER_NB,
};

/// List of unsupported event handlers
enum hl_hci_unsuported_handler_lid
{
    HL_HCI_CON_CMP_EVT                                 = HL_HCI_INVALID_LID,
    HL_HCI_CON_REQ_EVT                                 = HL_HCI_INVALID_LID,
    HL_HCI_AUTH_CMP_EVT                                = HL_HCI_INVALID_LID,
    HL_HCI_REM_NAME_REQ_CMP_EVT                        = HL_HCI_INVALID_LID,
    HL_HCI_CHG_CON_LK_CMP_EVT                          = HL_HCI_INVALID_LID,
    HL_HCI_MASTER_LK_CMP_EVT                           = HL_HCI_INVALID_LID,
    HL_HCI_RD_REM_SUPP_FEATS_CMP_EVT                   = HL_HCI_INVALID_LID,
    HL_HCI_QOS_SETUP_CMP_EVT                           = HL_HCI_INVALID_LID,
    HL_HCI_HW_ERR_EVT                                  = HL_HCI_INVALID_LID,
    HL_HCI_FLUSH_OCCURRED_EVT                          = HL_HCI_INVALID_LID,
    HL_HCI_ROLE_CHG_EVT                                = HL_HCI_INVALID_LID,
    HL_HCI_MODE_CHG_EVT                                = HL_HCI_INVALID_LID,
    HL_HCI_RETURN_LINK_KEYS_EVT                        = HL_HCI_INVALID_LID,
    HL_HCI_PIN_CODE_REQ_EVT                            = HL_HCI_INVALID_LID,
    HL_HCI_LK_REQ_EVT                                  = HL_HCI_INVALID_LID,
    HL_HCI_LK_NOTIF_EVT                                = HL_HCI_INVALID_LID,
    HL_HCI_DATA_BUF_OVFLW_EVT                          = HL_HCI_INVALID_LID,
    HL_HCI_MAX_SLOT_CHG_EVT                            = HL_HCI_INVALID_LID,
    HL_HCI_RD_CLK_OFF_CMP_EVT                          = HL_HCI_INVALID_LID,
    HL_HCI_CON_PKT_TYPE_CHG_EVT                        = HL_HCI_INVALID_LID,
    HL_HCI_QOS_VIOL_EVT                                = HL_HCI_INVALID_LID,
    HL_HCI_PAGE_SCAN_REPET_MODE_CHG_EVT                = HL_HCI_INVALID_LID,
    HL_HCI_FLOW_SPEC_CMP_EVT                           = HL_HCI_INVALID_LID,
    HL_HCI_RD_REM_EXT_FEATS_CMP_EVT                    = HL_HCI_INVALID_LID,
    HL_HCI_SYNC_CON_CMP_EVT                            = HL_HCI_INVALID_LID,
    HL_HCI_SYNC_CON_CHG_EVT                            = HL_HCI_INVALID_LID,
    HL_HCI_SNIFF_SUB_EVT                               = HL_HCI_INVALID_LID,
    HL_HCI_IO_CAP_REQ_EVT                              = HL_HCI_INVALID_LID,
    HL_HCI_IO_CAP_RSP_EVT                              = HL_HCI_INVALID_LID,
    HL_HCI_USER_CFM_REQ_EVT                            = HL_HCI_INVALID_LID,
    HL_HCI_USER_PASSKEY_REQ_EVT                        = HL_HCI_INVALID_LID,
    HL_HCI_REM_OOB_DATA_REQ_EVT                        = HL_HCI_INVALID_LID,
    HL_HCI_SP_CMP_EVT                                  = HL_HCI_INVALID_LID,
    HL_HCI_LINK_SUPV_TO_CHG_EVT                        = HL_HCI_INVALID_LID,
    HL_HCI_ENH_FLUSH_CMP_EVT                           = HL_HCI_INVALID_LID,
    HL_HCI_USER_PASSKEY_NOTIF_EVT                      = HL_HCI_INVALID_LID,
    HL_HCI_KEYPRESS_NOTIF_EVT                          = HL_HCI_INVALID_LID,
    HL_HCI_REM_HOST_SUPP_FEATS_NOTIF_EVT               = HL_HCI_INVALID_LID,
    HL_HCI_SYNC_TRAIN_CMP_EVT                          = HL_HCI_INVALID_LID,
    HL_HCI_SYNC_TRAIN_REC_EVT                          = HL_HCI_INVALID_LID,
    HL_HCI_CON_SLV_BCST_REC_EVT                        = HL_HCI_INVALID_LID,
    HL_HCI_CON_SLV_BCST_TO_EVT                         = HL_HCI_INVALID_LID,
    HL_HCI_TRUNC_PAGE_CMP_EVT                          = HL_HCI_INVALID_LID,
    HL_HCI_SLV_PAGE_RSP_TO_EVT                         = HL_HCI_INVALID_LID,
    HL_HCI_CON_SLV_BCST_CH_MAP_CHG_EVT                 = HL_HCI_INVALID_LID,
    HL_HCI_SAM_STATUS_CHANGE_EVT                       = HL_HCI_INVALID_LID,
    HL_HCI_DBG_ASSERT_EVT                              = HL_HCI_INVALID_LID,
    HL_HCI_VS_AUSY_CIS_ESTAB_PARAM_EVT                 = HL_HCI_INVALID_LID,
    HL_HCI_VS_AUSY_BIS_ESTAB_PARAM_EVT                 = HL_HCI_INVALID_LID,
    HL_HCI_LE_CON_CMP_EVT                              = HL_HCI_INVALID_LID,
    HL_HCI_LE_ADV_REPORT_EVT                           = HL_HCI_INVALID_LID,
    HL_HCI_LE_DIR_ADV_REP_EVT                          = HL_HCI_INVALID_LID,
    HL_HCI_LE_REQ_PEER_SCA_CMP_EVT                     = HL_HCI_INVALID_LID,
    #if !(BT_HOST_PRESENT)
    HL_HCI_INQ_CMP_EVT                                 = HL_HCI_INVALID_LID,
    HL_HCI_INQ_RES_EVT                                 = HL_HCI_INVALID_LID,
    HL_HCI_INQ_RES_WITH_RSSI_EVT                       = HL_HCI_INVALID_LID,
    HL_HCI_EXT_INQ_RES_EVT                             = HL_HCI_INVALID_LID,
    #endif // !(BT_HOST_PRESENT)
    #if (!BLE_HOST_PRESENT)
    HL_HCI_LE_EXT_ADV_REPORT_EVT                       = HL_HCI_INVALID_LID,
    HL_HCI_LE_SCAN_TIMEOUT_EVT                         = HL_HCI_INVALID_LID,
    HL_HCI_LE_PER_ADV_SYNC_EST_EVT                     = HL_HCI_INVALID_LID,
    HL_HCI_LE_PER_ADV_REPORT_EVT                       = HL_HCI_INVALID_LID,
    HL_HCI_LE_PER_ADV_SYNC_LOST_EVT                    = HL_HCI_INVALID_LID,
    HL_HCI_LE_BIG_INFO_ADV_REPORT_EVT                  = HL_HCI_INVALID_LID,
    HL_HCI_LE_PER_ADV_SYNC_TRANSF_REC_EVT              = HL_HCI_INVALID_LID,
    HL_HCI_LE_CONLESS_IQ_REPORT_EVT                    = HL_HCI_INVALID_LID,
    HL_HCI_LE_CTE_REQ_FAILED_EVT                       = HL_HCI_INVALID_LID,
    HL_HCI_LE_ENH_CON_CMP_EVT                          = HL_HCI_INVALID_LID,
    HL_HCI_LE_RD_LOC_P256_PUB_KEY_CMP_EVT              = HL_HCI_INVALID_LID,
    HL_HCI_LE_GEN_DHKEY_CMP_EVT                        = HL_HCI_INVALID_LID,
    HL_HCI_LE_ADV_SET_TERMINATED_EVT                   = HL_HCI_INVALID_LID,
    HL_HCI_LE_SCAN_REQ_RCVD_EVT                        = HL_HCI_INVALID_LID,
    HL_HCI_LE_RD_REM_FEATS_CMP_EVT                     = HL_HCI_INVALID_LID,
    HL_HCI_LE_CON_UPDATE_CMP_EVT                       = HL_HCI_INVALID_LID,
    HL_HCI_LE_REM_CON_PARAM_REQ_EVT                    = HL_HCI_INVALID_LID,
    HL_HCI_LE_DATA_LEN_CHG_EVT                         = HL_HCI_INVALID_LID,
    HL_HCI_LE_PHY_UPD_CMP_EVT                          = HL_HCI_INVALID_LID,
    HL_HCI_LE_CH_SEL_ALGO_EVT                          = HL_HCI_INVALID_LID,
    HL_HCI_LE_LTK_REQUEST_EVT                          = HL_HCI_INVALID_LID,
    HL_HCI_LE_CON_IQ_REPORT_EVT                        = HL_HCI_INVALID_LID,
    HL_HCI_LE_TX_POWER_REPORTING_EVT                   = HL_HCI_INVALID_LID,
    HL_HCI_LE_PATH_LOSS_THRESHOLD_EVT                  = HL_HCI_INVALID_LID,
    HL_HCI_LE_CIS_ESTABLISHED_EVT                      = HL_HCI_INVALID_LID,
    HL_HCI_LE_CIS_REQUEST_EVT                          = HL_HCI_INVALID_LID,
    HL_HCI_LE_CREATE_BIG_CMP_EVT                       = HL_HCI_INVALID_LID,
    HL_HCI_LE_TERMINATE_BIG_CMP_EVT                    = HL_HCI_INVALID_LID,
    HL_HCI_LE_BIG_SYNC_ESTABLISHED_EVT                 = HL_HCI_INVALID_LID,
    HL_HCI_LE_BIG_SYNC_LOST_EVT                        = HL_HCI_INVALID_LID,
    #endif //  (!BLE_HOST_PRESENT)
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// HCI operation is done callback definition
typedef void (*hl_hci_done_cb)(void);


/**
 ****************************************************************************************
 * @brief Format of a HCI Command Complete or Status event handler function
 *
 * @param[in] opcode    HCI code:
 *                          - HCI Command OP Code for command complete event and command status
 *                          - HCI Event Code for general HCI Events
 *                          - HCI LE Event sub-code for general HCI LE Meta Events
 * @param[in] handle    Handle provided when asking execution
 * @param[in] p_evt     Pointer to command complete or status event parameters. (specific to the HCI command)
 *
 ****************************************************************************************
 */
typedef void (*hl_hci_cmd_evt_func_t)(uint16_t opcode, uint16_t handle, void const *p_evt);


struct hci_acl_data;
/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Retrieve host message transport layer destination from connection handle
 *
 * @param[in] conhdl  Connection handle
 *
 * @return Host destination (see enum #HCI_MSG_HOST_TL_DEST)
 *
 ****************************************************************************************
 */
uint8_t hl_hci_get_host_tl_dest_from_conhdl(uint16_t conhdl);

/**
 ****************************************************************************************
 * @brief Retrieve host message transport layer destination for an HCI ACL Data
 *
 * @param[in] conhdl  Connection handle
 *
 * @return Host destination (see enum #HCI_MSG_HOST_TL_DEST)
 *
 ****************************************************************************************
 */
uint8_t hl_hci_get_acl_data_host_tl_dest(const struct hci_acl_data *p_acl_data);

/**
 ****************************************************************************************
 * @brief Retrieve Command complete or status event message transport layer destination
 *        from connection operation code
 *
 * @param[in] opcode  Command operation code
 *
 * @return Host destination (see enum #HCI_MSG_HOST_TL_DEST)
 *
 ****************************************************************************************
 */
uint8_t hl_hci_cmd_evt_get_host_tl_dest(uint16_t opcode);

/**
 ****************************************************************************************
 * @brief Handle an HCI Event message sent by controller to Host - in Unpacked format
 *
 * @param[in] evt_type  Event Type to handle (see enum #hl_hci_evt_type)
 * @param[in] evt_code  HCI code:
 *                          - HCI Event Code for general HCI Events
 *                          - HCI LE Event sub-code for general HCI LE Meta Events
 * @param[in] host_lid  Host local event identifier according to event type:
 *                       - HL_HCI_EVT: see enum #hl_hci_le_evt_handler_lid
 *                       - HL_HCI_LE_EVT: see enum #hl_hci_evt_handler_lid
 *                       - HL_HCI_DBG_EVT: see enum #hl_hci_vs_evt_handler_lid
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
void hl_hci_send_evt_to_host(uint8_t evt_type, uint8_t evt_code, uint8_t host_lid, const void* p_evt);

/**
 ****************************************************************************************
 * @brief Handle an HCI command complete or command status message sent by controller to Host
 *        in Unpacked format
 *
 * @param[in] evt_type  Event Type to handle (see enum #hl_hci_evt_type)
 * @param[in] opcode    HCI command operation code
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
void hl_hci_send_cmd_evt_to_host(uint8_t evt_type, uint16_t opcode, const void* p_evt);

/**
 ****************************************************************************************
 * @brief Handle an HCI data sent by controller to Host
 *
 * @param[in] evt_type  Event Type to handle (see enum #hl_hci_evt_type)
 * @param[in] p_evt     Pointer to data descriptor.
 *
 ****************************************************************************************
 */
void hl_hci_send_data_to_host(uint8_t evt_type, const void* p_evt);

/**
 ****************************************************************************************
 * @brief Send an HCI command to controller - HCI message in unpacked format
 *
 * @param[in] p_cmd     Pointer to command parameters.
 * @param[in] handle    Value returned in command complete or command status event
 * @param[in] cmp_evt_cb  Function to call when command complete or status event is received
 *
 ****************************************************************************************
 */
void hl_hci_send_cmd_to_ctrl(void* p_cmd, uint16_t handle, hl_hci_cmd_evt_func_t cmd_evt_cb);


/**
 ****************************************************************************************
 * @brief Send an HCI data to controller
 *
 * @param[in] p_data   Pointer to data message to send to controller
 *
 ****************************************************************************************
 */
void hl_hci_send_data_to_ctrl(struct hci_acl_data* p_data);

/**
 ****************************************************************************************
 * @brief function used to send a basic command, without parameters to the controller.
 *
 * @param[in] opcode      Operation code of the command
 * @param[in] handle      Value returned in command complete or command status event
 * @param[in] cmp_evt_cb  Function to call when command complete or status event is received
 *
 * @return Status of HCI message transmission (see enum #hl_err)
 *****************************************************************************************
 */
uint16_t hl_hci_basic_cmd_send_to_ctrl(uint16_t opcode, uint16_t handle, hl_hci_cmd_evt_func_t cmp_evt_cb);

/**
 ****************************************************************************************
 * @brief function used to send a basic command with unique conhdl parameter to the controller.
 *
 * @param[in] opcode      Operation code of the command
 * @param[in] conhdl      Connection handle
 * @param[in] handle      Value returned in command complete or command status event
 * @param[in] cmp_evt_cb  Function to call when command complete or status event is received
 *
 * @return Status of HCI message transmission (see enum #hl_err)
 *****************************************************************************************
 */
uint16_t hl_hci_basic_cmd_send_with_conhdl_to_ctrl(uint16_t opcode, uint16_t conhdl, uint16_t handle,
                                                   hl_hci_cmd_evt_func_t cmp_evt_cb);

/**
 ****************************************************************************************
 * @brief Handle an HCI message sent by controller in Host - in packed format
 *
 * @param[in] type      Event Type to handle (see enum #hci_msg_type)
 * @param[in] p_buf     Pointer to buffer that contains HCI data in packed format
 * @param[in] len       Length of HCI Buffer
 * @param[in] cb_done   Callback to execute when Host stack has handle the HCI message
 *
 ****************************************************************************************
 */
void hl_hci_pk_send_to_host(uint8_t type, uint8_t *p_buf, uint16_t len, hl_hci_done_cb cb_done);

/**
 ****************************************************************************************
 * @brief Send an HCI message to controller - HCI data already packed
 *
 * @param[in] type      Event Type to handle (see enum #hci_msg_type)
 * @param[in] p_buf     Pointer to buffer that contains HCI data in packed format
 * @param[in] len       Length of HCI Buffer
 * @param[in] cb_done   Callback to execute when Controller stack has handle the HCI message
 *
 ****************************************************************************************
 */
void hl_hci_pk_send_to_ctrl(uint8_t type, uint8_t *p_buf, uint16_t len, hl_hci_done_cb cb_done);

/**
 ****************************************************************************************
 * @brief Allocate an HCI command message
 *
 * @note Dedicated macro should be used: @see HL_HCI_CMD_ALLOC or @see HL_HCI_CMD_ALLOC_DYN
 *
 * @param[in] opcode     Command Operation code
 * @param[in] size       Size of message to allocate
 *
 * @return Pointer to allocated HCI command message
 ****************************************************************************************
 */
uint8_t* hl_hci_cmd_alloc(uint16_t opcode, uint16_t size);

/**
 ****************************************************************************************
 * @brief Allocate a command
 *
 * @note Dedicated macro should be used @see HL_HCI_DATA_ALLOC
 *
 * @param[in] op_code    Command Operation code
 * @param[in] size       Size of message to allocate
 * @param[in] cmp_evt_cb Function to call once command complete or status event is received
 *
 * @return Pointer to allocated HCI data message
 ****************************************************************************************
 */
struct hci_acl_data* hl_hci_data_alloc();

/// @} HL_HCI_H_

#endif // HL_H_
