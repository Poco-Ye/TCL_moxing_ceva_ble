/**
 ****************************************************************************************
 *
 * @file gapm_bt_test.c
 *
 * @brief Generic Access Profile Manager - Test mode for BT-Classic
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#if(BT_HOST_PRESENT && HOST_TEST_MODE)
#include "gapm_bt_test.h"
#include "gapm_proc.h"    // Procedure API
#include "gapc.h"         // Get connection handle


/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * Read loopback mode procedure
 ****************************************************************************************
 */

/// @brief Handle reception of HCI_RD_LOOPBACK_MODE_CMD complete event.
__STATIC void hci_rd_loopback_mode_cmd_cmp_evt_handler(uint16_t opcode, uint16_t conhdl,
                                                       struct hci_rd_loopback_mode_cmd_cmp_evt const *p_evt)
{
    uint32_t                     dummy;
    gapm_bt_read_loopback_res_cb res_cb;

    // stop procedure
    gapm_proc_info_stop(&dummy, (gapm_proc_info_res_cb*) &res_cb);

    // send back result
    res_cb(dummy, RW_ERR_HCI_TO_HL(p_evt->status), p_evt->lb_mode);
}

uint16_t gapm_bt_read_loopback_mode(uint32_t dummy, gapm_bt_read_loopback_res_cb res_cb)
{
    return (GAPM_INFO_PROC_START(dummy, HCI_RD_LOOPBACK_MODE_CMD_OPCODE,
                                 hci_rd_loopback_mode_cmd_cmp_evt_handler, res_cb));
}

/*
 * Write loopback mode procedure
 ****************************************************************************************
 */
/// Write loopback_mode procedure parameters structure
typedef struct gapm_bt_write_loopback_mode_proc
{
    /// Procedure inheritance
    gapm_proc_send_hci_cmd_t hdr;
    /// Loopback mode value (see #gapm_bt_loopback_mode)
    uint8_t                  loopback_mode;
} gapm_bt_write_loopback_mode_proc_t;

/// prepare Write loopback_mode hci command
__STATIC void gapm_bt_prepare_write_loopback_mode_cmd(gapm_bt_write_loopback_mode_proc_t* p_proc, struct hci_wr_loopback_mode_cmd* p_cmd)
{
    p_cmd->lb_mode = p_proc->loopback_mode;
}

/// Send command  Write loopback_mode hci command information
__STATIC const gapm_proc_hci_cmd_info_t gapm_bt_send_write_loopback_cmd_info =
{
    .prep_cb     = (gapm_proc_prepare_hci_cmd_cb) gapm_bt_prepare_write_loopback_mode_cmd,
    .cmd_op_code = HCI_WR_LOOPBACK_MODE_CMD_OPCODE,
    .cmd_length  = sizeof(struct hci_wr_loopback_mode_cmd),
};

uint16_t gapm_bt_write_loopback_mode(uint32_t dummy, uint8_t loopback_mode, gapm_proc_cmp_cb cmp_cb)
{
    gapm_bt_write_loopback_mode_proc_t* p_proc;
    uint16_t status = gapm_proc_send_hci_cmd_create(sizeof(gapm_bt_write_loopback_mode_proc_t),
                                                    &gapm_bt_send_write_loopback_cmd_info,
                                                    dummy, cmp_cb, (gapm_proc_send_hci_cmd_t**)&p_proc);
    if(status == GAP_ERR_NO_ERROR)
    {
        p_proc->loopback_mode = loopback_mode;
    }

    return (status);
}


/*
 * Enable device under test mode procedure
 ****************************************************************************************
 */

/// Prepare enable device under test mode hci command
__STATIC void gapm_bt_prepare_enable_device_under_test_mode_cmd(gapm_proc_send_hci_cmd_t* p_proc, void* p_cmd)
{
    // Nothing to do
}

/// Send  enable device under test mode hci command information
__STATIC const gapm_proc_hci_cmd_info_t gapm_bt_send_enable_device_under_test_mode_cmd_info =
{
    .prep_cb     = (gapm_proc_prepare_hci_cmd_cb) gapm_bt_prepare_enable_device_under_test_mode_cmd,
    .cmd_op_code = HCI_EN_DUT_MODE_CMD_OPCODE,
    .cmd_length  = 0,
};

uint16_t gapm_bt_enable_device_under_test_mode(uint32_t dummy, gapm_proc_cmp_cb cmp_cb)
{
    gapm_proc_send_hci_cmd_t* p_proc;
    uint16_t status = gapm_proc_send_hci_cmd_create(sizeof(gapm_proc_send_hci_cmd_t),
                                                    &gapm_bt_send_enable_device_under_test_mode_cmd_info,
                                                    dummy, cmp_cb, &p_proc);
    return (status);
}

/*
 * Write Simple Pairing debug mode procedure
 ****************************************************************************************
 */

/// Write Simple Pairing debug mode procedure parameters structure
typedef struct gapm_bt_write_simple_pairing_debug_mode_proc
{
    /// Procedure inheritance
    gapm_proc_send_hci_cmd_t hdr;
    /// True to enable debug mode, false otherwise.
    bool                     enable_debug_mode;
} gapm_bt_write_simple_pairing_debug_mode_proc_t;

/// Prepare Write Simple Pairing debug mode hci command
__STATIC void gapm_bt_prepare_write_simple_pairing_debug_mode_cmd(gapm_bt_write_simple_pairing_debug_mode_proc_t* p_proc,
                                                                      struct hci_wr_sp_dbg_mode_cmd* p_cmd)
{
     p_cmd->sp_mode = p_proc->enable_debug_mode;
}

/// Send command Write Simple Pairing debug mode information
__STATIC const gapm_proc_hci_cmd_info_t gapm_bt_send_write_simple_pairing_debug_mode_cmd_info =
{
    .prep_cb     = (gapm_proc_prepare_hci_cmd_cb) gapm_bt_prepare_write_simple_pairing_debug_mode_cmd,
    .cmd_op_code = HCI_WR_SP_DBG_MODE_CMD_OPCODE,
    .cmd_length  = sizeof(struct hci_wr_sp_dbg_mode_cmd),
};

uint16_t gapm_bt_write_simple_pairing_debug_mode(uint32_t dummy, bool enable_debug_mode, gapm_proc_cmp_cb cmp_cb)
{
    gapm_bt_write_simple_pairing_debug_mode_proc_t* p_proc;
    uint16_t status = gapm_proc_send_hci_cmd_create(sizeof(gapm_bt_write_simple_pairing_debug_mode_proc_t),
                                                    &gapm_bt_send_write_simple_pairing_debug_mode_cmd_info,
                                                    dummy, cmp_cb, (gapm_proc_send_hci_cmd_t**)&p_proc);
    if(status == GAP_ERR_NO_ERROR)
    {
        p_proc->enable_debug_mode = enable_debug_mode;
    }

    return (status);
}

/*
 * Write secure connections test mode procedure
 ****************************************************************************************
 */

/// Write secure connections test mode procedure parameters structure
typedef struct gapm_bt_write_secure_connections_test_mode_proc
{
    /// Procedure inheritance
    gapm_proc_send_hci_cmd_t hdr;
    /// Connection Handle
    uint16_t                 conhdl;
    /// Enables or disables the use of DM1 packets for transmitting ACL-U data.
    bool                     enable_dm1_acl_u_mode;
    /// enable_esco_loopback_mode
    bool                     enable_esco_loopback_mode;
} gapm_bt_write_secure_connections_test_mode_proc_t;

/// Prepare Write secure connections test mode hci command
__STATIC void gapm_bt_prepare_write_secure_connections_test_mode_cmd(gapm_bt_write_secure_connections_test_mode_proc_t* p_proc,
                                                                         struct hci_wr_sec_con_test_mode_cmd* p_cmd)
{
    p_cmd->conhdl             = p_proc->conhdl;
    p_cmd->dm1_acl_u_mode     = p_proc->enable_dm1_acl_u_mode;
    p_cmd->esco_loopback_mode = p_proc->enable_esco_loopback_mode;
}

/// Send Write secure connections test mode hci command information
__STATIC const gapm_proc_hci_cmd_info_t gapm_bt_send_write_secure_connections_test_mode_cmd_info =
{
    .prep_cb     = (gapm_proc_prepare_hci_cmd_cb) gapm_bt_prepare_write_secure_connections_test_mode_cmd,
    .cmd_op_code = HCI_WR_SEC_CON_TEST_MODE_CMD_OPCODE,
    .cmd_length  = sizeof(struct hci_wr_sec_con_test_mode_cmd),
};

uint16_t gapm_bt_write_secure_connections_test_mode(uint32_t dummy, uint8_t conidx, bool enable_dm1_acl_u_mode,
                                                    bool enable_esco_loopback_mode, gapm_proc_cmp_cb cmp_cb)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    uint16_t conhdl = gapc_get_conhdl(conidx);
    if(conhdl != GAP_INVALID_CONIDX)
    {
        gapm_bt_write_secure_connections_test_mode_proc_t* p_proc;
        status = gapm_proc_send_hci_cmd_create(sizeof(gapm_bt_write_secure_connections_test_mode_proc_t),
                                               &gapm_bt_send_write_secure_connections_test_mode_cmd_info,
                                               dummy, cmp_cb, (gapm_proc_send_hci_cmd_t**)&p_proc);
        if(status == GAP_ERR_NO_ERROR)
        {
            p_proc->conhdl = conhdl;
            p_proc->enable_dm1_acl_u_mode = enable_dm1_acl_u_mode;
            p_proc->enable_esco_loopback_mode = enable_esco_loopback_mode;
        }
    }

    return (status);
}
#endif // (BT_HOST_PRESENT && HOST_TEST_MODE)
