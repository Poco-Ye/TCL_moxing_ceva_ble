/**
 ****************************************************************************************
 *
 * @file gapc_le_smp_encrypt.c
 *
 * @brief GAPC Handle encryption on LE connection
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#if (BLE_GAPC)
#include "gapc_int.h"
#include "gapc_le_smp.h"
#include "hl_hci.h"
#include "hci.h"
#include "co_utils.h"

#include <string.h>

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

#if (HL_LE_CENTRAL)
/// Encrypt procedure
typedef struct gapc_le_smp_encrypt_proc
{
    /// Inherited from Simple procedure
    gapc_proc_simple_t hdr;
    /// LTK information
    gapc_ltk_t         ltk_info;
} gapc_le_smp_encrypt_proc_t;
#endif // (HL_LE_CENTRAL)

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
#if (HL_LE_PERIPHERAL)
/// verify encryption context before accepting or rejected the encryption
__STATIC uint16_t gapc_check_encrypt_context_for_cfm(uint8_t conidx, gapc_le_con_t* p_con)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;

    do
    {
        if(p_con == NULL) break;

        if(gapc_get_role(conidx) != ROLE_SLAVE)
        {
            status = GAP_ERR_NOT_SUPPORTED;
            break;
        }

        // Pairing or no encryption on-going
        if(gapc_le_smp_is_pairing_ongoing(p_con) || !GETB(p_con->hdr.info_bf, GAPC_WAIT_ENCRYPTION)) break;

        status = GAP_ERR_NO_ERROR;
    } while(0);

    return (status);
}
#endif // (HL_LE_PERIPHERAL)


/*
 * PROCEDURE STATE MACHINE
 ****************************************************************************************
 */

#if (HL_LE_CENTRAL)
/// Encrypt procedure state machine granted function
__STATIC uint16_t gapc_le_smp_encrypt_proc_granted(uint8_t conidx, gapc_le_smp_encrypt_proc_t* p_proc)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    struct hci_le_en_enc_cmd *p_hci_cmd = HL_HCI_CMD_ALLOC(HCI_LE_EN_ENC_CMD_OPCODE, hci_le_en_enc_cmd);
    if(p_hci_cmd != NULL)
    {
        gapc_ltk_t* p_ltk_info = &(p_proc->ltk_info);
        p_hci_cmd->conhdl    = gapc_get_conhdl(conidx);
        p_hci_cmd->enc_div = p_ltk_info->ediv;
        memcpy(&p_hci_cmd->nb.nb[0], &(p_ltk_info->randnb), GAP_RAND_NB_LEN);
        memcpy(&p_hci_cmd->ltk.ltk[0], &(p_ltk_info->key), GAP_KEY_LEN);

        HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, GAPC_PROC_TOKEN(conidx, HL_PROC_CONTINUE),
                gapc_proc_default_hci_stat_evt_handler);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

/// Encrypt procedure state machine finished function
__STATIC void gapc_le_smp_encrypt_proc_finished(uint8_t conidx, gapc_le_smp_encrypt_proc_t* p_proc, uint16_t status)
{
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);
    if(p_con != NULL)
    {
        if(status == GAP_ERR_NO_ERROR)
        {
            // store encryption key size
            p_con->smp.key_size = p_proc->ltk_info.key_size;
        }

        SETB(p_con->hdr.info_bf, GAPC_WAIT_ENCRYPTION, false);
    }

    gapc_proc_simple_default_finished_cb(conidx, (gapc_proc_simple_t*)p_proc, status);
}

/// Encrypt procedure interface
__STATIC const gapc_proc_simple_itf_t gapc_le_smp_encrypt_proc_itf =
{
    .granted  = (gapc_proc_simple_granted_cb)  gapc_le_smp_encrypt_proc_granted,
    .finished = (gapc_proc_simple_finished_cb) gapc_le_smp_encrypt_proc_finished,
};
#endif // (HL_LE_CENTRAL)

/*
 * HCI HANDLERS
 ****************************************************************************************
 */

extern void gapc_le_smp_pairing_enc_change_evt_handle(gapc_le_con_t* p_con, uint16_t status);

/// Handle the encryption change event
__STATIC void gapc_le_smp_encrypt_handle_enc_change_evt(uint8_t conidx, uint8_t hci_status)
{
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);
    if(p_con != NULL)
    {
        uint16_t status;
        // Error code conversion
        switch(hci_status)
        {
            // The peer device cannot find the keys to start encryption
            case CO_ERROR_PIN_MISSING:     { status =  SMP_ERR_ENC_KEY_MISSING;     } break;
            // The peer device doesn't support encryption
            case CO_ERROR_UNSUPPORTED:     { status =  SMP_ERR_ENC_NOT_SUPPORTED;   } break;
            // The encryption didn't because a timeout has occurred
            case CO_ERROR_LMP_RSP_TIMEOUT: { status =  SMP_ERR_ENC_TIMEOUT;         } break;
            default:                       { status = RW_ERR_HCI_TO_HL(hci_status); } break;
        }

        if(status == GAP_ERR_NO_ERROR)
        {
            // informs that link is now encrypted
            SETB(p_con->hdr.bond.info_bf, GAPC_LE_ENCRYPTED, true);

            #if (BLE_GATT_CLI)
            // Inform Client that link is encrypted
            gapc_cli_link_encrypted(conidx);
            #endif // (BLE_GATT_CLI)
        }

        // encryption on-going
        if(GETB(p_con->hdr.info_bf, GAPC_WAIT_ENCRYPTION))
        {
            SETB(p_con->hdr.info_bf, GAPC_WAIT_ENCRYPTION, false);

            if(status == GAP_ERR_NO_ERROR)
            {
                // Indicate that connection is encrypted
                if(gapc_env.p_sec_cbs->auth_info != NULL)
                {
                    gapc_env.p_sec_cbs->auth_info(conidx, p_con->hdr.dummy,
                                                  GETF(p_con->hdr.bond.info_bf, GAPC_SEC_LVL), true);
                }
            }

            #if(HL_LE_CENTRAL)
            if(gapc_get_role(conidx) == ROLE_MASTER)
            {
                gapc_proc_transition(conidx, HL_PROC_FINISHED, status);
            }
            #endif // (HL_LE_CENTRAL)
        }
        // pairing on-going
        else if(gapc_le_smp_is_pairing_ongoing(p_con))
        {
            gapc_le_smp_pairing_enc_change_evt_handle(p_con, status);
        }
        else
        {
            ASSERT_ERR(0); // not expected
        }
    }
}

/// @brief Handles the HCI encryption change event.
void gapc_hci_enc_chg_evt_handler(uint8_t evt_code, struct hci_enc_change_evt const *p_evt)
{
    gapc_le_smp_encrypt_handle_enc_change_evt(gapc_get_conidx(p_evt->conhdl), p_evt->status);
}

/// @brief Handles the HCI encryption key refresh event.
void gapc_hci_enc_key_refr_evt_handler(uint8_t evt_code, struct hci_enc_key_ref_cmp_evt const *p_evt)
{
    gapc_le_smp_encrypt_handle_enc_change_evt(gapc_get_conidx(p_evt->conhdl), p_evt->status);
}


#if (HL_LE_PERIPHERAL)
extern void gapc_le_smp_hci_le_ltk_request_evt_handler(uint8_t conidx, gapc_le_con_t* p_con);

/// @brief Handles long term key request from link layer.
void gapc_hci_le_ltk_request_evt_handler(uint8_t evt_code, struct hci_le_ltk_request_evt const *p_evt)
{
    // Recover connection index
    uint8_t conidx = gapc_get_conidx(p_evt->conhdl);
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);

    if(p_con != NULL)
    {
        if(!gapc_le_smp_is_pairing_ongoing(p_con))
        {
            // mark wait encryption to true
            SETB(p_con->hdr.info_bf, GAPC_WAIT_ENCRYPTION, true);
            if(gapc_env.p_sec_cbs->le_encrypt_req != NULL)
            {
                gapc_env.p_sec_cbs->le_encrypt_req(conidx, p_con->hdr.dummy, p_evt->ediv,
                                                       (const rand_nb_t*) &(p_evt->rand));
            }
            else
            {
                gapc_le_encrypt_req_reply(conidx, false, NULL, 0);
            }
        }
        else
        {
            gapc_le_smp_hci_le_ltk_request_evt_handler(conidx, p_con);
        }
    }
}
#endif //(HL_LE_PERIPHERAL)

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if (HL_LE_CENTRAL)
uint16_t gapc_le_encrypt(uint8_t conidx, uint32_t dummy, const gapc_ltk_t* p_ltk_info, gapc_proc_cmp_cb cmp_cb)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);

    do
    {
        gapc_le_smp_encrypt_proc_t* p_proc;
        if(p_con == NULL) break;

        if((p_ltk_info == NULL) || !gapc_le_smp_is_key_size_valid(p_ltk_info->key_size))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // check connection role.
        if(gapc_get_role(conidx) != ROLE_MASTER)
        {
            status = GAP_ERR_NOT_SUPPORTED;
            break;
        }

        // Pairing or encryption on-going
        if(gapc_le_smp_is_pairing_ongoing(p_con) || GETB(p_con->hdr.info_bf, GAPC_WAIT_ENCRYPTION))
        {
            status = GAP_ERR_BUSY;
            break;
        }

        // create a procedure to handle encryption execution
        status = gapc_proc_simple_create(conidx, dummy, cmp_cb, sizeof(gapc_le_smp_encrypt_proc_t),
                                         &gapc_le_smp_encrypt_proc_itf, (gapc_proc_simple_t**) &p_proc);
        if(status == GAP_ERR_NO_ERROR)
        {
            p_proc->ltk_info  = *p_ltk_info;
            SETB(p_con->hdr.info_bf, GAPC_WAIT_ENCRYPTION, true);
        }
    } while(0);

    return (status);
}
#endif // (HL_LE_CENTRAL)


#if (HL_LE_PERIPHERAL)
uint16_t gapc_le_encrypt_req_reply(uint8_t conidx, bool accept, const gap_sec_key_t* p_ltk, uint8_t key_size)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);

    do
    {
        status = gapc_check_encrypt_context_for_cfm(conidx, p_con);
        if(status != GAP_ERR_NO_ERROR) break;

        if(!accept)
        {
             // Send reject command
            status = HL_HCI_BASIC_CMD_SEND_WITH_CONHDL(HCI_LE_LTK_REQ_NEG_REPLY_CMD_OPCODE, gapc_get_conhdl(conidx), 0,
                                                       gapc_proc_ignore_hci_evt_handler);
            if(status != GAP_ERR_NO_ERROR) break;

            // mark wait encryption to false
            SETB(p_con->hdr.info_bf, GAPC_WAIT_ENCRYPTION, false);
        }
        else
        {
            if((p_ltk == NULL) || !gapc_le_smp_is_key_size_valid(key_size))
            {
                status = GAP_ERR_INVALID_PARAM;
            }

            // Reply that the encryption key has been found
            struct hci_le_ltk_req_reply_cmd *p_hci_cmd =
                    HL_HCI_CMD_ALLOC(HCI_LE_LTK_REQ_REPLY_CMD_OPCODE, hci_le_ltk_req_reply_cmd);

            if(p_hci_cmd == NULL)
            {
                status = GAP_ERR_INSUFF_RESOURCES;
                break;
            }

            p_hci_cmd->conhdl = gapc_get_conhdl(conidx);
            memcpy(&(p_hci_cmd->ltk), p_ltk, GAP_KEY_LEN);
            HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, conidx, gapc_proc_ignore_hci_evt_handler);

            // keep information about new key size
            p_con->smp.key_size = key_size;
            status = GAP_ERR_NO_ERROR;
        }
    } while(0);

    return (status);
}
#endif // (HL_LE_PERIPHERAL)

#endif // (BLE_GAPC)
/// @} GAPC
