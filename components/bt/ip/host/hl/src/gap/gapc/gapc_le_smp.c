/**
 ****************************************************************************************
 *
 * @file gapc_le_smp.c
 *
 * @brief GAPC_LE_SMP implementation.
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup SMP Security Manager Protocol
 * @ingroup HOST
 * @brief Security Manager Protocol.
 *
 * The SMP is responsible for the over-all security policies of BLE.
 * It defines methods for pairing and key distribution, handles encryption,
 * data signing and privacy features such as random addressing generation and resolution.
 *
 * Pairing is performed to exchange pairing features and generate a short term
 * key for link encryption.
 * A transport specific key distribution is performed to
 * share the keys that can be used to encrypt the link in the future
 * reconnection process, signed data verification and random address
 * resolution.
 *
 * There exist 3 phases in the complete security procedure:
 * 1. Feature exchange (IO capabilities, OOB flags, Authentication Requirements, Key distributions)
 * 2. Short Term Key generation
 *    Generation method depends on exchanged features:
 *     - Just Works - use Temporary key = 0
 *     - PassKey Entry - use Temporary Key = 6-digit provided by user
 *     - Out of Band (OOB) - use Temporary Key = 16-octet key, available form OOB source
 * 3. Transport Specific Key Distribution (TKDP)(LTK+EDIV+RAND_NB, IRK+ADDR, CSRK)
 *---------------------------------------------------------------------
 * @addtogroup GAPC_LE_SMP Security Manager Protocol Controller
 * @ingroup SMP
 * @brief Security Manager Protocol Controller.
 *
 * This block handles control of SM procedures for several possible existing connections,
 * for which the security procedure may be conducted simultaneously.
 *
 * It allows flow control for HCI access to encryption and random number generation, used
 * at different moments in the procedure.
 *
 * It handles PDU creation and sending through L2CAP, also their reception from L2CAP
 * and interpretation.
 *
 * Other small utilities such as maximum key size determination and TKDP organization are
 * implemented in GAPC_LE_SMP.
 * @{
 *
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
#include "hci.h"
#include "hl_hci.h"
#include "co_math.h"
#include "ke_mem.h"
#include "aes.h"
#if (BT_HOST_PRESENT)
#include "gapc_bt.h"
#endif // (BT_HOST_PRESENT)


/*
 * MACROS
 ****************************************************************************************
 */

#define GAP_LE_SMP_PAIR_METHOD(legacy, sec_con) (((GAPC_LE_SMP_##legacy) << 4) | (GAPC_LE_SMP_##sec_con))
#define GAP_LE_SMP_LEGACY_PAIR_METHOD_GET(rsp_io, req_io)  ((gapc_le_smp_pair_method[(rsp_io)][(req_io)]) >> 4)
#define GAP_LE_SMP_SEC_CON_PAIR_METHOD_GET(rsp_io, req_io) ((gapc_le_smp_pair_method[(rsp_io)][(req_io)]) & 0xf)

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// GAPC_LE_SMP table for pairing method used for different IO capabilities, 1st conidx = R, 2nd = I
__STATIC const uint8_t gapc_le_smp_pair_method[GAP_IO_CAP_LAST][GAP_IO_CAP_LAST] =
{
    // Method:                                                                            | LEGACY | SEC_CON |
    [GAP_IO_CAP_DISPLAY_ONLY][GAP_IO_CAP_DISPLAY_ONLY]             = GAP_LE_SMP_PAIR_METHOD(METH_JW, METH_JW),
    [GAP_IO_CAP_DISPLAY_ONLY][GAP_IO_CAP_DISPLAY_YES_NO]           = GAP_LE_SMP_PAIR_METHOD(METH_JW, METH_JW),
    [GAP_IO_CAP_DISPLAY_ONLY][GAP_IO_CAP_KB_ONLY]                  = GAP_LE_SMP_PAIR_METHOD(METH_PK, METH_PK),
    [GAP_IO_CAP_DISPLAY_ONLY][GAP_IO_CAP_NO_INPUT_NO_OUTPUT]       = GAP_LE_SMP_PAIR_METHOD(METH_JW, METH_JW),
    [GAP_IO_CAP_DISPLAY_ONLY][GAP_IO_CAP_KB_DISPLAY]               = GAP_LE_SMP_PAIR_METHOD(METH_PK, METH_PK),

    [GAP_IO_CAP_DISPLAY_YES_NO][GAP_IO_CAP_DISPLAY_ONLY]           = GAP_LE_SMP_PAIR_METHOD(METH_JW, METH_JW),
    [GAP_IO_CAP_DISPLAY_YES_NO][GAP_IO_CAP_DISPLAY_YES_NO]         = GAP_LE_SMP_PAIR_METHOD(METH_JW, METH_NC),
    [GAP_IO_CAP_DISPLAY_YES_NO][GAP_IO_CAP_KB_ONLY]                = GAP_LE_SMP_PAIR_METHOD(METH_PK, METH_PK),
    [GAP_IO_CAP_DISPLAY_YES_NO][GAP_IO_CAP_NO_INPUT_NO_OUTPUT]     = GAP_LE_SMP_PAIR_METHOD(METH_JW, METH_JW),
    [GAP_IO_CAP_DISPLAY_YES_NO][GAP_IO_CAP_KB_DISPLAY]             = GAP_LE_SMP_PAIR_METHOD(METH_PK, METH_NC),

    [GAP_IO_CAP_KB_ONLY][GAP_IO_CAP_DISPLAY_ONLY]                  = GAP_LE_SMP_PAIR_METHOD(METH_PK, METH_PK),
    [GAP_IO_CAP_KB_ONLY][GAP_IO_CAP_DISPLAY_YES_NO]                = GAP_LE_SMP_PAIR_METHOD(METH_PK, METH_PK),
    [GAP_IO_CAP_KB_ONLY][GAP_IO_CAP_KB_ONLY]                       = GAP_LE_SMP_PAIR_METHOD(METH_PK, METH_PK),
    [GAP_IO_CAP_KB_ONLY][GAP_IO_CAP_NO_INPUT_NO_OUTPUT]            = GAP_LE_SMP_PAIR_METHOD(METH_JW, METH_JW),
    [GAP_IO_CAP_KB_ONLY][GAP_IO_CAP_KB_DISPLAY]                    = GAP_LE_SMP_PAIR_METHOD(METH_PK, METH_PK),

    [GAP_IO_CAP_NO_INPUT_NO_OUTPUT][GAP_IO_CAP_DISPLAY_ONLY]       = GAP_LE_SMP_PAIR_METHOD(METH_JW, METH_JW),
    [GAP_IO_CAP_NO_INPUT_NO_OUTPUT][GAP_IO_CAP_DISPLAY_YES_NO]     = GAP_LE_SMP_PAIR_METHOD(METH_JW, METH_JW),
    [GAP_IO_CAP_NO_INPUT_NO_OUTPUT][GAP_IO_CAP_KB_ONLY]            = GAP_LE_SMP_PAIR_METHOD(METH_JW, METH_JW),
    [GAP_IO_CAP_NO_INPUT_NO_OUTPUT][GAP_IO_CAP_NO_INPUT_NO_OUTPUT] = GAP_LE_SMP_PAIR_METHOD(METH_JW, METH_JW),
    [GAP_IO_CAP_NO_INPUT_NO_OUTPUT][GAP_IO_CAP_KB_DISPLAY]         = GAP_LE_SMP_PAIR_METHOD(METH_JW, METH_JW),

    [GAP_IO_CAP_KB_DISPLAY][GAP_IO_CAP_DISPLAY_ONLY]               = GAP_LE_SMP_PAIR_METHOD(METH_PK, METH_PK),
    [GAP_IO_CAP_KB_DISPLAY][GAP_IO_CAP_DISPLAY_YES_NO]             = GAP_LE_SMP_PAIR_METHOD(METH_PK, METH_NC),
    [GAP_IO_CAP_KB_DISPLAY][GAP_IO_CAP_KB_ONLY]                    = GAP_LE_SMP_PAIR_METHOD(METH_PK, METH_PK),
    [GAP_IO_CAP_KB_DISPLAY][GAP_IO_CAP_NO_INPUT_NO_OUTPUT]         = GAP_LE_SMP_PAIR_METHOD(METH_JW, METH_JW),
    [GAP_IO_CAP_KB_DISPLAY][GAP_IO_CAP_KB_DISPLAY]                 = GAP_LE_SMP_PAIR_METHOD(METH_PK, METH_NC),
};

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
#if (HL_LE_PERIPHERAL)
__STATIC uint16_t gapc_le_smp_periph_pairing_start_proc_transition(gapc_le_smp_pair_proc_t* p_proc,
                                                           uint8_t event, bool* p_is_finished);
#endif // (HL_LE_PERIPHERAL)

/// Check Pairing request/response parameters
__STATIC uint8_t gapc_le_smp_check_pairing_param(uint8_t iocap, uint8_t auth, uint8_t oob, uint8_t key_size)
{
    uint8_t smp_status = L2CAP_SMP_ERR_NO_ERROR;

    // pairing not supported at all
    if(!gapm_is_legacy_pairing_supp() && !gapm_is_sec_con_pairing_supp())
    {
        smp_status = L2CAP_SMP_ERR_PAIRING_NOT_SUPP;
    }

    else if (   (iocap > GAP_IO_CAP_KB_DISPLAY)           // Check IO Capabilities value
             || !gapc_le_smp_is_auth_valid(auth)          // Check Authentication Requirements
             || (oob > GAP_OOB_AUTH_DATA_PRESENT)         // Check Out Of Band status
             || !gapc_le_smp_is_key_size_valid(key_size)  // Check Encryption Key Size
             || (((auth & GAP_AUTH_SEC_CON) != 0) && (key_size != GAPC_LE_SMP_MAX_ENC_SIZE_LEN)))
    {
        smp_status = L2CAP_SMP_ERR_ENC_KEY_SIZE;
    }

    return (smp_status);
}

/// Check pairing parameters provided by application
__STATIC uint8_t gapc_le_smp_check_app_pairing_param(const gapc_pairing_t* p_pairing_info)
{
    uint8_t smp_status = L2CAP_SMP_ERR_NO_ERROR;

    // Check Key Distribution
    if ((p_pairing_info->ikey_dist > GAP_KDIST_LAST) || (p_pairing_info->rkey_dist > GAP_KDIST_LAST))
    {
        smp_status = L2CAP_SMP_ERR_INVALID_PARAM;
    }
    // Check pairing mode accepted required
    else if (   (((p_pairing_info->auth & GAP_AUTH_SEC_CON) != 0) && !gapm_is_sec_con_pairing_supp())
             || (((p_pairing_info->auth & GAP_AUTH_SEC_CON) == 0) && !gapm_is_legacy_pairing_supp()))
    {
        smp_status = L2CAP_SMP_ERR_AUTH_REQ;
    }
    else // check other parameters
    {
        smp_status = gapc_le_smp_check_pairing_param(p_pairing_info->iocap, p_pairing_info->auth, p_pairing_info->oob,
                                                     p_pairing_info->key_size);
    }

    return (smp_status);
}


/// Create pairing procedure
__STATIC uint16_t gapc_le_smp_pairing_proc_create(gapc_le_con_t* p_con, gapc_le_smp_proc_transition_cb transition_cb,
                                                  gapc_le_smp_pair_proc_t** pp_proc)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    gapc_le_smp_pair_proc_t* p_proc;

    if(gapc_env.p_sec_proc != NULL) // reject if a pairing is on-going
    {
        status = GAP_ERR_BUSY;
    }
    else
    {
        // Allocate memory for the pairing information structure
        p_proc = (gapc_le_smp_pair_proc_t*) ke_malloc_user(sizeof(gapc_le_smp_pair_proc_t), KE_MEM_KE_MSG);
        if(p_proc != NULL)
        {
            uint8_t conidx = p_con->hdr.conidx;
            memset(p_proc, 0x00, sizeof(gapc_le_smp_pair_proc_t));
            gapc_env.p_sec_proc   = &(p_proc->hdr);
            p_proc->app_exp_value = GAPC_LE_SMP_VALUE_INVALID;
            SETB(p_proc->info_bf, GAPC_LE_SMP_USER_ACCEPT, true);
            p_proc->hdr.transition_cb = (gapc_sec_proc_transition_cb) transition_cb;
            p_proc->hdr.conidx        = conidx;
            p_proc->hdr.dummy         = p_con->hdr.dummy;
            gapc_sdt_prepare(&(p_proc->hdr.trans_timer), conidx, GAPC_SDT_SMP);
            status = GAP_ERR_NO_ERROR;
            *pp_proc = p_proc;
        }
    }

    return (status);
}

/// @brief Check if the keys distribution scheme is compliant with the required security level
__STATIC bool gapc_le_smp_check_key_distrib(gapc_le_smp_pair_proc_t* p_proc)
{
    // Returned is_compliant
    bool is_compliant = true;

    // Keys distributed by the initiator
    uint8_t i_keys    = p_proc->pair_req_pdu.ikey_dist & p_proc->pair_rsp_pdu.ikey_dist;
    // Keys distributed by the responder
    uint8_t r_keys    = p_proc->pair_req_pdu.rkey_dist & p_proc->pair_rsp_pdu.rkey_dist;

    // If both device are bondable check that at least one key is distributed
    if (   ((p_proc->pair_req_pdu.auth & GAP_AUTH_BOND) == GAP_AUTH_BOND)
        && ((p_proc->pair_rsp_pdu.auth & GAP_AUTH_BOND) == GAP_AUTH_BOND))
    {
        if (!GETB(p_proc->info_bf, GAPC_LE_SMP_SC_PAIRING) && ((i_keys == GAP_KDIST_NONE) && (r_keys == GAP_KDIST_NONE)))
        {
            is_compliant = false;
        }
    }

    if (GETB(p_proc->info_bf, GAPC_LE_SMP_SC_PAIRING))
    {
        //GAP_SEC1_SEC_CON_PAIR_ENC
        SETB(p_proc->info_bf, GAPC_LE_SMP_LTK_EXCH, true);
    }
    else
    {
        SETB(p_proc->info_bf, GAPC_LE_SMP_LTK_EXCH,
                ((i_keys & GAP_KDIST_ENCKEY) == GAP_KDIST_ENCKEY) || ((r_keys & GAP_KDIST_ENCKEY) == GAP_KDIST_ENCKEY));
    }

    // If a security mode 1 is required, check if a LTK is distributed
    if ((p_proc->sec_req_level == GAP_SEC1_NOAUTH_PAIR_ENC) || (p_proc->sec_req_level == GAP_SEC1_AUTH_PAIR_ENC))
    {
        if (!GETB(p_proc->info_bf, GAPC_LE_SMP_LTK_EXCH))
        {
            is_compliant = false;
        }
    }

    // If a security mode 2 is required, check if a CSRK is distributed
    if ((p_proc->sec_req_level == GAP_SEC2_NOAUTH_DATA_SGN) || (p_proc->sec_req_level == GAP_SEC2_AUTH_DATA_SGN))
    {
        if (!(   ((i_keys & GAP_KDIST_SIGNKEY) == GAP_KDIST_SIGNKEY)
              || ((r_keys & GAP_KDIST_SIGNKEY) == GAP_KDIST_SIGNKEY)))
        {
            is_compliant = false;
        }
    }

    return (is_compliant);
}

/// @brief Determine the method which will be used to generate the STK during a pairing procedure
__STATIC void gapc_le_smp_compute_key_sec_prop(gapc_le_smp_pair_proc_t* p_proc)
{
    if (!GETB(p_proc->info_bf, GAPC_LE_SMP_SC_PAIRING))
    {
        // Check if the TK will be OOB data
        if (   (p_proc->pair_req_pdu.oob == GAP_OOB_AUTH_DATA_PRESENT)
            && (p_proc->pair_rsp_pdu.oob == GAP_OOB_AUTH_DATA_PRESENT))
        {
            // Will have to get the TK from host
            p_proc->pair_method  = GAPC_LE_SMP_METH_OOB;
        }
        // Both have no MITM set in authentication requirements
        else if (((p_proc->pair_req_pdu.auth & GAP_AUTH_MITM) == 0x00) &&
                ((p_proc->pair_rsp_pdu.auth & GAP_AUTH_MITM) == 0x00))
        {
            // Will have to use TK = 0, no need to ask Host
            p_proc->pair_method  = GAPC_LE_SMP_METH_JW;
        }
        // In function of IOs, the PASSKEY ENTRY or JW methods will be used
        else
        {
            p_proc->pair_method = GAP_LE_SMP_LEGACY_PAIR_METHOD_GET(p_proc->pair_rsp_pdu.iocap, p_proc->pair_req_pdu.iocap);
        }

        // Security properties of the STK and all distributed keys
        switch (p_proc->pair_method)
        {
            case GAPC_LE_SMP_METH_OOB:
            case GAPC_LE_SMP_METH_PK:
            {
                // All distributed keys will have these properties
                p_proc->pairing_lvl = GAP_PAIRING_AUTH;
            } break;

            case GAPC_LE_SMP_METH_JW:
            {
                // All distributed keys will have these properties
                p_proc->pairing_lvl = GAP_PAIRING_UNAUTH;
            } break;

            default:
            {
                ASSERT_ERR(0);
            } break;
        }
    }
    else // Secure Connections
    {
        if (   (p_proc->pair_req_pdu.oob == GAP_OOB_AUTH_DATA_PRESENT)
            || (p_proc->pair_rsp_pdu.oob == GAP_OOB_AUTH_DATA_PRESENT))
        {
            // Will have to get the TK from host
            p_proc->pair_method  = GAPC_LE_SMP_METH_OOB;
        }
        // Both have no MITM set in authentication requirements
        else if (   ((p_proc->pair_req_pdu.auth & GAP_AUTH_MITM) == 0x00)
                 && ((p_proc->pair_rsp_pdu.auth & GAP_AUTH_MITM) == 0x00))
        {
            // Will have to use TK = 0, no need to ask Host
            p_proc->pair_method  = GAPC_LE_SMP_METH_JW;
        }
        else
        {
            p_proc->pair_method = GAP_LE_SMP_SEC_CON_PAIR_METHOD_GET(p_proc->pair_rsp_pdu.iocap, p_proc->pair_req_pdu.iocap);
        }

        // Security properties of the STK and all distributed keys
        switch (p_proc->pair_method)
        {
            case GAPC_LE_SMP_METH_OOB:
            case GAPC_LE_SMP_METH_PK:
            case GAPC_LE_SMP_METH_NC:
            {
                // All distributed keys will have these properties
                p_proc->pairing_lvl = GAP_PAIRING_SECURE_CON;
            } break;

            case GAPC_LE_SMP_METH_JW:
            {
                // All distributed keys will have these properties
                p_proc->pairing_lvl = GAP_PAIRING_UNAUTH;
            } break;

            default:
            {
                ASSERT_ERR(0);
            } break;
        }

    }

    // Check if both devices are bondable
    if (   ((p_proc->pair_req_pdu.auth & GAP_AUTH_BOND) == GAP_AUTH_BOND)
        && ((p_proc->pair_rsp_pdu.auth & GAP_AUTH_BOND) == GAP_AUTH_BOND))
    {
        SETB(p_proc->pairing_lvl, GAP_PAIRING_BOND_PRESENT, true);
    }
}

/// Check if the security mode requested by the application or the peer device can
/// be reached with the exchanged pairing features.
__STATIC bool gapc_le_smp_is_sec_mode_reached(gapc_le_smp_pair_proc_t* p_proc)
{
    bool reached;
    uint8_t pairing_lvl = p_proc->pairing_lvl;
    SETB(pairing_lvl, GAP_PAIRING_BOND_PRESENT, false);

    // check if security requirement are OK according to pairing authentication level
    switch(p_proc->sec_req_level)
    {
        case GAP_NO_SEC:
        case GAP_SEC1_NOAUTH_PAIR_ENC:
        case GAP_SEC2_NOAUTH_DATA_SGN:
        {
            reached = true;
        } break;
        case GAP_SEC1_AUTH_PAIR_ENC:
        case GAP_SEC2_AUTH_DATA_SGN:
        {
            reached = ((pairing_lvl == GAP_PAIRING_AUTH) || (pairing_lvl == GAP_PAIRING_SECURE_CON));
        } break;
        case GAP_SEC1_SEC_CON_PAIR_ENC:
        {
            reached = (pairing_lvl == GAP_PAIRING_SECURE_CON);
        } break;
        default:
        {
            reached = false;
        } break;
    }

    return (reached);
}

/// Check if security requirement reached for pairing
__STATIC uint8_t gapc_le_check_pairing_security_reached(gapc_le_smp_pair_proc_t* p_proc)
{
    uint8_t smp_status = L2CAP_SMP_ERR_NO_ERROR;

    // Check that secure connection mode is not required
    if (!GETB(p_proc->info_bf, GAPC_LE_SMP_SC_PAIRING) && !gapm_is_legacy_pairing_supp())
    {
        smp_status = L2CAP_SMP_ERR_AUTH_REQ;
    }
    else if (!gapc_le_smp_check_key_distrib(p_proc))
    {
        // The resultant key distribution doesn't match with the provided parameters
        smp_status = L2CAP_SMP_ERR_UNSPECIFIED_REASON;
    }
    else
    {
        // Compute security properties and STK generation type
        gapc_le_smp_compute_key_sec_prop(p_proc);

        // Check if the required security mode can be reached
        if (!gapc_le_smp_is_sec_mode_reached(p_proc))
        {
            // The pairing procedure cannot be performed as authentication requirements cannot
            // be met due to IO capabilities of one or both devices.
            smp_status = L2CAP_SMP_ERR_AUTH_REQ;
        }
    }
    return (smp_status);
}


/// Check if link key derivation and prepare PDU if enabled
__STATIC void gapc_le_smp_prepare_for_link_key_derivation(gapc_pairing_t* p_pairing)
{
    #if (BT_HOST_PRESENT)
    // Key derivation only if secure connection enabled and Link key present in both initiator and responder keys
    if(   ((gapm_get_suppported_roles() & GAP_ROLE_BT_CLASSIC) != 0)
       && ((p_pairing->auth & GAP_AUTH_SEC_CON) != 0)
       && ((p_pairing->ikey_dist & GAP_KDIST_LINKKEY) != 0 )
       && ((p_pairing->rkey_dist & GAP_KDIST_LINKKEY) != 0 ))
    {
        p_pairing->auth      |= GAP_AUTH_CT2;
    }
    else
    #endif  // (BT_HOST_PRESENT)
    {
        p_pairing->ikey_dist &= ~GAP_KDIST_LINKKEY;
        p_pairing->rkey_dist &= ~GAP_KDIST_LINKKEY;
        p_pairing->auth      &= ~GAP_AUTH_CT2;
    }
}

/*
 * HCI HANDLERS DEFINITIONS
 ****************************************************************************************
 */

#if (HL_LE_PERIPHERAL)
/// @brief Handles long term key request from link layer.
void gapc_le_smp_hci_le_ltk_request_evt_handler(uint8_t conidx, gapc_le_con_t* p_con)
{
    gapc_le_smp_pair_proc_t* p_proc;
    if(gapc_le_smp_is_peer_message_expected(p_con, GAPC_LE_SMP_W4_ENC, &p_proc))
    {
        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_HCI_LTK_REQ_RECEIVED, GAP_ERR_NO_ERROR);
    }
    else
    {
        // reject request
        HL_HCI_BASIC_CMD_SEND_WITH_CONHDL(HCI_LE_LTK_REQ_NEG_REPLY_CMD_OPCODE, gapc_get_conhdl(conidx), 0,
                                          gapc_proc_ignore_hci_evt_handler);
    }
}
#endif //(HL_LE_PERIPHERAL)

/// @brief Handles common command status events from HCI.
__STATIC void gapc_hci_le_en_enc_stat_evt_handler(uint16_t opcode, uint16_t conidx, struct hci_cmd_stat_event const *p_evt)
{
    gapc_le_smp_pair_proc_t* p_proc = gapc_le_smp_get_pairing_proc(conidx);
    if(p_proc != NULL)
    {
        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_HCI_ENC_CMD_EVT_RECEIVED, RW_ERR_HCI_TO_HL(p_evt->status));
    }
}

/// @brief Define what to do once a start encryption procedure has been successfully finished.
void gapc_le_smp_pairing_enc_change_evt_handle(gapc_le_con_t* p_con, uint16_t status)
{
    gapc_le_smp_pair_proc_t *p_proc = gapc_le_smp_get_pairing_proc(p_con->hdr.conidx);
    ASSERT_ERR(p_proc != NULL);
    // if fails -- No RA timer, no PDU
    gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_HCI_ENC_CHANGED_EVT_RECEIVED, status);
}

#if (HL_LE_CENTRAL)
/// @brief Send a request to the controller to start the encryption procedure.
uint16_t gapc_le_smp_send_start_enc_cmd(gapc_le_smp_pair_proc_t* p_proc)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;
    struct hci_le_en_enc_cmd *p_cmd = HL_HCI_CMD_ALLOC(HCI_LE_EN_ENC_CMD_OPCODE, hci_le_en_enc_cmd);
    if(p_cmd != NULL)
    {
        p_cmd->conhdl = gapc_get_conhdl(p_proc->hdr.conidx);

        // Set EDIV and Rand values to 0
        p_cmd->enc_div = 0;
        memset(&p_cmd->nb.nb[0], 0x00, GAP_RAND_NB_LEN);
        // Copy the p_key
        memcpy(&p_cmd->ltk.ltk[0], p_proc->key, GAP_KEY_LEN);

        HL_HCI_CMD_SEND_TO_CTRL(p_cmd, p_proc->hdr.conidx, gapc_hci_le_en_enc_stat_evt_handler);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}
#endif // (HL_LE_CENTRAL)

#if (HL_LE_PERIPHERAL)
/// @brief Send the LTK provided by the HL to the controller.
uint16_t gapc_le_smp_send_ltk_req_rsp(gapc_le_smp_pair_proc_t* p_proc)
{
    uint16_t status = GAP_ERR_INSUFF_RESOURCES;

    // Reply that the encryption key has been found
    struct hci_le_ltk_req_reply_cmd *p_hci_cmd =
            HL_HCI_CMD_ALLOC(HCI_LE_LTK_REQ_REPLY_CMD_OPCODE, hci_le_ltk_req_reply_cmd);

    if(p_hci_cmd)
    {
        p_hci_cmd->conhdl = gapc_get_conhdl(p_proc->hdr.conidx);
        memcpy(&p_hci_cmd->ltk.ltk[0], p_proc->key, GAP_KEY_LEN);

        HL_HCI_CMD_SEND_TO_CTRL(p_hci_cmd, p_proc->hdr.conidx, gapc_hci_le_en_enc_stat_evt_handler);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}
#endif // (HL_LE_PERIPHERAL)

/*
 * L2CAP HANDLERS DEFINITIONS
 ****************************************************************************************
 */

/// @brief Callback used to handle PAIRING REQUEST PDU
void l2cap_smp_pairing_req_handler(gapc_le_con_t* p_con, l2cap_smp_pairing_req_t* p_pdu, co_buf_t* p_buf)
{
    #if (HL_LE_PERIPHERAL)
    uint8_t smp_status = L2CAP_SMP_ERR_NO_ERROR;
    do
    {
        gapc_le_smp_pair_proc_t *p_proc;
        uint8_t ra_status;

        // Ignore PDU if received by central
        if (GETB(p_con->hdr.info_bf, GAPC_ROLE) != ROLE_SLAVE) break;

        // Check SMP parameters
        smp_status = gapc_le_smp_check_pairing_param(p_pdu->iocap, p_pdu->auth, p_pdu->oob, p_pdu->key_size);
        if(smp_status != L2CAP_SMP_ERR_NO_ERROR) break;

        // Check the repeated attempts status
        ra_status = gapc_le_smp_check_repeated_attempts(p_con);

        // failed due to repeated attempts
        if(ra_status == GAPC_LE_SMP_REP_ATTEMPT) { smp_status = L2CAP_SMP_ERR_REPEATED_ATTEMPTS; }
        // ignore other repeated attempt status
        if(ra_status != GAPC_LE_SMP_NO_REP_ATTEMPTS) break;

        if(gapc_env.p_sec_cbs->pairing_req == NULL)
        {
            smp_status = L2CAP_SMP_ERR_PAIRING_NOT_SUPP;
            break;
        }

        // Check if a pairing or encryption procedure is currently in progress - If yes drop packet
        if (gapc_le_smp_is_pairing_ongoing(p_con) || GETB(p_con->hdr.info_bf, GAPC_WAIT_ENCRYPTION)) break;

        // check that minimum pairing authentication requirements are present
        if(((p_pdu->auth & GAP_AUTH_SEC_CON) == 0) && !gapm_is_legacy_pairing_supp())
        {
            smp_status = L2CAP_SMP_ERR_AUTH_REQ;
            gapc_le_smp_indicate_pairing_failed(p_con, GAPC_LE_SMP_ERR_TO_HL_ERR(LOC, smp_status));
            break;
        }

        // Create pairing structure
        if(gapc_le_smp_pairing_proc_create(p_con, gapc_le_smp_periph_pairing_start_proc_transition, &p_proc) != GAP_ERR_NO_ERROR)
        {
            #if (BT_HOST_PRESENT)
            if((gapc_env.p_sec_proc != NULL) && gapc_is_bt_connection(gapc_env.p_sec_proc->conidx))
            {
                smp_status = L2CAP_SMP_ERR_BREDR_PAIRING_IN_PROGRESS;
            }
            else
            #endif // (BT_HOST_PRESENT)
            {
                smp_status = L2CAP_SMP_ERR_UNSPECIFIED_REASON;
            }
            break;
        }

        // Keep the packet for later use
        memcpy(&(p_proc->pair_req_pdu), &(p_pdu->iocap), GAPC_LE_SMP_CODE_PAIRING_REQ_RESP_LEN - 1);

        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_PAIRING_REQ_RECEIVED, GAP_ERR_NO_ERROR);
    } while (0);

    if(smp_status != L2CAP_SMP_ERR_NO_ERROR)
    {
        gapc_le_smp_send_pairing_fail_pdu(p_con, smp_status);
    }
    #endif //(HL_LE_PERIPHERAL)
}

/// @brief Callback used to handle PAIRING RESPONSE PDU
void l2cap_smp_pairing_rsp_handler(gapc_le_con_t* p_con, l2cap_smp_pairing_rsp_t* p_pdu, co_buf_t* p_buf)
{
    #if (HL_LE_CENTRAL)
    gapc_le_smp_pair_proc_t* p_proc;
    if(gapc_le_smp_is_peer_message_expected(p_con, GAPC_LE_SMP_W4_RX_PAIRING_RSP, &p_proc))
    {
        // Keep the packet for confirm calculation use
        memcpy(&p_proc->pair_rsp_pdu, &(p_pdu->iocap), GAPC_LE_SMP_CODE_PAIRING_REQ_RESP_LEN - 1);
        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_PAIRING_RSP_RECEIVED, GAP_ERR_NO_ERROR);
    } // else ignore if not expected
    #endif // (HL_LE_CENTRAL)
}


/// @brief Callback used to handle PAIRING CONFIRM PDU
void l2cap_smp_pairing_confirm_handler(gapc_le_con_t* p_con, l2cap_smp_pairing_cfm_t* p_pdu, co_buf_t* p_buf)
{
    gapc_le_smp_pair_proc_t* p_proc;
    if(gapc_le_smp_is_peer_message_expected(p_con, GAPC_LE_SMP_W4_RX_PAIRING_CONFIRM, &p_proc))
    {
        memcpy(p_proc->peer_confirm, p_pdu->cfm_val, CFM_LEN);
        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_PEER_CONFIRM_RECEIVED, GAP_ERR_NO_ERROR);
    }  // else ignore if not expected
}

/// @brief Callback used to handle PAIRING RANDOM PDU
void l2cap_smp_pairing_random_handler(gapc_le_con_t* p_con, l2cap_smp_pairing_random_t* p_pdu, co_buf_t* p_buf)
{
    gapc_le_smp_pair_proc_t* p_proc;
    if(gapc_le_smp_is_peer_message_expected(p_con, GAPC_LE_SMP_W4_RX_PAIRING_RAND, &p_proc))
    {
        memcpy(&(p_proc->peer_rand[0]), &(p_pdu->random[0]), RAND_VAL_LEN);
        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_PEER_RANDOM_RECEIVED, GAP_ERR_NO_ERROR);
    } // else ignore if not expected
}

/*
 * AES FUNCTIONS AND HANDLERS
 ****************************************************************************************
 */

/// Local Random value computed
__STATIC void gapc_le_smp_generate_rand_cmp_cb(uint8_t aes_status, const uint8_t* aes_res, uint32_t conidx)
{
    gapc_le_smp_pair_proc_t* p_proc = gapc_le_smp_get_pairing_proc(conidx);
    if(p_proc != NULL)
    {
        uint16_t status = GAP_ERR_UNEXPECTED;
        if(aes_status == CO_BUF_ERR_NO_ERROR)
        {
            memcpy(&p_proc->local_rand[0], aes_res, RAND_VAL_LEN);
            status = GAP_ERR_NO_ERROR;
        }

        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_LOCAL_RAND_GENERATED, status);
    }
}

void gapc_le_smp_generate_rand(gapc_le_smp_pair_proc_t* p_proc)
{
    aes_rand(gapc_le_smp_generate_rand_cmp_cb, p_proc->hdr.conidx);
}

/// Local value confirm computed
void gapc_le_smp_local_confirm_cmp_cb(uint8_t aes_status, const uint8_t* aes_res, uint32_t conidx)
{
    gapc_le_smp_pair_proc_t* p_proc = gapc_le_smp_get_pairing_proc(conidx);
    if(p_proc != NULL)
    {
        uint16_t status = GAP_ERR_UNEXPECTED;
        if(aes_status == CO_BUF_ERR_NO_ERROR)
        {
            memcpy(p_proc->local_confirm, aes_res, CFM_LEN);
            status = GAP_ERR_NO_ERROR;
        }

        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_LOCAL_CONFIRM_COMPUTED, status);
    }
}


/// remote value confirm computed
void gapc_le_smp_peer_confirm_cmp_cb(uint8_t aes_status, const uint8_t* aes_res, uint32_t conidx)
{
    gapc_le_smp_pair_proc_t* p_proc = gapc_le_smp_get_pairing_proc(conidx);
    if(p_proc != NULL)
    {
        uint16_t status = GAP_ERR_UNEXPECTED;
        SETB(p_proc->info_bf, GAPC_LE_SMP_OOB_CFM_COMP, true);
        if(aes_status == CO_BUF_ERR_NO_ERROR)
        {
            status = (memcmp(p_proc->peer_confirm, aes_res, GAP_KEY_LEN) == 0)
                   ? GAP_ERR_NO_ERROR : GAPC_LE_SMP_ERR_TO_HL_ERR(LOC, L2CAP_SMP_ERR_CONF_VAL_FAILED);
        }

        if(status != GAP_ERR_NO_ERROR)
        {
            // inform peer device that pairing fails
            gapc_le_smp_send_pairing_fail_pdu_and_start_ra_timer(p_proc, L2CAP_SMP_ERR_CONF_VAL_FAILED);
        }

        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_PEER_CONFIRM_COMPUTED, status);
    }
}


/*
 * STATE MACHINE FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if (HL_LE_CENTRAL)
/// Function that handles transition for central to start pairing procedure
__STATIC uint16_t gapc_le_smp_central_pairing_start_proc_transition(gapc_le_smp_pair_proc_t* p_proc,
                                                                    uint8_t event, bool* p_is_finished)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    *p_is_finished = false;

    switch(event)
    {
        case GAPC_LE_SMP_EVT_APPLI_INITIATE_PAIRING:
        {
            // Force key size to Max value if Secure connection enabled
            if((p_proc->pair_req_pdu.auth & GAP_AUTH_SEC_CON) != 0)
            {
                p_proc->pair_req_pdu.key_size = GAPC_LE_SMP_MAX_ENC_SIZE_LEN;
            }

            // If device is not bondable, useless to ask for keys
            if ((p_proc->pair_req_pdu.auth & GAP_AUTH_BOND) == 0)
            {
                p_proc->pair_req_pdu.ikey_dist = 0x00;
                p_proc->pair_req_pdu.rkey_dist = 0x00;
            }

            // prepare link key derivation if enabled
            gapc_le_smp_prepare_for_link_key_derivation(&(p_proc->pair_req_pdu));

            // Send the pairing request PDU
            status = gapc_le_smp_send_pairing_req_pdu(p_proc);
            // Waiting for the pairing response
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_PAIRING_RSP;
        } break;

        case GAPC_LE_SMP_EVT_PAIRING_RSP_RECEIVED:
        {
            gapc_pairing_t* p_pair_rsp = &(p_proc->pair_rsp_pdu);
            gapc_pairing_t* p_pair_req = &(p_proc->pair_req_pdu);

            // Check received parameters
            uint8_t smp_status = gapc_le_smp_check_pairing_param(p_pair_rsp->iocap, p_pair_rsp->auth, p_pair_rsp->oob,
                                                                 p_pair_rsp->key_size);
            if(smp_status == L2CAP_SMP_ERR_NO_ERROR)
            {
                // compute key size
                p_proc->key_size = co_min(p_pair_req->key_size, p_pair_rsp->key_size);

                // Check if secure connection must be used
                if ((p_pair_rsp->auth & GAP_AUTH_SEC_CON) && (p_pair_req->auth & GAP_AUTH_SEC_CON))
                {
                    SETB(p_proc->info_bf, GAPC_LE_SMP_SC_PAIRING, true);
                }

                // Check if security requirement reached for pairing
                smp_status = gapc_le_check_pairing_security_reached(p_proc);
            }

            if(smp_status != L2CAP_SMP_ERR_NO_ERROR)
            {
                // pairing failed
                gapc_le_smp_send_pairing_fail_pdu_and_start_ra_timer(p_proc, smp_status);
                status = GAPC_LE_SMP_ERR_TO_HL_ERR(LOC, smp_status);
            }
            else
            {
                // select next transition function
                if(GETB(p_proc->info_bf, GAPC_LE_SMP_SC_PAIRING))
                {
                    status = gapc_le_smp_sc_central_proc_start(p_proc, p_is_finished);
                }
                else
                {
                    status = gapc_le_smp_legacy_central_proc_start(p_proc, p_is_finished);
                }
            }
        } break;
        default: { /* ignore */ } break;
    }

    return(status);
}
#endif // (HL_LE_CENTRAL)

#if (HL_LE_PERIPHERAL)
/// Function that handles transition for peripheral to start pairing procedure
__STATIC uint16_t gapc_le_smp_periph_pairing_start_proc_transition(gapc_le_smp_pair_proc_t* p_proc,
                                                                   uint8_t event, bool* p_is_finished)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    *p_is_finished = false;

    switch(event)
    {
        case GAPC_LE_SMP_EVT_PAIRING_REQ_RECEIVED:
        {
            // Update the internal state
            p_proc->app_exp_value = GAPC_LE_SMP_VALUE_PAIRING_CFM;
            SETB(p_proc->info_bf, GAPC_LE_SMP_IN_PAIRING_PROC, true);
            // Inform the application that a pairing request has been received
            gapc_env.p_sec_cbs->pairing_req(p_proc->hdr.conidx,p_proc->hdr.dummy,
                                            p_proc->pair_req_pdu.auth & GAP_AUTH_REQ_MASK);
            SETB(p_proc->info_bf, GAPC_LE_SMP_IN_PAIRING_PROC, false);
            // if application anwer not yet received
            if(!gapc_le_smp_is_app_answer_request(p_proc)) break;
        }
        // no break;
        case GAPC_LE_SMP_EVT_APPLI_ANSWER_PAIRING_REQ:
        {
            uint16_t smp_status = L2CAP_SMP_ERR_UNSPECIFIED_REASON;
            do
            {
                gapc_pairing_t* p_pair_rsp = &(p_proc->pair_rsp_pdu);
                gapc_pairing_t* p_pair_req = &(p_proc->pair_req_pdu);

                // no need to continue if user abort pairing
                if(!GETB(p_proc->info_bf, GAPC_LE_SMP_USER_ACCEPT)) break;

                // if secure connection not supported, ensure that Secure connection flag is not set
                if(!gapm_is_sec_con_pairing_supp())
                {
                    p_pair_rsp->auth &= ~GAP_AUTH_SEC_CON;
                }

                // for key size to 16 if secure connection pairing
                if((p_pair_rsp->auth & GAP_AUTH_SEC_CON) != 0)
                {
                    p_proc->key_size = GAPC_LE_SMP_MAX_ENC_SIZE_LEN;
                }

                smp_status = gapc_le_smp_check_app_pairing_param(p_pair_rsp);
                if(smp_status != L2CAP_SMP_ERR_NO_ERROR) break;

                // compute key size
                p_proc->key_size = co_min(p_pair_req->key_size, p_pair_rsp->key_size);
                // Check if secure connection must be used
                SETB(p_proc->info_bf, GAPC_LE_SMP_SC_PAIRING,
                        ((p_pair_rsp->auth & GAP_AUTH_SEC_CON) && (p_pair_req->auth & GAP_AUTH_SEC_CON)));

                // Check if security requirement reached for pairing
                smp_status = gapc_le_check_pairing_security_reached(p_proc);
                if(smp_status != L2CAP_SMP_ERR_NO_ERROR) break;

                // Adjust the pairing features according on master's
                // If device is not bondable, useless to ask for keys
                if ((p_pair_rsp->auth & GAP_AUTH_BOND) == 0)
                {
                    p_pair_rsp->ikey_dist    = 0x00;
                    p_pair_rsp->rkey_dist    = 0x00;
                }
                else
                {
                    p_pair_rsp->ikey_dist &= p_pair_req->ikey_dist;
                    p_pair_rsp->rkey_dist &= p_pair_req->rkey_dist;
                }

                // prepare link key derivation if enabled
                gapc_le_smp_prepare_for_link_key_derivation(p_pair_rsp);

                // Send the pairing response pdu
                status = gapc_le_smp_send_pairing_rsp_pdu(p_proc);
                if(status != GAP_ERR_NO_ERROR) break;

                // select next transition function
                if(GETB(p_proc->info_bf, GAPC_LE_SMP_SC_PAIRING))
                {
                    status = gapc_le_smp_sc_periph_proc_start(p_proc, p_is_finished);
                }
                else
                {
                    status = gapc_le_smp_legacy_periph_proc_start(p_proc, p_is_finished);
                }
            } while (0);

            // send pairing failed PDU and stop procedure
            if(smp_status != L2CAP_SMP_ERR_NO_ERROR)
            {
                gapc_le_smp_send_pairing_fail_pdu_and_start_ra_timer(p_proc, smp_status);
                status = GAPC_LE_SMP_ERR_TO_HL_ERR(LOC, smp_status);
            }
        } break;

        default: { /* ignore */ } break;
    }

    return(status);
}
#endif // (HL_LE_PERIPHERAL)


/*
 * INFORM APPLICATION
 ****************************************************************************************
 */

/// Ask a specific information to the application
void gapc_le_smp_ask_info_to_app(gapc_le_smp_pair_proc_t* p_proc, uint8_t exp_info, uint8_t exp_value)
{
    SETB(p_proc->info_bf, GAPC_LE_SMP_IN_PAIRING_PROC, true);
    p_proc->app_exp_value = exp_value;
    gapc_env.p_sec_cbs->info_req(p_proc->hdr.conidx, p_proc->hdr.dummy, exp_info);
    SETB(p_proc->info_bf, GAPC_LE_SMP_IN_PAIRING_PROC, false);
}


/// Indicate that pairing failes
void gapc_le_smp_indicate_pairing_failed(gapc_le_con_t* p_con, uint16_t reason)
{
    gapc_env.p_sec_cbs->pairing_failed(p_con->hdr.conidx, p_con->hdr.dummy, reason);
}

/*
 * INTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/// Get if application involved in pairing
bool gapc_le_smp_is_app_involved(uint8_t conidx, uint8_t provided_value, gapc_le_smp_pair_proc_t** pp_proc)
{
    bool is_app_involved = false;
    gapc_le_smp_pair_proc_t* p_proc = gapc_le_smp_get_pairing_proc(conidx);
    if(p_proc != NULL)
    {
        if(p_proc->app_exp_value == provided_value)
        {
            p_proc->app_exp_value = GAPC_LE_SMP_VALUE_INVALID;
            *pp_proc = p_proc;
            is_app_involved = true;
        }
    }
    return (is_app_involved);
}

bool gapc_le_smp_is_peer_message_expected(gapc_le_con_t* p_con, uint8_t message, gapc_le_smp_pair_proc_t** pp_proc)
{
    bool is_peer_message_expected = false;
    gapc_le_smp_pair_proc_t* p_proc = gapc_le_smp_get_pairing_proc(p_con->hdr.conidx);
    if(p_proc != NULL)
    {
        if(p_proc->peer_exp_message == message)
        {
            // prevent duplicated message received
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_NOTHING;
            is_peer_message_expected = true;
            *pp_proc = p_proc;
        }
    }
    return (is_peer_message_expected);
}

gapc_le_smp_pair_proc_t* gapc_le_smp_get_pairing_proc(uint8_t conidx)
{
    gapc_le_smp_pair_proc_t* p_proc = NULL;
    if((gapc_env.p_sec_proc != NULL) && (gapc_env.p_sec_proc->conidx == conidx))
    {
        p_proc = (gapc_le_smp_pair_proc_t*) gapc_env.p_sec_proc;
    }
    return p_proc;
}

/// @brief Handles SMP Transaction or Repeatead attempt timeout expiration
void gapc_le_smp_timeout_handler(gapc_sdt_t* p_hdl, uint8_t conidx)
{
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);

    // Repeated attempt timer
    if(p_hdl == &(p_con->smp.rep_attempt_timer))
    {
        ASSERT_ERR(GETB(p_con->smp.timer_state, GAPC_LE_SMP_TIMER_REP_ATT));

        // Reset the timer value
        p_con->smp.rep_att_timer_val = GAPC_LE_SMP_REP_ATTEMPTS_TIMER_DEF_VAL_MS;
        // Update the timer flag
        SETB(p_con->smp.timer_state, GAPC_LE_SMP_TIMER_REP_ATT, false);
    }
    // Transaction timer
    else
    {
        gapc_le_smp_pair_proc_t *p_proc = gapc_le_smp_get_pairing_proc(p_con->hdr.conidx);
        ASSERT_ERR(p_proc != NULL);

        // No more SM procedure can be initiated
        SETB(p_con->smp.timer_state, GAPC_LE_SMP_TIMER_TIMEOUT_BLOCKED, true);
        // Stop Pairing due to a timeout
        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_PAIRING_ABORTED, GAP_ERR_TIMEOUT);
    }
}

/// @brief Start the timer used to detect a Repeated Attempts attack
void gapc_le_smp_start_rep_attempt_timer(gapc_le_con_t* p_con)
{
    // Start the timer
    gapc_sdt_timer_set(&(p_con->smp.rep_attempt_timer), p_con->smp.rep_att_timer_val);

    // Set the status in the environment
    SETB(p_con->smp.timer_state, GAPC_LE_SMP_TIMER_REP_ATT, true);
}

/// @brief Check if an attack by repeated attempts has been triggered by the peer device
uint8_t gapc_le_smp_check_repeated_attempts(gapc_le_con_t* p_con)
{
    // Returned status
    uint8_t ra_status = GAPC_LE_SMP_NO_REP_ATTEMPTS;

    if (GETB(p_con->smp.timer_state, GAPC_LE_SMP_TIMER_REP_ATT))
    {
        // Check if an attack has already been detected
        if (p_con->smp.rep_att_timer_val != GAPC_LE_SMP_REP_ATTEMPTS_TIMER_MAX_VAL_MS)
        {
            // The timer value shall be increased exponentially if a repeated attempt occurs
            p_con->smp.rep_att_timer_val *= GAPC_LE_SMP_REP_ATTEMPTS_TIMER_MULT;

            // Check if the timer value is upper than the max limit
            if (p_con->smp.rep_att_timer_val >= GAPC_LE_SMP_REP_ATTEMPTS_TIMER_MAX_VAL_MS)
            {
                p_con->smp.rep_att_timer_val = GAPC_LE_SMP_REP_ATTEMPTS_TIMER_MAX_VAL_MS;

                // Inform application about repeated attempt problem
                if(gapc_env.p_sec_cbs->repeated_attempt != NULL)
                {
                    gapc_env.p_sec_cbs->repeated_attempt(p_con->hdr.conidx, p_con->hdr.dummy);
                }
            }

            ra_status = GAPC_LE_SMP_REP_ATTEMPT;
        }
        else
        {
            // New attack attempt, the pairing request PDU will be dropped
            ra_status = GAPC_LE_SMP_REP_ATTEMPTS_ATTACK;
        }

        // Restart the timer
        gapc_le_smp_start_rep_attempt_timer(p_con);
    }
    // else status is GAP_ERR_NO_ERROR

    return (ra_status);
}

uint16_t gapc_le_smp_check_user_accept(gapc_le_smp_pair_proc_t* p_proc, uint8_t reason)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    if(!GETB(p_proc->info_bf, GAPC_LE_SMP_USER_ACCEPT))
    {
        // Send the Pairing Failed PDU to the peer device
        gapc_le_smp_send_pairing_fail_pdu_and_start_ra_timer(p_proc, reason);
        status = GAPC_LE_SMP_ERR_TO_HL_ERR(LOC, reason);
    }
    return (status);
}

void gapc_le_smp_proc_transition(gapc_le_smp_pair_proc_t *p_proc, uint8_t event, uint16_t status)
{
    bool is_finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        status = p_proc->hdr.transition_cb((gapc_sec_proc_hdr_t*)p_proc, event, &is_finished);
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        is_finished = true;
    }

    if((status != GAP_ERR_NO_ERROR) || is_finished)
    {
        gapc_le_con_t* p_con = gapc_le_con_env_get(p_proc->hdr.conidx);

        // Stop the timeout timer if needed
        gapc_le_smp_stop_trans_timer(&(p_proc->hdr));
        // release any OOB data present in memory
        gapm_oob_data_release();

        if(status != GAP_ERR_NO_ERROR)
        {
            // pairing failed
            gapc_le_smp_indicate_pairing_failed(p_con, status);
        }
        else
        {
            // update link security level
            gapc_sec_lvl_set(&(p_con->hdr), true, p_proc->pairing_lvl, GETB(p_proc->info_bf, GAPC_LE_SMP_LTK_EXCH));

            // pairing succeed
            gapc_env.p_sec_cbs->pairing_succeed(p_proc->hdr.conidx, p_proc->hdr.dummy);
            p_con->smp.key_size = p_proc->key_size;

            // Send Bond data if internal states have been updated
            if(GETB(p_con->hdr.bond.info_bf, GAPC_BONDED) && GETB(p_con->hdr.bond.info_bf, GAPC_BOND_DATA_UPDATED))
            {
                gapc_bond_info_send(p_proc->hdr.conidx);
            }
        }

        ke_free(gapc_env.p_sec_proc);
        gapc_env.p_sec_proc = NULL;
    }
}

/// Try to do procedure transition, nothing is performed if already in procedure.
void gapc_le_smp_try_proc_transition(gapc_le_smp_pair_proc_t* p_proc, uint8_t event)
{
    if(!GETB(p_proc->info_bf, GAPC_LE_SMP_IN_PAIRING_PROC))
    {
        gapc_le_smp_proc_transition(p_proc, event, GAP_ERR_NO_ERROR);
    }
}

void gapc_le_smp_pairing_proc_cleanup(gapc_le_con_t* p_con)
{
    gapc_le_smp_pair_proc_t* p_proc = gapc_le_smp_get_pairing_proc(p_con->hdr.conidx);
    // Release the pairing information structure
    if (p_proc != NULL)
    {
        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_PAIRING_ABORTED, GAP_ERR_DISCONNECTED);
    }
}

/*
 * EXTERNALS FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if(HL_LE_CENTRAL)
uint16_t gapc_le_start_pairing(uint8_t conidx, const gapc_pairing_t* p_pairing_info, uint8_t sec_req_level)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);

    do
    {
        gapc_le_smp_pair_proc_t* p_proc;

        // sanity checks
        if(p_con == NULL) break;

        if(GETB(p_con->hdr.info_bf, GAPC_ROLE) != ROLE_MASTER)
        {
            status = GAP_ERR_NOT_SUPPORTED;
            break;
        }

        // New pairing cannot be started if pairing on-going or encryption procedure
        if(gapc_le_smp_is_pairing_ongoing(p_con) || GETB(p_con->hdr.info_bf, GAPC_WAIT_ENCRYPTION))
        {
            status = GAP_ERR_BUSY;
            break;
        }

        // Check SMP Timeout timer state
        if (GETB(p_con->smp.timer_state, GAPC_LE_SMP_TIMER_TIMEOUT_BLOCKED))
        {
            /// Once a timeout has occurred, no security procedure can be initiated until a new
            /// physical link has been established.
            status = GAP_ERR_TIMEOUT;
            break;
        }

        // Check Repeated Attempts Timer state
        if (GETB(p_con->smp.timer_state, GAPC_LE_SMP_TIMER_REP_ATT))
        {
            status = GAPC_LE_SMP_ERR_TO_HL_ERR(LOC, L2CAP_SMP_ERR_REPEATED_ATTEMPTS);
            break;
        }

        // Check provided parameters and convert to HL error code
        status = gapc_le_smp_check_app_pairing_param(p_pairing_info);
        status = ((status == L2CAP_SMP_ERR_NO_ERROR) ? GAP_ERR_NO_ERROR : GAPC_LE_SMP_ERR_TO_HL_ERR(LOC, status));
        if(status != GAP_ERR_NO_ERROR) break;

        // Create/allocate pairing procedure
        status = gapc_le_smp_pairing_proc_create(p_con, gapc_le_smp_central_pairing_start_proc_transition, &p_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        // Copy the pairing features
        p_proc->pair_req_pdu = *p_pairing_info;
        p_proc->sec_req_level = sec_req_level;

        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_APPLI_INITIATE_PAIRING, GAP_ERR_NO_ERROR);
    } while(0);

    return (status);

}
#endif // (HL_LE_CENTRAL)

#if(HL_LE_PERIPHERAL)

uint16_t gapc_le_send_security_request(uint8_t conidx, uint8_t auth)
{
    uint16_t status;

    do
    {
        gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);

        if((p_con == NULL) || gapc_le_smp_is_pairing_ongoing(p_con))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        if(GETB(p_con->hdr.info_bf, GAPC_ROLE) != ROLE_SLAVE)
        {
            status = GAP_ERR_NOT_SUPPORTED;
            break;
        }

        // Check SMP Timeout timer state
        if (GETB(p_con->smp.timer_state, GAPC_LE_SMP_TIMER_TIMEOUT_BLOCKED))
        {
             // Once a timeout has occurred, no security procedure can be initiated until a new
             // physical link has been established.
            status = GAP_ERR_TIMEOUT;
            break;
        }

        // Check Repeated Attempts Timer state
        if (GETB(p_con->smp.timer_state, GAPC_LE_SMP_TIMER_REP_ATT))
        {
            status = GAPC_LE_SMP_ERR_TO_HL_ERR(LOC, L2CAP_SMP_ERR_REPEATED_ATTEMPTS);
            break;
        }

        // Check pairing mode accepted
        if (   (((auth & GAP_AUTH_SEC_CON) != 0) && !gapm_is_sec_con_pairing_supp())
            || (((auth & GAP_AUTH_SEC_CON) == 0) && !gapm_is_legacy_pairing_supp()))
        {
            status = GAPC_LE_SMP_ERR_TO_HL_ERR(LOC, L2CAP_SMP_ERR_AUTH_REQ);
            break;
        }

        status = gapc_le_smp_send_security_req_pdu(p_con, auth);
    } while (0);

    return (status);
}

uint16_t gapc_pairing_accept(uint8_t conidx, bool accept, const gapc_pairing_t* p_pairing_info, uint8_t sec_req_level)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapc_le_smp_pair_proc_t* p_proc;

    if(accept && (p_pairing_info == NULL))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    if(gapc_le_smp_is_app_involved(conidx, GAPC_LE_SMP_VALUE_PAIRING_CFM, &p_proc))
    {
        SETB(p_proc->info_bf, GAPC_LE_SMP_USER_ACCEPT, accept != false);
        if(accept)
        {
            p_proc->sec_req_level = sec_req_level;
            p_proc->pair_rsp_pdu  = *p_pairing_info;
        }

        gapc_le_smp_try_proc_transition(p_proc, GAPC_LE_SMP_EVT_APPLI_ANSWER_PAIRING_REQ);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}
#endif //(HL_LE_PERIPHERAL)


uint16_t gapc_le_smp_transition_to_new_state_machine(gapc_le_smp_pair_proc_t* p_proc,
                                                     gapc_le_smp_proc_transition_cb transition_cb,
                                                     uint8_t event,
                                                     bool* p_is_finished)
{
    p_proc->hdr.transition_cb = (gapc_sec_proc_transition_cb) transition_cb;
    return (transition_cb(p_proc, event, p_is_finished));
}



#endif // (BLE_GAPC)

/// @} GAPC
