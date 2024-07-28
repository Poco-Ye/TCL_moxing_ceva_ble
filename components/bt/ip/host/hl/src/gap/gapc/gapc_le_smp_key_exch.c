/**
 ****************************************************************************************
 *
 * @file gapc_le_smp_key_exch.c
 *
 * @brief GAPC LE SMP - Handle Key Exchange
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
#if (BT_HOST_PRESENT)
#include "aes.h"
#endif // (BT_HOST_PRESENT)

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

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

#if (BT_HOST_PRESENT)
/// KEY identifier for Link Key Generation using h7() algorithm : "tmp1"
__STATIC const uint8_t gapc_le_smp_h7_ilk_salt[]   = {'1', 'p', 'm', 't', '\x00','\x00','\x00','\x00','\x00','\x00','\x00','\x00','\x00','\x00','\x00','\x00'};
/// KEY identifier for Link Key Generation using h6() algorithm : "tmp1"
__STATIC const uint8_t gapc_le_smp_h6_ilk_key_id[] = {'1', 'p', 'm', 't'};
/// KEY identifier for Link Key Generation using h6() algorithm : "lebr"
__STATIC const uint8_t gapc_le_smp_h6_lk_key_id[]  = {'r', 'b', 'e', 'l'};

///// KEY identifier for ILTK Generation using h7() algorithm : "tmp2"
//__STATIC const uint8_t gapc_le_smp_h7_iltk_salt[]   = {'2', 'p', 'm', 't', '\x00','\x00','\x00','\x00','\x00','\x00','\x00','\x00','\x00','\x00','\x00','\x00'};
///// KEY identifier for ILTK Generation using h6() algorithm : "tmp2"
//__STATIC const uint8_t gapc_le_smp_h6_iltk_key_id[] = {'2', 'p', 'm', 't'};
///// KEY identifier for LTK Generation using h6() algorithm : "brle"
//__STATIC const uint8_t gapc_le_smp_h6_ltk_key_id[]  = {'e', 'l', 'r', 'b'};
#endif // (BT_HOST_PRESENT)

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/*
 * HCI HANDLERS FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/*
 * L2CAP SMP HANDLERS FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/// @brief Callback used to handle ENCRYPTION INFOPDU
void l2cap_smp_encryption_inf_handler(gapc_le_con_t* p_con, l2cap_smp_encryption_inf_t* p_pdu, co_buf_t* p_buf)
{
    gapc_le_smp_pair_proc_t* p_proc;
    if(gapc_le_smp_is_peer_message_expected(p_con, GAPC_LE_SMP_W4_RX_LTK, &p_proc))
    {
        // Store the LTK
        memcpy(&(p_proc->hdr.pairing_keys.ltk.key), p_pdu->ltk, GAP_KEY_LEN);
        p_proc->hdr.pairing_keys.valid_key_bf |= GAP_KDIST_ENCKEY;
        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_PEER_LTK_RECEIVED, GAP_ERR_NO_ERROR);
    } // else ignore if not expected
}

/// @brief Callback used to handle MASTER IDENTIFICATION PDU
void l2cap_smp_master_id_handler(gapc_le_con_t* p_con, l2cap_smp_master_id_t* p_pdu, co_buf_t* p_buf)
{
    gapc_le_smp_pair_proc_t* p_proc;
    if(gapc_le_smp_is_peer_message_expected(p_con, GAPC_LE_SMP_W4_RX_MST_ID, &p_proc))
    {
        // Store EDIV and RandNB
        p_proc->hdr.pairing_keys.ltk.ediv = co_read16p(&(p_pdu->ediv));
        p_proc->hdr.pairing_keys.ltk.key_size = p_proc->key_size;
        memcpy(&(p_proc->hdr.pairing_keys.ltk.randnb), p_pdu->nb, GAP_RAND_NB_LEN);
        p_proc->hdr.pairing_keys.valid_key_bf |= GAP_KDIST_ENCKEY;
        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_PEER_MASTER_ID_RECEIVED, GAP_ERR_NO_ERROR);
    } // else ignore if not expected
}

/// @brief Callback used to handle IDENTITY INFORMATION PDU
void l2cap_smp_identity_inf_handler(gapc_le_con_t* p_con, l2cap_smp_identity_inf_t* p_pdu, co_buf_t* p_buf)
{
    gapc_le_smp_pair_proc_t* p_proc;
    if(gapc_le_smp_is_peer_message_expected(p_con, GAPC_LE_SMP_W4_RX_IRK, &p_proc))
    {
        // Store the IRK
        memcpy(&(p_proc->hdr.pairing_keys.irk.key), p_pdu->irk, GAP_KEY_LEN);
        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_PEER_IRK_RECEIVED, GAP_ERR_NO_ERROR);
    } // else ignore if not expected
}

/// @brief Callback used to handle IDENTITY ADDRESS INFORMATION PDU
void l2cap_smp_id_addr_inf_handler(gapc_le_con_t* p_con, l2cap_smp_id_addr_inf_t* p_pdu, co_buf_t* p_buf)
{
    gapc_le_smp_pair_proc_t* p_proc;
    if(gapc_le_smp_is_peer_message_expected(p_con, GAPC_LE_SMP_W4_RX_BD_ADDR, &p_proc))
    {
        // Store BD Address
        memcpy(&(p_proc->hdr.pairing_keys.irk.addr), &(p_pdu->addr), BD_ADDR_LEN);
        p_proc->hdr.pairing_keys.irk.addr.addr_type = p_pdu->addr_type;
        p_proc->hdr.pairing_keys.valid_key_bf |= GAP_KDIST_IDKEY;
        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_PEER_ADDR_INFO_RECEIVED, GAP_ERR_NO_ERROR);
    } // else ignore if not expected
}

/// @brief Callback used to handle SIGNING INFORMATION PDU
void l2cap_smp_signing_inf_handler(gapc_le_con_t* p_con, l2cap_smp_signing_inf_t* p_pdu, co_buf_t* p_buf)
{
    gapc_le_smp_pair_proc_t* p_proc;
    if(gapc_le_smp_is_peer_message_expected(p_con, GAPC_LE_SMP_W4_RX_CSRK, &p_proc))
    {
        // Store CSRK
        memcpy(&(p_proc->hdr.pairing_keys.csrk), p_pdu->csrk, GAP_KEY_LEN);
        p_proc->hdr.pairing_keys.valid_key_bf |= GAP_KDIST_SIGNKEY;

        // update connection environment variables and reset sign counter
        memcpy(&(p_con->hdr.bond.csrk[GAPC_INFO_SRC_PEER]), p_pdu->csrk, GAP_KEY_LEN);
        p_con->hdr.bond.sign_counter[GAPC_INFO_SRC_PEER] = 0;

        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_PEER_CSRK_RECEIVED, GAP_ERR_NO_ERROR);

    } // else ignore if not expected
}

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if (BT_HOST_PRESENT)
/// Function called when Link Key is computed
__STATIC void gapc_le_smp_key_lk_computed(uint8_t aes_status, const uint8_t* p_link_key, uint32_t conidx)
{
    gapc_le_smp_pair_proc_t* p_proc = gapc_le_smp_get_pairing_proc(conidx);
    if(p_proc != NULL)
    {
        uint16_t status = RW_ERR_HCI_TO_HL(aes_status);
        if(status == GAP_ERR_NO_ERROR)
        {
            memcpy(p_proc->hdr.pairing_keys.link_key.key, p_link_key, KEY_LEN);
            p_proc->hdr.pairing_keys.valid_key_bf |= GAP_KDIST_LINKKEY;
        }

        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_DERIV_KEY_COMPUTED, status);
    }
}

/// Function called when ILK is computed
__STATIC void gapc_le_smp_key_ilk_computed(uint8_t aes_status, const uint8_t* p_ilk, uint32_t conidx)
{
    gapc_le_smp_pair_proc_t* p_proc = gapc_le_smp_get_pairing_proc(conidx);
    if(p_proc != NULL)
    {
        uint16_t status = RW_ERR_HCI_TO_HL(aes_status);
        if(status == GAP_ERR_NO_ERROR)
        {
            // BR/EDR link key = h6(ILK, "lebr")
            aes_h6(p_ilk, gapc_le_smp_h6_lk_key_id, gapc_le_smp_key_lk_computed, p_proc->hdr.conidx);
        }
        else
        {
            gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_DERIV_KEY_COMPUTED, status);
        }
    }
}

/// Start computation of Link key derived from LTK
__STATIC uint16_t gapc_le_smp_key_start_compute_derivation_from_link_key(gapc_le_smp_pair_proc_t* p_proc)
{
    if((p_proc->pair_req_pdu.auth & p_proc->pair_rsp_pdu.auth & GAP_AUTH_CT2) != 0)
    {
        // ILK = h7(SALT, LTK)
        aes_h7(gapc_le_smp_h7_ilk_salt, p_proc->hdr.pairing_keys.link_key.key, gapc_le_smp_key_ilk_computed, p_proc->hdr.conidx);
    }
    else
    {
        // ILK = h6(LTK, "tmp1")
        aes_h6(p_proc->hdr.pairing_keys.link_key.key, gapc_le_smp_h6_ilk_key_id, gapc_le_smp_key_ilk_computed, p_proc->hdr.conidx);
    }

    return (GAP_ERR_NO_ERROR);
}
#endif // (BT_HOST_PRESENT)



/// State machine that handle key reception
__STATIC uint16_t gapc_le_smp_key_exch_received_transition(gapc_le_smp_pair_proc_t* p_proc, uint8_t event,
                                                           bool* p_is_finished)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    switch(event)
    {
        case GAPC_LE_SMP_EVT_KEY_EXCHANGE_STARTED:
        {
            // ensure that pairing information are clean
            memset(&(p_proc->hdr.pairing_keys), 0, sizeof(gapc_pairing_keys_t));
            p_proc->hdr.pairing_keys.pairing_lvl = p_proc->pairing_lvl;

            if(GETB(p_proc->info_bf, GAPC_LE_SMP_SC_PAIRING))
            {
                p_proc->keys_dist &= ~GAP_KDIST_ENCKEY;
                memcpy(&(p_proc->hdr.pairing_keys.ltk.key), &(p_proc->key), GAP_KEY_LEN);
                p_proc->hdr.pairing_keys.ltk.key_size = p_proc->key_size;
                p_proc->hdr.pairing_keys.valid_key_bf |= GAP_KDIST_ENCKEY;
            }

            // Start key reception
            else if((p_proc->keys_dist & GAP_KDIST_ENCKEY) != 0)
            {
                p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_LTK;
                break;
            }
        }
        // no break

        case GAPC_LE_SMP_EVT_PEER_MASTER_ID_RECEIVED:
        {
            if((p_proc->keys_dist & GAP_KDIST_IDKEY) != 0)
            {
                p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_IRK;
                break;
            }
        }
        // no break

        case GAPC_LE_SMP_EVT_PEER_ADDR_INFO_RECEIVED:
        {
            if((p_proc->keys_dist & GAP_KDIST_SIGNKEY) != 0)
            {
                p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_CSRK;
                break;
            }
        }
        // no break

        case GAPC_LE_SMP_EVT_PEER_CSRK_RECEIVED:
        #if (BT_HOST_PRESENT)
        {
            if((p_proc->keys_dist & GAP_KDIST_LINKKEY) != 0)
            {
                status = gapc_le_smp_key_start_compute_derivation_from_link_key(p_proc);
                break;
            }
        }
        // no break
        case GAPC_LE_SMP_EVT_DERIV_KEY_COMPUTED:
        #endif // (BT_HOST_PRESENT)
        {
            if(p_proc->hdr.pairing_keys.valid_key_bf != 0)
            {
                // inform application about received pairing keys
                gapc_env.p_sec_cbs->key_received(p_proc->hdr.conidx, p_proc->hdr.dummy, &(p_proc->hdr.pairing_keys));
            }

            *p_is_finished = true;
        } break;

        // Simple State transition
        case GAPC_LE_SMP_EVT_PEER_LTK_RECEIVED: { p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_MST_ID;  } break;
        case GAPC_LE_SMP_EVT_PEER_IRK_RECEIVED: { p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_BD_ADDR; } break;
        default: { /* ignore */ } break;
    }

    return (status);
}


/// State machine that handle key transmission
__STATIC uint16_t gapc_le_smp_key_exch_send_transition(gapc_le_smp_pair_proc_t* p_proc, uint8_t event, bool* p_is_finished)
{
    uint16_t status = GAP_ERR_NO_ERROR;

    switch(event)
    {
        case GAPC_LE_SMP_EVT_KEY_EXCHANGE_STARTED:
        {
            if(GETB(p_proc->info_bf, GAPC_LE_SMP_SC_PAIRING))
            {
                p_proc->keys_dist &= ~GAP_KDIST_ENCKEY;
            }

            if(p_proc->keys_dist == 0)
            {
                *p_is_finished = true;
                break;
            }

            // ensure that pairing information are clean
            memset(&(p_proc->hdr.pairing_keys), 0, sizeof(gapc_pairing_keys_t));

            if((p_proc->keys_dist & GAP_KDIST_ENCKEY) != 0)
            {
                SETB(p_proc->info_bf, GAPC_LE_SMP_IN_PAIRING_PROC, true);
                p_proc->app_exp_value = GAPC_LE_SMP_VALUE_LTK;
                gapc_env.p_sec_cbs->ltk_req(p_proc->hdr.conidx, p_proc->hdr.dummy, p_proc->key_size);
                SETB(p_proc->info_bf, GAPC_LE_SMP_IN_PAIRING_PROC, false);
                // response not received in function execution
                if(!gapc_le_smp_is_app_answer_request(p_proc)) break;
            }
        }
        // no break

        case GAPC_LE_SMP_EVT_LOCAL_LTK_PROVIDED:
        {
            if((p_proc->keys_dist & GAP_KDIST_ENCKEY) != 0)
            {
                status = gapc_le_smp_send_enc_info_pdu(p_proc);
                if(status != GAP_ERR_NO_ERROR) break;

                status = gapc_le_smp_send_master_id_pdu(p_proc);
                if(status != GAP_ERR_NO_ERROR) break;
            }

            if((p_proc->keys_dist & GAP_KDIST_IDKEY) != 0)
            {
                // Use default identity key
                if(!gapm_is_controler_privacy_enabled())
                {
                    memcpy(&(p_proc->hdr.pairing_keys.irk.key), gapm_get_irk(), GAP_KEY_LEN);
                }
                else
                {
                    gapc_le_smp_ask_info_to_app(p_proc, GAPC_INFO_IRK, GAPC_LE_SMP_VALUE_IRK);

                    // response not received in function execution
                    if(!gapc_le_smp_is_app_answer_request(p_proc)) break;
                }
            }
        }
        // no break

        case GAPC_LE_SMP_EVT_LOCAL_IRK_PROVIDED:
        {
            if((p_proc->keys_dist & GAP_KDIST_IDKEY) != 0)
            {
                status = gapc_le_smp_send_id_info_pdu(p_proc);
                if(status != GAP_ERR_NO_ERROR) break;

                status = gapc_le_smp_send_id_addr_info_pdu(p_proc);
                if(status != GAP_ERR_NO_ERROR) break;
            }

            if((p_proc->keys_dist & GAP_KDIST_SIGNKEY) != 0)
            {
                gapc_le_smp_ask_info_to_app(p_proc, GAPC_INFO_CSRK, GAPC_LE_SMP_VALUE_CSRK);

                // response not received in function execution
                if(!gapc_le_smp_is_app_answer_request(p_proc)) break;
            }
        }
        // no break

        case GAPC_LE_SMP_EVT_LOCAL_CSRK_PROVIDED:
        {
            if((p_proc->keys_dist & GAP_KDIST_SIGNKEY) != 0)
            {
                status = gapc_le_smp_send_signing_info_pdu(p_proc);
                if(status != GAP_ERR_NO_ERROR) break;
            }

            *p_is_finished = true;
        } break;
        default: { /* ignore */ } break;
    }

    return (status);
}

/*
 * STATE MACHINE FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if (HL_LE_CENTRAL)
/// Function that handles transition of central key exchange pairing procedure
__STATIC uint16_t gapc_le_smp_key_exch_central_proc_transition(gapc_le_smp_pair_proc_t* p_proc, uint8_t event,
                                                               bool* p_is_finished)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    *p_is_finished = false;

    switch(event)
    {
        case GAPC_LE_SMP_EVT_KEY_EXCHANGE_STARTED:
        {
            // no bond data - job is finished
            if(!GETB(p_proc->pairing_lvl, GAP_PAIRING_BOND_PRESENT))
            {
                *p_is_finished = true;
                break;
            }
            // get list of key to receive
            p_proc->keys_dist = p_proc->pair_req_pdu.rkey_dist & p_proc->pair_rsp_pdu.rkey_dist;
        }
        // no break

        case GAPC_LE_SMP_EVT_PEER_LTK_RECEIVED:
        case GAPC_LE_SMP_EVT_PEER_MASTER_ID_RECEIVED:
        case GAPC_LE_SMP_EVT_PEER_IRK_RECEIVED:
        case GAPC_LE_SMP_EVT_PEER_ADDR_INFO_RECEIVED:
        case GAPC_LE_SMP_EVT_PEER_CSRK_RECEIVED:
        #if (BT_HOST_PRESENT)
        case GAPC_LE_SMP_EVT_DERIV_KEY_COMPUTED:
        #endif // (BT_HOST_PRESENT)
        {
            bool is_finished = false;
            status = gapc_le_smp_key_exch_received_transition(p_proc, event, &is_finished);
            if(!is_finished) break; // wait for transmission termination

            // get list of key to distribute
            p_proc->keys_dist = p_proc->pair_req_pdu.ikey_dist & p_proc->pair_rsp_pdu.ikey_dist;
            event = GAPC_LE_SMP_EVT_KEY_EXCHANGE_STARTED;
        }
        // no break;

        case GAPC_LE_SMP_EVT_LOCAL_LTK_PROVIDED:
        case GAPC_LE_SMP_EVT_LOCAL_IRK_PROVIDED:
        case GAPC_LE_SMP_EVT_LOCAL_CSRK_PROVIDED:
        {
            status = gapc_le_smp_key_exch_send_transition(p_proc, event, p_is_finished);
        } break;

       default: { /* ignore */ } break;
    }

    return(status);
}
#endif // (HL_LE_CENTRAL)

#if (HL_LE_PERIPHERAL)
/// Function that handles transition of peripheral key exchange pairing procedure
__STATIC uint16_t gapc_le_smp_key_exch_periph_proc_transition(gapc_le_smp_pair_proc_t* p_proc, uint8_t event,
                                                              bool* p_is_finished)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    *p_is_finished = false;

    switch(event)
    {
        case GAPC_LE_SMP_EVT_KEY_EXCHANGE_STARTED:
        {
            // no bond data - job is finished
            if(!GETB(p_proc->pairing_lvl, GAP_PAIRING_BOND_PRESENT))
            {
                *p_is_finished = true;
                break;
            }
            // get list of key to receive
            p_proc->keys_dist = p_proc->pair_req_pdu.rkey_dist & p_proc->pair_rsp_pdu.rkey_dist;
        }
        // no break

        case GAPC_LE_SMP_EVT_LOCAL_LTK_PROVIDED:
        case GAPC_LE_SMP_EVT_LOCAL_IRK_PROVIDED:
        case GAPC_LE_SMP_EVT_LOCAL_CSRK_PROVIDED:
        {
            bool is_finished = false;
            status = gapc_le_smp_key_exch_send_transition(p_proc, event, &is_finished);
            if(!is_finished) break; // wait for transmission termination

            // get list of key to distribute
            p_proc->keys_dist = p_proc->pair_req_pdu.ikey_dist & p_proc->pair_rsp_pdu.ikey_dist;
            event = GAPC_LE_SMP_EVT_KEY_EXCHANGE_STARTED;
        }
        // no break;

        case GAPC_LE_SMP_EVT_PEER_LTK_RECEIVED:
        case GAPC_LE_SMP_EVT_PEER_MASTER_ID_RECEIVED:
        case GAPC_LE_SMP_EVT_PEER_IRK_RECEIVED:
        case GAPC_LE_SMP_EVT_PEER_ADDR_INFO_RECEIVED:
        case GAPC_LE_SMP_EVT_PEER_CSRK_RECEIVED:
        #if (BT_HOST_PRESENT)
        case GAPC_LE_SMP_EVT_DERIV_KEY_COMPUTED:
        #endif // (BT_HOST_PRESENT)
        {
            status = gapc_le_smp_key_exch_received_transition(p_proc, event, p_is_finished);
        }
        // no break;

       default: { /* ignore */ } break;
    }

    return(status);
}
#endif // (HL_LE_PERIPHERAL)

/*
 * INTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if(HL_LE_CENTRAL)
uint16_t gapc_le_smp_central_start_key_exch_proc_transition(gapc_le_smp_pair_proc_t* p_proc, bool* p_is_finished)
{
    return gapc_le_smp_transition_to_new_state_machine(p_proc, gapc_le_smp_key_exch_central_proc_transition,
                                                       GAPC_LE_SMP_EVT_KEY_EXCHANGE_STARTED, p_is_finished);
}
#endif // (HL_LE_CENTRAL)

#if(HL_LE_PERIPHERAL)
uint16_t gapc_le_smp_periph_start_key_exch_proc_transition(gapc_le_smp_pair_proc_t* p_proc, bool* p_is_finished)
{
    return gapc_le_smp_transition_to_new_state_machine(p_proc, gapc_le_smp_key_exch_periph_proc_transition,
                                                       GAPC_LE_SMP_EVT_KEY_EXCHANGE_STARTED, p_is_finished);
}
#endif // (HL_LE_PERIPHERAL)


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint16_t gapc_pairing_provide_ltk(uint8_t conidx, const gapc_ltk_t* p_ltk)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapc_le_smp_pair_proc_t* p_proc;

    if(p_ltk == NULL)
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else if(gapc_le_smp_is_app_involved(conidx, GAPC_LE_SMP_VALUE_LTK, &p_proc))
    {
        p_proc->hdr.pairing_keys.ltk = *p_ltk;
        gapc_le_smp_try_proc_transition(p_proc, GAPC_LE_SMP_EVT_LOCAL_LTK_PROVIDED);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

uint16_t gapc_pairing_provide_irk(uint8_t conidx, const gap_sec_key_t* p_irk)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapc_le_smp_pair_proc_t* p_proc;

    if(p_irk == NULL)
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else if(gapc_le_smp_is_app_involved(conidx, GAPC_LE_SMP_VALUE_IRK, &p_proc))
    {
        p_proc->hdr.pairing_keys.irk.key = *p_irk;
        gapc_le_smp_try_proc_transition(p_proc, GAPC_LE_SMP_EVT_LOCAL_IRK_PROVIDED);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

uint16_t gapc_pairing_provide_csrk(uint8_t conidx, const gap_sec_key_t* p_csrk)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapc_le_smp_pair_proc_t* p_proc;

    if(p_csrk == NULL)
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else if(gapc_le_smp_is_app_involved(conidx, GAPC_LE_SMP_VALUE_CSRK, &p_proc))
    {
        gapc_le_con_t* p_con = gapc_le_con_env_get(conidx);
        p_proc->hdr.pairing_keys.csrk = *p_csrk;

        // update connection environment variables and reset sign counter
        memcpy(&(p_con->hdr.bond.csrk[GAPC_INFO_SRC_LOCAL]), p_csrk, GAP_KEY_LEN);
        p_con->hdr.bond.sign_counter[GAPC_INFO_SRC_LOCAL] = 0;

        gapc_le_smp_try_proc_transition(p_proc, GAPC_LE_SMP_EVT_LOCAL_CSRK_PROVIDED);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

#endif // (BLE_GAPC)
/// @} GAPC
