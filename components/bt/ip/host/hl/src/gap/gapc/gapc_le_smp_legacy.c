/**
 ****************************************************************************************
 *
 * @file gapc_le_smp_legacy.c
 *
 * @brief GAPC LE SMP - Legacy pairing state machines
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
#include "aes.h"


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


/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/// @brief Compute the Confirmation value (Local or remote)
__STATIC void gapc_le_smp_legacy_compute_confirm(gapc_le_smp_pair_proc_t* p_proc,
                                                 uint8_t init_addr_src, uint8_t resp_addr_src,
                                                 const uint8_t* p_rand, aes_func_result_cb res_cb)
{
    gapc_le_con_t* p_con = gapc_le_con_env_get(p_proc->hdr.conidx);
    const gap_bdaddr_t* p_init_addr = &(p_con->src[init_addr_src]);
    const gap_bdaddr_t* p_resp_addr = &(p_con->src[resp_addr_src]);

    // P1 value (LSB->MSB)
    uint8_t p1[GAP_KEY_LEN];
    // P2
    uint8_t p2[CFM_LEN];
    // Offset
    uint8_t offset = 0;

    //
    //  p1 = PRES || PREQ || 0 || RAT || 0 || IAT
    //      PRES = Pairing Response Command
    //      PREQ = Pairing Request Command
    //      RAT  = Responding device address type
    //      IAT  = Initiating Device Address Type
    //
    //  p2 = 0[0:4] || IA || RA
    //      RA  = Responding device address
    //      IA  = Initiating Device address
    //
    memset(&p1[0], 0x00, GAP_KEY_LEN);
    memset(&p2[0], 0x00, GAP_KEY_LEN);

    // Initiating Device Address Type
    // Responding Device Address Type
    p1[offset]     = p_init_addr->addr_type;
    p1[offset + 1] = p_resp_addr->addr_type;

    memcpy(&p2[0], &(p_resp_addr->addr[0]), BD_ADDR_LEN);
    memcpy(&p2[BD_ADDR_LEN], &(p_init_addr->addr[0]), BD_ADDR_LEN);

    offset += 2;

    // Pairing Request Command
    p1[offset] = L2CAP_SMP_PAIRING_REQ_OPCODE;
    offset++;

    memcpy(&p1[offset], &p_proc->pair_req_pdu, GAPC_LE_SMP_CODE_PAIRING_REQ_RESP_LEN - 1);
    offset += (GAPC_LE_SMP_CODE_PAIRING_REQ_RESP_LEN - 1);

    // Pairing Response Command
    p1[offset] = L2CAP_SMP_PAIRING_RSP_OPCODE;
    offset++;
    memcpy(&p1[offset], &p_proc->pair_rsp_pdu, GAPC_LE_SMP_CODE_PAIRING_REQ_RESP_LEN - 1);

    // execute C1 function
    aes_c1(p_proc->key, p_rand, p1, p2, res_cb, p_proc->hdr.conidx);
}


#if (HL_LE_CENTRAL)
/// Central compute local confirm
__STATIC void gapc_le_smp_legacy_central_compute_local_confirm(gapc_le_smp_pair_proc_t* p_proc)
{
    gapc_le_smp_legacy_compute_confirm(p_proc, GAPC_INFO_SRC_LOCAL, GAPC_INFO_SRC_PEER,
                                       p_proc->local_rand, gapc_le_smp_local_confirm_cmp_cb);
}

/// Central compute peer confirm
__STATIC void gapc_le_smp_legacy_central_compute_peer_confirm(gapc_le_smp_pair_proc_t* p_proc)
{
    gapc_le_smp_legacy_compute_confirm(p_proc, GAPC_INFO_SRC_LOCAL, GAPC_INFO_SRC_PEER,
                                       p_proc->peer_rand, gapc_le_smp_peer_confirm_cmp_cb);
}
#endif // (HL_LE_CENTRAL)

#if (HL_LE_PERIPHERAL)
/// Peripheral compute local confirm
__STATIC void gapc_le_smp_legacy_periph_compute_local_confirm(gapc_le_smp_pair_proc_t* p_proc)
{
    gapc_le_smp_legacy_compute_confirm(p_proc, GAPC_INFO_SRC_PEER, GAPC_INFO_SRC_LOCAL,
                                       p_proc->local_rand, gapc_le_smp_local_confirm_cmp_cb);
}

/// Peripheral compute peer confirm
__STATIC void gapc_le_smp_legacy_periph_compute_peer_confirm(gapc_le_smp_pair_proc_t* p_proc)
{
    gapc_le_smp_legacy_compute_confirm(p_proc, GAPC_INFO_SRC_PEER, GAPC_INFO_SRC_LOCAL,
                                       p_proc->peer_rand, gapc_le_smp_peer_confirm_cmp_cb);
}
#endif // (HL_LE_PERIPHERAL)

/// STK computed
__STATIC void gapc_le_smp_stk_gen_cmp_cb(uint8_t aes_status, const uint8_t* aes_res, uint32_t conidx)
{
    gapc_le_smp_pair_proc_t* p_proc = gapc_le_smp_get_pairing_proc(conidx);
    if(p_proc != NULL)
    {
        uint16_t status = GAP_ERR_UNEXPECTED;
        if(aes_status == CO_BUF_ERR_NO_ERROR)
        {
            uint8_t key_size = p_proc->key_size;

            // Copy key but crop according to key size
            memcpy(p_proc->key, aes_res, GAP_KEY_LEN);
            memset(&(p_proc->key[key_size]), 0, GAP_KEY_LEN - key_size);
            status = GAP_ERR_NO_ERROR;
        }

        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_STK_COMPUTED, status);
    }
}

/// @brief Generate the STK used to encrypt a link after the pairing procedure
__STATIC void gapc_le_smp_generate_stk(gapc_le_smp_pair_proc_t* p_proc, const uint8_t* p_init_rand,
                                       const uint8_t* p_resp_rand)
{
    /*
     * ********************************************
     * CALCULATE STK
     *      STK = AES_128(TK, r) with:
     *            r = LSB64(Srand) || LSB64(Mrand)
     * ********************************************
     */

    uint8_t r[GAP_KEY_LEN];

    memcpy(&(r[0]), p_init_rand, (RAND_VAL_LEN/2));
    memcpy(&(r[RAND_VAL_LEN/2]), p_resp_rand, (RAND_VAL_LEN/2));

    // Execute AES
    aes_encrypt(p_proc->key, r, true, gapc_le_smp_stk_gen_cmp_cb, p_proc->hdr.conidx);
}

/*
 * STATE MACHINE
 ****************************************************************************************
 */

#if (HL_LE_CENTRAL)
// Function that handles transition of central legacy pairing procedure
uint16_t gapc_le_smp_legacy_central_proc_transition(gapc_le_smp_pair_proc_t* p_proc, uint8_t event,
                                                    bool* p_is_finished)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    *p_is_finished = false;

    switch(event)
    {
        case GAPC_LE_SMP_EVT_PAIRING_AUTHENTICATION_STARTED:
        {
            // If Just Works, the TK is 0
            if (p_proc->pair_method != GAPC_LE_SMP_METH_JW)
            {
                uint8_t exp_info;

                // Both have OOB -> The length of the TK shall be 16 bytes
                if ((p_proc->pair_req_pdu.oob == GAP_OOB_AUTH_DATA_PRESENT) && (p_proc->pair_rsp_pdu.oob == GAP_OOB_AUTH_DATA_PRESENT))
                {
                    exp_info = GAPC_INFO_TK_OOB;
                }
                else
                    if (   ((p_proc->pair_rsp_pdu.iocap == GAP_IO_CAP_KB_ONLY) || (p_proc->pair_rsp_pdu.iocap == GAP_IO_CAP_KB_DISPLAY))
                        && (p_proc->pair_req_pdu.iocap != GAP_IO_CAP_KB_ONLY))
                {
                    // The application shall display the PIN Code
                    exp_info = GAPC_INFO_TK_DISPLAYED;
                }
                else
                {
                    // The use shall enter the key using the keyboard
                    exp_info = GAPC_INFO_TK_ENTERED;
                }

                // Send a TK request to the HL
                gapc_le_smp_ask_info_to_app(p_proc, exp_info, GAPC_LE_SMP_VALUE_TK);
                if(!gapc_le_smp_is_app_answer_request(p_proc)) break;
            }
        }
        // no break
        case GAPC_LE_SMP_EVT_TK_PROVIDED:
        {
            // Check if user accept or reject request
            status = gapc_le_smp_check_user_accept(p_proc, ((p_proc->pair_method == GAPC_LE_SMP_METH_PK)
                                                                    ? L2CAP_SMP_ERR_PASSKEY_ENTRY_FAILED
                                                                    : L2CAP_SMP_ERR_UNSPECIFIED_REASON));
            if(status != GAP_ERR_NO_ERROR) break;

            gapc_le_smp_generate_rand(p_proc);

        } break;
        case GAPC_LE_SMP_EVT_LOCAL_RAND_GENERATED:
        {
            // Random value has been generated => start confirm value generation
            gapc_le_smp_legacy_central_compute_local_confirm(p_proc);
        } break;

        case GAPC_LE_SMP_EVT_LOCAL_CONFIRM_COMPUTED:
        {
            status = gapc_le_smp_send_pairing_confirm_pdu(p_proc); // Send Confirm value
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_PAIRING_CONFIRM;
        } break;

        case GAPC_LE_SMP_EVT_PEER_CONFIRM_RECEIVED:
        {
            // Send the generated random value
            status = gapc_le_smp_send_pair_rand_pdu(p_proc);
            // Wait for rand reception
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_PAIRING_RAND;
        } break;

        case GAPC_LE_SMP_EVT_PEER_RANDOM_RECEIVED:
        {
            // compute peer confirm
            gapc_le_smp_legacy_central_compute_peer_confirm(p_proc);
        } break;

        case GAPC_LE_SMP_EVT_PEER_CONFIRM_COMPUTED:
        {
            // Generate STK
            gapc_le_smp_generate_stk(p_proc, p_proc->local_rand, p_proc->peer_rand);
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_ENC;
        } break;

        case GAPC_LE_SMP_EVT_STK_COMPUTED:
        {
            // Send a Start Encryption Request using the STK
            status = gapc_le_smp_send_start_enc_cmd(p_proc);
        } break;

        case GAPC_LE_SMP_EVT_HCI_ENC_CHANGED_EVT_RECEIVED:
        {
            // start key exchange
            status = gapc_le_smp_central_start_key_exch_proc_transition(p_proc, p_is_finished);
        } break;

        default: { /* ignore */ } break;
    }

    return(status);
}
#endif // (HL_LE_CENTRAL)

#if (HL_LE_PERIPHERAL)
/// Function that handles transition of peripheral legacy pairing procedure
uint16_t gapc_le_smp_legacy_periph_proc_transition(gapc_le_smp_pair_proc_t* p_proc, uint8_t event,
                                                   bool* p_is_finished)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    *p_is_finished = false;

    switch(event)
    {
        case GAPC_LE_SMP_EVT_PAIRING_AUTHENTICATION_STARTED:
        {
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_PAIRING_CONFIRM;
            if (p_proc->pair_method != GAPC_LE_SMP_METH_JW)
            {
                uint8_t exp_info;

                // Both have OOB -> The length of the TK shall be 16 bytes
                if ((p_proc->pair_req_pdu.oob == GAP_OOB_AUTH_DATA_PRESENT)
                        && (p_proc->pair_rsp_pdu.oob == GAP_OOB_AUTH_DATA_PRESENT))
                {
                    exp_info = GAPC_INFO_TK_OOB;
                }
                else if(   (p_proc->pair_rsp_pdu.iocap == GAP_IO_CAP_DISPLAY_ONLY)
                        || (p_proc->pair_rsp_pdu.iocap == GAP_IO_CAP_DISPLAY_YES_NO)
                        || ((p_proc->pair_rsp_pdu.iocap == GAP_IO_CAP_KB_DISPLAY)
                                && (p_proc->pair_req_pdu.iocap == GAP_IO_CAP_KB_ONLY) ))
                {
                    // The application shall display the PIN Code
                    exp_info = GAPC_INFO_TK_DISPLAYED;
                }
                else
                {
                    // The use shall enter the key using the keyboard
                    exp_info = GAPC_INFO_TK_ENTERED;
                }

                // Send a TK request to the HL
                gapc_le_smp_ask_info_to_app(p_proc, exp_info, GAPC_LE_SMP_VALUE_TK);
                if(!gapc_le_smp_is_app_answer_request(p_proc)) break;
            }
        }
        // no break;
        case GAPC_LE_SMP_EVT_TK_PROVIDED:
        {
            // Check if user accept or reject request
            status = gapc_le_smp_check_user_accept(p_proc, ((p_proc->pair_method == GAPC_LE_SMP_METH_PK)
                                                                    ? L2CAP_SMP_ERR_PASSKEY_ENTRY_FAILED
                                                                    : L2CAP_SMP_ERR_UNSPECIFIED_REASON));
            if(status != GAP_ERR_NO_ERROR) break;

            // Check if central confirm has been received
            if(p_proc->peer_exp_message == GAPC_LE_SMP_W4_RX_PAIRING_CONFIRM) break;
        }
        // no break;
        case GAPC_LE_SMP_EVT_PEER_CONFIRM_RECEIVED:
        {
            if(!gapc_le_smp_is_app_answer_request(p_proc)) break;

            // Generate random number
            gapc_le_smp_generate_rand(p_proc);
        } break;

        case GAPC_LE_SMP_EVT_LOCAL_RAND_GENERATED:
        {
            // Random value has been generated => start confirm value generation
            gapc_le_smp_legacy_periph_compute_local_confirm(p_proc);
        } break;

        case GAPC_LE_SMP_EVT_LOCAL_CONFIRM_COMPUTED:
        {
            status = gapc_le_smp_send_pairing_confirm_pdu(p_proc); // Send Confirm value
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_PAIRING_RAND;
        } break;

        case GAPC_LE_SMP_EVT_PEER_RANDOM_RECEIVED:
        {
            // Start the check confirm value procedure
            gapc_le_smp_legacy_periph_compute_peer_confirm(p_proc);
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_PAIRING_CONFIRM;
        } break;

        case GAPC_LE_SMP_EVT_PEER_CONFIRM_COMPUTED:
        {
            status = gapc_le_smp_send_pair_rand_pdu(p_proc);
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_ENC;
        } break;

        case GAPC_LE_SMP_EVT_HCI_LTK_REQ_RECEIVED:
        {
            gapc_le_smp_generate_stk(p_proc, p_proc->peer_rand, p_proc->local_rand);
        } break;

        case GAPC_LE_SMP_EVT_STK_COMPUTED:
        {
            // Send a Start Encryption Request using the STK
            status = gapc_le_smp_send_ltk_req_rsp(p_proc);
        } break;

        case GAPC_LE_SMP_EVT_HCI_ENC_CHANGED_EVT_RECEIVED:
        {
            // start key exchange
            status = gapc_le_smp_periph_start_key_exch_proc_transition(p_proc, p_is_finished);
        } break;

        default: { /* ignore */ } break;
    }

    return(status);
}
#endif // (HL_LE_PERIPHERAL)


/*
 * INTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
#if (HL_LE_CENTRAL)
/// Central start legacy pairing
uint16_t gapc_le_smp_legacy_central_proc_start(gapc_le_smp_pair_proc_t* p_proc, bool* p_is_finished)
{
    return gapc_le_smp_transition_to_new_state_machine(p_proc, gapc_le_smp_legacy_central_proc_transition,
                                                       GAPC_LE_SMP_EVT_PAIRING_AUTHENTICATION_STARTED, p_is_finished);
}
#endif // (HL_LE_CENTRAL)

#if (HL_LE_PERIPHERAL)
/// Peripheral start legacy pairing
uint16_t gapc_le_smp_legacy_periph_proc_start(gapc_le_smp_pair_proc_t* p_proc,  bool* p_is_finished)
{
    return gapc_le_smp_transition_to_new_state_machine(p_proc, gapc_le_smp_legacy_periph_proc_transition,
                                                       GAPC_LE_SMP_EVT_PAIRING_AUTHENTICATION_STARTED, p_is_finished);
}
#endif // (HL_LE_PERIPHERAL)

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint16_t gapc_pairing_provide_tk(uint8_t conidx, bool accept, const gap_sec_key_t* p_tk)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapc_le_smp_pair_proc_t* p_proc;

    if(accept && (p_tk == NULL))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else if(gapc_le_smp_is_app_involved(conidx, GAPC_LE_SMP_VALUE_TK, &p_proc))
    {
        SETB(p_proc->info_bf, GAPC_LE_SMP_USER_ACCEPT, accept != false);
        if(accept)
        {
            // Store TK
            memcpy(p_proc->key, p_tk, GAP_KEY_LEN);
        }
        gapc_le_smp_try_proc_transition(p_proc, GAPC_LE_SMP_EVT_TK_PROVIDED);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}
#endif // (BLE_GAPC)

/// @} GAPC
