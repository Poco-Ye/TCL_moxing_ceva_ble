/**
 ****************************************************************************************
 *
 * @file gapc_le_smp_sc.c
 *
 * @brief GAPC LE SMP - Secure Connection pairing state machines
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
#include "gapm_le.h"
#include "aes.h"

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */
#define BLE_LE_SMP_SC_PASSKEY_NUM_BIT   (19)

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

#if (HL_LE_CENTRAL)
__STATIC uint16_t gapc_le_smp_sc_central_proc_transition(gapc_le_smp_pair_proc_t* p_proc, uint8_t event,
                                                         bool* p_is_finished);
#endif // (HL_LE_CENTRAL)

#if (HL_LE_PERIPHERAL)
__STATIC uint16_t gapc_le_smp_sc_periph_proc_transition(gapc_le_smp_pair_proc_t* p_proc, uint8_t event,
                                                        bool* p_is_finished);
#endif // (HL_LE_PERIPHERAL)

/// Get if passkey must be displayed or entered
__STATIC uint8_t gapc_le_smp_sc_pk_get_app_expected_value(gapc_le_smp_pair_proc_t *p_proc, uint8_t local_iocap,
                                                          uint8_t peer_iocap)
{
    uint8_t exp_value = GAPC_INFO_PASSKEY_ENTERED;

    if (   (local_iocap == GAP_IO_CAP_DISPLAY_ONLY) || (local_iocap == GAP_IO_CAP_DISPLAY_YES_NO)
        || ((local_iocap == GAP_IO_CAP_KB_DISPLAY) && (peer_iocap == GAP_IO_CAP_KB_ONLY)))
    {
        // The application shall display the PIN Code
        exp_value = GAPC_INFO_PASSKEY_DISPLAYED;
    }

   return (exp_value);
}

/// Ask a numeric comparison
__STATIC void gapc_le_smp_ask_numeric_comp_to_app(gapc_le_smp_pair_proc_t* p_proc)
{
    SETB(p_proc->info_bf, GAPC_LE_SMP_IN_PAIRING_PROC, true);
    p_proc->app_exp_value = GAPC_LE_SMP_VALUE_NUMERIC_COMPARE;
    gapc_env.p_sec_cbs->numeric_compare_req(p_proc->hdr.conidx, p_proc->hdr.dummy, p_proc->passkey);
    SETB(p_proc->info_bf, GAPC_LE_SMP_IN_PAIRING_PROC, false);
}

/*
 * L2CAP PDU HANDLERS FUNCTIONS
 ****************************************************************************************
 */

/// @brief Callback used to handle PAIRING PUBLIC KEYS PDU
void l2cap_smp_public_key_handler(gapc_le_con_t* p_con, l2cap_smp_public_key_t* p_pdu, co_buf_t* p_buf)
{
    uint8_t smp_status = L2CAP_SMP_ERR_UNSPECIFIED_REASON;
    gapc_le_smp_pair_proc_t* p_proc = NULL;

    if(gapc_le_smp_is_peer_message_expected(p_con, GAPC_LE_SMP_W4_RX_PUBLIC_KEY, &p_proc))
    {
        // Prevent passkey entry vulnerability, ensure that X coordinate of public keys are not equals
        if(memcmp(p_proc->local_public_key.x, p_pdu->x, GAP_P256_KEY_LEN) == 0)
        {
            smp_status = L2CAP_SMP_ERR_DHKEY_CHECK_FAILED;
        }
        else
        {
            // Store  Public Key X and Y co-ordinates.
            memcpy(&(p_proc->peer_public_key.x[0]), &(p_pdu->x[0]), GAP_P256_KEY_LEN);
            memcpy(&(p_proc->peer_public_key.y[0]), &(p_pdu->y[0]), GAP_P256_KEY_LEN);
            smp_status = L2CAP_SMP_ERR_NO_ERROR;
        }
    }

    if(p_proc != NULL)
    {
        uint16_t status = GAP_ERR_NO_ERROR;
        if(smp_status != L2CAP_SMP_ERR_NO_ERROR)
        {
            gapc_le_smp_send_pairing_fail_pdu_and_start_ra_timer(p_proc, smp_status);
            status = GAPC_LE_SMP_ERR_TO_HL_ERR(LOC, smp_status);
        }

        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_PEER_PUB_KEY_RECEIVED, status);
    }
}

/// @brief Callback used to handle PAIRING DHKEY CHECK PDU
void l2cap_smp_dhkey_check_handler(gapc_le_con_t* p_con, l2cap_smp_dhkey_check_t* p_pdu, co_buf_t* p_buf)
{
    gapc_le_smp_pair_proc_t* p_proc;
    if(gapc_le_smp_is_peer_message_expected(p_con, GAPC_LE_SMP_W4_RX_DHKEY_CHECK, &p_proc))
    {
        // Save the Peers DHkey Check.
        memcpy(p_proc->peer_confirm, &(p_pdu->check[0]), DHKEY_CHECK_LEN);
        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_PEER_DH_KEY_CHECK_RECEIVED, GAP_ERR_NO_ERROR);

    } // else ignore if not expected
}

/// @brief Callback used to handle PAIRING KEY-PRESS NOTIFICATION PDU
void l2cap_smp_keypress_ntf_handler(gapc_le_con_t* p_con, l2cap_smp_keypress_ntf_t* p_pdu, co_buf_t* p_buf)
{
    gapc_le_smp_pair_proc_t* p_proc = gapc_le_smp_get_pairing_proc(p_con->hdr.conidx);
    // Check if we are waiting the TK (Pairing procedure on going)
    if ((p_proc != NULL) && (p_proc->pair_method == GAPC_LE_SMP_METH_PK))
    {
        uint8_t smp_status = L2CAP_SMP_ERR_NO_ERROR;
        // check value
        if(p_pdu->ntf_type > GAPC_KEY_NTF_ENTRY_COMPLETED)
        {
            smp_status = L2CAP_SMP_ERR_INVALID_PARAM;

            gapc_le_smp_send_pairing_fail_pdu_and_start_ra_timer(p_proc, L2CAP_SMP_ERR_INVALID_PARAM);
            gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_PAIRING_ABORTED,
                                        GAPC_LE_SMP_ERR_TO_HL_ERR(LOC, smp_status));
        }
        else
        {
            // Restart transaction timer
            gapc_le_smp_start_trans_timer(&(p_proc->hdr));

            if(gapc_env.p_sec_cbs->key_pressed != NULL)
            {
                gapc_env.p_sec_cbs->key_pressed(p_proc->hdr.conidx, p_proc->hdr.dummy, p_pdu->ntf_type);
            }
        }
    }
}

/*
 * AES FUNCTIONS
 ****************************************************************************************
 */

/// Compute peer confirm for OOB pairing
__STATIC void gapc_le_smp_oob_compute_peer_confirm(gapc_le_smp_pair_proc_t* p_proc)
{
    aes_f4(p_proc->peer_public_key.x, p_proc->peer_public_key.x, p_proc->peer_r, 0x00,
           gapc_le_smp_peer_confirm_cmp_cb, p_proc->hdr.conidx);
}

/// Compute peer confirm pairing
__STATIC void gapc_le_smp_compute_local_confirm(gapc_le_smp_pair_proc_t* p_proc, uint8_t ri)
{
    aes_f4(p_proc->local_public_key.x, p_proc->peer_public_key.x, p_proc->local_rand, ri,
           gapc_le_smp_local_confirm_cmp_cb, p_proc->hdr.conidx);
}

/// Compute peer confirm pairing
__STATIC void gapc_le_smp_compute_peer_confirm(gapc_le_smp_pair_proc_t* p_proc, uint8_t ri)
{
    aes_f4(p_proc->peer_public_key.x, p_proc->local_public_key.x, p_proc->peer_rand, ri,
           gapc_le_smp_peer_confirm_cmp_cb, p_proc->hdr.conidx);
}

/// MAC key value computed
__STATIC void gapc_le_smp_compute_mac_key_cmp_cb(uint8_t aes_status, const uint8_t* aes_res, uint32_t conidx)
{
    gapc_le_smp_pair_proc_t* p_proc = gapc_le_smp_get_pairing_proc(conidx);
    if(p_proc != NULL)
    {
        uint16_t status = GAP_ERR_UNEXPECTED;
        if(aes_status == CO_BUF_ERR_NO_ERROR)
        {
            memcpy(p_proc->mac_key,  &(aes_res[0]), GAP_KEY_LEN);
            memcpy(p_proc->key, &(aes_res[GAP_KEY_LEN]), GAP_KEY_LEN);
            status = GAP_ERR_NO_ERROR;
        }

        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_MAC_KEY_COMPUTED, status);
    }
}

/// @brief Initiates the f5 algorithm to calculate the MacKey and LTK for a link
__STATIC void gapc_le_smp_compute_mac_key(gapc_le_smp_pair_proc_t* p_proc, const uint8_t* p_dh_key,
                                          uint8_t addr_src_init, uint8_t addr_src_resp,
                                          const uint8_t* p_init_rand, const uint8_t* p_resp_rand)
{
    gapc_le_con_t* p_con = gapc_le_con_env_get(p_proc->hdr.conidx);
    const gap_bdaddr_t* p_init_addr = &(p_con->src[addr_src_init]);
    const gap_bdaddr_t* p_resp_addr = &(p_con->src[addr_src_resp]);

    // Begin f5
    // MacKey || LTK = f5(DHKey, N_master, N_slave, BD_ADDR_master, BD_ADDR_slave)
    uint8_t  addr_master[7];
    uint8_t  addr_slave[7];

    memcpy(&addr_master[0], &p_init_addr->addr[0], 6);
    addr_slave[6]  = p_resp_addr->addr_type;
    memcpy(&addr_slave[0],  &p_resp_addr->addr[0], 6);
    addr_master[6] = p_init_addr->addr_type;

    aes_f5(p_dh_key, p_init_rand, p_resp_rand, addr_master, addr_slave,
           gapc_le_smp_compute_mac_key_cmp_cb, p_proc->hdr.conidx);
}

/// Local DH-Key Check value computed
__STATIC void gapc_le_smp_compute_local_dhkey_check_cmp_cb(uint8_t aes_status, const uint8_t* aes_res, uint32_t conidx)
{
    gapc_le_smp_pair_proc_t* p_proc = gapc_le_smp_get_pairing_proc(conidx);
    if(p_proc != NULL)
    {
        uint16_t status = GAP_ERR_UNEXPECTED;
        if(aes_status == CO_BUF_ERR_NO_ERROR)
        {
            // Store local DHKeyCheck - will need it later
            memcpy(p_proc->local_confirm, aes_res, GAP_KEY_LEN);

            status = GAP_ERR_NO_ERROR;
        }

        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_LOCAL_DHKEY_CHECK_COMPUTED, status);
    }
}

/// Peer DH-Key Check value computed
__STATIC void gapc_le_smp_compute_peer_dhkey_check_cmp_cb(uint8_t aes_status, const uint8_t* aes_res, uint32_t conidx)
{
    gapc_le_smp_pair_proc_t* p_proc = gapc_le_smp_get_pairing_proc(conidx);
    if(p_proc != NULL)
    {
        uint16_t status = GAP_ERR_UNEXPECTED;
        if(aes_status == CO_BUF_ERR_NO_ERROR)
        {
            // check computed DH-KEY with received one
            status = !memcmp(aes_res, p_proc->peer_confirm, GAP_KEY_LEN)
                   ? GAP_ERR_NO_ERROR : GAPC_LE_SMP_ERR_TO_HL_ERR(LOC, L2CAP_SMP_ERR_DHKEY_CHECK_FAILED);
        }

        if(status != GAP_ERR_NO_ERROR)
        {
            // Inform peer device about pairing failed and start RA timer
            gapc_le_smp_send_pairing_fail_pdu_and_start_ra_timer(p_proc, L2CAP_SMP_ERR_DHKEY_CHECK_FAILED);
        }

        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_PEER_DHKEY_CHECK_COMPUTED, status);
    }
}

/**
 ******************************************************
 *  The DHkey Check is performed using f6.
 *  Kept as separate function as the inputs to F6 are dependent on the Association model used for the pairing.
 *
 *  --------------------------------------------
 *  |  Numeric Comparison/Just Works
 *  --------------------------------------------
 *  |    Ea = f6(MacKey, Na, Nb, 0,IOcapA, A, B)
 *  |    Eb = f6(MacKey, Nb, Na, 0,IOcapB, B, A)
 *  |
 *  |    E = f6(MaxKey,Nlocal,Npeer,IOcap_local,Addr_local,Addr_Peer)
 *  |===========================================
 *  |  Out Of Band
 *  --------------------------------------------
 *  |    Ea = f6(MacKey, Na, Nb, rb,IOcapA, A, B)
 *  |    Eb = f6(MacKey, Nb, Na, ra,IOcapB, B, A)
 *  |===========================================
 *  |  PassKey
 *  --------------------------------------------
 *  |    Ea = f6(MacKey, Na20, Nb20, rb,IOcapA, A, B)
 *  |    Eb = f6(MacKey, Nb20, Na20, ra,IOcapB, B, A)
 *  |===========================================
 ******************************************************
 */
__STATIC void gapc_le_smp_compute_dhkey_check(gapc_le_smp_pair_proc_t* p_proc, const uint8_t* p_mac_key,
                                              uint8_t addr_src_a, uint8_t addr_src_b,
                                              uint8_t* p_n_a, uint8_t* p_n_b, const uint8_t* p_r,
                                              const gapc_pairing_t* p_pair_info, aes_func_result_cb res_cb)
{
    gapc_le_con_t* p_con = gapc_le_con_env_get(p_proc->hdr.conidx);
    const gap_bdaddr_t* p_addr_a = &(p_con->src[addr_src_a]);
    const gap_bdaddr_t* p_addr_b = &(p_con->src[addr_src_b]);
    uint8_t  iocap[3];
    uint8_t  addr_a[7];
    uint8_t  addr_b[7];

    // Construct the Address for local and peer.
    memcpy(&addr_a[0], p_addr_a->addr, 6);
    memcpy(&addr_b[0], p_addr_b->addr,  6);

    addr_a[6] = p_addr_a->addr_type;
    addr_b[6] = p_addr_b->addr_type;

    /* IOcap is  three octets with the most significant octet as the AuthReq parameter,
     * the middle octet as the OOB data flag and the least significant octet as the IO
     * capability parameter.
     */
    iocap[2] = p_pair_info->auth;
    iocap[1] = p_pair_info->oob;
    iocap[0] = p_pair_info->iocap;

    aes_f6(p_mac_key, p_n_a, p_n_b, p_r, iocap, addr_a, addr_b,
           res_cb, p_proc->hdr.conidx);
}


/// Compute local DH-Key Check value
__STATIC void gapc_le_smp_compute_local_dhkey_check(gapc_le_smp_pair_proc_t* p_proc,
                                                    const gapc_pairing_t* p_local_pair_info)
{
    gapc_le_smp_compute_dhkey_check(p_proc, p_proc->mac_key,
                                    GAPC_INFO_SRC_LOCAL, GAPC_INFO_SRC_PEER,
                                    p_proc->local_rand, p_proc->peer_rand, p_proc->peer_r,
                                    p_local_pair_info, gapc_le_smp_compute_local_dhkey_check_cmp_cb);
}

/// Compute peer DH-Key Check value
__STATIC void gapc_le_smp_compute_peer_dhkey_check(gapc_le_smp_pair_proc_t* p_proc,
                                                    const gapc_pairing_t* p_peer_pair_info)
{
    gapc_le_smp_compute_dhkey_check(p_proc, p_proc->mac_key,
                                    GAPC_INFO_SRC_PEER, GAPC_INFO_SRC_LOCAL,
                                    p_proc->peer_rand, p_proc->local_rand, p_proc->local_r,
                                    p_peer_pair_info, gapc_le_smp_compute_peer_dhkey_check_cmp_cb);
}



/// Numeric value computed
__STATIC void gapc_le_smp_compute_numeric_value_cmp_cb(uint8_t aes_status, const uint8_t* aes_res, uint32_t conidx)
{
    gapc_le_smp_pair_proc_t* p_proc = gapc_le_smp_get_pairing_proc(conidx);
    if(p_proc != NULL)
    {
        uint16_t status = GAP_ERR_UNEXPECTED;
        if(aes_status == CO_BUF_ERR_NO_ERROR)
        {
            // If AES_CMAC complete then G2 is complete
            // we need to determine the number for comparison - Va.
            // Write the First 4 bytes of AES Result to TK.
            // Note ! we use the uint32_t for passkey to store the Numeric Value
            p_proc->passkey = (co_read32p(aes_res) % 0xF4240);

            status = GAP_ERR_NO_ERROR;
        }

        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_NUMERIC_VALUE_COMPUTED, status);
    }
}

/// @brief Compute LE Secure Connections Numeric Comparison Value Generation Function g2
__STATIC void gapc_le_smp_compute_numeric_value(gapc_le_smp_pair_proc_t* p_proc,
                                                const uint8_t* p_init_pub_x, const uint8_t* p_resp_pub_x,
                                                const uint8_t* p_init_rand, const uint8_t* p_resp_rand)
{
    // Determine Va = g2(Pka,Pkb,Na,Nb)
    aes_g2(p_init_pub_x, p_resp_pub_x, p_init_rand, p_resp_rand,
           gapc_le_smp_compute_numeric_value_cmp_cb, p_proc->hdr.conidx);
}


/// DH-Key computation result
__STATIC void gapc_le_smp_compute_dh_key_cmp_cb(uint32_t conidx, uint16_t status, const gap_dh_key_t* p_dh_key)
{
    gapc_le_smp_pair_proc_t* p_proc = gapc_le_smp_get_pairing_proc(conidx);
    if(p_proc != NULL)
    {
        if(status == GAP_ERR_NO_ERROR)
        {
            // mark local DHKey calculated
            SETB(p_proc->info_bf, GAPC_LE_SMP_LOC_DHKEY_COMP, true);
            memcpy(&(p_proc->dh_key[0]), p_dh_key, DH_KEY_LEN);
        }
        else
        {
            status = GAPC_LE_SMP_ERR_TO_HL_ERR(LOC, L2CAP_SMP_ERR_DHKEY_CHECK_FAILED);
            // Start RA timer + send pair failed pdu
        }

        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_DH_KEY_COMPUTED, status);
    }
}


/// Public Key computed
__STATIC void gapc_compute_public_key_cmp_cb(uint32_t conidx, uint16_t status, const public_key_t* p_pub_key)
{
    gapc_le_smp_pair_proc_t* p_proc = gapc_le_smp_get_pairing_proc(conidx);
    if(p_proc != NULL)
    {
        if(status == GAP_ERR_NO_ERROR)
        {
            // Mark that local public key for pairing is available
            SETB(p_proc->info_bf, GAPC_LE_SMP_LOC_PUB_KEY_GEN, true);
            // Stored The local Public Key X and Y co-ordinates.
            p_proc->local_public_key = *p_pub_key;
        }
        else
        {
            status = GAPC_LE_SMP_ERR_TO_HL_ERR(LOC, L2CAP_SMP_ERR_UNSPECIFIED_REASON);
            // Start RA timer + send pair failed pdu
        }

        gapc_le_smp_proc_transition(p_proc, GAPC_LE_SMP_EVT_PUBLIC_KEY_COMPUTED, status);
    }
}

/// @brief Get local public key by reading it from controller, or from OOB data generated
__STATIC uint16_t gapc_le_smp_get_local_pub_key(gapc_le_smp_pair_proc_t* p_proc)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    const gapm_oob_data_t* p_oob_data = gapm_oob_data_get();

    // for OOB pairing, check if OOB data already requested by application to reuse public key for following pairing
    if((p_proc->pair_method == GAPC_LE_SMP_METH_OOB) && (p_oob_data != NULL))
    {
        memcpy(&(p_proc->local_public_key), &(p_oob_data->pub_key), sizeof(public_key_t));
        SETB(p_proc->info_bf, GAPC_LE_SMP_LOC_PUB_KEY_GEN, true);
    }
    // Ask for public key generation
    else
    {
        status = gapm_get_public_key(p_proc->hdr.conidx, gapc_compute_public_key_cmp_cb);
    }

    return (status);
}

/// Load local OOB data
__STATIC uint16_t gapc_le_smp_load_local_oob_data(gapc_le_smp_pair_proc_t* p_proc, uint8_t rem_oob)
{
    uint16_t status = GAP_ERR_NO_ERROR;

    // Remote device received OOB data
    if (rem_oob == GAP_OOB_AUTH_DATA_PRESENT)
    {
        const gapm_oob_data_t* p_oob = gapm_oob_data_get();
        if(p_oob != NULL)
        {
            // Copy local random value and confirmation value
            memcpy(&p_proc->local_r,          p_oob->data.rand, GAP_KEY_LEN);
            memcpy(&p_proc->local_confirm, p_oob->data.conf, GAP_KEY_LEN);
        }
        else
        {
            // Stop pairing, since local device never provide OOB data
            gapc_le_smp_send_pairing_fail_pdu_and_start_ra_timer(p_proc, L2CAP_SMP_ERR_OOB_NOT_AVAILABLE);
            status = GAPC_LE_SMP_ERR_TO_HL_ERR(LOC, L2CAP_SMP_ERR_OOB_NOT_AVAILABLE);
        }
    } // by default local commitment = 0

    return (status);
}

/// Get peer OOB data
__STATIC void gapc_le_smp_load_peer_oob_data(gapc_le_smp_pair_proc_t* p_proc, uint8_t loc_oob)
{
    if (loc_oob == GAP_OOB_AUTH_DATA_PRESENT)
    {
        gapc_le_smp_ask_info_to_app(p_proc, GAPC_INFO_OOB, GAPC_LE_SMP_VALUE_OOB);
    }
    // else continue pairing procedure
    else
    {
        // Set the remote commitment = 0
        memset(&p_proc->peer_r[0],       0, 16);
        memset(&p_proc->peer_confirm, 0, 16);

        // continue pairing
        SETB(p_proc->info_bf, GAPC_LE_SMP_OOB_CFM_COMP, true);
    }
}


#if (HL_LE_CENTRAL)
/// Central computes MAC Key
__STATIC void gapc_le_smp_central_compute_mac_key(gapc_le_smp_pair_proc_t* p_proc)
{
    gapc_le_smp_compute_mac_key(p_proc, p_proc->dh_key, GAPC_INFO_SRC_LOCAL,
                                GAPC_INFO_SRC_PEER, p_proc->local_rand, p_proc->peer_rand);
}
#endif // (HL_LE_CENTRAL)


#if (HL_LE_PERIPHERAL)
/// Peripheral computes MAC Key
__STATIC void gapc_le_smp_periph_compute_mac_key(gapc_le_smp_pair_proc_t* p_proc)
{
    gapc_le_smp_compute_mac_key(p_proc, p_proc->dh_key, GAPC_INFO_SRC_PEER,
            GAPC_INFO_SRC_LOCAL, p_proc->peer_rand, p_proc->local_rand);
}
#endif // (HL_LE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Determines the next bit of the passkey to be used
 *
 * @param[in] conidx        Connection Index
 *
 ****************************************************************************************
 */
__STATIC uint8_t gapc_le_smp_get_next_passkey_bit(gapc_le_smp_pair_proc_t *p_proc)
{
    uint32_t passkey = p_proc->passkey;
    uint8_t  bit_num = p_proc->passkey_bit_cursor;

    return (((passkey >> bit_num) & 0x01) | 0x80);
}

/*
 * STATE MACHINE
 ****************************************************************************************
 */
#if(HL_LE_CENTRAL)
/// Start the DH-Key Check Procedure
__STATIC  uint16_t gapc_le_smp_sc_central_start_dh_key_check(gapc_le_smp_pair_proc_t* p_proc,
                                                             bool* p_is_finished)
{
    return gapc_le_smp_transition_to_new_state_machine(p_proc, gapc_le_smp_sc_central_proc_transition,
                                                       GAPC_LE_SMP_EVT_DH_KEY_CHECK_STARTED, p_is_finished);
}

// Function that handles transition of central secure connection OOB pairing procedure
__STATIC uint16_t gapc_le_smp_sc_oob_central_proc_transition(gapc_le_smp_pair_proc_t* p_proc, uint8_t event,
                                                            bool* p_is_finished)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    switch(event)
    {
        case GAPC_LE_SMP_EVT_PAIRING_STAGE_1_STARTED:
        {
            // Start OOB exchange request to application
            status = gapc_le_smp_load_local_oob_data(p_proc, p_proc->pair_rsp_pdu.oob);
            if(status != GAP_ERR_NO_ERROR) break;
            gapc_le_smp_load_peer_oob_data(p_proc, p_proc->pair_req_pdu.oob);
            if(!gapc_le_smp_is_app_answer_request(p_proc)) break;
        }
        // no break;

        case GAPC_LE_SMP_EVT_OOB_DATA_PROVIDED:
        {
            if(!GETB(p_proc->info_bf, GAPC_LE_SMP_OOB_CFM_COMP))
            {
                // Device reject OOB
                status = gapc_le_smp_check_user_accept(p_proc, L2CAP_SMP_ERR_OOB_NOT_AVAILABLE);
                if(status != GAP_ERR_NO_ERROR) break;

                //  initiate the Verification of the peers confirm.
                // Ca = (PKa, PKa, ra, 0) if master
                gapc_le_smp_oob_compute_peer_confirm(p_proc);
                break;
            }
        }
        // no break;
        case GAPC_LE_SMP_EVT_PEER_CONFIRM_COMPUTED:
        {
            // Generate a local Random number
            gapc_le_smp_generate_rand(p_proc);
        } break;

        case GAPC_LE_SMP_EVT_LOCAL_RAND_GENERATED:
        {
            // Send local Rand to peer.
            status = gapc_le_smp_send_pair_rand_pdu(p_proc);
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_PAIRING_RAND;
        } break;

        case GAPC_LE_SMP_EVT_PEER_RANDOM_RECEIVED:
        {
            status = gapc_le_smp_sc_central_start_dh_key_check(p_proc, p_is_finished);
        } break;

        default: { /* ignore */ } break;
    }
    return (status);
}

// Function that handles transition of central secure connection numeric comparison or Just work pairing procedure
__STATIC uint16_t gapc_le_smp_sc_nc_jw_central_proc_transition(gapc_le_smp_pair_proc_t* p_proc, uint8_t event,
                                                              bool* p_is_finished)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    switch(event)
    {
        case GAPC_LE_SMP_EVT_PAIRING_STAGE_1_STARTED:
        {
            // In the master we wait until we Rx the slave confirm - Cb, before we generate the local loca_rand.
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_PAIRING_CONFIRM;
        } break;

        case GAPC_LE_SMP_EVT_PEER_CONFIRM_RECEIVED:
        {
            // Start the generation of a Random number Nb
            gapc_le_smp_generate_rand(p_proc);
        } break;

        case GAPC_LE_SMP_EVT_LOCAL_RAND_GENERATED:
        {
            // Send locally generated Random number to peer.
            status = gapc_le_smp_send_pair_rand_pdu(p_proc);
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_PAIRING_RAND;
        } break;

        case GAPC_LE_SMP_EVT_PEER_RANDOM_RECEIVED:
        {
            // Check the commitment
            // Begin the Local Commitment check
            // Cb = f4(Pkb,Pka,Nb,0)
            gapc_le_smp_compute_peer_confirm(p_proc, 0);
        } break;

        case GAPC_LE_SMP_EVT_PEER_CONFIRM_COMPUTED:
        {
            if (p_proc->pair_method == GAPC_LE_SMP_METH_NC)
            {
                gapc_le_smp_compute_numeric_value(p_proc, p_proc->local_public_key.x, p_proc->peer_public_key.x,
                                                  p_proc->local_rand, p_proc->peer_rand);
            }
            else // if (p_proc->pair_method == GAPC_LE_SMP_METH_JW)
            {
                status = gapc_le_smp_sc_central_start_dh_key_check(p_proc, p_is_finished);
            }
        } break;

        case GAPC_LE_SMP_EVT_NUMERIC_VALUE_COMPUTED:
        {
            // Ask application to accept numeric comparison
            gapc_le_smp_ask_numeric_comp_to_app(p_proc);
            if(!gapc_le_smp_is_app_answer_request(p_proc)) break;
        }
        // no break

        case GAPC_LE_SMP_EVT_NUMERIC_COMPARE_RSP_PROVIDED:
        {
            // check if application accepts
            status = gapc_le_smp_check_user_accept(p_proc, L2CAP_SMP_ERR_NUMERIC_COMPARISON_FAILED);
            if(status != GAP_ERR_NO_ERROR) break;

            // prepare for DH-Key Check computation
            memset(p_proc->local_r, 0, GAP_KEY_LEN);
            memset(p_proc->peer_r,  0, GAP_KEY_LEN);

            // stark Authentication phase 2
            status = gapc_le_smp_sc_central_start_dh_key_check(p_proc, p_is_finished);
        } break;

        default: { /* ignore */ } break;
    }
    return (status);
}

// Function that handles transition of central secure connection Passkey entry pairing procedure
__STATIC uint16_t gapc_le_smp_sc_pk_central_proc_transition(gapc_le_smp_pair_proc_t* p_proc, uint8_t event,
                                                              bool* p_is_finished)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    switch(event)
    {
        case GAPC_LE_SMP_EVT_PAIRING_STAGE_1_STARTED:
        {
            // Ask passkey value
            uint8_t exp_value = gapc_le_smp_sc_pk_get_app_expected_value(p_proc, p_proc->pair_req_pdu.iocap,
                                                                         p_proc->pair_rsp_pdu.iocap);
            gapc_le_smp_ask_info_to_app(p_proc, exp_value, GAPC_LE_SMP_VALUE_PASSKEY);
            if(!gapc_le_smp_is_app_answer_request(p_proc)) break;
        }
        // no break;

        case GAPC_LE_SMP_EVT_PASSKEY_PROVIDED:
        {
            // Check if user accept
            status = gapc_le_smp_check_user_accept(p_proc, L2CAP_SMP_ERR_PASSKEY_ENTRY_FAILED);
            if(status != GAP_ERR_NO_ERROR) break;

            // prepare for DH-Key Check computation
            memset(p_proc->local_r, 0, GAP_KEY_LEN);
            co_write32p(p_proc->local_r, p_proc->passkey);
            memcpy(p_proc->peer_r, p_proc->local_r, GAP_KEY_LEN);

            p_proc->passkey_bit_cursor = 0; // First Round
            gapc_le_smp_generate_rand(p_proc);
        } break;

        case GAPC_LE_SMP_EVT_LOCAL_RAND_GENERATED:
        {
            // Begin the Local Commitment determination -
            // if master:  Cb = f4(Pka,Pkb,Nai,rai)
            gapc_le_smp_compute_local_confirm(p_proc, gapc_le_smp_get_next_passkey_bit(p_proc));
        } break;

        case GAPC_LE_SMP_EVT_LOCAL_CONFIRM_COMPUTED:
        {
            status = gapc_le_smp_send_pairing_confirm_pdu(p_proc);
            // Now Wait for the Random Na from the master
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_PAIRING_CONFIRM;
        } break;

        case GAPC_LE_SMP_EVT_PEER_CONFIRM_RECEIVED:
        {
            // Send locally generated Random number to peer.
            status = gapc_le_smp_send_pair_rand_pdu(p_proc);
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_PAIRING_RAND;
        } break;

        case GAPC_LE_SMP_EVT_PEER_RANDOM_RECEIVED:
        {
            // If Master Cbi = f4(Pkb,Pka,Nai,rai)
            gapc_le_smp_compute_peer_confirm(p_proc, gapc_le_smp_get_next_passkey_bit(p_proc));
        } break;

        case GAPC_LE_SMP_EVT_PEER_CONFIRM_COMPUTED:
        {
            // Check the PassKey bit counter to see if this is the last round
            if (p_proc->passkey_bit_cursor < BLE_LE_SMP_SC_PASSKEY_NUM_BIT)
            {
                p_proc->passkey_bit_cursor++;
                gapc_le_smp_generate_rand(p_proc);
            }
            else
            {
                status = gapc_le_smp_sc_central_start_dh_key_check(p_proc, p_is_finished);
            }
        } break;

        default: { /* ignore */ } break;
    }
    return (status);
}


/// Start Authentication stage 1
__STATIC  uint16_t gapc_le_smp_sc_central_start_stage_1(gapc_le_smp_pair_proc_t* p_proc,
                                                        bool* p_is_finished)
{
    gapc_le_smp_proc_transition_cb transition_cb = NULL;
    switch(p_proc->pair_method)
    {
        case GAPC_LE_SMP_METH_OOB: { transition_cb = gapc_le_smp_sc_oob_central_proc_transition;   } break;
        case GAPC_LE_SMP_METH_NC:
        case GAPC_LE_SMP_METH_JW:  { transition_cb = gapc_le_smp_sc_nc_jw_central_proc_transition; } break;
        case GAPC_LE_SMP_METH_PK:  { transition_cb = gapc_le_smp_sc_pk_central_proc_transition;    } break;
        default: { ASSERT_ERR(0) ; } break;
    }

    return gapc_le_smp_transition_to_new_state_machine(p_proc, transition_cb,
                                                       GAPC_LE_SMP_EVT_PAIRING_STAGE_1_STARTED, p_is_finished);
}

// Function that handles transition of central secure connection pairing procedure
__STATIC uint16_t gapc_le_smp_sc_central_proc_transition(gapc_le_smp_pair_proc_t* p_proc, uint8_t event,
                                                         bool* p_is_finished)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    *p_is_finished = false;

    switch(event)
    {
        case GAPC_LE_SMP_EVT_PAIRING_AUTHENTICATION_STARTED:
        {
            // get local public key
            status = gapc_le_smp_get_local_pub_key(p_proc);
            if(status != GAP_ERR_NO_ERROR) break;

            // If public key already available handle next pairing step
            if(!GETB(p_proc->info_bf, GAPC_LE_SMP_LOC_PUB_KEY_GEN)) break;
        }
        // no break

        case GAPC_LE_SMP_EVT_PUBLIC_KEY_COMPUTED:
        {
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_PUBLIC_KEY;
            status = gapc_le_smp_send_public_key_pdu(p_proc);
        } break;

        case GAPC_LE_SMP_EVT_PEER_PUB_KEY_RECEIVED:
        {
            status = gapm_compute_dh_key(p_proc->hdr.conidx, &(p_proc->peer_public_key), gapc_le_smp_compute_dh_key_cmp_cb);
            if(status != GAP_ERR_NO_ERROR) break;

            // Start Pairing stage 1
            status = gapc_le_smp_sc_central_start_stage_1(p_proc, p_is_finished);
        } break;

        // pairing stage 2 - DH-Key Checked
        case GAPC_LE_SMP_EVT_DH_KEY_CHECK_STARTED:
        {
            if(!GETB(p_proc->info_bf, GAPC_LE_SMP_LOC_DHKEY_COMP)) break;
        }
        // no break;

        case GAPC_LE_SMP_EVT_DH_KEY_COMPUTED:
        {
            // compute mac key
            gapc_le_smp_central_compute_mac_key(p_proc);
        } break;

        case GAPC_LE_SMP_EVT_MAC_KEY_COMPUTED:
        {
            gapc_le_smp_compute_local_dhkey_check(p_proc, &(p_proc->pair_req_pdu));
        } break;

        case GAPC_LE_SMP_EVT_LOCAL_DHKEY_CHECK_COMPUTED:
        {
            // Send the DHKeyCheck to the peer device
            status = gapc_le_smp_send_dhkey_check_pdu(p_proc);
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_DHKEY_CHECK;
        } break;

        case GAPC_LE_SMP_EVT_PEER_DH_KEY_CHECK_RECEIVED:
        {
            gapc_le_smp_compute_peer_dhkey_check(p_proc, &(p_proc->pair_rsp_pdu));
        } break;

        case GAPC_LE_SMP_EVT_PEER_DHKEY_CHECK_COMPUTED:
        {
            // Begin to encrypt the link..
            status = gapc_le_smp_send_start_enc_cmd(p_proc);
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_ENC;
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
/// Start the DH-Key Check Procedure
__STATIC  uint16_t gapc_le_smp_sc_periph_start_dh_key_check(gapc_le_smp_pair_proc_t* p_proc,
                                                             bool* p_is_finished)
{
    return gapc_le_smp_transition_to_new_state_machine(p_proc, gapc_le_smp_sc_periph_proc_transition,
                                                       GAPC_LE_SMP_EVT_DH_KEY_CHECK_STARTED, p_is_finished);
}


// Function that handles transition of peripheral secure connection OOB pairing procedure
__STATIC uint16_t gapc_le_smp_sc_oob_periph_proc_transition(gapc_le_smp_pair_proc_t* p_proc, uint8_t event,
                                                            bool* p_is_finished)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    switch(event)
    {
        case GAPC_LE_SMP_EVT_PAIRING_STAGE_1_STARTED:
        {
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_PAIRING_RAND;
        } break;
        case GAPC_LE_SMP_EVT_PEER_RANDOM_RECEIVED:
        {
            // Start OOB exchange request to application
            status = gapc_le_smp_load_local_oob_data(p_proc, p_proc->pair_req_pdu.oob);
            if(status != GAP_ERR_NO_ERROR) break;

            gapc_le_smp_load_peer_oob_data(p_proc, p_proc->pair_rsp_pdu.oob);
            if(!gapc_le_smp_is_app_answer_request(p_proc)) break;
        }
        // no break;
        case GAPC_LE_SMP_EVT_OOB_DATA_PROVIDED:
        {
            if(!GETB(p_proc->info_bf, GAPC_LE_SMP_OOB_CFM_COMP))
            {
                // Device reject OOB
                status = gapc_le_smp_check_user_accept(p_proc, L2CAP_SMP_ERR_OOB_NOT_AVAILABLE);
                if(status != GAP_ERR_NO_ERROR) break;

                //  initiate the Verification of the peers confirm.
                // Cb = (PKb, PKb, rb, 0) if slave
                gapc_le_smp_oob_compute_peer_confirm(p_proc);
                break;
            }
        }
        // no break;
        case GAPC_LE_SMP_EVT_PEER_CONFIRM_COMPUTED:
        {
            // Generate a local Random number
            gapc_le_smp_generate_rand(p_proc);
        } break;

        case GAPC_LE_SMP_EVT_LOCAL_RAND_GENERATED:
        {
            // Send local Rand to peer.
            gapc_le_smp_send_pair_rand_pdu(p_proc);
            status = gapc_le_smp_sc_periph_start_dh_key_check(p_proc, p_is_finished);
        } break;

        default: { /* ignore */ } break;
    }
    return (status);
}

// Function that handles transition of peripheral secure connection numeric comparison or Just work pairing procedure
__STATIC uint16_t gapc_le_smp_sc_nc_jw_periph_proc_transition(gapc_le_smp_pair_proc_t* p_proc, uint8_t event,
                                                              bool* p_is_finished)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    switch(event)
    {
        case GAPC_LE_SMP_EVT_PAIRING_STAGE_1_STARTED:
        {
            // Start the generation of a Random number Nb
            gapc_le_smp_generate_rand(p_proc);

        } break;
        case GAPC_LE_SMP_EVT_LOCAL_RAND_GENERATED:
        {
            // Cb = f4(Pkb,Pka,Nb,0)
            gapc_le_smp_compute_local_confirm(p_proc, 0);
        } break;
        case GAPC_LE_SMP_EVT_LOCAL_CONFIRM_COMPUTED:
        {
            status = gapc_le_smp_send_pairing_confirm_pdu(p_proc);

            // Now Wait for the Random Na from the master
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_PAIRING_RAND;
        } break;
        case GAPC_LE_SMP_EVT_PEER_RANDOM_RECEIVED:
        {
            // Send the localy generated random value
            status = gapc_le_smp_send_pair_rand_pdu(p_proc);

            if(status != GAP_ERR_NO_ERROR) break;
            if (p_proc->pair_method == GAPC_LE_SMP_METH_JW)
            {
                // phase 1 finished
                status = gapc_le_smp_sc_periph_start_dh_key_check(p_proc, p_is_finished);
            }
            else if (p_proc->pair_method == GAPC_LE_SMP_METH_NC)
            {
                gapc_le_smp_compute_numeric_value(p_proc, p_proc->peer_public_key.x, p_proc->local_public_key.x,
                                                  p_proc->peer_rand, p_proc->local_rand);
            }
        } break;

        case GAPC_LE_SMP_EVT_NUMERIC_VALUE_COMPUTED:
        {
            // Ask application to accept numeric comparison
            gapc_le_smp_ask_numeric_comp_to_app(p_proc);
            if(!gapc_le_smp_is_app_answer_request(p_proc)) break;
        }
        // no break

        case GAPC_LE_SMP_EVT_NUMERIC_COMPARE_RSP_PROVIDED:
        {
            // check if application accepts
            status = gapc_le_smp_check_user_accept(p_proc, L2CAP_SMP_ERR_NUMERIC_COMPARISON_FAILED);
            if(status != GAP_ERR_NO_ERROR) break;

            // prepare for DH-Key Check computation
            memset(p_proc->local_r, 0, GAP_KEY_LEN);
            memset(p_proc->peer_r,  0, GAP_KEY_LEN);

            // phase 1 finished
            status = gapc_le_smp_sc_periph_start_dh_key_check(p_proc, p_is_finished);
        } break;

        default: { /* ignore */ } break;
    }
    return (status);
}

// Function that handles transition of peripheral secure connection Passkey entry pairing procedure
__STATIC uint16_t gapc_le_smp_sc_pk_periph_proc_transition(gapc_le_smp_pair_proc_t* p_proc, uint8_t event,
                                                              bool* p_is_finished)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    switch(event)
    {
        case GAPC_LE_SMP_EVT_PAIRING_STAGE_1_STARTED:
        {
            // Ask passkey value
            uint8_t exp_value = gapc_le_smp_sc_pk_get_app_expected_value(p_proc, p_proc->pair_rsp_pdu.iocap,
                                                                         p_proc->pair_req_pdu.iocap);
            gapc_le_smp_ask_info_to_app(p_proc, exp_value, GAPC_LE_SMP_VALUE_PASSKEY);

            p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_PAIRING_CONFIRM;
            if(!gapc_le_smp_is_app_answer_request(p_proc)) break;
        }
        // no break;
        case GAPC_LE_SMP_EVT_PASSKEY_PROVIDED:
        {
            // Check if user accept
            status = gapc_le_smp_check_user_accept(p_proc, L2CAP_SMP_ERR_PASSKEY_ENTRY_FAILED);
            if(status != GAP_ERR_NO_ERROR) break;

            // prepare for DH-Key Check computation
            memset(p_proc->local_r, 0, GAP_KEY_LEN);
            co_write32p(p_proc->local_r, p_proc->passkey);
            memcpy(p_proc->peer_r, p_proc->local_r, GAP_KEY_LEN);

            p_proc->passkey_bit_cursor = 0; // First Round

            // continue only if peer confirm received
            if(p_proc->peer_exp_message == GAPC_LE_SMP_W4_RX_PAIRING_CONFIRM) break;
        }
        // no break;

        case GAPC_LE_SMP_EVT_PEER_CONFIRM_RECEIVED:
        {
            if(!gapc_le_smp_is_app_answer_request(p_proc)) break; //  wait application passkey
            gapc_le_smp_generate_rand(p_proc);
        } break;

        case GAPC_LE_SMP_EVT_LOCAL_RAND_GENERATED:
        {
            // Begin the Local Commitment determination -
            // if slave: Cb = f4(Pkb,Pka,Nbi,rbi)
            gapc_le_smp_compute_local_confirm(p_proc, gapc_le_smp_get_next_passkey_bit(p_proc));
        } break;

        case GAPC_LE_SMP_EVT_LOCAL_CONFIRM_COMPUTED:
        {
            status = gapc_le_smp_send_pairing_confirm_pdu(p_proc);

            // Now Wait for the Random Na from the master
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_PAIRING_RAND;
        } break;

        case GAPC_LE_SMP_EVT_PEER_RANDOM_RECEIVED:
        {
            // If slave  Cai = f4(Pka,Pkb,Nai,rbi)
            gapc_le_smp_compute_peer_confirm(p_proc, gapc_le_smp_get_next_passkey_bit(p_proc));
        } break;

        case GAPC_LE_SMP_EVT_PEER_CONFIRM_COMPUTED:
        {
            // Send the localy generated random value
            status = gapc_le_smp_send_pair_rand_pdu(p_proc);

            // Check the PassKey bit counter to see if this is the last round
            if (p_proc->passkey_bit_cursor < BLE_LE_SMP_SC_PASSKEY_NUM_BIT)
            {
                p_proc->passkey_bit_cursor++;
                p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_PAIRING_CONFIRM;
            }
            else
            {
                // phase 1 finished
                status = gapc_le_smp_sc_periph_start_dh_key_check(p_proc, p_is_finished);
            }
        } break;

        default: { /* ignore */ } break;
    }
    return (status);
}

/// Start authentication stage 1
__STATIC  uint16_t gapc_le_smp_sc_periph_start_stage_1(gapc_le_smp_pair_proc_t* p_proc,
                                                       bool* p_is_finished)
{
    gapc_le_smp_proc_transition_cb transition_cb = NULL;

    switch(p_proc->pair_method)
    {
        case GAPC_LE_SMP_METH_OOB: { transition_cb = gapc_le_smp_sc_oob_periph_proc_transition;   } break;
        case GAPC_LE_SMP_METH_NC:
        case GAPC_LE_SMP_METH_JW:  { transition_cb = gapc_le_smp_sc_nc_jw_periph_proc_transition; } break;
        case GAPC_LE_SMP_METH_PK:  { transition_cb = gapc_le_smp_sc_pk_periph_proc_transition;    } break;
        default: { ASSERT_ERR(0) ; } break;
    }

    return gapc_le_smp_transition_to_new_state_machine(p_proc, transition_cb,
                                                       GAPC_LE_SMP_EVT_PAIRING_STAGE_1_STARTED, p_is_finished);
}

// Function that handles transition of peripheral secure connection pairing procedure
__STATIC uint16_t gapc_le_smp_sc_periph_proc_transition(gapc_le_smp_pair_proc_t* p_proc, uint8_t event,
                                                        bool* p_is_finished)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    *p_is_finished = false;

    switch(event)
    {
        case GAPC_LE_SMP_EVT_PAIRING_AUTHENTICATION_STARTED:
        {
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_PUBLIC_KEY;
            // Read the local public key
            status = gapc_le_smp_get_local_pub_key(p_proc);
        } break;

        case GAPC_LE_SMP_EVT_PEER_PUB_KEY_RECEIVED:
        {
            // check if it has not been already received
            if(!GETB(p_proc->info_bf, GAPC_LE_SMP_LOC_PUB_KEY_GEN)) break;
        }
        // no break;

        case GAPC_LE_SMP_EVT_PUBLIC_KEY_COMPUTED:
        {
            // Peer public key must have been received
            if(p_proc->peer_exp_message == GAPC_LE_SMP_W4_RX_PUBLIC_KEY) break;

            // send public key
            status = gapc_le_smp_send_public_key_pdu(p_proc);
            if(status != GAP_ERR_NO_ERROR) break;

            // Generate DH key with public key received from peer device
            status = gapm_compute_dh_key(p_proc->hdr.conidx, &(p_proc->peer_public_key), gapc_le_smp_compute_dh_key_cmp_cb);
            if(status != GAP_ERR_NO_ERROR) break;

            // Use pairing mode function transition
            status = gapc_le_smp_sc_periph_start_stage_1(p_proc, p_is_finished);
        } break;

        case GAPC_LE_SMP_EVT_DH_KEY_CHECK_STARTED:
        {
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_RX_DHKEY_CHECK;
        } break;

        case GAPC_LE_SMP_EVT_PEER_DH_KEY_CHECK_RECEIVED:
        {
            if(!GETB(p_proc->info_bf, GAPC_LE_SMP_LOC_DHKEY_COMP)) break;
        }
        // no break;

        case GAPC_LE_SMP_EVT_DH_KEY_COMPUTED:
        {
            // ensure that peer DH-KEY check received
            if(p_proc->peer_exp_message == GAPC_LE_SMP_W4_RX_DHKEY_CHECK) break;
            // compute mac key
            gapc_le_smp_periph_compute_mac_key(p_proc);
        } break;

        case GAPC_LE_SMP_EVT_MAC_KEY_COMPUTED:
        {
            gapc_le_smp_compute_peer_dhkey_check(p_proc, &(p_proc->pair_req_pdu));
        } break;

        case GAPC_LE_SMP_EVT_PEER_DHKEY_CHECK_COMPUTED:
        {
            // Begin the DHKey Check.
            gapc_le_smp_compute_local_dhkey_check(p_proc, &(p_proc->pair_rsp_pdu));
        } break;
        case GAPC_LE_SMP_EVT_LOCAL_DHKEY_CHECK_COMPUTED:
        {
            // Wait for the Encryption to be initiated by the Master
            p_proc->peer_exp_message = GAPC_LE_SMP_W4_ENC;

            // Send the Local DHKeyCheck to the peer device
            status = gapc_le_smp_send_dhkey_check_pdu(p_proc);
        } break;

        case GAPC_LE_SMP_EVT_HCI_LTK_REQ_RECEIVED:
        {
            // accept LTK request
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
/// Central start secure connection pairing
uint16_t gapc_le_smp_sc_central_proc_start(gapc_le_smp_pair_proc_t* p_proc, bool* p_is_finished)
{
    return gapc_le_smp_transition_to_new_state_machine(p_proc, gapc_le_smp_sc_central_proc_transition,
                                                       GAPC_LE_SMP_EVT_PAIRING_AUTHENTICATION_STARTED, p_is_finished);
}
#endif // (HL_LE_CENTRAL)

#if (HL_LE_PERIPHERAL)
/// Peripheral start secure connection pairing
uint16_t gapc_le_smp_sc_periph_proc_start(gapc_le_smp_pair_proc_t* p_proc,  bool* p_is_finished)
{
    return gapc_le_smp_transition_to_new_state_machine(p_proc, gapc_le_smp_sc_periph_proc_transition,
                                                       GAPC_LE_SMP_EVT_PAIRING_AUTHENTICATION_STARTED, p_is_finished);
}
#endif // (HL_LE_PERIPHERAL)

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint16_t gapc_le_pairing_numeric_compare_rsp(uint8_t conidx, bool accept)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapc_le_smp_pair_proc_t* p_proc;

    if(gapc_le_smp_is_app_involved(conidx, GAPC_LE_SMP_VALUE_NUMERIC_COMPARE, &p_proc))
    {
        SETB(p_proc->info_bf, GAPC_LE_SMP_USER_ACCEPT, accept != false);
        gapc_le_smp_try_proc_transition(p_proc, GAPC_LE_SMP_EVT_NUMERIC_COMPARE_RSP_PROVIDED);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}


uint16_t gapc_pairing_provide_oob_data(uint8_t conidx, bool accept, const gap_oob_t* p_data)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapc_le_smp_pair_proc_t* p_proc;

    if(accept && (p_data == NULL))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else if(gapc_le_smp_is_app_involved(conidx, GAPC_LE_SMP_VALUE_OOB, &p_proc))
    {
        SETB(p_proc->info_bf, GAPC_LE_SMP_USER_ACCEPT, accept != false);

        if(accept)
        {
            int8_t loop;

            // Copy the Confirm and Rand values from the peer.
            memcpy(p_proc->peer_confirm, p_data->conf, CFM_LEN);
            memcpy(p_proc->peer_r,       p_data->rand, RAND_VAL_LEN);

            // check if received random number is valid or not.
            for (loop = (GAP_KEY_LEN-1) ; loop >= 0 ; loop--)
            {
                if (p_data->rand[loop] != 0) { break; }
            }

            // automatic reject if random equals zero
            SETB(p_proc->info_bf, GAPC_LE_SMP_USER_ACCEPT, (loop != 0));
        }

        gapc_le_smp_try_proc_transition(p_proc, GAPC_LE_SMP_EVT_OOB_DATA_PROVIDED);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

uint16_t gapc_le_pairing_provide_passkey(uint8_t conidx, bool accept, uint32_t passkey)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    gapc_le_smp_pair_proc_t* p_proc;

    if(gapc_le_smp_is_app_involved(conidx, GAPC_LE_SMP_VALUE_PASSKEY, &p_proc))
    {
        SETB(p_proc->info_bf, GAPC_LE_SMP_USER_ACCEPT, accept != false);
        p_proc->passkey = passkey;
        gapc_le_smp_try_proc_transition(p_proc, GAPC_LE_SMP_EVT_PASSKEY_PROVIDED);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

uint16_t gapc_send_key_pressed_notification(uint8_t conidx, uint8_t notification_type)
{
    uint16_t status;

    do
    {
        gapc_le_smp_pair_proc_t* p_proc = gapc_le_smp_get_pairing_proc(conidx);

        // send notification only for an active passkey pairing
        if((p_proc == NULL) || (p_proc->pair_method != GAPC_LE_SMP_METH_PK))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        status = gapc_le_smp_send_keypress_ntf_pdu(p_proc, notification_type);
    } while (0);

    return (status);
}
#endif // (BLE_GAPC)

/// @} GAPC
