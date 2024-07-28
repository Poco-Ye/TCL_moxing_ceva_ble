/**
 ****************************************************************************************
 *
 * @file gapm_smp.c
 *
 * @brief Generic Access Profile Manager Security manager handler module.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPM_SMP Generic Access Profile Manager Security manager
 * @ingroup GAPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#if (BLE_HOST_PRESENT)
#include "gapm_int.h"
#include "../gap_int.h"
#include "gapm_le.h"

#include "hl.h"
#include "rwip.h"            // Common API to retrieve device parameters

#include <string.h>

#include "co_error.h"
#include "co_bt.h"
#include "co_math.h"
#include "co_version.h"
#include "co_utils.h"        // core utility functions


#include "ke_mem.h"

#include "hl_hci.h"
#include "hci.h"

#include "aes.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Event transition for ECDH and generate LE OOB Data state machines
enum gapm_smp_ecdh_oob_event
{
    /// ECDH status event command received
    GAPM_SMP_ECDH_HCI_STAT_EVT_RECV = HL_PROC_EVENT_FIRST,
    /// ECDH command complete event received
    GAPM_SMP_ECDH_HCI_CMP_EVT_RECV,
    #if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
    /// Generate random number done
    GAPM_SMP_OOB_RAND_GENERATED,
    /// Confirm computed
    GAPM_SMP_OOB_CONFIRM_COMPUTED,
    #endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure of GET Public Key or Gen DH-Key procedure
typedef struct gapm_smp_proc_ecdh
{
    /// Procedure inheritance
    hl_proc_t            hdr;
    /// Dummy parameter provided by upper layer SW
    uint32_t             dummy;
    /// Callback to execute at end of procedure execution
    gapm_proc_cmp_cb     res_cb;
    /// Public key resut
    public_key_t         pub_key;
} gapm_smp_proc_ecdh_t;

/*
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */

__STATIC bool gapm_smp_proc_get_pub_key_transition(gapm_smp_proc_ecdh_t* p_proc, uint8_t event, uint16_t status);
__STATIC bool gapm_smp_proc_compute_dhkey_transition(gapm_smp_proc_ecdh_t* p_proc, uint8_t event, uint16_t status);

#if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
__STATIC bool gapm_smp_proc_gen_le_oob_data_transition(gapm_smp_proc_ecdh_t* p_proc, uint8_t event, uint16_t status);
#endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */


/// Get P256 ECDH Public key procedure transition
__STATIC const hl_proc_itf_t gapm_smp_proc_get_pub_key_itf =
{
    .transition  = (hl_proc_transition_cb) gapm_smp_proc_get_pub_key_transition,
    .cleanup     = hl_proc_cleanup,
};

/// Compute P256 DH Key procedure transition
__STATIC const hl_proc_itf_t gapm_smp_proc_compute_dhkey_itf =
{
    .transition  = (hl_proc_transition_cb) gapm_smp_proc_compute_dhkey_transition,
    .cleanup     = hl_proc_cleanup,
};


#if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
/// Generate LE OOB data using P256 public key procedure transition
__STATIC const hl_proc_itf_t gapm_smp_proc_gen_le_oob_data_itf =
{
    .transition  = (hl_proc_transition_cb) gapm_smp_proc_gen_le_oob_data_transition,
    .cleanup     = hl_proc_cleanup,
};
#endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


#if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
/// AES RAND Result
__STATIC void gapm_smp_oob_rand_res_cb(uint8_t aes_status, const uint8_t* aes_res, uint32_t event)
{
    uint16_t status = RW_ERR_HCI_TO_HL(aes_status);
    if((status == GAP_ERR_NO_ERROR) && (gapm_env.p_oob != NULL))
    {
        memcpy(gapm_env.p_oob->data.rand, aes_res, GAP_KEY_LEN);
    }

    gapm_proc_transition(GAPM_PROC_ECDH, event, status);
}

__STATIC void gapm_smp_oob_f4_res_cb(uint8_t aes_status, const uint8_t* aes_res, uint32_t event)
{
    uint16_t status = RW_ERR_HCI_TO_HL(aes_status);
    if((status == GAP_ERR_NO_ERROR) && (gapm_env.p_oob != NULL))
    {
        memcpy(gapm_env.p_oob->data.conf, aes_res, GAP_KEY_LEN);
    }

    gapm_proc_transition(GAPM_PROC_ECDH, event, status);
}
#endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)

/*
 * MESSAGES HANDLERS DEFINITIONS
 ****************************************************************************************
 */


/*
 * HCI EVENT HANDLERS DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handles the command status event.
 *
 * @param[in] opcode    HCI Command OP Code for command complete event and command status
 * @param[in] event     Event transition
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
__STATIC void gapm_smp_hci_cmd_stat_evt_handler(uint16_t opcode, uint16_t event, struct hci_cmd_stat_event const *p_evt)
{
    gapm_proc_transition(GAPM_PROC_ECDH, event, RW_ERR_HCI_TO_HL(p_evt->status));
}

/**
 ****************************************************************************************
 * @brief Handles the LE generate dh key complete event.
 *
 * @param[in] evt_code  HCI code:
 *                          - HCI Event Code for general HCI Events
 *                          - HCI LE Event sub-code for general HCI LE Meta Events
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
void gapm_hci_le_gen_dhkey_cmp_evt_handler(uint8_t evt_code, struct hci_le_gen_dhkey_cmp_evt const *p_evt)
{
    uint8_t status = RW_ERR_HCI_TO_HL(p_evt->status);

    if(status == GAP_ERR_NO_ERROR)
    {
        gapm_smp_proc_ecdh_t* p_proc = (gapm_smp_proc_ecdh_t*) gapm_proc_get(GAPM_PROC_ECDH);
        memcpy(p_proc->pub_key.x, &p_evt->dh_key[0],  32);
    }

    gapm_proc_transition(GAPM_PROC_ECDH, GAPM_SMP_ECDH_HCI_CMP_EVT_RECV, status);
}

/**
 ****************************************************************************************
 * @brief Handles the LE read local public key complete event.
 *
 * @param[in] evt_code  HCI code:
 *                          - HCI Event Code for general HCI Events
 *                          - HCI LE Event sub-code for general HCI LE Meta Events
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
void gapm_hci_le_rd_local_p256_public_key_cmp_evt_handler(uint8_t evt_code,
                                                          struct hci_rd_local_p256_public_key_cmp_evt const *p_evt)
{
    uint8_t status = RW_ERR_HCI_TO_HL(p_evt->status);

    if(status == GAP_ERR_NO_ERROR)
    {
        gapm_smp_proc_ecdh_t* p_proc = (gapm_smp_proc_ecdh_t*) gapm_proc_get(GAPM_PROC_ECDH);
        memcpy(p_proc->pub_key.x, &p_evt->public_key[0],  32);
        memcpy(p_proc->pub_key.y,  &p_evt->public_key[32], 32);
    }

    gapm_proc_transition(GAPM_PROC_ECDH, GAPM_SMP_ECDH_HCI_CMP_EVT_RECV, status);
}

/*
 * PROCEDURE STATE MACHINES
 ****************************************************************************************
 */

/// Create the ECDH procedure
__STATIC uint16_t gapm_smp_ecdh_proc_create(uint32_t dummy, gapm_proc_cmp_cb res_cb, const hl_proc_itf_t* p_itf,
                                            gapm_smp_proc_ecdh_t** pp_proc)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;

    do
    {
        if(res_cb == NULL)
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // Create new procedure
        status = gapm_proc_create(GAPM_PROC_ECDH, sizeof(gapm_smp_proc_ecdh_t), p_itf, (hl_proc_t**) pp_proc);
        if(status != GAP_ERR_NO_ERROR) break;

        // Initialize procedure parameters
        (*pp_proc)->res_cb   = res_cb;
        (*pp_proc)->dummy    = dummy;
    } while (0);

    return (status);
}

/// Function called once Get Public key procedure is granted
__STATIC bool gapm_smp_proc_get_pub_key_transition(gapm_smp_proc_ecdh_t* p_proc, uint8_t event, uint16_t status)
{
    bool finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                // Generate a new Public Key.
                status = HL_HCI_BASIC_CMD_SEND(HCI_LE_RD_LOC_P256_PUB_KEY_CMD_OPCODE, GAPM_SMP_ECDH_HCI_STAT_EVT_RECV,
                                               gapm_smp_hci_cmd_stat_evt_handler);
            }break;
            case GAPM_SMP_ECDH_HCI_STAT_EVT_RECV: { /* Do Nothing */ } break;
            case GAPM_SMP_ECDH_HCI_CMP_EVT_RECV:  { finished = true; } break;
            default:                              { ASSERT_ERR(0);   } break;
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        finished = true;
    }

    if(finished)
    {
        gapm_public_key_cb res_cb = (gapm_public_key_cb) p_proc->res_cb;
        // inform upper layer about procedure execution
        res_cb(p_proc->dummy, status, &(p_proc->pub_key));
    }

    return (finished);
}

/// Function called once compute DH-key procedure is granted
__STATIC bool gapm_smp_proc_compute_dhkey_transition(gapm_smp_proc_ecdh_t* p_proc, uint8_t event, uint16_t status)
{
    bool finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                struct hci_le_gen_dhkey_v2_cmd *p_cmd =
                        HL_HCI_CMD_ALLOC(HCI_LE_GEN_DHKEY_V2_CMD_OPCODE, hci_le_gen_dhkey_v2_cmd);
                if(!p_cmd)
                {
                    status = GAP_ERR_INSUFF_RESOURCES;
                    break;
                }

                p_cmd->key_type = USE_GEN_PRIV_KEY;
                memcpy(&p_cmd->public_key[0],  p_proc->pub_key.x, 32);
                memcpy(&p_cmd->public_key[32], p_proc->pub_key.y, 32);
                HL_HCI_CMD_SEND_TO_CTRL(p_cmd, GAPM_SMP_ECDH_HCI_STAT_EVT_RECV, gapm_smp_hci_cmd_stat_evt_handler);
            }break;
            case GAPM_SMP_ECDH_HCI_STAT_EVT_RECV: { /* Do Nothing */ } break;
            case GAPM_SMP_ECDH_HCI_CMP_EVT_RECV:  { finished = true; } break;
            default:   { ASSERT_ERR(0);   } break;
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        finished = true;
    }

    if(finished)
    {
        gapm_dh_key_cb res_cb = (gapm_dh_key_cb)p_proc->res_cb;
        // inform upper layer about procedure execution
        res_cb(p_proc->dummy, status, (gap_dh_key_t*) &(p_proc->pub_key));
    }

    return (finished);
}

#if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
__STATIC bool gapm_smp_proc_gen_le_oob_data_transition(gapm_smp_proc_ecdh_t* p_proc, uint8_t event, uint16_t status)
{
    bool finished = false;

    if(status == GAP_ERR_NO_ERROR)
    {
        switch(event)
        {
            case HL_PROC_GRANTED:
            {
                // Generate a new Public Key.
                status = HL_HCI_BASIC_CMD_SEND(HCI_LE_RD_LOC_P256_PUB_KEY_CMD_OPCODE, GAPM_SMP_ECDH_HCI_STAT_EVT_RECV,
                                               gapm_smp_hci_cmd_stat_evt_handler);
            }break;
            case GAPM_SMP_ECDH_HCI_STAT_EVT_RECV: { /* Do Nothing */ } break;
            case GAPM_SMP_ECDH_HCI_CMP_EVT_RECV:
            {
                // use a cache for OOB data structure
                if(gapm_env.p_oob == NULL)
                {
                    gapm_env.p_oob = (gapm_oob_data_t*) ke_malloc_user(sizeof(gapm_oob_data_t) , KE_MEM_KE_MSG);
                    if(gapm_env.p_oob == NULL)
                    {
                        status = GAP_ERR_INSUFF_RESOURCES;
                        break;
                    }
                }

                gapm_env.p_oob->pub_key = p_proc->pub_key;
                aes_rand(gapm_smp_oob_rand_res_cb, GAPM_SMP_OOB_RAND_GENERATED);
            } break;
            case GAPM_SMP_OOB_RAND_GENERATED:
            {
                aes_f4(gapm_env.p_oob->pub_key.x, gapm_env.p_oob->pub_key.x, gapm_env.p_oob->data.rand, 0,
                       gapm_smp_oob_f4_res_cb, GAPM_SMP_OOB_CONFIRM_COMPUTED);
            } break;
            case GAPM_SMP_OOB_CONFIRM_COMPUTED:
            {
                finished = true;
            } break;
            default:                              { ASSERT_ERR(0);   } break;
        }
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        finished = true;
        gapm_oob_data_release();
    }

    if(finished)
    {
        gap_oob_t* p_data = gapm_env.p_oob ? &(gapm_env.p_oob->data) : NULL;
        gapm_le_oob_cb res_cb = (gapm_le_oob_cb)p_proc->res_cb;
        // inform upper layer about procedure execution
        res_cb(p_proc->dummy, status, p_data);
    }

    return (finished);
}
#endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)


/*
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
#if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
const gapm_oob_data_t* gapm_oob_data_get(void)
{
    return (gapm_env.p_oob);
}

/**
 ****************************************************************************************
 * Release if present OOB data generated for OOB pairing
 ****************************************************************************************
 */
void gapm_oob_data_release(void)
{
    if(gapm_env.p_oob != NULL)
    {
        ke_free(gapm_env.p_oob);
        gapm_env.p_oob = NULL;
    }
}
#endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)

// ************* GENERATE Random Address ******************* //

// result of address generation
__STATIC void gapm_aes_gen_rand_addr_cb(uint8_t aes_status, const gap_addr_t* p_addr, gapm_random_address_cb res_cb)
{
    uint16_t status = RW_ERR_HCI_TO_HL(aes_status);
    res_cb(status, p_addr);
}

uint16_t gapm_generate_random_address(uint8_t rnd_type, gapm_random_address_cb res_cb)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;

    do
    {
        if(res_cb == NULL)
        {
            status = GAP_ERR_MISSING_CALLBACK;
            break;
        }

        // Check the operation code and the address type
        switch (rnd_type)
        {
            case (GAP_BD_ADDR_STATIC):
            case (GAP_BD_ADDR_NON_RSLV):
            case (GAP_BD_ADDR_RSLV):
            {
                aes_gen_rand_addr(gapm_get_irk()->key, rnd_type, (aes_func_result_cb)gapm_aes_gen_rand_addr_cb,
                                  (uint32_t) res_cb);
                status = GAP_ERR_NO_ERROR;
            } break;

            default: { status = GAP_ERR_INVALID_PARAM; } break;
        }
    } while (0);

    return (status);
}

// ************* Resolve Random Address ******************* //

// Result of address resolution
__STATIC void gapm_smp_addr_resolv_res_cb(uint8_t aes_status, uint8_t index, gapm_resolve_address_res_cb res_cb, const gap_addr_t *p_addr,
                                     const gap_sec_key_t* p_irk)
{
    uint16_t status = (aes_status == CO_ERROR_NO_ERROR) ? GAP_ERR_NO_ERROR : GAP_ERR_NOT_FOUND;
    res_cb(status, p_addr, p_irk);
}

uint16_t gapm_resolve_address(const gap_addr_t* p_addr, uint8_t nb_irk, const gap_sec_key_t* p_irk, gapm_resolve_address_res_cb res_cb)
{
    uint16_t status;
    if(   (p_addr == NULL) || (p_irk == NULL) || (nb_irk == 0)
       || ((p_addr->addr[BD_ADDR_LEN-1] & BD_ADDR_RND_ADDR_TYPE_MSK) != BD_ADDR_RSLV))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else if (res_cb == NULL)
    {
        status = GAP_ERR_MISSING_CALLBACK;
    }
    else
    {
        aes_rpa_resolve(nb_irk, (const aes_key_t*) p_irk, (const uint8_t*) p_addr,
                        (aes_rpa_func_result_cb) gapm_smp_addr_resolv_res_cb, (uint32_t) res_cb);
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

// ************* AES Cipher Data ******************* //

// result of aes cipher generation
__STATIC void gapm_aes_cipher_decipher_cb(uint8_t aes_status, const gap_aes_cipher_t* p_cipher, gapm_aes_cipher_res_cb res_cb)
{
    uint16_t status = RW_ERR_HCI_TO_HL(aes_status);
    res_cb(status, p_cipher);
}

uint16_t gapm_aes_cipher(const uint8_t* p_key, const uint8_t* p_data, gapm_aes_cipher_res_cb res_cb)
{
    uint16_t status;
    if((p_key == NULL) || (p_data == NULL))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else if (res_cb == NULL)
    {
        status = GAP_ERR_MISSING_CALLBACK;
    }
    else
    {
        aes_encrypt(p_key, p_data, true, (aes_func_result_cb)gapm_aes_cipher_decipher_cb, (uint32_t) res_cb);
        status = GAP_ERR_NO_ERROR;
    }
    return (status);
}

uint16_t gapm_aes_decipher(const uint8_t* p_key, const uint8_t* p_data, gapm_aes_cipher_res_cb res_cb)
{
    uint16_t status;
    if((p_key == NULL) || (p_data == NULL))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else if (res_cb == NULL)
    {
        status = GAP_ERR_MISSING_CALLBACK;
    }
    else
    {
        aes_decrypt(p_key, p_data, true, (aes_func_result_cb)gapm_aes_cipher_decipher_cb, (uint32_t) res_cb);
        status = GAP_ERR_NO_ERROR;
    }
    return (status);
}

// ************* AES Number generation ******************* //
// result of aes cipher generation
__STATIC void gapm_aes_rand_cb(uint8_t aes_status, const gap_aes_rand_nb_t* p_rand, gapm_random_number_res_cb res_cb)
{
    uint16_t status = RW_ERR_HCI_TO_HL(aes_status);
    res_cb(status, p_rand);
}

uint16_t gapm_generate_random_number(gapm_random_number_res_cb res_cb)
{
    uint16_t status;
    if (res_cb == NULL)
    {
        status = GAP_ERR_MISSING_CALLBACK;
    }
    else
    {
        aes_rand((aes_func_result_cb)gapm_aes_rand_cb, (uint32_t) res_cb);
        status = GAP_ERR_NO_ERROR;
    }
    return (status);
}

uint16_t gapm_get_public_key(uint32_t dummy, gapm_public_key_cb res_cb)
{
    gapm_smp_proc_ecdh_t* p_proc = NULL;
    uint16_t status = gapm_smp_ecdh_proc_create(dummy, (gapm_proc_cmp_cb) res_cb, &gapm_smp_proc_get_pub_key_itf,
                                                &p_proc);
    return (status);
}


uint16_t gapm_compute_dh_key(uint32_t dummy, const public_key_t* p_pub_key, gapm_dh_key_cb res_cb)
{
    uint16_t status;
    gapm_smp_proc_ecdh_t* p_proc = NULL;
    if(p_pub_key == NULL)
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else
    {
        status = gapm_smp_ecdh_proc_create(dummy, (gapm_proc_cmp_cb) res_cb, &gapm_smp_proc_compute_dhkey_itf,
                                           &p_proc);
        if(status == GAP_ERR_NO_ERROR)
        {
            p_proc->pub_key = *p_pub_key;
        }
    }

    return (status);
}



#if (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
uint16_t gapm_generate_le_oob_data(uint32_t dummy, gapm_le_oob_cb res_cb)
{
    gapm_smp_proc_ecdh_t* p_proc = NULL;
    uint16_t status = gapm_smp_ecdh_proc_create(dummy, (gapm_proc_cmp_cb) res_cb, &gapm_smp_proc_gen_le_oob_data_itf,
                                                &p_proc);
    return (status);
}
#endif // (HL_LE_CENTRAL || HL_LE_PERIPHERAL)
#endif // (BLE_HOST_PRESENT)

/// @} GAPM_SMP
