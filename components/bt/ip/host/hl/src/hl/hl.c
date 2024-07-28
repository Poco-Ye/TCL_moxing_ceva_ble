/**
 ****************************************************************************************
 *
 * @file rwble_hl.c
 *
 * @brief Entry points the BLE software
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <string.h>
#include "rwip_config.h"
#include "rwip.h"
#include "hl.h"

#include "gapm.h"
#if (BLE_L2CAP)
#include "../inc/l2cap_hl_api.h"
#endif // (BLE_L2CAP)

#if (BLE_GATT)
#include "../inc/gatt_hl_api.h"
#endif // (BLE_GATT)

#if (HOST_PROFILES)
#include "../inc/prf_hl_api.h"
#endif // (HOST_PROFILES)

#if (HOST_PROFILES)
#include "prf.h"
#endif // (HOST_PROFILES)

#if (BT_HOST_PRESENT)
#include "bk_al.h"
#endif // (BT_HOST_PRESENT)

#if (!EMB_PRESENT)
#include "hl_hci.h"
#include "aes.h"
#endif // (!EMB_PRESENT)

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

extern void gapm_initialize(uint8_t init_type);
#if (GAPC_PRESENT)
extern void gapc_initialize(uint8_t init_type);
#endif // (GAPC_PRESENT)
extern void hl_hci_initialize(uint8_t init_type);
extern void hl_proc_initialize(uint8_t init_type);


void hl_initialize(uint8_t init_type)
{
    // Initialize connection to host interface
    hl_hci_initialize(init_type);
    // Initialize Procedure
    hl_proc_initialize(init_type);

    #if(BLE_L2CAP)
    // Initialize L2CAP
    l2cap_initialize(init_type);
    #endif //(BLE_L2CAP)

    #if (BLE_GATT)
    // Initialize GATT
    gatt_initialize(init_type);
    #endif // (BLE_GATT)

    // Initialize GAP
    gapm_initialize(init_type);

    // Profile manager initialization
    #if (HOST_PROFILES)
    prf_initialize(init_type);
    #endif // (HOST_PROFILES)

    #if (GAPC_PRESENT)
    // Initialize GAP controllers
    gapc_initialize(init_type);
    #endif //(GAPC_PRESENT)

    #if (BT_HOST_PRESENT)
    // Initialize BT Host stack
    bk_al_init(init_type);
    #endif // (BT_HOST_PRESENT)
}

#if(!EMB_PRESENT)
/**
 ****************************************************************************************
 * @brief HCI LE Encrypt command complete event handler.
 * The received encrypted data is to be sent to the saved source task ID (which is the
 * origin of the request).
 *
 * @param[in] opcode    HCI Command OP Code for command complete event and command status
 * @param[in] conhdl    Unused
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
__STATIC void hl_hci_le_enc_dec_cmd_cmp_evt_handler(uint16_t opcode, uint16_t conhdl,
                                                    struct hci_le_enc_cmd_cmp_evt *p_evt)
{
    // inform AES result handler
    aes_result_handler(p_evt->status, &p_evt->encrypted_data[0]);
}

/**
 ****************************************************************************************
 * @brief Start AES function
 *
 * The exchange memory must be filled before calling this function.
 * This function expect to be called from a BLE Module
 *
 * @param[in] key           AES Encryption key must be 16 bytes
 * @param[in] val           16 bytes value array to encrypt using AES
 * @param[in] cipher        True to cipher data; de-cipher data otherwise
 ****************************************************************************************
 */
void rwip_aes(const uint8_t *key, const uint8_t* val, bool cipher)
{
    if(cipher)
    {
        struct hci_le_enc_cmd *p_cmd = HL_HCI_CMD_ALLOC(HCI_LE_ENC_CMD_OPCODE, hci_le_enc_cmd);

        if(p_cmd != NULL)
        {
            memcpy(&p_cmd->key.ltk[0],    key, GAP_KEY_LEN);
            memcpy(&p_cmd->plain_data[0], val, GAP_KEY_LEN);

            HL_HCI_CMD_SEND_TO_CTRL(p_cmd, 0, hl_hci_le_enc_dec_cmd_cmp_evt_handler);
        }
    }
    else
    {
        struct hci_vs_le_decrypt_cmd *p_cmd = HL_HCI_CMD_ALLOC(HCI_VS_LE_DECRYPT_CMD_OPCODE, hci_vs_le_decrypt_cmd);

        if(p_cmd != NULL)
        {
            memcpy(&p_cmd->key.ltk[0],      key, GAP_KEY_LEN);
            memcpy(&p_cmd->encrypt_data[0], val, GAP_KEY_LEN);

            HL_HCI_CMD_SEND_TO_CTRL(p_cmd, 0, hl_hci_le_enc_dec_cmd_cmp_evt_handler);
        }
    }
}
#endif // (!EMB_PRESENT)

/// @} RWBLE
