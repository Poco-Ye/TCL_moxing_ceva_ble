/**
 ****************************************************************************************
 *
 * @file gapm_bt_actv.c
 *
 * @brief Generic Access Profile Manager - BT Activities
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @ingroup GAPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#if (BT_HOST_PRESENT)

#include <string.h>
#include "gap.h"
#include "gapm_int.h"

#include "classic/btstack_device_id_server.h" // Device Identification function used to register SDP record
#include "classic/btstack_sdp_server.h"       // Third party SDP API
#include "classic/btstack_sdp_util.h"         // Retrieve size of SDP information
#include "bk_al.h"                            // BK Adaptation layer
#include "ke_mem.h"                           // Allocated SDP record

/*
 * DEFINES
 ****************************************************************************************
 */
#define GAPM_BT_BASE_SERVICE_RECORD_HANDLE    (0x10000)
#define GAPM_BT_DEVICE_IDENTITY_SDP_SVC_SIZE  (64)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DECLARATION
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

/*
 * HCI HANDLERS DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint32_t gapm_sdp_get_next_service_record_handle(void)
{
    // Increment SDP record handle counter and return BASE HANDLE + counter
    return (GAPM_BT_BASE_SERVICE_RECORD_HANDLE + (++gapm_env.sdp_rec_hdl_cnt));
}


uint16_t gapm_set_sdp_device_identification_record(uint16_t vendor_id_source, uint16_t vendor_id, uint16_t product_id,
                                                   uint16_t version)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    if(gapm_env.configured && (gapm_env.p_device_identification_sdp_record == NULL))
    {
        gapm_env.p_device_identification_sdp_record = (uint8_t*) ke_malloc_user(GAPM_BT_DEVICE_IDENTITY_SDP_SVC_SIZE, KE_MEM_PROFILE);
        if(gapm_env.p_device_identification_sdp_record == NULL)
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
        else
        {
            device_id_create_sdp_record(gapm_env.p_device_identification_sdp_record, gapm_sdp_get_next_service_record_handle(),
                                        vendor_id_source, vendor_id, product_id, version);
            status = sdp_register_service(gapm_env.p_device_identification_sdp_record);
            status = bk_al_convert_err_code(status);
        }
    }

    return (status);
}

#endif // (BT_HOST_PRESENT)

/// @} GAPM
