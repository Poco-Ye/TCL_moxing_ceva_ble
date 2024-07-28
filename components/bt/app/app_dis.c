/**
 ****************************************************************************************
 *
 * @file app_dis.c
 *
 * @brief Device Information Application Module Entry point
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (BLE_APP_DIS)

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app.h"                     // Application Manager Definitions
#include "app_dis.h"                 // Device Information Service Application Definitions
#include "diss.h"                    // Device Information Profile Functions
#include "diss_msg.h"                // 
#include "prf_types.h"               // Profile Common Types Definitions
#include "prf.h"
#include "ke_task.h"                 // Kernel

#include "gap.h"
#include "gapc_msg.h"
#include "gapc.h"
#include "gapm_msg.h"
#include "gapm.h"

#include <string.h>
#include "co_utils.h"

static diss_cb_t diss_cb;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void diss_value_req_ind(uint32_t token, uint8_t val_id)
{
    // Initialize length
    uint8_t len = 0;
    // Pointer to the data
    uint8_t *data = NULL;

    // Check requested value
    switch (val_id)
    {
        case DIS_VAL_MANUFACTURER_NAME :
        {
            // Set information
            len = APP_DIS_MANUFACTURER_NAME_LEN;
            data = (uint8_t *)APP_DIS_MANUFACTURER_NAME;
        } break;

        case DIS_VAL_MODEL_NB_STR :
        {
            // Set information
            len = APP_DIS_MODEL_NB_STR_LEN;
            data = (uint8_t *)APP_DIS_MODEL_NB_STR;
        } break;

        case DIS_VAL_SYSTEM_ID :
        {
            // Set information
            len = APP_DIS_SYSTEM_ID_LEN;
            data = (uint8_t *)APP_DIS_SYSTEM_ID;
        } break;

        case DIS_VAL_PNP_ID :
        {
            // Set information
            len = APP_DIS_PNP_ID_LEN;
            data = (uint8_t *)APP_DIS_PNP_ID;
        } break;

        case DIS_VAL_SERIAL_NB_STR :
        {
            // Set information
            len = APP_DIS_SERIAL_NB_STR_LEN;
            data = (uint8_t *)APP_DIS_SERIAL_NB_STR;
        } break;

        case DIS_VAL_HARD_REV_STR :
        {
            // Set information
            len = APP_DIS_HARD_REV_STR_LEN;
            data = (uint8_t *)APP_DIS_HARD_REV_STR;
        } break;

        case DIS_VAL_FIRM_REV_STR :
        {
            // Set information
            len = APP_DIS_FIRM_REV_STR_LEN;
            data = (uint8_t *)APP_DIS_FIRM_REV_STR;
        } break;

        case DIS_VAL_SW_REV_STR :
        {
            // Set information
            len = APP_DIS_SW_REV_STR_LEN;
            data = (uint8_t *)APP_DIS_SW_REV_STR;
        } break;

        case DIS_VAL_IEEE :
        {
            // Set information
            len = APP_DIS_IEEE_LEN;
            data = (uint8_t *)APP_DIS_IEEE;
        } break;

        default:
            ASSERT_ERR(0);
            break;
    }

    // Allocate confirmation to send the value
    diss_value_cfm(token,len,&data[0]);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

uint16_t app_dis_init(void)
{
    // Nothing to do
	return GAP_ERR_NO_ERROR;
}

void app_dis_add_dis(void)
{
	uint16_t features = APP_DIS_FEATURES;
	uint16_t handle = 0;

	diss_cb.cb_value_get = diss_value_req_ind;

    prf_add_profile(TASK_ID_DISS,0x00 /* Security - should be SVC_SEC_LVL(NO_AUTH) */,0x0 /* user Priority*/,(void*)&features,(void*)&diss_cb, &handle);

}


#endif //BLE_APP_DIS

/// @} APP
