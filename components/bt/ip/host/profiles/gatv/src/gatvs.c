/**
 ****************************************************************************************
 *
 * @file gatvs.c
 *
 * @brief google atv Server Implementation.
 *
 *Copyright (C) MooreSilicon 2021-2031
 *
 *
 ****************************************************************************************
 */

 /**
  ****************************************************************************************
  * @addtogroup HIDS
  * @{
  ****************************************************************************************
  */

  /*
   * INCLUDE FILES
   ****************************************************************************************
   */

#include "rwip_config.h"

#include "rwip_task.h" // Task definitions
#include "prf_types.h"

#if (BLE_GATV_SERVER)
#include "gatvs.h"
#include "gatt.h"

#include "prf_utils.h"
#include "prf.h"

#include "co_utils.h"
#include "co_endian.h"
#include "co_math.h"

#include "ke_mem.h"
#include <string.h>

#include "atiny_log.h"


   /*
	* DEFINES
	****************************************************************************************
	*/
	///Maximum number of gatv Server task instances
#define GATVS_VAL_MAX_LEN                         (310)


	/*
	 * TYPES DEFINITION
	 ****************************************************************************************
	 */


	 /// gatv service server environment variable
typedef struct gatvs_env
{
	/// profile environment
	prf_hdr_t           prf_env;
	cb_gatvs_att_read_get cb_get;
	cb_gatvs_att_set cb_set;
	/// Service Attribute Start Handle
	uint16_t            start_hdl;
	/// GATT user local identifier
	uint8_t             user_lid;
} gatvs_env_t;



/*
 * GATT USER SERVICE HANDLERS
 ****************************************************************************************
 */

 /**
  ****************************************************************************************
  * @brief This function is called when peer want to read local attribute datahide value.
  *
  *        #gatt_srv_att_read_get_cfm shall be called to provide attribute value
  *
  * @param[in] conidx        Connection index
  * @param[in] user_lid      GATT user local identifier
  * @param[in] token         Procedure token that must be returned in confirmation function
  * @param[in] hdl           Attribute handle
  * @param[in] offset        Data offset
  * @param[in] max_length    Maximum data length to return
  ****************************************************************************************
  */
__STATIC void gatvs_cb_att_read_get(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl, uint16_t offset,
	uint16_t max_length)
{

	gatvs_env_t* p_gatvs_env = PRF_ENV_GET(GATVS, gatvs);
	co_buf_t*   p_buf;
	uint16_t    status = PRF_APP_ERROR;
	dbg_print("gatvs_cb_att_read_get\r\n");

	if (p_gatvs_env != NULL)
	{
		int32_t att_idx = hdl - p_gatvs_env->start_hdl;

		// 应用分配缓存并填写内容，但应用不释放
		status = p_gatvs_env->cb_get(att_idx, &p_buf);
	}

	// Send result to peer device
	gatt_srv_att_read_get_cfm(conidx, user_lid, token, status, co_buf_data_len(p_buf), p_buf);

	if (p_buf != NULL)
	{
		co_buf_release(p_buf);
	}
	// dbg_print("hids_cb_att_read_get %d %d\r\n", co_buf_data_len(p_buf), status);
}

/**
 ****************************************************************************************
 * @brief This function is called during a write procedure to modify attribute handle.
 *
 *        #gatt_srv_att_val_set_cfm shall be called to accept or reject attribute
 *        update.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] token         Procedure token that must be returned in confirmation function
 * @param[in] hdl           Attribute handle
 * @param[in] offset        Value offset
 * @param[in] p_data        Pointer to buffer that contains data to write starting from offset
 ****************************************************************************************
 */
__STATIC void gatvs_cb_att_val_set(uint8_t conidx, uint8_t user_lid, uint16_t token, uint16_t hdl, uint16_t offset,
	co_buf_t* p_buf)
{
	gatvs_env_t* p_gatvs_env = PRF_ENV_GET(GATVS, gatvs);
	uint16_t status = PRF_APP_ERROR;
	
	dbg_print("gatvs_cb_att_val_set %08x %d\r\n", p_gatvs_env, status);

	if (p_gatvs_env != NULL)
	{
		int32_t att_idx = hdl - p_gatvs_env->start_hdl;

		status = p_gatvs_env->cb_set(conidx,att_idx, p_buf);
	}

	gatt_srv_att_val_set_cfm(conidx, user_lid, token, status);
}


uint16_t gatvs_send_report(uint8_t conidx,uint16_t att_idx, co_buf_t* p_buf)
{
	gatvs_env_t* p_gatvs_env = PRF_ENV_GET(GATVS, gatvs);
	uint16_t status = PRF_ERR_REQ_DISALLOWED;

	if (p_gatvs_env != NULL)
	{
		int32_t hdl = p_gatvs_env->start_hdl + att_idx;

		status = gatt_srv_event_send(conidx, p_gatvs_env->user_lid, 0, GATT_NOTIFY, hdl, p_buf);
	}
   //dbg_print("gatvs_send_report:%x\r\n",status);
	return (status);

}



/**
 ****************************************************************************************
 * @brief This function is called when GATT server user has initiated event send to peer
 *        device or if an error occurs.
 *
 * @param[in] conidx        Connection index
 * @param[in] user_lid      GATT user local identifier
 * @param[in] dummy         Dummy parameter provided by upper layer for command execution
 * @param[in] status        Status of the procedure (see enum #hl_err)
 ****************************************************************************************
 */
__STATIC void gatvs_cb_event_sent(uint8_t conidx, uint8_t user_lid, uint16_t dummy, uint16_t status)
{
	//dbg_print("gatvs_cb_event_sent %x,%x\r\n", conidx, status);

}

/// Service callback hander
__STATIC const gatt_srv_cb_t gatvs_cb =
{
		.cb_event_sent = gatvs_cb_event_sent,
		.cb_att_read_get = gatvs_cb_att_read_get,
		.cb_att_event_get = NULL,
		.cb_att_info_get = NULL,
		.cb_att_val_set = gatvs_cb_att_val_set,
};

/*
 * PROFILE DEFAULT HANDLERS
 ****************************************************************************************
 */

 /**
  ****************************************************************************************
  * @brief Initialization of the gatvs module.
  * This function performs all the initializations of the Profile module.
  *  - Creation of datahide (if it's a service)
  *  - Allocation of profile required memory
  *  - Initialization of task descriptor to register application
  *      - Task State array
  *      - Number of tasks
  *      - Default task handler
  *
  * @param[out]    p_env        Collector or Service allocated environment data.
  * @param[in,out] p_start_hdl  Service start handle (0 - dynamically allocated), only applies for services.
  * @param[in]     sec_lvl      Security level (see enum #gatt_svc_info_bf)
  * @param[in]     user_prio    GATT User priority
  * @param[in]     p_param      Configuration parameters of profile collector or service (32 bits aligned)
  * @param[in]     p_cb         Callback structure that handles event from profile
  *
  * @return status code to know if profile initialization succeed or not.
  ****************************************************************************************
  */
__STATIC uint16_t gatvs_init(prf_data_t *p_env, uint16_t *p_start_hdl, uint8_t sec_lvl, uint8_t user_prio,
	struct gatvs_db_cfg *p_params, const void* p_cb)
{
	//------------------ create the attribute datahide for the profile -------------------

	// DB Creation Status
	uint16_t status = GAP_ERR_NO_ERROR;
	uint8_t user_lid = GATT_INVALID_USER_LID;
   extern uint8_t gatvs_uuid[];
	do
	{
		gatvs_env_t* p_gatvs_env;
		//uint8_t cfg_flag = 0x1F;

		if ((p_params == NULL) || (p_start_hdl == NULL) || p_params->cb_get == NULL || p_params->cb_set == NULL)
		{
			status = GAP_ERR_INVALID_PARAM;
			break;
		}


		// register  user
		status = gatt_user_srv_register(GATVS_VAL_MAX_LEN, user_prio, &gatvs_cb, &user_lid);
		if (status != GAP_ERR_NO_ERROR) break;


		// Add GAP service
		dbg_print("gatvs_init %08x %x,sec_lvl:%x\r\n", p_params->att_table, p_params->att_table_size,sec_lvl);
		//sec_lvl = 0x40;
		status = gatt_db_svc_add(user_lid, sec_lvl, gatvs_uuid, p_params->att_table_size,
			/*(uint8_t *)&cfg_flag*/NULL, &p_params->att_table[0], p_params->att_table_size, p_start_hdl);


		if (status != GAP_ERR_NO_ERROR) break;

		//-------------------- allocate memory required for the profile  ---------------------
		p_gatvs_env = (gatvs_env_t *)ke_malloc_user(sizeof(gatvs_env_t), KE_MEM_PROFILE);

		if (p_gatvs_env != NULL)
		{
			// allocate HIDS required environment variable
			p_env->p_env = (prf_hdr_t *)p_gatvs_env;
			p_gatvs_env->start_hdl = *p_start_hdl;
			p_gatvs_env->user_lid = user_lid;


			// initialize profile environment variable
			p_gatvs_env->prf_env.p_cb = p_cb;
#if (HOST_MSG_API)
			p_env->desc.msg_handler_tab = NULL;
			p_env->desc.msg_cnt = 0;
#endif // (HOST_MSG_API)

			// 记录应用回调
			p_gatvs_env->cb_get = p_params->cb_get;
			p_gatvs_env->cb_set = p_params->cb_set;
		}
		else
		{
			status = GAP_ERR_INSUFF_RESOURCES;
		}

	} while (0);

	if ((status != GAP_ERR_NO_ERROR) && (user_lid != GATT_INVALID_USER_LID))
	{
		gatt_user_unregister(user_lid);
	}

	return (status);
}

/**
 ****************************************************************************************
 * @brief Destruction of the profile module - due to a reset or profile remove.
 *
 * This function clean-up allocated memory.
 *
 * @param[in,out]    p_env        Collector or Service allocated environment data.
 * @param[in]        reason       Destroy reason (see enum #prf_destroy_reason)
 *
 * @return status of the destruction, if fails, profile considered not removed.
 ****************************************************************************************
 */
__STATIC uint16_t gatvs_destroy(prf_data_t *p_env, uint8_t reason)
{
	uint16_t status = GAP_ERR_NO_ERROR;
	gatvs_env_t *p_gatvs_env = (gatvs_env_t *)p_env->p_env;
	dbg_print("gatvs_destroy\r\n");

	if (reason != PRF_DESTROY_RESET)
	{
		status = gatt_user_unregister(p_gatvs_env->user_lid);
	}

	if (status == GAP_ERR_NO_ERROR)
	{
		// free profile environment variables
		p_env->p_env = NULL;
		ke_free(p_gatvs_env);
	}

	return (status);
}

/**
 ****************************************************************************************
 * @brief @brief Handles Connection creation
 *
 * @param[in,out]    env          Collector or Service allocated environment data.
 * @param[in]        conidx       Connection index
 * @param[in]        is_le_con    True if it's a BLE connection, False if it's a BT-Classic connection
 ****************************************************************************************
 */
__STATIC void gatvs_con_create(prf_data_t *p_env, uint8_t conidx, bool is_le_con)
{
	dbg_print("gatvs_con_create\r\n");

	// hids_env_t *p_hids_env = (hids_env_t *)p_env->p_env;
	ASSERT_ERR(conidx < HOST_CONNECTION_MAX);
}

/**
 ****************************************************************************************
 * @brief Handles Disconnection
 *
 * @param[in,out]    p_env      Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 * @param[in]        reason     Detach reason
 ****************************************************************************************
 */
__STATIC void gatvs_con_cleanup(prf_data_t *p_env, uint8_t conidx, uint16_t reason)
{
   dbg_print("gatvs_con_cleanup\r\n");
	// hids_env_t *p_hids_env = (hids_env_t *)p_env->p_env;
	ASSERT_ERR(conidx < HOST_CONNECTION_MAX);
}



/// HIDS Task interface required by profile manager
const prf_task_cbs_t gatvs_itf =
{
	.cb_init = (prf_init_cb)gatvs_init,
	.cb_destroy = gatvs_destroy,
	.cb_con_create = gatvs_con_create,
	.cb_con_cleanup = gatvs_con_cleanup,
};

/**
 ****************************************************************************************
 * @brief Retrieve service profile interface
 *
 * @return service profile interface
 ****************************************************************************************
 */
const prf_task_cbs_t *gatvs_prf_itf_get(void)
{
	dbg_print("gatvs_prf_itf_get\r\n");

	return &gatvs_itf;
}


#endif 


