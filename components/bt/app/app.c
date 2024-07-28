/**
 ****************************************************************************************
 *
 * @file app.c
 *
 * @brief Application entry point
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"             // SW configuration
#include "rwapp_config.h"

#include "rwhl_config.h"
#if (APP_PRESENT)

#include <string.h>

#include "app_task.h"                // Application task Definition
#include "app.h"                     // Application Definition
#include "gap.h"                     // GAP Definition

#include "gapc.h"
#include "gapm.h"
#include "gapc_msg.h"
#include "gapm_msg.h"
#include "ke_msg.h"
#include "ke_event.h"

//add by liujin
#include "gapm_le_adv.h"            
#include "gapm_le_msg.h"
#include "system_ms.h"

//add end
#include "co_math.h"                 // Common Maths Definition

#if (BLE_APP_SEC)
#include "app_sec.h"                 // Application security Definition
#endif // (BLE_APP_SEC)

#if (BLE_APP_HT)
#include "app_ht.h"                  // Health Thermometer Application Definitions
#endif //(BLE_APP_HT)

#if (BLE_APP_DIS)
#include "app_dis.h"                 // Device Information Service Application Definitions
#endif //(BLE_APP_DIS)

#if (BLE_APP_BATT)
#include "app_batt.h"                // Battery Application Definitions
#endif //(BLE_APP_DIS)

#if (BLE_APP_HID)
#include "app_hid.h"                 // HID Application Definitions
#endif //(BLE_APP_HID)
#if (BLE_APP_GATV)
#include "app_gatv.h"         
#endif

#if (DISPLAY_SUPPORT)
#include "app_display.h"             // Application Display Definition
#endif //(DISPLAY_SUPPORT)

#if (BLE_APP_AM0)
#include "app_am0.h"                 // Audio Mode 0 Application
#endif //(BLE_APP_AM0)

#if (BLE_APP_MESH)
#include "app_mesh.h"                 // HID Application Definitions
#endif //(BLE_APP_MESH)

#include "atiny_log.h"
#include <stdio.h>
#include <string.h>  
#include "ms_keyscan.h"

#if (NVDS_SUPPORT)
#include "nvds.h"           // NVDS API Definitions
#endif //(NVDS_SUPPORT)

#include "ms1008.h"
#include "log.h"
#include "ms_sys_ctrl_regs.h"
#include "ms_interrupt.h"
#include "pm_imp.h"
#include "ir1.h"



/*
 * DEFINES
 ****************************************************************************************
 */

/// Default Device Name
#define APP_DFLT_DEVICE_NAME            ("TCL_RC8092")  //liujn modify device name
#define APP_DFLT_DEVICE_NAME_LEN        (sizeof(APP_DFLT_DEVICE_NAME))  

#if (BLE_APP_AM0)
#define DEVICE_NAME        "Ceva Hearing Aid"
#elif (BLE_APP_HID)
// HID Mouse
#define DEVICE_NAME        "Hid Mouse"
#else
#define DEVICE_NAME        "TCL DEVICE"
#endif

#define DEVICE_NAME_SIZE    sizeof(DEVICE_NAME)



/**
 * Default Scan response data
 * --------------------------------------------------------------------------------------
 * x09                             - Length
 * xFF                             - Vendor specific advertising type
 * x00\x60\x52\x57\x2D\x42\x4C\x45 - "RW-BLE"
 * --------------------------------------------------------------------------------------
 */
//#define APP_SCNRSP_DATA         "\x09\xFF\x00\x60\x52\x57\x2D\x42\x4C\x45"
//#define APP_SCNRSP_DATA_LEN     (10)

//#define APP_SCNRSP_DATA         "\x03\x19\xC1\x03"
#define APP_SCNRSP_DATA         "\x03\x19\x80\x01"

#define APP_SCNRSP_DATA_LEN     (4)



/**
 * Advertising Parameters
 */
#if (BLE_APP_HID)
/// Default Advertising duration - 30s (in multiple of 10ms)
#define APP_DFLT_ADV_DURATION   (3000)
#endif //(BLE_APP_HID)
/// Advertising channel map - 37, 38, 39
#define APP_ADV_CHMAP           (0x07)
/// Advertising minimum interval - 40ms (64*0.625ms)
#define APP_ADV_INT_MIN         (64)
/// Advertising maximum interval - 40ms (64*0.625ms)
#define APP_ADV_INT_MAX         (64)
/// Fast advertising interval
#define APP_ADV_FAST_INT        (32)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

typedef void (*app_add_svc_func_t)(void);

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// List of service to add in the database
enum app_svc_list
{
    #if (BLE_APP_HT)
    APP_SVC_HTS,
    #endif //(BLE_APP_HT)
    #if (BLE_APP_DIS)
    APP_SVC_DIS,
    #endif //(BLE_APP_DIS)
    #if (BLE_APP_BATT)
    APP_SVC_BATT,
    #endif //(BLE_APP_BATT)
    #if (BLE_APP_HID)
    APP_SVC_HIDS,
    #endif //(BLE_APP_HID)
    #if (BLE_APP_AM0)
    APP_SVC_AM0_HAS,
    #endif //(BLE_APP_AM0)
    #if (BLE_APP_MESH)
    APP_SVC_MESH,
    #endif //(BLE_APP_MESH)

    APP_SVC_LIST_STOP,
};


/*
 * LOCAL VARIABLES DEFINITIONS
 ****************************************************************************************
 */

/// Application Task Descriptor
extern const struct ke_task_desc TASK_DESC_APP;

uint8_t peer_dbaddr[APP_BD_ADDR_LEN] = {0};
bonded_dev_info_list_t bonded_dev_info = {0};
static uint8_t gRepeatTimes = 0xFF;
static uint8_t last_cmd;
static bool is_ir_pair = false;     //是否在IR配对流程中
static bool is_recon_flag = false; //是否已经在发广播重连

uint8_t last_adv_type = 0xff; //0 normal adv, 1 direct adv

uint8_t direct_adv_idx = 0xff;
uint8_t normal_adv_idx = 0xff;


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Application Environment Structure
struct app_env_tag app_env;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
#if(APP_A2DP)
extern void a2dp_app_start();
#endif // (APP_A2DP)

#if (BLE_APP_PRF)
#if 0
static void app_build_adv_data(uint16_t max_length, uint16_t *p_length, uint8_t *p_buf)
{
    // Remaining Length
    uint8_t rem_len = max_length;

    #if (BLE_APP_HT)
    // Set list of UUIDs
    memcpy(p_buf, APP_HT_ADV_DATA_UUID, APP_HT_ADV_DATA_UUID_LEN);
    *p_length += APP_HT_ADV_DATA_UUID_LEN;
    p_buf += APP_HT_ADV_DATA_UUID_LEN;

    // Set appearance
    memcpy(p_buf, APP_HT_ADV_DATA_APPEARANCE, APP_ADV_DATA_APPEARANCE_LEN);
    *p_length += APP_ADV_DATA_APPEARANCE_LEN;
    p_buf += APP_ADV_DATA_APPEARANCE_LEN;
    #endif //(BLE_APP_HT)

    #if (BLE_APP_HID)
    // Set list of UUIDs
    memcpy(p_buf, APP_HID_ADV_DATA_UUID, APP_HID_ADV_DATA_UUID_LEN);
    *p_length += APP_HID_ADV_DATA_UUID_LEN;
    p_buf += APP_HID_ADV_DATA_UUID_LEN;

    // Set appearance
    memcpy(p_buf, APP_HID_ADV_DATA_APPEARANCE, APP_ADV_DATA_APPEARANCE_LEN);
    *p_length += APP_ADV_DATA_APPEARANCE_LEN;
    p_buf += APP_ADV_DATA_APPEARANCE_LEN;
    #endif //(BLE_APP_HID)

    // Sanity check
    ASSERT_ERR(rem_len >= max_length);

    // Get remaining space in the Advertising Data - 2 bytes are used for name length/flag
    rem_len -= *p_length;

    // Check if additional data can be added to the Advertising data - 2 bytes needed for type and length
    if (rem_len > 2)
    {
        uint8_t dev_name_length = co_min(app_env.dev_name_len, (rem_len - 2));

        // Device name length
        *p_buf = dev_name_length + 1;
        // Device name flag (check if device name is complete or not)
        *(p_buf + 1) = (dev_name_length == app_env.dev_name_len) ? '\x09' : '\x08';
        // Copy device name
        memcpy(p_buf + 2, app_env.dev_name, dev_name_length);

        // Update advertising data length
        *p_length += (dev_name_length + 2);
    }
}
#endif //(!BLE_APP_AM0)

static void app_start_advertising(uint8_t actv_idx)
   {
  
   dbg_print2("app_start_advertising:act_idx:%x,%x,dirct:%x,nor:%x\r\n",app_env.adv_actv_idx,actv_idx,direct_adv_idx,normal_adv_idx);
   if (0xff != actv_idx)
      {
      
      
      struct gapm_activity_start_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_START_CMD,
                                                     TASK_GAPM, TASK_APP,
                                                     gapm_activity_start_cmd);
      
      
      gapm_adv_param_t * padv_para;
      
      
      p_cmd->operation = GAPM_START_ACTIVITY;
      p_cmd->actv_idx = actv_idx;//app_env.adv_actv_idx;
      //p_cmd->actv_idx = GAPM_ACTV_TYPE_ADV;
      
      padv_para = (gapm_adv_param_t*) p_cmd->u_param;
      padv_para->duration = 3000;
      padv_para->max_adv_evt = 0;
      
      // Send the message
      ke_msg_send(p_cmd);
      
      // Keep the current operation
      app_env.adv_state = APP_ADV_STATE_STARTING;
      // And the next expected operation code for the command completed event
      app_env.adv_op = GAPM_START_ACTIVITY;
      }
   else
   	{
   	app_env.adv_state = APP_ADV_STATE_IDLE;
      app_adv_fsm_next();	
   	}
   }


void app_stop_advertising(uint8_t adv_idx)
{
    if ( app_env.adv_state >= APP_ADV_STATE_STARTING )
       {
       // Prepare the GAPM_ACTIVITY_STOP_CMD message
       struct gapm_activity_stop_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_STOP_CMD,
                                                 TASK_GAPM, TASK_APP,
                                                 gapm_activity_stop_cmd);
       
       // Fill the allocated kernel message
       p_cmd->operation = GAPM_STOP_ACTIVITY;
       p_cmd->actv_idx = adv_idx;//app_env.adv_actv_idx;
       
       // Send the message
       ke_msg_send(p_cmd);
       dbg_print2("app_stop_advertising,adv_state:%x,adv_idx%x\r\n",app_env.adv_state,adv_idx);
       // Update advertising state
       app_env.adv_state = APP_ADV_STATE_CREATED;
       // And the next expected operation code for the command completed event
       app_env.adv_op = GAPM_STOP_ACTIVITY;
       
       }

}


static void app_set_adv_data(void)
{

    uint8_t advData[] = { //Advertising data format
            // 必须满足   XXX_RCXXX 的格式，否则不能快捷配对
            // _之前为品牌缩写，RC之后为遥控器型号
            // 不能超过16个字符
            11, GAP_AD_TYPE_COMPLETE_NAME, 'T', 'C', 'L', '_', 'R', 'C', '8', '0', '9', '2',

            //2, GAP_AD_TYPE_FLAGS, 0x06,
            5, GAP_AD_TYPE_MANU_SPECIFIC_DATA, 0x66, 0x79, 0x30, 0x01,
            2, GAP_AD_TYPE_TRANSMIT_POWER, 0x03,
            //3, GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID, 0x12, 0x18,   // 0x1812 Human Interface Device
            3, GAP_AD_TYPE_APPEARANCE, 0x80, 0x01        //0x03C1: HID Keyboard
    };


    // Prepare the GAPM_SET_ADV_DATA_CMD message
    struct gapm_set_adv_data_cmd *p_cmd = KE_MSG_ALLOC_DYN(GAPM_SET_ADV_DATA_CMD,
                                                           TASK_GAPM, TASK_APP,
                                                           gapm_set_adv_data_cmd,
                                                           ADV_DATA_LEN);

    // Fill the allocated kernel message
    p_cmd->operation = GAPM_SET_ADV_DATA;
    p_cmd->actv_idx = app_env.adv_actv_idx;//;

    //p_cmd->length = 0;
    // GAP will use 3 bytes for the AD Type
    //app_build_adv_data(ADV_DATA_LEN - 3, &p_cmd->length, &p_cmd->data[0]);
    p_cmd->length = sizeof(advData);
	 memcpy(&p_cmd->data[0], &advData[0],  sizeof(advData));
    dbg_print2("app_set_adv_data,length:%x\r\n----------------",p_cmd->length);
	 {
	 uint8_t i;
	 for (i=0;i<p_cmd->length;i++)
	 	{
		
	 	dbg_print2("%x,",p_cmd->data[i]);
	 	}
	 dbg_print2("\r\n----------------\r\n");
    }
	 
    // Send the message
    ke_msg_send(p_cmd);

    // Update advertising state
    app_env.adv_state = APP_ADV_STATE_SETTING_ADV_DATA;
    // And the next expected operation code for the command completed event
    app_env.adv_op = GAPM_SET_ADV_DATA;
}

static void app_set_scan_rsp_data(void)
{
    // Prepare the GAPM_SET_ADV_DATA_CMD message
    struct gapm_set_adv_data_cmd *p_cmd = KE_MSG_ALLOC_DYN(GAPM_SET_ADV_DATA_CMD,
                                                           TASK_GAPM, TASK_APP,
                                                           gapm_set_adv_data_cmd,
                                                           ADV_DATA_LEN);

    // Fill the allocated kernel message
    p_cmd->operation = GAPM_SET_SCAN_RSP_DATA;
    p_cmd->actv_idx = app_env.adv_actv_idx;

	

    #if (BLE_APP_AM0)
    app_am0_fill_scan_rsp_data(p_cmd);
    #else //(BLE_APP_AM0)
    p_cmd->length = APP_SCNRSP_DATA_LEN;
    memcpy(&p_cmd->data[0], APP_SCNRSP_DATA, APP_SCNRSP_DATA_LEN);
    #endif //(BLE_APP_AM0)
	 dbg_print2("app_set_scan_rsp_data,length:%x\r\n------------",p_cmd->length);

   {
	uint8_t i;
	for (i=0;i<p_cmd->length;i++)
   	{
 
 		dbg_print2("%x,",p_cmd->data[i]);
 		}
	dbg_print2("\r\n----------------\r\n");
	}

    // Send the message
    ke_msg_send(p_cmd);

    // Update advertising state
    app_env.adv_state = APP_ADV_STATE_SETTING_SCAN_RSP_DATA;
    // And the next expected operation code for the command completed event
    app_env.adv_op = GAPM_SET_SCAN_RSP_DATA;
	 dbg_print2("\r\n--------%x,%x--------\r\n",app_env.adv_state,app_env.adv_op);

	 
}
#endif //(BLE_APP_PRF)

// Set device configuration
__STATIC void app_set_dev_cfg(void)
{
    #if (NVDS_SUPPORT)
    nvds_tag_len_t len;
    #endif //(NVDS_SUPPORT)
    //gapm_config_t dev_cfg;

    // Prepare the GAPM_SET_DEV_CONFIG_CMD message
    struct gapm_set_dev_config_cmd *p_cmd = KE_MSG_ALLOC(GAPM_SET_DEV_CONFIG_CMD,
                                                           TASK_GAPM, TASK_APP,
                                                           gapm_set_dev_config_cmd
                                                           );

    p_cmd->operation = GAPM_SET_DEV_CONFIG;
    


	 dbg_print2("app_set_dev_cfg\r\n");


    // Set the operation
    #if (BLE_APP_PRF)
    // Set the device role - Peripheral
    p_cmd->cfg.role = GAP_ROLE_PERIPHERAL;
    #elif (BLE_APP_MESH)
    p_cmd->cfg.role = GAP_ROLE_ALL;
    #elif (BT_HOST_PRESENT)
    p_cmd->cfg.role = GAP_ROLE_BT_CLASSIC;
    #else
    p_cmd->cfg.role = GAP_ROLE_ALL;
    #endif //(BLE_APP_PRF)
	 p_cmd->cfg.role = GAP_ROLE_PERIPHERAL;

    #if (BLE_APP_SEC_CON)
    // The Max MTU is increased to support the Public Key exchange
    // HOWEVER, with secure connections enabled you cannot sniff the
    // LEAP and LEAS protocols
    p_cmd->cfg.pairing_mode = GAPM_PAIRING_SEC_CON | GAPM_PAIRING_LEGACY;
    #else // !(BLE_APP_SEC_CON)
    // Do not support secure connections
    p_cmd->cfg.pairing_mode = GAPM_PAIRING_LEGACY;
    #endif //(BLE_APP_SEC_CON)

    // Set Data length parameters
   // dev_cfg.sugg_max_tx_octets = GAP_BLE_MIN_OCTETS;
   // dev_cfg.sugg_max_tx_time   = GAP_BLE_MIN_TIME;
   //modified by liujin
    p_cmd->cfg.sugg_max_tx_octets = GAP_BLE_MAX_OCTETS;
    p_cmd->cfg.sugg_max_tx_time   = GAP_BLE_MAX_TIME;

	 

    #if (BLE_APP_HID)
    // Enable Slave Preferred Connection Parameters present
    p_cmd->cfg.att_cfg = 0;
    //SETF(p_cmd->cfg.att_cfg, GAPM_ATT_SLV_PREF_CON_PAR_EN, 1); //GAPM_ATT_SLV_PREF_CON_PAR_EN_MASK
    #endif //(BLE_APP_HID)

    // Host privacy enabled by default
    p_cmd->cfg.privacy_cfg = 0;


    #if (NVDS_SUPPORT)
    if (nvds_get(NVDS_TAG_BD_ADDRESS, &len, &p_cmd->cfg.addr.addr[0]) == NVDS_OK)
    {
        // Check if address is a static random address
        if (p_cmd->cfg.addr.addr[5] & 0xC0)
        {
            // Host privacy enabled by default
            p_cmd->cfg.privacy_cfg |= GAPM_PRIV_CFG_PRIV_ADDR_BIT;
        }
    }
    #endif //(NVDS_SUPPORT)

    #if 0  //(NVDS_SUPPORT)
    #if (BLE_APP_PRF)
    if ((app_sec_get_bond_status()==true) &&
        (nvds_get(NVDS_TAG_LOC_IRK, &key_len, app_env.loc_irk) == NVDS_OK))
    {
        memcpy(p_cmd->cfg.irk.key, app_env.loc_irk, 16);
    }
    else
    #elif (BLE_APP_MESH)
    if (nvds_get(NVDS_TAG_LOC_IRK, &key_len, app_env.loc_irk) == NVDS_OK)
    {
        memcpy(p_cmd->cfg.irk.key, app_env.loc_irk, 16);
    }
    #endif // (BLE_APP_PRF)
    #endif //(NVDS_SUPPORT)
    {
        memset((void *)&p_cmd->cfg.irk.key[0], 0x00, KEY_LEN);
    }

    #if (BLE_APP_MESH)
    // Set the duration before regenerate device address. - in seconds
    p_cmd->cfg.renew_dur = 0x0096;
    p_cmd->cfg.gap_start_hdl = 0;
    p_cmd->cfg.gatt_start_hdl = 0;
    p_cmd->cfg.att_cfg = 0x0080;
    p_cmd->cfg.max_mtu = 0x02A0;
    p_cmd->cfg.max_mps = 0x02A0;
    p_cmd->cfg.max_nb_lecb = 0x0A;
    //p_cmd->cfg.Pad = 0x00;
    p_cmd->cfg.tx_pref_phy = 0;
    p_cmd->cfg.rx_pref_phy = 0;
    p_cmd->cfg.tx_path_comp = 0;
    p_cmd->cfg.rx_path_comp = 0;
    #elif (APP_A2DP)
    // Set the duration before regenerate device address. - in seconds
    p_cmd->cfg.renew_dur = 0x0096;
    p_cmd->cfg.gap_start_hdl = 0;
    p_cmd->cfg.gatt_start_hdl = 0;
    p_cmd->cfg.att_cfg = 0x0080;
    //p_cmd->cfg.Pad = 0x00;
    p_cmd->cfg.tx_pref_phy = 0;
    p_cmd->cfg.rx_pref_phy = 0;
    p_cmd->cfg.tx_path_comp = 0;
    p_cmd->cfg.rx_path_comp = 0;
    #else
   //add by liujin
    p_cmd->cfg.renew_dur = 1500;
	 
    memset(&(p_cmd->cfg.addr)     , 0x00, sizeof(gap_addr_t)); // Use public address - no private static
    memset(&(p_cmd->cfg.irk)      , 0x00, KEY_LEN);
	 
    p_cmd->cfg.gap_start_hdl = 0;
    p_cmd->cfg.gatt_start_hdl = 0;	 
    p_cmd->cfg.tx_path_comp       = 0;
    p_cmd->cfg.rx_path_comp       = 0;  
	 p_cmd->cfg.tx_pref_phy        = GAP_PHY_LE_1MBPS;//GAP_PHY_ANY;
    p_cmd->cfg.rx_pref_phy        = GAP_PHY_LE_1MBPS;// GAP_PHY_ANY; 
    p_cmd->cfg.class_of_device    = 0;
    p_cmd->cfg.dflt_link_policy   = 0;
    p_cmd->cfg.ssp_enable         = true;	 
	 //add end
    #endif //(BLE_APP_MESH)

    #if (BT_HOST_PRESENT)
    p_cmd->cfg.class_of_device  = 0x200408;
	 
    p_cmd->cfg.dflt_link_policy = GAPM_ROLE_SWITCH_ENABLE_BIT | GAPM_HOLD_MODE_ENABLE_BIT | GAPM_SNIFF_MODE_ENABLE_BIT;
    p_cmd->cfg.ssp_enable       = true;
    #endif // (BT_HOST_PRESENT)

   // gapm_set_dev_config(0, &p_cmd->cfg, app_set_dev_cfg_cmp_handler);
	// gapm_set_dev_config(event, &dev_cfg, &app_gap_cbs, cmp_cb);

    // Send the message
    ke_msg_send(p_cmd);

    // Update advertising state
    app_env.adv_state = APP_ADV_STATE_IDLE;
    // And the next expected operation code for the command completed event
   // app_env.adv_op = GAPM_SET_ADV_DATA;

	 
}



/// Device boot properly perform, entry point of application
__STATIC void app_start(void)
{
    int8_t intvalue;
		
    dbg_print2("app_start\r\n");

#ifndef WIN32 // peterlee
	MS_LOGI(MS_DRIVER, "sys ble regiser %x\r\n", SYS_CTRL->BLE_CTRL);
	MS_LOGI(MS_DRIVER, "sys clockdiv6 regiser %x\r\n", SYS_CTRL->CLK_DIV6);
	MS_LOGI(MS_DRIVER, "sys lp keep regiser %x\r\n", SYS_CTRL->LP_KEEP);
    MS_LOGI(MS_DRIVER, "sys clock DIV_TOG %x\r\n", SYS_CTRL->DIV_TOG);
#endif

    intvalue = INTERRUPT_GET_ENABLE_IRQ (KEYSCAN_IRQn);
    MS_LOGI(MS_DRIVER, "KEYSCAN_IRQn %d\r\n", intvalue);

    intvalue = INTERRUPT_GET_ENABLE_IRQ (BLE_IRQn);
    MS_LOGI(MS_DRIVER, "BLE_IRQn %d\r\n", intvalue);
	


    ke_event_clear(APP_EVENT);
    // Set Device configuration
    app_set_dev_cfg();
}

#ifdef PM_DEBUG_BLE_DEEP_SLEEP
extern ms_sleep_global_data g_sleep_data;
#endif



 void app_keypad_report_key(uint32_t key_value, uint32_t type)
{

#ifdef PM_DEBUG_BLE_DEEP_SLEEP
    g_sleep_data.is_allowed_to_enter_sleep = 0; 
    dsleep_timer_restart();  // restart timer, the time is to be decided
#endif

    // Allocate the BASS_CREATE_DB_REQ
    struct app_keypad_db_cfg *req = KE_MSG_ALLOC(APP_KEYPAD_MSG,
                            TASK_APP, TASK_NONE,
                            app_keypad_db_cfg);

    req->key_value= key_value;
    req->press = type;
   // ATINY_LOG(LOG_INFO,"app_keypad_report keyvalue=%x, press=%d!!",key_value, type);	

    // Send the message
    ke_msg_send(req);
}

void app_start_pair(void)
	{
	 is_ir_pair = true;
    co_time_timer_set(&(app_env.app_pair_timer), 900); 
	}

void app_stop_pair_timer(void)
	{
    co_time_timer_stop(&(app_env.app_pair_timer)); 
	}


void app_start_pair_handler(void* p_env)
   {
	
	//如果之前已经创建非定向广播，则不用重新创建
	//if ((1 == last_adv_type )||(app_env.adv_state != APP_ADV_STATE_STARTED))
	if (1 == last_adv_type )
		{
		app_stop_advertising(direct_adv_idx);
		//app_delete_advertising();
		}
	else
		{
		app_stop_advertising(normal_adv_idx);
		
		}
	if (0xff == normal_adv_idx)
		{
		//创建非定向广播
		dbg_print("create normal adv\r\n");
		app_env.adv_state = APP_ADV_STATE_IDLE;
		}
  // app_adv_fsm_next();	
	//app_ir_send_code_and_repert(0xA0, 3);		  // 触发电视扫描
   //停止adv后，100ms后再启动adv,否则可能会启动失败
	co_time_timer_set(&(app_env.app_adv_timer), 100); 
   
   }

void app_adv_timer_handler(void* p_env)
   {
   app_adv_fsm_next();	
	app_ir_send_code_and_repert(0xA0, 3);		  // 触发电视扫描


   }


void app_ir_start_repeat(void)
{
    co_time_timer_set(&(app_env.IR_repeat_timer), 8); 
}

void app_ir_stop_repeat(void)
{
    co_time_timer_stop(&(app_env.IR_repeat_timer)); 
}


/* @function app_ir_send_code_and_repert
 * @brief 发送IR红外码
 * @param code -- 键值
 * @param times -- 发送IR码次数，times为0xFF表示不计次数
 * @note 本函数仅适配RCA协议
 */
void app_ir_send_code_and_repert(uint8_t code, uint8_t times)
{
    // 如果发送红外码未定义，则不做处理
    last_cmd = code;
    gRepeatTimes = times+1;
    // 发送一次红外码
    //ir_rca_send_frame(0, code);
    // 定时发送重复码
    app_ir_start_repeat();
}

void app_IR_repeat_handler(void* p_env)
   {
	
	 if (gRepeatTimes == 0xFF || --gRepeatTimes)
       {
        
       
       dbg_print("APP IR: Send IR Code:%x\r\n",last_cmd);
       
       #ifdef IR1_TO2
       ir_rca_send_frame(0, last_cmd);
       #endif
       app_ir_start_repeat();
       }
	 else
       {
       // 若计数完成，则停止红外发送
       app_ir_stop_repeat();
		 is_ir_pair = false;
       }

   
   }

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void app_init()
{

    dbg_print2("app_init\r\n");
    // Reset the application manager environment
    memset(&app_env, 0, sizeof(app_env));

    // Create APP task
    ke_task_create(TASK_APP, &TASK_DESC_APP);

    // Initialize Task state
    ke_state_set(TASK_APP, APP_INIT);

    #if (BLE_APP_MESH)
    // Get the mesh demo type from NVDS
    #if (NVDS_SUPPORT)
    app_env.demo_type_len = APP_MESH_DEMO_TYPE_LEN;
    if (nvds_get(NVDS_TAG_MESH_DEMO_TYPE, &(app_env.demo_type_len), &(app_env.demo_type)) != NVDS_OK)
    #endif //(NVDS_SUPPORT)
    {
        app_env.demo_type = APP_MESH_DEMO_GENS_ONOFF;
    }
    #endif //(BLE_APP_MESH)

    #if (NVDS_SUPPORT)
    // Get the Device Name to add in the Advertising Data (Default one or NVDS one)
    app_env.dev_name_len = APP_DEVICE_NAME_MAX_LEN;
    if (nvds_get(NVDS_TAG_DEVICE_NAME, &(app_env.dev_name_len), app_env.dev_name) != NVDS_OK)
    #endif //(NVDS_SUPPORT)
    {
#if (BLE_APP_AM0)
        // Get default Device Name (No name if not enough space)
        memcpy(app_env.dev_name, DEVICE_NAME, DEVICE_NAME_SIZE);
        app_env.dev_name_len = DEVICE_NAME_SIZE;
#else
        // Get default Device Name (No name if not enough space)
        memcpy(app_env.dev_name, APP_DFLT_DEVICE_NAME, APP_DFLT_DEVICE_NAME_LEN);
        app_env.dev_name_len = APP_DFLT_DEVICE_NAME_LEN;
#endif
        // TODO update this value per profiles
    }

    /*------------------------------------------------------
     * INITIALIZE ALL MODULES
     *------------------------------------------------------*/

    // load device information:
    #if (DISPLAY_SUPPORT)
    // Pass the device name and mesh demo type to app display initialization function
    app_display_init(app_env.dev_name, app_env.demo_type);
    #endif //(DISPLAY_SUPPORT)

    #if (BLE_APP_HT)
    // Health Thermometer Module
    app_ht_init();
    #endif //(BLE_APP_HT)

    #if (BLE_APP_DIS)
    // Device Information Module
    app_dis_init();
    #endif //(BLE_APP_DIS)

    #if (BLE_APP_HID)
    app_hid_init();
    #endif //(BLE_APP_HID)
	 #if (BLE_APP_GATV)
    app_gatv_init();
	 #endif //(BLE_APP_HID)

    #if (BLE_APP_BATT)
    // Battery Module
    app_batt_init();
    #endif //(BLE_APP_BATT)

    #if (BLE_APP_AM0)
    // Audio Mode 0 Module
    app_am0_init();
    #endif //(BLE_APP_AM0)

    #if (BLE_APP_MESH)
    // Mesh Module
    app_mesh_init(app_env.demo_type);
    #endif //(BLE_APP_MESH)
     //app_keypad_msg_init();
   // ms_keyscan_init(app_keypad_report_key);
#ifdef PM_DEBUG_BLE_DEEP_SLEEP
	dsleep_timer_start();
#endif
	app_sec_init();

    ke_event_callback_set(APP_EVENT, app_start);
    ke_event_set(APP_EVENT);
	 
	 co_time_timer_init(&(app_env.IR_repeat_timer), app_IR_repeat_handler, NULL);
	 co_time_timer_init(&(app_env.app_pair_timer), app_start_pair_handler, NULL);
	 co_time_timer_init(&(app_env.app_recon_timer), app_start_reconnet_handler, NULL);
	 co_time_timer_init(&(app_env.app_adv_timer), app_adv_timer_handler, NULL);
}

uint8_t app_get_dev_name(uint8_t* name)
{
    // copy name to provided pointer
    memcpy(name, app_env.dev_name, app_env.dev_name_len);
    // return name length
    return app_env.dev_name_len;
}



int8_t app_check_device_isbonded(const uint16_t in_ediv, const uint8_t *in_nb)
{
    
    //check the latest device first
    for (int i = bonded_dev_info.current_dev_index; i < MAX_BONDED_DEV_NUM; i++)
    {
 //liujin del for test   
        if (in_ediv == bonded_dev_info.bonded_device_info[i].ltk.ediv &&
                       !memcmp(&in_nb[0], bonded_dev_info.bonded_device_info[i].ltk.randnb, RAND_NB_LEN))
        {
          
            return i;
        }
    }
    for (int i = 0; i < bonded_dev_info.current_dev_index; i++)
    {
        if (in_ediv == bonded_dev_info.bonded_device_info[i].ltk.ediv &&
                       !memcmp(&in_nb[0], bonded_dev_info.bonded_device_info[i].ltk.randnb, RAND_NB_LEN))
        {
            
            return i;
        }
    }
    return INVALID_BONDED_INDEX;
}


/* @function app_reconnect
 * @brief 对已经绑定过的设备可以进行回连
 * @param none
 * @return
 */
void app_reconnect(void)
	{
    dbg_print2("bonded:%x,adv_state:%x,is_recon_flag:%x\r\n",app_sec_env.bonded,app_env.adv_state,is_recon_flag);
    if ((app_sec_get_bond_status())&&(!is_recon_flag)&&(!is_ir_pair)&&(( app_env.adv_state < APP_ADV_STATE_STARTING )))
       {
       dbg_print2("app_reconnect\r\n");
		 if (0 == last_adv_type )
			 {
			 app_stop_advertising(normal_adv_idx);
			 //app_delete_advertising();
			 }
	

   	 if (0xff == direct_adv_idx)
          {
          //创建定向广播
          dbg_print("create direct adv\r\n");
          app_env.adv_state = APP_ADV_STATE_IDLE;
          }
		 app_adv_fsm_next();
		 is_recon_flag = true;
		 co_time_timer_set(&(app_env.app_recon_timer), 3000); 
          }
	}

void app_start_reconnet_handler(void* p_env)
   {
   //app_stop_advertising();
   is_recon_flag = false;
   }



#if (BLE_APP_PRF)
void app_disconnect(void)
{
    struct gapc_disconnect_cmd *p_cmd = KE_MSG_ALLOC(GAPC_DISCONNECT_CMD, TASK_GAPC, TASK_APP, gapc_disconnect_cmd);

    p_cmd->conidx    = app_env.conidx;
    p_cmd->operation = GAPC_DISCONNECT;
    p_cmd->reason    = CO_ERROR_REMOTE_USER_TERM_CON;

    // Send the message
    ke_msg_send(p_cmd);
}

static void app_create_advertising(uint8_t directed)
	{
		
	dbg_print2("app_create_advertising,last_adv_type:%x\r\n",last_adv_type);

	last_adv_type = directed;
	if (app_env.adv_state == APP_ADV_STATE_IDLE)
		{
		  // Prepare the GAPM_ACTIVITY_CREATE_CMD message
		  struct gapm_activity_create_adv_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_CREATE_CMD, TASK_GAPM, TASK_APP,
																						gapm_activity_create_adv_cmd);
		
		
		  // Set operation code
		  p_cmd->operation = GAPM_CREATE_ADV_ACTIVITY;
		
		  // Fill the allocated kernel message
		  //modified  by liujin
		  p_cmd->own_addr_type = GAPM_STATIC_ADDR;  
		  //p_cmd->adv_param.type = GAPM_ADV_TYPE_LEGACY;
		  p_cmd->type = GAPM_ADV_TYPE_LEGACY;
		
		  
		  //GAPM_ADV_PROP_NON_CONN_NON_SCAN_MASK;//GAPM_ADV_PROP_UNDIR_CONN_MASK;
		  
		  p_cmd->adv_param.prim_cfg.chnl_map = APP_ADV_CHMAP; 
		  p_cmd->adv_param.prim_cfg.phy = GAPM_PHY_TYPE_LE_1M;//GAP_PHY_LE_1MBPS; 
		
   #if (BLE_APP_HID)

		
		  if(directed)
		  {
				dbg_print("###set hdc adversting\r\n");
		
				uint8_t length = NVDS_LEN_PEER_BD_ADDRESS;
				
				// Get bond status from NVDS
				if (nvds_get(NVDS_TAG_PEER_BD_ADDRESS, &length, (uint8_t *)&p_cmd->adv_param.peer_addr) == NVDS_OK)
					{
					dbg_print("read peer type:%x,peer_addr:",p_cmd->adv_param.peer_addr.addr_type);
					for (int i = GAP_BD_ADDR_LEN - 1; i >= 0; --i)
						 {
						 dbg_print("%02X ", p_cmd->adv_param.peer_addr.addr[i]);
						 }
					dbg_print(" \r\n");
		
					}
				else
					{
					dbg_print("\r\n read error \r\n");
					
					}
		
				p_cmd->adv_param.filter_pol = ADV_ALLOW_SCAN_WLST_CON_WLST;
				p_cmd->adv_param.disc_mode = GAPM_ADV_MODE_NON_DISC;
				p_cmd->adv_param.prop = GAPM_ADV_PROP_DIR_CONN_HDC_MASK;//GAPM_ADV_PROP_DIR_CONN_MASK;//GAPM_ADV_PROP_DIR_CONN_HDC_MASK;
				p_cmd->adv_param.prim_cfg.adv_intv_min = APP_ADV_FAST_INT;
				p_cmd->adv_param.prim_cfg.adv_intv_max = APP_ADV_FAST_INT;
		  }
		  else
		  {	dbg_print("#set normal adversting\r\n");
				p_cmd->adv_param.filter_pol = ADV_ALLOW_SCAN_ANY_CON_ANY;
				p_cmd->adv_param.disc_mode = GAPM_ADV_MODE_GEN_DISC;
				p_cmd->adv_param.prop = GAPM_ADV_PROP_UNDIR_CONN_MASK;
				p_cmd->adv_param.prim_cfg.adv_intv_min = APP_ADV_FAST_INT;
				p_cmd->adv_param.prim_cfg.adv_intv_max = APP_ADV_FAST_INT;
		  }
		  
   #else //(BLE_APP_HID)
		  p_cmd->adv_param.disc_mode = GAPM_ADV_MODE_GEN_DISC;  
		  p_cmd->adv_param.prim_cfg.adv_intv_min = APP_ADV_INT_MIN;
		  p_cmd->adv_param.prim_cfg.adv_intv_max = APP_ADV_INT_MAX;
   #endif //(BLE_APP_HID)
		
		
		  // Send the message
		  ke_msg_send(p_cmd);
		
		  // Keep the current operation
		  app_env.adv_state = APP_ADV_STATE_CREATING;
		  // And the next expected operation code for the command completed event
		  app_env.adv_op = GAPM_CREATE_ADV_ACTIVITY;
		}
	}


void app_delete_advertising(void)
   {
   
   if (APP_ADV_STATE_CREATED == app_env.adv_state )
      {
      
      
      struct gapm_activity_delete_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_DELETE_CMD,
                                                         TASK_GAPM, TASK_APP,
                                                         gapm_activity_delete_cmd);
      dbg_print2("app_delete_advertising:%d,is_ir_pair:%x,last_adv_type:%x\r\n",app_env.adv_state,is_ir_pair,last_adv_type);
      
      // Set operation code
      //p_cmd->operation = GAPM_DELETE_ALL_ACTIVITIES;
      p_cmd->operation = GAPM_DELETE_ACTIVITY;
      p_cmd->actv_idx = app_env.adv_actv_idx;
      
      // Send the message
      ke_msg_send(p_cmd);
      
      // Keep the current operation
      // And the next expected operation code for the command completed event
      app_env.adv_state = APP_ADV_STATE_IDLE;
      app_env.adv_op = GAPM_DELETE_ACTIVITY; //GAPM_DELETE_ALL_ACTIVITIES;
      
      }
   }

void app_adv_fsm_next(void)
{
    dbg_print2("app_adv_fsm_next:%d,is_ir_pair:%x\r\n",app_env.adv_state,is_ir_pair);
    switch (app_env.adv_state)
    {
        case (APP_ADV_STATE_IDLE):
        {
            // Create advertising
            if (is_ir_pair)
            	app_create_advertising(false);
				else
					app_create_advertising(app_sec_env.bonded);
        } break;

        case (APP_ADV_STATE_CREATING):
        {
            // Set advertising data
            #if (HL_LE_BROADCASTER)
            app_set_adv_data();
				#else
				
				#endif
        } break;

        case (APP_ADV_STATE_SETTING_ADV_DATA):
        {
            // Set scan response data
            app_set_scan_rsp_data();
        } break;

        case (APP_ADV_STATE_CREATED):
        case (APP_ADV_STATE_SETTING_SCAN_RSP_DATA):
        {
            // Start advertising activity
            
            if ((!is_ir_pair)&&(app_sec_env.bonded))
            	{
            	last_adv_type = 1;
            	app_start_advertising(direct_adv_idx);
            	}

				else
					{
					last_adv_type = 0;
					app_start_advertising(normal_adv_idx);
					}
#ifdef PM_DEBUG_BLE_DEEP_SLEEP
            dsleep_timer_restart();		
			MS_LOGI(MS_BLUETOOTH, "dsleep_timer_restart");
#endif
        } break;

        case (APP_ADV_STATE_STARTING):
        {
		  	   //extern void test_bleip_reg(void);
			   //test_bleip_reg();
            #if (DISPLAY_SUPPORT)
            // Update advertising state screen
            app_display_set_adv(true);
            #endif //(DISPLAY_SUPPORT)
				//app_start_advertising();

            // Go to started state
            app_env.adv_state = APP_ADV_STATE_STARTED;
        } break;

        case (APP_ADV_STATE_STARTED):
        {
            // Stop advertising activity
            //app_stop_advertising();
        } break;

        case (APP_ADV_STATE_STOPPING):
        {
            #if (DISPLAY_SUPPORT)
            // Update advertising state screen
            app_display_set_adv(false);
            #endif //(DISPLAY_SUPPORT)
				if (1 == last_adv_type )
					{
					app_stop_advertising(direct_adv_idx);
					//app_delete_advertising();
					}
				else
					{
					app_stop_advertising(normal_adv_idx);
					
					}


            // Go created state
            //app_env.adv_state = APP_ADV_STATE_CREATED;
        } break;

        default:
        {
            ASSERT_ERR(0);
        } break;
    }
}

void app_update_param(struct gapc_conn_param *p_conn_param)
{
    // Prepare the GAPC_PARAM_UPDATE_CMD message
    struct gapc_param_update_cmd *p_cmd = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CMD, TASK_GAPC, TASK_APP, gapc_param_update_cmd);

    p_cmd->conidx     = app_env.conidx;
    p_cmd->operation  = GAPC_UPDATE_PARAMS;
    p_cmd->intv_min   = p_conn_param->intv_min;
    p_cmd->intv_max   = p_conn_param->intv_max;
    p_cmd->latency    = p_conn_param->latency;
    p_cmd->time_out   = p_conn_param->time_out;

    // not used by a slave device
    p_cmd->ce_len_min = 0xFFFF;
    p_cmd->ce_len_max = 0xFFFF;

    // Send the message
    ke_msg_send(p_cmd);
}

void app_update_adv_state(bool start)
{
    // TODO [LT] - Check current advertising state

    // Start or stop advertising
    //app_adv_fsm_next();
	 //如果之前已经创建定向广播，则不用重新创建
	 #if 1
	 if ( start  )
	 	{
   	 if (0 == last_adv_type )
   		 {
   		 app_stop_advertising(normal_adv_idx);
   		 //app_delete_advertising();
   		 }
	   if (0xff == direct_adv_idx)
         {
         //创建定向广播
         dbg_print("create direct adv\r\n");
         app_env.adv_state = APP_ADV_STATE_IDLE;
         }
   	 app_adv_fsm_next();
	 	}
	 else
	 	{
 		if (1 == last_adv_type )
			{
			app_stop_advertising(direct_adv_idx);
			//app_delete_advertising();
			}
		else
			{
			app_stop_advertising(normal_adv_idx);
			
			}
		 	
	 	}
	 #endif
	 
}

#endif //(BLE_APP_PRF)


#if (NVDS_SUPPORT)

bool app_nv_set_bonded_device_info(bonded_dev_info_list_t *dev_info)
{
    if (dev_info == NULL)
    {
        return false;
    }
    if (nvds_put(NVDS_TAG_BONDED_DEV_INFO, NVDS_LEN_BONDED_DEV_INFO, (uint8_t *) dev_info) == NVDS_OK)
    {
        return true;
    }
    return false;
}

bool app_nv_get_bonded_device_info(bonded_dev_info_list_t *dev_info)
{
    if (dev_info == NULL)
    {
        return false;
    }
    nvds_tag_len_t length = NVDS_LEN_BONDED_DEV_INFO;
    if (nvds_get(NVDS_TAG_BONDED_DEV_INFO, &length, (uint8_t *)dev_info) == NVDS_OK)
    {
        return true;
    }
    return false;
}
#endif







#endif //(APP_PRESENT)

/// @} APP
