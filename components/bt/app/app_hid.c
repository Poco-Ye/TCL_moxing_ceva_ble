/**
 ****************************************************************************************
 *
 * @file app_hid.c
 *
 * @brief HID Application Module entry point
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

#include "rwip_config.h"            // SW configuration

#include <stdio.h>
#include <string.h>

#if (BLE_APP_HID)

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app.h"                    // Application Definitions
#include "app_sec.h"                // Application Security Module API
#include "app_task.h"               // Application task definitions
#include "app_hid.h"                // HID Application Module Definitions
#include "hogpd.h"             // HID Over GATT Profile Device Role Functions
#include "prf_types.h"              // Profile common types Definition
#include "arch.h"                    // Platform Definitions
#include "prf.h"
#include "ke_timer.h"

#if (NVDS_SUPPORT)
#include "nvds.h"                   // NVDS Definitions
#endif //(NVDS_SUPPORT)

#if (DISPLAY_SUPPORT)
#include "app_display.h"            // Application Display Module
#endif //(DISPLAY_SUPPORT)

#include "co_utils.h"               // Common functions

#if (KE_PROFILING)
#include "ke_mem.h"
#endif //(KE_PROFILING)

#include <string.h>
#include "gatt.h"
#include "gapm_msg.h"
#include "prf_utils.h"
#include "atiny_log.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Length of the HID Mouse Report
#define AUDIO_REPORT_LEN       (36)

#define KEYBOARD_REPORT_LEN   (8)

#define CONSUME_REPORT_LEN  (2)
/// Length of the Report Descriptor for an HID Mouse
//#define APP_HID_MOUSE_REPORT_MAP_LEN   (sizeof(app_hid_mouse_report_map))

/// Duration before connection update procedure if no report received (mouse is silent) - 20s
#define APP_HID_SILENCE_DURATION_1     (2000)
/// Duration before disconnection if no report is received after connection update - 60s
#define APP_HID_SILENCE_DURATION_2     (6000)

/// Number of reports that can be sent
#define APP_HID_NB_SEND_REPORT         (10)


#define APP_HID_KEYPAD_REPORT_MAP_LEN   (sizeof(app_hid_report_map))

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// States of the Application HID Module
enum app_hid_states
{
    /// Module is disabled (Service not added in DB)
    APP_HID_DISABLED,
    /// Module is idle (Service added but profile not enabled)
    APP_HID_IDLE,
    /// Module is enabled (Device is connected and the profile is enabled)
    APP_HID_ENABLED,
    /// The application can send reports
    APP_HID_READY,
    /// Waiting for a report
    APP_HID_WAIT_REP,

    APP_HID_STATE_MAX,
};

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// HID Application Module Environment Structure
static struct app_hid_env_tag app_hid_env;



/// HID Mouse Report Descriptor
static const uint8_t app_hid_report_map[] =
{
        /* 定义键盘 */
        0x05, 0x01,					                // USAGE PAGE (Generic Desktop)
        0x09, 0x06,					                // keyboard
        0xA1, 0x01,					                // COLLECTION（APP)
            0x85, APP_HID_REPORT_ID_KEYBOARD,	    //		REPORT ID
            // 8个按键输入
            0x05, 0x07,					            //		Usage Page(Key code)
            0x19, 0xE0,					            //		Usage Minimum (Left Control)	
            0x29, 0xE7,					            //		Usage Maximum (Right GUI)
            0x15, 0x00 ,				            //		Logical Minimum	0	每项取值最小值
            0x25, 0x01,					            //		Logical Maximum	1	每项取值最大值
            0x75, 0x01,					            //		Report Size  每个button用1bit表示
            0x95, 0x08,					            //		Report Count 总共8个1bit表示8个按键
            0x81, 0x02,					            //		Input (Data, Variable, Absolute) 可写，用途，绝对数据

            // 预留8bit输入
            0x95, 0x01,					            //		Report Count 1
            0x75, 0x08,					            //		Report Size 8
            0x81, 0x01,					            //		Input (Constant, Array, Absolute, Bit Field) 只读，绝对数据

            // 用6个字节表示按键键值
            0x95, 0x06,					            //		Report Count	6
            0x75, 0x08,					            //		Report Size	8
            0x15, 0x00,					            //		Logical minimum 0
            0x25, 0xFF,					            //		Logical maximum	255
            0x05, 0x07,					            //		Usage Page(Key code)
            0x19, 0x00,					            //		Usage Minimum(No event indicated)
            0x29, 0xFF,					            //		Usage Maximum(Reserved)
            0x81, 0x00,					            //		Input(Data, Array, Absolute, Bit Field)

            // 配置5个LED灯输出
            0x05, 0x08,					            //		Usage Page(LEDs)
            0x95, 0x05,					            //		Report Count 5
            0x75, 0x01,					            //		Report Size 1
            0x19, 0x01,					            //		Usage Minimum (Num lock)
            0x29, 0x05,					            //		Usage Maximum (Kana)
            0x91, 0x02,					            //		Output(Data, Value, Absolute, Non-volatile, Bit Field)

            // 预留3bit输出位
            0x95, 0x01,					            //		Report Count 1
            0x75, 0x03,					            //		Report Size	3
            0x91, 0x01,					            //		Output (Constant, Array, Absolute, Non-volatile, Bit Field)
        
        0xC0,						                //	End Collection

        // ===========================================================================================
        // ===========================================================================================

        /* 多媒体键盘 */
        0x05, 0x0C,                                 // Usage Page(Consumer)
        0x09, 0x01,                                 // Usage (Consumer Control)
        0xA1, 0x01,                                 // Collection (Application)
            0x85, APP_HID_REPORT_ID_MMKB,           //		REPORT ID
            0x19, 0x00,                             //      USAGE MINIMUM (0)
            0x2A, 0x9C, 0x02,                       //      USAGE MAXIMUM (668)
            0x15, 0x00,                             //      LOGICAL MINIMUM (0)
            0x26, 0x9C, 0x02,                       //      LOGICAL MAXIMUM (668)
            0x95, 0x01,                             //      REPORT COUNT (1)
            0x75, 0x10,                             //      REPORT SIZE (16)
            0x81, 0x00,                             //		Input (Data,Array,Absolute,Bit Field)
        0xC0,                                       //	End Collection

        // ===========================================================================================

        /* 语音数据通道 */
        0x06, 0x01, 0xFF,				// Usage Page (Vendor-defined 0xFF00)	06 00 FF	
        //0x0A, 0x10, 0xFF, 				// Usage (Vendor-defined 0xFF00)	0A 00 FF	
        0x09, 0x01,
        // 255个字节输入数据,传输语音数据的通道
        0xA1,0x02,						// Collection (Application)
            0x85, APP_HID_REPORT_ID_VENDOR1,//		Report Id (0xFC)
            0x09,0x14,                      //      Usage(Vendor-defined 0x0014)
            0x95,0x24,						//		Report Count (36)
            0x75,0x08,						//		Report Size (8)	
            0x15,0x80,						//		Logical minimum (0)		
            0x25,0x7F,						//		Logical maximum (255)
            //0x19,0x00,						//		Usage Minimum (Vendor-defined 0x0000)	
            //0x29,0xFF,						//		Usage Maximum (Vendor-defined 0x00FF)	
            0x81,0x22,						//		Input (Data,Array,Absolute,Bit Field)	
        
            0x85, APP_HID_REPORT_ID_VENDOR1,//		Report Id (0xFC)
            0x09, 0x04,                      //      Usage(Vendor-defined 0x0014)
            0x95, 0x01,						//		Report Count (36)
            0x75, 0x08,						//		Report Size (8)	
            0x91, 0x02,						//		Input (Data,Array,Absolute,Bit Field)
        0xC0,							// End Collection	

        // ===========================================================================================

        /* TV控制请求通道 */
        0x06, 0x00, 0xff,				// Usage Page (Vendor-defined 0xFF00)	06 00 FF	
        0x0a, 0x10, 0xff, 				// Usage (Vendor-defined 0xFF00)	0A 00 FF
        0xA1,0x01,						// Collection (Application)	
            0x85, APP_HID_REPORT_ID_VENDOR2,//		Report Id (0xFB)	
            0x95,0xFF,						//		Report Count (255)	
            0x75,0x08,						//		Report Size (8)	
            0x15,0x00,						//		Logical minimum (0)	
            0x25,0xFF,						//		Logical maximum (255)	
            0x19,0x00,						//		Usage Minimum (Vendor-defined 0x0000)	
            0x29,0xFF,						//		Usage Maximum (Vendor-defined 0x00FF)	
            0x91,0x00,						//		Output (Data,Array,Absolute,Bit Field)	

            // 回应TV控制请求
            0x95,0xFF,						//		Report Count (255)	
            0x75,0x08,						//		Report Size (8)	
            0x15,0x00,						//		Logical minimum (0)	
            0x25,0xFF,						//		Logical maximum (255)	
            0x19,0x00,						//		Usage Minimum (Vendor-defined 0x0000)	
            0x29,0xFF,						//		Usage Maximum (Vendor-defined 0x00FF)	
            0x81,0x00,						//		Input (Data,Array,Absolute,Bit Field)
        0xC0,							// End Collection	C0 	


};


/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
	void app_timer_hid_handler(void* p_env)
		{

	
			dbg_print("hid update param_handler\r\n");
		   app_hid_change_param();
		
			//co_time_timer_set(&(app_env.app_timer), 30000); 
		
		}

void app_hid_init(void)
{
    // Length of the mouse timeout value
   // uint8_t length = NVDS_LEN_MOUSE_TIMEOUT;

    // Reset the environment
    memset(&app_hid_env, 0, sizeof(app_hid_env));

    app_hid_env.nb_report = APP_HID_NB_SEND_REPORT;

    /*
     * Get the timeout value from the NVDS - This value is used each time a report is received
     * from the PS2 driver, store it in the environment.
     */
   // if (nvds_get(NVDS_TAG_MOUSE_TIMEOUT, &length, (uint8_t *)&app_hid_env.timeout) != NVDS_OK)
    {
    //    app_hid_env.timeout = APP_HID_SILENCE_DURATION_1;
    }

						 
	co_time_timer_init(&(app_env.app_timer), app_timer_hid_handler, NULL);


	 
}


/*
 ****************************************************************************************
 * @brief Function called when get GAP manager command complete events.
 *
 ****************************************************************************************
 */
void app_hid_start_mouse(void)
{
    /*-----------------------------------------------------------------------------------
     * CONFIGURE THE MOUSE
     *----------------------------------------------------------------------------------*/
    #if (PS2_SUPPORT)
    // Default mouse rate (200 report/s)
    uint8_t rate = 200;

    #if (NVDS_SUPPORT)
    uint8_t length = NVDS_LEN_MOUSE_SAMPLE_RATE;

    // Get sample rate from NVDS
    if (nvds_get(NVDS_TAG_MOUSE_SAMPLE_RATE, &length, &rate) == NVDS_OK)
    {
        // Check if value is among supported set
        if ((rate != 10) && (rate != 20) && (rate != 40) && (rate != 60) &&
            (rate != 80) && (rate != 100) && (rate != 200) )
        {
            // Default value
            rate = 200;
        }
    }
    #endif //(NVDS_SUPPORT)

    // Start PS2_mouse
    ps2_mouse_start(rate, &app_hid_send_mouse_report);
    #endif //(PS2_SUPPORT)
}

void app_hid_add_hids(void)
{
    struct hogpd_db_cfg *db_cfg;
    // Prepare the HOGPD_CREATE_DB_REQ message
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                                   TASK_GAPM, TASK_APP,
                                                   gapm_profile_task_add_cmd, sizeof(struct hogpd_db_cfg));

#if 1
    // Fill message
    req->operation   = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl     = SVC_SEC_LVL(NO_AUTH);//PERM(SVC_AUTH, AUTH);
    req->prf_api_id = TASK_ID_HOGPD;
    req->app_task    = TASK_APP;
    req->start_hdl   = 0;

    // Set parameters
    db_cfg = (struct hogpd_db_cfg* ) req->param;

    // Only one HIDS instance is useful
    db_cfg->hids_nb = 1;

    // The device is a mouse
    db_cfg->cfg[0].svc_features = HOGPD_CFG_PROTO_MODE_BIT | HOGPD_CFG_KEYBOARD_BIT;

    // Only one Report Characteristic is requested
    db_cfg->cfg[0].report_nb    = 7;
   /// db_cfg->cfg[0].report_id    = 1;
    db_cfg->cfg[0].report_id[APP_HID_REPORT_ID_KEYBOARD_INDEX] = APP_HID_REPORT_ID_KEYBOARD;

    // The report is an input report
    db_cfg->cfg[0].report_char_cfg[APP_HID_REPORT_ID_KEYBOARD_INDEX] = HOGPD_CFG_REPORT_IN;
    db_cfg->cfg[0].report_id[APP_HID_REPORT_ID_KEYBOARD_OUT_INDEX] = APP_HID_REPORT_ID_KEYBOARD;
    db_cfg->cfg[0].report_char_cfg[APP_HID_REPORT_ID_KEYBOARD_OUT_INDEX] = HOGPD_CFG_REPORT_OUT;

    db_cfg->cfg[0].report_id[APP_HID_REPORT_ID_MMKB_INDEX] = APP_HID_REPORT_ID_MMKB;
    db_cfg->cfg[0].report_char_cfg[APP_HID_REPORT_ID_MMKB_INDEX] = HOGPD_CFG_REPORT_IN;
    db_cfg->cfg[0].report_id[APP_HID_REPORT_ID_VENDOR1_INDEX] = APP_HID_REPORT_ID_VENDOR1;
    db_cfg->cfg[0].report_char_cfg[APP_HID_REPORT_ID_VENDOR1_INDEX] = HOGPD_CFG_REPORT_IN /* | SONATA_HOGPD_CFG_REPORT_WR*/;
    db_cfg->cfg[0].report_id[APP_HID_REPORT_ID_VENDOR1_OUT_INDEX] = APP_HID_REPORT_ID_VENDOR1;
    db_cfg->cfg[0].report_char_cfg[APP_HID_REPORT_ID_VENDOR1_OUT_INDEX] = HOGPD_CFG_REPORT_OUT /* | SONATA_HOGPD_CFG_REPORT_WR*/;
    db_cfg->cfg[0].report_id[APP_HID_REPORT_ID_VENDOR2_INDEX] = APP_HID_REPORT_ID_VENDOR2;
    db_cfg->cfg[0].report_char_cfg[APP_HID_REPORT_ID_VENDOR2_INDEX] = HOGPD_CFG_REPORT_IN;
    db_cfg->cfg[0].report_id[APP_HID_REPORT_ID_VENDOR2_OUT_INDEX] = APP_HID_REPORT_ID_VENDOR2;
    db_cfg->cfg[0].report_char_cfg[APP_HID_REPORT_ID_VENDOR2_OUT_INDEX] = HOGPD_CFG_REPORT_OUT;

    // HID Information
    db_cfg->cfg[0].hid_info.bcdHID       = 0x0111;         // HID Version 1.11
    db_cfg->cfg[0].hid_info.bCountryCode = 0x00;
    db_cfg->cfg[0].hid_info.flags        = HIDS_REMOTE_WAKE_CAPABLE | HIDS_NORM_CONNECTABLE;
    #endif
    // Send the message
    ke_msg_send(req);
}




/*
 ****************************************************************************************
 * @brief Function called when get connection complete event from the GAP
 *
 ****************************************************************************************
 */
void app_hid_enable_prf(uint8_t conidx)
{
    // Requested connection parameters
   // struct gapc_conn_param conn_param;

    uint16_t ntf_cfg;

    // Store the connection handle
    app_hid_env.conidx = conidx;

    // Allocate the message
    struct hogpd_enable_req * req = KE_MSG_ALLOC(HOGPD_ENABLE_REQ,
                                                 prf_get_task_num_from_api_id(TASK_ID_HOGPD),
                                                 TASK_APP,
                                                 hogpd_enable_req);

    // Fill in the parameter structure
    req->conidx     = conidx;
    // Notifications are disabled
    ntf_cfg         =  0xc1; //HOGPD_CFG_REPORT_NTF_EN;

    // Go to Enabled state
    app_hid_env.state = APP_HID_ENABLED;

    //now not save notify para
    #if 0 //(NVDS_SUPPORT)
    // If first connection with the peer device
    if (app_sec_get_bond_status())
    {
        // Length of the value read in NVDS
        uint8_t length   = NVDS_LEN_MOUSE_NTF_CFG;
        // Notification configuration

        if (nvds_get(NVDS_TAG_MOUSE_NTF_CFG, &length, (uint8_t *)&ntf_cfg) != NVDS_OK)
        {
            // If we are bonded this information should be present in the NVDS
            ASSERT_ERR(0);
        }

        // CCC enable notification
        if ((ntf_cfg & HOGPD_CFG_REPORT_NTF_EN ) != 0)
        {
            // The device is ready to send reports to the peer device
            app_hid_env.state = APP_HID_READY;
            app_hid_env.nb_report = APP_HID_NB_SEND_REPORT;

            // Restart the mouse timeout timer if needed
            if (app_hid_env.timeout != 0)
            {
                ke_timer_set(APP_HID_MOUSE_TIMEOUT_TIMER, TASK_APP, (uint16_t)(app_hid_env.timeout));
                app_hid_env.timer_enabled = true;
            }
        }
    }
    #endif //(NVDS_SUPPORT)

    req->ntf_cfg[conidx] = ntf_cfg;

    ke_msg_send(req);

	 co_time_timer_set(&(app_env.app_timer),3000);


	 
}


void app_hid_change_param(void)
{
	struct gapc_conn_param conn_param;
	
	conn_param.intv_min = 6;
	conn_param.intv_max = 12;
	conn_param.latency  = 49;
	conn_param.time_out = 300;
	
	
	app_update_param(&conn_param);

}







/*
 ****************************************************************************************
 * @brief Function called from PS2 driver
 *
 ****************************************************************************************
 */
void app_hid_send_keypad_report(uint32_t report_code)
{

if(1 == app_get_connection_state())

	{
               
    struct hogpd_report_upd_req * req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_UPD_REQ,
                                                      prf_get_task_num_from_api_id(TASK_ID_HOGPD),
                                                      TASK_APP,
                                                      hogpd_report_upd_req,
                                                      KEYBOARD_REPORT_LEN);
	 
    uint8_t report_buff[KEYBOARD_REPORT_LEN]= {0};
    req->conidx  = app_hid_env.conidx;
    //now fill report
    req->hid_idx = 0;//app_hid_env.hid_idx;
    req->report_type     = HOGPD_REPORT; //HOGPD_BOOT_MOUSE_INPUT_REPORT;//
    req->report_idx      = APP_HID_REPORT_ID_KEYBOARD_INDEX; //0 for boot reports and report map
    req->report.length   = KEYBOARD_REPORT_LEN;
   
	 report_buff[2] = (int8_t)report_code;
	 memcpy(&req->report.value[0], &report_buff[0], KEYBOARD_REPORT_LEN);

	
    ke_msg_send(req);
    }
   
 
}


void app_hid_send_consumer_key(uint16_t keycode)
{
if(1 == app_get_connection_state())
	 
	{				 
	struct hogpd_report_upd_req * req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_UPD_REQ,
																	  prf_get_task_num_from_api_id(TASK_ID_HOGPD),
																	  TASK_APP,
																	  hogpd_report_upd_req,
																	  CONSUME_REPORT_LEN);
	
	uint8_t report_buff[CONSUME_REPORT_LEN]= {0};
	req->conidx  = app_hid_env.conidx;
	//now fill report
	req->hid_idx = 0;//app_hid_env.hid_idx;
	req->report_type		= HOGPD_REPORT; //HOGPD_BOOT_MOUSE_INPUT_REPORT;//
	req->report_idx		= APP_HID_REPORT_ID_MMKB_INDEX; //0 for boot reports and report map
	req->report.length	= CONSUME_REPORT_LEN;
	
	report_buff[0] = (uint8_t) (keycode & 0xFF);
   report_buff[1] = (uint8_t) (keycode >> 8 & 0xFF);
	memcpy(&req->report.value[0], &report_buff[0], CONSUME_REPORT_LEN);
	
	
	ke_msg_send(req);
   }
    
}



void app_hid_send_audio_report(uint8_t* data, uint8_t data_len)
{

if(1 == app_get_connection_state())
	 
	{

               
    struct hogpd_report_upd_req * req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_UPD_REQ,
                                                      prf_get_task_num_from_api_id(TASK_ID_HOGPD),
                                                      TASK_APP,
                                                      hogpd_report_upd_req,
                                                      AUDIO_REPORT_LEN);
	 

    req->conidx  = app_hid_env.conidx;
    //now fill report
    req->hid_idx = 0;//app_hid_env.hid_idx;
    req->report_type     = HOGPD_REPORT; //HOGPD_BOOT_MOUSE_INPUT_REPORT;//
    req->report_idx      = APP_HID_REPORT_ID_VENDOR1_INDEX; //0 for boot reports and report map
    req->report.length   = data_len;
   
	 memcpy(&req->report.value[0], data, data_len);

	
    ke_msg_send(req);
   
	 }
}


/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */



static int hogpd_ctnl_pt_ind_handler(ke_msg_id_t const msgid,
                                     struct hogpd_ctnl_pt_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
	dbg_print2("%s %d\r\n",__FUNCTION__,__LINE__);

#if 1

    if (param->conidx == app_hid_env.conidx)
    {

        struct hogpd_report_read_cfm *req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_READ_CFM,
                                                        src_id, ///prf_get_task_from_id(TASK_ID_HOGPD),/* src_id */
                                                        dest_id, ///TASK_APP,
                                                        hogpd_report_read_cfm,
                                                        0);

        //req->conidx = app_hid_env.conidx; ///???
        req->conidx = param->conidx; 
        /// Operation requested (read/write @see enum hogpd_op)
   
        /// Status of the request
        req->status = GAP_ERR_NO_ERROR;  ///???
        //req->token = param->token;
		  
        req->report.length = 0;
		  req->tot_length = 0;

        // Send the message
       // ke_msg_send(req);
    }
	 #endif
    return (KE_MSG_CONSUMED);
}




static int hogpd_ntf_cfg_ind_handler(ke_msg_id_t const msgid,
                                     struct hogpd_ntf_cfg_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
	dbg_print2("%s,condix:%d\r\n",__FUNCTION__,param->conidx);

    if (app_hid_env.conidx == param->conidx)
    {
        if ((param->ntf_cfg[param->conidx] & HOGPD_CFG_REPORT_NTF_EN ) != 0)
        {
            // The device is ready to send reports to the peer device
            app_hid_env.state = APP_HID_READY;
        }
        else
        {
            // Come back to the Enabled state
            if (app_hid_env.state == APP_HID_READY)
            {
                app_hid_env.state = APP_HID_ENABLED;
            }
        }

        // Store the notification configuration in the database
       // if (nvds_put(NVDS_TAG_MOUSE_NTF_CFG, NVDS_LEN_MOUSE_NTF_CFG,
        //             (uint8_t *)&param->ntf_cfg[param->conidx]) != NVDS_OK)
        //{
            // Should not happen
        //    ASSERT_ERR(0);
        //}
    }

    return (KE_MSG_CONSUMED);
}


static int hogpd_report_req_ind_handler(ke_msg_id_t const msgid,
                                    struct hogpd_report_read_req_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
	

dbg_print("hogpd_report_req_ind_handler,condix:%x,idx:%x,type:%x\r\n",param->conidx,param->hid_idx,param->report_type);

    app_hid_env.hid_idx = param->hid_idx;
	 switch (param->report_type)
	 {
		  // An Input Report
		  case HOGPD_BOOT_KEYBOARD_INPUT_REPORT:	
		  case HOGPD_BOOT_KEYBOARD_OUTPUT_REPORT:
		  	{
		  uint8_t localValue[] = "Keyboard Report";
        struct hogpd_report_read_cfm *req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_READ_CFM,
                                                        src_id, ///prf_get_task_from_id(TASK_ID_HOGPD),/* src_id */
                                                        dest_id, ///TASK_APP,
                                                        hogpd_report_read_cfm,
                                                        sizeof(localValue));

        //req->conidx = app_hid_env.conidx; ///???
        req->conidx = param->conidx; 
        /// Operation requested (read/write @see enum hogpd_op)
   
        /// Status of the request
        req->status = GAP_ERR_NO_ERROR;  ///???
        req->token = param->token;
		  
        req->report.length = sizeof(localValue);

         memcpy(&req->report.value[0], &localValue[0],  sizeof(localValue));
			req->tot_length = sizeof(localValue);

        // Send the message
        ke_msg_send(req);
    } 
			break;

		  // Boot Mouse input report
		  case HOGPD_BOOT_MOUSE_INPUT_REPORT: 
		  	{
            uint8_t localValue[] = "Mouse report";
            struct hogpd_report_read_cfm *req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_READ_CFM,
                                                     src_id, ///prf_get_task_from_id(TASK_ID_HOGPD),/* src_id */
                                                     dest_id, ///TASK_APP,
                                                     hogpd_report_read_cfm,
                                                     sizeof(localValue));
            
            //req->conidx = app_hid_env.conidx; ///???
            req->conidx = param->conidx; 
            /// Operation requested (read/write @see enum hogpd_op)
            
            /// Status of the request
            req->status = GAP_ERR_NO_ERROR;  ///???
            req->token = param->token;
            
            req->report.length = sizeof(localValue);
            
            memcpy(&req->report.value[0], &localValue[0],  sizeof(localValue));
            req->tot_length = sizeof(localValue);
            
            // Send the message
            ke_msg_send(req);
            }
		  break;
		  // Normal report
		  case HOGPD_REPORT:					 
            {
            uint8_t localValue[] = "Report value";
            struct hogpd_report_read_cfm *req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_READ_CFM,
                                                     src_id, ///prf_get_task_from_id(TASK_ID_HOGPD),/* src_id */
                                                     dest_id, ///TASK_APP,
                                                     hogpd_report_read_cfm,
                                                     sizeof(localValue));
            
            //req->conidx = app_hid_env.conidx; ///???
            req->conidx = param->conidx; 
            /// Operation requested (read/write @see enum hogpd_op)
            
            /// Status of the request
            req->status = GAP_ERR_NO_ERROR;  ///???
            req->token = param->token;
            
            req->report.length = sizeof(localValue);
            
            memcpy(&req->report.value[0], &localValue[0],  sizeof(localValue));
            req->tot_length = sizeof(localValue);
            
            // Send the message
            ke_msg_send(req);
            } 
		  break;
		  // Report MAP
		  case HOGPD_REPORT_MAP: 			 
		  	{
        struct hogpd_report_read_cfm *req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_READ_CFM,
                                                        src_id, ///prf_get_task_from_id(TASK_ID_HOGPD),/* src_id */
                                                        dest_id, ///TASK_APP,
                                                        hogpd_report_read_cfm,
                                                        APP_HID_KEYPAD_REPORT_MAP_LEN);

        //req->conidx = app_hid_env.conidx; ///???
        req->conidx = param->conidx; 
        /// Operation requested (read/write @see enum hogpd_op)
   
        /// Status of the request
        req->status = GAP_ERR_NO_ERROR;  ///???
        req->token = param->token;
		  
        req->report.length = APP_HID_KEYPAD_REPORT_MAP_LEN;

         memcpy(&req->report.value[0], &app_hid_report_map[0], APP_HID_KEYPAD_REPORT_MAP_LEN);
			req->tot_length = APP_HID_KEYPAD_REPORT_MAP_LEN;

        // Send the message
        ke_msg_send(req);
    } 
		  break;
		  default:											 
		  	{  											 
			} 
		  break;
	 }



 
    

    return (KE_MSG_CONSUMED);
}


static int hogpd_proto_mode_req_ind_handler(ke_msg_id_t const msgid,
                                        struct hogpd_proto_mode_req_ind const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
	dbg_print("%s %d,condix:%x,hid_idx:%x,mode:%x\r\n",__FUNCTION__,__LINE__,param->conidx,param->hid_idx,param->proto_mode);

   // if ((param->conidx == app_hid_env.conidx) && (param->operation == HOGPD_OP_PROT_UPDATE))
   if (1) //(param->conidx == app_hid_env.conidx) 
    {

        //make use of param->proto_mode
        struct hogpd_proto_mode_cfm *req = KE_MSG_ALLOC_DYN(HOGPD_PROTO_MODE_CFM,
                                                        prf_get_task_num_from_api_id(TASK_ID_HOGPD),/* src_id */
                                                        TASK_APP,
                                                        hogpd_proto_mode_cfm,
                                                        0);
        /// Connection Index
        req->conidx = param->conidx;//app_hid_env.conidx; 
        /// Status of the request
        req->status = GAP_ERR_NO_ERROR;
        /// HIDS Instance
        req->hid_idx =param->hid_idx;//app_hid_env.conidx;
        /// New Protocol Mode Characteristic Value
        req->proto_mode = param->proto_mode;
        

        // Send the message
        ke_msg_send(req);
    }
    else
    {
        struct hogpd_proto_mode_cfm *req = KE_MSG_ALLOC_DYN(HOGPD_PROTO_MODE_CFM,
                                                        prf_get_task_num_from_api_id(TASK_ID_HOGPD),/* src_id */
                                                        TASK_APP,
                                                        hogpd_proto_mode_cfm,
                                                        0);
        /// Status of the request
        req->status = ATT_ERR_APP_ERROR;

        /// Connection Index
        req->conidx = app_hid_env.conidx;
        /// HIDS Instance
        req->hid_idx = app_hid_env.conidx;
        /// New Protocol Mode Characteristic Value
        req->proto_mode = param->proto_mode;
        
        // Send the message
        ke_msg_send(req);
    }
    return (KE_MSG_CONSUMED);
}


static int hogpd_report_upd_handler(ke_msg_id_t const msgid,
                                   struct hogpd_report_upd_rsp const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
	dbg_print2("%s %d\r\n",__FUNCTION__,__LINE__);

    if (app_hid_env.conidx == param->conidx)
    {
        if (GAP_ERR_NO_ERROR == param->status)
        {
        	    dbg_print("send data success!\r\n");
            //if (app_hid_env.nb_report < APP_HID_NB_SEND_REPORT)
            //{
            //    app_hid_env.nb_report++;
            //}
				
        }
        else
        {
            dbg_print("send data fail:%x\r\n",param->status);
            // we get this message if error occur while sending report
            // most likely - disconnect
            // Go back to the ready state
            app_hid_env.state = APP_HID_IDLE;
            // change mode
            // restart adv
            // Try to restart advertising if needed
            app_update_adv_state(true);

            //report was not success - need to restart???
        }
    }
    return (KE_MSG_CONSUMED);
}

static int hogpd_report_write_req_handler(ke_msg_id_t const msgid,
                                   struct hogpd_report_write_req_ind const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
	{
		
	
	dbg_print("hogpd_report_write_req_handler,condix:%x,idx:%x,type:%x,report_idx:%x,len:%x\r\n",param->conidx,param->hid_idx,param->report_type,param->report_idx,param->report.length);
	
      for (int i = 0; i < param->report.length; ++i)
          { 
              dbg_print("%x ", param->report.value[i]); 
          }
          dbg_print("\r\n"); 
		     
	if (APP_HID_REPORT_ID_VENDOR1_OUT_INDEX == param->report_idx)
		{
		if (0x01 == param->report.value[0])
			{
			dbg_print("tv allow send audio data\r\n");
			
			}
		
		}
	
	
	 
		 
	
		 return (KE_MSG_CONSUMED);
	}



static int hogpd_enable_rsp_handler(ke_msg_id_t const msgid,
                                     struct hogpd_enable_rsp const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Function called when the APP_HID_MOUSE_TIMEOUT_TIMER expires.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int app_hid_mouse_timeout_timer_handler(ke_msg_id_t const msgid,
                                               void const *param,
                                               ke_task_id_t const dest_id,
                                               ke_task_id_t const src_id)
{

    app_hid_env.timer_enabled = false;

    if (app_hid_env.state == APP_HID_READY)
    {
        // Requested connection parameters
        struct gapc_conn_param conn_param;
        // Length
       // uint8_t length = NVDS_LEN_MOUSE_TIMEOUT;
        // Timer value
        uint16_t timer_val;

        /*
         * Request an update of the connection parameters
         * Requested connection interval: 10ms
         * Latency: 200
         * Supervision Timeout: 5s
         */
        conn_param.intv_min = 8;
        conn_param.intv_max = 8;
        conn_param.latency  = 200;
        conn_param.time_out = 500;

        app_update_param(&conn_param);

        // Go to the Wait for Report state
        app_hid_env.state = APP_HID_WAIT_REP;

        // Get the timer value from the NVDS
       // if (nvds_get(NVDS_TAG_MOUSE_ENERGY_SAFE, &length, (uint8_t *)&timer_val) != NVDS_OK)
       // {
            timer_val = APP_HID_SILENCE_DURATION_2;
        //}

        // Relaunch the timer
        ke_timer_set(APP_HID_MOUSE_TIMEOUT_TIMER, TASK_APP, timer_val);
        app_hid_env.timer_enabled = true;
    }
    else if (app_hid_env.state == APP_HID_WAIT_REP)
    {
      // Disconnect the link with the device
        app_disconnect();


        // Go back to the ready state
        app_hid_env.state = APP_HID_IDLE;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int app_hid_msg_dflt_handler(ke_msg_id_t const msgid,
                                    void const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // Drop the message

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Set the value of the Report Map Characteristic in the database
 ****************************************************************************************
 */


/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Default State handlers definition
const struct ke_msg_handler app_hid_msg_handler_list[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,        (ke_msg_func_t)app_hid_msg_dflt_handler},

    {HOGPD_ENABLE_RSP,              (ke_msg_func_t)hogpd_enable_rsp_handler},
    {HOGPD_NTF_CFG_IND,             (ke_msg_func_t)hogpd_ntf_cfg_ind_handler},
    {HOGPD_REPORT_READ_REQ_IND,     (ke_msg_func_t)hogpd_report_req_ind_handler},
    {HOGPD_PROTO_MODE_REQ_IND,      (ke_msg_func_t)hogpd_proto_mode_req_ind_handler},
    {HOGPD_CTNL_PT_IND,             (ke_msg_func_t)hogpd_ctnl_pt_ind_handler},
    {HOGPD_REPORT_UPD_RSP,          (ke_msg_func_t)hogpd_report_upd_handler},
	 {HOGPD_REPORT_WRITE_REQ_IND, 	(ke_msg_func_t)hogpd_report_write_req_handler},

    {APP_HID_MOUSE_TIMEOUT_TIMER,   (ke_msg_func_t)app_hid_mouse_timeout_timer_handler},
};

const struct app_subtask_handlers app_hid_handlers = APP_HANDLERS(app_hid);

#endif //(BLE_APP_HID)

/// @} APP
