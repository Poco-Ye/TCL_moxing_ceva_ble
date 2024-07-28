/**
 ****************************************************************************************
 *
 * @file app_task.c
 *
 * @brief RW APP Task implementation
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APPTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"          // SW configuration

#if (APP_PRESENT)
#include "app_task.h"             // Application Manager Task API
#include "app.h"                  // Application Manager Definition
#include "gapc.h"
#include "gapm.h"
#include "gapc_msg.h"
#include "gapm_msg.h"
#include <string.h>
#include "co_utils.h"
#include "ke_timer.h"             // Kernel timer

#if (BLE_APP_SEC)
#include "app_sec.h"              // Security Module Definition
#endif //(BLE_APP_SEC)

#if (BLE_APP_HT)
#include "app_ht.h"               // Health Thermometer Module Definition
#include "htpt.h"
#endif //(BLE_APP_HT)

#if (BLE_APP_DIS)
#include "app_dis.h"              // Device Information Module Definition

#endif //(BLE_APP_DIS)

#if (BLE_APP_BATT)
#include "app_batt.h"             // Battery Module Definition
//#include "bass_task.h"
#endif //(BLE_APP_BATT)

#if (BLE_APP_HID)
#include "app_hid.h"              // HID Module Definition
//#include "hogpd_task.h"
#endif //(BLE_APP_HID)

#if (BLE_APP_GATV)
#include "app_gatv.h"         
#endif

#if (BLE_APP_AM0)
#include "app_am0.h"              // Audio Mode 0 Application
#endif //(BLE_APP_AM0)

#if (DISPLAY_SUPPORT)
#include "app_display.h"          // Application Display Definition
#endif //(DISPLAY_SUPPORT)

#if (BLE_APP_MESH)
#include "app_mesh.h"                // Mesh Module Definition
#if (DISPLAY_SUPPORT)
#include "app_display_mesh.h"     // Mesh Display Definition
#endif //(DISPLAY_SUPPORT)
#endif //(BLE_APP_MESH)
#include "app_keypad.h"

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#include "gapc_le_msg.h"
#include "atiny_log.h"
#include "gapm_le_msg.h"
#include "gap.h"
#include "app_voice.h"
#include "app_hid.h"                // HID Application Module Definitions
#include "ms_keyscan.h"
#include "app_gatv.h"               


static uint8_t app_connection_state = 0;

// 获取蓝牙连接状态
uint8_t app_get_connection_state(void)
{
    return app_connection_state;
}

// 设置蓝牙连接状态
void app_set_connection_state(uint8_t state)
{
    app_connection_state = state;
}



#if (BLE_APP_PRESENT)
#if (BLE_APP_SEC)
uint8_t app_loc_irk[KEY_LEN]= {0};

static uint8_t app_get_handler(const struct app_subtask_handlers *handler_list_desc,
                               ke_msg_id_t msgid,
                               void *p_param,
                               ke_task_id_t src_id)
{
    // Counter
    uint8_t counter;

    // Get the message handler function by parsing the message table
    for (counter = handler_list_desc->msg_cnt; 0 < counter; counter--)
    {
        struct ke_msg_handler handler
#ifdef WIN32
				= *(struct ke_msg_handler*)(handler_list_desc->p_msg_handler_tab + counter - 1);
#else
                = (struct ke_msg_handler)(*(handler_list_desc->p_msg_handler_tab + counter - 1));
#endif

        if ((handler.id == msgid) ||
            (handler.id == KE_MSG_DEFAULT_HANDLER))
        {
            // If handler is NULL, message should not have been received in this state
            ASSERT_ERR(handler.func);

            return (uint8_t)(handler.func(msgid, p_param, TASK_APP, src_id));
        }
    }

    // If we are here no handler has been found, drop the message
    return (KE_MSG_CONSUMED);
}
#endif // (BLE_APP_PRESENT)
#endif
/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */



/**
 ****************************************************************************************
 * @brief Handles GAP manager command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapm_cmp_evt const *p_param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    #if (BLE_APP_PRESENT && NVDS_SUPPORT)
    uint8_t key_len = KEY_LEN;
    #endif //(BLE_APP_PRESENT && NVDS_SUPPORT)
	 dbg_print2("gapm_cmp_evt_handler,%x\r\n",p_param->operation);

    switch(p_param->operation)
    {   
        //add config rsp code
    	  //case GAPM_SET_DEV_CONFIG:
		  //		{
				
			//   app_adv_fsm_next();
    	  //		}

			//break;
			
        #if(BLE_APP_PRESENT)
        case (GAPM_PROFILE_TASK_ADD):
        {
            #if 1 //(BLE_APP_PRF)
            if (app_sec_get_bond_status()==true) 
            {
                #if (NVDS_SUPPORT)
                // If Bonded retrieve the local IRK from NVDS
                if (nvds_get(NVDS_TAG_LOC_IRK, &key_len, app_env.loc_irk) == NVDS_OK)
                {
                    // Set the IRK in the GAP
                    struct gapm_set_irk_cmd *p_cmd = KE_MSG_ALLOC(GAPM_SET_IRK_CMD,
                                                                TASK_GAPM, TASK_APP,
                                                                gapm_set_irk_cmd);
                    ///  - GAPM_SET_IRK: 
                    p_cmd->operation = GAPM_SET_IRK;
                    memcpy(&p_cmd->irk.key[0], &app_env.loc_irk[0], KEY_LEN);
                    ke_msg_send(p_cmd);
                }
                else
                #endif //(NVDS_SUPPORT)
                {
                    // If cannot read IRK from NVDS ASSERT
                    ASSERT_ERR(0);
                }
            }
            else // Need to start the generation of new IRK  
            {    //生成解析随机addr  new
                struct gapm_gen_rand_nb_cmd *p_cmd = KE_MSG_ALLOC(GAPM_GEN_RAND_NB_CMD,
                                                                TASK_GAPM, TASK_APP,
                                                                gapm_gen_rand_nb_cmd);
					 dbg_print2("send msg for first irk\r\n");
                p_cmd->operation   = GAPM_GEN_RAND_NB;
                app_env.rand_cnt = 1;
                ke_msg_send(p_cmd);
            }
            
            #elif (BLE_APP_MESH)
            // Go to the ready state
            ke_state_set(TASK_APP, APP_READY);

            // Send current event status to mesh state machine then check what to do next
            app_mesh_evt(APP_MESH_EVT_TASK_ADDED);
            #endif // (BLE_APP_PRF)
        }
        break;

        #if (BLE_APP_PRF)
        case (GAPM_GEN_RAND_NB) :
        {
		  	   dbg_print2("GAPM_GEN_RAND_NB,%x\r\n",app_env.rand_cnt);
            if (app_env.rand_cnt == 1)
            {
                // Generate a second random number
                app_env.rand_cnt++;
                struct gapm_gen_rand_nb_cmd *p_cmd = KE_MSG_ALLOC(GAPM_GEN_RAND_NB_CMD,
                                                                TASK_GAPM, TASK_APP,
                                                                gapm_gen_rand_nb_cmd);
                p_cmd->operation = GAPM_GEN_RAND_NB;
                ke_msg_send(p_cmd);
            }
            else
            {
                struct gapm_set_irk_cmd *p_cmd = KE_MSG_ALLOC(GAPM_SET_IRK_CMD,
                                                        TASK_GAPM, TASK_APP,
                                                        gapm_set_irk_cmd);
                app_env.rand_cnt=0;
                ///  - GAPM_SET_IRK
                p_cmd->operation = GAPM_SET_IRK;
                //memcpy(&p_cmd->irk.key[0], &app_env.loc_irk[0], KEY_LEN);
                memcpy(&p_cmd->irk.key[0], &app_loc_irk[0], KEY_LEN);
                ke_msg_send(p_cmd);
            }
        }
        break;
        #endif //(BLE_APP_PRF)

        #if (BLE_APP_PRF)
        case (GAPM_SET_IRK):
        {
            // ASSERT_INFO(p_param->status == GAP_ERR_NO_ERROR, p_param->operation, p_param->status);
            // If not Bonded already store the generated value in NVDS
            if (app_sec_get_bond_status()==false)
            {
                #if (NVDS_SUPPORT)
                if (nvds_put(NVDS_TAG_LOC_IRK, KEY_LEN, (uint8_t *)&app_env.loc_irk) != NVDS_OK)
                #endif //(NVDS_SUPPORT)
                {
                    //ASSERT_INFO(0, 0, 0);
                    dbg_print2("just set none,no nvram!!!");
                }
            }

            app_env.rand_cnt = 0;
        }
        break;
        #endif //(BLE_APP_PRF)

        #if (BLE_APP_PRF)
		  
	//add by liujin for dev config RSP	  
        case (GAPM_SET_DEV_CONFIG):  
        {
	#if 1		  	
		  static bool add_flag = false;
	
			if (!add_flag)
				{
	#if (BLE_APP_BATT)			
				// add batt service
				app_batt_add_bas();
	#endif
	#if (BLE_APP_HID)
				// add hogpd service
				app_hid_add_hids();
	#endif			

	#if (BLE_APP_GATV)			
				app_gatv_add_gatvs();
	#endif	


	
   #if (BLE_APP_HT)	
		      app_ht_add_hts();
	#endif
	
	#if (BLE_APP_DIS)
	         app_dis_add_dis();
	#endif	
				}
	#endif

	
			// Perform next operation
			app_adv_fsm_next();

        }
		  break;
	//add end  
	     case (GAPM_STOP_ACTIVITY):
			 // app_adv_fsm_next();
           dbg_print2("stop activity\r\n");
		  break;
         
		  case (GAPM_DELETE_ACTIVITY):
		  	   dbg_print2("*GAPM_DELETE_ACTIVITY,status:%x,operation:%x,adv_op:%x\r\n",p_param->status,p_param->operation,app_env.adv_op);
	         dbg_print2("del activity\r\n");
				//app_adv_fsm_next();
		  break;		
        case (GAPM_CREATE_ADV_ACTIVITY):
       // case (GAPM_STOP_ACTIVITY):
        case (GAPM_START_ACTIVITY):
        case (GAPM_SET_ADV_DATA):       //set  adv rsp
        case (GAPM_SET_SCAN_RSP_DATA):  //set scan rsp
        {
            // Sanity checks
            //ASSERT_INFO(app_env.adv_op == p_param->operation, app_env.adv_op, p_param->operation);
            //ASSERT_INFO(p_param->status == GAP_ERR_NO_ERROR, p_param->status, app_env.adv_op);
				dbg_print2("*status:%x,operation:%x,adv_op:%x\r\n",p_param->status,p_param->operation,app_env.adv_op);

            // Perform next operation
            app_adv_fsm_next();
        } break;

        case (GAPM_DELETE_ALL_ACTIVITIES) :
        {
            // Re-Invoke Advertising
            app_env.adv_state = APP_ADV_STATE_IDLE;
            //app_adv_fsm_next();
        } break;
        #endif //(BLE_APP_PRF)
        #endif // (BLE_APP_PRESENT)
        default:
        {
            // Drop the message
        }
        break;
    }

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles reception of all messages sent from the lower layers to the application
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int app_msg_handler(ke_msg_id_t const msgid,
                            void *p_param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    // Message policy
    uint8_t msg_pol = KE_MSG_CONSUMED;
    #if (BLE_APP_PRESENT)
    // Retrieve identifier of the task from received message
    ke_task_id_t src_task_id = MSG_T(msgid);

    dbg_print2("app_msg_handler:%d\r\n",src_task_id);
    switch (src_task_id)
    {
        case (TASK_ID_GAPC):
        {
            #if (BLE_APP_SEC)
            if ((msgid >= GAPC_BOND_CMD) &&
                  (msgid <= GAPC_ENCRYPT_IND))
            {
                // Call the Security Module
                dbg_print2("\r\n---call the security module---\r\n");
                msg_pol = app_get_handler(&app_sec_handlers, msgid, p_param, src_id);
            }
            #endif //(BLE_APP_SEC)
            // else drop the message
        } break;

        case (TASK_ID_GATT):
        {
            // Service Changed - Drop
        } break;

        #if (BLE_APP_HT)
        case (TASK_ID_HTPT):
        {
            // Call the Health Thermometer Module
            msg_pol = app_get_handler(&app_ht_handlers, msgid, p_param, src_id);
        } break;
        #endif //(BLE_APP_HT)

        #if (BLE_APP_HID)
        case (TASK_ID_HOGPD):
        {
            // Call the HID Module
            msg_pol = app_get_handler(&app_hid_handlers, msgid, p_param, src_id);
        } break;
        #endif //(BLE_APP_HID)

#if (BLE_APP_GATV)
        case (TASK_ID_GATVS):
        {
            // Call the HID Module
            msg_pol = app_get_handler(&app_gatv_handlers, msgid, p_param, src_id);
        } break;
#endif //(BLE_APP_HID)


        #if (BLE_APP_MESH)
        case (TASK_ID_MESH) :
        {
            // Call the mesh module
            msg_pol = app_get_handler(&app_mesh_handlers, msgid, p_param, src_id);
        }break;
        #endif //(BLE_APP_MESH)

        #if (BLE_APP_BATT)
        case (TASK_ID_BASS):
        {
            // Call the Battery Module
            msg_pol = app_get_handler(&app_batt_handlers, msgid, p_param, src_id);
        } break;
        #endif //(BLE_APP_BATT)

        #if (BLE_APP_AM0)
        case (TASK_ID_AM0):
        {
            // Call the Audio Mode 0 Module
            msg_pol = app_get_handler(&app_am0_handlers, msgid, p_param, src_id);
        } break;

        #endif //(BLE_APP_AM0)

        default:
        {
            #if (BLE_APP_HT)
            if (msgid == APP_HT_MEAS_INTV_TIMER)
            {
                msg_pol = app_get_handler(&app_ht_handlers, msgid, p_param, src_id);
            }
            #endif //(BLE_APP_HT)

            #if (BLE_APP_HID)
            if (msgid == APP_HID_MOUSE_TIMEOUT_TIMER)
            {
                msg_pol = app_get_handler(&app_hid_handlers, msgid, p_param, src_id);
            }
            #endif //(BLE_APP_HID)

            #if (BLE_APP_MESH)
            #if (DISPLAY_SUPPORT)
            if (msgid == APP_MESH_ATTENTION_TIMER
                    || msgid == APP_MESH_SAVING_TIMER
                    || msgid == APP_MESH_REMOVING_TIMER)
            {
                // Call the mesh display module
                msg_pol = app_get_handler(&app_display_mesh_handlers, msgid, p_param, src_id);
            }

            // Timer used to wait transition time to toggle the on off state.
            if (msgid == APP_MESH_TRANSITION_TIMER)
            {
                // Call the mesh display module
                msg_pol = app_get_handler(&app_display_mesh_handlers, msgid, p_param, src_id);
            }

            #endif //(DISPLAY_SUPPORT)
            #endif //(BLE_APP_MESH)

        } break;
    }
    #endif // (BLE_APP_PRESENT)

    return (msg_pol);
}

static int app_keypad_msg_handler(ke_msg_id_t const msgid,
                            void *p_param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    struct app_keypad_db_cfg *p_keypad = (struct app_keypad_db_cfg *)p_param;

    //ke_task_id_t src_task_id = MSG_T(msgid);

    dbg_print("app_keypad_msg:key_value:%x,press=%d\r\n",
            p_keypad->key_value, p_keypad->press);
    if ((p_param) &&(app_get_connection_state()))
       {
   
   		  
   #if (BLE_APP_HID)
   
       switch(p_keypad->key_value)
   	 	{
   	 case IR_MUTE: //k1
				{
   			app_hid_send_keypad_report(p_keypad->press < 2 ? 0xef : 0);//mute
				
   			}
		 break;



   	 case IR_POWER: //k2
      		{
      		app_hid_send_keypad_report(p_keypad->press < 2 ? 0x66 : 0);//power
      		}
		 break;

   	 case IR_EXIT: //k3
      		{
      		app_hid_send_keypad_report(p_keypad->press < 2 ? 0xf1 : 0);//back
      		}
		 break;	
   	 case IR_SETTING: //k4
      		{
      		app_hid_send_keypad_report(p_keypad->press < 2 ? 0xa8 : 0);//setting
      		}
		 break;		
   	 case IR_HOME: //k5
      		{
      		app_hid_send_keypad_report(p_keypad->press < 2 ? 0x83 : 0);//HOME 
      		}
		 break;	

   	 case IR_ARROW_UP: //k6
      		{
      		app_hid_send_keypad_report(p_keypad->press < 2 ? 0x52 : 0);//up 
      		}
		 break;			

   	 case IR_ARROW_DOWN:  //k7
   			{
   			app_hid_send_keypad_report(p_keypad->press < 2 ? 0x51 : 0);//Down
   			}
   	 break;
		 case IR_ARROW_LEFT:	//k8
				{
				app_hid_send_keypad_report(p_keypad->press < 2 ? 0x50 : 0);//left
				}
				break;

			case IR_ARROW_RIGHT:	//k9
				{
				app_hid_send_keypad_report(p_keypad->press < 2 ? 0x4f : 0);//right
				}
				break;	
				
			case IR_OK:	//k10
				{
				app_hid_send_keypad_report(p_keypad->press < 2 ? 0x28 : 0);//enter
				}
				break;
			case IR_VOL_UP:	//k11
				{
				app_hid_send_keypad_report(p_keypad->press < 2 ? 237 : 0);//volume up

				}
				break;
		   case IR_VOL_DOWN:	//k12
				{
				app_hid_send_keypad_report(p_keypad->press < 2 ? 238 : 0);//volume down
				}
				break;
			case IR_0:	//k13
				{
				app_hid_send_keypad_report(p_keypad->press < 2 ? 0x27 : 0);//0
				}
				break;
			case IR_1:	//k14
				{
				app_hid_send_keypad_report(p_keypad->press < 2 ? 0x1e : 0);// 1
				}
				break;
         case IR_2:	//k15
            {
            app_hid_send_keypad_report(p_keypad->press < 2 ? 0x1f : 0);// 2
            }
         break;
        
         case IR_3:	//k16
            {
            app_hid_send_keypad_report(p_keypad->press < 2 ? 0x20 : 0);// 3
            }
         break;
         case IR_4:	//k17
            {
            app_hid_send_keypad_report(p_keypad->press < 2 ? 0x21 : 0);// 4
            }
         break;
        
        case IR_5:	//k18
           {
           app_hid_send_keypad_report(p_keypad->press < 2 ? 0x22 : 0);// 5
           }
        break;
        case IR_6:	//k19
           {
           app_hid_send_keypad_report(p_keypad->press < 2 ? 0x23 : 0);// 6
           }
        break;
        
        case IR_7:	//k20
           {
           app_hid_send_keypad_report(p_keypad->press < 2 ? 0x24 : 0);// 7
           }
        break;
        
        case IR_8:	//k21
           {
           app_hid_send_keypad_report(p_keypad->press < 2 ? 0x25 : 0);// 8
           }
        break;
        case IR_9:	//k22
           {
           app_hid_send_keypad_report(p_keypad->press < 2 ? 0x26 : 0);// 9
           }
        break;

		  //下面全部设为IR_NEXT ,仅做测试用待keycode确定再调整
        case IR_NEXT:	//k23 ~ k29
           {
           app_hid_send_keypad_report(p_keypad->press < 2 ? 0x04 : 0);// a
           }
        	break;

        case IR_MIC:   //k30
           {
  				if (p_keypad->press < 2)
					{
					//app_hid_send_consumer_key(0x221);//start search
					
					}
				else
					{
   				//gatvs_rc_start_search_handler();
   				//app_hid_send_keypad_report(0);

					
	   	      extern void start_PTT_test(void);
   				start_PTT_test();	      
   							

					}         
           }
        break;

   			
   		default:
   			dbg_print("no report key!!!\r\n");
   			break;
   
   			
   
       	}
   #endif
   
      }
   else
	 	{
 		if (IR_PAIR == p_keypad->key_value)
 			{
 		
			if (p_keypad->press < 2)
				{
          	dbg_print("###ok + back key press\r\n");

   			app_start_pair();	
				}
			else
				{
				dbg_print("ok + back key release\r\n");
				app_stop_pair_timer();
				}
			
			

 			}
		else
			{
         if (p_keypad->press == 2)
            {
            app_reconnect();
            }
			}
	 
	 	dbg_print("ble is not connected!!!\r\n");
	 	}

    return 0;
}


RAM_FUNCTION static int app_voice_raw_data_msg_handler(ke_msg_id_t const msgid,
                            void *p_param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    // Message policy
    uint8_t msg_pol = KE_MSG_CONSUMED;

 /*   //ke_task_id_t src_task_id = MSG_T(msgid);

    //ATINY_LOG(LOG_INFO,"app_voice_raw_data_msg_handler :%d\r\n",src_task_id);

	struct ms_api_app_msg *  p_req_param = ( struct ms_api_app_msg *)p_param;
    
    app_voice_raw_data_handler(p_req_param->p_param); */  // pyxue  remove voice temporary
	
    return msg_pol;
}

#if (BLE_APP_PRESENT)
#if (BLE_APP_PRF)
/**
 ****************************************************************************************
 * @brief Handles GAPM_ACTIVITY_CREATED_IND event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_activity_created_ind_handler(ke_msg_id_t const msgid,
                                             struct gapm_activity_created_ind const *p_param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
   {
    dbg_print2("gapm_activity_created_ind_handler:%x,type:%x\r\n",p_param->actv_idx,p_param->actv_type);
	 
    if (app_env.adv_state == APP_ADV_STATE_CREATING)
       {
        // Store the advertising activity index
        app_env.adv_actv_idx = p_param->actv_idx;
		  if (0 == last_adv_type)
  		  	  {
  		  	  normal_adv_idx = p_param->actv_idx;
  		  	  }
		  else if (1 == last_adv_type)
		  	  {
		  	  direct_adv_idx = p_param->actv_idx;
		  	  }
		 app_adv_fsm_next(); 
       }
	 

    return (KE_MSG_CONSUMED);
   }

/**
 ****************************************************************************************
 * @brief Handles GAPM_ACTIVITY_STOPPED_IND event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_activity_stopped_ind_handler(ke_msg_id_t const msgid,
                                             struct gapm_activity_stopped_ind const *p_param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{	 dbg_print2("%s,actv_idx:%x,actv_type:%x,reason:%x\r\n",__FUNCTION__,p_param->actv_idx,p_param->actv_type,p_param->reason);
    if (app_env.adv_state == APP_ADV_STATE_STARTED)
    {
        // Act as if activity had been stopped by the application
        app_env.adv_state = APP_ADV_STATE_STOPPING;

        // Perform next operation
        app_adv_fsm_next();
    }

    return (KE_MSG_CONSUMED);
}
#endif //(BLE_APP_PRF)

/**
 ****************************************************************************************
 * @brief Handles GAPM_PROFILE_ADDED_IND event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_profile_added_ind_handler(ke_msg_id_t const msgid,
                                          struct gapm_profile_added_ind *p_param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{	 dbg_print2("%s %d\r\n",__FUNCTION__,__LINE__);
    // Current State
    ke_state_t state = ke_state_get(dest_id);

    if (state == APP_CREATE_DB)
    {
        switch (p_param->prf_task_id)
        {
            #if (BLE_APP_AM0)
            case (TASK_ID_AM0):
            {
                app_am0_set_prf_task(p_param->prf_task_nb);
            } break;
            #endif //(BLE_APP_AM0)

            #if (BLE_APP_MESH)
            case (TASK_ID_MESH):
            {
                app_mesh_set_prf_task(p_param->prf_task_nb);
            } break;
            #endif //(BLE_APP_MESH)

            default: /* Nothing to do */ break;
        }
    }
    else
    {
        ASSERT_INFO(0, state, src_id);
    }

    return (KE_MSG_CONSUMED);
}


static int gapc_get_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_get_dev_info_req_ind const *p_param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{	 dbg_print2("%s %d\r\n",__FUNCTION__,__LINE__);
    switch(p_param->req)
    {

        case GAPC_DEV_NAME:
        {

            struct gapc_get_dev_info_cfm * cfm = KE_MSG_ALLOC_DYN(GAPC_GET_DEV_INFO_CFM,
                                                    src_id, dest_id,
                                                    gapc_get_dev_info_cfm, APP_DEVICE_NAME_MAX_LEN);
		    cfm->conidx = p_param->conidx;
            cfm->req = p_param->req;
            cfm->token = p_param->token;
            cfm->status = GAP_ERR_NO_ERROR;
            cfm->info.name.value_length = app_get_dev_name(cfm->info.name.value);

            // Send message
            ke_msg_send(cfm);

        } break;

        case GAPC_DEV_APPEARANCE:
        {
            // Allocate message
            struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
                                                             src_id, dest_id,
                                                             gapc_get_dev_info_cfm);
            cfm->req = p_param->req;
            cfm->token = p_param->token;
            cfm->status = GAP_ERR_NO_ERROR;

            // Set the device appearance
            #if (BLE_APP_HT)
            // Generic Thermometer - TODO: Use a flag
            cfm->info.appearance = 728;
            #elif (BLE_APP_HID)
            // HID 
            //cfm->info.appearance = 961;
				cfm->info.appearance = 384;
            #elif (BLE_APP_MESH)
            cfm->info.appearance = 800;
            #else
            // No appearance
            cfm->info.appearance = 0;
            #endif

            // Send message
            ke_msg_send(cfm);
        } break;

        case GAPC_DEV_SLV_PREF_PARAMS:
        {
            // Allocate message
            struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
                    src_id, dest_id,
                                                            gapc_get_dev_info_cfm);
            cfm->req = p_param->req;
            // Slave preferred Connection interval Min
            cfm->info.slv_pref_params.con_intv_min = 6;
            // Slave preferred Connection interval Max
            cfm->info.slv_pref_params.con_intv_max = 12;
            // Slave preferred Connection latency
            cfm->info.slv_pref_params.latency  = 49;  
            // Slave preferred Link supervision timeout
            cfm->info.slv_pref_params.conn_timeout    = 300;  // 2s (500*10ms)

            // Send message
            ke_msg_send(cfm);
        } break;

        default: /* Do Nothing */ break;
    }

    return (KE_MSG_CONSUMED);
}

#if (BLE_APP_PRF)
/**
 ****************************************************************************************
 * @brief Handles GAPC_SET_DEV_INFO_REQ_IND message.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_set_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_set_dev_info_req_ind const *p_param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{	 dbg_print2("%s %d\r\n",__FUNCTION__,__LINE__);
    // Set Device configuration
    struct gapc_set_dev_info_cfm* cfm = KE_MSG_ALLOC(GAPC_SET_DEV_INFO_CFM, TASK_GAPC, dest_id, gapc_set_dev_info_cfm);
    // Reject to change parameters
    cfm->conidx = p_param->conidx;
    cfm->status = GAP_ERR_REJECTED;
    cfm->req = p_param->req;
    // Send message
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

#endif //(BLE_APP_PRF)

/**
 ****************************************************************************************
 * @brief Handles connection complete event from the GAP. Enable all required profiles
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_le_connection_req_ind_handler(ke_msg_id_t const msgid,
                                           struct gapc_le_connection_req_ind const *p_param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
	 

    
    app_env.conidx = p_param->conidx;

    {
    dbg_print2("peer_type:%x,peer_addr:\r\n",p_param->peer_addr_type);
    for(int i = GAP_BD_ADDR_LEN-1; i >=0; --i)
       {
       dbg_print2("%02X ", p_param->peer_addr.addr[i]);
       }
    dbg_print2(" \r\n");
    }
	 dbg_print2("gapc_le_connection_req_ind_handler:%x\r\n",app_env.conidx);


    // Check if the received connection index is valid
    if (app_env.conidx != GAP_INVALID_CONIDX)
    {
        // Allocate connection confirmation
        struct gapc_connection_cfm *cfm = KE_MSG_ALLOC(GAPC_CONNECTION_CFM, TASK_GAPC, TASK_APP, gapc_connection_cfm);

		  app_set_connection_state(CONNECTED_STATE);

        // Store received connection handle
        app_env.conhdl = p_param->conhdl;

        cfm->conidx = p_param->conidx;//app_env.conidx;
        #if (BLE_APP_SEC)
        //cfm->bond_data.pairing_lvl = GAP_PAIRING_BOND_SECURE_CON;//
		  cfm->bond_data.pairing_lvl = GAP_PAIRING_BOND_AUTH;//GAP_PAIRING_BOND_UNAUTH;//GAP_PAIRING_UNAUTH; //GAP_SEC_UNAUTH,just work
        #else // !(BLE_APP_SEC)
		  
        cfm->auth      = GAP_AUTH_REQ_MITM_BOND;//GAP_AUTH_REQ_NO_MITM_BOND;//GAP_AUTH_REQ_NO_MITM_NO_BOND; 
        #endif // (BLE_APP_SEC)

        // Send the message
        ke_msg_send(cfm);

        #if DISPLAY_SUPPORT
        #if (BLE_APP_PRF)
        // Update displayed information
        app_display_set_adv(false);
        app_display_set_con(true);

        #elif (BLE_APP_MESH)
        app_display_mesh_set_con(true);
        #endif //(BLE_APP_PRF)
        #endif //(DISPLAY_SUPPORT)

        /*--------------------------------------------------------------
         * ENABLE REQUIRED PROFILES
         *--------------------------------------------------------------*/

        #if (BLE_APP_BATT)
        // Enable Battery Service
        app_batt_enable_prf(app_env.conhdl);
        #endif //(BLE_APP_BATT)

#if (BLE_APP_HT)
        // Enable Battery Service
        app_ht_enable_prf(app_env.conidx);
#endif //(BLE_APP_BATT)

        #if (BLE_APP_HID)
        // Enable HID Service
        app_hid_enable_prf(app_env.conidx);
        #endif //(BLE_APP_HID)

        // We are now in connected State
        ke_state_set(dest_id, APP_CONNECTED);

        #if (BLE_APP_SEC && !defined(BLE_APP_AM0))
        if (app_sec_get_bond_status())
        {
            // Ask for the peer device to either start encryption
            app_sec_send_security_req(app_env.conidx);
        }
        #endif // (BLE_APP_SEC && !defined(BLE_APP_AM0))
    }
    else
    {
        #if (BLE_APP_PRF)
        // No connection has been established, restart advertising
        app_update_adv_state(true);

        #endif //(BLE_APP_PRF)
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles connection complete event from the GAP. Enable all required profiles
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_param_update_req_ind_handler(ke_msg_id_t const msgid,
                                           struct gapc_param_update_req_ind const *p_param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{	 dbg_print2("%s %d\r\n",__FUNCTION__,__LINE__);
    app_env.conidx = p_param->conidx;

    // Check if the received Connection Handle was valid
    if (app_env.conidx != GAP_INVALID_CONIDX)
    {
        // Send connection confirmation
        struct gapc_param_update_cfm *cfm = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CFM, TASK_GAPC, TASK_APP, gapc_param_update_cfm);
        cfm->conidx = app_env.conidx;
        cfm->accept = true;
        cfm->ce_len_min = 2; //10ms
        cfm->ce_len_max = 8;

        // Send message
        ke_msg_send(cfm);
    }
    else
    {
#if (BLE_APP_PRF)
        // No connection has been established, restart advertising
        app_update_adv_state(true);
#endif // (BLE_APP_PRF)
    }

    return (KE_MSG_CONSUMED);
}

#if (BLE_APP_PRF)
/**
 ****************************************************************************************
 * @brief Handles GAP controller command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int app_gapc_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapc_cmp_evt const *p_param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{	 dbg_print2("%s %d\r\n",__FUNCTION__,__LINE__);
    switch(p_param->operation)
    {
        case (GAPC_UPDATE_PARAMS):
        {
            if (p_param->status != GAP_ERR_NO_ERROR)
            {
//              app_disconnect();
            }
        }
        break;

        default:
        {
        }
        break;
    }

    return (KE_MSG_CONSUMED);
}
#endif //(BLE_APP_PRF)

/**
 ****************************************************************************************
 * @brief Handles disconnection complete event from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_disconnect_ind_handler(ke_msg_id_t const msgid,
                                      struct gapc_disconnect_ind const *p_param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
#ifdef PM_DEBUG_BLE_DEEP_SLEEP
    extern int ms_sys_check_if_enter_sleep();
#endif
    app_set_connection_state(DISCONNECTED_STATE);

    // Go to the ready state
    ke_state_set(TASK_APP, APP_READY);
    #if (DISPLAY_SUPPORT)
    #if (BLE_APP_MESH)
    // Update Connection State screen
    app_display_mesh_set_con(false);
    #elif (BLE_APP_PRF)
    app_display_set_con(false);
    #endif //(BLE_APP_MESH)
    #endif //(DISPLAY_SUPPORT)

    #if (BLE_APP_PRF)
    #if (BLE_APP_HT)
    // Stop interval timer
    app_stop_timer();
    #endif //(BLE_APP_HT)

    #if (BLE_ISO_MODE_0_PROFILE)
    app_env.adv_state = APP_ADV_STATE_CREATING;
    #endif //(BLE_ISO_MODE_0_PROFILE)

    //#if (!BLE_APP_HID)
    // Restart Advertising
  	 if (((APP_ADV_STATE_CREATED == app_env.adv_state)||(APP_ADV_STATE_IDLE == app_env.adv_state))
#ifdef PM_DEBUG_BLE_DEEP_SLEEP
	 &&(0 == ms_sys_check_if_enter_sleep())
#endif
	 )
        
		{
		app_update_adv_state(true);
		}  

    //#endif //(!BLE_APP_HID)
    #endif //(BLE_APP_PRF)
    return (KE_MSG_CONSUMED);
}



#if (BLE_APP_PRF)
/**
 ****************************************************************************************
 * @brief Handles reception of random number generated message
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_gen_rand_nb_ind_handler(ke_msg_id_t const msgid, struct gapm_gen_rand_nb_ind *p_param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{   dbg_print2("gapm_gen_rand_nb_ind_handler,gen irk success");

    if (app_env.rand_cnt==1)      // First part of IRK
    {
        memcpy(&app_env.loc_irk[0], &p_param->randnb[0], 8);
       {
       uint8_t i;
       for (i=0;i<8;i++)
         {
         
         dbg_print2("%x,",app_env.loc_irk[i]);
         }
       dbg_print2("\r\n---------irk1-------\r\n");
       }  
    }
    else if (app_env.rand_cnt==2) // Second part of IRK
    {
        memcpy(&app_env.loc_irk[8], &p_param->randnb[0], 8);
        {
        uint8_t i;
        for (i=0;i<8;i++)
           {
          
           dbg_print2("%x,",app_env.loc_irk[i+8]);
           }
        dbg_print2("\r\n--------irk2--------\r\n");
        }

		  
    }
    return (KE_MSG_CONSUMED);
}

#endif //(BLE_APP_PRF)

__STATIC int gapc_param_updated_ind_handler(ke_msg_id_t const msgid,
                             struct gapc_param_updated_ind *p_param,
                                         ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{   dbg_print2("%s %d\r\n",__FUNCTION__,__LINE__);
    return (KE_MSG_CONSUMED);
}

__STATIC int gapc_le_pkt_size_ind_handler(ke_msg_id_t const msgid,
                             struct gapc_le_pkt_size_ind *p_param,
                                       ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
	 dbg_print2("%s %d\r\n",__FUNCTION__,__LINE__);
    return (KE_MSG_CONSUMED);
}

__STATIC int gapc_bond_data_ind_handler(ke_msg_id_t const msgid,
        struct gapc_le_pkt_size_ind *p_param,
                  ke_task_id_t const dest_id,
                   ke_task_id_t const src_id)
{   dbg_print2("%s %d\r\n",__FUNCTION__,__LINE__);
    return (KE_MSG_CONSUMED);
}

#endif // (BLE_APP_PRESENT)
RAM_FUNCTION uint16_t ms_api_send_app_msg(ke_msg_id_t msgid, void *p_param)
{
    // Allocate the BASS_CREATE_DB_REQ
    
    struct ms_api_app_msg *req = KE_MSG_ALLOC(msgid,
                            TASK_APP, TASK_NONE,
                            ms_api_app_msg);

  
    req->operation = msgid;
    req->p_param = p_param;
   
    //ATINY_LOG(LOG_INFO,"ms_api_send_app_msg !!");	

    // Send the message
    ke_msg_send(req);
    return 0;

}

/*
 * GLOBAL VARIABLES DEFINITION
 ****************************************************************************************
 */

/* Default State handlers definition. */
KE_MSG_HANDLER_TAB(app)
{
    // Note: all messages must be sorted in ID ascending order
    {GAPM_CMP_EVT,              (ke_msg_func_t)gapm_cmp_evt_handler},

    #if (BLE_APP_PRESENT)
    // GAPM messages
    #if (BLE_APP_PRF)
    {GAPM_GEN_RAND_NB_IND,      (ke_msg_func_t)gapm_gen_rand_nb_ind_handler},
    {GAPM_ACTIVITY_CREATED_IND, (ke_msg_func_t)gapm_activity_created_ind_handler},
    {GAPM_ACTIVITY_STOPPED_IND, (ke_msg_func_t)gapm_activity_stopped_ind_handler},
    #endif //(BLE_APP_PRF)

    {GAPM_PROFILE_ADDED_IND,    (ke_msg_func_t)gapm_profile_added_ind_handler},

    #if (BLE_APP_PRF)
    {GAPC_CMP_EVT,              (ke_msg_func_t)app_gapc_cmp_evt_handler},
    #endif //(BLE_APP_PRF)

    // GAPC messages
    {GAPC_LE_CONNECTION_REQ_IND,   (ke_msg_func_t)gapc_le_connection_req_ind_handler},
    {GAPC_DISCONNECT_IND,       (ke_msg_func_t)gapc_disconnect_ind_handler},
    {GAPC_GET_DEV_INFO_REQ_IND, (ke_msg_func_t)gapc_get_dev_info_req_ind_handler},

    #if (BLE_APP_PRF)
    {GAPC_SET_DEV_INFO_REQ_IND, (ke_msg_func_t)gapc_set_dev_info_req_ind_handler},
    #endif //(BLE_APP_PRF)

    {GAPC_PARAM_UPDATE_REQ_IND, (ke_msg_func_t)gapc_param_update_req_ind_handler},
    {GAPC_PARAM_UPDATED_IND,    (ke_msg_func_t)gapc_param_updated_ind_handler},

    {GAPC_LE_PKT_SIZE_IND,      (ke_msg_func_t)gapc_le_pkt_size_ind_handler},

    {GAPC_BOND_DATA_UPDATE_IND, (ke_msg_func_t)gapc_bond_data_ind_handler},

    #endif // (BLE_APP_PRESENT)
   
    {APP_KEYPAD_MSG,    (ke_msg_func_t)app_keypad_msg_handler},
	{APP_MSG_VOICE_RAW,    (ke_msg_func_t)app_voice_raw_data_msg_handler},
   
    {KE_MSG_DEFAULT_HANDLER,    (ke_msg_func_t)app_msg_handler},
};

/* Defines the place holder for the states of all the task instances. */
ke_state_t app_state[APP_IDX_MAX];

// Application task descriptor
const struct ke_task_desc TASK_DESC_APP = {app_msg_handler_tab, app_state, APP_IDX_MAX, ARRAY_LEN(app_msg_handler_tab)};

#endif //(APP_PRESENT)

/// @} APPTASK
