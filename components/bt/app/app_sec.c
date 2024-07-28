/**
 ****************************************************************************************
 *
 * @file app_sec.c
 *
 * @brief Application Security Entry Point
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

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "rwapp_config.h"
#if (BLE_APP_SEC)

#include <string.h>
#include "co_utils.h"
#include "co_math.h"

#include "gap.h"
#include "gapc.h"
#include "gapc_int.h" // TODO [FBE] GAPC internal shall not be included
#include "atiny_log.h"
#include <stdio.h>
#include <string.h>  


#include "gapc_msg.h"
#include "gapm.h"

#include "prf_types.h"
#include "app_ht.h"
#include "app_hid.h"


#include "app.h"            // Application API Definition
#include "app_sec.h"        // Application Security API Definition
#include "app_task.h"       // Application Manager API Definition

#if (DISPLAY_SUPPORT)
#include "app_display.h"    // Display Application Definitions
#endif //(DISPLAY_SUPPORT)

#if (NVDS_SUPPORT)
#include "nvds.h"           // NVDS API Definitions
#endif //(NVDS_SUPPORT)

#if (BLE_APP_AM0)
#include "app_am0.h"
#endif //(BLE_APP_AM0)

#include "gapc_le_msg.h"


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Application Security Environment Structure
struct app_sec_env_tag app_sec_env;
extern uint8_t app_loc_irk[KEY_LEN];
/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void app_sec_init()
{
    /*------------------------------------------------------
     * RETRIEVE BOND STATUS
     *------------------------------------------------------*/
    #if (NVDS_SUPPORT)
    uint8_t length = NVDS_LEN_PERIPH_BONDED;

    // Get bond status from NVDS
    if (nvds_get(NVDS_TAG_PERIPH_BONDED, &length, (uint8_t *)&app_sec_env.bonded) != NVDS_OK)
    {
        // If read value is invalid, set status to not bonded
        app_sec_env.bonded = false;
    }

    if ((app_sec_env.bonded != true) && (app_sec_env.bonded != false))
    {
        app_sec_env.bonded = false;
    }
    app_nv_get_bonded_device_info(&bonded_dev_info);

    #if (DISPLAY_SUPPORT)
    // Update the bond status screen value
    app_display_set_bond(app_sec_env.bonded);
    #endif //(DISPLAY_SUPPORT)
    #endif //(NVDS_SUPPORT)
}

bool app_sec_get_bond_status(void)
{
    return app_sec_env.bonded;
}

#if (NVDS_SUPPORT)
void app_sec_remove_bond(void)
{
    #if (BLE_APP_HID)
    //uint16_t ntf_cfg = PRF_CLI_STOP_NTFIND;
    #endif //(BLE_APP_HID)

    // Check if we are well bonded
    if (app_sec_env.bonded == true)
    {
        // Update the environment variable
        app_sec_env.bonded = false;

        if (nvds_put(NVDS_TAG_PERIPH_BONDED, NVDS_LEN_PERIPH_BONDED,
                     (uint8_t *)&app_sec_env.bonded) != NVDS_OK)
        {
            ASSERT_ERR(0);
        }

        #if (BLE_APP_HT)
        if (nvds_del(NVDS_TAG_LTK) != NVDS_OK)
        {
            ASSERT_ERR(0);
        }

        if (nvds_del(NVDS_TAG_PEER_BD_ADDRESS) != NVDS_OK)
        {
            ASSERT_ERR(0);
        }
        #endif //(BLE_APP_HT)

    }
}
#endif //(NVDS_SUPPORT)

void app_sec_send_security_req(uint8_t conidx)
{
    // Send security request
    struct gapc_security_cmd *cmd = KE_MSG_ALLOC(GAPC_SECURITY_CMD, TASK_GAPC, TASK_APP, gapc_security_cmd);
    cmd->conidx    = conidx;
    cmd->operation = GAPC_SECURITY_REQ;

    //#if (BLE_APP_HID || BLE_APP_HT)
    //cmd->auth      = GAP_AUTH_REQ_MITM_BOND;
   // #elif defined(BLE_APP_AM0)
    //cmd->auth      = GAP_AUTH_REQ_NO_MITM_BOND;
   // #else
    cmd->auth      = GAP_AUTH_REQ_NO_MITM_NO_BOND;
   // #endif //(BLE_APP_HID || BLE_APP_HT)

    // Send the message
    ke_msg_send(cmd);
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

static int gapc_bond_req_ind_handler(ke_msg_id_t const msgid,
                                     struct gapc_bond_req_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    // Prepare the GAPC_BOND_CFM message
    struct gapc_bond_cfm *cfm = KE_MSG_ALLOC(GAPC_BOND_CFM,
                                             src_id, TASK_APP,
                                             gapc_bond_cfm);
    cfm->conidx = param->conidx;

    switch (param->request)
    {
        case (GAPC_PAIRING_REQ):
        {
            cfm->request = GAPC_PAIRING_RSP;

            #ifndef BLE_APP_AM0
            cfm->accept  = false;

            // Check if we are already bonded (Only one bonded connection is supported)
            if (!app_sec_env.bonded)
            #endif // !BLE_APP_AM0
            {
                cfm->accept  = true;

                #if (BLE_APP_HID || BLE_APP_HT)
                // Pairing Features
                //cfm->data.pairing_feat.pairing_info.auth      = GAP_AUTH_REQ_MITM_BOND;//modified by liujin
                
             
                cfm->data.pairing_feat.pairing_info.auth      = GAP_AUTH_REQ_NO_MITM_BOND;
                                                              
	
                #else
                cfm->data.pairing_feat.pairing_info.auth      = GAP_AUTH_REQ_NO_MITM_NO_BOND;
                #endif //(BLE_APP_HID || BLE_APP_HT)
               
                //cfm->data.pairing_feat.pairing_info.iocap     = GAP_IO_CAP_DISPLAY_ONLY;  //modified 
             
                cfm->data.pairing_feat.pairing_info.iocap     = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
             

            
                dbg_print2("GAPC_PAIRING_REQ,%x,%x,%x\r\n",param->data.auth_req,param->data.key_size,param->data.tk_type);//GAP_AUTH_SEC_CON

                cfm->data.pairing_feat.pairing_info.key_size  = 16;  
                cfm->data.pairing_feat.pairing_info.oob       = GAP_OOB_AUTH_DATA_NOT_PRESENT;
                cfm->data.pairing_feat.sec_req_level = GAP_SEC1_NOAUTH_PAIR_ENC; 
             
                cfm->data.pairing_feat.pairing_info.rkey_dist = GAP_KDIST_ENCKEY | GAP_KDIST_IDKEY; 
               
                cfm->data.pairing_feat.pairing_info.ikey_dist = GAP_KDIST_ENCKEY | GAP_KDIST_IDKEY;
              
   

					 
               
            }
        } break;

        case (GAPC_LTK_EXCH):
        {
            // Counter
            uint8_t counter;

            cfm->accept  = true;
            cfm->request = GAPC_LTK_EXCH;

            // Generate all the values
            cfm->data.ltk.ediv = (uint16_t)co_rand_word();

            for (counter = 0; counter < RAND_NB_LEN; counter++)
            {
                cfm->data.ltk.key.key[counter]    = (uint8_t)co_rand_word();//modified by liujin
                cfm->data.ltk.randnb.nb[counter] = (uint8_t)co_rand_word();
            }

            for (counter = RAND_NB_LEN; counter < KEY_LEN; counter++)
            {
                cfm->data.ltk.key.key[counter]    = (uint8_t)co_rand_word();//
            }

            #if (NVDS_SUPPORT)
            // Store the generated value in NVDS
            if (nvds_put(NVDS_TAG_LTK, NVDS_LEN_LTK,
                         (uint8_t *)&cfm->data.ltk) != NVDS_OK)
            {
                ASSERT_ERR(0);
            }
            #endif // #if (NVDS_SUPPORT)
	         if (bonded_dev_info.current_dev_index < MAX_BONDED_DEV_INDEX)
               {
                   bonded_dev_info.current_dev_index++;
               }
            else if (bonded_dev_info.current_dev_index == MAX_BONDED_DEV_INDEX)
               {
                   bonded_dev_info.current_dev_index = 0;
               }			
            memcpy(bonded_dev_info.bonded_device_info[bonded_dev_info.current_dev_index].peer_addr, peer_dbaddr, APP_BD_ADDR_LEN);
            memcpy(bonded_dev_info.bonded_device_info[bonded_dev_info.current_dev_index].ltk.ltk,  cfm->data.ltk.key.key, GAP_KEY_LEN);
            memcpy(bonded_dev_info.bonded_device_info[bonded_dev_info.current_dev_index].ltk.randnb, cfm->data.ltk.randnb.nb, 8);
            bonded_dev_info.bonded_device_info[bonded_dev_info.current_dev_index].ltk.ediv = cfm->data.ltk.ediv;
			#if (NVDS_SUPPORT)	
            app_nv_set_bonded_device_info(&bonded_dev_info);
			#endif

				
        } break;


        case (GAPC_IRK_EXCH):
        {
            #if (NVDS_SUPPORT)
            uint8_t addr_len = BD_ADDR_LEN;
            #endif //(NVDS_SUPPORT)

            cfm->accept  = true;
            cfm->request = GAPC_IRK_EXCH;

            // Load IRK
            // memcpy(cfm->data.irk.key.key, app_env.loc_irk, KEY_LEN);
			memcpy(cfm->data.irk.key.key, app_loc_irk, KEY_LEN);//modified by liujin

            #if (NVDS_SUPPORT)
            if (nvds_get(NVDS_TAG_BD_ADDRESS, &addr_len, cfm->data.irk.addr.addr) != NVDS_OK)
            #endif //(NVDS_SUPPORT)
            {
                ASSERT_ERR(0);
            }
            // load device address
            cfm->data.irk.addr.addr_type = (cfm->data.irk.addr.addr[5] & 0xC0) ? ADDR_RAND : ADDR_PUBLIC;

				
        } break;

                           
        #if 1 //(BLE_APP_HT)    
        case (GAPC_TK_EXCH):
        {                   
            // Generate a PIN Code- (Between 100000 and 999999)
            uint32_t pin_code = (100000 + (co_rand_word()%900000));
			dbg_print2("***GAPC_TK_EXCH,%d\r\n",pin_code);

            #if DISPLAY_SUPPORT
            // Display the PIN Code
            app_display_pin_code(pin_code);
            #endif //DISPLAY_SUPPORT


            cfm->accept  = true;
            cfm->request = GAPC_TK_EXCH;

            // Set the TK value
            memset(cfm->data.tk.key, 0, KEY_LEN);

            cfm->data.tk.key[0] = (uint8_t)((pin_code & 0x000000FF) >>  0);
            cfm->data.tk.key[1] = (uint8_t)((pin_code & 0x0000FF00) >>  8);
            cfm->data.tk.key[2] = (uint8_t)((pin_code & 0x00FF0000) >> 16);
            cfm->data.tk.key[3] = (uint8_t)((pin_code & 0xFF000000) >> 24);
        } break;

		  /// Numeric Comparison - Exchange of Numeric Value -
		                   
		  case GAPC_NC_EXCH: 
		  	 {
	         dbg_print("error! GAPC_NC_EXCH,no process!!!\r\n\r\n");
        		cfm->accept  = true;
		    }
         break;

         
         case GAPC_CSRK_EXCH:
       	 {
			
          dbg_print("GAPC_CSRK_EXCH,no process!!!\r\n\r\n");
          cfm->accept  = true;
         
          //return;
       		 
        }
       break;		
		  
        #endif //(BLE_APP_HT)

        default:
        {
            ASSERT_ERR(0);
        } break;
    }

    // Send the message
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

static int gapc_bond_ind_handler(ke_msg_id_t const msgid,
                                 struct gapc_bond_ind const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    switch (param->info)
    {
        case (GAPC_PAIRING_SUCCEED):
        {
		  	   dbg_print("\r\n bond success!!!\r\n");
            // Update the bonding status in the environment
            app_sec_env.bonded = true;

            // Update the bonding status in the environment
            #if (PLF_NVDS)
            if (nvds_put(NVDS_TAG_PERIPH_BONDED, NVDS_LEN_PERIPH_BONDED,
                         (uint8_t *)&app_sec_env.bonded) != NVDS_OK)
            {
                // An error has occurred during access to the NVDS
                ASSERT_ERR(0);
            }

            // Set the BD Address of the peer device in NVDS
            if (nvds_put(NVDS_TAG_PEER_BD_ADDRESS, NVDS_LEN_PEER_BD_ADDRESS,
                         (uint8_t *)gapc_le_get_bdaddr(0, GAPC_INFO_SRC_PEER)) != NVDS_OK)
            {
                // An error has occurred during access to the NVDS
                ASSERT_ERR(0);
            }

            #endif //(PLF_NVDS)



        } break;

        case (GAPC_REPEATED_ATTEMPT):
        {
            app_disconnect();
        } break;

        case (GAPC_IRK_EXCH):
        {
           #if (NVDS_SUPPORT)
           // Store peer identity in NVDS
           if (nvds_put(NVDS_TAG_PEER_IRK, NVDS_LEN_PEER_IRK, (uint8_t *)&param->data.irk) != NVDS_OK)
           {
               ASSERT_ERR(0);
           }
           #endif // (NVDS_SUPPORT)
        } break;

        case (GAPC_PAIRING_FAILED):
        {
		    dbg_print("GAPC_PAIRING_FAILED!\r\n");
            app_sec_send_security_req(0);
        } break;

        // In Secure Connections we get BOND_IND with SMPC calculated LTK
        case (GAPC_LTK_EXCH) :
        {
            if (app_env.sec_con_enabled == true)
            {
                #if 0 //(NVDS_SUPPORT)
                // Store LTK in NVDS
                if (nvds_put(NVDS_TAG_LTK, NVDS_LEN_LTK,(uint8_t *)&param->data.ltk.ltk.key[0]) != NVDS_OK)
                {
                    ASSERT_ERR(0);
                }
                #endif // (NVDS_SUPPORT)
            }
 
        }
        break;
       case GAPC_CSRK_EXCH:
          {
                
          }
		  break;
		  case GAPC_NC_EXCH: 
		  	 {
		  	 }
		  break;

        default:
        {
            ASSERT_ERR(0);
        } break;
    }

    return (KE_MSG_CONSUMED);
}

static int gapc_le_encrypt_req_ind_handler(ke_msg_id_t const msgid,
                                        struct gapc_le_encrypt_req_ind const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    #if (NVDS_SUPPORT)
    // LTK value
 //   gapc_ltk_t ltk;
    // Length
//    uint8_t length = NVDS_LEN_LTK;
    #endif // #if (NVDS_SUPPORT)

    // Prepare the GAPC_LE_ENCRYPT_CFM message
    struct gapc_le_encrypt_cfm *cfm = KE_MSG_ALLOC(GAPC_LE_ENCRYPT_CFM,
                                                src_id, TASK_APP,
                                                gapc_le_encrypt_cfm);

    cfm->found    = false;

    if (app_sec_env.bonded)
       {
       #if 0 //(NVDS_SUPPORT)
       // Retrieve the required informations from NVDS
       if (nvds_get(NVDS_TAG_LTK, &length, (uint8_t *)&ltk) == NVDS_OK)
       {
           // Check if the provided EDIV and Rand Nb values match with the stored values
           if ((param->ediv == ltk.ediv) &&
               !memcmp(&param->rand_nb.nb[0], &ltk.randnb.nb[0], sizeof(struct rand_nb)))
           {
               cfm->found    = true;
               cfm->key_size = 16;
               memcpy(&cfm->ltk.key, &ltk.key.key, GAP_KEY_LEN);
           }
           /*
            * else we are bonded with another device, disconnect the link
            */
       }
       else
       {
           ASSERT_ERR(0);
       }
  	  #else
	  //add 1215  
     // Check if the provided EDIV and Rand Nb values match with the stored values
     
#if (NVDS_SUPPORT)	
	  if (app_nv_get_bonded_device_info(&bonded_dev_info))
#endif  	
	  {
	  
		int8_t index = app_check_device_isbonded(param->ediv, param->rand_nb.nb);
      if (index > INVALID_BONDED_INDEX)
         {
         cfm->found    = true;
         cfm->key_size = 16;
         memcpy(cfm->ltk.key, &bonded_dev_info.bonded_device_info[index].ltk.ltk, GAP_KEY_LEN);        
          for (int i = 0; i < GAP_KEY_LEN; ++i)
          { 
          }
          
          }
      else
          {
          
          memset(cfm->ltk.key, 0, GAP_KEY_LEN); 
          cfm->key_size = 16;
          cfm->found    = false;
          }	
	  	}
#if (NVDS_SUPPORT)
	  else
#endif 	  	
	  	{
	  	dbg_print("encrypt read nvds error\r\n"); 
	  	}
  
     #endif // #if (NVDS_SUPPORT)
   		  
       }
	 else
	 	{
		 dbg_print("APP_CB: %s, not bound, send encrypt confirm\r\n", __FUNCTION__);
		 
		 memset(cfm->ltk.key, 0, GAP_KEY_LEN); 
		 cfm->key_size = 16;
		 cfm->found 	= false;
	 	
	 	}
    /*
     * else the peer device is not known, an error should trigger a new pairing procedure.
     */

    // Send the message
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}


static int gapc_encrypt_ind_handler(ke_msg_id_t const msgid,
                                    struct gapc_encrypt_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // encryption/ re-encryption succeeded

    #if (BLE_APP_AM0)

    // Need to Setup Authenicated Payload TO for the connection.
    struct gapc_set_le_ping_to_cmd *cmd = KE_MSG_ALLOC(GAPC_SET_LE_PING_TO_CMD, TASK_GAPC, TASK_APP,
                                                       gapc_set_le_ping_to_cmd);

    // encryption/ re-encryption succeeded
    cmd->conidx = param->conidx;
    cmd->operation = GAPC_SET_LE_PING_TO;
    cmd->timeout = 1000; // 10 Sec

    // Send the message
    ke_msg_send(cmd);

    app_am0_send_audio_init(param->conidx);

    #endif //(BLE_APP_AM0)
    return (KE_MSG_CONSUMED);
}

static int app_sec_msg_dflt_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // Drop the message

    return (KE_MSG_CONSUMED);
}

 /*
  * LOCAL VARIABLE DEFINITIONS
  ****************************************************************************************
  */

/// Default State handlers definition
const struct ke_msg_handler app_sec_msg_handler_list[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,  (ke_msg_func_t)app_sec_msg_dflt_handler},

    {GAPC_BOND_REQ_IND,       (ke_msg_func_t)gapc_bond_req_ind_handler},
    {GAPC_BOND_IND,           (ke_msg_func_t)gapc_bond_ind_handler},

    {GAPC_LE_ENCRYPT_REQ_IND,    (ke_msg_func_t)gapc_le_encrypt_req_ind_handler},
    {GAPC_ENCRYPT_IND,        (ke_msg_func_t)gapc_encrypt_ind_handler},
};

const struct app_subtask_handlers app_sec_handlers = {&app_sec_msg_handler_list[0], ARRAY_LEN(app_sec_msg_handler_list)};

#endif //(BLE_APP_SEC)

/// @} APP
