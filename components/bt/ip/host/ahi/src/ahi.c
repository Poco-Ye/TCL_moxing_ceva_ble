/**
 ****************************************************************************************
 *
 * @file ahi.c
 *
 * @brief This file contains definitions related to the Application Host Interface
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup AHI
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (AHI_TL_SUPPORT)
#include "ahi.h"
#include "ke_task.h"
#include "ke_mem.h"
#include "co_bt.h"           // BT standard definitions
#include "co_list.h"
#include "co_utils.h"
#include "h4tl.h"            // H4 Transport Layer

#include "gapm.h"
#if(BLE_HOST_PRESENT)
#include "gapm_le_msg.h"
#endif // (BLE_HOST_PRESENT)
#if(BT_HOST_PRESENT)
#include "gapm_bt_msg.h"
#endif // (BT_HOST_PRESENT)


#if (BLE_GAF_PRESENT)
#include "gaf_inc.h"         // Generic Audio Framework Definitions
#endif //(BLE_GAF_PRESENT)
#if(HOST_PROFILES)
#include "prf_utils.h"
#endif // (HOST_PROFILES)

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of instances of the AHI task
#define AHI_IDX_MAX 1

/// GAPM Reset Message use to resync.
#define AHI_RESET_MSG_BUF    {AHI_KE_MSG_TYPE,                                           \
                              (GAPM_RESET_CMD & 0xFF), ((GAPM_RESET_CMD >> 8) & 0xFF),   \
                              (TASK_ID_GAPM   & 0xFF), ((TASK_ID_GAPM   >> 8) & 0xFF),   \
                              (TASK_ID_AHI    & 0xFF), ((TASK_ID_AHI    >> 8) & 0xFF),   \
                              0x01, 0x00, GAPM_RESET};

/*
 * STATES
 ****************************************************************************************
 */
/// Possible states of the AHI task
enum AHI_STATE
{
    /// TX IDLE state
    AHI_TX_IDLE,
    /// TX ONGOING state
    AHI_TX_ONGOING,
    /// Number of states.
    AHI_STATE_MAX
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

///Structure for application system interface packet header
typedef struct ahi_kemsghdr
{
    ///Message id
    uint16_t id;
    ///Destination task identifier for KE
    uint16_t dest_id;
    ///Source task identifier for KE
    uint16_t src_id;
    ///Message parameter length
    uint16_t param_len;
} ahi_kemsghdr_t;

/// Data structure that describe packet to send
struct ahi_tx_data
{
    /// list element
    struct co_list_hdr hdr;
    /// Message data pointer to send
    uint8_t*           data;
    /// TX callback to be call when TX is over
    void               (*tx_callback)(uint8_t*);
    /// Data length to send.
    uint16_t           len;
    /// Message type
    uint8_t            msg_type;
};


///AHI Environment context structure
struct ahi_env_tag
{
    /// list of TX buffer in pending queue
    struct co_list      tx_queue;

    /// message wich is currently TX.
    struct ahi_tx_data* tx_ptr;
    /// Number of advertising reports in transmission queue.
    uint8_t             adv_report_nb;
};


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern const struct ke_task_desc TASK_DESC_AHI;


/// AHI environment context
struct ahi_env_tag ahi_env;


const uint8_t ahi_reset_msg[AHI_RESET_MSG_LEN] = AHI_RESET_MSG_BUF;

/*
 * LOCAL FUNCTION DECLARTIONS
 ****************************************************************************************
 */

__STATIC void ahi_h4tl_send(struct ahi_tx_data *tx_data);

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Actions after external interface TX.
 *
 * Analyzes the status value and sets the AHI environment state to TX_DONE/ERR
 * accordingly. This allows the higher function calling write to have feedback
 * and decide the following action (repeat/abort tx in case of error, continue otherwise).
 *
 * Function called after sending message through external interface, to free ke_msg space and
 * push the next message for transmission if any.
 *
 * The message is popped from the tx queue kept in ahi_env and freed using ke_msg_free.
 *
 * Note: This function is always call out of interrupt context.
 *****************************************************************************************
 */
__STATIC void ahi_tx_done(void)
{
    struct ahi_tx_data *tx_data = ahi_env.tx_ptr;

    // try to call the tx end callback
    if(tx_data->tx_callback != NULL)
    {
        // message free is managed by another handler
        ahi_env.tx_ptr->tx_callback(tx_data->data);
    }

    // Free the temporary tx pointer
    ke_free(ahi_env.tx_ptr);

    // check if there is something in TX queue
    if(! co_list_is_empty(&ahi_env.tx_queue))
    {
        //extract the ke_msg pointer from the top of the queue
        struct ahi_tx_data *tx_data = (struct ahi_tx_data *) co_list_pop_front(&ahi_env.tx_queue);

        // send the message on top of queue using H4TL
        ahi_h4tl_send(tx_data);
    }
    else
    {
        // Set AHI task to TX IDLE state
        ke_state_set(TASK_AHI, AHI_TX_IDLE);
    }
}

/**
 ****************************************************************************************
 * @brief Send Messages over H4TL
 *
 * @param[in] tx_data contains information of buffer to send
 ****************************************************************************************
 */
__STATIC void ahi_h4tl_send(struct ahi_tx_data *tx_data)
{
    ASSERT_ERR(tx_data != NULL);
    ahi_env.tx_ptr = tx_data;
    // Set AHI task busy
    ke_state_set(TASK_AHI, AHI_TX_ONGOING);

    // Send data over the AHI EIF Transport
    h4tl_write(tx_data->msg_type, tx_data->data, tx_data->len, &ahi_tx_done);
}


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void ahi_init(void)
{
    // Create AHI Task
    ke_task_create(TASK_AHI, &TASK_DESC_AHI);

    // Initialize AHI task to idle state
    ke_state_set(TASK_AHI, AHI_TX_IDLE);

    // Initialize TX queue
    co_list_init(&ahi_env.tx_queue);

    ahi_env.adv_report_nb = 0;
}

/**
 ****************************************************************************************
 * @brief Send a data message over the Application to Host Interface
 *
 * @param[in] msg_type Message type (logical channel)
 * @param[in] len      Message data length
 * @param[in] data     Data buffer to send
 * @param[in] msg_type Callback called when message is send. It provides message pointer
 *****************************************************************************************
 */
void ahi_send_msg(uint8_t msg_type, uint16_t len, uint8_t* data, void (*tx_callback)(uint8_t*))
{
    switch(msg_type)
    {
        case AHI_KE_MSG_TYPE:
        {
            struct ahi_tx_data* tx_data = (struct ahi_tx_data*) ke_malloc_system(sizeof(struct ahi_tx_data), KE_MEM_KE_MSG);
            tx_data->len         = len;
            tx_data->data        = data;
            tx_data->msg_type    = msg_type;
            tx_data->tx_callback = tx_callback;

            // mark memory initialized - for UART transfer
            DBG_MEM_INIT(data, len);

            // Check if there is no transmission ongoing
            if (ke_state_get(TASK_AHI) == AHI_TX_IDLE)
            {
                // request to send the message to H4 TL.
                ahi_h4tl_send(tx_data);
            }
            else
            {
                // put message at end of queue
                co_list_push_back(&ahi_env.tx_queue, &(tx_data->hdr));
            }
        } break;
        // ensure that only supported messages are sent over AHI interface
        default:
        {
            ASSERT_INFO(0, msg_type, len);
        } break;
    }
}

uint8_t* ahi_rx_hdr_handle(uint8_t* p_hdr_data, uint16_t* p_rem_length)
{
    ahi_kemsghdr_t * p_msg_hdr = (struct ahi_kemsghdr *) p_hdr_data;
    ke_msg_t* p_rx_msg = NULL;
    uint8_t* p_rx_data;
    ke_task_id_t dest_id;
    ke_task_id_t src_id;
    ke_msg_id_t  msg_id;
    bool unknown = false;

    dest_id = ahi_get_task_from_id(p_msg_hdr->dest_id);

    // destination task is unknown
    if(KE_TYPE_GET(dest_id) == TASK_NONE)
    {
        src_id  = p_msg_hdr->dest_id;
        dest_id = TASK_GAPM;
        msg_id  = GAPM_UNKNOWN_TASK_MSG;
        unknown = true;
    }
    // destination task is known
    else
    {
        msg_id = p_msg_hdr->id;
        src_id = ahi_get_task_from_id(p_msg_hdr->src_id);
    }

    // Allocate the kernel message
    p_rx_data = (uint8_t*) ke_msg_alloc(msg_id, dest_id, src_id, p_msg_hdr->param_len);

     if(p_rx_data != NULL)
     {
         p_rx_msg = ke_param2msg(p_rx_data);
         if(unknown)
         {
             // Store identifier in parameter length field
             p_rx_msg->param_len = p_msg_hdr->id;
         }
     }

    *p_rem_length = p_msg_hdr->param_len;

    return p_rx_data;
}


void ahi_rx_done(uint8_t* p_data)
{
    // Send message to destination handlerS
    ke_msg_send(p_data);
}

void ahi_reset(void)
{
    // send reset message to application
    struct gapm_reset_cmd* reset = KE_MSG_ALLOC(GAPM_RESET_CMD, TASK_GAPM, APP_MAIN_TASK, gapm_reset_cmd);
    reset->operation = GAPM_RESET;
    ke_msg_send(reset);
}


uint16_t ahi_get_id_from_task(uint16_t task)
{
    ke_task_id_t id = TASK_ID_INVALID;

    switch(task)
    {
        case TASK_GAPM:  { id = TASK_ID_GAPM;  } break;
        #if(GAPC_PRESENT)
        case TASK_GAPC:  { id = TASK_ID_GAPC;  } break;
        #endif //(GAPC_PRESENT)
        case TASK_GATT:  { id = TASK_ID_GATT;  } break;
        case TASK_L2CAP: { id = TASK_ID_L2CAP; } break;
        #if (AHI_TL_SUPPORT)
        case TASK_AHI:   { id = TASK_ID_AHI;   } break;
        #endif // (AHI_TL_SUPPORT)
        #if (BLE_GAF_PRESENT)
        case TASK_GAF:   { id = TASK_ID_GAF;   } break;
        #endif //(BLE_GAF_PRESENT)
        default:
        {
            #if (HOST_PROFILES)
            // check if profile manager is able to retrieve the task id
            id = prf_get_api_id_from_task_num(task);
            #endif // (HOST_PROFILES)
        }
        break;
    }

    return id;
}

uint16_t ahi_get_task_from_id(uint16_t api_id)
{
    ke_task_id_t task = TASK_NONE;

    switch(api_id)
    {
        case TASK_ID_GAPM:  { task = TASK_GAPM;  } break;
        #if(GAPC_PRESENT)
        case TASK_ID_GAPC:  { task = TASK_GAPC;  } break;
        #endif //(GAPC_PRESENT)
        case TASK_ID_GATT:  { task = TASK_GATT;  } break;
        case TASK_ID_L2CAP: { task = TASK_L2CAP; } break;
        #if (AHI_TL_SUPPORT)
        case TASK_ID_AHI:   { task = TASK_AHI;   } break;
        #endif // (AHI_TL_SUPPORT)
        #if (BLE_GAF_PRESENT)
        case TASK_ID_GAF:   { task = TASK_GAF;   } break;
        #endif //(BLE_GAF_PRESENT)

        default:
        {
            #if (HOST_PROFILES)
            // check if profile manager is able to retrieve the task number
            task = prf_get_task_num_from_api_id(api_id);
            #endif // (HOST_PROFILES)
        }
        break;
    }

    return (ke_task_check(task));
};


/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

__STATIC void ahi_ke_msg_tx_done(uint8_t* tx_data)
{
    // retrieve message pointer
    struct ke_msg* msg = (struct ke_msg*) (tx_data - sizeof(struct co_list_hdr));

    #if (BLE_OBSERVER)
    // Advertising report flow control
    if(msg->id == GAPM_EXT_ADV_REPORT_IND)
    {
        ahi_env.adv_report_nb--;
        // Enable transmission of new advertising reports
        gapm_adv_report_flow_ctrl(true);
    }
    #endif // (BLE_OBSERVER)

    // free it.
    ke_msg_free(msg);
}


/**
 ****************************************************************************************
 * @brief Function called to send a message through UART.
 *
 * @param[in]  msgid   U16 message id from ke_msg.
 * @param[in] *param   Pointer to parameters of the message in ke_msg.
 * @param[in]  dest_id Destination task id.
 * @param[in]  src_id  Source task ID.
 *
 * @return             Kernel message state, must be KE_MSG_NO_FREE.
 *****************************************************************************************
 */
__STATIC int ahi_msg_send_handler (ke_msg_id_t const msgid, void *param,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    //extract the ke_msg pointer from the param passed and push it in AHI queue
    struct ke_msg *msg = ke_param2msg(param);

    msg->src_id  = ahi_get_id_from_task(msg->src_id);
    msg->dest_id  = ahi_get_id_from_task(msg->dest_id);

    #if (BLE_OBSERVER)
    // Advertising report flow control
    if(msg->id == GAPM_EXT_ADV_REPORT_IND)
    {
        ahi_env.adv_report_nb++;
        if(ahi_env.adv_report_nb >= AHI_MAX_ADV_REPORT)
        {
            // Disable transmission of new advertising reports
            gapm_adv_report_flow_ctrl(false);
        }
    }
    #endif // (BLE_OBSERVER)

    // request to send the message.
    ahi_send_msg(AHI_KE_MSG_TYPE, msg->param_len+AHI_KE_MSG_HDR_LEN, (uint8_t *)&(msg->id),  &ahi_ke_msg_tx_done);

    //return NO_FREE always since ahi_eif_write handles the freeing
    return KE_MSG_NO_FREE;
}

/*
 * TASK DESCRIPTOR DEFINITIONS
 ****************************************************************************************
 */

/// Specifies the message handlers that are common to all states.
KE_MSG_HANDLER_TAB(ahi)
{
    // Note: all messages must be sorted in ID ascending order

    /** Default handler for AHI TX message, this entry has to be put first as table is
        parsed from end to start by Kernel */
    {KE_MSG_DEFAULT_HANDLER,  (ke_msg_func_t)ahi_msg_send_handler},
};

/// Defines the placeholder for the states of all the task instances.
ke_state_t ahi_state[AHI_IDX_MAX];

/// AHI task descriptor
const struct ke_task_desc TASK_DESC_AHI = {ahi_msg_handler_tab, ahi_state, AHI_IDX_MAX,  ARRAY_LEN(ahi_msg_handler_tab)};


#endif //AHI_TL_SUPPORT

/// @} AHI
