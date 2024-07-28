/**
 ****************************************************************************************
 * @file l2cap_chan.c
 *
 * @brief  L2CAP channel management
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup L2CAP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"    // IP configuration
#if (BLE_L2CAP)
#include "l2cap.h"          // Native API
#include "l2cap_int.h"      // Internals

#include "ke_mem.h"         // Memory allocation
#include "gapc.h"           // To convert connection handle to connection index
#include "co_math.h"        // co_min, CO_BIT use

#if (HOST_MSG_API)
#include "ke_task.h"        // For task communication
#endif // (HOST_MSG_API

#include "dbg.h"            // Debug API
#include <string.h>         // for memory copy

/*
 * MACROS
 ****************************************************************************************
 */



/*
 * DEFINES
 ****************************************************************************************
 */



/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */


/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */



/*
 * INTERNAL FUNCTIONS
 ****************************************************************************************
 */

l2cap_chan_t* l2cap_chan_get_env(uint8_t conidx, uint8_t chan_lid)
{
    l2cap_con_env_t* p_con = l2cap_get_con_env(conidx);
    return (((p_con != NULL) && (chan_lid < p_con->nb_channel)) ? p_con->p_chan[chan_lid] : NULL);
}


l2cap_chan_coc_t* l2cap_chan_coc_get(uint8_t conidx, uint8_t chan_lid)
{
    l2cap_chan_t* p_chan = l2cap_chan_get_env(conidx, chan_lid);
    return (((p_chan!= NULL) && GETB(p_chan->config_bf, L2CAP_CHAN_CREDIT_FLOW_EN)) ?   (l2cap_chan_coc_t*) p_chan : NULL);
}



uint8_t l2cap_chan_find(uint8_t conidx, uint8_t cid_type, uint16_t cid, l2cap_chan_t** pp_chan)
{
    uint8_t chan_lid = L2CAP_INVALID_CHAN_LID;
    l2cap_con_env_t* p_con = l2cap_get_con_env(conidx);
    if(p_con != NULL)
    {
        uint8_t i;

        for(i = 0 ; i < p_con->nb_channel ; i++)
        {
            l2cap_chan_t* p_chan = p_con->p_chan[i];

            // check if CID found
            if(    (p_chan != NULL)
                    // Local CID searched
                && (   ((cid_type == L2CAP_CHAN_CID_LOCAL) && (p_chan->rx_cid == cid))
                    // Peer CID searched
                    || ((cid_type == L2CAP_CHAN_CID_PEER)  && (p_chan->tx_cid == cid))))
            {
                chan_lid = i;
                if(pp_chan != NULL)
                {
                    *pp_chan = p_chan;
                }
                break;
            }
        }
    }

    return (chan_lid);
}


void l2cap_chan_rx_credit_add(uint8_t conidx, uint8_t chan_lid, uint16_t credit)
{
    // increment number of reception credits
    l2cap_chan_coc_t* p_chan = l2cap_chan_coc_get(conidx, chan_lid);

    // sanity check
    if(   (p_chan != NULL) && GETB(p_chan->config_bf, L2CAP_CHAN_EN)
       && GETB(p_chan->config_bf, L2CAP_CHAN_CREDIT_FLOW_EN))
    {
        uint16_t new_credit = co_min(credit, p_chan->rx_credit_max - p_chan->rx_credit);

        // increment number of reception credits
        l2cap_coc_rx_credit_add(p_chan, new_credit);
        p_chan->rx_credit += new_credit;
    }
}

void l2cap_chan_ll_buf_info_set(uint16_t buf_size, uint16_t nb_buf)
{
    l2cap_env.ll_buf_size     = buf_size;
    l2cap_env.ll_buf_nb_total = nb_buf;
    l2cap_env.ll_buf_nb_avail = l2cap_env.ll_buf_nb_total;
}

uint16_t l2cap_chan_ll_buf_nb_get(void)
{
    return l2cap_env.ll_buf_nb_total;
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t l2cap_chan_fix_register(uint8_t conidx, uint16_t cid, uint16_t mtu, const l2cap_chan_cb_t* p_cb, uint8_t* p_chan_lid)
{
    l2cap_con_env_t* p_con = l2cap_get_con_env(conidx);
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    *p_chan_lid = L2CAP_INVALID_CHAN_LID;

    // check provided parameters
    if((p_cb == NULL) || (p_cb->cb_sdu_rx == NULL) || (p_cb->cb_sdu_sent == NULL))
    {
        status = GAP_ERR_INVALID_PARAM;
    }
    else if (mtu > GAP_LE_MTU_MAX)
    {
        status = L2CAP_ERR_INVALID_MTU;
    }
    // ensure that connection is available
    else if(p_con != NULL)
    {
        uint8_t chan_lid = L2CAP_INVALID_CHAN_LID;
        l2cap_chan_t* p_chan;

        status = GAP_ERR_INSUFF_RESOURCES;

        uint8_t i;
        for(i = 0 ; i < p_con->nb_channel ; i++)
        {
            p_chan = p_con->p_chan[i];

            // check if RX CID found
            if((p_chan != NULL) && ((p_chan->rx_cid == cid) || (p_chan->tx_cid == cid)))
            {
                chan_lid = L2CAP_INVALID_CHAN_LID;
                status   = GAP_ERR_COMMAND_DISALLOWED;
                break;
            }
            // mark position of available channel id found
            else if((p_chan == NULL) && (chan_lid == L2CAP_INVALID_CHAN_LID))
            {
                chan_lid = i;
            }
        }

        // Available channel identifier found
        if(chan_lid != L2CAP_INVALID_CHAN_LID)
        {
            p_chan = (l2cap_chan_t*) ke_malloc_user(sizeof(l2cap_chan_t), KE_MEM_ENV);

            if(p_chan != NULL)
            {
                p_con->p_chan[chan_lid] = p_chan;

                co_list_init(&(p_chan->tx_queue));
                p_chan->p_rx_sdu      = NULL;
                p_chan->config_bf     = L2CAP_CHAN_EN_BIT | L2CAP_CHAN_FIX_BIT;
                p_chan->conidx        = conidx;
                p_chan->chan_lid      = chan_lid;
                p_chan->p_cb          = p_cb;
                p_chan->rx_cid        = cid;
                p_chan->tx_cid        = cid;
                p_chan->mtu           = mtu;
                #if (HOST_MSG_API)
                p_chan->dest_task_nbr = KE_TASK_INVALID;
                #endif // (HOST_MSG_API)

                // return parameters
                *p_chan_lid       = chan_lid;
                status = GAP_ERR_NO_ERROR;
            }
        }
    }

    return (status);
}

uint16_t l2cap_chan_dyn_reserve(uint8_t conidx, uint8_t* p_chan_lid, l2cap_chan_coc_t** pp_chan)
{
    l2cap_con_env_t* p_con = l2cap_get_con_env(conidx);
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    *p_chan_lid = L2CAP_INVALID_CHAN_LID;

    // ensure that connection is available
    if(p_con != NULL)
    {
        uint8_t chan_lid = L2CAP_INVALID_CHAN_LID;

        uint8_t i;
        for(i = 0 ; i < p_con->nb_channel ; i++)
        {
            if((p_con->p_chan[i] == NULL) && (chan_lid == L2CAP_INVALID_CHAN_LID))
            {
                chan_lid = i;
                break;
            }
        }

        status = GAP_ERR_INSUFF_RESOURCES;

        // Available channel identifier found
        if(chan_lid != L2CAP_INVALID_CHAN_LID)
        {
            l2cap_chan_coc_t* p_chan = (l2cap_chan_coc_t*) ke_malloc_user(sizeof(l2cap_chan_coc_t), KE_MEM_ENV);

            if(p_chan != NULL)
            {
                p_con->p_chan[chan_lid] = (l2cap_chan_t*)p_chan;
                memset(p_chan, 0, sizeof(l2cap_chan_coc_t));
                p_chan->conidx        = conidx;
                p_chan->chan_lid      = chan_lid;
                p_chan->config_bf     = L2CAP_CHAN_CREDIT_FLOW_EN_BIT; // channel disabled by default
                #if (HOST_MSG_API)
                p_chan->dest_task_nbr = KE_TASK_INVALID;
                #endif // (HOST_MSG_API)

                // return parameters
                *pp_chan          = p_chan;
                *p_chan_lid       = chan_lid;

                status = GAP_ERR_NO_ERROR;
            }
        }
    }

    return (status);
}

uint16_t l2cap_chan_unregister(uint8_t conidx, uint8_t chan_lid)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    l2cap_chan_t* p_chan = l2cap_chan_get_env(conidx, chan_lid);

    if(p_chan != NULL)
    {
        l2cap_con_env_t* p_con = l2cap_env.p_con[conidx];

        // clean-up channel
        l2cap_chan_enable_set(p_chan, false);

        // release channel memory
        ke_free(p_chan);
        p_con->p_chan[chan_lid] = NULL;

        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

uint16_t l2cap_chan_fix_mtu_update(uint8_t conidx, uint8_t chan_lid, uint16_t mtu)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    l2cap_chan_t* p_chan = l2cap_chan_get_env(conidx, chan_lid);

    if((p_chan != NULL) && (GETB(p_chan->config_bf, L2CAP_CHAN_FIX)))
    {
        p_chan->mtu = mtu;
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}

uint16_t l2cap_chan_tx_credit_add(uint8_t conidx, uint8_t chan_lid, uint16_t credit)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    l2cap_chan_coc_t* p_chan = l2cap_chan_coc_get(conidx, chan_lid);

    if(p_chan != NULL)
    {
        // check if too much credit received
        if((p_chan->tx_credit + credit) < p_chan->tx_credit)
        {
            status = L2CAP_ERR_CREDIT_ERROR;
        }
        else
        {
            p_chan->tx_credit += credit;

            // check that buffer are available
            if(   !GETB(p_chan->config_bf, L2CAP_CHAN_IN_TX_QUEUE)
               && !co_list_is_empty(&(p_chan->tx_queue))
               // check if TX flow is on
               && !GETB(p_chan->config_bf, L2CAP_CHAN_PDU_TX_PAUSED)
               // and there is TX credit available
               && (p_chan->tx_credit != 0))
            {
                co_list_push_back(&(l2cap_env.le_tx_queue), &(p_chan->hdr)); // TODO create a function
                SETB(p_chan->config_bf, L2CAP_CHAN_IN_TX_QUEUE, true);

                // mark that transmission can be started
                co_djob_reg(CO_DJOB_LOW, &(l2cap_env.le_tx_bg_job));
            }

            status = GAP_ERR_NO_ERROR;
        }
    }

    return (status);
}

uint16_t l2cap_chan_enable_set(l2cap_chan_t* p_chan, uint8_t enable)
{
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;
    uint8_t conidx = p_chan->conidx;
    uint8_t chan_lid =  p_chan->chan_lid;
    l2cap_con_env_t* p_con = l2cap_env.p_con[conidx];

    // Channel transition to off
    if(!enable && GETB(p_chan->config_bf, L2CAP_CHAN_EN))
    {
        // release transmission queue
        l2cap_chan_tx_release_queue(p_con, p_chan);


        // release reception buffer
        if(p_chan->p_rx_sdu != NULL)
        {
            co_buf_release(p_chan->p_rx_sdu);
            p_chan->p_rx_sdu = NULL;
        }

        // stop reception for current channel
        if(p_con->rx_chan_lid == chan_lid)
        {
            l2cap_chan_rx_init(p_con);
        }
    }

    // Update channel state
    SETB(p_chan->config_bf, L2CAP_CHAN_EN, enable);

    status = GAP_ERR_NO_ERROR;

    return (status);
}


uint16_t l2cap_chan_max_sdu_tx_size_get(uint8_t conidx, uint8_t chan_lid)
{
    uint16_t max_sdu_tx_size = 0;
    l2cap_chan_t* p_chan = l2cap_chan_get_env(conidx, chan_lid);

    if(p_chan != NULL)
    {
        if(!GETB(p_chan->config_bf, L2CAP_CHAN_FIX))
        {
            l2cap_chan_coc_t* p_chan_coc = (l2cap_chan_coc_t* ) p_chan;
            uint32_t max_size = p_chan_coc->tx_credit * p_chan_coc->tx_mps;
            if(max_size > L2CAP_SDU_LEN)
            {
                max_size -= L2CAP_SDU_LEN;
            }
            // update max transmission size according to number of available credits
            max_sdu_tx_size = co_min(p_chan_coc->tx_mtu, max_size);
        }
        else
        {
            max_sdu_tx_size = p_chan->mtu;
        }
    }

    return max_sdu_tx_size;
}

void l2cap_chan_cleanup(l2cap_con_env_t* p_con)
{
    uint8_t chan_lid;

    // clean-up L2CAP channels
    for(chan_lid = 0 ; chan_lid < p_con->nb_channel; chan_lid++)
    {
        l2cap_chan_t* p_chan = l2cap_chan_get_env(p_con->conidx, chan_lid);

        if(p_chan != NULL)
        {
            const l2cap_chan_coc_cb_t* p_cb = (const l2cap_chan_coc_cb_t*) p_chan->p_cb;
            bool  fix_chan = GETB(p_chan->config_bf, L2CAP_CHAN_FIX);
            // clean-up channel
            l2cap_chan_enable_set(p_chan, false);

            // release channel memory
            ke_free(p_chan);

            if(!fix_chan && (p_cb != NULL))
            {
                // inform upper layer about COC termination
                p_cb->cb_coc_terminated(p_con->conidx, 0, chan_lid, GAP_ERR_DISCONNECTED);
            }
        }
    }

    // Clean-up RX queue
    l2cap_rx_queue_cleanup(p_con);
}

#endif // (BLE_L2CAP)
/// @} L2CAP

