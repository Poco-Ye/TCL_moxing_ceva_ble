/**
 ****************************************************************************************
 *
 * @file llm.c
 *
 * @brief Definition of the functions used by the link layer manager
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"  // stack configuration

#include <string.h>
#include "co_bt.h"        // BLE standard definitions
#include "co_math.h"
#include "co_utils.h"
#include "ble_util.h"     // BLE utility functions

#include "ke_mem.h"       // kernel memory
#include "ke_timer.h"
#include "ke_task.h"      // kernel task definition

#include "llm.h"          // link layer manager definitions
#include "llc.h"          // link layer driver definitions
#include "lld.h"          // link layer driver definitions

#include "llm_int.h"      // link layer manager internal definitions

#include "rwip.h"

#if HCI_PRESENT
#include "hci.h"          // host controller interface
#endif //HCI_PRESENT

/*
 * DEFINES
 ****************************************************************************************
 */



/*
 * CONSTANTS DEFINITION
 *****************************************************************************************
 */
/// Local LE supported features
__STATIC const struct le_features llm_local_le_feats =
    {{ BLE_FEATURES_BYTE0, BLE_FEATURES_BYTE1, BLE_FEATURES_BYTE2, BLE_FEATURES_BYTE3,
       BLE_FEATURES_BYTE4, BLE_FEATURES_BYTE5, BLE_FEATURES_BYTE6, BLE_FEATURES_BYTE7 }};

/// LE default event mask
__STATIC const struct evt_mask le_default_evt_mask = {BLE_EVT_MASK};

/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LLM environment variable
struct llm_env_tag llm_env;


/*
 * MODULE INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

uint8_t ROM_VT_FUNC(llm_per_adv_chain_dur)(uint16_t len, uint8_t phy)
{
   ASSERT_ERR(len <= BLE_CFG_MAX_ADV_DATA_LEN);

   uint8_t pkt_nb = len/240 + 1;
   uint8_t frag_size = (pkt_nb > 1) ? BLE_ADV_FRAG_SIZE_TX + 1 : co_min(len + 15, BLE_ADV_FRAG_SIZE_TX + 1);
   uint32_t dur_hus = pkt_nb*2*ble_util_pkt_dur_in_us(frag_size, phy - 1) + (pkt_nb - 1)*2*BLE_AFS_DUR;

   return (dur_hus/HALF_SLOT_SIZE + 1);
}

void ROM_VT_FUNC(llm_cmd_cmp_send)(uint16_t opcode, uint8_t status)
{
    // allocate the complete event message
    struct hci_basic_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_cmd_cmp_evt);
    // update the status
    evt->status = status;
    // send the message
    hci_send_2_host(evt);
}

void ROM_VT_FUNC(llm_cmd_stat_send)(uint16_t opcode, uint8_t status)
{
    // allocate the status event message
    struct hci_cmd_stat_event *evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, 0, opcode, hci_cmd_stat_event);
    // update the status
    evt->status = status;
    // send the message
    hci_send_2_host(evt);
}

#if (BLE_CENTRAL || BLE_PERIPHERAL || BLE_OBSERVER)
bool ROM_VT_FUNC(llm_is_dev_connected)(struct bd_addr const *peer_addr, uint8_t peer_addr_type)
{
    uint8_t act_id = 0;

    // Check all entries of the connection information table
    for(act_id = 0 ; act_id < BLE_ACTIVITY_MAX ; act_id++)
    {
        // Check if there is a connection
        if(llm_env.act_info[act_id].state == LLM_CONNECTED)
        {
            // Compare BD address and BD address type
            // Mask the RPA bit from stored peer address type to Public or Random
            if(co_bdaddr_compare(peer_addr, &llm_env.act_info[act_id].info.con.bd_addr) && ((peer_addr_type & ADDR_MASK) == (llm_env.act_info[act_id].info.con.addr_type & ADDR_MASK)))
                break;
        }
    }

    return (act_id < BLE_ACTIVITY_MAX);
}
#endif //(BLE_CENTRAL || BLE_PERIPHERAL || BLE_OBSERVER)


#if (BLE_CENTRAL || BLE_OBSERVER)
bool ROM_VT_FUNC(llm_is_dev_synced)(struct bd_addr const *peer_addr, uint8_t peer_addr_type, uint8_t adv_sid)
{
    uint8_t act_id = 0;

    // Check all entries of the connection information table
    for(act_id = 0 ; act_id < BLE_ACTIVITY_MAX ; act_id++)
    {
        // Check if there is a connection
        if(    llm_env.act_info[act_id].state == LLM_PER_SCAN_SYNCED
            || llm_env.act_info[act_id].state == LLM_PER_SCAN_SYNCING
            || llm_env.act_info[act_id].state == LLM_PER_SCAN_SYNCING_FROM_SYNC_TRANSF)
        {
            // Compare BD address, BD address type and ADV SID
            // Mask the RPA bit from stored peer address type to Public or Random
            if(   co_bdaddr_compare(peer_addr, &llm_env.act_info[act_id].info.sync.bd_addr)
               && ((peer_addr_type & ADDR_MASK) == (llm_env.act_info[act_id].info.sync.addr_type & ADDR_MASK))
               && (adv_sid == llm_env.act_info[act_id].info.sync.adv_sid) )
                break;
        }
    }

    return (act_id < BLE_ACTIVITY_MAX);
}
#endif //(BLE_CENTRAL || BLE_OBSERVER)

uint8_t ROM_VT_FUNC(llm_dev_list_empty_entry)(void)
{
    uint8_t position;

    // Parse the list linearly
    for(position = 0; position < BLE_WHITELIST_MAX ; position++)
    {
        // Check if list entry is used
        if(!GETB(llm_env.dev_list[position].status, LLM_DEV_LIST_ENTRY_USED))
            break;
    }

    return position;
}

uint8_t ROM_VT_FUNC(llm_dev_list_search)(const struct bd_addr *bd_addr, uint8_t bd_addr_type)
{
    uint8_t position;

    // Parse the list linearly
    for(position = 0; position < BLE_WHITELIST_MAX ; position++)
    {
        // Check if list entry is used
        if(!GETB(llm_env.dev_list[position].status, LLM_DEV_LIST_ENTRY_USED))
            continue;

        // Compare address type
        if(llm_env.dev_list[position].addr_type != bd_addr_type)
            continue;

        // Compare BD address
        if(!co_bdaddr_compare(&llm_env.dev_list[position].addr, bd_addr))
            continue;

        break;
    }

    return position;
}

uint8_t ROM_VT_FUNC(llm_ral_search)(const struct bd_addr *bd_addr, uint8_t bd_addr_type)
{
    uint8_t position;

    for(position = 0 ; position < BLE_RAL_MAX ; position++)
    {
        if(llm_env.ral[position].addr_type != 0xFF)
        {
            // Compare the 2 BD addresses
            if (((bd_addr_type & ADDR_RAND) == llm_env.ral[position].addr_type) && (co_bdaddr_compare(&llm_env.ral[position].bd_addr, bd_addr)))
                break;
        }
    }

    return position;
}

bool ROM_VT_FUNC(llm_ral_is_empty)(void)
{
    uint8_t position;

    // Parse the resolving list linearly
    for(position = 0; position < BLE_RAL_MAX ; position++)
    {
        // If entry is valid
        if(llm_env.ral[position].addr_type != 0xFF)
            break;
    }

    return (position == BLE_RAL_MAX);
}

#if (BLE_CENTRAL || BLE_BROADCASTER)
void ROM_VT_FUNC(llm_ch_map_update)(void)
{
    #if ((BLE_BIS) && (BLE_BROADCASTER))
    bool big_chmap_update = false;
    #endif // ((BLE_BIS) && (BLE_BROADCASTER))
    bool ch_map_timer = false;
    bool ch_map_updated = false;

    uint8_t nbchgood = 0;

    // Get the current channel map for the local piconet
    struct le_chnl_map* ch_map = &llm_env.ch_map_info.master_ch_map;

    // Get Host channel classification
    struct le_chnl_map* host_ch_class = &llm_env.ch_map_info.host_ch_class;

    // Save the old channel map before processing
    struct le_chnl_map old_ch_map;
    memcpy(&old_ch_map, &llm_env.ch_map_info.master_ch_map, LE_CHNL_MAP_LEN);

    // Compute the channel map
    for(int i = 0 ; i < DATA_CHANNEL_NB ; i++)
    {
        /*
         * Decision made from local assessment:
         *  - channel bad     => disable
         *  - channel unknown => enable
         *  - channel good    => enable
         * Each bit of the host channel classification corresponds to:
         *      0: bad channel
         *      1: unknown channel (treated as good)
         */

        uint8_t byte_idx = i >> 3;
        uint8_t bit_pos = i & 0x7;

        // Host indicates the channel as bad
        if (((host_ch_class->map[byte_idx] >> bit_pos) & 0x1) == 0x00)
        {
            ch_map->map[byte_idx] &= ~(1 << bit_pos);
            continue;
        }

        if (rwip_update_ch_map_with_ch_assess_ble(i, &ch_map->map[byte_idx]))
            continue;
    }

    // Count number of good channels
    nbchgood = ble_util_nb_good_channels(ch_map);

    // Check if the map has a sufficient number of used channels
    if(nbchgood < DATA_CHANNEL_USED_NB_MIN)
    {
        // Re-introduce 1 channel: take the first enabled channel from Host classification (shall be non-zero, as per HCI_LE_Set_Host_Ch_Class command)
        for(int i = 0 ; i < DATA_CHANNEL_NB ; i++)
        {
            uint8_t byte_idx = i >> 3;
            uint8_t bit_pos = i & 0x7;

            // Host indicates the channel as good
            if ((((host_ch_class->map[byte_idx] >> bit_pos) & 0x1) == 0x01) && ((ch_map->map[byte_idx] & (1 << bit_pos)) == 0x00))
            {
                ch_map->map[byte_idx] |= (1 << bit_pos);

                // Check if the map has a sufficient number of used channels
                if(++nbchgood >= DATA_CHANNEL_USED_NB_MIN)
                    break;
            }
        }
    }
    ASSERT_ERR(nbchgood >= DATA_CHANNEL_USED_NB_MIN);

    // Check if the channel map has changed
    ch_map_updated = (memcmp(&old_ch_map, ch_map, LE_CHNL_MAP_LEN) != 0);

    #if (BLE_BROADCASTER)
    if (ch_map_updated)
    {
        // Set the connectionless LLD channel map
        lld_ch_map_set(ch_map);
    }
    #endif // (BLE_BROADCASTER)

    // Search for master active link(s), BIG or periodic advertising
    for(int i = 0; i < BLE_ACTIVITY_MAX; i++)
    {
        #if (BLE_CENTRAL)
        // Check if activity is a master link
        if ((llm_env.act_info[i].state == LLM_CONNECTED) && (llm_env.act_info[i].info.con.role == ROLE_MASTER))
        {
            if (ch_map_updated)
            {
                // Send an indication to LLC to trigger channel map update on the link
                ke_msg_send_basic(LLM_CH_MAP_UPDATE_IND, KE_BUILD_ID(TASK_LLC, i), TASK_LLM);
            }

            ch_map_timer = true;
        }
        #endif // (BLE_CENTRAL)

        #if ((BLE_BIS) && (BLE_BROADCASTER))
        if (llm_env.act_info[i].state == LLM_BIS_RSVD)
        {
            ch_map_timer = true;
        }
        #endif // ((BLE_BIS) && (BLE_BROADCASTER))

        if (ch_map_updated)
        {
            #if (BLE_BROADCASTER)
            // Check if activity is a periodic advertising
            if (llm_env.act_info[i].state == LLM_PER_ADV_EN)
            {
                // Request periodic advertising to update the channel map
                lld_per_adv_ch_map_update(i, ch_map);
            }
            #if (BLE_BIS)
            else if (llm_env.act_info[i].state == LLM_BIS_RSVD)
            {
                big_chmap_update = true;
            }
            #endif // (BLE_BIS)
            #endif // (BLE_BROADCASTER)
        }
    }

    #if ((BLE_BIS) && (BLE_BROADCASTER))
    if(big_chmap_update)
    {
        ke_msg_send_basic(LLM_CH_MAP_UPDATE_IND, TASK_LLI, TASK_LLM);
    }
    #endif // ((BLE_BIS) && (BLE_BROADCASTER))

    // If channel map management is needed
    if(ch_map_timer)
    {
        // Restart channel map update timer
        ke_timer_set(LLM_CH_MAP_TO, TASK_LLM, BLE_CH_MAP_UPDATE_PERIOD*1000);
    }
    else
    {
        // Indicate channel map management as inactive
        llm_env.ch_map_info.active = false;
    }
}
#endif //(BLE_CENTRAL || BLE_BROADCASTER)


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void ROM_VT_FUNC(llm_init)(uint8_t init_type)
{
    // boot configuration
    switch (init_type)
    {
        case RWIP_INIT:
        {
            // Create LLM task
            ke_task_create(TASK_LLM, &TASK_DESC_LLM);
        }
        break;

        case RWIP_RST:
        {
            // Check if host parameters are stored
            for (uint8_t act_id = 0; act_id < BLE_ACTIVITY_MAX; act_id++)
            {
                if (llm_env.act_info[act_id].host_params != NULL)
                {
                    ke_msg_free(ke_param2msg(llm_env.act_info[act_id].host_params));
                }

                #if BLE_CONLESS_CTE_TX
                if ((llm_env.act_info[act_id].state == LLM_ADV_RSVD) || (llm_env.act_info[act_id].state == LLM_ADV_EN) || (llm_env.act_info[act_id].state == LLM_ADV_STOPPING))
                {
                    if(llm_env.act_info[act_id].info.adv.cte_tx_params != NULL)
                    {
                        ke_msg_free(ke_param2msg(llm_env.act_info[act_id].info.adv.cte_tx_params));
                    }
                }
                #endif // BLE_CONLESS_CTE_TX

                #if BLE_CONLESS_CTE_RX
                if ((llm_env.act_info[act_id].state == LLM_PER_SCAN_SYNCED) || (llm_env.act_info[act_id].state == LLM_PER_SCAN_STOPPING))
                {
                    if(llm_env.act_info[act_id].info.sync.cte_rx_params != NULL)
                    {
                        ke_msg_free(ke_param2msg(llm_env.act_info[act_id].info.sync.cte_rx_params));
                    }
                }
                #endif // BLE_CONLESS_CTE_RX


            }
        }
        // No break

        case RWIP_1ST_RST:
        {
            uint8_t length;
            uint8_t supp_phy_msk = PHY_1MBPS_BIT;

            SETB(supp_phy_msk, PHY_2MBPS, BLE_PHY_2MBPS_SUPPORT);
            SETB(supp_phy_msk, PHY_CODED, BLE_PHY_CODED_SUPPORT);

            memset(&llm_env, 0, sizeof(llm_env));

            // Initialize non-zero HCI configuration parameters
            llm_env.rpa_renew_to            = RPA_TO_DFT;

            #if (BLE_CENTRAL || BLE_PERIPHERAL)
            llm_env.suggested_max_tx_octets = BLE_MIN_OCTETS;
            llm_env.suggested_max_tx_time   = BLE_MIN_TIME;
            llm_env.tx_phys                 = (supp_phy_msk);
            llm_env.rx_phys                 = (supp_phy_msk);
            #endif // (BLE_CENTRAL || BLE_PERIPHERAL)


            // Initialize LE event mask
            memcpy(&llm_env.le_event_mask.mask[0], &le_default_evt_mask.mask[0], EVT_MASK_LEN);

            // Initialize Host channel classification
            memset(&llm_env.ch_map_info.host_ch_class.map[0], 0xFF, LE_CHNL_MAP_LEN);
            llm_env.ch_map_info.host_ch_class.map[LE_CHNL_MAP_LEN-1] &= 0x1F;
            memset(&llm_env.ch_map_info.master_ch_map.map[0], 0xFF, LE_CHNL_MAP_LEN);
            llm_env.ch_map_info.master_ch_map.map[LE_CHNL_MAP_LEN-1] &= 0x1F;

            // BD address
            length = BD_ADDR_LEN;
            if (rwip_param.get(PARAM_ID_BD_ADDRESS, &length, (uint8_t *)&llm_env.local_pub_addr) != PARAM_OK)
            {
                memcpy(&llm_env.local_pub_addr, &co_default_bdaddr, sizeof(co_default_bdaddr));
            }

            length = PARAM_LEN_PRIVATE_KEY_P256;
            rwip_param.get(PARAM_ID_LE_PRIVATE_KEY_P256, &length, &llm_env.secret_key256[0]);

            #if (BLE_CENTRAL || BLE_PERIPHERAL)
            uint8_t act_move_cfg;
            length = PARAM_LEN_ACTIVITY_MOVE_CONFIG;
            if (rwip_param.get(PARAM_ID_ACTIVITY_MOVE_CONFIG, &length, &act_move_cfg) != PARAM_OK)
            {
                // If no value is set in persistent storage enable the feature by default
                llm_env.con_move_en = true;
            }
            else
            {
                // Bit 0 denotes support for BLE connections
                llm_env.con_move_en = (act_move_cfg & 0x01);
            }
            #endif // (BLE_CENTRAL || BLE_PERIPHERAL)

            #if (BLE_OBSERVER)
            length = PARAM_LEN_SCAN_EXT_ADV;
            if (rwip_param.get(PARAM_ID_SCAN_EXT_ADV, &length, (uint8_t*) &llm_env.ext_scan) != PARAM_OK)
            {
                // Extended scanning is enabled by default
                llm_env.ext_scan = true;
            }
            #endif //(BLE_OBSERVER)

            // Initialize RAL
            for(uint8_t position = 0; position < BLE_RAL_MAX; position++)
            {
                // Set invalid address type
                llm_env.ral[position].addr_type = 0xFF;
            }

            #if (BLE_ISO_PRESENT && BLE_HOST_PRESENT)
            llm_env.hci.iso_chan_host_support = 1;
            #endif // (BLE_ISO_PRESENT && BLE_HOST_PRESENT)

            // Start the timer that triggers renewal of resolvable private address
            ke_timer_set(LLM_RPA_RENEW_TO, TASK_LLM, 1000*llm_env.rpa_renew_to);
        }
        break;

        default:
        {
            // Do nothing
        }
        break;
    }
}

#if (BLE_CENTRAL || BLE_PERIPHERAL)
void ROM_VT_FUNC(llm_link_disc)(uint8_t link_id)
{
    struct sch_plan_elt_tag *plan_elt = &llm_env.act_info[link_id].plan_elt;
    uint8_t position;

    // Indicate that link does not require active clock anymore
    llm_clk_acc_set(link_id, false);

    // Free link identifier and LT address
    llm_env.act_info[link_id].state = LLM_FREE;

    // Unregister the connection from bandwidth allocation system
    sch_plan_rem(plan_elt);

    // Check if the device is in the device list
    position = llm_dev_list_search(&llm_env.act_info[link_id].info.con.bd_addr, llm_env.act_info[link_id].info.con.addr_type);

    // Restore presence of device in EM white list, if it is in white list
    if((position < BLE_WHITELIST_MAX) && GETB(llm_env.dev_list[position].status, LLM_DEV_IN_WL))
    {
        lld_white_list_add(position, &llm_env.act_info[link_id].info.con.bd_addr, llm_env.act_info[link_id].info.con.addr_type);
    }

    // Unregister linkID at HCI level
    hci_ble_conhdl_unregister(link_id);
}

void ROM_VT_FUNC(llm_clk_acc_set)(uint8_t act_id, bool clk_acc)
{
    if(clk_acc)
    {
        llm_env.act_clk_acc |= (1 << act_id);

        // Switch to active clock immediately
        rwip_prevent_sleep_set(RW_BLE_ACTIVE_MODE);
    }
    else
    {
        llm_env.act_clk_acc &= ~(1 << act_id);

        // Check if no BLE connection needs the active clock anymore
        if(llm_env.act_clk_acc == 0)
        {
            rwip_prevent_sleep_clear(RW_BLE_ACTIVE_MODE);
        }
    }
}
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

struct le_chnl_map* ROM_VT_FUNC(llm_master_ch_map_get)(void)
{
    return &llm_env.ch_map_info.master_ch_map;
}


bool ROM_VT_FUNC(llm_le_evt_mask_check)(uint8_t event_id)
{
    return ((llm_env.le_event_mask.mask[event_id>>3] & (1<<(event_id & 0x07))) != 0);
}

void ROM_VT_FUNC(llm_le_features_get)(struct le_features *feats)
{
    uint8_t byte_nb, bit_nb;

    memcpy(&feats->feats[0], &llm_local_le_feats.feats[0], LE_FEATS_LEN);
    // Update LE feature according to Host supported features
    byte_nb = BLE_FEAT_ISO_CHANNELS_HOST_SUPPORT/8;
    bit_nb = CO_MOD(BLE_FEAT_ISO_CHANNELS_HOST_SUPPORT, 8);
    feats->feats[byte_nb] |= (llm_env.hci.iso_chan_host_support << bit_nb);
}


int16_t ROM_VT_FUNC(llm_tx_path_comp_get)(void)
{
    return (llm_env.tx_path_comp);
}

int16_t ROM_VT_FUNC(llm_rx_path_comp_get)(void)
{
    return (llm_env.rx_path_comp);
}

struct ral_entry* ROM_VT_FUNC(llm_ral_get)(void)
{
    return &llm_env.ral[0];
}

struct sch_plan_elt_tag *ROM_VT_FUNC(llm_plan_elt_get)(uint8_t act_id)
{
    return (&llm_env.act_info[act_id].plan_elt);
}

uint8_t ROM_VT_FUNC(llm_activity_free_get)(uint8_t* act_id)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    // If not present, try to allocate an activity identifier
    for (*act_id = 0; *act_id < BLE_ACTIVITY_MAX; (*act_id)++)
    {
        if (llm_env.act_info[*act_id].state == LLM_FREE)
        {
            // Initialize activity info
            memset(&llm_env.act_info[*act_id].info, 0, sizeof(llm_env.act_info[*act_id].info));

            break;
        }
    }

    // If not possible to start a new advertising activity, reject the command
    if(*act_id >= BLE_ACTIVITY_MAX)
    {
        status = CO_ERROR_MEMORY_CAPA_EXCEED;
    }

    return (status);
}

void ROM_VT_FUNC(llm_activity_free_set)(uint8_t act_id)
{
    llm_env.act_info[act_id].state = LLM_FREE;
    // clean-up possible interval in use
    sch_plan_rem(&(llm_env.act_info[act_id].plan_elt));
}

#if (BLE_CIS)
void llm_activity_cis_reserve(uint8_t act_id)
{
    llm_env.act_info[act_id].state = LLM_CIS_RSVD;
}
#endif // (BLE_CIS)

#if (BLE_BIS)
void llm_activity_bis_reserve(uint8_t act_id)
{
    llm_env.act_info[act_id].state = LLM_BIS_RSVD;

    #if (BLE_BROADCASTER)
    // Check if master channel map monitoring is already active
    if(!llm_env.ch_map_info.active)
    {
        llm_env.ch_map_info.active = true;

        // Start channel map update timer
        ke_timer_set(LLM_CH_MAP_TO, TASK_LLM, BLE_CH_MAP_UPDATE_PERIOD*1000);
    }
    #endif // (BLE_BROADCASTER)
}
#endif // (BLE_BIS)

#if (BLE_ISO_MODE_0)
bool llm_link_active(uint8_t act_id)
{
    return (llm_env.act_info[act_id].state == LLM_CONNECTED);
}
#endif // (BLE_ISO_MODE_0)

#if BT_DUAL_MODE
bool llm_activity_ongoing_check(void)
{
    int act_id;

    // check if any BLE activities ongoing
    for (act_id = 0; act_id < BLE_ACTIVITY_MAX; act_id++)
    {
        if ((1L << llm_env.act_info[act_id].state) &
            ~((1L<<LLM_FREE)|(1L<<LLM_SCAN_RSVD)|(1L<<LLM_ADV_RSVD)|(1L<<LLM_PER_ADV_RSVD)|(1L<<LLM_CON_RSVD)))
        {
            break;
        }
    }

    return (act_id < BLE_ACTIVITY_MAX);
}
#endif //BT_DUAL_MODE


#if (BLE_ADV_LEGACY_ITF)
bool llm_is_adv_itf_legacy()
{
    return (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY);
}

void llm_adv_itf_extended_set()
{
    if (llm_env.adv_itf_version == LLM_ADV_ITF_UNKNOWN)
    {
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
    }
}
#endif //(BLE_ADV_LEGACY_ITF)

/// @} LLM
