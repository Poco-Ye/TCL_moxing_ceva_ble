/**
 ****************************************************************************************
 *
 * @file gapm_info.c
 *
 * @brief Generic Access Profile Manager Device Information
 *
 * Copyright (C) RivieraWaves 2009-2020
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#include "gapm_int.h"    // GAP Manager Internals
#include "gapm.h"        // GAP Manager API
#include "hci.h"         // HCI interface definition
#include "co_bt.h"       // Bluetooth defines

#include "co_version.h"  // Version information
#include "co_utils.h"    // Bit Field manipulation

#include <string.h>      // memory copy

#if(BLE_HOST_PRESENT)
#include "gapm_le.h"
#include "gapm_le_adv.h"
#endif // (BLE_HOST_PRESENT)

#if(BT_HOST_PRESENT)
#include "gapm_bt.h"
#endif // (BT_HOST_PRESENT)
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
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */
#if(BLE_HOST_PRESENT)
__STATIC void gapm_info_list_size_cmd_cmp_evt_handler(uint16_t status, uint8_t size);
#endif // (BLE_HOST_PRESENT)

/*
 * HCI EVENT HANDLERS DEFINITIONS
 ****************************************************************************************
 */

#if (!EMB_PRESENT)
/**
 ****************************************************************************************
 * @brief Handles the read Bluetooth device version complete event.
 *
 * @param[in] opcode    HCI Command OP Code for command complete event and command status
 * @param[in] conhdl    Unused
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
__STATIC void gapm_hci_rd_local_ver_info_cmd_cmp_evt_handler(uint16_t opcode, uint16_t conhdl,
                                                    struct hci_rd_local_ver_info_cmd_cmp_evt const *p_evt)
{
    gapm_version_t* p_version = NULL;
    gapm_version_t  version;
    uint32_t        dummy;
    gapm_version_cb res_cb;

    gapm_proc_info_stop(&dummy, (gapm_proc_info_res_cb*) &res_cb);

    if(p_evt->status == CO_ERROR_NO_ERROR)
    {
        version.hci_ver     = p_evt->hci_ver;
        version.hci_subver  = p_evt->hci_rev;
        version.lmp_subver  = p_evt->lmp_subver;
        version.lmp_ver     = p_evt->lmp_ver;
        version.manuf_name  = p_evt->manuf_name;
        version.host_ver    = RWBT_SW_VERSION_MAJOR;
        version.host_subver = CO_SUBVERSION_BUILD(RWBT_SW_VERSION_MINOR, RWBT_SW_VERSION_BUILD);

        p_version = &(version);
    }

    // send back result
    res_cb(dummy, RW_ERR_HCI_TO_HL(p_evt->status), p_version);
}
#endif // (!EMB_PRESENT)

#if ((HL_LE_OBSERVER) || (HL_LE_PERIPHERAL))
/**
 ****************************************************************************************
 * @brief Handles LE read adv tx power level value.
 *
 * @param[in] opcode    HCI Command OP Code for command complete event and command status
 * @param[in] conhdl    Unused
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
__STATIC void gapm_hci_le_rd_adv_chnl_tx_pwr_cmd_cmp_evt_handler(uint16_t opcode, uint16_t conhdl,
                                                    struct hci_rd_adv_chnl_tx_pw_cmd_cmp_evt const *p_evt)
{
    uint32_t dummy;
    gapm_adv_tx_power_cb res_cb;

    gapm_proc_info_stop(&dummy, (gapm_proc_info_res_cb*) &res_cb);

    // send back result
    res_cb(dummy, RW_ERR_HCI_TO_HL(p_evt->status), p_evt->adv_tx_pw_lvl);
}
#endif // ((HL_LE_OBSERVER) || (HL_LE_PERIPHERAL))

#if(BLE_HOST_PRESENT)

/**
 ****************************************************************************************
 * @brief LE Read Suggested Default LE Data Length complete event handler.
  *
 * @param[in] opcode    HCI Command OP Code for command complete event and command status
 * @param[in] conhdl    Unused
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
__STATIC void gapm_hci_le_rd_suggted_dft_data_len_cmd_cmp_evt_handler(uint16_t opcode, uint16_t conhdl,
                                            struct hci_le_rd_suggted_dft_data_len_cmd_cmp_evt const *p_evt)
{
    gapm_sugg_dflt_data_len_t* p_info = NULL;
    gapm_sugg_dflt_data_len_t  info;
    uint32_t                   dummy;
    gapm_sugg_dflt_data_len_cb res_cb;

    gapm_proc_info_stop(&dummy, (gapm_proc_info_res_cb*) &res_cb);

    if(p_evt->status == CO_ERROR_NO_ERROR)
    {
        info.suggted_max_tx_octets = p_evt->suggted_max_tx_octets;
        info.suggted_max_tx_time   = p_evt->suggted_max_tx_time;

        p_info = &(info);
    }

    // send back result
    res_cb(dummy, RW_ERR_HCI_TO_HL(p_evt->status), p_info);
}

/**
 ****************************************************************************************
 * @brief LE Read Maximum LE Data Lenght complete event handler.
  *
 * @param[in] opcode    HCI Command OP Code for command complete event and command status
 * @param[in] conhdl    Unused
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
__STATIC void gapm_hci_le_rd_max_data_len_cmd_cmp_evt_handler(uint16_t opcode, uint16_t conhdl,
                                            struct hci_le_rd_max_data_len_cmd_cmp_evt const *p_evt)
{
    gapm_max_le_data_len_t* p_info = NULL;
    gapm_max_le_data_len_t  info;
    uint32_t                dummy;
    gapm_max_le_data_len_cb res_cb;

    gapm_proc_info_stop(&dummy, (gapm_proc_info_res_cb*) &res_cb);

    if(p_evt->status == CO_ERROR_NO_ERROR)
    {
        info.suppted_max_tx_octets = p_evt->suppted_max_tx_octets;
        info.suppted_max_tx_time   = p_evt->suppted_max_tx_time;
        info.suppted_max_rx_octets = p_evt->suppted_max_rx_octets;
        info.suppted_max_rx_time   = p_evt->suppted_max_rx_time;

        p_info = &(info);
    }

    // send back result
    res_cb(dummy, RW_ERR_HCI_TO_HL(p_evt->status), p_info);
}

/**
 ****************************************************************************************
 * @brief Handle reception of HCI Read Transmit Power command complete event.
 *
 * @param[in] opcode    HCI Command OP Code for command complete event and command status
 * @param[in] conhdl    Unused
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
__STATIC void gapm_hci_le_rd_tx_pwr_cmd_cmp_evt_handler(uint16_t opcode, uint16_t conhdl,
                                         struct hci_le_rd_tx_pwr_cmd_cmp_evt const *p_evt)
{
    gapm_tx_pwr_rng_t* p_rng = NULL;
    gapm_tx_pwr_rng_t  rng;
    uint32_t           dummy;
    gapm_tx_pwr_rng_cb res_cb;

    gapm_proc_info_stop(&dummy, (gapm_proc_info_res_cb*) &res_cb);

    if(p_evt->status == CO_ERROR_NO_ERROR)
    {
        rng.min_tx_pwr = p_evt->min_tx_pwr;
        rng.max_tx_pwr   = p_evt->max_tx_pwr;

        p_rng = &(rng);
    }

    // send back result
    res_cb(dummy, RW_ERR_HCI_TO_HL(p_evt->status), p_rng);
}

/**
 ****************************************************************************************
 * @brief Handle reception of HCI Read RF Path Compensation command complete event.
 *
 * @param[in] opcode    HCI Command OP Code for command complete event and command status
 * @param[in] conhdl    Unused
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
__STATIC void gapm_hci_le_rd_rf_path_comp_cmd_cmp_evt_handler(uint16_t opcode, uint16_t conhdl,
                                               struct hci_le_rd_rf_path_comp_cmd_cmp_evt const *p_evt)
{
    gapm_rf_path_comp_t* p_rf_path_comp = NULL;
    gapm_rf_path_comp_t  rf_path_comp;
    uint32_t             dummy;
    gapm_rf_path_comp_cb res_cb;

    gapm_proc_info_stop(&dummy, (gapm_proc_info_res_cb*) &res_cb);

    if(p_evt->status == CO_ERROR_NO_ERROR)
    {
        rf_path_comp.tx = p_evt->tx_path_comp;
        rf_path_comp.rx = p_evt->rx_path_comp;

        p_rf_path_comp = &(rf_path_comp);
    }

    // send back result
    res_cb(dummy, RW_ERR_HCI_TO_HL(p_evt->status), p_rf_path_comp);
}


#if (BLE_AOD | BLE_AOA)
/**
 ****************************************************************************************
 * @brief Handles Antenna information command complete event
 *
 * @param[in] opcode    HCI Command OP Code for command complete event and command status
 * @param[in] conhdl    Unused
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
__STATIC void gapm_hci_le_rd_antenna_inf_cmd_cmp_evt_list_handler(uint16_t opcode, uint16_t conhdl,
                                                struct hci_le_rd_antenna_inf_cmd_cmp_evt const *p_evt)
{
    gapm_antenna_inf_t* p_info = NULL;
    gapm_antenna_inf_t  info;
    uint32_t            dummy;
    gapm_antenna_inf_cb res_cb;

    gapm_proc_info_stop(&dummy, (gapm_proc_info_res_cb*) &res_cb);

    if(p_evt->status == CO_ERROR_NO_ERROR)
    {
        info.supp_switching_sampl_rates = p_evt->supp_switching_sampl_rates;
        info.antennae_num               = p_evt->antennae_num;
        info.max_switching_pattern_len  = p_evt->max_switching_pattern_len;
        info.max_cte_len                = p_evt->max_cte_len;
        p_info = &(info);
    }

    // send back result
    res_cb(dummy, RW_ERR_HCI_TO_HL(p_evt->status), p_info);
}
#endif // (BLE_AOD | BLE_AOA)



/**
 ****************************************************************************************
 * @brief Handle HCI Read White List Size command complete event
 *
 * @param[in] opcode    HCI Command OP Code for command complete event and command status
 * @param[in] conhdl    Unused
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
__STATIC void gapm_hci_le_rd_wlst_size_cmd_cmp_evt_handler(uint16_t opcode, uint16_t conhdl,
                                            struct hci_le_rd_wlst_size_cmd_cmp_evt const *p_evt)
{
    gapm_info_list_size_cmd_cmp_evt_handler(RW_ERR_HCI_TO_HL(p_evt->status), p_evt->wlst_size);
}

/**
 ****************************************************************************************
 * @brief Handle HCI Read Resolving List Size command complete event
 *
 * @param[in] opcode    HCI Command OP Code for command complete event and command status
 * @param[in] conhdl    Unused
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
__STATIC void gapm_hci_le_rd_ral_size_cmd_cmp_evt_handler(uint16_t opcode, uint16_t conhdl,
                                        struct hci_le_rd_rslv_list_size_cmd_cmp_evt const *p_evt)
{
    gapm_info_list_size_cmd_cmp_evt_handler(RW_ERR_HCI_TO_HL(p_evt->status), p_evt->size);
}

/**
 ****************************************************************************************
 * @brief Handle HCI Read Periodic Advertiser List Size command complete event
 *
 * @param[in] opcode    HCI Command OP Code for command complete event and command status
 * @param[in] conhdl    Unused
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
__STATIC void gapm_hci_le_rd_pal_size_cmd_cmp_evt_handler(uint16_t opcode, uint16_t conhdl,
                                        struct hci_le_rd_per_adv_list_size_cmd_cmp_evt const *p_evt)
{
    gapm_info_list_size_cmd_cmp_evt_handler(RW_ERR_HCI_TO_HL(p_evt->status), p_evt->size);
}


#if (HL_LE_BROADCASTER)
/**
 ****************************************************************************************
 * @brief Handle reception of HCI Read Maximum Advertising Data Length command complete
 * event.
 *
 * @param[in] opcode    HCI Command OP Code for command complete event and command status
 * @param[in] conhdl    Unused
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
__STATIC void gapm_hci_info_hci_le_read_max_adv_data_len_cmd_cmp_evt_handler(uint16_t opcode, uint16_t conhdl,
                                                           struct hci_le_rd_max_adv_data_len_cmd_cmp_evt const *p_evt)
{
    uint32_t            dummy;
    gapm_max_adv_len_cb res_cb;

    gapm_proc_info_stop(&dummy, (gapm_proc_info_res_cb*) &res_cb);

    // send back result
    res_cb(dummy, RW_ERR_HCI_TO_HL(p_evt->status), p_evt->max_adv_data_len);
}

/**
 ****************************************************************************************
 * @brief Handle reception of HCI Read Number of Supported Advertising Sets command complete
 * event. The read value may change over time.
 *
 * @param[in] opcode    HCI Command OP Code for command complete event and command status
 * @param[in] conhdl    Unused
 * @param[in] p_evt     Pointer to event parameters.
 *
 ****************************************************************************************
 */
__STATIC void gapm_info_hci_le_rd_nb_supp_adv_sets_cmd_cmp_evt_handler(uint16_t opcode, uint16_t conhdl,
                                                                  struct hci_le_rd_nb_supp_adv_sets_cmd_cmp_evt const *p_evt)
{
    uint32_t            dummy;
    gapm_max_adv_len_cb res_cb;

    gapm_proc_info_stop(&dummy, (gapm_proc_info_res_cb*) &res_cb);

    // send back result
    res_cb(dummy, RW_ERR_HCI_TO_HL(p_evt->status), p_evt->nb_supp_adv_sets);
}
#endif //(HL_LE_BROADCASTER)


/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/// @brief Common handler of Get list size procedures
__STATIC void gapm_info_list_size_cmd_cmp_evt_handler(uint16_t status, uint8_t size)
{
    uint32_t          dummy;
    gapm_list_size_cb res_cb;

    gapm_proc_info_stop(&dummy, (gapm_proc_info_res_cb*) &res_cb);

    // send back result
    res_cb(dummy, status, size);
}

#endif // (BLE_HOST_PRESENT)

/*
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint16_t gapm_get_identity(gap_bdaddr_t* p_addr)
{
    uint16_t status = GAP_ERR_INVALID_PARAM;

    if(p_addr != NULL)
    {
        memcpy(p_addr->addr, gapm_env.addr.addr, GAP_BD_ADDR_LEN);
        p_addr->addr_type = GETB(gapm_env.priv_cfg, GAPM_PRIV_CFG_PRIV_ADDR) ? ADDR_RAND : ADDR_PUBLIC;
        status = GAP_ERR_NO_ERROR;
    }

    return (status);
}


uint16_t gapm_get_version(uint32_t dummy, gapm_version_cb res_cb)
{
    #if (EMB_PRESENT)
    gapm_version_t  version;
    version.hci_ver     = RWBT_SW_VERSION_MAJOR;
    version.hci_subver  = CO_SUBVERSION_BUILD(RWBT_SW_VERSION_MINOR,RWBT_SW_VERSION_BUILD);
    version.lmp_ver     = RWBT_SW_VERSION_MAJOR;
    version.lmp_subver  = CO_SUBVERSION_BUILD(RWBT_SW_VERSION_MINOR,RWBT_SW_VERSION_BUILD);
    version.manuf_name  = RW_COMP_ID;
    version.host_ver    = RWBT_SW_VERSION_MAJOR;
    version.host_subver = CO_SUBVERSION_BUILD(RWBT_SW_VERSION_MINOR, RWBT_SW_VERSION_BUILD);
    res_cb(dummy, GAP_ERR_NO_ERROR, &version);
    return (GAP_ERR_NO_ERROR);
    #else
    return (GAPM_INFO_PROC_START(dummy, HCI_RD_LOCAL_VER_INFO_CMD_OPCODE, gapm_hci_rd_local_ver_info_cmd_cmp_evt_handler,
                                 res_cb));
    #endif // (EMB_PRESENT)
}

#if(BLE_HOST_PRESENT)
uint16_t gapm_get_adv_tx_power(uint32_t dummy, gapm_adv_tx_power_cb res_cb)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;
    #if ((HL_LE_OBSERVER) || (HL_LE_PERIPHERAL))
    status = GAPM_INFO_PROC_START(dummy, HCI_LE_RD_ADV_CHNL_TX_PW_CMD_OPCODE, gapm_hci_le_rd_adv_chnl_tx_pwr_cmd_cmp_evt_handler,
                                  res_cb);
    #endif // ((HL_LE_OBSERVER) || (HL_LE_PERIPHERAL))

    return (status);
}


uint16_t gapm_get_antenna_inf(uint32_t dummy, gapm_antenna_inf_cb res_cb)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;
    #if (BLE_AOD | BLE_AOA)
    status = GAPM_INFO_PROC_START(dummy, HCI_LE_RD_ANTENNA_INF_CMD_OPCODE, gapm_hci_le_rd_antenna_inf_cmd_cmp_evt_list_handler,
                                  res_cb);
    #endif // (BLE_AOD | BLE_AOA)

    return (status);
}

uint16_t gapm_get_sugg_dflt_data_len(uint32_t dummy, gapm_sugg_dflt_data_len_cb res_cb)
{
    return (GAPM_INFO_PROC_START(dummy, HCI_LE_RD_SUGGTED_DFT_DATA_LEN_CMD_OPCODE,
                                 gapm_hci_le_rd_suggted_dft_data_len_cmd_cmp_evt_handler, res_cb));
}

uint16_t gapm_get_max_le_data_len(uint32_t dummy, gapm_max_le_data_len_cb res_cb)
{
    return (GAPM_INFO_PROC_START(dummy, HCI_LE_RD_MAX_DATA_LEN_CMD_OPCODE,
                                 gapm_hci_le_rd_max_data_len_cmd_cmp_evt_handler, res_cb));
}


uint16_t gapm_get_wlist_size(uint32_t dummy, gapm_list_size_cb res_cb)
{
    return (GAPM_INFO_PROC_START(dummy, HCI_LE_RD_WLST_SIZE_CMD_OPCODE, gapm_hci_le_rd_wlst_size_cmd_cmp_evt_handler,
                                res_cb));
}

uint16_t gapm_get_pal_size(uint32_t dummy, gapm_list_size_cb res_cb)
{
    return (GAPM_INFO_PROC_START(dummy, HCI_LE_RD_PER_ADV_LIST_SIZE_CMD_OPCODE,
                                 gapm_hci_le_rd_ral_size_cmd_cmp_evt_handler, res_cb));
}

uint16_t gapm_get_ral_size(uint32_t dummy, gapm_list_size_cb res_cb)
{
    return (GAPM_INFO_PROC_START(dummy, HCI_LE_RD_RSLV_LIST_SIZE_CMD_OPCODE,
                                 gapm_hci_le_rd_pal_size_cmd_cmp_evt_handler, res_cb));
}

uint16_t gapm_get_nb_adv_set(uint32_t dummy, gapm_nb_adv_set_cb res_cb)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;
    #if (HL_LE_BROADCASTER)
    status = GAPM_INFO_PROC_START(dummy, HCI_LE_RD_NB_SUPP_ADV_SETS_CMD_OPCODE,
                                  gapm_info_hci_le_rd_nb_supp_adv_sets_cmd_cmp_evt_handler, res_cb);
    #endif //(HL_LE_BROADCASTER)
    return (status);
}

uint16_t gapm_get_max_adv_len(uint32_t dummy, gapm_max_adv_len_cb res_cb)
{
    uint16_t status = GAP_ERR_NOT_SUPPORTED;
    #if (HL_LE_BROADCASTER)
    status = GAPM_INFO_PROC_START(dummy, HCI_LE_RD_MAX_ADV_DATA_LEN_CMD_OPCODE,
                                  gapm_hci_info_hci_le_read_max_adv_data_len_cmd_cmp_evt_handler, res_cb);
    #endif //(HL_LE_BROADCASTER)
    return (status);
}

uint16_t gapm_get_tx_pwr_rng(uint32_t dummy, gapm_tx_pwr_rng_cb res_cb)
{
    return (GAPM_INFO_PROC_START(dummy, HCI_LE_RD_TX_PWR_CMD_OPCODE, gapm_hci_le_rd_tx_pwr_cmd_cmp_evt_handler,
                                res_cb));
}

uint16_t gapm_get_rf_path_comp(uint32_t dummy, gapm_rf_path_comp_cb res_cb)
{
    return (GAPM_INFO_PROC_START(dummy, HCI_LE_RD_RF_PATH_COMP_CMD_OPCODE,
                                 gapm_hci_le_rd_rf_path_comp_cmd_cmp_evt_handler, res_cb));
}
#endif // (BLE_HOST_PRESENT)
/// @} GAPM
