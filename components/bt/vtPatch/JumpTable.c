#ifdef CFG_ROM_VT



#define ROM_VT_ARGS  int p1, int p2, int p3, int p4, int p5, int p6, int p7, int p8, int p9, int p10, int p11, int p12
#define ROM_VT_ARGS_CALL  p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12
typedef int(*PFN_ROM_VT)(ROM_VT_ARGS);

#define VT_IMPORT_PTR(var) extern char var[]; char* vti_##var = var;

#define VT_IMPORT_FUNC0(func) \
	PFN_ROM_VT vti_##func = (PFN_ROM_VT)&func; \
	int rom_##func(ROM_VT_ARGS) { return vti_##func(ROM_VT_ARGS_CALL); }

#define VT_IMPORT_FUNC(func) extern int func(ROM_VT_ARGS); \
	PFN_ROM_VT vti_##func = (PFN_ROM_VT)&func; \
	int rom_##func(ROM_VT_ARGS) { return vti_##func(ROM_VT_ARGS_CALL); }

#define VT_IMPORT_FUNC2(func, myfunc) extern int myfunc(ROM_VT_ARGS); \
	PFN_ROM_VT vti_##func = (PFN_ROM_VT)&myfunc; \
	int rom_##func(ROM_VT_ARGS) { return vti_##func(ROM_VT_ARGS_CALL); }

#define VT_EXPORT_FUNC(func) extern int rom_##func(ROM_VT_ARGS); \
	PFN_ROM_VT vti_##func = (PFN_ROM_VT)&rom_##func; \
	int func(ROM_VT_ARGS) { return vti_##func(ROM_VT_ARGS_CALL); }

#ifdef CFG_ROM_VT_IMPORT

#if 1 // �����Ż���ROM
extern int rand(void);
extern void *memcpy(void *dest, const void *src, unsigned int count);
extern int memcmp(const void *buf1, const void *buf2, unsigned int count);
extern void *memset(void *dest, int c, unsigned int count);
#ifndef WIN32
VT_IMPORT_FUNC2(__builtin_clz, __clzsi2)
VT_IMPORT_FUNC2(__builtin_ctz, __ctzsi2)
#endif
VT_IMPORT_FUNC0(rand)
VT_IMPORT_FUNC0(memcmp)
VT_IMPORT_FUNC0(memcpy)
VT_IMPORT_FUNC0(memset)
#endif

VT_IMPORT_PTR(co_default_bdaddr)
VT_IMPORT_PTR(co_null_bdaddr)
VT_IMPORT_PTR(co_null_key)
VT_IMPORT_PTR(DebugE256SecretKey)
VT_IMPORT_PTR(rwip_param)
VT_IMPORT_PTR(rwip_rf)
VT_IMPORT_PTR(co_phy_mask_to_rate)
VT_IMPORT_PTR(co_phy_mask_to_value)
VT_IMPORT_PTR(co_phy_to_rate)
VT_IMPORT_PTR(co_phy_value_to_mask)
VT_IMPORT_PTR(co_phypwr_mask_to_value)
VT_IMPORT_PTR(co_phypwr_to_rate)
VT_IMPORT_PTR(co_phypwr_value_to_mask)
VT_IMPORT_PTR(co_rate_to_phy)
VT_IMPORT_PTR(co_rate_to_phy_mask)
VT_IMPORT_PTR(co_rate_to_phypwr)
VT_IMPORT_PTR(co_rate_to_phypwr_mask)
VT_IMPORT_PTR(co_sca2ppm)
VT_IMPORT_PTR(one_bits)
VT_IMPORT_PTR(rwip_priority)
VT_IMPORT_PTR(rwip_prog_delay)


VT_IMPORT_FUNC(aes_encrypt)
VT_IMPORT_FUNC(aes_rand)
VT_IMPORT_FUNC(aes_rpa_gen)
VT_IMPORT_FUNC(aes_rpa_resolve)
VT_IMPORT_FUNC(co_bdaddr_compare)
VT_IMPORT_FUNC(co_list_extract)
VT_IMPORT_FUNC(co_list_extract_sublist)
VT_IMPORT_FUNC(co_list_init)
VT_IMPORT_FUNC(co_list_insert_after)
VT_IMPORT_FUNC(co_list_pool_init)
VT_IMPORT_FUNC(co_list_pop_front)
VT_IMPORT_FUNC(co_list_push_back)
VT_IMPORT_FUNC(co_list_push_back_sublist)
VT_IMPORT_FUNC(co_list_push_front)
VT_IMPORT_FUNC(co_util_pack)
VT_IMPORT_FUNC(co_util_unpack)
VT_IMPORT_FUNC(ecc_gen_dh_key)
VT_IMPORT_FUNC(ecc_gen_new_public_key)
VT_IMPORT_FUNC(ecc_gen_new_secret_key)
VT_IMPORT_FUNC(hci_ble_conhdl_register)
VT_IMPORT_FUNC(hci_ble_conhdl_unregister)
VT_IMPORT_FUNC(hci_send_2_host)
VT_IMPORT_FUNC(ke_free)
VT_IMPORT_FUNC(ke_malloc_system)
VT_IMPORT_FUNC(ke_msg_alloc)
VT_IMPORT_FUNC(ke_msg_forward)
VT_IMPORT_FUNC(ke_msg_free)
VT_IMPORT_FUNC(ke_msg_send)
VT_IMPORT_FUNC(ke_msg_send_basic)
VT_IMPORT_FUNC(ke_state_get)
VT_IMPORT_FUNC(ke_state_set)
VT_IMPORT_FUNC(ke_task_create)
VT_IMPORT_FUNC(ke_timer_clear)
VT_IMPORT_FUNC(ke_timer_set)
VT_IMPORT_FUNC(rwip_ch_ass_en_get)
VT_IMPORT_FUNC(rwip_ch_ass_en_set)
VT_IMPORT_FUNC(rwip_channel_assess_ble)
VT_IMPORT_FUNC(rwip_current_drift_get)
VT_IMPORT_FUNC(rwip_max_drift_get)
VT_IMPORT_FUNC(rwip_prevent_sleep_clear)
VT_IMPORT_FUNC(rwip_prevent_sleep_set)
VT_IMPORT_FUNC(rwip_sca_get)
VT_IMPORT_FUNC(rwip_sw_int_req)
VT_IMPORT_FUNC(rwip_time_get)
VT_IMPORT_FUNC(rwip_timer_alarm_set)
VT_IMPORT_FUNC(rwip_timer_arb_set)
VT_IMPORT_FUNC(rwip_update_ch_map_with_ch_assess_ble)

#endif // CFG_ROM_VT_IMPORT


VT_EXPORT_FUNC(ble_util_nb_good_channels)
VT_EXPORT_FUNC(ble_util_pkt_dur_in_us)
VT_EXPORT_FUNC(ble_util_big_info_unpack)
VT_EXPORT_FUNC(ble_util_buf_init)
VT_EXPORT_FUNC(ble_util_buf_llcp_tx_alloc)
VT_EXPORT_FUNC(ble_util_buf_llcp_tx_free)
VT_EXPORT_FUNC(ble_util_buf_rx_alloc)
VT_EXPORT_FUNC(ble_util_buf_rx_free)
VT_EXPORT_FUNC(ble_util_buf_elt_rx_get)
VT_EXPORT_FUNC(ble_util_buf_acl_tx_alloc)
VT_EXPORT_FUNC(ble_util_buf_acl_tx_elt_get)
VT_EXPORT_FUNC(ble_util_buf_acl_tx_free)
VT_EXPORT_FUNC(ble_util_buf_adv_tx_alloc)
VT_EXPORT_FUNC(ble_util_buf_adv_tx_free)
VT_EXPORT_FUNC(rwble_init)
VT_EXPORT_FUNC(rwble_isr)
VT_EXPORT_FUNC(llc_init)
VT_EXPORT_FUNC(llc_cleanup)
VT_EXPORT_FUNC(llc_stop)
VT_EXPORT_FUNC(llc_start)
VT_EXPORT_FUNC(llc_role_get)
VT_EXPORT_FUNC(llc_llcp_state_set)
VT_EXPORT_FUNC(llc_proc_reg)
VT_EXPORT_FUNC(llc_proc_unreg)
VT_EXPORT_FUNC(llc_proc_id_get)
VT_EXPORT_FUNC(llc_proc_init)
VT_EXPORT_FUNC(llc_proc_state_get)
VT_EXPORT_FUNC(llc_proc_state_set)
VT_EXPORT_FUNC(llc_proc_get)
VT_EXPORT_FUNC(llc_proc_err_ind)
VT_EXPORT_FUNC(llc_proc_timer_set)
VT_EXPORT_FUNC(llc_proc_timer_pause_set)
VT_EXPORT_FUNC(llc_proc_collision_check)
VT_EXPORT_FUNC(ll_channel_map_ind_handler)
VT_EXPORT_FUNC(ll_min_used_channels_ind_handler)
VT_EXPORT_FUNC(hci_le_rd_chnl_map_cmd_handler)
VT_EXPORT_FUNC(llc_op_ch_map_upd_ind_handler)
VT_EXPORT_FUNC(llm_ch_map_update_ind_handler)
VT_EXPORT_FUNC(lld_ch_map_upd_cfm_handler)
VT_EXPORT_FUNC(ll_clk_acc_req_handler)
VT_EXPORT_FUNC(ll_clk_acc_rsp_handler)
VT_EXPORT_FUNC(llc_op_clk_acc_ind_handler)
VT_EXPORT_FUNC(llc_clk_acc_modify)
VT_EXPORT_FUNC(ll_connection_update_ind_handler)
VT_EXPORT_FUNC(ll_connection_param_req_handler)
VT_EXPORT_FUNC(ll_connection_param_rsp_handler)
VT_EXPORT_FUNC(hci_le_con_upd_cmd_handler)
VT_EXPORT_FUNC(hci_le_rem_con_param_req_reply_cmd_handler)
VT_EXPORT_FUNC(hci_le_rem_con_param_req_neg_reply_cmd_handler)
VT_EXPORT_FUNC(llc_op_con_upd_ind_handler)
VT_EXPORT_FUNC(lld_con_param_upd_cfm_handler)
VT_EXPORT_FUNC(lld_con_offset_upd_ind_handler)
VT_EXPORT_FUNC(llc_con_move_cbk)
VT_EXPORT_FUNC(ll_terminate_ind_handler)
VT_EXPORT_FUNC(lld_disc_ind_handler)
VT_EXPORT_FUNC(llc_stopped_ind_handler)
VT_EXPORT_FUNC(llc_op_disconnect_ind_handler)
VT_EXPORT_FUNC(hci_disconnect_cmd_handler)
VT_EXPORT_FUNC(llc_disconnect)
VT_EXPORT_FUNC(llc_init_term_proc)
VT_EXPORT_FUNC(ll_length_req_handler)
VT_EXPORT_FUNC(ll_length_rsp_handler)
VT_EXPORT_FUNC(hci_le_set_data_len_cmd_handler)
VT_EXPORT_FUNC(hci_vs_set_max_rx_size_and_time_cmd_handler)
VT_EXPORT_FUNC(llc_op_dl_upd_ind_handler)
VT_EXPORT_FUNC(dl_upd_proc_start)
VT_EXPORT_FUNC(ll_pause_enc_req_handler)
VT_EXPORT_FUNC(ll_pause_enc_rsp_handler)
VT_EXPORT_FUNC(ll_enc_req_handler)
VT_EXPORT_FUNC(ll_enc_rsp_handler)
VT_EXPORT_FUNC(ll_start_enc_req_handler)
VT_EXPORT_FUNC(ll_start_enc_rsp_handler)
VT_EXPORT_FUNC(hci_le_en_enc_cmd_handler)
VT_EXPORT_FUNC(hci_le_ltk_req_reply_cmd_handler)
VT_EXPORT_FUNC(hci_le_ltk_req_neg_reply_cmd_handler)
VT_EXPORT_FUNC(llc_encrypt_ind_handler)
VT_EXPORT_FUNC(llc_op_encrypt_ind_handler)
VT_EXPORT_FUNC(ll_feature_req_handler)
VT_EXPORT_FUNC(ll_slave_feature_req_handler)
VT_EXPORT_FUNC(ll_feature_rsp_handler)
VT_EXPORT_FUNC(hci_le_rd_rem_feats_cmd_handler)
VT_EXPORT_FUNC(llc_op_feats_exch_ind_handler)
VT_EXPORT_FUNC(llc_cmd_cmp_send)
VT_EXPORT_FUNC(llc_cmd_stat_send)
VT_EXPORT_FUNC(hci_vs_set_pref_slave_latency_cmd_handler)
VT_EXPORT_FUNC(hci_vs_set_pref_slave_evt_dur_cmd_handler)
VT_EXPORT_FUNC(hci_acl_data_handler)
VT_EXPORT_FUNC(lld_acl_rx_ind_handler)
VT_EXPORT_FUNC(lld_acl_tx_cfm_handler)
VT_EXPORT_FUNC(ll_ping_req_handler)
VT_EXPORT_FUNC(ll_ping_rsp_handler)
VT_EXPORT_FUNC(hci_rd_auth_payl_to_cmd_handler)
VT_EXPORT_FUNC(hci_wr_auth_payl_to_cmd_handler)
VT_EXPORT_FUNC(llc_op_le_ping_ind_handler)
VT_EXPORT_FUNC(llc_auth_payl_nearly_to_handler)
VT_EXPORT_FUNC(llc_auth_payl_real_to_handler)
VT_EXPORT_FUNC(llc_le_ping_restart)
VT_EXPORT_FUNC(llc_le_ping_set)
VT_EXPORT_FUNC(llc_ll_reject_ind_pdu_send)
VT_EXPORT_FUNC(llc_llcp_send)
VT_EXPORT_FUNC(llc_llcp_tx_check)
VT_EXPORT_FUNC(lld_llcp_rx_ind_handler)
VT_EXPORT_FUNC(lld_llcp_tx_cfm_handler)
VT_EXPORT_FUNC(llc_op_past_ind_handler)
VT_EXPORT_FUNC(ll_per_sync_ind_handler)
VT_EXPORT_FUNC(hci_le_per_adv_sync_transf_cmd_handler)
VT_EXPORT_FUNC(hci_le_per_adv_set_info_transf_cmd_handler)
VT_EXPORT_FUNC(hci_le_set_per_adv_sync_transf_param_cmd_handler)
VT_EXPORT_FUNC(ll_phy_req_handler)
VT_EXPORT_FUNC(ll_phy_rsp_handler)
VT_EXPORT_FUNC(ll_phy_update_ind_handler)
VT_EXPORT_FUNC(hci_le_rd_phy_cmd_handler)
VT_EXPORT_FUNC(hci_le_set_phy_cmd_handler)
VT_EXPORT_FUNC(llc_op_phy_upd_ind_handler)
VT_EXPORT_FUNC(lld_phy_upd_cfm_handler)
VT_EXPORT_FUNC(phy_upd_proc_start)
VT_EXPORT_FUNC(ll_pwr_ctrl_req_handler)
VT_EXPORT_FUNC(ll_pwr_ctrl_rsp_handler)
VT_EXPORT_FUNC(ll_pwr_change_ind_handler)
VT_EXPORT_FUNC(lld_con_pwr_ctrl_ind_handler)
VT_EXPORT_FUNC(lld_con_path_loss_change_ind_handler)
VT_EXPORT_FUNC(lld_con_pwr_change_ind_handler)
VT_EXPORT_FUNC(lld_cis_pwr_change_ind_handler)
VT_EXPORT_FUNC(llc_op_pwr_ctrl_ind_handler)
VT_EXPORT_FUNC(hci_rd_tx_pwr_lvl_cmd_handler)
VT_EXPORT_FUNC(hci_rd_rssi_cmd_handler)
VT_EXPORT_FUNC(hci_le_enh_rd_tx_pwr_lvl_cmd_handler)
VT_EXPORT_FUNC(hci_le_rd_remote_tx_pwr_lvl_cmd_handler)
VT_EXPORT_FUNC(hci_le_set_path_loss_rep_param_cmd_handler)
VT_EXPORT_FUNC(hci_le_set_path_loss_rep_en_cmd_handler)
VT_EXPORT_FUNC(hci_le_set_tx_power_rep_en_cmd_handler)
VT_EXPORT_FUNC(llc_op_ver_exch_ind_handler)
VT_EXPORT_FUNC(ll_version_ind_handler)
VT_EXPORT_FUNC(hci_rd_rem_ver_info_cmd_handler)
VT_EXPORT_FUNC(lld_ral_search)
VT_EXPORT_FUNC(lld_rxdesc_free)
VT_EXPORT_FUNC(lld_rxdesc_check)
VT_EXPORT_FUNC(lld_init)
VT_EXPORT_FUNC(lld_read_clock)
VT_EXPORT_FUNC(lld_rxdesc_buf_ready)
VT_EXPORT_FUNC(lld_rpa_renew)
VT_EXPORT_FUNC(lld_white_list_add)
VT_EXPORT_FUNC(lld_white_list_rem)
VT_EXPORT_FUNC(lld_per_adv_list_add)
VT_EXPORT_FUNC(lld_per_adv_list_rem)
VT_EXPORT_FUNC(lld_res_list_clear)
VT_EXPORT_FUNC(lld_res_list_add)
VT_EXPORT_FUNC(lld_res_list_rem)
VT_EXPORT_FUNC(lld_res_list_priv_mode_update)
VT_EXPORT_FUNC(lld_res_list_peer_rpa_get)
VT_EXPORT_FUNC(lld_res_list_local_rpa_get)
VT_EXPORT_FUNC(lld_aa_gen)
VT_EXPORT_FUNC(lld_calc_aux_rx)
VT_EXPORT_FUNC(lld_rx_timing_compute)
VT_EXPORT_FUNC(lld_ch_map_set)
VT_EXPORT_FUNC(lld_ch_idx_get)
VT_EXPORT_FUNC(lld_adv_init)
VT_EXPORT_FUNC(lld_adv_start)
VT_EXPORT_FUNC(lld_adv_stop)
VT_EXPORT_FUNC(lld_adv_adv_data_update)
VT_EXPORT_FUNC(lld_adv_scan_rsp_data_update)
VT_EXPORT_FUNC(lld_adv_duration_update)
VT_EXPORT_FUNC(lld_adv_rand_addr_update)
VT_EXPORT_FUNC(lld_adv_restart)
VT_EXPORT_FUNC(lld_adv_sync_info_update)
VT_EXPORT_FUNC(lld_ch_scan_start)
VT_EXPORT_FUNC(lld_ch_scan_stop)
VT_EXPORT_FUNC(lld_ch_scan_init)
VT_EXPORT_FUNC(lld_con_start)
VT_EXPORT_FUNC(lld_con_stop)
VT_EXPORT_FUNC(lld_con_llcp_tx)
VT_EXPORT_FUNC(lld_con_data_tx)
VT_EXPORT_FUNC(lld_con_data_flow_set)
VT_EXPORT_FUNC(lld_con_param_update)
VT_EXPORT_FUNC(lld_con_ch_map_update)
VT_EXPORT_FUNC(lld_con_data_len_update)
VT_EXPORT_FUNC(lld_con_phys_update)
VT_EXPORT_FUNC(lld_con_tx_len_update_for_intv)
VT_EXPORT_FUNC(lld_con_tx_len_update_for_rate)
VT_EXPORT_FUNC(lld_con_event_counter_get)
VT_EXPORT_FUNC(lld_con_tx_enc)
VT_EXPORT_FUNC(lld_con_rx_enc)
VT_EXPORT_FUNC(lld_con_enc_key_load)
VT_EXPORT_FUNC(lld_con_current_tx_power_get)
VT_EXPORT_FUNC(lld_con_rssi_get)
VT_EXPORT_FUNC(lld_con_tx_power_get)
VT_EXPORT_FUNC(lld_con_tx_power_adj)
VT_EXPORT_FUNC(lld_con_apr_get)
VT_EXPORT_FUNC(lld_con_rx_rate_get)
VT_EXPORT_FUNC(lld_con_tx_rate_get)
VT_EXPORT_FUNC(lld_con_tx_pwr_lvl_get)
VT_EXPORT_FUNC(lld_con_remote_tx_pwr_set)
VT_EXPORT_FUNC(lld_con_remote_tx_pwr_get)
VT_EXPORT_FUNC(lld_con_path_loss_monitor_config)
VT_EXPORT_FUNC(lld_con_path_loss_monitor_en)
VT_EXPORT_FUNC(lld_con_rssi_update)
VT_EXPORT_FUNC(lld_con_offset_get)
VT_EXPORT_FUNC(lld_con_pref_slave_latency_set)
VT_EXPORT_FUNC(lld_con_pref_slave_evt_dur_set)
VT_EXPORT_FUNC(lld_con_init)
VT_EXPORT_FUNC(lld_con_peer_sca_set)
VT_EXPORT_FUNC(lld_con_time_get)
VT_EXPORT_FUNC(lld_init_connect_req_pack)
VT_EXPORT_FUNC(lld_init_init)
VT_EXPORT_FUNC(lld_init_start)
VT_EXPORT_FUNC(lld_init_stop)
VT_EXPORT_FUNC(lld_init_rand_addr_update)
VT_EXPORT_FUNC(lld_per_adv_init)
VT_EXPORT_FUNC(lld_per_adv_start)
VT_EXPORT_FUNC(lld_per_adv_stop)
VT_EXPORT_FUNC(lld_per_adv_data_update)
VT_EXPORT_FUNC(lld_per_adv_sync_info_get)
VT_EXPORT_FUNC(lld_per_adv_init_info_get)
VT_EXPORT_FUNC(lld_per_adv_ch_map_update)
VT_EXPORT_FUNC(lld_per_adv_info_get)
VT_EXPORT_FUNC(lld_scan_init)
VT_EXPORT_FUNC(lld_scan_restart)
VT_EXPORT_FUNC(lld_scan_start)
VT_EXPORT_FUNC(lld_scan_stop)
VT_EXPORT_FUNC(lld_scan_params_update)
VT_EXPORT_FUNC(lld_scan_create_sync)
VT_EXPORT_FUNC(lld_scan_create_sync_cancel)
VT_EXPORT_FUNC(lld_scan_rand_addr_update)
VT_EXPORT_FUNC(lld_sync_init)
VT_EXPORT_FUNC(lld_sync_start)
VT_EXPORT_FUNC(lld_sync_ch_map_update)
VT_EXPORT_FUNC(lld_sync_stop)
VT_EXPORT_FUNC(lld_sync_info_get)
VT_EXPORT_FUNC(lld_test_start)
VT_EXPORT_FUNC(lld_test_stop)
VT_EXPORT_FUNC(lld_test_init)
VT_EXPORT_FUNC(llm_per_adv_chain_dur)
VT_EXPORT_FUNC(llm_cmd_cmp_send)
VT_EXPORT_FUNC(llm_cmd_stat_send)
VT_EXPORT_FUNC(llm_is_dev_connected)
VT_EXPORT_FUNC(llm_is_dev_synced)
VT_EXPORT_FUNC(llm_dev_list_empty_entry)
VT_EXPORT_FUNC(llm_dev_list_search)
VT_EXPORT_FUNC(llm_ral_search)
VT_EXPORT_FUNC(llm_ral_is_empty)
VT_EXPORT_FUNC(llm_ch_map_update)
VT_EXPORT_FUNC(llm_init)
VT_EXPORT_FUNC(llm_link_disc)
VT_EXPORT_FUNC(llm_clk_acc_set)
VT_EXPORT_FUNC(llm_master_ch_map_get)
VT_EXPORT_FUNC(llm_le_evt_mask_check)
VT_EXPORT_FUNC(llm_le_features_get)
VT_EXPORT_FUNC(llm_tx_path_comp_get)
VT_EXPORT_FUNC(llm_rx_path_comp_get)
VT_EXPORT_FUNC(llm_ral_get)
VT_EXPORT_FUNC(llm_plan_elt_get)
VT_EXPORT_FUNC(llm_activity_free_get)
VT_EXPORT_FUNC(llm_activity_free_set)
VT_EXPORT_FUNC(llm_adv_hdl_to_id)
VT_EXPORT_FUNC(hci_le_rd_adv_ch_tx_pw_cmd_handler)
VT_EXPORT_FUNC(hci_le_set_ext_adv_param_cmd_handler)
VT_EXPORT_FUNC(hci_le_set_adv_set_rand_addr_cmd_handler)
VT_EXPORT_FUNC(hci_le_set_ext_adv_data_cmd_handler)
VT_EXPORT_FUNC(hci_le_set_ext_scan_rsp_data_cmd_handler)
VT_EXPORT_FUNC(hci_le_set_ext_adv_en_cmd_handler)
VT_EXPORT_FUNC(hci_le_set_per_adv_param_cmd_handler)
VT_EXPORT_FUNC(hci_le_set_per_adv_data_cmd_handler)
VT_EXPORT_FUNC(hci_le_set_per_adv_en_cmd_handler)
VT_EXPORT_FUNC(hci_le_rmv_adv_set_cmd_handler)
VT_EXPORT_FUNC(hci_le_clear_adv_sets_cmd_handler)
VT_EXPORT_FUNC(llm_adv_act_id_get)
VT_EXPORT_FUNC(llm_adv_set_id_get)
VT_EXPORT_FUNC(hci_vs_le_ch_scan_cmd_handler)
VT_EXPORT_FUNC(hci_vs_le_ch_scan_end_cmd_handler)
VT_EXPORT_FUNC(hci_le_ext_create_con_cmd_handler)
VT_EXPORT_FUNC(hci_le_create_con_cancel_cmd_handler)
VT_EXPORT_FUNC(hci_le_set_ext_scan_param_cmd_handler)
VT_EXPORT_FUNC(hci_le_set_ext_scan_en_cmd_handler)
VT_EXPORT_FUNC(hci_le_per_adv_create_sync_cmd_handler)
VT_EXPORT_FUNC(hci_le_per_adv_create_sync_cancel_cmd_handler)
VT_EXPORT_FUNC(hci_le_per_adv_term_sync_cmd_handler)
VT_EXPORT_FUNC(hci_le_add_dev_to_per_adv_list_cmd_handler)
VT_EXPORT_FUNC(hci_le_rmv_dev_from_per_adv_list_cmd_handler)
VT_EXPORT_FUNC(hci_le_clear_per_adv_list_cmd_handler)
VT_EXPORT_FUNC(hci_le_rd_per_adv_list_size_cmd_handler)
VT_EXPORT_FUNC(hci_le_set_per_adv_rec_en_cmd_handler)
VT_EXPORT_FUNC(llm_scan_sync_acad_attach)
VT_EXPORT_FUNC(llm_scan_sync_info_get)
VT_EXPORT_FUNC(llm_per_adv_sync)
VT_EXPORT_FUNC(llm_scan_sync_acad_detach)
VT_EXPORT_FUNC(hci_le_rx_test_v1_cmd_handler)
VT_EXPORT_FUNC(hci_le_tx_test_v1_cmd_handler)
VT_EXPORT_FUNC(hci_le_rx_test_v2_cmd_handler)
VT_EXPORT_FUNC(hci_le_tx_test_v2_cmd_handler)
VT_EXPORT_FUNC(hci_le_rx_test_v3_cmd_handler)
VT_EXPORT_FUNC(hci_le_tx_test_v3_cmd_handler)
VT_EXPORT_FUNC(hci_le_tx_test_v4_cmd_handler)
VT_EXPORT_FUNC(hci_le_test_end_cmd_handler)
VT_EXPORT_FUNC(sch_alarm_init)
VT_EXPORT_FUNC(sch_alarm_timer_isr)
VT_EXPORT_FUNC(sch_alarm_set)
VT_EXPORT_FUNC(sch_alarm_clear)
VT_EXPORT_FUNC(sch_arb_init)
VT_EXPORT_FUNC(sch_arb_insert)
VT_EXPORT_FUNC(sch_arb_remove)
VT_EXPORT_FUNC(sch_arb_event_start_isr)
VT_EXPORT_FUNC(sch_arb_sw_isr)
VT_EXPORT_FUNC(sch_plan_init)
VT_EXPORT_FUNC(sch_plan_set)
VT_EXPORT_FUNC(sch_plan_rem)
VT_EXPORT_FUNC(sch_plan_req)
VT_EXPORT_FUNC(sch_plan_chk)
VT_EXPORT_FUNC(sch_plan_shift)
VT_EXPORT_FUNC(sch_prog_rx_isr)
VT_EXPORT_FUNC(sch_prog_tx_isr)
VT_EXPORT_FUNC(sch_prog_skip_isr)
VT_EXPORT_FUNC(sch_prog_end_isr)
VT_EXPORT_FUNC(sch_prog_init)
VT_EXPORT_FUNC(sch_prog_fifo_isr)
VT_EXPORT_FUNC(sch_prog_push)
VT_EXPORT_FUNC(sch_slice_compute)
VT_EXPORT_FUNC(sch_slice_init)
VT_EXPORT_FUNC(sch_slice_bg_add)
VT_EXPORT_FUNC(sch_slice_bg_remove)
VT_EXPORT_FUNC(sch_slice_fg_add)
VT_EXPORT_FUNC(sch_slice_fg_remove)
VT_EXPORT_FUNC(sch_slice_per_add)
VT_EXPORT_FUNC(sch_slice_per_remove)

VT_EXPORT_FUNC(hci_command_llc_handler)
VT_EXPORT_FUNC(lld_scan_req_ind_handler)
VT_EXPORT_FUNC(lld_adv_end_ind_handler)
VT_EXPORT_FUNC(lld_per_adv_end_ind_handler)
VT_EXPORT_FUNC(lld_ch_scan_end_ind_handler)
VT_EXPORT_FUNC(hci_command_llm_handler)
VT_EXPORT_FUNC(lld_init_end_ind_handler)
VT_EXPORT_FUNC(llm_scan_period_to_handler)
VT_EXPORT_FUNC(lld_adv_rep_ind_handler)
VT_EXPORT_FUNC(lld_sync_start_req_handler)
VT_EXPORT_FUNC(lld_per_adv_rep_ind_handler)
VT_EXPORT_FUNC(lld_per_adv_rx_end_ind_handler)
VT_EXPORT_FUNC(lld_scan_end_ind_handler)
VT_EXPORT_FUNC(lld_test_end_ind_handler)


#endif // CFG_ROM_VT
