#ifdef CFG_ROM_VT


typedef int(*PFN_ROM_VT)(void);

#define VT_PATCH(func) extern int my_##func(); extern PFN_ROM_VT vti_##func; vti_##func = (PFN_ROM_VT)&my_##func;
#define VT_PATCH0(func) extern PFN_ROM_VT vti_##func; vti_##func = (PFN_ROM_VT)&func;
#define VT_PATCH1(func) extern int func(); extern PFN_ROM_VT vti_##func;vti_##func = (PFN_ROM_VT)&func;
#define VT_PATCH2(func, myfunc) extern int myfunc(); extern PFN_ROM_VT vti_##func; vti_##func = (PFN_ROM_VT)&myfunc;

void my_patch(void)
{
#if 1 // C库函数
#ifndef WIN32
	VT_PATCH2(__builtin_clz, __clzsi2)
	VT_PATCH2(__builtin_ctz, __ctzsi2)
#endif
	extern int rand(void);
	extern void *memcpy(void *dest, const void *src, unsigned int count);
	extern int memcmp(const void *buf1, const void *buf2, unsigned int count);
	extern void *memset(void *dest, int c, unsigned int count);
	VT_PATCH0(rand)
	VT_PATCH0(memcmp)
	VT_PATCH0(memcpy)
	VT_PATCH0(memset)
#endif

#if 1 // CevaLL内部没有定义，设置其对外部的引用
	VT_PATCH1(DebugE256SecretKey) // ptr
	VT_PATCH1(rwip_param)
	VT_PATCH1(rwip_rf)
	VT_PATCH1(co_default_bdaddr)
	VT_PATCH1(co_null_bdaddr)
	VT_PATCH1(co_null_key)
	VT_PATCH1(co_phy_mask_to_rate)
	VT_PATCH1(co_phy_mask_to_value)
	VT_PATCH1(co_phy_to_rate)
	VT_PATCH1(co_phy_value_to_mask)
	VT_PATCH1(co_phypwr_mask_to_value)
	VT_PATCH1(co_phypwr_to_rate)
	VT_PATCH1(co_phypwr_value_to_mask)
	VT_PATCH1(co_rate_to_phy)
	VT_PATCH1(co_rate_to_phy_mask)
	VT_PATCH1(co_rate_to_phypwr)
	VT_PATCH1(co_rate_to_phypwr_mask)
	VT_PATCH1(co_sca2ppm)
	VT_PATCH1(one_bits)
	VT_PATCH1(rwip_priority)
	VT_PATCH1(rwip_prog_delay)

	VT_PATCH1(aes_encrypt)
	VT_PATCH1(aes_rand)
	VT_PATCH1(aes_rpa_gen)
	VT_PATCH1(aes_rpa_resolve)
	VT_PATCH1(co_bdaddr_compare)
	VT_PATCH1(co_list_extract)
	VT_PATCH1(co_list_extract_sublist)
	VT_PATCH1(co_list_init)
	VT_PATCH1(co_list_insert_after)
	VT_PATCH1(co_list_pool_init)
	VT_PATCH1(co_list_pop_front)
	VT_PATCH1(co_list_push_back)
	VT_PATCH1(co_list_push_back_sublist)
	VT_PATCH1(co_list_push_front)
	VT_PATCH1(co_util_pack)
	VT_PATCH1(co_util_unpack)
	VT_PATCH1(ecc_gen_dh_key)
	VT_PATCH1(ecc_gen_new_public_key)
	VT_PATCH1(ecc_gen_new_secret_key)
	VT_PATCH1(hci_ble_conhdl_register)
	VT_PATCH1(hci_ble_conhdl_unregister)
	VT_PATCH1(hci_send_2_host)
	VT_PATCH1(ke_free)
	VT_PATCH1(ke_malloc_system)
	VT_PATCH1(ke_msg_alloc)
	VT_PATCH1(ke_msg_forward)
	VT_PATCH1(ke_msg_free)
	VT_PATCH1(ke_msg_send)
	VT_PATCH1(ke_msg_send_basic)
	VT_PATCH1(ke_state_get)
	VT_PATCH1(ke_state_set)
	VT_PATCH1(ke_task_create)
	VT_PATCH1(ke_timer_clear)
	VT_PATCH1(ke_timer_set)
	VT_PATCH1(rwip_ch_ass_en_get)
	VT_PATCH1(rwip_ch_ass_en_set)
	VT_PATCH1(rwip_channel_assess_ble)
	VT_PATCH1(rwip_current_drift_get)
	VT_PATCH1(rwip_max_drift_get)
	VT_PATCH1(rwip_prevent_sleep_clear)
	VT_PATCH1(rwip_prevent_sleep_set)
	VT_PATCH1(rwip_sca_get)
	VT_PATCH1(rwip_sw_int_req)
	VT_PATCH1(rwip_time_get)
	VT_PATCH1(rwip_timer_alarm_set)
	VT_PATCH1(rwip_timer_arb_set)
	VT_PATCH1(rwip_update_ch_map_with_ch_assess_ble)

#endif

#if 1 // CevaLL 内部已有定义，替换以打补丁
	VT_PATCH(lld_con_start)
	VT_PATCH(lld_con_stop)
	VT_PATCH(lld_con_llcp_tx)
	VT_PATCH(lld_con_data_tx)
	VT_PATCH(lld_con_data_flow_set)
	VT_PATCH(lld_con_param_update)
	VT_PATCH(lld_con_ch_map_update)
	VT_PATCH(lld_con_data_len_update)
	VT_PATCH(lld_con_phys_update)
	VT_PATCH(lld_con_tx_len_update_for_intv)
	VT_PATCH(lld_con_tx_len_update_for_rate)
	VT_PATCH(lld_con_event_counter_get)
	VT_PATCH(lld_con_tx_enc)
	VT_PATCH(lld_con_rx_enc)
	VT_PATCH(lld_con_enc_key_load)
	VT_PATCH(lld_con_current_tx_power_get)
	VT_PATCH(lld_con_rssi_get)
	VT_PATCH(lld_con_tx_power_get)
	VT_PATCH(lld_con_tx_power_adj)
	VT_PATCH(lld_con_apr_get)
	VT_PATCH(lld_con_rx_rate_get)
	VT_PATCH(lld_con_tx_rate_get)
	VT_PATCH(lld_con_tx_pwr_lvl_get)
	VT_PATCH(lld_con_remote_tx_pwr_set)
	VT_PATCH(lld_con_remote_tx_pwr_get)
	VT_PATCH(lld_con_path_loss_monitor_config)
	VT_PATCH(lld_con_path_loss_monitor_en)
	VT_PATCH(lld_con_rssi_update)
	VT_PATCH(lld_con_offset_get)
	VT_PATCH(lld_con_pref_slave_latency_set)
	VT_PATCH(lld_con_pref_slave_evt_dur_set)
	VT_PATCH(lld_con_init)
	VT_PATCH(lld_con_peer_sca_set)
	VT_PATCH(lld_con_time_get)
#endif

}

#endif // CFG_ROM_VT
