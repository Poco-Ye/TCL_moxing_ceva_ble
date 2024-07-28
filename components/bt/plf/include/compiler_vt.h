/**
 ****************************************************************************************
 *
 * @file gcc/compiler.h
 *
 * @brief Definitions of compiler specific directives.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef _COMPILER_VT_H_
#define _COMPILER_VT_H_

#ifdef WIN32
#define __attribute__(...)
#define __BASE_FILE__ __FILE__
#define __asm(...)
#define __STATIC_INLINE static line
#endif

#define ROM_VT_FUN_(func) func
#ifdef CFG_ROM_VT
#define ROM_VT_FUNC(func) rom_##func
#else
#define ROM_VT_FUNC(func) func
#endif


#if defined(CFG_ROM_VT) && defined(CFG_ROM_VT_IMPORT) 

#define DebugE256SecretKey (*vti_DebugE256SecretKey)
#define rwip_param (*vti_rwip_param)
#define rwip_rf (*vti_rwip_rf)
#define co_default_bdaddr (*vti_co_default_bdaddr)
#define co_null_bdaddr (*vti_co_null_bdaddr)
#define co_null_key (*vti_co_null_key)
#define co_phy_mask_to_rate (*vti_co_phy_mask_to_rate)
#define co_phy_mask_to_value (*vti_co_phy_mask_to_value)
#define co_phy_to_rate (*vti_co_phy_to_rate)
#define co_phy_value_to_mask (*vti_co_phy_value_to_mask)
#define co_phypwr_mask_to_value (*vti_co_phypwr_mask_to_value)
#define co_phypwr_to_rate (*vti_co_phypwr_to_rate)
#define co_phypwr_value_to_mask (*vti_co_phypwr_value_to_mask)
#define co_rate_to_phy (*vti_co_rate_to_phy)
#define co_rate_to_phy_mask (*vti_co_rate_to_phy_mask)
#define co_rate_to_phypwr (*vti_co_rate_to_phypwr)
#define co_rate_to_phypwr_mask (*vti_co_rate_to_phypwr_mask)
#define co_sca2ppm (*vti_co_sca2ppm)
#define one_bits (*vti_one_bits)
#define rwip_priority (*vti_rwip_priority)
#define rwip_prog_delay (*vti_rwip_prog_delay)

#if 1
#define __builtin_clz rom___builtin_clz
#define __builtin_ctz rom___builtin_ctz
#define rand rom_rand
#define memcmp rom_memcmp
#define memcpy rom_memcpy
#define memset rom_memset
extern int __builtin_clz(unsigned int i);
extern int __builtin_ctz(unsigned int i);
extern int rand(void);
extern void *memcpy(void *dest, const void *src, unsigned int count);
extern int memcmp(const void *buf1, const void *buf2, unsigned int count);
extern void *memset(void *dest, int c, unsigned int count);
#endif
#define aes_encrypt rom_aes_encrypt
#define aes_rand rom_aes_rand
#define aes_rpa_gen rom_aes_rpa_gen
#define aes_rpa_resolve rom_aes_rpa_resolve
#define co_bdaddr_compare rom_co_bdaddr_compare
#define co_list_extract rom_co_list_extract
#define co_list_extract_sublist rom_co_list_extract_sublist
#define co_list_init rom_co_list_init
#define co_list_insert_after rom_co_list_insert_after
#define co_list_pool_init rom_co_list_pool_init
#define co_list_pop_front rom_co_list_pop_front
#define co_list_push_back rom_co_list_push_back
#define co_list_push_back_sublist rom_co_list_push_back_sublist
#define co_list_push_front rom_co_list_push_front
#define co_util_pack rom_co_util_pack
#define co_util_unpack rom_co_util_unpack
#define ecc_gen_dh_key rom_ecc_gen_dh_key
#define ecc_gen_new_public_key rom_ecc_gen_new_public_key
#define ecc_gen_new_secret_key rom_ecc_gen_new_secret_key
#define hci_ble_conhdl_register rom_hci_ble_conhdl_register
#define hci_ble_conhdl_unregister rom_hci_ble_conhdl_unregister
#define hci_send_2_host rom_hci_send_2_host
#define ke_free rom_ke_free
#define ke_malloc_system rom_ke_malloc_system
#define ke_msg_alloc rom_ke_msg_alloc
#define ke_msg_forward rom_ke_msg_forward
#define ke_msg_free rom_ke_msg_free
#define ke_msg_send rom_ke_msg_send
#define ke_msg_send_basic rom_ke_msg_send_basic
#define ke_state_get rom_ke_state_get
#define ke_state_set rom_ke_state_set
#define ke_task_create rom_ke_task_create
#define ke_timer_clear rom_ke_timer_clear
#define ke_timer_set rom_ke_timer_set
#define rwip_ch_ass_en_get rom_rwip_ch_ass_en_get
#define rwip_ch_ass_en_set rom_rwip_ch_ass_en_set
#define rwip_channel_assess_ble rom_rwip_channel_assess_ble
#define rwip_current_drift_get rom_rwip_current_drift_get
#define rwip_max_drift_get rom_rwip_max_drift_get
#define rwip_prevent_sleep_clear rom_rwip_prevent_sleep_clear
#define rwip_prevent_sleep_set rom_rwip_prevent_sleep_set
#define rwip_sca_get rom_rwip_sca_get
#define rwip_sw_int_req rom_rwip_sw_int_req
#define rwip_time_get rom_rwip_time_get
#define rwip_timer_alarm_set rom_rwip_timer_alarm_set
#define rwip_timer_arb_set rom_rwip_timer_arb_set
#define rwip_update_ch_map_with_ch_assess_ble rom_rwip_update_ch_map_with_ch_assess_ble


#endif // defined(CFG_ROM_VT) && defined(CFG_ROM_VT_IMPORT)


#endif // _COMPILER_VT_H_
