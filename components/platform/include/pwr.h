/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  uart.h
 * @brief
 * @author bingrui.chen
 * @date 2021年12月24日
 * @version 1.0
 * @Revision
 */
#ifndef PWR_H_
#define PWR_H_

#include "ms1008.h"
#include "ms_pwr.h"


#define pwr_register_user_sleep_cb(p_callback) ms_pwr_register_user_sleep_callback(p_callback)
#define pwr_register_flash_sleep_cb(p_callback)  ms_pwr_register_flash_sleep_callback(p_callback)
#define pwr_register_gpio_sleep_cb(p_callback)  ms_pwr_register_gpio_sleep_callback(p_callback)
#define pwr_register_user_wakeup_cb(p_callback)  ms_pwr_register_user_wakeup_callback(p_callback)


extern int32_t pwr_init();
extern int32_t pwr_enter_sleep();
extern int32_t pwr_after_wakeup();
extern int32_t pwr_exit();
extern int32_t pwr_get_pre_mode();
extern uint32_t pwr_getwakeup_resource();
extern int32_t pwr_config_wkup_sel(PWR_WAKEUP_SEL_Type pwr_wkup_sel);
extern int32_t pwr_config_sleep_mode(PWR_MODE_Type pwr_mode);


#endif /* PWR_H_ */
