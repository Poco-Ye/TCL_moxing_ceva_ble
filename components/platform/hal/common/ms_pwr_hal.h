/**
  * Copyright © 2021 by MooreSilicon. All rights reserved
  * @file  ms_pwr_hal.h
  * @brief
  * @author che.jiang
  * @date 2022年1月4日
  * @version 1.0
  * @Revision:
  */

#ifndef MS_PWR_HAL_H_
#define MS_PWR_HAL_H_

#include <ms1008.h>
#include "ms_pwr_ll.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PWR Hal clear wakeup souce mask
 * @retval None
 */
#define  ms_pwr_hal_set_lp_rst_mask(hsysctl, state) ms_pwr_ll_set_lp_rst_mask(hsysctl,state)


/**
 * @brief PWR Hal clear wakeup souce mask
 * @retval None
 */
#define  ms_pwr_hal_set_lp_ble_mask(hsysctl, state) ms_pwr_ll_set_lp_ble_mask(hsysctl, state)

/**
 * @brief PWR Hal clear wakeup souce mask
 * @retval None
 */
#define  ms_pwr_hal_clear_wk_csr(hsysctl)  ms_pwr_ll_clear_wk_csr(hsysctl)

/**
 * @brief PWR Hal clear wakeup souce mask
 * @retval None
 */
#define  ms_pwr_hal_enable_wk_csr_int(hsysctl) ms_pwr_ll_enable_wk_csr_int(hsysctl)


/**
 * @brief PWR Hal set wakeup souce mask
 * @retval None
 */
#define  ms_pwr_hal_disable_wk_csr_int(hsysctl) ms_pwr_ll_disable_wk_csr_int(hsysctl)


/**
 * @brief PWR Hal set pwr mode
 * @retval None
 */
#define  ms_pwr_hal_enable_pwr_pd(hsysctl) ms_pwr_ll_enable_pwr_pd(hsysctl)

/**
 * @brief PWR Hal set pwr mode
 * @retval None
 */
#define  ms_pwr_hal_enable_pwr_ds(hsysctl) ms_pwr_ll_enable_pwr_ds(hsysctl)

/**
 * @brief PWR Hal set pwr mode
 * @retval None
 */
#define  ms_pwr_hal_enable_pwr_slp(hsysctl) ms_pwr_ll_enable_pwr_slp(hsysctl)


#define ms_pwr_hal_set_sram0_pd(sysctl)  ms_pwr_ll_set_sram0_pd(sysctl)
#define ms_pwr_hal_set_sram1_pd(sysctl)  ms_pwr_ll_set_sram1_pd(sysctl)
#define ms_pwr_hal_set_sram2_pd(sysctl)  ms_pwr_ll_set_sram2_pd(sysctl)
#define ms_pwr_hal_set_retram_pd(sysctl) ms_pwr_ll_set_retram_pd(sysctl)
#define ms_pwr_hal_set_em_pd(sysctl)     ms_pwr_ll_set_em_pd(sysctl)
#define ms_pwr_hal_set_retem_pd(sysctl)  ms_pwr_ll_set_retem_pd(sysctl)
#define ms_pwr_hal_set_sram0_ds(sysctl)  ms_pwr_ll_set_sram0_ds(sysctl)
#define ms_pwr_hal_set_sram1_ds(sysctl)  ms_pwr_ll_set_sram1_ds(sysctl)
#define ms_pwr_hal_set_sram2_ds(sysctl)  ms_pwr_ll_set_sram2_ds(sysctl)
#define ms_pwr_hal_set_retram_ds(sysctl) ms_pwr_ll_set_retram_ds(sysctl)
#define ms_pwr_hal_set_em_ds(sysctl)     ms_pwr_ll_set_em_ds(sysctl)
#define ms_pwr_hal_set_retem_ds(sysctl)  ms_pwr_ll_set_retem_ds(sysctl)
#define ms_pwr_hal_ger_pre_mode(sysctl)  ms_pwr_ll_ger_pre_mode(sysctl)


#define ms_pwr_hal_enable_gpio_wakup(sysctl,state) ms_pwr_ll_enable_gpio_wakup(sysctl,state)
#define ms_pwr_hal_enable_ble_wakup(sysctl,state) ms_pwr_ll_enable_ble_wakup(sysctl,state)
#define ms_pwr_hal_enable_rtc_wakup(sysctl,state) ms_pwr_ll_enable_rtc_wakup(sysctl,state)

#define ms_pwr_hal_get_wakup_source(sysctl) ms_pwr_ll_get_wakup_source(sysctl)

#define ms_pwr_hal_get_wk_csr_state(sysctl) ms_pwr_ll_get_wk_csr_state(sysctl)



#ifdef __cplusplus
}
#endif

#endif /* MS_PWR_HAL_H_ */
