/**
  * Copyright © 2021 by MooreSilicon. All rights reserved
  * @file  ms_pwr_hll.h
  * @brief
  * @author che.jiang
  * @date 2022年1月4日
  * @version 1.0
  * @Revision:
  */

#ifndef MS_PWR_LL_H_
#define MS_PWR_LL_H_

#include <ms1008.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif



/** @defgroup PRE PWR mode Type
  * @{
  */
#define PRE_PWR_POWERDOWN                         0x00000000U
#define PRE_PWR_DEEPSLEEP                         0x00000002U
#define PRE_PWR_SLEEP                             0x00000004U
#define PRE_PWR_ACTIVE                            0x00000008U

/** @defgroup RF mode Type
  * @{
  */
#define RF_MODE_SW_DEEPSLEEP                      0x00000001U
#define RF_MODE_SW_SLEEP                          0x00000002U
#define RF_MODE_SW_STANDBY                        0x00000004U
#define RF_MODE_SW_TX                             0x00000008U
#define RF_MODE_SW_RX                             0x00000010U


/**
 * @brief  Clear pwr wakeup mask
 * @param  DMAC_Type *dmac: DMAC Instance
 * @param  uint32_t channel: 0~5
 * @retval None
 */
__STATIC_FORCEINLINE void ms_pwr_ll_clear_wk_csr(SysCtrlRegs_Type *sysctl)
{
    CLEAR_BIT(sysctl->WKUP_CSR, (1UL << (SYS_CTRL_WKUP_CSR_GPIO_WAKUP_MASK_POS)));
    CLEAR_BIT(sysctl->WKUP_CSR, (1UL << (SYS_CTRL_WKUP_CSR_UART2_WAKUP_MASK_POS)));
    CLEAR_BIT(sysctl->WKUP_CSR, (1UL << (SYS_CTRL_WKUP_CSR_RTC_WAKUP_MASK_POS)));
    CLEAR_BIT(sysctl->WKUP_CSR, (1UL << (SYS_CTRL_WKUP_CSR_BLE_WAKUP_MASK_POS)));
}

/**
 * @brief  enable pwr wakeup csr int
 * @param  SysCtrlRegs_Type *sysctl: sysctl Instance
 * @retval None
 */
__STATIC_FORCEINLINE void ms_pwr_ll_enable_wk_csr_int(SysCtrlRegs_Type *sysctl)
{
    SET_BIT(sysctl->WKUP_CSR, (1UL << (SYS_CTRL_WKUP_CSR_INT_EN_POS)));
}

/**
 * @brief  disable pwr wakeup csr int
 * @param  SysCtrlRegs_Type *sysctl: sysctl Instance
 * @retval None
 */
__STATIC_FORCEINLINE void ms_pwr_ll_disable_wk_csr_int(SysCtrlRegs_Type *sysctl)
{
    CLEAR_BIT(sysctl->WKUP_CSR, (1UL << (SYS_CTRL_WKUP_CSR_INT_EN_POS)));
}


/**
 * @brief  disable pwr wakeup csr int
 * @param  SysCtrlRegs_Type *sysctl: sysctl Instance
 * @retval None
 */
__STATIC_FORCEINLINE void ms_pwr_ll_disble_wk_csr_int(SysCtrlRegs_Type *sysctl)
{
    CLEAR_BIT(sysctl->WKUP_CSR, (1UL << (SYS_CTRL_WKUP_CSR_INT_EN_POS)));
}

/**
 * @brief  disable pwr wakeup csr int
 * @param  SysCtrlRegs_Type *sysctl: sysctl Instance
 * @retval None
 */
__STATIC_FORCEINLINE uint32_t ms_pwr_ll_get_wk_csr_state(SysCtrlRegs_Type *sysctl)
{
    return READ_REG(sysctl->WKUP_CSR);
}


/**
 * @brief  enable gpio wakeup during the deepsleep mode
 * @param  SysCtrlRegs_Type *sysctl: sysctl Instance
 * @retval None
 */
__STATIC_FORCEINLINE void ms_pwr_ll_enable_gpio_wakup(SysCtrlRegs_Type *sysctl, bool state)
{
    if(state == true)
        CLEAR_BIT(sysctl->WKUP_CSR, (1UL << (SYS_CTRL_WKUP_CSR_GPIO_WAKUP_MASK_POS)));
    else
		SET_BIT(sysctl->WKUP_CSR, (1UL << (SYS_CTRL_WKUP_CSR_GPIO_WAKUP_MASK_POS)));
}

/**
 * @brief  enable ble wakeup during the deepsleep mode
 * @param  SysCtrlRegs_Type *sysctl: sysctl Instance
 * @retval None
 */
__STATIC_FORCEINLINE void ms_pwr_ll_enable_ble_wakup(SysCtrlRegs_Type *sysctl, bool state)
{
    if(state == true)
        CLEAR_BIT(sysctl->WKUP_CSR, (1UL << (SYS_CTRL_WKUP_CSR_BLE_WAKUP_MASK_POS)));
    else
        SET_BIT(sysctl->WKUP_CSR, (1UL << (SYS_CTRL_WKUP_CSR_BLE_WAKUP_MASK_POS)));		
}


/**
 * @brief  enable rtc wakeup during the deepsleep mode
 * @param  SysCtrlRegs_Type *sysctl: sysctl Instance
 * @retval None
 */
__STATIC_FORCEINLINE void ms_pwr_ll_enable_rtc_wakup(SysCtrlRegs_Type *sysctl, bool state)
{
    if(state == true)
        CLEAR_BIT(sysctl->WKUP_CSR, (1UL << (SYS_CTRL_WKUP_CSR_RTC_WAKUP_MASK_POS)));
    else
        SET_BIT(sysctl->WKUP_CSR, (1UL << (SYS_CTRL_WKUP_CSR_RTC_WAKUP_MASK_POS)));     
}


/**
 * @brief  read  wakeup resource during the deepsleep mode
 * @param  SysCtrlRegs_Type *sysctl: sysctl Instance
 * @retval    0: unknow resource
 *            1: GPIO wakeup 
 *            2: uart2
 *            3: rtc
 *            4: ble
 */
__STATIC_FORCEINLINE uint32_t ms_pwr_ll_get_wakup_source(SysCtrlRegs_Type *sysctl)
{

	if(READ_BIT(sysctl->WKUP_CSR,SYS_CTRL_WKUP_CSR_GPIO_WAKUP_INT) == SYS_CTRL_WKUP_CSR_GPIO_WAKUP_INT)
        return 1;                     // bit 4  GPIO
    else if(READ_BIT(sysctl->WKUP_CSR,SYS_CTRL_WKUP_CSR_UART2_WAKUP_INT) == SYS_CTRL_WKUP_CSR_UART2_WAKUP_INT)  // bit 5 uart2
        return 2;
    else if(READ_BIT(sysctl->WKUP_CSR,SYS_CTRL_WKUP_CSR_RTC_WAKUP_INT) == SYS_CTRL_WKUP_CSR_RTC_WAKUP_INT)  // bit 6 rtc
        return 3;	
    else if( READ_BIT(sysctl->WKUP_CSR,SYS_CTRL_WKUP_CSR_BLE_WAKUP_INT) == SYS_CTRL_WKUP_CSR_BLE_WAKUP_INT)  // bit 8 ble
        return 4;	
	else
		return 0;
}


/**
 * @brief   pwr lp keep config
 * @param  SysCtrlRegs_Type *sysctl: sysctl Instance
 * @retval None
 */
__STATIC_FORCEINLINE void ms_pwr_ll_set_lp_rst_mask(SysCtrlRegs_Type *sysctl, uint32_t val)
{
    MODIFY_REG(sysctl->LP_KEEP, SYS_CTRL_LP_RST_MASK_MASK, (val << SYS_CTRL_LP_RST_MASK_POS));
}

/**
 * @brief  enable pwr wakeup csr int
 * @param  SysCtrlRegs_Type *sysctl: sysctl Instance
 * @retval None
 */
__STATIC_FORCEINLINE void ms_pwr_ll_set_lp_ble_mask(SysCtrlRegs_Type *sysctl, uint32_t val)
{
    MODIFY_REG(sysctl->LP_KEEP, SYS_CTRL_BLE_WKUP_DIS_MASK, (val << SYS_CTRL_BLE_WKUP_DIS_POS));
}

/**
 * @brief   pwr lp keep config
 * @param  SysCtrlRegs_Type *sysctl: sysctl Instance
 * @retval None
 */
__STATIC_FORCEINLINE void ms_pwr_ll_set_ble_wkup_dis(SysCtrlRegs_Type *sysctl,  uint32_t mask_val)
{
    MODIFY_REG(sysctl->WKUP_CSR, SYS_CTRL_BLE_WKUP_DIS_MASK, (mask_val << SYS_CTRL_BLE_WKUP_DIS_POS));
}

/**
 * @brief   pwr lp keep config
 * @param  SysCtrlRegs_Type *sysctl: sysctl Instance
 * @retval None
 */
__STATIC_FORCEINLINE void ms_pwr_ll_enable_pwr_pd(SysCtrlRegs_Type *sysctl)
{
    SET_BIT(sysctl->PWR_EN, SYS_CTRL_PWR_EN_PD_MASK);
}

/**
 * @brief   pwr lp keep config
 * @param  SysCtrlRegs_Type *sysctl: sysctl Instance
 * @retval None
 */
__STATIC_FORCEINLINE void ms_pwr_ll_enable_pwr_ds(SysCtrlRegs_Type *sysctl)
{
    SET_BIT(sysctl->PWR_EN, SYS_CTRL_PWR_EN_DS_MASK);
}

/**
 * @brief   pwr lp keep config
 * @param  SysCtrlRegs_Type *sysctl: sysctl Instance
 * @retval None
 */
__STATIC_FORCEINLINE void ms_pwr_ll_enable_pwr_slp(SysCtrlRegs_Type *sysctl)
{
    SET_BIT(sysctl->PWR_EN, SYS_CTRL_PWR_EN_SLP_MASK);
}

 /**
 * @brief   pwr lp keep config
 * @param  SysCtrlRegs_Type *sysctl: sysctl Instance
 * @retval None
 */
__STATIC_FORCEINLINE void ms_pwr_ll_set_sram0_pd(SysCtrlRegs_Type *sysctl)
{
    SET_BIT(sysctl->RAM_CTRL, SYS_CTRL_RAM_CTRL_SRAM0_SD_EN_MASK);
}

/**
* @brief   pwr lp keep config
* @param  SysCtrlRegs_Type *sysctl: sysctl Instance
* @retval None
*/
__STATIC_FORCEINLINE void ms_pwr_ll_set_sram1_pd(SysCtrlRegs_Type *sysctl)
{
   SET_BIT(sysctl->RAM_CTRL, SYS_CTRL_RAM_CTRL_SRAM1_SD_EN_MASK);
}

/**
* @brief   pwr lp keep config
* @param  SysCtrlRegs_Type *sysctl: sysctl Instance
* @retval None
*/
__STATIC_FORCEINLINE void ms_pwr_ll_set_sram2_pd(SysCtrlRegs_Type *sysctl)
{
   SET_BIT(sysctl->RAM_CTRL, SYS_CTRL_RAM_CTRL_SRAM2_SD_EN_MASK);
}

/**
* @brief   pwr lp keep config
* @param  SysCtrlRegs_Type *sysctl: sysctl Instance
* @retval None
*/
__STATIC_FORCEINLINE void ms_pwr_ll_set_retram_pd(SysCtrlRegs_Type *sysctl)
{
   SET_BIT(sysctl->RAM_CTRL, SYS_CTRL_RAM_CTRL_RETRAM_SD_EN_MASK);
}

/**
* @brief   pwr lp keep config
* @param  SysCtrlRegs_Type *sysctl: sysctl Instance
* @retval None
*/
__STATIC_FORCEINLINE void ms_pwr_ll_set_em_pd(SysCtrlRegs_Type *sysctl)
{
   SET_BIT(sysctl->RAM_CTRL, SYS_CTRL_RAM_CTRL_EM_NRM_SD_EN_MASK);
}

/**
* @brief   pwr lp keep config
* @param  SysCtrlRegs_Type *sysctl: sysctl Instance
* @retval None
*/
__STATIC_FORCEINLINE void ms_pwr_ll_set_retem_pd(SysCtrlRegs_Type *sysctl)
{
   SET_BIT(sysctl->RAM_CTRL, SYS_CTRL_RAM_CTRL_EM_RET_SD_EN_MASK);
}

/**
* @brief   pwr lp keep config
* @param  SysCtrlRegs_Type *sysctl: sysctl Instance
* @retval None
*/
__STATIC_FORCEINLINE void ms_pwr_ll_set_sram0_ds(SysCtrlRegs_Type *sysctl)
{
   SET_BIT(sysctl->RAM_CTRL, SYS_CTRL_RAM_CTRL_SRAM0_DS_EN_MASK);
}

/**
* @brief   pwr lp keep config
* @param  SysCtrlRegs_Type *sysctl: sysctl Instance
* @retval None
*/
__STATIC_FORCEINLINE void ms_pwr_ll_set_sram1_ds(SysCtrlRegs_Type *sysctl)
{
  SET_BIT(sysctl->RAM_CTRL, SYS_CTRL_RAM_CTRL_SRAM1_DS_EN_MASK);
}

/**
* @brief   pwr lp keep config
* @param  SysCtrlRegs_Type *sysctl: sysctl Instance
* @retval None
*/
__STATIC_FORCEINLINE void ms_pwr_ll_set_sram2_ds(SysCtrlRegs_Type *sysctl)
{
  SET_BIT(sysctl->RAM_CTRL, SYS_CTRL_RAM_CTRL_SRAM2_DS_EN_MASK);
}

/**
* @brief   pwr lp keep config
* @param  SysCtrlRegs_Type *sysctl: sysctl Instance
* @retval None
*/
__STATIC_FORCEINLINE void ms_pwr_ll_set_retram_ds(SysCtrlRegs_Type *sysctl)
{
   SET_BIT(sysctl->RAM_CTRL, SYS_CTRL_RAM_CTRL_RETRAM_DS_EN_MASK);
}

/**
* @brief   pwr lp keep config
* @param  SysCtrlRegs_Type *sysctl: sysctl Instance
* @retval None
*/
__STATIC_FORCEINLINE void ms_pwr_ll_set_em_ds(SysCtrlRegs_Type *sysctl)
{
  SET_BIT(sysctl->RAM_CTRL, SYS_CTRL_RAM_CTRL_EM_NRM_DS_EN_MASK);
}

/**
* @brief   pwr lp keep config
* @param  SysCtrlRegs_Type *sysctl: sysctl Instance
* @retval None
*/
__STATIC_FORCEINLINE void ms_pwr_ll_set_retem_ds(SysCtrlRegs_Type *sysctl)
{
  SET_BIT(sysctl->RAM_CTRL, SYS_CTRL_RAM_CTRL_EM_RET_DS_EN_MASK);
}

/**
* @brief   pwr get pre mode
* @param  SysCtrlRegs_Type *sysctl: sysctl Instance
* @retval None
*/
__STATIC_FORCEINLINE uint32_t ms_pwr_ll_ger_pre_mode(SysCtrlRegs_Type *sysctl)
{
  return READ_REG(sysctl->PRE_PWR_MODE);
}
#ifdef __cplusplus
}
#endif

#endif /* MS_PWR_LL_H_ */
