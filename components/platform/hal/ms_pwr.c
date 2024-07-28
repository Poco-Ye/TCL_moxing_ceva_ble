/**
  * Copyright © 2021 by MooreSilicon. All rights reserved
  * @file  ms_pwr.c
  * @brief power device driver
  * @author che.jiang
  * @date 2022年1月14日
  * @version 1.0
  * @Revision:
  */
#include <string.h>
#include <ms_interrupt.h>
#include <ms_clock_hal.h>
#include <ms_pinmux_hal.h>
#include "ms_pwr.h"
#include "ms_gpio.h"
#include "uart.h"
#include "log.h"

#define RETENTION_FLASH_DATA (RET_RAM_BASE_ADDR)
RETENTION_RAM_DATA_SECTION PWR_Config_Type g_pwr_cfg = {0};

/**
 * @brief This function initial pwr module  .
 * @param  PWRHandle_Type *hpwr : Pointer to  pwr module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_pwr_init(PWRHandle_Type *hpwr)
{
    PWR_Config_Type *p_pwr_cfg = (PWR_Config_Type *)&g_pwr_cfg;

    memcpy((int8_t*)p_pwr_cfg, (int8_t*)&hpwr->pwr_config, sizeof(PWR_Config_Type));

    return STATUS_SUCCESS;
}


/**
 * @brief This function deinitial pwr module  .
 * @param  PWRHandle_Type *hpwr : Pointer to  pwr module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_pwr_deinit(PWRHandle_Type *hpwr)
{
    return STATUS_SUCCESS;
}

/**
 * @brief This function do analog module enter low pwr mode   .
 * @param  none
 * @retval none
 */
void  ms_pwr_analog_enter()
{
    return;
}

/**
 * @brief This function do set pad module enter low pwr mode   .
 * @param  none
 * @retval none
 */
int32_t ms_pwr_pad_enter(void)
{
    PadNum_Type  pad;
    PadPull_Type pull_type;
    uint8_t gpio_dir = 0;
    uint8_t gpio_level = 0;
    PWR_Config_Type *p_pwr_cfg = (PWR_Config_Type *)&g_pwr_cfg;

    //config all pad pull type
    for(pad = PAD0; pad < PADMAX; pad++)
    {
        if(pad < PAD16)
        {
            pull_type = (p_pwr_cfg->pull_type_p00_p15>>(pad*2))&0x3;
        }
        else
        {
            pull_type = (p_pwr_cfg->pull_type_p16_p26>>((pad-PAD16)*2))&0x3;
        }
        if(pull_type != PULL_UP && pull_type != PULL_DOWN)
        {
            continue;
        }
        gpio_dir = (pull_type == PULL_DOWN)?GPIO_MODE_OUT_PUT:GPIO_MODE_IN_PUT;
        gpio_level = (pull_type == PULL_DOWN)?0:1;
        ms_pinmux_hal_set_pinmux(pad, 0);
        ms_pinmux_hal_config_pull_type(pad, pull_type);

        if(NULL == p_pwr_cfg->pwr_gpio_before_sleep_callback)
        {
            return STATUS_ERROR;
        }
        if(STATUS_SUCCESS != p_pwr_cfg->pwr_gpio_before_sleep_callback(pad, gpio_dir, gpio_level))
        {
            return STATUS_ERROR;
        }
    }

    if(PWR_MODE_DEEPSLEEP == p_pwr_cfg->pwr_mode ||
       PWR_MODE_SLEEP == p_pwr_cfg->pwr_mode)
    {
        if(p_pwr_cfg->ds_wkup_sel&PWR_WAKEUP_SEL_LP_UART)
        {
            ms_pinmux_hal_set_pinmux(PAD10, PIN10_UART2_TXD);
            ms_pinmux_hal_set_pinmux(PAD11, PIN11_UART2_RXD);
        }
    }


    return STATUS_SUCCESS;
}


void ms_pwr_disable_cpu_interrupt(PWRHandle_Type *hpwr)
{
    INTERRUPT_DISABLE_IRQ(WAKEUP_IRQn);
    INTERRUPT_DISABLE_IRQ(SysTimer_IRQn);
    INTERRUPT_DISABLE_IRQ(GPIO_IRQn);
}


/**
 * @brief This function set pwr sysctl config   .
 * @param  PWRHandle_Type *hpwr : Pointer to  pwr module.
 * @retval none
 */
void  ms_pwr_sysctl_config(PWRHandle_Type *hpwr)
{
    PWR_Config_Type *p_pwr_cfg = (PWR_Config_Type *)&g_pwr_cfg;
    if(PWR_MODE_SLEEP < p_pwr_cfg->pwr_mode)
    {
        return;
    }

    ms_pwr_hal_enable_gpio_wakup(hpwr->sysctl_instance, true); 
    ms_pwr_hal_enable_ble_wakup(hpwr->sysctl_instance, true); 
    ms_pwr_hal_enable_rtc_wakup(hpwr->sysctl_instance,true); 

    ms_pwr_hal_clear_wk_csr(hpwr->sysctl_instance);
    ms_pwr_hal_enable_wk_csr_int(hpwr->sysctl_instance);
    ms_pwr_hal_set_lp_rst_mask(hpwr->sysctl_instance, 1);
    if(p_pwr_cfg->ds_wkup_sel&PWR_WAKEUP_SEL_BLE_TIMER)
    {
        ms_pwr_hal_set_lp_ble_mask(hpwr->sysctl_instance, false);
    }
    else
    {
        ms_pwr_hal_set_lp_ble_mask(hpwr->sysctl_instance, true);
    }
    
    ms_pwr_disable_cpu_interrupt(hpwr);
    ms_clock_hal_uart2_fclk_sel(UART2_CLK_SEL_RC_CLK);
}

/**
 * @brief This function do ram/flash/clk enter pwr mode. d
 *        this function run in retention ram
 * @param  PWRHandle_Type *hpwr : Pointer to  pwr module.
 * @retval none
 */
RETENTION_RAM_FUNCTION void  ms_pwr_sleep_enter(PWRHandle_Type *hpwr)
{
    uint32_t size = 0;
    PWR_Config_Type *p_pwr_cfg = (PWR_Config_Type *)&g_pwr_cfg;
    if(PWR_MODE_SLEEP < p_pwr_cfg->pwr_mode)
    {
        return;
    }
	
    /*do run qspi flash enter pwr mode*/
    if (p_pwr_cfg->pwr_flash_before_sleep_callback)
    {
        p_pwr_cfg->pwr_flash_before_sleep_callback(RETENTION_FLASH_DATA, &size);
    }

    /*do pwr module and sram enter pwr mode*/
    switch(p_pwr_cfg->pwr_mode)
    {
        case PWR_MODE_POWERDOWN:
            ms_pwr_hal_enable_pwr_pd(hpwr->sysctl_instance);
            ms_pwr_hal_set_sram0_pd(hpwr->sysctl_instance);
            ms_pwr_hal_set_sram1_pd(hpwr->sysctl_instance);
            ms_pwr_hal_set_sram2_pd(hpwr->sysctl_instance);
            ms_pwr_hal_set_em_pd(hpwr->sysctl_instance);
            ms_pwr_hal_set_retem_pd(hpwr->sysctl_instance);
            break;

        case PWR_MODE_DEEPSLEEP:
            ms_pwr_hal_enable_pwr_ds(hpwr->sysctl_instance);
            ms_pwr_hal_set_sram0_pd(hpwr->sysctl_instance);
            ms_pwr_hal_set_sram1_pd(hpwr->sysctl_instance);
            ms_pwr_hal_set_sram2_pd(hpwr->sysctl_instance);
            ms_pwr_hal_set_em_pd(hpwr->sysctl_instance);
            ms_pwr_hal_set_retem_ds(hpwr->sysctl_instance);
            break;

        case PWR_MODE_SLEEP:
            ms_pwr_hal_enable_pwr_slp(hpwr->sysctl_instance);
            ms_pwr_hal_set_sram0_ds(hpwr->sysctl_instance);
            ms_pwr_hal_set_sram1_ds(hpwr->sysctl_instance);
            ms_pwr_hal_set_sram2_ds(hpwr->sysctl_instance);
            ms_pwr_hal_set_em_ds(hpwr->sysctl_instance);
            ms_pwr_hal_set_retem_ds(hpwr->sysctl_instance);
            break;

        default:
            return ;
    }

    ms_clock_hal_lf_clk_sel(LF_CLK_SEL_RC_OSC32K);

    __asm__("li a0,0x00000F00");
    __asm__("jr a0");
}


/**
 * @brief This function run pwr enter ds/slp/pd mode   .
 * @param  PWRHandle_Type *hpwr : Pointer to  pwr module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_pwr_enter(PWRHandle_Type *hpwr)
{
    int32_t status = STATUS_SUCCESS;
    PWR_Config_Type *p_pwr_cfg = (PWR_Config_Type *)&g_pwr_cfg;

    ms_pwr_analog_enter();

    if (p_pwr_cfg->pwr_before_sleep_callback)
    {
        p_pwr_cfg->pwr_before_sleep_callback();
    }

    status = ms_pwr_pad_enter();
    if(STATUS_SUCCESS != status)
    {
        return STATUS_ERROR;
    }

    ms_pwr_sysctl_config(hpwr);

    ms_pwr_sleep_enter(hpwr);

    return STATUS_SUCCESS;

}

/**
 * @brief This function run pwr exit from ds/slp/pd mode   .
 * @param  PWRHandle_Type *hpwr : Pointer to  pwr module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_pwr_exit(PWRHandle_Type *hpwr)
{
    PWR_Config_Type *p_pwr_cfg = (PWR_Config_Type *)&g_pwr_cfg;

    if (p_pwr_cfg->pwr_after_wakeup_callback)
    {
        p_pwr_cfg->pwr_after_wakeup_callback();
    }

    return STATUS_SUCCESS;
}

int32_t ms_pwr_config_pad_pulltype(uint32_t pin, PadPull_Type pull_type)
{
    PWR_Config_Type *p_pwr_cfg = (PWR_Config_Type *)&g_pwr_cfg;

    if(pin < PAD16)
    {
        p_pwr_cfg->pull_type_p00_p15 &= ~(0x3<<(pin*2));  //clear pad pulltype
        p_pwr_cfg->pull_type_p00_p15 |= (pull_type<<(pin*2));
    }
    else
    {
        p_pwr_cfg->pull_type_p16_p26 &= ~(0x3<<((pin-PAD16)*2));  //clear pad pulltype
        p_pwr_cfg->pull_type_p16_p26 |= (pull_type<<((pin-PAD16)*2));
    }
    return STATUS_SUCCESS;
}

int32_t  ms_pwr_config_sleep_mode(PWR_MODE_Type pwr_mode)
{
    PWR_Config_Type *p_pwr_cfg = (PWR_Config_Type *)&g_pwr_cfg;
    p_pwr_cfg->pwr_mode = pwr_mode;

    return STATUS_SUCCESS;
}

int32_t  ms_pwr_config_wkup_sel(PWR_WAKEUP_SEL_Type wakeup_sel)
{
    PWR_Config_Type *p_pwr_cfg = (PWR_Config_Type *)&g_pwr_cfg;
    if(p_pwr_cfg->pwr_mode == PWR_MODE_SLEEP)
    {
        p_pwr_cfg->slp_wkup_sel = wakeup_sel;
    }
    else if(p_pwr_cfg->pwr_mode == PWR_MODE_DEEPSLEEP)
    {
        p_pwr_cfg->ds_wkup_sel = wakeup_sel;
    }

    return STATUS_SUCCESS;
}

int32_t ms_pwr_get_pre_mode(PWRHandle_Type *hpwr)
{
    int32_t sleep_mode = PWR_MODE_ACTIVE;
    uint32_t pre_pwr_mode = PWR_MODE_ACTIVE;

    pre_pwr_mode = ms_pwr_hal_ger_pre_mode(hpwr->sysctl_instance);

    switch(pre_pwr_mode)
    {
        case PRE_PWR_POWERDOWN:
            sleep_mode = PWR_MODE_POWERDOWN;
            break;
        case PRE_PWR_DEEPSLEEP:
            sleep_mode = PWR_MODE_DEEPSLEEP;
            break;
        case PRE_PWR_SLEEP:
            sleep_mode = PWR_MODE_SLEEP;
            break;

        case PRE_PWR_ACTIVE:
        default:
            sleep_mode = PWR_MODE_ACTIVE;
            break;
    }

	MS_LOGI(MS_DRIVER, "sleep mode %d\r\n", sleep_mode);
    return sleep_mode;
}

void ms_pwr_register_user_sleep_callback(int32_t (*sleep_cb)())
{
    PWR_Config_Type *p_pwr_cfg = (PWR_Config_Type *)&g_pwr_cfg;
    p_pwr_cfg->pwr_before_sleep_callback = sleep_cb;
}

void ms_pwr_register_user_wakeup_callback(int32_t (*wakeup_cb)())
{
    PWR_Config_Type *p_pwr_cfg = (PWR_Config_Type *)&g_pwr_cfg;
    p_pwr_cfg->pwr_after_wakeup_callback = wakeup_cb;
}

void ms_pwr_register_gpio_sleep_callback(int32_t (*sleep_cb)(uint8_t pin, uint8_t dir, uint8_t level))
{
    PWR_Config_Type *p_pwr_cfg = (PWR_Config_Type *)&g_pwr_cfg;
    p_pwr_cfg->pwr_gpio_before_sleep_callback = sleep_cb;
}

void ms_pwr_register_flash_sleep_callback(int32_t (*sleep_cb)(uint32_t addr, uint32_t *size))
{
    PWR_Config_Type *p_pwr_cfg = (PWR_Config_Type *)&g_pwr_cfg;
    p_pwr_cfg->pwr_flash_before_sleep_callback = sleep_cb;
}

void ms_pwr_wakeup_irq_handler(PWRHandle_Type *hpwr)
{
    ms_pwr_hal_disable_wk_csr_int(hpwr->sysctl_instance);		

    uint32_t wakeup_state  =  ms_pwr_hal_get_wk_csr_state(hpwr->sysctl_instance);
				
    if(PWR_WAKEUP_IOMAX_STATE&wakeup_state)
    {
        ms_pwr_hal_enable_gpio_wakup(hpwr->sysctl_instance, false);
    }

    if(PWR_WAKEUP_RTC_STATE&wakeup_state)
    {
        ms_pwr_hal_enable_rtc_wakup(hpwr->sysctl_instance, false);	
    }

    if(PWR_WAKEUP_UART2_STATE&wakeup_state)
    {
    }
    
    if(PWR_WAKEUP_BLE_STATE&wakeup_state)
    {
        ms_pwr_hal_enable_ble_wakup(hpwr->sysctl_instance, false);
    }    
}

