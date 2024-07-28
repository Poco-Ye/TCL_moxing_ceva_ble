/*
 * test_ms_flash.c
 *
 *  Created on: 2021年12月16日
 *      Author: che.jiang
 */

#include <string.h>
#include "pwr.h"
#include "uart.h"
#include <string.h>
#include "ms_uart.h"
#include "ms_clock_hal.h"
#include "ms_pinmux_hal.h"
#include "unity.h"
#include "uart.h"
#include "flash.h"
#include "unity_test_runner.h"
#include "log.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"

extern char _sfunc[];
extern char _efunc[];
extern char _sifunc[];
extern char _sfunc_retram[];
extern char _efunc_retram[];
extern char _sifunc_retram[];
extern void wait_nop(unsigned int n);

FLASH_Handle_Type flash_handle;
PWR_Config_Type pwr_config;

static void test_copy_func(void)
{
    char *addr_sram, *addr_flash;

    MS_LOGI( MS_DRIVER, "[PWR]:test_copy_func\r\n" );
    /*critical retram func segment*/
    addr_sram =_sfunc;
    addr_flash = _sifunc;
    while(addr_sram < _efunc)
    {
        *(uint32_t*)addr_sram = *(uint32_t*)addr_flash;
        wait_nop(2000);
        addr_sram += 4;
        addr_flash += 4;

    }
}

void test_copy_retention_func(void)
{
    char *addr_sram, *addr_flash;

    MS_LOGI( MS_DRIVER, "[PWR]:test_copy_func_retention\r\n" );
    /*critical retram func segment*/
    addr_sram =_sfunc_retram;
    addr_flash = _sifunc_retram;
    while(addr_sram < _efunc_retram)
    {
        *(uint32_t*)addr_sram = *(uint32_t*)addr_flash;
        wait_nop(2000);
        addr_sram += 4;
        addr_flash += 4;

    }
}

int32_t test_pwr_user_before_sleep_callback()
{
    MS_LOGI( MS_DRIVER, "[PWR]:test_pwr_user_before_sleep_callback\r\n" );
    return STATUS_SUCCESS;
}

int32_t test_pwr_user_after_wakeup_callback()
{
    MS_LOGI( MS_DRIVER, "[PWR]:test_pwr_user_after_wakeup_callback\r\n" );
    return STATUS_SUCCESS;
}

int32_t test_pwr_gpio_before_sleep_callback(uint8_t pin, uint8_t dir, uint8_t level)
{
    MS_LOGI( MS_DRIVER, "[PWR]:test_pwr_gpio_before_sleep_callback\r\n" );
    MS_LOGI( MS_DRIVER, "[PWR]:pin=%d,dir=%d,level=%d\r\n", pin, dir, level);
    return gpio_enter_deep_sleep_callback(pin, dir, level);
}

int32_t test_pwr_flash_before_sleep_callback(uint32_t addr, uint32_t *size)
{
    MS_LOGI( MS_DRIVER, "[PWR]:test_pwr_flash_before_sleep_callback\r\n" );

    return flash_enter_deepsleep_mode( addr,size);
}

void test_pwr_register_sleep_callback()
{
    pwr_register_gpio_sleep_cb(test_pwr_gpio_before_sleep_callback);
    pwr_register_flash_sleep_cb(test_pwr_flash_before_sleep_callback);
    pwr_register_user_sleep_cb(test_pwr_user_before_sleep_callback);
    pwr_register_user_wakeup_cb(test_pwr_user_after_wakeup_callback);
}

TEST_CASE("pwr","test_pwr_deepsleep", "[Driver/power]")
//void test_ms_pwr(void)
{
    MS_LOGI( MS_DRIVER, "[PWR]:test_pwr_deepsleep\r\n" );
    test_copy_func();
    test_copy_retention_func();

    MS_LOGI( MS_DRIVER, "[PWR]:flash_init\r\n" );
    flash_handle.flash_type = FLASH_QSPI_TYPE;
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, flash_init(&flash_handle));

    MS_LOGI( MS_DRIVER, "[PWR]:pwr_init\r\n" );

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwr_init());

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwr_config_sleep_mode(PWR_MODE_DEEPSLEEP));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwr_config_wkup_sel(PWR_WAKEUP_SEL_GPIO_PORT));

    __disable_irq();

    MS_LOGI( MS_DRIVER, "[PWR]:pwr_enter_sleep %x\r\n",pwr_get_pre_mode());
    if(PWR_MODE_POWERDOWN == pwr_get_pre_mode())
    {
        test_pwr_register_sleep_callback();
        TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwr_enter_sleep());
    }
    else
    {
        TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwr_after_wakeup());

        test_pwr_register_sleep_callback();
        TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, pwr_enter_sleep());
    }
    //vTaskDelay(20000);
    __enable_irq();

}


TEST_CASE("pwr","test_pwr_lightsleep", "[Driver/power]")
{
    MS_LOGI( MS_DRIVER, "[PWR]:test_pwr_lightsleep\r\n" );


    MS_LOGI( MS_DRIVER, "[PWR]:pwr_get_pre_mode %x\r\n",pwr_get_pre_mode());

    __WFI();

    MS_LOGI( MS_DRIVER, "[PWR]:test_pwr_lightsleep over\r\n");

}


/*
int main(void)
{
    system_clock_init ();
    ms_pinmux_hal_config_default();

    uart0_int();
    UNITY_BEGIN();

    RUN_TEST(test_ms_pwr);

    while(1);
    return 0;
}
*/




