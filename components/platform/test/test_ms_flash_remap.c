/*
 * test_ms_flash_remap.c
 *
 *  Created on: 2021年12月16日
 *      Author: che.jiang
 */


#include "uart.h"
#include <string.h>
#include "ms_uart.h"
#include "ms_flash_remap.h"
#include "flash.h"
#include "ms_clock_hal.h"
#include "ms_pinmux_hal.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "log.h"


#define FLASH_TEST_SRC_ADDR         0x20040000
#define FLASH_TEST_DST_ADDR         0x20060000
#define FLASH_TEST_SIZE             0x1000

static char test_write_buf[FLASH_TEST_SIZE] = "hello Mooresilicon!!!!!\r\n";
static char test_read_buf_remap[FLASH_TEST_SIZE] = {0};
static char test_read_buf_normal[FLASH_TEST_SIZE] = {0};


extern char _sfunc[];
extern char _efunc[];
extern char _sifunc[];
extern void wait_nop(unsigned int n);


static void test_copy_func(void)
{
    char *addr_sram, *addr_flash;

    MS_LOGI( MS_DRIVER, "[FLASH]:test_copy_func\r\n" );
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

FLASH_Handle_Type flash_handle;
TEST_CASE("flash","test_ms_flash_remap", "[Driver/flash_remap]")
{
    int32_t i = 0;
    int32_t ret_val = 0;
    uint32_t read_addr = 0;
    uint8_t *read_buf = NULL;
    MS_LOGI( MS_DRIVER, "[FLASH]:hello flash remap test\r\n" );

    test_copy_func();

    flash_handle.flash_type = FLASH_QSPI_TYPE;

    ret_val = flash_init(&flash_handle);
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ret_val);


    ret_val = flash_erase(&flash_handle, FLASH_TEST_DST_ADDR, 4096);
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ret_val);
    
    ret_val =  flash_program(&flash_handle, FLASH_TEST_DST_ADDR, 
        FLASH_TEST_SIZE, test_write_buf);
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ret_val);
    wait_nop(2000);

    ms_flash_remap_set_addr(FLASH_TEST_SRC_ADDR, 
        FLASH_TEST_DST_ADDR, FLASH_TEST_SIZE);
    ms_flash_remap_enable();

    read_addr = FLASH_TEST_SRC_ADDR;
    read_buf = test_read_buf_remap;
    for(i = 0; i < 32; i+=4)
    {
        *(uint32_t *)read_buf = *(uint32_t *)read_addr;
        read_buf += 4;
        read_addr += 4;
    }

    wait_nop(2000);

    ms_flash_remap_disable();

    read_addr = FLASH_TEST_SRC_ADDR;
    read_buf = test_read_buf_normal;
    for(i = 0; i < 32; i+=4)
    {
        *(uint32_t *)read_buf = *(uint32_t *)read_addr;
        read_buf += 4;
        read_addr += 4;
    }

    wait_nop(2000);

    MS_LOGI( MS_DRIVER, "[FLASH]:flash_src =0x%x,flash_dest =0x%x\r\n",
        FLASH_TEST_SRC_ADDR,FLASH_TEST_DST_ADDR);
    MS_LOGI( MS_DRIVER, "[FLASH]:write:%s\r\n",test_write_buf );
    MS_LOGI( MS_DRIVER, "[FLASH]:read remap:%s\r\n",test_read_buf_remap);
    MS_LOGI( MS_DRIVER, "[FLASH]:read normal:%s\r\n",test_read_buf_normal);   
    TEST_ASSERT_EQUAL_INT8_ARRAY(test_write_buf, test_read_buf_remap, strlen(test_write_buf));
}

