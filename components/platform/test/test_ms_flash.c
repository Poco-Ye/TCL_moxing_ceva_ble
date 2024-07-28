/*
 * test_ms_flash.c
 *
 *  Created on: 2021年12月16日
 *      Author: che.jiang
 */

#include "flash.h"
#include "uart.h"
#include <string.h>
#include "ms_uart.h"
#include "ms_clock_hal.h"
#include "ms_pinmux_hal.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"

#define TEST_FLASH_ADDR              (0x20030000U)
#define TEST_FLASH_END_ADDR          FLASH_BASE_ADDR_END
#define TEST_FLASH_WRITE_SIZE        (0x00000100U)

char test_write_buf[TEST_FLASH_WRITE_SIZE] =
    "hellhellhellhellhellhellhellhellhellhellhellhellhellhell\n";
char test_read_buf[TEST_FLASH_WRITE_SIZE] = {0};


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
TEST_CASE("flash","test_ms_flash_read_write", "[Driver/flash]")
{
    uint32_t test_flash_addr = 0;
    MS_LOGI( MS_DRIVER, "[FLASH]:hello flash test\r\n" );

    test_copy_func();

    flash_handle.flash_type = FLASH_QSPI_TYPE;

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, flash_init(&flash_handle));
    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, flash_write_protected(&flash_handle, 0));

    for(uint32_t i = 0; i < 48; i++)
    {
        test_flash_addr = TEST_FLASH_ADDR + i * 4096;
        TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, flash_erase(&flash_handle, test_flash_addr, 4096));

        TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, flash_program(&flash_handle, test_flash_addr, TEST_FLASH_WRITE_SIZE, test_write_buf));
        wait_nop(2000);

        TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, flash_read(&flash_handle, test_flash_addr, TEST_FLASH_WRITE_SIZE, test_read_buf));

        wait_nop(2000);

        MS_LOGI( MS_DRIVER, "[FLASH]:test_flash_addr =0x%x, test count=%u\r\n",test_flash_addr, i);
        MS_LOGI( MS_DRIVER, "[FLASH]:write:%s\r\n",test_write_buf );
        MS_LOGI( MS_DRIVER, "[FLASH]:read:%s\r\n",test_read_buf );
       
        TEST_ASSERT_EQUAL_INT8_ARRAY(test_write_buf, test_read_buf, strlen(test_write_buf));
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, flash_write_protected(&flash_handle, 1));
}


TEST_CASE("flash","test_ms_flash_stig_read_write", "[Driver/flash]")
{
    uint32_t i = 0;
    uint32_t test_flash_addr = 0;
    MS_LOGI( MS_DRIVER, "[FLASH]:hello flash test\r\n" );

    test_copy_func();

    flash_handle.flash_type = FLASH_QSPI_TYPE;

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, flash_init(&flash_handle));

    test_flash_addr = TEST_FLASH_ADDR;

    for(i = 0; i < 64; i++)
    {
        MS_LOGI( MS_DRIVER, "[FLASH]:test_flash_addr =0x%x\r\n",test_flash_addr);

        TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, flash_erase(&flash_handle, test_flash_addr, 4096));
        MS_LOGI( MS_DRIVER, "[FLASH]:flash_erase\r\n");
        
        TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, flash_stig_write(&flash_handle, test_flash_addr, TEST_FLASH_WRITE_SIZE, test_write_buf));
        MS_LOGI( MS_DRIVER, "[FLASH]:flash_stig_write %s\r\n",test_write_buf );

        TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, flash_stig_read(&flash_handle, test_flash_addr, TEST_FLASH_WRITE_SIZE, test_read_buf));
        MS_LOGI( MS_DRIVER, "[FLASH]:flash_stig_read %s\r\n",test_read_buf );
        
        TEST_ASSERT_EQUAL_INT8_ARRAY(test_write_buf, test_read_buf, strlen(test_write_buf));
    }

}


TEST_CASE("flash","test_ms_flash_direct_write_stig_read", "[Driver/flash]")
{
    uint32_t i = 0, j = 0, k = 0;
    uint32_t test_flash_addr = 0;
    MS_LOGI( MS_DRIVER, "[FLASH]:hello flash test\r\n" );

    test_copy_func();

    flash_handle.flash_type = FLASH_QSPI_TYPE;

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, flash_init(&flash_handle));

    while(1)
    {
        for(i = 0; i < 64; i++)
        {
            test_flash_addr = TEST_FLASH_ADDR + i * 4096;
            TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, flash_erase(&flash_handle, test_flash_addr, 4096));


            MS_LOGI( MS_DRIVER, "[FLASH]:test_flash_addr =0x%x, test count=%u\r\n",test_flash_addr, i);
            for(j = 0; j < 16; j++)
            {
                TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, flash_program(&flash_handle, test_flash_addr+j*256, TEST_FLASH_WRITE_SIZE, test_write_buf));
                TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, flash_stig_read(&flash_handle, test_flash_addr+j*256, TEST_FLASH_WRITE_SIZE, test_read_buf));
                for(k = 0; k < 16; k++)
                {
                    MS_LOGI( MS_DRIVER, "[FLASH]:k=%d(%x),write:%x,stig read=%x\r\n", k,test_flash_addr+j*256+k*4,
                        *(uint32_t *)&test_write_buf[k*4], *(uint32_t *)&test_read_buf[k*4]);
                }
                wait_nop(2000);
            }

            if(test_flash_addr >= (TEST_FLASH_END_ADDR - 4096))
            {
                break;
            }
        }
    }

}

