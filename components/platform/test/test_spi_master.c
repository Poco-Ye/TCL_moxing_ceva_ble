/*
 * test_spi_master.c
 *
 *  Created on: 2021年12月16日
 *      Author:haijun.mai
 */

#include "spi_master.h"
#include "uart.h"
#include <string.h>
#include "ms_uart.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"


TEST_CASE("spi","test_spi_master read spi flash id", "[Driver/spi_master]")
{
    uint8_t cmd = 0x9f;
	
    uint8_t rx_data[4] = {0};



    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_cs_set_high());
     MS_LOGI(MS_DRIVER, "\r\nread spi flash id :\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_cs_set_low());

    /*send read id cmd*/
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_send(&cmd ,1, 0));

    /*receive spi flassh id*/
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  spi_master_receive(rx_data, 3, 0));
    
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_cs_set_high());
   
    MS_LOGI(MS_DRIVER, "\r\n man_id = %x\n",rx_data[0]);
    MS_LOGI(MS_DRIVER, "\r\n mem_type = %x\n",rx_data[1]);
    MS_LOGI(MS_DRIVER, "\r\n mem_capacity = %x\n",rx_data[2]);
   
	 

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_deinit());

}


TEST_CASE("spi","test_spi_master read spi flash unique id", "[Driver/spi_master]")
{
    uint8_t cmd = 0x4b;

    uint8_t address[4] = {0};

    uint8_t dummy_byte = 0x0;
	
    uint8_t rx_data[4] = {0};



    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_cs_set_high());
     MS_LOGI(MS_DRIVER, "\r\nread spi flash unique  id :\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_cs_set_low());

    /*send read unique id cmd*/
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_send(&cmd ,1, 0));

   /*send address */
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_send(address ,3, 0));

     /*send dummy_byte*/
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_send(&dummy_byte ,1, 0));

    /*receive spi flassh id*/
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  spi_master_receive(rx_data, 4, 0));
    
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_cs_set_high());
   
    MS_LOGI(MS_DRIVER, "\r\n unique id 0 = %x\n",rx_data[0]);
    MS_LOGI(MS_DRIVER, "\r\n unique id 1 = %x\n",rx_data[1]);
    MS_LOGI(MS_DRIVER, "\r\n unique id 2 = %x\n",rx_data[2]);
    MS_LOGI(MS_DRIVER, "\r\n unique id 3 = %x\n",rx_data[3]);
   
	 

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_deinit());

}


TEST_CASE("spi","test_spi_master interrupt read spi flash id", "[Driver/spi_master]")
{
    uint8_t cmd = 0x9f;
	
    uint8_t rx_data[4] = {0};



    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_cs_set_high());
     MS_LOGI(MS_DRIVER, "\r\nread spi flash id :\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_cs_set_low());

    /*send read id cmd*/
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_interrupt_send(&cmd ,1, 0));

    /*receive spi flassh id*/
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  spi_master_interrupt_receive(rx_data, 3, 0));
    
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_cs_set_high());
   
    MS_LOGI(MS_DRIVER, "\r\n man_id = %x\n",rx_data[0]);
    MS_LOGI(MS_DRIVER, "\r\n mem_type = %x\n",rx_data[1]);
    MS_LOGI(MS_DRIVER, "\r\n mem_capacity = %x\n",rx_data[2]);
   
	 

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_deinit());

}


TEST_CASE("spi","test_spi_master interrupt read spi flash unique id", "[Driver/spi_master]")
{
    uint8_t cmd = 0x4b;

    uint8_t address[4] = {0};

    uint8_t dummy_byte = 0x0;
	
    uint8_t rx_data[4] = {0};



    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_cs_set_high());
     MS_LOGI(MS_DRIVER, "\r\nread spi flash unique id :\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_cs_set_low());

    /*send read unique id cmd*/
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_interrupt_send(&cmd ,1, 0));
    

   /*send address */
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_interrupt_send(address ,3, 0));
   
     /*send dummy_byte*/
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_interrupt_send(&dummy_byte ,1, 0));
    
    /*receive spi flassh id*/
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  spi_master_interrupt_receive(rx_data, 4, 0));
    
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_cs_set_high());
   
    MS_LOGI(MS_DRIVER, "\r\n unique id 0 = %x\n",rx_data[0]);
    MS_LOGI(MS_DRIVER, "\r\n unique id 1 = %x\n",rx_data[1]);
    MS_LOGI(MS_DRIVER, "\r\n unique id 2 = %x\n",rx_data[2]);
    MS_LOGI(MS_DRIVER, "\r\n unique id 3 = %x\n",rx_data[3]);
   
	 

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_master_deinit());

}

