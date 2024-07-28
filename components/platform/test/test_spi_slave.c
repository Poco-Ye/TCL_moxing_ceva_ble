/*
 * test_spi_slave.c
 *
 *  Created on: 2021年12月16日
 *      Author:haijun.mai
 */

#include "spi_slave.h"
#include "uart.h"
#include <string.h>
#include "ms_uart.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"

TEST_CASE("spi","test_spi_slave  write test", "[Driver/spi_slave]")
{

     uint8_t tx_data[9] = {1,2,3,4,5,6,7,8,9};
	

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_slave_init());

     MS_LOGI(MS_DRIVER, "\r\nspi slave send data :\n");

    
    /*send tx data*/
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_slave_send(tx_data ,8, 0)); 

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_slave_deinit());

}


TEST_CASE("spi","test_spi_slave read  test", "[Driver/spi_slave]")
{


	
    uint8_t rx_data[8] = {0};
    uint8_t i = 0;



    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_slave_init());

     MS_LOGI(MS_DRIVER, "\r\nspi slave receive data :\n");

     /*receive spi flassh id*/
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  spi_slave_receive(rx_data, 8, 0));

     for(i=0;i<8;i++)
     {
            MS_LOGI(MS_DRIVER, "\r\nrx_data[%d] = %x\n",i,rx_data[i]);
     }

	 

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_slave_deinit());

}



TEST_CASE("spi","test_spi_slave  interrupt write test", "[Driver/spi_slave]")
{

     uint8_t tx_data[17] = {0x0,0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8,0x9,0xa,0xb,0xc,0xd,0xe,0xf};
	

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_slave_init());

     MS_LOGI(MS_DRIVER, "\r\nspi slave send data :\n");

    
    /*send tx data*/
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_slave_interrupt_send(tx_data ,16, 0)); 

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_slave_deinit());

}


TEST_CASE("spi","test_spi_slave interrupt read  test", "[Driver/spi_slave]")
{


	
    uint8_t rx_data[17] = {0};
    uint8_t i = 0;



    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_slave_init());

     MS_LOGI(MS_DRIVER, "\r\nspi slave receive data :\n");

     /*receive spi flassh id*/
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  spi_slave_interrupt_receive(rx_data, 16, 0));

     for(i=0;i<16;i++)
     {
            MS_LOGI(MS_DRIVER, "\r\nrx_data[%d] = %x\n",i,rx_data[i]);
     }

	 

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, spi_slave_deinit());

}