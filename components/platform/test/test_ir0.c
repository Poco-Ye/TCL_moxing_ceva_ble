/*
 * test_ir.c
 *
 *  Created on: 2021年12月16日
 *      Author:haijun.mai
 */

#include "ir0.h"
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



TEST_CASE("ir","test nec ir0 send up data", "[Driver/ir0 0]")
{
    MS_LOGI(MS_DRIVER, "\r\n nec send up data start\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_nec_init());
     MS_LOGI(MS_DRIVER, "\r\n nec ini finisht\n");
   
     for(uint32_t i = 0;i<3;i++)
    {

        //up 
	TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_nec_send_data(0x2fd09966));

       MS_LOGI(MS_DRIVER, "\r\n nec send up data end\n");
      // while(1);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_deinit());
  
}

TEST_CASE("ir","test nec ir0 send down data", "[Driver/ir0 1]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_nec_init());
    for(uint32_t i = 0;i<8000;i++)
    {
     
        //up 
	TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_nec_send_data(0x2ed19966));
        //down
	
    
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_deinit());
  
}

TEST_CASE("ir","test nec ir0 int send up data", "[Driver/ir0 2]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_nec_init());
    for(uint32_t i = 0;i<3;i++)
    {

        //up 
	TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_nec_int_send_data(0x2fd09966));

         vTaskDelay(500); 
    
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_deinit());
  
}

TEST_CASE("ir","test nec ir0 int send down data", "[Driver/ir0 3]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_nec_init());
    for(uint32_t i = 0;i<3;i++)
    {
     
        //up 
	TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_nec_int_send_data(0x2ed19966));
        //down
	   vTaskDelay(500); 
    
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_deinit());
  
}

TEST_CASE("ir","test nec ir0 repeat send up data", "[Driver/ir0 4]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_nec_init());
    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_nec_send_data_repeat(0x2fd09966));	
    
    for(uint32_t i = 0;i<800;i++)
    {
        MS_LOGI(MS_DRIVER,"nec repeat send data cnt =  %x\n",i);
        vTaskDelay(50); 
    }
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  ir0_nec_send_data_repeat_stop());
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_deinit());
  
}


TEST_CASE("ir","test nec ir0 repeat send down data", "[Driver/ir0 5]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_nec_init());
    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_nec_send_data_repeat(0x2ed19966));	
    
    for(uint32_t i = 0;i<800;i++)
    {
        MS_LOGI(MS_DRIVER,"nec repeat send data cnt =  %x\n",i);
        vTaskDelay(50); 
    }
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  ir0_nec_send_data_repeat_stop());
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_deinit());
  
}



TEST_CASE("ir","test rca ir0 send up data", "[Driver/ir0 6]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_rca_init());
    for(uint32_t i = 0;i<8000;i++)
    {
        TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_rca_send_data(0x0b0f4f&0xffffff));
        //MS_LOGI(MS_DRIVER,"rca send data cnt =  %x\n",i);
    
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_deinit());
  
}


TEST_CASE("ir","test rca ir0 send down data", "[Driver/ir0 7]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_rca_init());
    for(uint32_t i = 0;i<8000;i++)
    {
        TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_rca_send_data(0x8b074f&0xffffff));
        //MS_LOGI(MS_DRIVER,"rca send data cnt =  %x\n",i);
    
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_deinit());
  
}


TEST_CASE("ir","test rca ir0 repeat send up data", "[Driver/ir0 8]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_rca_init());

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_rca_send_data_repeat(0x0b0f4f&0xffffff));

    for(uint32_t i = 0;i<800;i++)
    {
        MS_LOGI(MS_DRIVER,"nec repeat send data cnt =  %x\n",i);
        vTaskDelay(50); 
    }
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_deinit());
  
}


TEST_CASE("ir","test rca ir0 repeat  send down data", "[Driver/ir0 9]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_rca_init());
  
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_rca_send_data_repeat(0x8b074f&0xffffff));
      
     for(uint32_t i = 0;i<800;i++)
    {
        MS_LOGI(MS_DRIVER,"nec repeat send data cnt =  %x\n",i);
        vTaskDelay(50); 
    }
	 
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_deinit());
  
}

TEST_CASE("ir","test rca ir0 int send up data", "[Driver/ir0 10]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_rca_init());
    for(uint32_t i = 0;i<3;i++)
    {
        TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_rca_int_send_data(0x0b0f4f&0xffffff));
        //MS_LOGI(MS_DRIVER,"rca send data cnt =  %x\n",i);
         vTaskDelay(500); 
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_deinit());
  
}


TEST_CASE("ir","test rca ir0 int send down data", "[Driver/ir0 11]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_rca_init());
    for(uint32_t i = 0;i<3;i++)
    {
        TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_rca_int_send_data(0x8b074f&0xffffff));
        //MS_LOGI(MS_DRIVER,"rca send data cnt =  %x\n",i);
         vTaskDelay(500); 
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir0_deinit());
  
}




