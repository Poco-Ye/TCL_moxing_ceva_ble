/*
 * test_gpio.c
 *
 *  Created on: 2021年12月16日
 *      Author:haijun.mai
 */

#include "gpio.h"
#include "uart.h"
#include <string.h>
#include "ms_uart.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"


TEST_CASE("gpio","test gpio out put", "[Driver/gpio]")
{


   for(int i = 1; i<8;i++)
   {
	 TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_out_init(i));

	 for(int j = 0;j<100;j++)
	 {
            gpio_out_put_high_level(i);
            MS_LOGI(MS_DRIVER, ".");
	     vTaskDelay( 20);

            gpio_out_put_low_level(i);
            MS_LOGI(MS_DRIVER, ".");
	     vTaskDelay( 20);
	 }
	 
	 TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_deinit(i));
   }
   
}



TEST_CASE("gpio","test gpio input", "[Driver/gpio]")
{


   for(int i = 1; i<8;i++)
   {
	 TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_input_init(i));

	 for(int j = 0;j<50;j++)
	 {
          
           
            MS_LOGI(MS_DRIVER, "\r\ngpio %d = %x\n",i,gpio_get_value(i));
	     vTaskDelay( 2000);
	 }
	 
	 TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_deinit(i));
   }
   
}





TEST_CASE("gpio","test gpio falling interrupt", "[Driver/gpio]")
{


   for(int i = 1; i<8;i++)
   {
	 TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_falling_edge_interrput_init(i));
   }
    for(int j = 0;j<60;j++)
    {
       
           MS_LOGI(MS_DRIVER, ".");
        vTaskDelay( 3000);
   
    }

  for(int i = 1; i<8;i++)
  {
	 TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_deinit(i));
   }
   
}


TEST_CASE("gpio","test gpio rising interrupt", "[Driver/gpio]")
{


   for(int i = 1; i<8;i++)
   {
	 TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_rising_edge_interrput_init(i));
   }
    for(int j = 0;j<60;j++)
    {
       
           MS_LOGI(MS_DRIVER, ".");
        vTaskDelay( 3000);
   
    }

  for(int i = 1; i<8;i++)
  {
	 TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_deinit(i));
   }
   
}

TEST_CASE("gpio","test gpio low level interrupt", "[Driver/gpio]")
{


   for(int i = 1; i<8;i++)
   {
	 TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_low_level_interrput_init(i));
   }
    for(int j = 0;j<60;j++)
    {
       
           MS_LOGI(MS_DRIVER, ".");
        vTaskDelay( 3000);
   
    }

  for(int i = 1; i<8;i++)
  {
	 TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_deinit(i));
   }
   
}


TEST_CASE("gpio","test gpio high level interrupt", "[Driver/gpio]")
{


   for(int i = 1; i<8;i++)
   {
	 TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_high_level_interrput_init(i));
   }
    for(int j = 0;j<60;j++)
    {
       
           MS_LOGI(MS_DRIVER, ".");
        vTaskDelay( 3000);
   
    }

  for(int i = 1; i<8;i++)
  {
	 TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_deinit(i));
   }
   
}




TEST_CASE("gpio","test gpio out put - in put", "[Driver/gpio]")
{
    int result = 0;
    
    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_out_init(1));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_input_init(2));
    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_out_init(3));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_input_init(4));
    
    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_out_init(5));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_input_init(6));

	 
    
    for(int j = 0;j<100000;j++)
    {
    
        MS_LOGI(MS_DRIVER, "gpio test cnt = %d!\n",j);
        gpio_out_put_high_level(1);
        gpio_out_put_high_level(3);
        gpio_out_put_high_level(5);
		
        vTaskDelay( 10);
		
        result = gpio_get_value(2);
        if(!result)
        {
            MS_LOGI(MS_DRIVER, "gpio 2 get high level error!\n");
            while(1);
        }
	 result = gpio_get_value(4);
        if(!result)
        {
            MS_LOGI(MS_DRIVER, "gpio 4 get high level error!\n");
            while(1);
        }

	 result = gpio_get_value(6);
        if(!result)
        {
            MS_LOGI(MS_DRIVER, "gpio 6 get high level error!\n");
            while(1);
        }
		
        vTaskDelay( 10);
        
        
        
        gpio_out_put_low_level(1);
        gpio_out_put_low_level(3);
        gpio_out_put_low_level(5);
        vTaskDelay( 10);


		
        result = gpio_get_value(2);
        if(result)
        {
            MS_LOGI(MS_DRIVER, "gpio 2 get low level error!\n");
            while(1);
        }
		
	  result = gpio_get_value(4);
        if(result)
        {
            MS_LOGI(MS_DRIVER, "gpio4 get low level error!\n");
            while(1);
        }
		
        result = gpio_get_value(6);
        if(result)
        {
            MS_LOGI(MS_DRIVER, "gpio6 get low level error!\n");
            while(1);
        }
        
        vTaskDelay( 10);
    }








		 
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_deinit(1));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_deinit(2));
    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_deinit(3));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_deinit(4));
    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_deinit(5));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, gpio_deinit(6));
 
}

