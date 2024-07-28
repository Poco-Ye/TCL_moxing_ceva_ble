/*
 * test_watchdog.c
 *
 *  Created on: 2021年12月16日
 *      Author:haijun.mai
 */

#include "i2s_master.h"
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

#define TEST_DATA_LEN     40

uint32_t left_chan_data[TEST_DATA_LEN] = {0};
uint32_t right_chan_data[TEST_DATA_LEN] = {0};
uint32_t dma_chan_data[TEST_DATA_LEN] = {0};
uint32_t dma_rx_chan_data[TEST_DATA_LEN] = {0};


TEST_CASE("i2s","test i2s0 master send", "[Driver/i2s0_master_cpu_send]")
{
    uint16_t i = 0;
	
    for(i=0;i<TEST_DATA_LEN;i++)
    {
           left_chan_data[i] = 0x80888501+(i<<2);
           right_chan_data[i] = 0x80888501+(i<<2);
    }

     MS_LOGI(MS_DRIVER,"\r\ni2s0 master test start\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_init());
    for(i=0;i<5;i++)
    {
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_send( left_chan_data, right_chan_data, 20));
          MS_LOGI(MS_DRIVER,"\r\ni2s0 send cnt = %d\n",i);
          vTaskDelay(300);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_deinit());

}

TEST_CASE("i2s","test i2s0 master receive", "[Driver/i2s0_master_cpu_receive]")
{
    uint16_t i = 0;
    uint16_t j = 0;
	
    for(i=0;i<TEST_DATA_LEN;i++)
    {
           left_chan_data[i] = 0;
           right_chan_data[i] = 0;
    }

     MS_LOGI(MS_DRIVER,"\r\ni2s0 master receive start\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_init());
    for(i=0;i<5;i++)
    {
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_receive( left_chan_data, right_chan_data, 20));
           for(j=0;j<20;j++)
           {
                   MS_LOGI(MS_DRIVER,"\r\nleft_chan_data[%d] = %x\n",j,left_chan_data[j]);
		     MS_LOGI(MS_DRIVER,"\r\nright_chan_datat[%d] = %x\n",j,right_chan_data[j]);
	    }
		  
          MS_LOGI(MS_DRIVER,"\r\ni2s0 receive  cnt = %d\n",i);
          vTaskDelay(300);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_deinit());

}



TEST_CASE("i2s","test i2s1 master send", "[Driver/i2s1_master_cpu_send]")
{
    uint16_t i = 0;
	
    for(i=0;i<TEST_DATA_LEN;i++)
    {
         left_chan_data[i] = 0x80888501+(i<<2);
         right_chan_data[i] = 0x80888501+(i<<2);
    }

     MS_LOGI(MS_DRIVER,"\r\ni2s1 master test start\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_init());
    for(i=0;i<5;i++)
    {
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_send( left_chan_data, right_chan_data, 20));
          MS_LOGI(MS_DRIVER,"\r\ni2s1 send cnt = %d\n",i);
          vTaskDelay(300);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_deinit());

}


TEST_CASE("i2s","test i2s1 master receive", "[Driver/i2s1_master_cpu_receive]")
{
    uint16_t i = 0;
    uint16_t j = 0;
	
    for(i=0;i<TEST_DATA_LEN;i++)
    {
           left_chan_data[i] = 0;
           right_chan_data[i] = 0;
    }

     MS_LOGI(MS_DRIVER,"\r\ni2s1 master receive start\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_init());
    for(i=0;i<5;i++)
    {
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_receive( left_chan_data, right_chan_data, 20));
           for(j=0;j<20;j++)
           {
                   MS_LOGI(MS_DRIVER,"\r\nleft_chan_data[%d] = %x\n",j,left_chan_data[j]);
		     MS_LOGI(MS_DRIVER,"\r\nright_chan_datat[%d] = %x\n",j,right_chan_data[j]);
	    }
		  
          MS_LOGI(MS_DRIVER,"\r\ni2s0 receive  cnt = %d\n",i);
          vTaskDelay(300);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_deinit());

}



TEST_CASE("i2s","test i2s0 master interrupt send", "[Driver/i2s0_master_interrupt_send]")
{
    uint16_t i = 0;
	
    for(i=0;i<TEST_DATA_LEN;i++)
    {
           left_chan_data[i] = 0x80888501+(i<<2);
           right_chan_data[i] = 0x80888501+(i<<2);
    }

     MS_LOGI(MS_DRIVER,"\r\ni2s0 master test start\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_init());
    for(i=0;i<5;i++)
    {
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_int_send( left_chan_data, right_chan_data, 20));
          MS_LOGI(MS_DRIVER,"\r\ni2s0 send cnt = %d\n",i);
          vTaskDelay(300);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_deinit());

}


TEST_CASE("i2s","test i2s0 master interrupt receive", "[Driver/i2s0_master_interrupt_receive]")
{
    uint16_t i = 0;
    uint16_t j = 0;
	
    for(i=0;i<TEST_DATA_LEN;i++)
    {
           left_chan_data[i] = 0;
           right_chan_data[i] = 0;
    }

     MS_LOGI(MS_DRIVER,"\r\ni2s0 master interrupt receive start\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_init());
    for(i=0;i<5;i++)
    {
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_int_receive( left_chan_data, right_chan_data, 20));
           for(j=0;j<TEST_DATA_LEN;j++)
           {
                   MS_LOGI(MS_DRIVER,"\r\nleft_chan_data[%d] = %x\n",j,left_chan_data[j]);
		     MS_LOGI(MS_DRIVER,"\r\nright_chan_datat[%d] = %x\n",j,right_chan_data[j]);
	    }
		  
          MS_LOGI(MS_DRIVER,"\r\ni2s0 receive  cnt = %d\n",i);
          vTaskDelay(300);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_deinit());

}





TEST_CASE("i2s","test i2s1 master interrupt send", "[Driver/i2s1_master_interrupt_send]")
{
    uint16_t i = 0;
	
    for(i=0;i<TEST_DATA_LEN;i++)
    {
         left_chan_data[i] = 0x80888501+(i<<2);
         right_chan_data[i] = 0x80888501+(i<<2);
    }

     MS_LOGI(MS_DRIVER,"\r\ni2s1 master test start\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_init());
    for(i=0;i<5;i++)
    {
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_int_send( left_chan_data, right_chan_data, 20));
          MS_LOGI(MS_DRIVER,"\r\ni2s1 send cnt = %d\n",i);
          vTaskDelay(300);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_deinit());

}


TEST_CASE("i2s","test i2s1 master interrupt receive", "[Driver/i2s1_master_interrupt_receive]")
{
    uint16_t i = 0;
    uint16_t j = 0;
	
    for(i=0;i<TEST_DATA_LEN;i++)
    {
           left_chan_data[i] = 0;
           right_chan_data[i] = 0;
    }

     MS_LOGI(MS_DRIVER,"\r\ni2s1 master interrupt receive start\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_init());
    for(i=0;i<5;i++)
    {
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_int_receive( left_chan_data, right_chan_data, 20));
           for(j=0;j<TEST_DATA_LEN;j++)
           {
                   MS_LOGI(MS_DRIVER,"\r\nleft_chan_data[%d] = %x\n",j,left_chan_data[j]);
		     MS_LOGI(MS_DRIVER,"\r\nright_chan_datat[%d] = %x\n",j,right_chan_data[j]);
	    }
		  
          MS_LOGI(MS_DRIVER,"\r\ni2s0 receive  cnt = %d\n",i);
          vTaskDelay(300);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_deinit());

}




TEST_CASE("i2s","test i2s0 master dma send", "[Driver/i2s0_master_dma_send]")
{
    uint16_t i = 0;
	
    for(i=0;i<TEST_DATA_LEN;i++)
    {
           dma_chan_data[i] = 0x80888501+(i<<2);
    }

     MS_LOGI(MS_DRIVER,"\r\ni2s0 master test start\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_init());
    for(i=0;i<5;i++)
    {
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_dma_send( dma_chan_data, sizeof(dma_chan_data)));
          MS_LOGI(MS_DRIVER,"\r\ni2s0 send cnt = %d\n",i);
          vTaskDelay(300);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_deinit());

}


TEST_CASE("i2s","test i2s0 master dma receive", "[Driver/i2s0_master_dma_receive]")
{
    uint16_t i = 0;
    uint16_t j = 0;
	
    for(i=0;i<TEST_DATA_LEN;i++)
    {
           dma_rx_chan_data[i] = 0;
    }

     MS_LOGI(MS_DRIVER,"\r\ni2s0 master dma receive start\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_init());
    //for(i=0;i<5;i++)
    {
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_dma_receive( dma_rx_chan_data, sizeof(dma_rx_chan_data)));
           for(j=0;j<20;j++)
           {
                   MS_LOGI(MS_DRIVER,"\r\ndma_rx_chan_data[%d] = %x\n",j,dma_rx_chan_data[j]);
		     
	    }
		  
          MS_LOGI(MS_DRIVER,"\r\ni2s0 receive  cnt = %d\n",i);
          vTaskDelay(300);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_deinit());

}




TEST_CASE("i2s","test i2s1 master dma send", "[Driver/i2s1_master_dma_send]")
{
    uint16_t i = 0;
	
    for(i=0;i<TEST_DATA_LEN;i++)
    {
           dma_chan_data[i] = 0x80888501+(i<<2);
    }

     MS_LOGI(MS_DRIVER,"\r\ni2s1 master test start\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_init());
    for(i=0;i<5;i++)
    {
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_dma_send( dma_chan_data, sizeof(dma_chan_data)));
          MS_LOGI(MS_DRIVER,"\r\ni2s1 send cnt = %d\n",i);
          vTaskDelay(300);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_deinit());

}



TEST_CASE("i2s","test i2s1 master dma receive", "[Driver/i2s1_master_dma_receive]")
{
    uint16_t i = 0;
    uint16_t j = 0;
	
    for(i=0;i<TEST_DATA_LEN;i++)
    {
           dma_rx_chan_data[i] = 0;
    }

     MS_LOGI(MS_DRIVER,"\r\ni2s1 master dma receive start\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_init());
   // for(i=0;i<5;i++)
    {
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_dma_receive( dma_rx_chan_data, sizeof(dma_rx_chan_data)));
           for(j=0;j<20;j++)
           {
                   MS_LOGI(MS_DRIVER,"\r\ndma_rx_chan_data[%d] = %x\n",j,dma_rx_chan_data[j]);
		     
	    }
		  
          MS_LOGI(MS_DRIVER,"\r\ni2s1  receive  cnt = %d\n",i);
          vTaskDelay(300);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_deinit());

}



TEST_CASE("i2s","test i2s0 master loop back", "[Driver/i2s0_master_cpu_look_back]")
{
    uint16_t i = 0;
    uint16_t j = 0;
    uint32_t tmp_left_chan_data[TEST_DATA_LEN] = {0};
    uint32_t tmp_right_chan_data[TEST_DATA_LEN] = {0};
	
    for(i=0;i<TEST_DATA_LEN;i++)
    {
           left_chan_data[i] = 0x80888501+(i<<2);
           right_chan_data[i] = 0x80888501+(i<<2);
    }

     MS_LOGI(MS_DRIVER,"\r\ni2s0 master test start\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_init());
   // for(j=0;j<5;j++)
    {
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_send( left_chan_data, right_chan_data, 8));
	   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_receive( tmp_left_chan_data, tmp_right_chan_data, 8));

	     for(i=0;i<8;i++)
	    {
	         MS_LOGI(MS_DRIVER,"\r\n tmp_left_chan_data[%x] = %x\n",i,tmp_left_chan_data[i]);
	         MS_LOGI(MS_DRIVER,"\r\n right_chan_data[%x] = %x\n",i,tmp_right_chan_data[i]);
	    }
	        
	    vTaskDelay(300);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_deinit());

}


TEST_CASE("i2s","test i2s1 master loop back", "[Driver/i2s1_master_cpu_look_back]")
{
    uint16_t i = 0;
    uint16_t j = 0;
    uint32_t tmp_left_chan_data[TEST_DATA_LEN] = {0};
    uint32_t tmp_right_chan_data[TEST_DATA_LEN] = {0};
	
    for(i=0;i<TEST_DATA_LEN;i++)
    {
           left_chan_data[i] = 0x80888501+(i<<2);
           right_chan_data[i] = 0x80888501+(i<<2);
    }

     MS_LOGI(MS_DRIVER,"\r\ni2s0 master test start\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_init());
   // for(j=0;j<5;j++)
    {
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_send( left_chan_data, right_chan_data, 8));
	   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_receive( tmp_left_chan_data, tmp_right_chan_data, 8));

	     for(i=0;i<8;i++)
	    {
	         MS_LOGI(MS_DRIVER,"\r\n tmp_left_chan_data[%x] = %x\n",i,tmp_left_chan_data[i]);
	         MS_LOGI(MS_DRIVER,"\r\n right_chan_data[%x] = %x\n",i,tmp_right_chan_data[i]);
	    }
	        
	    vTaskDelay(300);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_deinit());

}


TEST_CASE("i2s","test i2s0 master interrupt loop back", "[Driver/i2s0_master_interrupt_look_back]")
{
    uint16_t i = 0;
    uint16_t j = 0;
    uint32_t tmp_left_chan_data[TEST_DATA_LEN] = {0};
    uint32_t tmp_right_chan_data[TEST_DATA_LEN] = {0};
	
    for(i=0;i<TEST_DATA_LEN;i++)
    {
           left_chan_data[i] = 0x80888501+(i<<2);
           right_chan_data[i] = 0x80888501+(i<<2);
    }

     MS_LOGI(MS_DRIVER,"\r\ni2s0 master test start\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_init());
   // for(j=0;j<5;j++)
    {
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_int_send( left_chan_data, right_chan_data, 8));
	   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_int_receive( tmp_left_chan_data, tmp_right_chan_data, 8));

	     for(i=0;i<8;i++)
	    {
	         MS_LOGI(MS_DRIVER,"\r\n tmp_left_chan_data[%x] = %x\n",i,tmp_left_chan_data[i]);
	         MS_LOGI(MS_DRIVER,"\r\n right_chan_data[%x] = %x\n",i,tmp_right_chan_data[i]);
	    }
	        
	    vTaskDelay(300);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_deinit());

}


TEST_CASE("i2s","test i2s1 master interrupt loop back", "[Driver/i2s1_master_interrupt_look_back]")
{
    uint16_t i = 0;
    uint16_t j = 0;
    uint32_t tmp_left_chan_data[TEST_DATA_LEN] = {0};
    uint32_t tmp_right_chan_data[TEST_DATA_LEN] = {0};
	
    for(i=0;i<TEST_DATA_LEN;i++)
    {
           left_chan_data[i] = 0x80888501+(i<<2);
           right_chan_data[i] = 0x80888501+(i<<2);
    }

     MS_LOGI(MS_DRIVER,"\r\ni2s1 master test start\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_init());
   // for(j=0;j<5;j++)
    {
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_int_send( left_chan_data, right_chan_data, 8));
	   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_int_receive( tmp_left_chan_data, tmp_right_chan_data, 8));

	     for(i=0;i<8;i++)
	    {
	         MS_LOGI(MS_DRIVER,"\r\n tmp_left_chan_data[%x] = %x\n",i,tmp_left_chan_data[i]);
	         MS_LOGI(MS_DRIVER,"\r\n right_chan_data[%x] = %x\n",i,tmp_right_chan_data[i]);
	    }
	        
	    vTaskDelay(300);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_deinit());

}



TEST_CASE("i2s","test i2s0 master dma loop back", "[Driver/i2s0_master_dma_look_back]")
{
    uint16_t i = 0;
    uint16_t j = 0;


        for(i=0;i<TEST_DATA_LEN;i++)
    {
           left_chan_data[i] = 0x80888501+(i<<2);
           right_chan_data[i] = 0x80888501+(i<<2);
    }


      for(i=0;i<TEST_DATA_LEN;i++)
    {
           dma_chan_data[i] = 0x80888501+(i<<2);
    }	
     for(i=0;i<TEST_DATA_LEN;i++)
    {
           dma_rx_chan_data[i] = 0;
    }


     MS_LOGI(MS_DRIVER,"\r\ni2s0 master test start\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_init());
   // for(j=0;j<5;j++)
    {
         // TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_send( left_chan_data, right_chan_data, 16));
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_dma_send( dma_chan_data, 64));
	   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_dma_receive( dma_rx_chan_data, 64));
	   

	     for(i=0;i<16;i++)
	    {
	         MS_LOGI(MS_DRIVER,"\r\n dma_rx_chan_data[%x] = %x\n",i,dma_rx_chan_data[i]);
	    }
	        
	    vTaskDelay(300);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s0_master_deinit());

}


TEST_CASE("i2s","test i2s1 master dma loop back", "[Driver/i2s1_master_dma_look_back]")
{
      uint16_t i = 0;
    uint16_t j = 0;


        for(i=0;i<TEST_DATA_LEN;i++)
    {
           left_chan_data[i] = 0x80888501+(i<<2);
           right_chan_data[i] = 0x80888501+(i<<2);
    }


      for(i=0;i<TEST_DATA_LEN;i++)
    {
           dma_chan_data[i] = 0x80888501+(i<<2);
    }	
     for(i=0;i<TEST_DATA_LEN;i++)
    {
           dma_rx_chan_data[i] = 0;
    }


     MS_LOGI(MS_DRIVER,"\r\ni2s1 master test start\n");
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_init());
   // for(j=0;j<5;j++)
    {
         // TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_send( left_chan_data, right_chan_data, 16));
          TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_dma_send( dma_chan_data, 64));
	   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_dma_receive( dma_rx_chan_data, 64));
	   

	     for(i=0;i<16;i++)
	    {
	         MS_LOGI(MS_DRIVER,"\r\ndma_rx_chan_data[%x] = %x\n",i,dma_rx_chan_data[i]);
	    }
	        
	    vTaskDelay(300);
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, i2s1_master_deinit());

}
