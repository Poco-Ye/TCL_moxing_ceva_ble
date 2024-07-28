/*
 * test_aux_adc.c
 *
 *  Created on: 2021年12月16日
 *      Author:haijun.mai
 */

#include "aux_adc.h"
#include "trng.h"
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


TEST_CASE("adc","test_aux_adc single channel one time sample (random channel,cpu)", "[Driver/aux_adc 0]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, trng_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_mask_cpu_interrupt());
  

#if 0
        for(uint8_t i =0;i<8;i++)
	{
               MS_LOGI(MS_DRIVER, "data[%d] = %x\r\n",i,trng_get_data());
	
        }
	#endif

    for(uint8_t i =0;i<8;i++)
	{

           
         
	     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_channel_queue( MS_AUX_ADC_Q0,trng_get_data()&0x7));
            TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_enable());
            
            
            while(AUX_ADC_IRQSTATUS_SMP_DONE != (aux_adc_get_interrupt_status()&AUX_ADC_IRQSTATUS_SMP_DONE))
            {
                ;
            }
           
            
	     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_clear_interrupt_status());
            
            
            
            MS_LOGI(MS_DRIVER, "data = %x\r\n",aux_adc_get_data());
           
	     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,   aux_adc_disable());
            
            vTaskDelay( 2000);


	}


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, trng_deinit());

}



TEST_CASE("adc","test_aux_adc single channel one time sample (sequence channel,cpu)", "[Driver/aux_adc 1]")
{


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_mask_cpu_interrupt());


    for(uint8_t i =0;i<8;i++)
	{

           
	     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_channel_queue( MS_AUX_ADC_Q0,i&0x7));
            TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_enable());
            
            
            while(AUX_ADC_IRQSTATUS_SMP_DONE != (aux_adc_get_interrupt_status()&AUX_ADC_IRQSTATUS_SMP_DONE))
            {
                ;
            }
           
            
	     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_clear_interrupt_status());
            
            
            
            MS_LOGI(MS_DRIVER, "data = %x\r\n",aux_adc_get_data());
           
	     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,   aux_adc_disable());
            
            vTaskDelay( 2000);


	}



    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_deinit());


}


TEST_CASE("adc","test_aux_adc multi channel one time sample (sequence channel,cpu)", "[Driver/aux_adc 2]")
{


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_mask_cpu_interrupt());
  
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_channel_queue_depth( 8));



    for(uint8_t i =0;i<8;i++)
    {
	     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_channel_queue( i,i&0x7));
     }

	
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_enable());
     
     
     while(AUX_ADC_IRQSTATUS_SMP_DONE != (aux_adc_get_interrupt_status()&AUX_ADC_IRQSTATUS_SMP_DONE))
     {
     ;
     }
     
     
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_clear_interrupt_status());
     


     while(AUX_ADC_FIFOSTATUS_EMPTY != (AUX_ADC_FIFOSTATUS_EMPTY&aux_adc_get_fifo_status()))
     {
          MS_LOGI(MS_DRIVER, "data = %x\r\n",aux_adc_get_data());
     }

     
	 
     
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,   aux_adc_disable());
     
     vTaskDelay( 2000);






    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_deinit());


}



TEST_CASE("adc","test_aux_adc single channel multi time sample (sequence channel,cpu)", "[Driver/aux_adc 3]")
{


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_mask_cpu_interrupt());
  
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_sample_count( 8));



    for(uint8_t i =0;i<8;i++)
    {

      MS_LOGI(MS_DRIVER, "\r\nchannel %d 8  times sample test start \r\n",i);
	
      TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_channel_queue( 0,i&0x7));
    
	
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_enable());
     
     
     while(AUX_ADC_IRQSTATUS_SMP_DONE != (aux_adc_get_interrupt_status()&AUX_ADC_IRQSTATUS_SMP_DONE))
     {
     ;
     }
     
     
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_clear_interrupt_status());
     


     while(AUX_ADC_FIFOSTATUS_EMPTY != (AUX_ADC_FIFOSTATUS_EMPTY&aux_adc_get_fifo_status()))
     {
          MS_LOGI(MS_DRIVER, "data = %x\r\n",aux_adc_get_data());
     }

     
	 
     
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,   aux_adc_disable());
     
     vTaskDelay( 2000);


    }



    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_deinit());


}


TEST_CASE("adc","test_aux_adc multi channel(4) multi time(2) sample (sequence channel,cpu)", "[Driver/aux_adc 4]")
{


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_mask_cpu_interrupt());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_channel_queue_depth( 4));
    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_sample_count( 2));



    for(uint8_t i =0;i<4;i++)
    {

      MS_LOGI(MS_DRIVER, "\r\nchannel %d 8  times sample test start \r\n",i);
	
      TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_channel_queue( i,i&0x7));
    
     }
	
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_enable());
     
     
     while(AUX_ADC_IRQSTATUS_SMP_DONE != (aux_adc_get_interrupt_status()&AUX_ADC_IRQSTATUS_SMP_DONE))
     {
     ;
     }
     
     
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_clear_interrupt_status());
     


     while(AUX_ADC_FIFOSTATUS_EMPTY != (AUX_ADC_FIFOSTATUS_EMPTY&aux_adc_get_fifo_status()))
     {
          MS_LOGI(MS_DRIVER, "data = %x\r\n",aux_adc_get_data());
     }

     
	 
     
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,   aux_adc_disable());
     
     vTaskDelay( 2000);





    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_deinit());


}



TEST_CASE("adc","test_aux_adc multi channel(2) multi time(4) sample (sequence channel,cpu)", "[Driver/aux_adc 5]")
{


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_mask_cpu_interrupt());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_channel_queue_depth( 2));
    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_sample_count( 4));



    for(uint8_t i =0;i<2;i++)
    {

      MS_LOGI(MS_DRIVER, "\r\nchannel %d 8  times sample test start \r\n",i);
	
      TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_channel_queue( i,i&0x7));
    
     }
	
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_enable());
     
     
     while(AUX_ADC_IRQSTATUS_SMP_DONE != (aux_adc_get_interrupt_status()&AUX_ADC_IRQSTATUS_SMP_DONE))
     {
     ;
     }
     
     
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_clear_interrupt_status());
     


     while(AUX_ADC_FIFOSTATUS_EMPTY != (AUX_ADC_FIFOSTATUS_EMPTY&aux_adc_get_fifo_status()))
     {
          MS_LOGI(MS_DRIVER, "data = %x\r\n",aux_adc_get_data());
     }

     
	 
     
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,   aux_adc_disable());
     
     vTaskDelay( 2000);





    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_deinit());


}



TEST_CASE("adc","test_aux_adc multi channel(2) multi time(4) sample (random channel,cpu)", "[Driver/aux_adc 6]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, trng_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_mask_cpu_interrupt());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_channel_queue_depth( 4));
    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_sample_count( 2));
 


    for(uint8_t i =0;i<4;i++)
    {

      MS_LOGI(MS_DRIVER, "\r\nchannel %d 8  times sample test start \r\n",i);
	
      TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_channel_queue( i,trng_get_data()&0x7));
    
     }
	
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_enable());
     
     
     while(AUX_ADC_IRQSTATUS_SMP_DONE != (aux_adc_get_interrupt_status()&AUX_ADC_IRQSTATUS_SMP_DONE))
     {
     ;
     }
     
     
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_clear_interrupt_status());
     


     while(AUX_ADC_FIFOSTATUS_EMPTY != (AUX_ADC_FIFOSTATUS_EMPTY&aux_adc_get_fifo_status()))
     {
          MS_LOGI(MS_DRIVER, "data = %x\r\n",aux_adc_get_data());
     }

     
	 
     
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,   aux_adc_disable());
     
     vTaskDelay( 2000);





    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, trng_deinit());


}



TEST_CASE("adc","test_aux_adc multi channel(2) multi time(4) sample (random channel,cpu)", "[Driver/aux_adc 7]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, trng_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_mask_cpu_interrupt());
    

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_channel_queue_depth( 2));
    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_sample_count( 4));



    for(uint8_t i =0;i<2;i++)
    {

      MS_LOGI(MS_DRIVER, "\r\nchannel %d 8  times sample test start \r\n",i);
	
      TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_channel_queue( i,trng_get_data()&0x7));
    
     }
	
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_enable());
     
     
     while(AUX_ADC_IRQSTATUS_SMP_DONE != (aux_adc_get_interrupt_status()&AUX_ADC_IRQSTATUS_SMP_DONE))
     {
     ;
     }
     
     
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_clear_interrupt_status());

     while(AUX_ADC_FIFOSTATUS_EMPTY != (AUX_ADC_FIFOSTATUS_EMPTY&aux_adc_get_fifo_status()))
     {
          MS_LOGI(MS_DRIVER, "data = %x\r\n",aux_adc_get_data());
     }

     
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,   aux_adc_disable());
     
     vTaskDelay( 2000);


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, trng_deinit());


}


TEST_CASE("adc","test_aux_adc single channel one time sample (sequence channel,interrupt)", "[Driver/aux_adc 8]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_init());



    for(uint8_t i =0;i<8;i++)
    {

           
	     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_channel_queue( MS_AUX_ADC_Q0,i&0x7));
            
            /*note this must do before adc enable */
            aux_adc_clear_interrupt_flag();
			
            TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_enable());
			
	     while(0 == aux_adc_get_interrupt_flag())
	     {
                 vTaskDelay( 20);

	     }
            
            
            
              while(AUX_ADC_FIFOSTATUS_EMPTY != (AUX_ADC_FIFOSTATUS_EMPTY&aux_adc_get_fifo_status()))
            {
                 MS_LOGI(MS_DRIVER, "data = %x\r\n",aux_adc_get_data());
            }
           
	     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,   aux_adc_disable());
            
            vTaskDelay( 2000);


	}



    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_deinit());


}

TEST_CASE("adc","test_aux_adc multi channel(2) multi time(4) sample (random channel,interrupt)", "[Driver/aux_adc 9]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, trng_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_init());

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_channel_queue_depth( 2));
    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_sample_count( 4));



    for(uint8_t i =0;i<2;i++)
    {

      MS_LOGI(MS_DRIVER, "\r\nchannel %d 8  times sample test start \r\n",i);
	
      TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_channel_queue( i,trng_get_data()&0x7));
    
     }
	
      /*note this must do before adc enable */
        aux_adc_clear_interrupt_flag();
		
        TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_enable());
		
     while(0 == aux_adc_get_interrupt_flag())
     {
             vTaskDelay( 20);

     }
     
     
  

     while(AUX_ADC_FIFOSTATUS_EMPTY != (AUX_ADC_FIFOSTATUS_EMPTY&aux_adc_get_fifo_status()))
     {
          MS_LOGI(MS_DRIVER, "data = %x\r\n",aux_adc_get_data());
     }

     
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,   aux_adc_disable());
     
     vTaskDelay( 2000);


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, trng_deinit());


}


TEST_CASE("adc","test_aux_adc multi channel(2) multi time(4) sample (random channel,dma)", "[Driver/aux_adc 10]")
{

    uint32_t result_data[8] = {0};
    uint8_t i = 0;
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, trng_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_init());

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_channel_queue_depth( 2));
    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_sample_count( 4));
    /*for dma this is must set*/
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_watermask(1));

     

    for( i=0;i<2;i++)
    {

      MS_LOGI(MS_DRIVER, "\r\n channel %d 8  times sample test start \r\n",i);
	
      TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  aux_adc_set_channel_queue( i,trng_get_data()&0x7));
    
     }
	
      /*note this must do before adc enable */
        aux_adc_clear_interrupt_flag();
		
        TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_enable());
		
     while(0 == aux_adc_get_interrupt_flag())
     {
             vTaskDelay( 20);

     }
     
     aux_adc_dma_receive_data( result_data, 32);
     vTaskDelay( 20);

      for( i =0;i<8;i++)
     {
           MS_LOGI(MS_DRIVER, "data[%d] = %x\r\n",i,result_data[i]);
	 
     }
     TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_disable());
     
     vTaskDelay( 2000);


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, aux_adc_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, trng_deinit());


}