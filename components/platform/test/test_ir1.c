/*
 * test_ir.c
 *
 *  Created on: 2021年12月16日
 *      Author:haijun.mai
 */
#ifdef IR1_TO2
#include "ir1.h"
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
#define  IR1FREQCYCLE                                                             (0.04166) 


TEST_CASE("ir","test nec ir1 send up data", "[Driver/ir1 0]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_nec_init());
    for(uint32_t i = 0;i<8000;i++)
    {
        //TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_nec_send_data(0x8ce0f731));
        //up 
	TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_nec_send_data(0x2fd09966));
        //down
	//TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_nec_send_data(0x2ed19966));
      // MS_LOGI(MS_DRIVER,"nec send data cnt =  %x\n",i);
    
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_deinit());
  
}

TEST_CASE("ir","test nec ir1 send down data", "[Driver/ir1 1]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_nec_init());
    for(uint32_t i = 0;i<8000;i++)
    {
        //TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_nec_send_data(0x8ce0f731));
        //up 
	TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_nec_send_data(0x2ed19966));
        //down
	//TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_nec_send_data(0x2ed19966));
        //MS_LOGI(MS_DRIVER,"nec send data cnt =  %x\n",i);
    
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_deinit());
  
}


TEST_CASE("ir","test nec ir1 repeat send up data", "[Driver/ir1 2]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_nec_init());
    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_nec_send_data_repeat(0x2fd09966));	
    
    for(uint32_t i = 0;i<800;i++)
    {
        MS_LOGI(MS_DRIVER,"nec repeat send data cnt =  %x\n",i);
        vTaskDelay(50); 
    }
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  ir1_nec_send_data_repeat_stop());
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_deinit());
  
}


TEST_CASE("ir","test nec ir1 repeat send down data", "[Driver/ir1 3]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_nec_init());
    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_nec_send_data_repeat(0x2ed19966));	
    
    for(uint32_t i = 0;i<800;i++)
    {
        MS_LOGI(MS_DRIVER,"nec repeat send data cnt =  %x\n",i);
        vTaskDelay(50); 
    }
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,  ir1_nec_send_data_repeat_stop());
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_deinit());
  
}



TEST_CASE("ir","test rca ir1 send up data", "[Driver/ir1 4]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_rca_init());
    for(uint32_t i = 0;i<8000;i++)
    {
        TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_rca_send_data(0x0b0f4f&0xffffff));
        //MS_LOGI(MS_DRIVER,"rca send data cnt =  %x\n",i);
    
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_deinit());
  
}


TEST_CASE("ir","test rca ir1 send down data", "[Driver/ir1 5]")
{

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_rca_init());
    for(uint32_t i = 0;i<8000;i++)
    {
        TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_rca_send_data(0x8b074f&0xffffff));
        //MS_LOGI(MS_DRIVER,"rca send data cnt =  %x\n",i);
    
    }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_deinit());
  
}





TEST_CASE("ir","test arbitrary ir1 cpu  send data  (data control by sw on compensation)", "[Driver/ir1 6]")
{
#if 0
   int32_t ir_test_data[] = { 43800, 21600,  6600, 13200,  6600, 13800,  6600, 13200,
   	                                      7200,   6600,  6600,   6600,  6600,  7200,  6600, 13200,
   	                                      6600,   7200,  6600,   6600,  6600,  7200,  6600,   6600,
   	                                      6600,   7200,  6600, 13200,  7200,  6600,  6600,   6600,
   	                                      6600,   7200,  6600, 13200,  6600,  7200,  6600,   6600,
   	                                      6600,   7200,  6600,   6600,  6600,  7200,  6600, 13200,
   	                                      6600,   6600,  3650,   1800,  6600,  6600,  6600,   6600,
   	                                      6600, 13800,  6600,   6600,  7200,  6600,  6600,   6600,
   	                                      6600, 13800,  6600,   6600,  6600, 13200,  6600, 13800, 
   	                                      6600, 13200,  7200,  13200,  6600,  6600,  6600, 13800,
   	                                      6600, 13200,  7200,  13800, 7200};
#else
        int32_t ir_test_data[] =  { 3650/IR1FREQCYCLE,  1800/IR1FREQCYCLE, 550/IR1FREQCYCLE , 1100/IR1FREQCYCLE, 550/IR1FREQCYCLE, 1150/IR1FREQCYCLE, 550/IR1FREQCYCLE, 1100/IR1FREQCYCLE,
                              600/IR1FREQCYCLE ,  550/IR1FREQCYCLE ,  550/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE,  600/IR1FREQCYCLE , 550/IR1FREQCYCLE, 1100/IR1FREQCYCLE,
                              550/IR1FREQCYCLE ,  600/IR1FREQCYCLE ,  550/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE,  600/IR1FREQCYCLE , 550/IR1FREQCYCLE,  550/IR1FREQCYCLE ,
                              550/IR1FREQCYCLE ,  600/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 1100/IR1FREQCYCLE, 600/IR1FREQCYCLE,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE,  550/IR1FREQCYCLE ,
                              550/IR1FREQCYCLE ,  600/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 1100/IR1FREQCYCLE, 550/IR1FREQCYCLE,  600/IR1FREQCYCLE , 550/IR1FREQCYCLE,  550/IR1FREQCYCLE ,
                              550/IR1FREQCYCLE ,  600/IR1FREQCYCLE ,  550/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE,  600/IR1FREQCYCLE , 550/IR1FREQCYCLE, 1100/IR1FREQCYCLE,
                              550/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 3650/IR1FREQCYCLE, 1800/IR1FREQCYCLE, 550/IR1FREQCYCLE,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE,    550/IR1FREQCYCLE,
                              550/IR1FREQCYCLE , 1150/IR1FREQCYCLE,  550/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 600/IR1FREQCYCLE,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE,   550/IR1FREQCYCLE ,
                              550/IR1FREQCYCLE , 1150/IR1FREQCYCLE,  550/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE, 1100/IR1FREQCYCLE, 550/IR1FREQCYCLE,  1150/IR1FREQCYCLE,
                              550/IR1FREQCYCLE , 1100/IR1FREQCYCLE,  600/IR1FREQCYCLE ,  1100/IR1FREQCYCLE, 550/IR1FREQCYCLE,  550/IR1FREQCYCLE, 550/IR1FREQCYCLE,  1150/IR1FREQCYCLE,
                              550/IR1FREQCYCLE , 1100/IR1FREQCYCLE,  600/IR1FREQCYCLE,   1050/IR1FREQCYCLE, 600/IR1FREQCYCLE};
#endif

      for(uint32_t i = 0;i<(sizeof(ir_test_data)/4);i++)
    {
          if(i%2==0)
          {
              ir_test_data[i]  |= 0x80000000;
	   }
       
    }


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_arbitrary_sw_init());
    
    
      
    for(uint32_t i = 0;i<500;i++)
    {
        //MS_LOGI(MS_DRIVER,"arbitrary  send data cnt =  %x\n",i);
       TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_arbitrary_send_data(ir_test_data,16 ));	
        vTaskDelay(100); 
	 
    }


    
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_deinit());
  
}



TEST_CASE("ir","test arbitrary ir1 cpu  send data  (data control by hw  compensation high levle)", "[Driver/ir1 7]")
{
   #if 0
   int32_t ir_test_data[] = { 43800, 21600,  6600, 13200,  6600, 13800,  6600, 13200,
   	                                      7200,   6600,  6600,   6600,  6600,  7200,  6600, 13200,
   	                                      6600,   7200,  6600,   6600,  6600,  7200,  6600,   6600,
   	                                      6600,   7200,  6600, 13200,  7200,  6600,  6600,   6600,
   	                                      6600,   7200,  6600, 13200,  6600,  7200,  6600,   6600,
   	                                      6600,   7200,  6600,   6600,  6600,  7200,  6600, 13200,
   	                                      6600,   6600,  3650,   1800,  6600,  6600,  6600,   6600,
   	                                      6600, 13800,  6600,   6600,  7200,  6600,  6600,   6600,
   	                                      6600, 13800,  6600,   6600,  6600, 13200,  6600, 13800, 
   	                                      6600, 13200,  7200,  13200,  6600,  6600,  6600, 13800,
   	                                      6600, 13200,  7200,  13800, 7200};

   #else
      int32_t ir_test_data[] =  { 3650/IR1FREQCYCLE,  1800/IR1FREQCYCLE, 550/IR1FREQCYCLE , 1100/IR1FREQCYCLE, 550/IR1FREQCYCLE, 1150/IR1FREQCYCLE, 550/IR1FREQCYCLE, 1100/IR1FREQCYCLE,
                              600/IR1FREQCYCLE ,  550/IR1FREQCYCLE ,  550/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE,  600/IR1FREQCYCLE , 550/IR1FREQCYCLE, 1100/IR1FREQCYCLE,
                              550/IR1FREQCYCLE ,  600/IR1FREQCYCLE ,  550/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE,  600/IR1FREQCYCLE , 550/IR1FREQCYCLE,  550/IR1FREQCYCLE ,
                              550/IR1FREQCYCLE ,  600/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 1100/IR1FREQCYCLE, 600/IR1FREQCYCLE,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE,  550/IR1FREQCYCLE ,
                              550/IR1FREQCYCLE ,  600/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 1100/IR1FREQCYCLE, 550/IR1FREQCYCLE,  600/IR1FREQCYCLE , 550/IR1FREQCYCLE,  550/IR1FREQCYCLE ,
                              550/IR1FREQCYCLE ,  600/IR1FREQCYCLE ,  550/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE,  600/IR1FREQCYCLE , 550/IR1FREQCYCLE, 1100/IR1FREQCYCLE,
                              550/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 3650/IR1FREQCYCLE, 1800/IR1FREQCYCLE, 550/IR1FREQCYCLE,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE,    550/IR1FREQCYCLE,
                              550/IR1FREQCYCLE , 1150/IR1FREQCYCLE,  550/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 600/IR1FREQCYCLE,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE,   550/IR1FREQCYCLE ,
                              550/IR1FREQCYCLE , 1150/IR1FREQCYCLE,  550/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE, 1100/IR1FREQCYCLE, 550/IR1FREQCYCLE,  1150/IR1FREQCYCLE,
                              550/IR1FREQCYCLE , 1100/IR1FREQCYCLE,  600/IR1FREQCYCLE ,  1100/IR1FREQCYCLE, 550/IR1FREQCYCLE,  550/IR1FREQCYCLE, 550/IR1FREQCYCLE,  1150/IR1FREQCYCLE,
                              550/IR1FREQCYCLE , 1100/IR1FREQCYCLE,  600/IR1FREQCYCLE,   1050/IR1FREQCYCLE, 600/IR1FREQCYCLE};
   #endif

    
     MS_LOGI(MS_DRIVER,"\n\n");
    for(uint32_t i = 0;i<(sizeof(ir_test_data)/4);i++)
    {
        MS_LOGI(MS_DRIVER,"%08d,\n",ir_test_data[i]*4166/10000);
    }
    MS_LOGI(MS_DRIVER,"\n\n");



    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_arbitrary_init());
    
    
      
    for(uint32_t i = 0;i<70000;i++)
    {
        // MS_LOGI(MS_DRIVER,".");
        //MS_LOGI(MS_DRIVER,"arbitrary send data cnt =  %x\n",i);
	TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_arbitrary_send_data(ir_test_data,16 *4));	
      //  vTaskDelay(10); 
    }


    
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_deinit());
		
}

TEST_CASE("ir","test arbitrary ir1 cpu  send data  (data control by hw  compensation all", "[Driver/ir1 8]")
{
#if 0
   int32_t ir_test_data[] = { 43800, 21600,  6600, 13200,  6600, 13800,  6600, 13200,
   	                                      7200,   6600,  6600,   6600,  6600,  7200,  6600, 13200,
   	                                      6600,   7200,  6600,   6600,  6600,  7200,  6600,   6600,
   	                                      6600,   7200,  6600, 13200,  7200,  6600,  6600,   6600,
   	                                      6600,   7200,  6600, 13200,  6600,  7200,  6600,   6600,
   	                                      6600,   7200,  6600,   6600,  6600,  7200,  6600, 13200,
   	                                      6600,   6600,  3650,   1800,  6600,  6600,  6600,   6600,
   	                                      6600, 13800,  6600,   6600,  7200,  6600,  6600,   6600,
   	                                      6600, 13800,  6600,   6600,  6600, 13200,  6600, 13800, 
   	                                      6600, 13200,  7200,  13200,  6600,  6600,  6600, 13800,
   	                                      6600, 13200,  7200,  13800, 7200};
#else
    int32_t ir_test_data[] =  { 3650/IR1FREQCYCLE,  1800/IR1FREQCYCLE, 550/IR1FREQCYCLE , 1100/IR1FREQCYCLE, 550/IR1FREQCYCLE, 1150/IR1FREQCYCLE, 550/IR1FREQCYCLE, 1100/IR1FREQCYCLE,
                              600/IR1FREQCYCLE ,  550/IR1FREQCYCLE ,  550/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE,  600/IR1FREQCYCLE , 550/IR1FREQCYCLE, 1100/IR1FREQCYCLE,
                              550/IR1FREQCYCLE ,  600/IR1FREQCYCLE ,  550/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE,  600/IR1FREQCYCLE , 550/IR1FREQCYCLE,  550/IR1FREQCYCLE ,
                              550/IR1FREQCYCLE ,  600/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 1100/IR1FREQCYCLE, 600/IR1FREQCYCLE,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE,  550/IR1FREQCYCLE ,
                              550/IR1FREQCYCLE ,  600/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 1100/IR1FREQCYCLE, 550/IR1FREQCYCLE,  600/IR1FREQCYCLE , 550/IR1FREQCYCLE,  550/IR1FREQCYCLE ,
                              550/IR1FREQCYCLE ,  600/IR1FREQCYCLE ,  550/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE,  600/IR1FREQCYCLE , 550/IR1FREQCYCLE, 1100/IR1FREQCYCLE,
                              550/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 3650/IR1FREQCYCLE, 1800/IR1FREQCYCLE, 550/IR1FREQCYCLE,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE,    550/IR1FREQCYCLE,
                              550/IR1FREQCYCLE , 1150/IR1FREQCYCLE,  550/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 600/IR1FREQCYCLE,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE,   550/IR1FREQCYCLE ,
                              550/IR1FREQCYCLE , 1150/IR1FREQCYCLE,  550/IR1FREQCYCLE ,  550/IR1FREQCYCLE , 550/IR1FREQCYCLE, 1100/IR1FREQCYCLE, 550/IR1FREQCYCLE,  1150/IR1FREQCYCLE,
                              550/IR1FREQCYCLE , 1100/IR1FREQCYCLE,  600/IR1FREQCYCLE ,  1100/IR1FREQCYCLE, 550/IR1FREQCYCLE,  550/IR1FREQCYCLE, 550/IR1FREQCYCLE,  1150/IR1FREQCYCLE,
                              550/IR1FREQCYCLE , 1100/IR1FREQCYCLE,  600/IR1FREQCYCLE,   1050/IR1FREQCYCLE, 600/IR1FREQCYCLE};
#endif

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_arbitrary_all_com_init());
    
    
      
    for(uint32_t i = 0;i<50;i++)
    {
        //MS_LOGI(MS_DRIVER,"arbitrary  send data cnt =  %x\n",i);
	TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_arbitrary_send_data(ir_test_data,16 ));	
        //vTaskDelay(100); 
    }


    
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_deinit());
  
}


TEST_CASE("ir","test arbitrary for rca send up data", "[Driver/ir1 9]")
{

    int32_t ir_test_data[] =  { 4000/IR1FREQCYCLE,  4000/IR1FREQCYCLE, 500/IR1FREQCYCLE , 2000/IR1FREQCYCLE, 500/IR1FREQCYCLE, 2000/IR1FREQCYCLE, 500/IR1FREQCYCLE, 2000/IR1FREQCYCLE,
                              500/IR1FREQCYCLE ,  2000/IR1FREQCYCLE ,  500/IR1FREQCYCLE ,  1000/IR1FREQCYCLE , 500/IR1FREQCYCLE,  1000/IR1FREQCYCLE , 500/IR1FREQCYCLE, 2000/IR1FREQCYCLE,
                              500/IR1FREQCYCLE ,  1000/IR1FREQCYCLE ,  500/IR1FREQCYCLE ,  2000/IR1FREQCYCLE , 500/IR1FREQCYCLE,  2000/IR1FREQCYCLE , 500/IR1FREQCYCLE,  2000/IR1FREQCYCLE ,
                              500/IR1FREQCYCLE ,  2000/IR1FREQCYCLE ,  500/IR1FREQCYCLE ,  1000/IR1FREQCYCLE,  500/IR1FREQCYCLE,  1000/IR1FREQCYCLE , 500/IR1FREQCYCLE,  1000/IR1FREQCYCLE ,
                              500/IR1FREQCYCLE ,  1000/IR1FREQCYCLE ,  500/IR1FREQCYCLE ,  2000/IR1FREQCYCLE,  500/IR1FREQCYCLE,  2000/IR1FREQCYCLE , 500/IR1FREQCYCLE,  1000/IR1FREQCYCLE ,
                              500/IR1FREQCYCLE ,  2000/IR1FREQCYCLE ,  500/IR1FREQCYCLE ,  1000/IR1FREQCYCLE , 500/IR1FREQCYCLE,  1000/IR1FREQCYCLE ,  500/IR1FREQCYCLE, 1000/IR1FREQCYCLE,
                              500/IR1FREQCYCLE ,  1000/IR1FREQCYCLE ,  500/IR1FREQCYCLE ,  8500/IR1FREQCYCLE };


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_arbitrary_init());
    
    
      
    for(uint32_t i = 0;i<50;i++)
    {
        //MS_LOGI(MS_DRIVER,"arbitrary  send data cnt =  %x\n",i);
	TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_arbitrary_send_data(ir_test_data,52));	
        //vTaskDelay(100); 
    }


    
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_deinit());
  
}


TEST_CASE("ir","test arbitrary for rca send down data", "[Driver/ir1 10]")
{

    int32_t ir_test_data[] =  { 4000/IR1FREQCYCLE,  4000/IR1FREQCYCLE, 500/IR1FREQCYCLE , 2000/IR1FREQCYCLE, 500/IR1FREQCYCLE, 2000/IR1FREQCYCLE, 500/IR1FREQCYCLE, 2000/IR1FREQCYCLE,
                              500/IR1FREQCYCLE ,  2000/IR1FREQCYCLE ,  500/IR1FREQCYCLE ,  1000/IR1FREQCYCLE , 500/IR1FREQCYCLE,  1000/IR1FREQCYCLE , 500/IR1FREQCYCLE, 2000/IR1FREQCYCLE,
                              500/IR1FREQCYCLE ,  1000/IR1FREQCYCLE ,  500/IR1FREQCYCLE ,  2000/IR1FREQCYCLE , 500/IR1FREQCYCLE,  2000/IR1FREQCYCLE , 500/IR1FREQCYCLE,  2000/IR1FREQCYCLE ,
                              500/IR1FREQCYCLE ,  1000/IR1FREQCYCLE ,  500/IR1FREQCYCLE ,  1000/IR1FREQCYCLE , 500/IR1FREQCYCLE,  1000/IR1FREQCYCLE , 500/IR1FREQCYCLE,  1000/IR1FREQCYCLE ,
                              500/IR1FREQCYCLE ,  1000/IR1FREQCYCLE ,  500/IR1FREQCYCLE ,  2000/IR1FREQCYCLE , 500/IR1FREQCYCLE,  2000/IR1FREQCYCLE , 500/IR1FREQCYCLE,  1000/IR1FREQCYCLE ,
                              500/IR1FREQCYCLE ,  2000/IR1FREQCYCLE ,  500/IR1FREQCYCLE ,  1000/IR1FREQCYCLE , 500/IR1FREQCYCLE,  1000/IR1FREQCYCLE , 500/IR1FREQCYCLE,  1000/IR1FREQCYCLE,
                              500/IR1FREQCYCLE ,  2000/IR1FREQCYCLE ,   500/IR1FREQCYCLE,   8500/IR1FREQCYCLE };


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_arbitrary_init());
    
    
      
    for(uint32_t i = 0;i<50;i++)
    {
        //MS_LOGI(MS_DRIVER,"arbitrary  send data cnt =  %x\n",i);
	TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_arbitrary_send_data(ir_test_data,52 ));	
        //vTaskDelay(100); 
    }


    
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_deinit());
  
}




TEST_CASE("ir","test arbitrary ir1 interrupt  send data  ", "[Driver/ir1 11]")
{
    int32_t ir_test_data[] = { 43800, 21600,  6600, 13200,  6600, 13800,  6600, 13200,
   	                                      7200,   6600,  6600,   6600,  6600,  7200,  6600, 13200,
   	                                      6600,   7200,  6600,   6600,  6600,  7200,  6600,   6600,
   	                                      6600,   7200,  6600, 13200,  7200,  6600,  6600,   6600,
   	                                      6600,   7200,  6600, 13200,  6600,  7200,  6600,   6600,
   	                                      6600,   7200,  6600,   6600,  6600,  7200,  6600, 13200,
   	                                      6600,   6600,  3650,   1800,  6600,  6600,  6600,   6600,
   	                                      6600, 13800,  6600,   6600,  7200,  6600,  6600,   6600,
   	                                      6600, 13800,  6600,   6600,  6600, 13200,  6600, 13800, 
   	                                      6600, 13200,  7200,  13200,  6600,  6600,  6600, 13800,
   	                                      6600, 13200,  7200,  13800, 7200};


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_arbitrary_init());
    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_arbitrary_int_send_data(ir_test_data,50) );	
      
    for(uint32_t i = 0;i<10;i++)
    {
        MS_LOGI(MS_DRIVER,"rca  send data cnt =  %x\n",i);
        vTaskDelay(100); 
    }


    
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_deinit());
  
}



TEST_CASE("ir","test arbitrary ir1 dma  send data", "[Driver/ir1 12]")
{
   int32_t ir_test_data[] = { 43800, 21600,  6600, 13200,  6600, 13800,  6600, 13200,
   	                                      7200,   6600,  6600,   6600,  6600,  7200,  6600, 13200,
   	                                      6600,   7200,  6600,   6600,  6600,  7200,  6600,   6600,
   	                                      6600,   7200,  6600, 13200,  7200,  6600,  6600,   6600,
   	                                      6600,   7200,  6600, 13200,  6600,  7200,  6600,   6600,
   	                                      6600,   7200,  6600,   6600,  6600,  7200,  6600, 13200,
   	                                      6600,   6600,  3650,   1800,  6600,  6600,  6600,   6600,
   	                                      6600, 13800,  6600,   6600,  7200,  6600,  6600,   6600,
   	                                      6600, 13800,  6600,   6600,  6600, 13200,  6600, 13800, 
   	                                      6600, 13200,  7200,  13200,  6600,  6600,  6600, 13800,
   	                                      6600, 13200,  7200,  13800, 7200};

     MS_LOGI(MS_DRIVER,"\n\n");
    for(uint32_t i = 0;i<(sizeof(ir_test_data)/4);i++)
    {
        MS_LOGI(MS_DRIVER,"%08d,\n",ir_test_data[i]*417/1000);
    }
    MS_LOGI(MS_DRIVER,"\n\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_arbitrary_init());
    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_arbitrary_dma_send_data(ir_test_data,sizeof(ir_test_data)/4) );	
      
    for(uint32_t i = 0;i<40;i++)
    {
        MS_LOGI(MS_DRIVER," send data cnt =  %x\n",i);
        vTaskDelay(100); 
    }


    
	
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, ir1_deinit());
  
}
#endif
