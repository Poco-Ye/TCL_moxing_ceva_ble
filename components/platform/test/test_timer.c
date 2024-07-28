/*
 * test_trng.c
 *
 *  Created on: 2021年12月16日
 *      Author:haijun.mai
 */

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
#include "timer.h"

#define TIMER_PRINT_CNT   20
#define TIMER_PRINT_STOP_CNT   24
#define TIMER_USER_DEFINED  1  /* timer reload automatic */
#define TIMER_FREE_RUNNING  0  /* timer reload manual */


TEST_CASE("timer","test_timer0 user define mode", "[Driver/timer_0]")
{
    uint32_t i = 0; 

     MS_LOGI(MS_DRIVER, "\r\ntimer 0 test start!\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_init());


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_start());

     for(i=0;i<TIMER_PRINT_CNT;i++)
     {
           MS_LOGI(MS_DRIVER, "\r\n timer value[%d] = %x\n",i,timer0_get_curvalue());
     }
	

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_stop());
	
    MS_LOGI(MS_DRIVER, "\r\ntimer 0 stop!\n");
	
      for(i=TIMER_PRINT_CNT;i<TIMER_PRINT_STOP_CNT;i++)
     {
           MS_LOGI(MS_DRIVER, "\r\n timer value[%d] = %x\n",i,timer0_get_curvalue());
     }
  
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer0_deinit());

}


TEST_CASE("timer","test_timer1 user define mode", "[Driver/timer_1]")
{
    uint32_t i = 0; 

     MS_LOGI(MS_DRIVER, "\r\ntimer 1 test start!\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_init());


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_start());

     for(i=0;i<TIMER_PRINT_CNT;i++)
     {
           MS_LOGI(MS_DRIVER, "\r\n timer value[%d] = %x\n",i,timer1_get_curvalue());
     }
	

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_stop());
	
    MS_LOGI(MS_DRIVER, "\r\ntimer 1 stop!\n");
	
      for(i=TIMER_PRINT_CNT;i<TIMER_PRINT_STOP_CNT;i++)
     {
           MS_LOGI(MS_DRIVER, "\r\n timer value[%d] = %x\n",i,timer1_get_curvalue());
     }
  
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer1_deinit());

}


TEST_CASE("timer","test_timer2 user define mode", "[Driver/timer_2]")
{
    uint32_t i = 0; 

     MS_LOGI(MS_DRIVER, "\r\ntimer 2 test start!\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_init());


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_start());

     for(i=0;i<TIMER_PRINT_CNT;i++)
     {
           MS_LOGI(MS_DRIVER, "\r\n timer value[%d] = %x\n",i,timer2_get_curvalue());
     }
	

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_stop());
	
    MS_LOGI(MS_DRIVER, "\r\ntimer 2 stop!\n");
	
      for(i=TIMER_PRINT_CNT;i<TIMER_PRINT_STOP_CNT;i++)
     {
           MS_LOGI(MS_DRIVER, "\r\n timer value[%d] = %x\n",i,timer2_get_curvalue());
     }
  
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer2_deinit());

}


TEST_CASE("timer","test_timer3 user define mode", "[Driver/timer_3]")
{
    uint32_t i = 0; 

     MS_LOGI(MS_DRIVER, "\r\ntimer 3 test start!\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_init());


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_start());

     for(i=0;i<TIMER_PRINT_CNT;i++)
     {
           MS_LOGI(MS_DRIVER, "\r\n timer value[%d] = %x\n",i,timer3_get_curvalue());
     }
	

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_stop());
	
    MS_LOGI(MS_DRIVER, "\r\ntimer 3 stop!\n");
	
      for(i=TIMER_PRINT_CNT;i<TIMER_PRINT_STOP_CNT;i++)
     {
           MS_LOGI(MS_DRIVER, "\r\n timer value[%d] = %x\n",i,timer3_get_curvalue());
     }
  
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer3_deinit());

}

TEST_CASE("timer","test_timer4 user define mode", "[Driver/timer_4]")
{
    uint32_t i = 0; 

     MS_LOGI(MS_DRIVER, "\r\ntimer 4 test start!\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_init());


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_start());

     for(i=0;i<TIMER_PRINT_CNT;i++)
     {
           MS_LOGI(MS_DRIVER, "\r\n timer value[%d] = %x\n",i,timer4_get_curvalue());
     }
	

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_stop());
	
    MS_LOGI(MS_DRIVER, "\r\ntimer 4 stop!\n");
	
      for(i=TIMER_PRINT_CNT;i<TIMER_PRINT_STOP_CNT;i++)
     {
           MS_LOGI(MS_DRIVER, "\r\n timer value[%d] = %x\n",i,timer4_get_curvalue());
     }
  
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer4_deinit());

}


TEST_CASE("timer","test_timer5 user define mode", "[Driver/timer_5]")
{
    uint32_t i = 0; 

     MS_LOGI(MS_DRIVER, "\r\ntimer 5 test start!\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_init());


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_start());

     for(i=0;i<TIMER_PRINT_CNT;i++)
     {
           MS_LOGI(MS_DRIVER, "\r\n timer value[%d] = %x\n",i,timer5_get_curvalue());
     }

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_stop());
	
    MS_LOGI(MS_DRIVER, "\r\ntimer 5 stop!\n");
	
      for(i=TIMER_PRINT_CNT;i<TIMER_PRINT_STOP_CNT;i++)
     {
           MS_LOGI(MS_DRIVER, "\r\n timer value[%d] = %x\n",i,timer5_get_curvalue());
     }
  
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer5_deinit());

}

TEST_CASE("timer","test_timer6 user define mode", "[Driver/timer_6]")
{
    uint32_t i = 0; 

     MS_LOGI(MS_DRIVER, "\r\ntimer 6 test start!\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_init());


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_start());

     for(i=0;i<TIMER_PRINT_CNT;i++)
     {
           MS_LOGI(MS_DRIVER, "\r\n timer value[%d] = %x\n",i,timer6_get_curvalue());
     }
	

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_stop());
	
    MS_LOGI(MS_DRIVER, "\r\ntimer 6 stop!\n");
	
      for(i=TIMER_PRINT_CNT;i<TIMER_PRINT_STOP_CNT;i++)
     {
           MS_LOGI(MS_DRIVER, "\r\n timer value[%d] = %x\n",i,timer6_get_curvalue());
     }
  
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer6_deinit());

}


TEST_CASE("timer","test_timer7 user define mode", "[Driver/timer_7]")
{
    uint32_t i = 0; 

     MS_LOGI(MS_DRIVER, "\r\ntimer 7 test start!\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_init());


    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_start());

     for(i=0;i<TIMER_PRINT_CNT;i++)
     {
           MS_LOGI(MS_DRIVER, "\r\n timer value[%d] = %x\n",i,timer7_get_curvalue());
     }
	

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_stop());
	
    MS_LOGI(MS_DRIVER, "\r\ntimer 7 stop!\n");
	
      for(i=TIMER_PRINT_CNT;i<TIMER_PRINT_STOP_CNT;i++)
     {
           MS_LOGI(MS_DRIVER, "\r\n timer value[%d] = %x\n",i,timer7_get_curvalue());
     }
  
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer7_deinit());

}

TEST_CASE("timer","test_timer  0-7 user define mode", "[Driver/timer_8]")
{
    uint32_t i = 0; 

     MS_LOGI(MS_DRIVER, "\r\ntimer 0-7 test start!\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_init());

    
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_start());

     for(i=0;i<TIMER_PRINT_CNT;i++)
     {
          MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",0,i,timer0_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",1,i,timer1_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",2,i,timer2_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",3,i,timer3_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",4,i,timer4_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",5,i,timer5_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",6,i,timer6_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",7,i,timer7_get_curvalue());
     }
	

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_stop());
    
	
    MS_LOGI(MS_DRIVER, "\r\ntimer 0-7 stop!\n");
	
      for(i=TIMER_PRINT_CNT;i<TIMER_PRINT_STOP_CNT;i++)
     {
          MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",0,i,timer0_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",1,i,timer1_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",2,i,timer2_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",3,i,timer3_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",4,i,timer4_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",5,i,timer5_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",6,i,timer6_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",7,i,timer7_get_curvalue());
     }

    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer0_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer1_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer2_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer3_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer4_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer5_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer6_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer7_deinit());


}


TEST_CASE("timer","test_timer  0-7 free running mode", "[Driver/timer_9]")
{
    uint32_t i = 0; 

     MS_LOGI(MS_DRIVER, "\r\ntimer 0-7 test start!\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_init());

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_set_mode(TIMER_FREE_RUNNING));

    
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_start());

     for(i=0;i<TIMER_PRINT_CNT;i++)
     {
          MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",0,i,timer0_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",1,i,timer1_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",2,i,timer2_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",3,i,timer3_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",4,i,timer4_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",5,i,timer5_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",6,i,timer6_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",7,i,timer7_get_curvalue());
     }
	

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_stop());
    
	
    MS_LOGI(MS_DRIVER, "\r\ntimer 0-7 stop!\n");
	
      for(i=TIMER_PRINT_CNT;i<TIMER_PRINT_STOP_CNT;i++)
     {
          MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",0,i,timer0_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",1,i,timer1_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",2,i,timer2_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",3,i,timer3_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",4,i,timer4_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",5,i,timer5_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",6,i,timer6_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",7,i,timer7_get_curvalue());
     }

    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer0_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer1_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer2_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer3_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer4_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer5_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer6_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer7_deinit());


}


TEST_CASE("timer","test_timer  0-7 free running mode reload timer", "[Driver/timer_10]")
{
    uint32_t i = 0; 

     MS_LOGI(MS_DRIVER, "\r\ntimer 0-7 test start!\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_init());

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_set_mode(TIMER_FREE_RUNNING));

    
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_start());

     for(i=0;i<TIMER_PRINT_CNT;i++)
     {
          MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",0,i,timer0_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",1,i,timer1_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",2,i,timer2_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",3,i,timer3_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",4,i,timer4_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",5,i,timer5_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",6,i,timer6_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",7,i,timer7_get_curvalue());
     }
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_stop());

    timer0_relaod_value(0x10000000);
    timer1_relaod_value(0x20000000);
    timer2_relaod_value(0x30000000);
    timer3_relaod_value(0x40000000);
    timer4_relaod_value(0x50000000);
    timer5_relaod_value(0x60000000);
    timer6_relaod_value(0x70000000);
    timer7_relaod_value(0x80000000);

   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_start());
	
    MS_LOGI(MS_DRIVER, "\r\n\r\n reload timer 0-7 !\n");

    for(i=0;i<TIMER_PRINT_CNT;i++)
     {
          MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",0,i,timer0_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",1,i,timer1_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",2,i,timer2_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",3,i,timer3_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",4,i,timer4_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",5,i,timer5_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",6,i,timer6_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",7,i,timer7_get_curvalue());
     }


	

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_stop());
    
	
    MS_LOGI(MS_DRIVER, "\r\ntimer 0-7 stop!\n");
	
      for(i=TIMER_PRINT_CNT;i<TIMER_PRINT_STOP_CNT;i++)
     {
          MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",0,i,timer0_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",1,i,timer1_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",2,i,timer2_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",3,i,timer3_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",4,i,timer4_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",5,i,timer5_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",6,i,timer6_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",7,i,timer7_get_curvalue());
     }

    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer0_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer1_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer2_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer3_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer4_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer5_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer6_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer7_deinit());


}

TEST_CASE("timer","test_timer  0-7 user define mode timer reach 0", "[Driver/timer_11]")
{
    uint32_t i = 0; 

     MS_LOGI(MS_DRIVER, "\r\ntimer 0-7 test start!\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_init());

    
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_start());

     //for(i=0;i<TIMER_PRINT_CNT;i++)
     {
          MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",0,i,timer0_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",1,i,timer1_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",2,i,timer2_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",3,i,timer3_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",4,i,timer4_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",5,i,timer5_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",6,i,timer6_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",7,i,timer7_get_curvalue());
     }

     while(!timer0_get_timers_raw_status());
     MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",0,i,timer0_get_curvalue());
     while(!timer1_get_timers_raw_status());
     MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",1,i,timer1_get_curvalue());
     while(!timer2_get_timers_raw_status());
     MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",2,i,timer2_get_curvalue());
     while(!timer3_get_timers_raw_status());
     MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",3,i,timer3_get_curvalue());
     while(!timer4_get_timers_raw_status());
     MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",4,i,timer4_get_curvalue());
     while(!timer5_get_timers_raw_status());
     MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",5,i,timer5_get_curvalue());
     while(!timer6_get_timers_raw_status());
     MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",6,i,timer6_get_curvalue());
     while(!timer7_get_timers_raw_status());
     MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",7,i,timer7_get_curvalue());
    


	

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_stop());
    
	
    MS_LOGI(MS_DRIVER, "\r\ntimer 0-7 stop!\n");
	
      for(i=TIMER_PRINT_CNT;i<TIMER_PRINT_STOP_CNT;i++)
     {
          MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",0,i,timer0_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",1,i,timer1_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",2,i,timer2_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",3,i,timer3_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",4,i,timer4_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",5,i,timer5_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",6,i,timer6_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",7,i,timer7_get_curvalue());
     }

    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer0_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer1_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer2_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer3_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer4_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer5_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer6_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer7_deinit());


}

TEST_CASE("timer","test_timer  0-7 user define mode interrupt", "[Driver/timer_12]")
{
    uint32_t i = 0; 

     MS_LOGI(MS_DRIVER, "\r\ntimer 0-7 test start!\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_init());

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_interrupt_enable());


    
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_start());
   //  vTaskDelay( 1000);
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_start());
   //  vTaskDelay( 1000);
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_start());
   // vTaskDelay( 1000);
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_start());
    //vTaskDelay( 1000);
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_start());
   // vTaskDelay( 1000);
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_start());
  //   vTaskDelay( 1000);
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_start());
   // vTaskDelay( 1000);
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_start());


    vTaskDelay( 10000);

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_interrupt_disable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_interrupt_disable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_interrupt_disable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_interrupt_disable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_interrupt_disable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_interrupt_disable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_interrupt_disable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_interrupt_disable());

   
	
   

     for(i=0;i<TIMER_PRINT_CNT;i++)
     {
          MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",0,i,timer0_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",1,i,timer1_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",2,i,timer2_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",3,i,timer3_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",4,i,timer4_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",5,i,timer5_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",6,i,timer6_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",7,i,timer7_get_curvalue());
     }

   


	

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_stop());
    
	
    MS_LOGI(MS_DRIVER, "\r\ntimer 0-7 stop!\n");
	
      for(i=TIMER_PRINT_CNT;i<TIMER_PRINT_STOP_CNT;i++)
     {
          MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",0,i,timer0_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",1,i,timer1_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",2,i,timer2_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",3,i,timer3_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",4,i,timer4_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",5,i,timer5_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",6,i,timer6_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",7,i,timer7_get_curvalue());
     }

    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer0_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer1_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer2_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer3_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer4_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer5_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer6_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer7_deinit());


}


TEST_CASE("timer","test_timer  0-7 user define mode interrupt loop", "[Driver/timer_13]")
{
    uint32_t i = 0; 

     MS_LOGI(MS_DRIVER, "\r\ntimer 0-7 test start!\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_init());

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_interrupt_enable());


    
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_start());
   //  vTaskDelay( 1000);
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_start());
   //  vTaskDelay( 1000);
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_start());
   // vTaskDelay( 1000);
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_start());
    //vTaskDelay( 1000);
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_start());
   // vTaskDelay( 1000);
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_start());
  //   vTaskDelay( 1000);
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_start());
   // vTaskDelay( 1000);
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_start());

   while(1)
   {
        
        vTaskDelay( 10000);
   
       TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_interrupt_disable());
       TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_interrupt_disable());
       TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_interrupt_disable());
       TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_interrupt_disable());
       TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_interrupt_disable());
       TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_interrupt_disable());
       TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_interrupt_disable());
       TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_interrupt_disable());

   
	
   

       for(i=0;i<TIMER_PRINT_CNT;i++)
       {
            MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",0,i,timer0_get_curvalue());
  	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",1,i,timer1_get_curvalue());
  	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",2,i,timer2_get_curvalue());
  	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",3,i,timer3_get_curvalue());
  	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",4,i,timer4_get_curvalue());
  	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",5,i,timer5_get_curvalue());
  	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",6,i,timer6_get_curvalue());
  	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",7,i,timer7_get_curvalue());
       }

     vTaskDelay( 1000);
   
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_interrupt_enable());

   
    }

	

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_stop());
    
	
    MS_LOGI(MS_DRIVER, "\r\ntimer 0-7 stop!\n");
	
      for(i=TIMER_PRINT_CNT;i<TIMER_PRINT_STOP_CNT;i++)
     {
          MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",0,i,timer0_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",1,i,timer1_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",2,i,timer2_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",3,i,timer3_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",4,i,timer4_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",5,i,timer5_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",6,i,timer6_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer %d value[%d] = %x\n",7,i,timer7_get_curvalue());
     }

    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer0_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer1_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer2_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer3_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer4_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer5_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer6_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer7_deinit());


}


TEST_CASE("timer","test_timer  0-7 free running interrupt mode", "[Driver/timer_14]")
{
    uint32_t i = 0; 

     MS_LOGI(MS_DRIVER, "\r\ntimer 0-7 test start!\n");

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_init());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_init());

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_set_mode(TIMER_FREE_RUNNING));
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_set_mode(TIMER_FREE_RUNNING));

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_interrupt_enable());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_interrupt_enable());
	
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_start());
   TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_start());


   while(1)
   {
          //MS_LOGI(MS_DRIVER, ".");
	   MS_LOGI(MS_DRIVER, "\r\n timer  value[%d] = %x\n",0,timer0_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer  value[%d] = %x\n",1,timer1_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer  value[%d] = %x\n",2,timer2_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer  value[%d] = %x\n",3,timer3_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer  value[%d] = %x\n",4,timer4_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer  value[%d] = %x\n",5,timer5_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer  value[%d] = %x\n",6,timer6_get_curvalue());
	   MS_LOGI(MS_DRIVER, "\r\n timer  value[%d] = %x\n\r\n",7,timer7_get_curvalue());
            vTaskDelay( 5000);
   }


   

    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer0_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer1_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer2_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer3_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer4_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer5_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer6_stop());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, timer7_stop());
    

    
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer0_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer1_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer2_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer3_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer4_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer5_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer6_deinit());
    TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS,timer7_deinit());


}


