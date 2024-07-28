/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/* Kernel includes. */
#include "FreeRTOS.h" /* Must come first. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "semphr.h"   /* Semaphore related API prototypes. */
#include "task.h"     /* RTOS task related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "log.h"
#include "unity_test_runner.h"

/* The period of the example software timer, specified in milliseconds, and
converted to ticks using the pdMS_TO_TICKS() macro. */
#define mainSOFTWARE_TIMER_PERIOD_MS    pdMS_TO_TICKS(1000)
//#define mainQUEUE_LENGTH                (1)

static void vExampleTimerCallback(TimerHandle_t xTimer);

/* The queue used by the queue send and queue receive tasks. */
//static QueueHandle_t xQueue = NULL;

static TaskHandle_t createTask1_Handler;
static TaskHandle_t createTask2_Handler;


static void create_task1(void* pvParameters);
static void create_task2(void* pvParameters);

TEST_CASE("freertos","test_freertos_task_create", "[FreeRTOS/task]")
{
//    TimerHandle_t xExampleSoftwareTimer = NULL;

    /* Configure the system ready to run the demo.  The clock configuration
    can be done here if it was not done before main() was called. */
    //prvSetupHardware

    // xQueue = xQueueCreate(/* The number of items the queue can hold. */
    //             mainQUEUE_LENGTH,
    //             /* The size of each item the queue holds. */
    //             sizeof(uint32_t));

   // if (xQueue == NULL) {
   // 	MS_LOGD(MS_FREERTOS,"Unable to create xQueue due to low memory.\n");
   //     while (1);
   // }
    xTaskCreate((TaskFunction_t)create_task1, (const char*)"create_task1",
                (uint16_t)256, (void*)NULL, (UBaseType_t)2,
                (TaskHandle_t*)&createTask1_Handler);

    xTaskCreate((TaskFunction_t)create_task2, (const char*)"create_task2",
                (uint16_t)256, (void*)NULL, (UBaseType_t)3,
                (TaskHandle_t*)&createTask2_Handler);

 //   xExampleSoftwareTimer =
 //       xTimerCreate((const char*)"ExTimer", mainSOFTWARE_TIMER_PERIOD_MS,
 //                    pdTRUE, (void*)0, vExampleTimerCallback);

 //   xTimerStart(xExampleSoftwareTimer, 0);
	
 //   vTaskStartScheduler();

 //   xTimerStop(xExampleSoftwareTimer, 0); 
    MS_LOGD(MS_FREERTOS,"create task test....\r\n");
    vTaskDelay(5000); 
    vTaskDelete( createTask1_Handler );
    vTaskDelete( createTask2_Handler );	

  //  while (1);
}

void create_task1(void* pvParameters)
{
    int cnt = 0;
    MS_LOGD(MS_FREERTOS,"Enter to task_1\r\n");
    while (1) {
    	MS_LOGD(MS_FREERTOS,"task1 is running %d\r\n", cnt++);
        vTaskDelay(500);
        if(cnt > 10)
        	break;
    }
}

void create_task2(void* pvParameters)
{
    int cnt = 0;
    MS_LOGD(MS_FREERTOS,"Enter to task_2\r\n");
    while (1) {
    	MS_LOGD(MS_FREERTOS,"task2 is running %d\r\n", cnt++);
        vTaskDelay(500);
        if(cnt > 10)
        	break;
    }
}

static void vExampleTimerCallback(TimerHandle_t xTimer)
{
    /* The timer has expired.  Count the number of times this happens.  The
    timer that calls this function is an auto re-load timer, so it will
    execute periodically. */
    static int cnt = 0;
    MS_LOGD(MS_FREERTOS,"timers Callback %d\r\n", cnt++);
}

/*-----------------------------------------------------------*/
