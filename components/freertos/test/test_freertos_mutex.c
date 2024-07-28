


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
#include "semphr.h"
#include "sys_tick.h"


static TaskHandle_t mutexTask1_Handler;
static TaskHandle_t mutexTask2_Handler;

static void mutex_task1(void* pvParameters);
static void mutex_task2(void* pvParameters);

static SemaphoreHandle_t testmutex;

TEST_CASE("freertos","test_freertos_mutex_take", "[FreeRTOS/mutex]")
{

    uint32_t cycles;
    uint32_t instructions;	
	ms_sys_get_cycles();
	ms_sys_get_instruction();

    testmutex = xSemaphoreCreateMutex();
    xTaskCreate((TaskFunction_t)mutex_task1, (const char*)"mutex_task1",
                (uint16_t)256, (void*)NULL, (UBaseType_t)2,
                (TaskHandle_t*)&mutexTask1_Handler);

    xTaskCreate((TaskFunction_t)mutex_task2, (const char*)"mutex_task2",
                (uint16_t)256, (void*)NULL, (UBaseType_t)3,
                (TaskHandle_t*)&mutexTask2_Handler);

//    vTaskStartScheduler();

    MS_LOGD(MS_FREERTOS,"Free RTOS mutex test, please waiting\r\n");

    vTaskDelay(5000); 
    vTaskDelete( mutexTask1_Handler );
    vTaskDelete( mutexTask2_Handler ); 

    cycles = ms_sys_get_cycles();
    instructions = ms_sys_get_instruction();
	
    MS_LOGD(MS_FREERTOS,"tatal %d cycle taken in this test\r\n", cycles);
    MS_LOGD(MS_FREERTOS,"tatal %d instruction executedin this test\r\n", instructions);
	

  //  while (1);
}

void mutex_task1(void* pvParameters)
{
    int cnt = 0;
    MS_LOGD(MS_FREERTOS,"Enter to task_1\r\n");
    xSemaphoreTake(testmutex, portMAX_DELAY);
    while (1) 
    {
        MS_LOGD(MS_FREERTOS,"task1 is running %d\r\n", cnt++);
	    vTaskDelay(500);
        if(cnt == 5)
        {
            xSemaphoreGive(testmutex);
        }
        if(cnt > 10)
            break;
    }

    vTaskDelete( mutexTask1_Handler );	
}

void mutex_task2(void* pvParameters)
{
    int cnt = 0;
    MS_LOGD(MS_FREERTOS,"Enter to task_2\r\n");
    xSemaphoreTake(testmutex, portMAX_DELAY);
    MS_LOGD(MS_FREERTOS,"should not reach here until count1 >= 5\r\n");

    while (1) 
    {
        MS_LOGD(MS_FREERTOS,"task2 is running %d\r\n", cnt++);
        vTaskDelay(500);
        if(cnt > 10)
            break;
    }

}








