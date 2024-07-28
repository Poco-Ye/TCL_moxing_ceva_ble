// Copyright 2016-2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <string.h>
#include "unity.h"
#include "uart.h"
#include "log.h"
#include "sys_tick.h"
//#include "unity_test_runner.h"


#define MAX_SERIAL_TEST_BUF_LEN 256


typedef struct
{
   char     databuffer[MAX_SERIAL_TEST_BUF_LEN + 2];
   uint32_t   head;
   uint32_t    tail;
} SerialQueue_Type;

static SerialQueue_Type gb_serialtestqueue;

//extern test_desc_t *s_unity_tests_first;
//extern test_desc_t *s_unity_tests_last;


void unity_test_init(void)
{
    gb_serialtestqueue.head = 0;
	gb_serialtestqueue.tail = 0;
   
   // s_unity_tests_first = NULL;
   // s_unity_tests_last = NULL;
}




int putchar(int ch);
static uint32_t s_test_start, s_test_stop;

void unity_putc(int c)
{
	putchar(c);
}

void unity_flush(void)
{
	//pyxue
    //esp_rom_uart_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);
}



/*
 * internal function, get the data from uart0 buffer 
 * and push to global buffer
 */
bool unity_pushdata(void)
{
    bool  HaveReturn = false;
    int16_t length;
    int8_t len = 60; 
    int16_t index;
    char dst[64];
	int temp;

    length = uart0_read(dst, len);
    if(length > 0)
    {
        //for(temp = 0; temp < length; temp++)
        //{
        //	 MS_LOGI( MS_DRIVER, "dst[%d]:%d", temp, dst[temp]);    
        //}
    	dst[length] = 0;
    	MS_LOGI( MS_DRIVER, "%s",dst);
    }

    for (index = 0; index < length; index ++)
	{
		if((gb_serialtestqueue.tail + 1) % MAX_SERIAL_TEST_BUF_LEN == gb_serialtestqueue.head)
		{
			MS_LOGW( MS_DRIVER, "test serial buffer overflow\r\n");
			gb_serialtestqueue.tail =gb_serialtestqueue.head = 0;
			return false;
		}

		gb_serialtestqueue.databuffer[gb_serialtestqueue.tail] = dst[index];
        gb_serialtestqueue.tail = (gb_serialtestqueue.tail + 1) % MAX_SERIAL_TEST_BUF_LEN;
        //MS_LOGI( MS_DRIVER, "tail %d\r\n", gb_serialtestqueue.tail);
        if (!HaveReturn && dst[index] == 0x0d )
        {
            //MS_LOGI( MS_DRIVER, "get a line\r\n");
        	HaveReturn = true;
			return HaveReturn;
        }
	}
	
	return HaveReturn;

}


/* 
 * read a line from global buffer
 *   
 */
int32_t unity_gets(char *dst, size_t len)
{

    int16_t	temphead = gb_serialtestqueue.head;
    int32_t	lineLen = 0;
	
    while (temphead!=gb_serialtestqueue.tail && (lineLen+1)<len) 
    {
        dst[lineLen] = gb_serialtestqueue.databuffer[temphead];
        temphead = (temphead + 1) % MAX_SERIAL_TEST_BUF_LEN;
        lineLen++;
        if (dst[lineLen-1] == 0x0d )	  // a line
        {
            dst[lineLen] = '\0';
            //MS_LOGI( MS_DRIVER, "get a command line\r\n");
            gb_serialtestqueue.head = temphead;
            //MS_LOGI( MS_DRIVER, "head %d\r\n", gb_serialtestqueue.head);
            return lineLen;
        }
    }
    if(lineLen+1 == len)
    {
        gb_serialtestqueue.head = temphead;
    }
    dst[lineLen] = '\0';
    return 0;

}


void unity_exec_time_start(void)
{
	s_test_start = ms_sys_timer_get_systick();
}

void unity_exec_time_stop(void)
{
    s_test_stop = ms_sys_timer_get_systick();
}

uint32_t unity_exec_time_get_ms(void)
{
    return (s_test_stop - s_test_start);
}

void unity_exec_time_reset(void)
{
	s_test_start = s_test_stop = 0;
}

