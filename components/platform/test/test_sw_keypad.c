//--------------------------------------------------------------------
// Copyright (c) 2022 by MooreSilicon.
// All rights reserved.
// MooreSilicon Confidential Proprietary.
//--------------------------------------------------------------------
//     Project name: ms1008
//     File name: test_sw_keypad.c
//     Function    : scan keys by software method
//     Author      : Ethan Sun
//     Dates       : 2022-4-21
//     Version     : V1.0
//-------------------------------------------------------------------
//      Purpose: software keyscan  test
//
//-------------------------------------------------------------------

#include "gpio.h"
#include "uart.h"
#include <string.h>
#include "ms_uart.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"


#include "ms_sw_Key.h"
#include "ms_sw_Keypad.h"


#define KB_ZOOM          0x6F
#define KB_SEARCH_E   0x06
#define KB_SEARCH       0x7F
#define KB_STORE         0xE1
#define KB_MITV           0x9E
#define KB_MENU          0x62
#define KB_EIXT           0xF9
#define KB_BACK          0xD8
#define KB_APPSTORE  0x29
#define KB_POWER       0xD5
#define KB_MUTE         0xC0
#define KB_0                0xCF
#define KB_1                0xCE
#define KB_2                0xCD
#define KB_3                0xCC
#define KB_4                0xCB
#define KB_5                0xCA
#define KB_6                0xC9
#define KB_7                0xC8
#define KB_8                0xC7
#define KB_9                0xC6
#define KB_UP              0xA6
#define KB_DOWN         0xA7
#define KB_LEFT           0xA9
#define KB_RIGHT        0xA8
#define KB_OK              0x0B
#define KB_HOME         0xF7
#define KB_VOL_UP       0xD0
#define KB_VOL_DOWN 0xD1
#define KB_PR_UP         0xD2
#define KB_PR_DOWN   0xD3
#define KB_RED            0xFF
#define KB_GREEN        0x17
#define KB_YELLOW      0x1B
#define KB_BLUE          0x27
#define KB_PICTURE    0xED
#define KB_EPG           0xE5
#define KB_TV             0xC5
#define KB_USB           0xFD
#define KB_HOT           0x1C
#define KB_SETTING   0x30
#define KB_FREEZE     0xF3
#define KB_DISPLAY   0xC3
#define KB_INFO         0x2E
#define KB_SOURCE    0x5C
#define KB_GOLIVE     0x19
#define KB_HISTORY   0x31
#define KB_SLEEP       0xF8
#define KB_FAVORITE 0xFA
#define KB_T9             0x32
#define KB_3D             0x67



#define KB_RETURN_OK      0xA0
#define KB_REMOVE_PAIR  0xAB
#define KB_RF_ERR            0xA3
#define KB_PARE_ERR        0xAA
#define KB_SOUND             0xA5
#define KB_NEXT                0xAC
#define KB_RECORD            0xE8
#define KB_MANGO             0x70
#define KB_MORE                0x71
#define KB_MIRCAST          0x05
#define KB_VOICE               0xD6


const uint8_t keys[SW_KEYPAD_ROWS][SW_KEYPAD_COLS] = 
{
	{KB_EIXT,		KB_MENU,		   KB_MUTE, 		  KB_HOME,				 KB_TV, 	 KB_YELLOW},
	{KB_0,			KB_1,			   KB_2,			  KB_3, 				 KB_4  ,	 KB_5	  },
	{KB_6,			KB_7,			   KB_8,			  KB_9, 				 KB_LEFT,	 KB_RIGHT },
	{KB_UP, 		KB_DOWN ,		   KB_VOL_UP,		  KB_VOL_DOWN  ,		 KB_PR_UP ,  KB_PR_DOWN},
	{KB_RED,		KB_OK,			   KB_POWER,		  KB_VOICE, 			 KB_BACK,	 KB_GREEN }

};



uint8_t rowPins[SW_KEYPAD_ROWS] = {SW_KEYPAD_KEY_ROW1, SW_KEYPAD_KEY_ROW2, SW_KEYPAD_KEY_ROW3, SW_KEYPAD_KEY_ROW4, SW_KEYPAD_KEY_ROW5}; //connect to the row pinouts of the kpd
uint8_t colPins[SW_KEYPAD_COLS] = {SW_KEYPAD_KEY_COL1, SW_KEYPAD_KEY_COL2, SW_KEYPAD_KEY_COL3, SW_KEYPAD_KEY_COL4, SW_KEYPAD_KEY_COL5, SW_KEYPAD_KEY_COL6}; //connect to the column pinouts of the kpd
/*
const uint8_t keys[SW_KEYPAD_ROWS][SW_KEYPAD_COLS] = 
{
	{KB_EIXT,		KB_MENU,		   KB_MUTE, 		  KB_HOME,				 KB_TV    },
	{KB_0,			KB_1,			   KB_2,			  KB_3, 				 KB_4  	  },
	{KB_6,			KB_7,			   KB_8,			  KB_9, 				 KB_LEFT  },
	{KB_UP, 		KB_DOWN ,		   KB_VOL_UP,		  KB_VOL_DOWN  ,		 KB_PR_UP },
	{KB_RED,		KB_OK,			   KB_POWER,		  KB_VOICE, 			 KB_BACK  },
	{KB_YELLOW,     KB_5,              KB_RIGHT,          KB_PR_DOWN,            KB_GREEN }

};



uint8_t rowPins[SW_KEYPAD_ROWS] = {SW_KEYPAD_KEY_ROW1, SW_KEYPAD_KEY_ROW2, SW_KEYPAD_KEY_ROW3, SW_KEYPAD_KEY_ROW4, SW_KEYPAD_KEY_ROW5,SW_KEYPAD_KEY_ROW6 }; //connect to the row pinouts of the kpd
uint8_t colPins[SW_KEYPAD_COLS] = {SW_KEYPAD_KEY_COL1, SW_KEYPAD_KEY_COL2, SW_KEYPAD_KEY_COL3, SW_KEYPAD_KEY_COL4, SW_KEYPAD_KEY_COL5 }; //connect to the column pinouts of the kpd
*/

int32_t keypad_sw_setup()
{
    MS_LOGI(MS_DRIVER, "\r\n keypad_sw_setup entry \r\n");
	ms_sw_Keypad_init( makeKeymap(keys), rowPins, colPins, SW_KEYPAD_ROWS, SW_KEYPAD_COLS );
	MS_LOGI(MS_DRIVER, "\r\n keypad_sw_setup exit \r\n");
	return STATUS_SUCCESS;
}


TEST_CASE("sw keyscan","test software keyscan ", "[Driver/sw_keyscan]")
{
     MS_LOGI(MS_DRIVER, "\r\n test software keyscan entry...... \r\n");
	 TEST_ASSERT_EQUAL_INT32(STATUS_SUCCESS, keypad_sw_setup());  
	 MS_LOGI(MS_DRIVER, "\r\n test software keyscan running...... \r\n");

}


