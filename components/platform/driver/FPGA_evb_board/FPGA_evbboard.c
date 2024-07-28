/**
 * Copyright  2022 by MooreSilicon.All rights reserved
 * @file  evb borad.c
 * @brief
 * @author pengyu.xue
 * @date 2022-2-22
 * @version 1.0
 * @Revision
 */
#include "ms_pinmux_hal.h"
#include "FPGA_evbboard.h"
#include "ms1008.h"
#include "ms_gpio_ll.h"
#include "ms_gpio_hal.h"

#define KEYPAD_ROW_SIZE      5
#define KEYPAD_COLUMN_SIZE   6

#define KEYPAD_KEY_ROW1 PAD17 
#define KEYPAD_KEY_ROW2 PAD18
#define KEYPAD_KEY_ROW3 PAD19
#define KEYPAD_KEY_ROW4 PAD20
#define KEYPAD_KEY_ROW5 PAD21

#define KEYPAD_KEY_COL1 PAD1
#define KEYPAD_KEY_COL2 PAD2
#define KEYPAD_KEY_COL3 PAD3
#define KEYPAD_KEY_COL4 PAD4
#define KEYPAD_KEY_COL5 PAD5
#define KEYPAD_KEY_COL6 PAD6


#define KEYPAD_KEY_ROW1_MODE PIN17_KEY_ROW1
#define KEYPAD_KEY_ROW2_MODE PIN18_KEY_ROW2
#define KEYPAD_KEY_ROW3_MODE PIN19_KEY_ROW3
#define KEYPAD_KEY_ROW4_MODE PIN20_KEY_ROW4
#define KEYPAD_KEY_ROW5_MODE PIN21_KEY_ROW5

#define KEYPAD_KEY_COL1_MODE PIN01_KEY_COL1
#define KEYPAD_KEY_COL2_MODE PIN02_KEY_COL2
#define KEYPAD_KEY_COL3_MODE PIN03_KEY_COL3
#define KEYPAD_KEY_COL4_MODE PIN04_KEY_COL4
#define KEYPAD_KEY_COL5_MODE PIN05_KEY_COL5
#define KEYPAD_KEY_COL6_MODE PIN06_KEY_COL6



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
#define KB_VOL_DOWN   0xD1
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



void board_keypad_gpio_init(void)
{
   ms_pinmux_hal_set_pinmux(KEYPAD_KEY_ROW1,KEYPAD_KEY_ROW1_MODE);
   ms_pinmux_hal_set_pinmux(KEYPAD_KEY_ROW2,KEYPAD_KEY_ROW2_MODE);
   ms_pinmux_hal_set_pinmux(KEYPAD_KEY_ROW3,KEYPAD_KEY_ROW3_MODE);
   ms_pinmux_hal_set_pinmux(KEYPAD_KEY_ROW4,KEYPAD_KEY_ROW4_MODE);
   ms_pinmux_hal_set_pinmux(KEYPAD_KEY_ROW5,KEYPAD_KEY_ROW5_MODE);

   ms_gpio_interrupt_disen_hal(GPIO, PAD1);
   ms_gpio_interrupt_disen_hal(GPIO, PAD2);
   ms_gpio_interrupt_disen_hal(GPIO, PAD3);
   ms_gpio_interrupt_disen_hal(GPIO, PAD4);
   ms_gpio_interrupt_disen_hal(GPIO, PAD5);
   ms_gpio_interrupt_disen_hal(GPIO, PAD6);

   ms_pinmux_hal_set_pinmux(KEYPAD_KEY_COL1 ,KEYPAD_KEY_COL1_MODE);
   ms_pinmux_hal_set_pinmux(KEYPAD_KEY_COL2 ,KEYPAD_KEY_COL2_MODE);
   ms_pinmux_hal_set_pinmux(KEYPAD_KEY_COL3 ,KEYPAD_KEY_COL3_MODE);
   ms_pinmux_hal_set_pinmux(KEYPAD_KEY_COL4 ,KEYPAD_KEY_COL4_MODE);
   ms_pinmux_hal_set_pinmux(KEYPAD_KEY_COL5 ,KEYPAD_KEY_COL5_MODE);
   ms_pinmux_hal_set_pinmux(KEYPAD_KEY_COL6 ,KEYPAD_KEY_COL6_MODE);


    //io mux sel keypad, GPIO1~05,16,KEY COL 
    //pull up  
    ms_pinmux_hal_config_pull_type( KEYPAD_KEY_ROW1, PULL_UP);    
    ms_pinmux_hal_config_pull_type( KEYPAD_KEY_ROW2, PULL_UP);
    ms_pinmux_hal_config_pull_type( KEYPAD_KEY_ROW3, PULL_UP);
    ms_pinmux_hal_config_pull_type( KEYPAD_KEY_ROW4, PULL_UP);
    ms_pinmux_hal_config_pull_type( KEYPAD_KEY_ROW5, PULL_UP);

}
 


const uint32_t key_map[KEYPAD_ROW_SIZE][KEYPAD_COLUMN_SIZE] =
{
    {KB_0,       KB_MENU,           KB_UP,            KB_HOME,               KB_TV,      KB_YELLOW},
    {KB_5,       KB_6,             KB_9,              KB_4,                   KB_DOWN  ,     KB_5         },
    {KB_2,       KB_3,             KB_UP,             KB_1,                  KB_LEFT,    KB_RIGHT },
    {KB_8,       KB_DOWN ,          KB_OK,            KB_7  ,              KB_PR_UP ,  KB_PR_DOWN},
    {KB_RED,      KB_GREEN,          KB_POWER,      KB_VOICE,              KB_BACK,    KB_OK}
};

#define combinekey1 0x1ff
#define combinekey2 0x2ff
#define MAXCOMBINEKEY 2
const uint32_t combine_key_map[MAXCOMBINEKEY][3] =
{
    {KB_0,       KB_3,         combinekey1},
    {KB_0,       KB_1,         combinekey2}
};

/*
  207 204               
  207 206  
*/


/******************************************************************
 * @brief    Get key value according to key map
 */
uint32_t  board_get_keyValue(uint8_t column, uint8_t row)
{

    uint32_t errCode = 0xffff;
	
    if ((row  >= KEYPAD_ROW_SIZE) ||(row < 0) ||(column >= KEYPAD_COLUMN_SIZE )  ||(column < 0) )
    {
        return errCode ;
    }


   return (uint32_t) key_map[row][column];
}


uint32_t board_get_combinedkey(uint32_t firstkeyvalue, uint32_t secondkeyvalue)
{
    uint8_t index;
    uint32_t combinkey = 0xffff;

    for(index = 0; index < MAXCOMBINEKEY; index++ )
    {
	    if((firstkeyvalue == combine_key_map[index][0]) && (secondkeyvalue == combine_key_map[index][1]))
		{
		    combinkey =  combine_key_map[index][2];
			break;
	    }
    }

    return combinkey;
}



