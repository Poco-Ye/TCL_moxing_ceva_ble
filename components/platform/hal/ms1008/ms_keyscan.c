/**
 * Copyright © 2022 by MooreSilicon.All rights reserved
 * @file  ms_keyscan.c
 * @brief
 * @author pengyu.xue
 * @date 2022-2-22
 * @version 1.0
 * @Revision
 */

#include "log.h"
#include "ms_keyscan.h"
#include "ms_keyscan_hal.h"

app_keypad_msg_cb keypad_callback = 0;

KeypadGlobalData_Type g_keypad_data;
MultKeyData_Type g_multikey;

static void ms_keyscan_gpio_init(void);
static void ms_keypad_readfifo(void);
static uint32_t ms_get_keyValue(uint8_t coloun,  uint8_t row);
static uint32_t ms_keypad_multikey_pprocess(uint32_t value, uint8_t type);

extern void board_keypad_gpio_init();
extern uint32_t  board_get_keyValue(uint8_t column, uint8_t row);
extern uint32_t board_get_combinedkey(uint32_t firstkeyvalue, uint32_t secondkeyvalue);


/**
 * @brief This function init keyscan with default parameters and call back.
 * @param  app regiser call back function, will be called when a key is released
 *    
 * @retval none
 */
void  ms_keyscan_init(app_keypad_msg_cb func)
{

    MS_LOGI(MS_DRIVER, " keyscan init \r\n");

    ms_keyscan_gpio_init();
    ms_keyscan_hal_enable_clk();
    ms_keyscan_hal_enable_cpu_int();
	
    ms_keyscan_hal_disable_long_press(); // disable long press key
	ms_keyscan_hal_set_long_press_time(0x4000); /* default value 0x800 */

    ms_keyscan_hal_enable_multi_key();
	ms_keyscan_hal_enable_debounce();
    ms_keyscan_hal_enable_pushkd();
	
	ms_clock_hal_set_keyscan_div( 0x180); /* temp, tbd*/

	ms_keyscan_hal_set_fifo_ctrl(0x0);  /* keep key value in fifo when key release*/

    ms_keyscan_hal_enable_module(); //keyscan register ox18, enable key module
    ms_keyscan_hal_set_debounce_length(0x40);
    ms_keyscan_hal_set_debonce_timeout(0x80); /* default value 0x40 */

	g_keypad_data.cur_fifo_data.key_colum = 0xff;
	g_keypad_data.cur_fifo_data.key_row = 0xff;
	g_keypad_data.cur_fifo_data.long_press_event = g_keypad_data.cur_fifo_data.press_event = 0;
	g_keypad_data.pre_fifo_data.key_colum = 0xff;
	g_keypad_data.pre_fifo_data.key_row = 0xff;	
    g_keypad_data.pre_fifo_data.long_press_event = g_keypad_data.pre_fifo_data.press_event = 0;	
/**
 *			  KEYSCAN_INT_MASK_KEY_PRESS
 *			  KEYSCAN_INT_MASK_KEY_RELEASE
 *            KEYSCAN_INT_MASK_KEY_REPEAT
 *            KEYSCAN_INT_MASK_KEY_LONG_PRESS		
 *            KEYSCAN_INT_MASK_KEY_HOLD_TIMEOUT		
 *            KEYSCAN_INT_MASK_KEY_FIFO_FULL	
 */
  //  ms_keyscan_hal_set_interrupt_unmask(KEYSCAN_INT_MASK_KEY_PRESS | KEYSCAN_INT_MASK_KEY_RELEASE | KEYSCAN_INT_MASK_KEY_LONG_PRESS);

	ms_keyscan_hal_set_interrupt_unmask(KEYSCAN_INT_MASK_KEY_PRESS | KEYSCAN_INT_MASK_KEY_RELEASE);

 	MS_LOGI(MS_DRIVER, "control reg %d \r\n", KEYSCAN->CTRL);

    ms_keyscan_config(); // other config parameters

	g_multikey.curKeyValue = g_multikey.preKeyValue = 0xffff;
	g_multikey.is_Multikey = false;
	
	keypad_callback = func;
    return;
}

/**
 * @brief  This function is used to enable/disable keyscan interrupt.
 *         default status in keyscan init is enable
 * @param  flag    true:  enable keyscan interrupt  
 *                 false: disable keyscan interrupt  
 * @retval none
 */
void ms_keyscan_intrrupt_enable(bool flag)
{
    if(flag == true)
    {
        ms_keyscan_hal_enable_cpu_int();
    }
	else
    {
        ms_keyscan_hal_disable_cpu_int();   
    }
	
    return;
}


/**
 * @brief  This function is used to enable/disable HW debouce.
 *         default status in keyscan init is enable
 * @param  flag    true:  enable HW debouce  
 *                 false: disable HW debouce  
 * @retval none
 */
void ms_HWdebouce_enable(bool flag)
{
    if(flag == true)
    {
        ms_keyscan_ll_enable_debounce();
    }
	else
    {
        ms_keyscan_ll_disable_debounce();
    }
	
    return;
}


/**
 * @brief  This function is used to configure keyscan module.
 *         default status in keyscan init is enable
 * @retval none
 */
void ms_keyscan_config()
{

	ms_keyscan_hal_set_scan_freq(KEYSCAN_SCAN_FREQ_4_CYCLE);
	
    return;
}



void KEYSCAN_IRQHandler()
{

    uint32_t key_int_src;
	uint32_t keyvalue = 0xffff;
    uint32_t combinekey = 0xffff;

	ECLIC_DisableIRQ(KEYSCAN_IRQn);

    key_int_src = ms_keyscan_hal_get_int_src();
    //MS_LOGI(MS_DRIVER, "intsrc %d\r\n", key_int_src);
 
    ms_keypad_readfifo();

	if(ms_keyscan_hal_int_is_key_press(key_int_src) == true)
	{
       // MS_LOGI(MS_DRIVER, "key press coloum: %d, rom: %d\r\n", g_keypad_data.cur_fifo_data.key_colum, g_keypad_data.cur_fifo_data.key_row);
		        //send keyvalue to app_task
		keyvalue = ms_get_keyValue(g_keypad_data.cur_fifo_data.key_colum, g_keypad_data.cur_fifo_data.key_row);
        MS_LOGI(MS_DRIVER, "key press value %d\r\n", keyvalue);
        if(keyvalue == 0xffff)
        {
            MS_LOGI(MS_DRIVER, "keyvalue error\r\n", keyvalue);
        }
            // put the key pressed into event queue
            
        combinekey = ms_keypad_multikey_pprocess(keyvalue, KEYSCAN_INT_SRC_KEY_PRESS);
        if(combinekey != 0xffff)
        {
            MS_LOGI(MS_DRIVER, "combine key value: %d\r\n", combinekey);
        }
			// put the combine key into event queue   	
	}
    if(ms_keyscan_hal_int_is_key_release(key_int_src) == true)
    {
	    //MS_LOGI(MS_DRIVER, " key release coloum: %d, rom: %d\r\n", g_keypad_data.cur_fifo_data.key_colum, g_keypad_data.cur_fifo_data.key_row);

		keyvalue = ms_get_keyValue(g_keypad_data.cur_fifo_data.key_colum, g_keypad_data.cur_fifo_data.key_row);
		MS_LOGI(MS_DRIVER, "key release value %d\r\n", keyvalue);
		ms_keypad_multikey_pprocess(keyvalue, KEYSCAN_INT_SRC_KEY_RELEASE);
        if(keypad_callback)
        {    
            keypad_callback(keyvalue, KEYSCAN_INT_SRC_KEY_RELEASE);
        }			
       // combinekey = ms_keypad_multikey_pprocess(g_multikey, keyvalue, KEYSCAN_INT_SRC_KEY_RELEASE);     
           
       // put the key release into the event queue    
    }

	if(ms_keyscan_hal_int_is_key_long_press(key_int_src) == true)
	{
        
 		MS_LOGI(MS_DRIVER, "get key long press coloum: %d, rom: %d\r\n", g_keypad_data.cur_fifo_data.key_colum, g_keypad_data.cur_fifo_data.key_row);
		MS_LOGI(MS_DRIVER, "get key long press short: %d, long: %d\r\n", g_keypad_data.cur_fifo_data.press_event,g_keypad_data.cur_fifo_data.long_press_event);	
		    // put the key release into the event queue       	
	}
/*		
	switch (key_int_src)
    {
		case KEYSCAN_INT_SRC_KEY_PRESS:
		 	MS_LOGI(MS_DRIVER, "get key press coloum: %d, rom: %d\r\n", g_keypad_data.cur_fifo_data.key_colum, g_keypad_data.cur_fifo_data.key_row);
	        MS_LOGI(MS_DRIVER, "get key press short: %d, long: %d\r\n", g_keypad_data.cur_fifo_data.press_event,g_keypad_data.cur_fifo_data.long_press_event);	
		        //send keyvalue to app_task
		    keyvalue = ms_get_keyValue(g_keypad_data.cur_fifo_data.key_colum, g_keypad_data.cur_fifo_data.key_row);
            MS_LOGI(MS_DRIVER, "keyvalue %d\r\n", keyvalue);
            if(keyvalue == 0xff)
            {
                MS_LOGI(MS_DRIVER, "keyvalue error\r\n", keyvalue);
            }
            // put the key pressed into event queue
            
            combinekey = ms_keypad_multikey_pprocess(g_multikey, keyvalue, KEYSCAN_INT_SRC_KEY_PRESS);
            if(combinekey != 0)
            {
                MS_LOGI(MS_DRIVER, "combine key value: %d\r\n", combinekey);
            }
			// put the combine key into event queue
			
		    break;
		case KEYSCAN_INT_SRC_KEY_RELEASE:
		    MS_LOGI(MS_DRIVER, "key release\r\n");
		 	MS_LOGI(MS_DRIVER, " key release coloum: %d, rom: %d\r\n", g_keypad_data.cur_fifo_data.key_colum, g_keypad_data.cur_fifo_data.key_row);
	        MS_LOGI(MS_DRIVER, " key release short: %d, long: %d\r\n", g_keypad_data.cur_fifo_data.press_event,g_keypad_data.cur_fifo_data.long_press_event);			
            if(keypad_callback)
            {    
                keypad_callback(keyvalue, KEYSCAN_INT_SRC_KEY_RELEASE);
            }			
           // combinekey = ms_keypad_multikey_pprocess(g_multikey, keyvalue, KEYSCAN_INT_SRC_KEY_RELEASE);     
           
            // put the key release into the event queue
            
            break;
        case KEYSCAN_INT_SRC_REP_HIT:
		    MS_LOGI(MS_DRIVER, "get repeated key\r"); 
		    break;
        case KEYSCAN_INT_SRC_LNP_HIT:
		    MS_LOGI(MS_DRIVER, "get key long press coloum: %d, rom: %d\r\n", g_keypad_data.cur_fifo_data.key_colum, g_keypad_data.cur_fifo_data.key_row);
		    MS_LOGI(MS_DRIVER, "get key long press short: %d, long: %d\r\n", g_keypad_data.cur_fifo_data.press_event,g_keypad_data.cur_fifo_data.long_press_event);	
		    // put the key release into the event queue
		    
		    break;
        case KEYSCAN_INT_SRC_KHTOUT:
            MS_LOGI(MS_DRIVER, "key held time out\r\n");
            break;
        case KEYSCAN_INT_SRC_FIFO_FULL:
		    MS_LOGI(MS_DRIVER, "key fifo full\r\n");	
            break;	
        default:
            MS_LOGI(MS_DRIVER, "unknow key int %d\r\n", key_int_src);	
			break;
    }
	*/
    //NVIC_EnableIRQ(KEYSCAN_IRQn);	
    ECLIC_EnableIRQ(KEYSCAN_IRQn);

}


static void ms_keyscan_gpio_init()
{
    board_keypad_gpio_init(); /* call to different evb board function*/
}


/*---------------------------------------------------------------
 * KEYPAD wait fifo not empty 
 * Meed to add break after a certain of time later
 */
void ms_keypad_wait_fifo_not_empty(void) 
{ 
    bool keypad_fifostate;   

    do 
    {
        keypad_fifostate = ms_keyscan_hal_event_is_fifo_ne();
    }
	
    while(keypad_fifostate != true);

    return;
}


static void ms_keypad_readfifo(void)
{	
	
    //wait fifo not empty
  //  ms_keypad_wait_fifo_not_empty();

	// there is a key not release yet
	if( g_keypad_data.cur_fifo_data.key_colum != 0xff && g_keypad_data.cur_fifo_data.long_press_event != 1)  
    {
           g_keypad_data.pre_fifo_data = g_keypad_data.cur_fifo_data;  // copy the pressed key to previous  
    }
	
    // read the new key to current fifo
    g_keypad_data.cur_fifo_data.key_colum = ms_keyscan_hal_get_fifo_rdata_col();
    g_keypad_data.cur_fifo_data.key_row = ms_keyscan_hal_get_fifo_rdata_row();
    g_keypad_data.cur_fifo_data.press_event = ms_keyscan_hal_get_fifo_rdata_keypress();
    g_keypad_data.cur_fifo_data.long_press_event = 	ms_keyscan_hal_get_fifo_rdata_longkey();
			
    return;
	
}




/* accroding to the current key input/ release status, check if it is combined key value*/
static uint32_t ms_keypad_multikey_pprocess(uint32_t value, uint8_t type)
{
    int combinekey = 0xffff;

    if(type == KEYSCAN_INT_SRC_KEY_PRESS)
    {
        if(g_multikey.curKeyValue != 0xffff)  // already a key
        {
            g_multikey.preKeyValue = g_multikey.curKeyValue;
			g_multikey.curKeyValue = value;
			combinekey = board_get_combinedkey( g_multikey.preKeyValue, g_multikey.curKeyValue);
			//MS_LOGI(MS_DRIVER, "combine key %d\r\n", combinekey);	
        }
		else
		{
		    g_multikey.curKeyValue = value;
			if(g_multikey.preKeyValue != 0xffff)
			{
			    combinekey = board_get_combinedkey( g_multikey.preKeyValue, g_multikey.curKeyValue);
			    //MS_LOGI(MS_DRIVER, "combine key %d\r\n", combinekey);  
			}
		}
    }
	else if( type == KEYSCAN_INT_SRC_KEY_RELEASE)
	{
	    if(g_multikey.curKeyValue == value)
	    {
	        g_multikey.curKeyValue = 0xffff;  // reset to none key
	    }
		else if (g_multikey.preKeyValue == value)
		{
		    g_multikey.preKeyValue = g_multikey.curKeyValue;
			g_multikey.curKeyValue = 0xffff;    // wait for next possible combine key
		}
		else
		{
		    MS_LOGI(MS_DRIVER, "error. key release:%d, curkey: %d, prekey %d\r\n", value, g_multikey.curKeyValue, g_multikey.preKeyValue);	
			g_multikey.preKeyValue = g_multikey.curKeyValue = 0xffff;
		}
	}
    
     return combinekey;
}

static uint32_t ms_get_keyValue(uint8_t coloum,  uint8_t row)
{
	uint32_t keyvalue = 0xffff;
	
    keyvalue = board_get_keyValue(coloum, row);
	return keyvalue;
}



