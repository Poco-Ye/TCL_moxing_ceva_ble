/*
||
|| @file Keypad.cpp
|| @version 3.1
|| @author Mark Stanley, Alexander Brevig
|| @contact mstanley@technologist.com, alexanderbrevig@gmail.com
||
|| @description
|| | This library provides a simple interface for using matrix
|| | keypads. It supports multiple keypresses while maintaining
|| | backwards compatibility with the old single key library.
|| | It also supports user selectable pins and definable keymaps.
|| #
||
|| @license
|| | This library is free software; you can redistribute it and/or
|| | modify it under the terms of the GNU Lesser General Public
|| | License as published by the Free Software Foundation; version
|| | 2.1 of the License.
|| |
|| | This library is distributed in the hope that it will be useful,
|| | but WITHOUT ANY WARRANTY; without even the implied warranty of
|| | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
|| | Lesser General Public License for more details.
|| |
|| | You should have received a copy of the GNU Lesser General Public
|| | License along with this library; if not, write to the Free Software
|| | Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
|| #
||
*/

#include "ms_sw_Keypad.h"

#include "gpio.h"

#include <ms_clock_hal.h>
#include "ms_gpio.h"
#include <stddef.h>
#include "ms_pinmux_hal.h"
#include "log.h"
#include "timer.h"

//ms_gpio_dev_t g_ms_gpio_row[SW_KEYPAD_ROWS] ;
//ms_gpio_dev_t g_ms_gpio_col[SW_KEYPAD_COLS];


T_SW_Keypad_data_conf g_sw_keypad_conf = {0};
ms_sw_keypad_press_release_callback_func g_ms_sw_keypad_pressed_release_callback_handler = NULL;




// <<constructor>> Allows custom keymap, pin configuration, and keypad sizes.
//Keypad::Keypad(char *userKeymap, byte *row, byte *col, byte numRows, byte numCols) 
void ms_sw_keypad_gpio_config(T_SW_Keypad_data_conf * keypad_data)
{
    uint8_t i = 0;
	uint8_t j = 0;

	if(NULL == keypad_data)
	{
	    MS_LOGI(MS_DRIVER,"error keypad_data \r\n");
	    return ;
	}
	
    for(i = 0; i < SW_KEYPAD_ROWS; i++)
    {
	    //gpio input
	    /*
	    g_ms_gpio_row[i].port = keypad_data->rowPins[i];
	    g_ms_gpio_row[i].direction = GPIO_DIR_OUTPUT;
	    
	    g_ms_gpio_row[i].priv = NULL;
	    ms_gpio_init(&g_ms_gpio_row[i]);
		ms_gpio_output_low_pro(g_ms_gpio_row[i].port);
		*/
		
		gpio_out_init(keypad_data->rowPins[i]);
		gpio_out_put_low_level(keypad_data->rowPins[i]);
		
		//ATINY_LOG(LOG_INFO,"fun:%s , g_ms_gpio_row[%d].port = %d \r\n",__FUNCTION__, i, g_ms_gpio_row[i].port); 
    }

	
	
    for(j = 0; j < SW_KEYPAD_COLS; j++)
    {
    
	    /*
	    g_ms_gpio_col[j].port = keypad_data->columnPins[j];
	    g_ms_gpio_col[j].direction = GPIO_DIR_INPUT;
		g_ms_gpio_col[j].interrupt_type = GPIO_INT_EDGE_BOTH;
	    g_ms_gpio_col[j].priv = NULL;
	    ms_gpio_init(&g_ms_gpio_col[j]);
		
		 ms_gpio_config_irq(&g_ms_gpio_col[j], GPIO_INT_EDGE_BOTH, ms_sw_keypad_gpio_irq_handler, NULL);  
	    ms_gpio_enable_irq(&g_ms_gpio_col[j]);
		ms_pad_config( g_ms_gpio_col[j].port, PULL_UP);  
		*/
		
		gpio_falling_edge_interrput_init(keypad_data->columnPins[j]);
		ms_pinmux_hal_config_pull_type(keypad_data->columnPins[j],PULL_UP);
		
		//MS_LOGI(MS_DRIVER,"fun:%s , g_ms_gpio_col[%d].port = %d \r\n",__FUNCTION__, j, g_ms_gpio_col[j].port); 
    }

    
		
}

/*
void ms_sw_keypad_gpio_config(T_SW_Keypad_data_conf * keypad_data)
{
    uint8_t i = 0;
	uint8_t j = 0;

	if(NULL == keypad_data)
	{
	    return ;
	}
	
    for(i = 0; i < ROWS; i++)
    {
	    //gpio input
	    g_ms_gpio_row[i].port = keypad_data->rowPins[i];
	    g_ms_gpio_row[i].direction = GPIO_DIR_INPUT;
	    g_ms_gpio_row[i].interrupt_type = GPIO_INT_EDGE_BOTH;
	    g_ms_gpio_row[i].priv = NULL;
	    ms_gpio_init(&g_ms_gpio_row[i]);
	    ms_gpio_config_irq(&g_ms_gpio_row[i], GPIO_INT_EDGE_BOTH, ms_sw_keypad_gpio_irq_handler, NULL);  
	    ms_gpio_enable_irq(&g_ms_gpio_row[i]);
    }
	
    for(j = 0; j < COLS; j++)
    {
	    //gpio input
	    g_ms_gpio_col[j].port = colPins[j];
	    g_ms_gpio_col[j].direction = GPIO_DIR_OUTPUT;
	    g_ms_gpio_col[j].priv = NULL;
	    ms_gpio_init(&g_ms_gpio_col[j]);
		ms_gpio_output_low(&g_ms_gpio_col[j]); 
    }
    
		
}

*/



void ms_sw_keypad__Timer_Handle(void *arg)
{
    return;
}

void ms_sw_Keypad_init(uint8_t *userKeymap, uint8_t *row, uint8_t *col, uint8_t numRows, uint8_t numCols) 
{
	g_sw_keypad_conf.rowPins = row;
	g_sw_keypad_conf.columnPins = col;
	g_sw_keypad_conf.sizeKpd.rows = numRows;
	g_sw_keypad_conf.sizeKpd.columns = numCols;

	for(uint8_t i = 0; i < LIST_MAX; i++)
	{
	    MS_LOGI(MS_DRIVER," key[%d]:%d, %d, %d, %d \r\n", i,g_sw_keypad_conf.key[i].kchar,g_sw_keypad_conf.key[i].kcode,g_sw_keypad_conf.key[i].kstate,g_sw_keypad_conf.key[i].stateChanged);
	    
	}

	ms_sw_keypad_gpio_config(&g_sw_keypad_conf);

	//ms_set_gpio_interrupt_debounce_enable(GPIO_BIT_ALL);

	ms_sw_Keypad_begin(userKeymap);

	ms_sw_Keypad_setDebounceTime(1);
	ms_sw_Keypad_setHoldTime(500);
	//keypadEventListener = 0;

	g_sw_keypad_conf.startTime = 0;
	g_sw_keypad_conf.single_key = false;



    timer1_init();

	
}

// Let the user define a keymap - assume the same row/column count as defined in constructor
//void Keypad::begin(char *userKeymap) 
void ms_sw_Keypad_begin(uint8_t *userKeymap) 

{
    g_sw_keypad_conf.keymap = userKeymap;
}

// Returns a single key only. Retained for backwards compatibility.
uint8_t ms_sw_Keypad_getKey() 
{
	g_sw_keypad_conf.single_key = true;
	MS_LOGI(MS_DRIVER,"  ms_sw_Keypad_getKey \r\n");

	//if (ms_sw_Keypad_getKeys() && g_sw_keypad_conf.key[0].stateChanged && (g_sw_keypad_conf.key[0].kstate==PRESSED))
	if(ms_sw_Keypad_getKeys() && (g_sw_keypad_conf.key[0].stateChanged 
		|| g_sw_keypad_conf.key[1].stateChanged
		|| g_sw_keypad_conf.key[2].stateChanged) )
	{
	    if(g_sw_keypad_conf.key[0].kchar)
	    {
	        MS_LOGI(MS_DRIVER,"key0 = 0x%x \r\n",g_sw_keypad_conf.key[0].kchar);
	    }
		
		if(g_sw_keypad_conf.key[1].kchar)
	    {
	        MS_LOGI(MS_DRIVER,"key1 = 0x%x \r\n",g_sw_keypad_conf.key[1].kchar);
	    }
		
		if(g_sw_keypad_conf.key[2].kchar)
	    {
	        MS_LOGI(MS_DRIVER," key2 = 0x%x \r\n",g_sw_keypad_conf.key[2].kchar);
	    }
	    

		return g_sw_keypad_conf.key[0].kchar;
	}
	
	g_sw_keypad_conf.single_key = false;

	return NO_KEY;
}

// Populate the key list.
bool ms_sw_Keypad_getKeys() 
{
	bool keyActivity = false;
	int32_t tTime0 = 0;
	int32_t tTime1 = 0;
    int32_t tTime2 = 0;
	// Limit how often the keypad is scanned. This makes the loop() run 10 times as fast.
	//if ( (millis()-startTime)>debounceTime ) 
	
    timer1_relaod_value(0x20000000);
    timer1_start();
	tTime0 = timer1_get_curvalue();
	
	delay(g_sw_keypad_conf.debounceTime * 1000);


	tTime1 = timer1_get_curvalue();

	{
		ms_sw_Keypad_scanKeys();
		keyActivity = ms_sw_Keypad_updateList();
		//startTime = millis();
	}

	tTime2 = timer1_get_curvalue();

	MS_LOGI(MS_DRIVER,"tTime0 = %d, t1 = %d ,t2 = %d, t1-t2 = %d , t0 -t2 = %d \r\n",tTime0, tTime1,tTime2, (tTime1 -tTime2),(tTime0 -tTime2));

	return keyActivity;
}


//--------------------------------------------------------------------
// Copyright (c) 2022 by MooreSilicon.
// All rights reserved.
// MooreSilicon Confidential Proprietary.
//--------------------------------------------------------------------
//     Function    : void ms_sw_Keypad_scanKeys()  
//     Author      : Ethan Sun
//     Dates       : 2022-04-1
//     Version     : V1.0
//-------------------------------------------------------------------
/* 
 method way for scankeys
{
	row[4:0] = output enable = 0;  //close output , set gpio input pull up;
		{
		col = 1,2,3,4,5,6
		col[5:0] = input ;
		col[5:0] = pull up;
		}
	
	
		row[0] = output 0;
		if(col[x])==0  >> key��6 col check)
		delay;
		row[0] = pull up;
		row[0] ������Ϊ���루����ǵ��л�����
		row dir = output
		row[1] = output 0;
		������

}
*/

//-------------------------------------------------------------------


// Private : Hardware scan
void ms_sw_Keypad_scanKeys() 
{
	// Re-intialize the row pins. Allows sharing these pins with other hardware.
	for (uint8_t c = 0; c< g_sw_keypad_conf.sizeKpd.columns; c++) 
	{
		//pin_mode(rowPins[r],INPUT_PULLUP);
		//ATINY_LOG(LOG_INFO," c = %d, g_sw_keypad_conf.columnPins[c] = %d \r\n", c, g_sw_keypad_conf.columnPins[c]);	
/*
		ms_sw_keypad_gpio_port_config(g_sw_keypad_conf.columnPins[c],GPIO_DIR_INPUT);
		ms_pad_config( g_sw_keypad_conf.columnPins[c], PULL_UP);  
*/
				
		gpio_falling_edge_interrput_init(g_sw_keypad_conf.columnPins[c]);
		ms_pinmux_hal_config_pull_type(g_sw_keypad_conf.columnPins[c],PULL_UP);
	}

    
	for (uint8_t r = 0; r< g_sw_keypad_conf.sizeKpd.rows; r++) 
	{
/*
		ms_sw_keypad_gpio_port_config(g_sw_keypad_conf.rowPins[r],GPIO_DIR_INPUT);
		ms_pad_config( g_sw_keypad_conf.rowPins[r], PULL_UP); 
*/
		
		gpio_input_init( g_sw_keypad_conf.rowPins[r]);
		ms_pinmux_hal_config_pull_type( g_sw_keypad_conf.rowPins[r],PULL_UP);
	}


	// bitMap stores ALL the keys that are being pressed.
	for (uint8_t r=0; r < g_sw_keypad_conf.sizeKpd.rows; r++) 
	{
		/*
		ms_sw_keypad_gpio_port_config(g_sw_keypad_conf.rowPins[r],GPIO_DIR_OUTPUT);
		ms_gpio_output_low_pro(g_sw_keypad_conf.rowPins[r]);
		*/

		gpio_out_init(g_sw_keypad_conf.rowPins[r]);
		gpio_out_put_low_level(g_sw_keypad_conf.rowPins[r]);
		

		
		
		for (uint8_t c=0; c < g_sw_keypad_conf.sizeKpd.columns; c++) 
		{
			//bitWrite(bitMap[r], c, !pin_read(rowPins[r]));  // keypress is active low so invert to high.
			//int32_t gpioValue = ms_gpio_input_get_pro(g_sw_keypad_conf.columnPins[c]);
			int32_t gpioValue = gpio_get_value(g_sw_keypad_conf.columnPins[c]);
			if(!gpioValue)
			{
			    MS_LOGI(MS_DRIVER,"c = %d, r = %d, gpioValue = %d \r\n",c, r, gpioValue );	
			}

			bitWrite(g_sw_keypad_conf.bitMap[c], r,!gpioValue);  // keypress is active low so invert to high.
			//bitWrite(g_sw_keypad_conf.bitMap[r], c, !ms_gpio_input_get_pro(g_sw_keypad_conf.rowPins[r]));  // keypress is active low so invert to high.
		}

		/*
		ms_sw_keypad_gpio_port_config(g_sw_keypad_conf.rowPins[r],GPIO_DIR_INPUT);
		ms_pad_config( g_sw_keypad_conf.rowPins[r], PULL_UP); 
		*/
		gpio_input_init(g_sw_keypad_conf.rowPins[r]);
		ms_pinmux_hal_config_pull_type( g_sw_keypad_conf.rowPins[r],PULL_UP);

	}
	
	for (uint8_t r = 0; r< g_sw_keypad_conf.sizeKpd.rows; r++) 
	{
	    /*
	    ms_sw_keypad_gpio_port_config(g_sw_keypad_conf.rowPins[r],GPIO_DIR_OUTPUT);
		ms_gpio_output_low_pro(g_sw_keypad_conf.rowPins[r]);
		*/

		gpio_out_init(g_sw_keypad_conf.rowPins[r]);
		gpio_out_put_low_level(g_sw_keypad_conf.rowPins[r]);

	}
}
	

// Manage the list without rearranging the keys. Returns true if any keys on the list changed state.
bool ms_sw_Keypad_updateList() 
{

	bool anyActivity = false;

	// Delete any IDLE keys
	for (uint8_t i=0; i<LIST_MAX; i++) 
	{
		if (g_sw_keypad_conf.key[i].kstate==IDLE) {
			g_sw_keypad_conf.key[i].kchar = NO_KEY;
			g_sw_keypad_conf.key[i].kcode = -1;
			g_sw_keypad_conf.key[i].stateChanged = false;
		}
	}

	// Add new keys to empty slots in the key list.
	for (uint8_t c=0; c<g_sw_keypad_conf.sizeKpd.columns; c++) 
	{
		for (uint8_t r=0; r<g_sw_keypad_conf.sizeKpd.rows; r++) 
		{
			bool    button = bitRead(g_sw_keypad_conf.bitMap[c],r);
			uint8_t keyChar = g_sw_keypad_conf.keymap[c * g_sw_keypad_conf.sizeKpd.rows + r];
			int keyCode = c * g_sw_keypad_conf.sizeKpd.rows + r;
			int idx = ms_sw_Keypad_findInList (keyCode);
			//ATINY_LOG(LOG_INFO,"r = %d, c = %d, idx = %d, button = %d , keyChar = 0x%x  , keyCode = %d \r\n",r, c, idx ,button, keyChar, keyCode );	
			
			// Key is already on the list so set its next state.
			if (idx > -1)	
			{
			    MS_LOGI(MS_DRIVER,"c = %d, r = %d, idx = %d, button = %d   \r\n",c, r, idx ,button );	
			    
				ms_sw_Keypad_nextKeyState(idx, button);
			}
			// Key is NOT on the list so add it.
			if ((idx == -1) && button) 
			{
			    MS_LOGI(MS_DRIVER,"c = %d, r = %d, idx = %d, button = %d   \r\n",c, r, idx ,button );	
				for (uint8_t i=0; i<LIST_MAX; i++) 
				{
					if (g_sw_keypad_conf.key[i].kchar==NO_KEY) 
					{		// Find an empty slot or don't add key to list.
					    MS_LOGI(MS_DRIVER,"Find an empty slot or don't add key to list  \r\n");	
						g_sw_keypad_conf.key[i].kchar = keyChar;
						g_sw_keypad_conf.key[i].kcode = keyCode;
						g_sw_keypad_conf.key[i].kstate = IDLE;		// Keys NOT on the list have an initial state of IDLE.
						ms_sw_Keypad_nextKeyState (i, button);
						
						break;	// Don't fill all the empty slots with the same key.
					}
				}
			}
		}
	}

	// Report if the user changed the state of any key.
	for (uint8_t i=0; i<LIST_MAX; i++) 
	{
		if (g_sw_keypad_conf.key[i].stateChanged) 
			anyActivity = true;
	}

	return anyActivity;
}

// Private
// This function is a state machine but is also used for debouncing the keys.
void ms_sw_Keypad_nextKeyState(uint8_t idx, bool button) 
{
	g_sw_keypad_conf.key[idx].stateChanged = false;

	if(button)
	{
	    
	    MS_LOGI(MS_DRIVER,"idx = %d, button = CLOSED  , g_sw_keypad_conf.key[idx].kstate = %d \r\n",idx  , g_sw_keypad_conf.key[idx].kstate);	
	}
	else
	{
	    MS_LOGI(MS_DRIVER,"idx = %d, button = OPEN  , g_sw_keypad_conf.key[idx].kstate = %d \r\n",idx  , g_sw_keypad_conf.key[idx].kstate);
	}

	

	switch (g_sw_keypad_conf.key[idx].kstate) 
	{
		case IDLE:
			if (button==CLOSED) 
			{
				ms_sw_Keypad_transitionTo(idx, PRESSED);
				//g_sw_keypad_conf.holdTimer = millis(); }		// Get ready for next HOLD state.
			}
			break;
		case PRESSED:
			//if ((millis()-g_sw_keypad_conf.holdTimer)>g_sw_keypad_conf.holdTime)	// Waiting for a key HOLD...
				ms_sw_Keypad_transitionTo(idx, HOLD);
			//else if (button==OPEN)				// or for a key to be RELEASED.
			    if (button==OPEN)
			    {
				    ms_sw_Keypad_transitionTo(idx, RELEASED);
			    }
			break;
		case HOLD:
			if (button==OPEN)
				ms_sw_Keypad_transitionTo(idx, RELEASED);
			break;
		case RELEASED:
			ms_sw_Keypad_transitionTo(idx, IDLE);
			break;
		default:
			break;
	}
}

// New in 2.1
bool ms_sw_Keypad_isPressed(char keyChar) 
{
	for (uint8_t i=0; i<LIST_MAX; i++) 
	{
		if ( g_sw_keypad_conf.key[i].kchar == keyChar ) 
		{
			if ( (g_sw_keypad_conf.key[i].kstate == PRESSED) && g_sw_keypad_conf.key[i].stateChanged )
			{
				return true;
			}
		}
	}
	return false;	// Not pressed.
}

// Search by character for a key in the list of active keys.
// Returns -1 if not found or the index into the list of active keys.
int ms_sw_Keypad_findInList_keychar (char keyChar) 
{
	for (uint8_t i=0; i<LIST_MAX; i++) 
	{
		if (g_sw_keypad_conf.key[i].kchar == keyChar) 
		{
			return i;
		}
	}
	return -1;
}

// Search by code for a key in the list of active keys.
// Returns -1 if not found or the index into the list of active keys.
int ms_sw_Keypad_findInList (int keyCode) 
{
	for (uint8_t i=0; i<LIST_MAX; i++) 
	{
		if (g_sw_keypad_conf.key[i].kcode == keyCode) 
		{
			return i;
		}
	}
	return -1;
}

// New in 2.0
uint8_t ms_sw_Keypad_waitForKey() 
{
	uint8_t waitKey = NO_KEY;
	while( (waitKey = ms_sw_Keypad_getKey()) == NO_KEY );	// Block everything while waiting for a keypress.

	
	
	return waitKey;
}

// Backwards compatibility function.
KeyState ms_sw_Keypad_getState() 
{
	return g_sw_keypad_conf.key[0].kstate;
}

// The end user can test for any changes in state before deciding
// if any variables, etc. needs to be updated in their code.
bool ms_sw_Keypad_keyStateChanged() 
{
	return g_sw_keypad_conf.key[0].stateChanged;
}

// The number of keys on the key list, key[LIST_MAX], equals the number
// of bytes in the key list divided by the number of bytes in a Key object.

/*
uint8_t ms_sw_Keypad_numKeys() 
{
	return sizeof(g_sw_keypad_conf.key)/sizeof(T_KEY);
}*/

// Minimum debounceTime is 1 mS. Any lower *will* slow down the loop().
void ms_sw_Keypad_setDebounceTime(int debounce)
{
	(debounce <1) ? (g_sw_keypad_conf.debounceTime=1) : (g_sw_keypad_conf.debounceTime=debounce);
}

void ms_sw_Keypad_setHoldTime(int hold)
{
    g_sw_keypad_conf.holdTime = hold;
}

/*
void ms_sw_Keypad_addEventListener(void (*listener)(char))
{
	keypadEventListener = listener;
}*/

void ms_sw_Keypad_transitionTo(uint8_t idx, KeyState nextState) 
{
	g_sw_keypad_conf.key[idx].kstate = nextState;
	g_sw_keypad_conf.key[idx].stateChanged = true;

	MS_LOGI(MS_DRIVER,"g_sw_keypad_conf.key[%d].kstate = %d \r\n",idx,  g_sw_keypad_conf.key[idx].kstate);

	// Sketch used the getKey() function.
	// Calls keypadEventListener only when the first key in slot 0 changes state.
	/*
	if (g_sw_keypad_conf.single_key)  {
	  	if ( (keypadEventListener!=NULL) && (idx==0) )  {
			keypadEventListener(g_sw_keypad_conf.key[0].kchar);
		}
	}
	// Sketch used the getKeys() function.
	// Calls keypadEventListener on any key that changes state.
	else {
	  	if (keypadEventListener!=NULL)  {
			keypadEventListener(g_sw_keypad_conf.key[idx].kchar);
		}
	}*/
}

//void ms_sw_keypad_gpio_irq_handler(void * arg)

void ms_sw_keypad_gpio_irq_handler(void * arg)
{
    char waitKey;
	MS_LOGI(MS_DRIVER," ms_sw_keypad_gpio_irq_handler  \r\n");	
    waitKey = ms_sw_Keypad_waitForKey();
    MS_LOGI(MS_DRIVER,"waitKey = 0x%x \r\n", waitKey);	
    for(int i=0; i<LIST_MAX; i++)   // Scan the whole key list.
    {
        if(g_sw_keypad_conf.key[i].stateChanged)   // Only find keys that have changed state.
        {
			MS_LOGI(MS_DRIVER," get key ,exit handler i = %d, kstate = %d, kchar =0x%x , kcode = 0x%x \r\n",i, g_sw_keypad_conf.key[i].kstate, g_sw_keypad_conf.key[i].kchar ,g_sw_keypad_conf.key[i].kcode);	
        }
   }


	if(NULL != g_ms_sw_keypad_pressed_release_callback_handler) 
    {
        g_ms_sw_keypad_pressed_release_callback_handler(g_sw_keypad_conf.key);
    }
	
     //reset key bitmap
    if((RELEASED == g_sw_keypad_conf.key[0].kstate) || (RELEASED == g_sw_keypad_conf.key[1].kstate)|| (RELEASED == g_sw_keypad_conf.key[2].kstate))
    {
       MS_LOGI(MS_DRIVER,"fun:%s exit get keys ,clean status \r\n",__FUNCTION__ );
        for(uint32_t i = 0; i < MAPSIZE; i++) 
        {
    	    g_sw_keypad_conf.bitMap[i] = 0;
        }
		
		for(uint8_t i = 0; i < LIST_MAX; i++)
        {
            g_sw_keypad_conf.key[i].kchar  = 0;
			g_sw_keypad_conf.key[i].kcode  = 0;
			g_sw_keypad_conf.key[i].kstate = IDLE;
			g_sw_keypad_conf.key[i].stateChanged = false;;
    
        }
    }
}

/*
|| @changelog
|| | 3.1 2013-01-15 - Mark Stanley     : Fixed missing RELEASED & IDLE status when using a single key.
|| | 3.0 2012-07-12 - Mark Stanley     : Made library multi-keypress by default. (Backwards compatible)
|| | 3.0 2012-07-12 - Mark Stanley     : Modified pin functions to support Keypad_I2C
|| | 3.0 2012-07-12 - Stanley & Young  : Removed static variables. Fix for multiple keypad objects.
|| | 3.0 2012-07-12 - Mark Stanley     : Fixed bug that caused shorted pins when pressing multiple keys.
|| | 2.0 2011-12-29 - Mark Stanley     : Added waitForKey().
|| | 2.0 2011-12-23 - Mark Stanley     : Added the public function keyStateChanged().
|| | 2.0 2011-12-23 - Mark Stanley     : Added the private function scanKeys().
|| | 2.0 2011-12-23 - Mark Stanley     : Moved the Finite State Machine into the function getKeyState().
|| | 2.0 2011-12-23 - Mark Stanley     : Removed the member variable lastUdate. Not needed after rewrite.
|| | 1.8 2011-11-21 - Mark Stanley     : Added decision logic to compile WProgram.h or Arduino.h
|| | 1.8 2009-07-08 - Alexander Brevig : No longer uses arrays
|| | 1.7 2009-06-18 - Alexander Brevig : Every time a state changes the keypadEventListener will trigger, if set.
|| | 1.7 2009-06-18 - Alexander Brevig : Added setDebounceTime. setHoldTime specifies the amount of
|| |                                          microseconds before a HOLD state triggers
|| | 1.7 2009-06-18 - Alexander Brevig : Added transitionTo
|| | 1.6 2009-06-15 - Alexander Brevig : Added getState() and state variable
|| | 1.5 2009-05-19 - Alexander Brevig : Added setHoldTime()
|| | 1.4 2009-05-15 - Alexander Brevig : Added addEventListener
|| | 1.3 2009-05-12 - Alexander Brevig : Added lastUdate, in order to do simple debouncing
|| | 1.2 2009-05-09 - Alexander Brevig : Changed getKey()
|| | 1.1 2009-04-28 - Alexander Brevig : Modified API, and made variables private
|| | 1.0 2007-XX-XX - Mark Stanley : Initial Release
|| #
*/
