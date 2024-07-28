/*
||
|| @file Keypad.h
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

#ifndef __MS_SW_KEYPAD_H
#define __MS_SW_KEYPAD_H

#include <stddef.h>
#include "stdbool.h"

#include "ms_pinmux_ll.h"
#include "ms_gpio.h"
//#include "ms_sys_ctrl.h"


#include "ms_sw_Key.h"


#define SW_KEYPAD_KEY_ROW1 PAD17

#define SW_KEYPAD_KEY_ROW2 PAD18
#define SW_KEYPAD_KEY_ROW3 PAD19
#define SW_KEYPAD_KEY_ROW4 PAD20
#define SW_KEYPAD_KEY_ROW5 PAD21




#define SW_KEYPAD_KEY_COL1 PAD1
#define SW_KEYPAD_KEY_COL2 PAD2

#define SW_KEYPAD_KEY_COL3 PAD3
#define SW_KEYPAD_KEY_COL4 PAD4
#define SW_KEYPAD_KEY_COL5 PAD5
#define SW_KEYPAD_KEY_COL6 PAD6

#define SW_KEYPAD_ROWS 5
#define SW_KEYPAD_COLS 6

/*
#define SW_KEYPAD_KEY_ROW1 PAD1

#define SW_KEYPAD_KEY_ROW2 PAD2
#define SW_KEYPAD_KEY_ROW3 PAD3
#define SW_KEYPAD_KEY_ROW4 PAD4
#define SW_KEYPAD_KEY_ROW5 PAD5
#define SW_KEYPAD_KEY_ROW6 PAD6




#define SW_KEYPAD_KEY_COL1 PAD17
#define SW_KEYPAD_KEY_COL2 PAD18

#define SW_KEYPAD_KEY_COL3 PAD19
#define SW_KEYPAD_KEY_COL4 PAD20
#define SW_KEYPAD_KEY_COL5 PAD21


#define SW_KEYPAD_ROWS 6
#define SW_KEYPAD_COLS 5

*/





#define lowByte(w)  ((uint8_t)((w) & 0xff))   
#define highByte(w) ((uint8_t)((w) >> 8))   

// read bit value, keep bit value, others is set to zero
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

//reset bit value to 1
#define bitSet(value, bit)    ((value) |= (1UL << (bit)))

//clear bit, set bit value to 0
#define bitClear(value,bit)    ((value) &= ~(1UL << (bit)))

//write bit value to 1/0
#define bitWrite(value, bit, bitvalue)    (bitvalue ? bitSet(value, bit) :bitClear(value, bit))


typedef void (*ms_sw_keypad_press_release_callback_func)(T_KEY* key);
extern ms_sw_keypad_press_release_callback_func g_ms_sw_keypad_pressed_release_callback_handler;



//typedef char KeypadEvent;
//typedef unsigned int uint;
//typedef unsigned long ulong;

// Made changes according to this post http://arduino.cc/forum/index.php?topic=58337.0
// by Nick Gammon. Thanks for the input Nick. It actually saved 78 bytes for me. :)
typedef struct {
    uint8_t rows;
    uint8_t columns;
} KeypadSize;

#define LIST_MAX 10		// Max number of keys on the active list.
#define MAPSIZE 10		// MAPSIZE is the number of rows (times 16 columns)
#define makeKeymap(x) ((uint8_t*)x)


typedef struct {
	uint32_t bitMap[MAPSIZE];	// 10 row x 16 column array of bits. Except Due which has 32 columns.
	T_KEY key[LIST_MAX];
	unsigned long holdTimer;
	unsigned long startTime;
	uint8_t *keymap;
    uint8_t *rowPins;
    uint8_t *columnPins;
	KeypadSize sizeKpd;
	int debounceTime;
	int holdTime;
	bool single_key;

} T_SW_Keypad_data_conf;

    //class Keypad : public Key, public HAL_obj {

	void ms_sw_keypad_gpio_config(T_SW_Keypad_data_conf * keypad_data);


	void ms_sw_Keypad_init(uint8_t *userKeymap, uint8_t *row, uint8_t *col, uint8_t numRows, uint8_t numCols);

	//virtual void pin_mode(uint8_t pinNum, byte mode) { pinMode(pinNum, mode); }
	//virtual void pin_write(uint8_t pinNum, boolean level) { digitalWrite(pinNum, level); }
	//virtual int  pin_read(uint8_t pinNum) { return digitalRead(pinNum); }



	uint8_t ms_sw_Keypad_getKey();
	bool ms_sw_Keypad_getKeys();
	KeyState ms_sw_Keypad_getState();
	void ms_sw_Keypad_begin(uint8_t *userKeymap);
	bool ms_sw_Keypad_isPressed(char keyChar);
	void ms_sw_Keypad_setDebounceTime(int debounce);
	void ms_sw_Keypad_setHoldTime(int hold);
	// ms_sw_Keypad_addEventListener(void (*listener)(char));
	int ms_sw_Keypad_findInList_keychar(char keyChar);
	int ms_sw_Keypad_findInList(int keyCode);
	uint8_t ms_sw_Keypad_waitForKey();
	bool ms_sw_Keypad_keyStateChanged();
	//uint8_t ms_sw_Keypad_numKeys();



	void ms_sw_Keypad_scanKeys();
	bool ms_sw_Keypad_updateList();
	void ms_sw_Keypad_nextKeyState(uint8_t idx, bool button);
	void ms_sw_Keypad_transitionTo(uint8_t idx, KeyState nextState);
	void ms_sw_keypad_gpio_irq_handler(void * arg);
	//void (*keypadEventListener)(char);

    

#endif

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
|| | 1.8 2011-11-21 - Mark Stanley     : Added test to determine which header file to compile,
|| |                                          WProgram.h or Arduino.h.
|| | 1.8 2009-07-08 - Alexander Brevig : No longer uses arrays
|| | 1.7 2009-06-18 - Alexander Brevig : This library is a Finite State Machine every time a state changes
|| |                                          the keypadEventListener will trigger, if set
|| | 1.7 2009-06-18 - Alexander Brevig : Added setDebounceTime setHoldTime specifies the amount of
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
