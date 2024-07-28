/*
||
|| @file Key.h
|| @version 1.0
|| @author Mark Stanley
|| @contact mstanley@technologist.com
||
|| @description
|| | Key class provides an abstract definition of a key or button
|| | and was initially designed to be used in conjunction with a
|| | state-machine.
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

#ifndef __MS_SW_KEY_H_
#define __MS_SW_KEY_H_
	
#ifdef __cplusplus
	extern "C"{
#endif

#include <ms1008.h>
#include <stddef.h>
#include "stdbool.h"




#define OPEN 0 //LOW
#define CLOSED 1 //HIGH
#define NO_KEY 0x0

    //typedef unsigned int uint;
    typedef enum { IDLE, PRESSED, HOLD, RELEASED } KeyState;




	typedef struct
	{
	    uint8_t kchar;
	    int kcode;
	    KeyState kstate;
	    bool stateChanged;
	} T_KEY;
	

    extern T_KEY g_sw_key;

	// methods
	void ms_sw_Key_init();
	void ms_sw_Key_init_pro(char userKeyChar);
	void ms_sw_key_update(char userKeyChar, KeyState userState, bool userStatus);




/*
|| @changelog
|| | 1.0 2012-06-04 - Mark Stanley : Initial Release
|| #
*/
#ifdef __cplusplus
}
#endif

#endif /* __MS_SW_KEY_H_ */

