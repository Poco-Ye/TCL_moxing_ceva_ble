/*
|| @file Key.cpp
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
#include "ms_sw_Key.h"


// default constructor
void ms_sw_Key_init() 
{
	g_sw_key.kchar = NO_KEY;
	g_sw_key.kstate = IDLE;
	g_sw_key.stateChanged = false;
}

// constructor
void ms_sw_Key_init_pro(char userKeyChar) 
{
	g_sw_key.kchar = userKeyChar;
	g_sw_key.kcode = -1;
	g_sw_key.kstate = IDLE;
	g_sw_key.stateChanged = false;
}


void ms_sw_key_update (char userKeyChar, KeyState userState, bool userStatus) 
{
	g_sw_key.kchar = userKeyChar;
	g_sw_key.kstate = userState;
	g_sw_key.stateChanged = userStatus;
}



/*
|| @changelog
|| | 1.0 2012-06-04 - Mark Stanley : Initial Release
|| #
*/
