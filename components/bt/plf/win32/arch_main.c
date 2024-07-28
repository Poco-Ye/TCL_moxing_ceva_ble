
#ifdef WIN32

#include <stddef.h>    // standard definitions

#include <stdio.h>     // io definitions
#include <string.h>    // string manipulation and functions
#include "co_math.h"   // math definitions
#include "rwip.h"      // rw functions
#include "arch.h"      // architectural platform definitions

/**
****************************************************************************************
* @brief Main init function.
****************************************************************************************
*/
void main_init(void)
{
#if (PLF_NVDS)
	static uint8_t aNvds[NVDS_MEM_SIZE];
	nvds_init((uint8_t *)aNvds, NVDS_MEM_SIZE);
#endif // PLF_NVDS

	/*
	************************************************************************************
	* RW SW stack initialization
	************************************************************************************
	*/

	// Initialize RW SW stack
	rwip_init(RESET_NO_ERROR);

}

/**
****************************************************************************************
* @brief Main loop function.
****************************************************************************************
*/
void main_loop(void)
{
	/*
	************************************************************************************
	* Main loop
	************************************************************************************
	*/

	// start interrupt handling
	GLOBAL_INT_START();

	for (;;)
	{
		// schedule all pending events
		rwip_schedule();

		// Checks for sleep have to be done with interrupt disabled
		GLOBAL_INT_DISABLE();

		// Checks for sleep have to be done with interrupt disabled
		GLOBAL_INT_RESTORE();
	}
}


/**
****************************************************************************************
* @brief Main function.
*
* This function is called right after the booting process has completed.
*
* @return status   exit status
****************************************************************************************
*/
#ifdef CFG_ROM_VT
void my_patch(void);
#endif // CFG_ROM_VT
extern int win32_main(void)
{
	// Initialize platform
#ifdef CFG_ROM_VT
	my_patch();
#endif // CFG_ROM_VT
	main_init();

	// Branch to main loop
	main_loop();

	return 0;
}

#endif // WIN32

