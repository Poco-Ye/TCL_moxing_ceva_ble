/**
 * Copyright © 2022 by MooreSilicon.All rights reserved
 * @file  pm_imp.h
 * @brief
 * @author pengyu.xue
 * @date 2022-3-16
 * @version 1.0
 * @Revision
 */

#ifndef MS_POWERMANAGE_IMPLEMENTATION_H_
#define MS_POWERMANAGE_IMPLEMENTATION_H_

#ifdef __cplusplus
extern "C" {
#endif


void app_init(void);

void app_sleep_wkup(void);

int app_sleep_init(void);





#include <stdint.h>
#include "stdbool.h"
#include "ms_section.h"

#define PM_DEBUG_BLE_DEEP_SLEEP

#ifdef PM_DEBUG_BLE_DEEP_SLEEP
typedef struct
{
    bool is_allowed_to_enter_sleep;  /* to indicate whether to allow to enter sleep or not */

} ms_sleep_global_data;

extern ms_sleep_global_data g_sleep_data;

/**
 * This function will run get pre sleep mode
 *
 *
 * @param[in] none
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int ms_get_pre_mode();

/**
 * This function will run exit sleep mode
 *
 *
 * @param[in]  none
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
void ms_sys_exit_sleep(void);


/**
 * This function will run enter sleep mode
 *
 *
 * @param[in] none
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
void ms_sys_enter_sleep(void);


/**
 * This function will initialize sleep module
 *
 *
 * @param[in]  p_ds_cfg  sleep config
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
//int ms_sys_sleep_init(void);


/**
 * This function check if can enter sleep mode
 *
 *
 * @param[in]  void
 *
 * @return  1: ok to enter sleep mpde, 
 *          0: can't enter sleep mode due to wakelock etc
 */
uint32_t ms_sys_check_if_enter_sleep(void);


/**
 * This function will register the callback from app (components), to do somthing before sleep
 *
 *
 * @param[in]  p_ds_cfg  sleep config
 *
 * @return  0 : on success,  -1  fail
 */
int32_t app_registerbefore_sleepaction(void);


/**
 * This function will deregister the callback from app (components)
 *
 *
 * @param[in]  p_ds_cfg  sleep config
 *
 * @return  0 : on success,  -1  fail
 */
int32_t app_deregisterbefore_sleepaction(void);
	

/**
 * This function will register the callback from app (components), to do somthing when exit from sleep
 *
 *
 * @param[in]  p_ds_cfg  sleep config
 *
 * @return  0 : on success,  -1  fail
 */
int32_t app_registerafter_sleepaction(void);


/**
 * This function will deregister the callback from app (components)
 *
 *
 * @param[in]  p_ds_cfg  sleep config
 *
 * @return  0 : on success,  -1  fail
 */	
int32_t app_deregisterafter_sleepaction(void);


/**
 * This function will start deep sleep timer. When timer expired, system can enter deep sleep mode
 *
 *
 * @param[in]  p_ds_cfg  sleep config
 *
 * @return  0 : on success,  -1  fail
 */	
void dsleep_timer_start(void);
	

/**
 * This function will restart deep sleep timer. When timer expired, system can enter deep sleep mode
 *
 *
 * @param[in]  p_ds_cfg  sleep config
 *
 * @return  0 : on success,  -1  fail
 */	
void dsleep_timer_restart(void);

	
/**
 * This function will stop deep sleep timer. When timer expired, system can enter deep sleep mode
 *
 *
 * @param[in]  p_ds_cfg  sleep config
 *
 * @return  0 : on success,  -1  fail
 */		
void dsleep_timer_stop(void);

#endif



#ifdef __cplusplus
}
#endif


#endif // MS_POWERMANAGE_IMPLEMENTATION_H_

