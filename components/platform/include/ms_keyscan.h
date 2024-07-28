/**
 * Copyright © 2022 by MooreSilicon.All rights reserved
 * @file  ms_keyscan.h
 * @brief
 * @author pengyu.xue
 * @date 2022-2-22
 * @version 1.0
 * @Revision
 */

#ifndef __MS_KEYSCAN_H__
#define __MS_KEYSCAN_H__

typedef struct 
{	   
	 uint8_t key_colum;
	 uint8_t key_row;
	 uint8_t press_event;
	 uint8_t long_press_event;  
}KeypadFifoData_Type;


/**
 * @brief  KeyScan global data struct definition.
 */
typedef struct
{
 //   uint8_t is_allowed_to_repeat_report;           /* to indicate whether to allow to report repeat keyscan data event or not */
 //   uint8_t is_allowed_to_enter_dlps;              /* to indicate whether to allow to enter dlps or not */
 //   uint8_t is_key_pressed;                        /* to indicate whether any key is pressed or not */
 //   uint8_t is_all_key_released;          /* to indicate whether all keys are released or not */
    uint8_t is_valid_key;                   /* to indicate whether the  keys is right for software debounce */	
    KeypadFifoData_Type pre_fifo_data;     /* to indicate the previous keyscan FIFO data, for combination key check*/ 
    KeypadFifoData_Type cur_fifo_data;     /* to indicate the current keyscan FIFO data */
} KeypadGlobalData_Type;
		



typedef struct
{
    uint32_t preKeyValue;       
    uint32_t curKeyValue;             
    bool    is_Multikey;                          
 
} MultKeyData_Type;



/* app key message*/
struct app_keypad_db_cfg
{
    uint32_t             key_value;
    uint32_t             press;
};

/**
 * @brief  KeyScan application call back function, to get the keyvalue and presstype.
 */
typedef void (*app_keypad_msg_cb)(uint32_t value, uint32_t type);


void  ms_keyscan_init(app_keypad_msg_cb func);

void ms_keyscan_config(void);

void ms_keyscan_intrrupt_enable(bool flag);

void ms_HWdebouce_enable(bool flag);

#include "FreeRTOS.h"
#include "queue.h"
 

#define KEYPAD_QUEUE_MAX (10)


#define KPAD_EVENT_KEY_NONE                                  (0x0)
#define KPAD_EVENT_KEY_PRESS                                 (0x1)
#define KPAD_EVENT_KEY_RELEASE                             (0x2)
#define KPAD_EVENT_KEY_LONG_PRESS                      (0x3)
#define KPAD_EVENT_KEY_LONG_PRESS_REPEAT        (0x4)
#define KPAD_EVENT_KEY_TIMEOUT                            (0x5)
#define KPAD_EVENT_KEY_INVALID                             (0xFF)

#define KPAD_NONE_KEYVAL (0xFF)

#define KPAD_INVALID          (0x00)
#define KPAD_VALID              (0x01)


#define KPAD_MULTI_KEY_ERR            (0xFF)
#define KPAD_MULTI_KEY_RELEASE    (0)
#define KPAD_MULTI_KEY_ONE           (1)
#define KPAD_MULTI_KEY_TWO          (2)
#define KPAD_MULTI_KEY_THREE       (3)

typedef uint32_t kpad_event_type;

/**
 * @brief  KeyScan application call back function, to get the keyvalue and presstype.
 */
typedef void (*app_keypad_msg_cb)(uint32_t value, uint32_t type);

typedef struct{
    uint8_t row;
    uint8_t col;
}key_coordinate_type;

typedef struct
{
    uint8_t map_val;
    key_coordinate_type key1;
    key_coordinate_type key2;
} key_mutil2_type;

typedef struct
{
    uint8_t map_val;
    key_coordinate_type key1;
    key_coordinate_type key2;
    key_coordinate_type key3;
} key_mutil3_type;

typedef struct 
{
    key_coordinate_type key;
    kpad_event_type key_event;
}kpadevent_type;



typedef struct kpad_event_list
{
    struct kpad_event_list * next;
    kpadevent_type event;
}kpad_event_list;


typedef struct
{
    uint8_t map_val;
    uint8_t event;
    key_coordinate_type key;
} key_signal_type;


/**
 * @brief  KeyScan global data struct definition.
 */
typedef struct
{
    uint8_t key_multi_type;
    uint8_t map_val;

    kpadevent_type cur_event;     /* to indicate the current keyscan FIFO data */
    key_mutil2_type kmutil2;
    key_mutil3_type kmutil3;
} kpadevent_list_type;


/**
  *@brief  KeyScan init  struct definition.
  */
typedef struct
{
    bool long_press_enable;
    bool long_press_repeat_enable;
    uint32_t lp_parameter;
    bool long_press_fifo_enble;
    bool long_press_repeat_fifo_enble;
    bool long_press_int_enable;
}kpad_long_press_type;

typedef struct
{
    bool press_int_enable;
    bool press_timeout_int_enable;
    bool press_write_fifo_enble;
    bool press_timeout_write_fifo_enble;
    bool press_debounce_enble;
    bool press_timeout_enable;
    uint32_t press_debounce_parameter;
    uint32_t press_timeout_parameter;
}kpad_press_type;

typedef struct
{
    bool release_int_enable;
    bool release_write_fifo_enble;
    uint32_t release_debounce_parameter;
}kpad_release_type;


typedef struct
{
    kpad_long_press_type longpre_config;
    kpad_press_type  press_config;
    kpad_release_type relese_config;
    uint8_t fifo_watermark_val;
    bool fifo_wtermark_int;
    bool fifo_empty_int;
    bool fifo_full_int;
    uint32_t scan_period_val;
    uint32_t debounce_timeout;
}kpad_config_type;


typedef struct {
    void (*func_release)(key_signal_type key);
    void (*func_signal_press)(key_signal_type key);
    void (*func_muil2_press)(key_mutil2_type km2);
    void (*func_mutil3_press)(key_mutil3_type km3);
}keyscan_process_func;



extern QueueHandle_t keyPadQueue;


uint8_t ms_keypad_key_event_probe(kpadevent_type kpdevent);
void ms_keypad_key_event_process(uint8_t mutil_type);
void  ms_keyscan_default_config(kpad_config_type *cfg);
int ms_keyscan_init_config(kpad_config_type *config);
void ms_keypad_app_register(keyscan_process_func *key_func);
void keyscan_module_init(kpad_config_type *cfg, keyscan_process_func *kpad_app);
void ms_keypad_signal_map_register(void **table, uint8_t max_row, uint8_t max_col);
void ms_keypad_mutil2_map_register(void *table, uint8_t max);
void ms_keypad_mutil3_map_register(void *table, uint8_t max);


#endif
