/**
 * Copyright © 2022 by MooreSilicon.All rights reserved
 * @file  ms_keyscan.c
 * @brief
 * @author pengyu.xue
 * @date 2022-2-22
 * @version 1.0
 * @Revision
 */
#include <string.h>
#include "stdlib.h"
#include "log.h"
#include "ms_keyscan.h"
#include "ms_keyscan_hal.h"
#include "ms_keyscan_regs.h"
#include "FreeRTOS.h"
#include "queue.h"

#define KPAD_ROW_MAX        (5)
#define KPAD_COL_MAX         (6)

keyscan_process_func key_event_func;
kpadevent_list_type g_kpad_event_list;
QueueHandle_t keyPadQueue;
kpad_event_list list_head_node;
kpad_event_list *list_head;


uint8_t (*key_signal_table)[KPAD_COL_MAX];
key_mutil2_type *key_mutil2_table;
key_mutil3_type *key_mutil3_table;
uint8_t map_row, map_col, mutil2_len,mutil3_len;

uint8_t scan_state[KPAD_ROW_MAX][KPAD_COL_MAX] = {0x00};
uint32_t state_bitmap;
//////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

static void ms_keyscan_gpio_init(void);
static uint8_t ms_key_press_check(kpadevent_type kpdevent);
extern void board_keypad_gpio_init();

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

static void ms_keypad_list_init(void)
{
    list_head = &list_head_node;
    list_head->next = NULL;
}

static void ms_keypad_list_delete(kpadevent_type kpdevent)
{
    uint8_t  row = kpdevent.key.row;
    uint8_t  col = kpdevent.key.col;
    kpad_event_list *cur_list;
    kpad_event_list *pre_list;

    if(kpdevent.key_event != KPAD_EVENT_KEY_RELEASE)
    {
        return;
    }

    for(cur_list = list_head->next, pre_list = list_head; cur_list != NULL; cur_list = cur_list->next)
    {
        if((cur_list->event.key.row == row) && (cur_list->event.key.col == col))
        {
             pre_list->next = cur_list->next;
	      free(cur_list);
	      break;
	 }
        pre_list = cur_list;
    }

}

static void ms_keypad_list_add_tail(kpadevent_type kpdevent)
{
    kpad_event_list *cur_list;

    if(!ms_key_press_check(kpdevent))
    {
        return;
    }
    if(list_head->next == NULL)
    {
        list_head->next  = (kpad_event_list *)malloc(sizeof(kpad_event_list));
        list_head->next->event = kpdevent;
        list_head->next->next = NULL;
	 return;
    }

    for(cur_list = list_head->next; cur_list != NULL; cur_list = cur_list->next)
    {
        if(cur_list->next == NULL)
        {
            cur_list->next = (kpad_event_list *)malloc(sizeof(kpad_event_list));
	     cur_list = cur_list->next;
            cur_list->event = kpdevent;
            cur_list->next = NULL;
            break;
        }
    }


}

static uint8_t ms_key_press_check(kpadevent_type kpdevent)
{
    if((kpdevent.key_event == KPAD_EVENT_KEY_PRESS) || 
		(kpdevent.key_event == KPAD_EVENT_KEY_LONG_PRESS) ||
		(kpdevent.key_event == KPAD_EVENT_KEY_LONG_PRESS_REPEAT))
    {
        return true;
    }
    return false;
}

static uint8_t  key_state_scan(void)
{
    uint8_t ret = 0;
    uint8_t *sp = ( uint8_t *)(&scan_state[0][0]);

    for(uint8_t num = 0; num < KPAD_ROW_MAX * KPAD_COL_MAX;num++)
    {
         sp[num] = (state_bitmap >> num) & 0x01;
         if(sp[num] == 0)
         {
             ret++;
         }
    }
    return ret;
}


static bool ms_keypad_signal_table_match(kpadevent_type kpdevent)
{
    uint8_t  row = kpdevent.key.row;
    uint8_t  col = kpdevent.key.col;
    if(key_signal_table == NULL)
    {
        return false;
    }
    g_kpad_event_list.map_val = key_signal_table[row-1][col-1];
    return true;
}

static bool ms_keypad_signal_status_match(kpadevent_type kpdevent)
{
    return ms_key_press_check(kpdevent);
}

static bool ms_keypad_multikey2_table_match(kpadevent_type kpdevent1, kpadevent_type kpdevent2)
{
    uint8_t  row1 = kpdevent1.key.row;
    uint8_t  col1 = kpdevent1.key.col;
    uint8_t  row2 = kpdevent2.key.row;
    uint8_t  col2 = kpdevent2.key.col;

    if(key_mutil2_table == NULL)
    {
        MS_LOGI(MS_DRIVER, "m2 table no register!\r\n");
        return false;
    }

    for(uint8_t index = 0; index < mutil2_len; index++)
    {
        if(((row1 == key_mutil2_table[index].key1.row) && (col1 == key_mutil2_table[index].key1.col) &&
           (row2 == key_mutil2_table[index].key2.row) && (col2 == key_mutil2_table[index].key2.col))  ||
           ((row2 == key_mutil2_table[index].key1.row) && (col2 == key_mutil2_table[index].key1.col) &&
           (row1== key_mutil2_table[index].key2.row) && (col1 == key_mutil2_table[index].key2.col)))
        {
              g_kpad_event_list.kmutil2.map_val = key_mutil2_table[index].map_val;
              return true;
	 }
    }
    MS_LOGI(MS_DRIVER, "m2 table no match, not mutil2\r\n");
    return false;
}

static bool ms_keypad_multikey2_status_match(kpadevent_type kpdevent1, kpadevent_type kpdevent2)
{
    uint8_t  row1 = kpdevent1.key.row;
    uint8_t  col1 = kpdevent1.key.col;
    uint8_t  row2 = kpdevent2.key.row;
    uint8_t  col2 = kpdevent2.key.col;

    if(!(ms_key_press_check(kpdevent1) || ms_key_press_check(kpdevent2)))
    {
        MS_LOGI(MS_DRIVER, " m2 status no match\r\n");
        return false;
    }

    if((scan_state[row1-1][col1-1] != 0) || (scan_state[row2-1][col2-1] != 0))
    {
        MS_LOGI(MS_DRIVER, " m2 voltage no match, not m2\r\n");
        return false;
    }
return true;
}

static bool ms_keypad_mutil3_signal_key_match(kpadevent_type for_match_key, uint8_t index)
{
    uint8_t  row = for_match_key.key.row;
    uint8_t  col = for_match_key.key.col;

    if(((row == key_mutil3_table[index].key1.row) && (col == key_mutil3_table[index].key1.col)) || 
        ((row == key_mutil3_table[index].key2.row) && (col == key_mutil3_table[index].key2.col)) ||
        ((row == key_mutil3_table[index].key3.row) && (col == key_mutil3_table[index].key3.col)))
    {
        return true;
    }
    return false;
}

static bool ms_keypad_multikey3_key_valid_check(kpadevent_type key1, kpadevent_type key2, kpadevent_type key3)
{
     uint8_t  row1 = key1.key.row;
    uint8_t  col1 = key1.key.col;
    uint8_t  row2 = key2.key.row;
    uint8_t  col2 = key2.key.col;
    uint8_t  row3 = key3.key.row;
    uint8_t  col3 = key3.key.col;

    if((row1 == row2) && (col1 == col2))
        return false;
    if((row1 == row3) && (col1 == col3))
        return false;
    if((row2 == row3) && (col2 == col3))
        return false;

return true;
}

static bool ms_keypad_multikey3_table_match(kpadevent_type key1, kpadevent_type key2, kpadevent_type key3)
{

   if(!ms_keypad_multikey3_key_valid_check(key1,key2,key3))
   {
        return false;
   }
    if(key_mutil3_table == NULL)
    {
         MS_LOGI(MS_DRIVER, "m3 table no register!\r\n");
	 return false;
    }
    for(uint8_t index = 0; index < mutil3_len; index++)
    {
        if(ms_keypad_mutil3_signal_key_match(key1, index) && 
           ms_keypad_mutil3_signal_key_match(key2, index) && 
           ms_keypad_mutil3_signal_key_match(key3, index))
        {
              g_kpad_event_list.kmutil3.map_val = key_mutil3_table[index].map_val;
              return true;
	 }
    }
    MS_LOGI(MS_DRIVER, "m3 table no match, not mutil3\r\n");
    return false;
}

static bool ms_keypad_multikey3_status_match(kpadevent_type kpdevent1, kpadevent_type kpdevent2, kpadevent_type kpdevent3)
{
    uint8_t  row1 = kpdevent1.key.row;
    uint8_t  col1 = kpdevent1.key.col;
    uint8_t  row2 = kpdevent2.key.row;
    uint8_t  col2 = kpdevent2.key.col;
    uint8_t  row3 = kpdevent3.key.row;
    uint8_t  col3= kpdevent3.key.col;

    if(!((kpdevent1.key_event & (KPAD_EVENT_KEY_PRESS | KPAD_EVENT_KEY_LONG_PRESS | KPAD_EVENT_KEY_LONG_PRESS_REPEAT)) ||
		(kpdevent2.key_event & (KPAD_EVENT_KEY_PRESS | KPAD_EVENT_KEY_LONG_PRESS | KPAD_EVENT_KEY_LONG_PRESS_REPEAT)) ||
		(kpdevent1.key_event & (KPAD_EVENT_KEY_PRESS | KPAD_EVENT_KEY_LONG_PRESS | KPAD_EVENT_KEY_LONG_PRESS_REPEAT))))
    {
        return false;
    }

    if((scan_state[row1-1][col1-1] != 0) || (scan_state[row2-1][col2-1] != 0) || (scan_state[row3-1][col3-1] != 0))
    {
        MS_LOGI(MS_DRIVER, "m3 status no match\r\n");
        return false;
    }
    return true;
}

static void ms_keyscan_gpio_init(void)
{
    board_keypad_gpio_init(); /* call to different evb board function*/
}

void  ms_keyscan_default_config(kpad_config_type *cfg)
{
    if(cfg ==  NULL)
    {
        return;
    }
    // config ..
    cfg->debounce_timeout = 30000;
    cfg->scan_period_val = 10;
    cfg->fifo_full_int = true;
    cfg->fifo_empty_int = false;
    cfg->fifo_wtermark_int = false;
    cfg->fifo_watermark_val = 4;

    //config long press option 
    cfg->longpre_config.long_press_enable = false;
    cfg->longpre_config.long_press_fifo_enble = false;
    cfg->longpre_config.long_press_int_enable = false;
    cfg->longpre_config.long_press_repeat_enable = false;
    cfg->longpre_config.long_press_repeat_fifo_enble = false;
    cfg->longpre_config.lp_parameter = 0;

    // config press option
    cfg->press_config.press_debounce_enble = true;
    cfg->press_config.press_debounce_parameter = 200;
    cfg->press_config.press_int_enable = true;
    cfg->press_config.press_timeout_enable = true;
    cfg->press_config.press_timeout_int_enable = false;
    cfg->press_config.press_timeout_parameter = 10000000;
    cfg->press_config.press_timeout_write_fifo_enble = false;
    cfg->press_config.press_write_fifo_enble = true;

    //config release option
    cfg->relese_config.release_debounce_parameter = 200;
    cfg->relese_config.release_int_enable = true;
    cfg->relese_config.release_write_fifo_enble = true;

    MS_LOGI(MS_DRIVER, " keyscan us default config \r\n");
}

int ms_keyscan_init_config(kpad_config_type *config)
{
    if(config == NULL)
    {
        return -1;
    }
     keyPadQueue = xQueueCreate(KEYPAD_QUEUE_MAX, sizeof(kpadevent_type));

     ms_keyscan_hal_config_sel();
     ms_keyscan_gpio_init();
     ms_keyscan_hal_enable_clk();
     ms_clock_hal_set_keyscan_div(4);

     ms_keyscan_hal_fifo_flush();

     ms_keyscan_hal_config_fifo_watermark(config->fifo_watermark_val);
     ms_keyscan_hal_config_scan_period(config->scan_period_val);
     ms_keyscan_hal_config_debounce_timeout(config->debounce_timeout);

     if(config->fifo_wtermark_int == true)
     {
        ms_keyscan_hal_enable_fifo_watermark_int();
     }
     else
     {
        ms_keyscan_hal_disable_fifo_watermark_int();
     }

     if(config->fifo_empty_int == true)
     {
          ms_keyscan_hal_enable_fifo_empty_int();
     }
     else
     {
          ms_keyscan_hal_disable_fifo_empty_int();
     }

     if(config->fifo_full_int == true)
     {
          ms_keyscan_hal_enable_fifo_full_int();
     }
     else
     {
          ms_keyscan_hal_disable_fifo_full_int();
     }

      if(config->longpre_config.long_press_enable == true)
      {
          ms_keyscan_hal_enable_long_press();
          ms_keyscan_hal_enable_long_press_int();
      }
      else
      {
	   ms_keyscan_hal_disable_long_press();
          ms_keyscan_hal_disable_long_press_int();
      }

      if(config->longpre_config.long_press_repeat_enable == true)
      {
          ms_keyscan_hal_enable_long_press_repeat_int();
      }
      else
      {
          ms_keyscan_hal_disable_long_press_repeat_int();
      }

      if(config->longpre_config.long_press_fifo_enble == true)
      {
           ms_keyscan_hal_enable_long_press_fifo();
      }
      else
      {
           ms_keyscan_hal_disable_long_press_fifo();
      }

     if(config->longpre_config.long_press_repeat_fifo_enble == true)
     {
           ms_keyscan_hal_enable_long_repeat_press_fifo();
     }
     else
     {
          ms_keyscan_hal_disable_long_repeat_press_fifo();
     }
     ms_keyscan_hal_config_long_press_value(config->longpre_config.lp_parameter);
   

    if(config->press_config.press_int_enable == true)
    {
        ms_keyscan_hal_enable_press_int();
    }
    else
    {
        ms_keyscan_hal_disable_press_int();
    }
    
    if(config->press_config.press_write_fifo_enble == true)
    {
        ms_keyscan_hal_enable_press_fifo();
    }
    else
    {
        ms_keyscan_hal_disable_press_fifo();
    }
    
    if(config->press_config.press_timeout_enable == true)
    {
        ms_keyscan_hal_config_press_timeout(config->press_config.press_timeout_parameter);
        ms_keyscan_hal_enable_press_timeout();
        if(config->press_config.press_timeout_write_fifo_enble == true)
        {
            ms_keyscan_hal_enable_press_timeout_fifo();
        }
        else
        {
            ms_keyscan_hal_disable_press_timeout_fifo();
        }
    }
    else
    {
        ms_keyscan_hal_disable_press_timeout();
        ms_keyscan_hal_disable_press_timeout_fifo();
    }

    if(config->press_config.press_debounce_enble == true)
    {
        ms_keyscan_hal_config_press_debounce(config->press_config.press_debounce_parameter);
        ms_keyscan_hal_enable_press_debounce();
    }
    else
    {
        ms_keyscan_hal_disable_press_debounce();
    }
    

    if(config->relese_config.release_int_enable == true)
    {
        if(config->relese_config.release_write_fifo_enble == true)
        {
            ms_keyscan_hal_enable_release_fifo();
        }
	 else
	 {
            ms_keyscan_hal_disable_release_fifo();
	 }
	 ms_keyscan_hal_config_release_debounce(config->relese_config.release_debounce_parameter);
        ms_keyscan_hal_enable_release_int();
    }
    else
    {
        ms_keyscan_hal_disable_release_int();
    }

    memset(&g_kpad_event_list, 0,  sizeof(g_kpad_event_list));
    g_kpad_event_list.key_multi_type = KPAD_MULTI_KEY_ONE;
    g_kpad_event_list.cur_event.key_event = KPAD_NONE_KEYVAL;

    ms_keypad_list_init();
    ms_keyscan_hal_enable_cpu_int();
    ms_keyscan_hal_enable_kpad_int();
    ms_keyscan_hal_enable_module();
}
 

static bool ms_keypad_fifo_empty(void)
{
    if(ms_keyscan_hal_get_fifo_empty_int_status())
    {
        return true;
    }
    return false;
}

static bool ms_keypad_key_invalid_check(kpadevent_type kpdevent)
{
    if((kpdevent.key_event <= KPAD_EVENT_KEY_NONE)  ||  (kpdevent.key_event >= KPAD_EVENT_KEY_INVALID) ||
	(kpdevent.key.row > map_row)  || (kpdevent.key.col > map_col))
    {
        return true;
    }

    return false;
}


void ms_keypad_app_register(keyscan_process_func *key_func)
{
    key_event_func.func_release = key_func->func_release;
    key_event_func.func_signal_press = key_func->func_signal_press;
    key_event_func.func_muil2_press = key_func->func_muil2_press;
    key_event_func.func_mutil3_press = key_func->func_mutil3_press;
}

void ms_keypad_key_event_process(uint8_t mutil_type)
{
    key_signal_type key;

    switch(mutil_type)
    {
        case KPAD_MULTI_KEY_ONE:
            key.event = g_kpad_event_list.cur_event.key_event;
	     key.map_val = g_kpad_event_list.map_val;
	     key.key = g_kpad_event_list.cur_event.key;
            if(key_event_func.func_signal_press)
            {
                key_event_func.func_signal_press(key);
            }
	     break;
        case KPAD_MULTI_KEY_TWO:
            if(key_event_func.func_muil2_press)
            {
                key_event_func.func_muil2_press(g_kpad_event_list.kmutil2);
            }
            break;
        case KPAD_MULTI_KEY_THREE:
            if(key_event_func.func_mutil3_press)
            {
                key_event_func.func_mutil3_press(g_kpad_event_list.kmutil3);
            }
            break;
        case KPAD_MULTI_KEY_RELEASE:
	     key.event = g_kpad_event_list.cur_event.key_event;
	     key.map_val = g_kpad_event_list.map_val;
	     key.key = g_kpad_event_list.cur_event.key;
            if(key_event_func.func_release)
            {
	         key_event_func.func_release(key);
            }
            break;
         case KPAD_MULTI_KEY_ERR:
         default:
            MS_LOGI(MS_DRIVER,"table no match \r\n");
            break;
    }
}

 

uint8_t ms_keypad_key_event_probe(kpadevent_type kpdevent)
{
    uint8_t press_count = 0;
    kpadevent_type press1, press2, press3;

    if(ms_keypad_key_invalid_check(kpdevent))
    {
	  return KPAD_MULTI_KEY_ERR;
    }

    press_count = key_state_scan();
    g_kpad_event_list.key_multi_type = KPAD_MULTI_KEY_ERR;

    if(kpdevent.key_event == KPAD_EVENT_KEY_RELEASE)
    {
         ms_keypad_signal_table_match(kpdevent); // just updat map val
         ms_keypad_list_delete(kpdevent);
         g_kpad_event_list.cur_event = kpdevent;
         g_kpad_event_list.key_multi_type = KPAD_MULTI_KEY_RELEASE;
         return KPAD_MULTI_KEY_RELEASE;
    }

    ms_keypad_list_add_tail(kpdevent);
    switch(press_count)
    {
        case KPAD_MULTI_KEY_ONE:
                if(ms_keypad_signal_table_match(kpdevent))
                {
	            if(ms_keypad_signal_status_match(kpdevent))
	            {
	                g_kpad_event_list.cur_event = kpdevent;
	                g_kpad_event_list.key_multi_type =  KPAD_MULTI_KEY_ONE;
		     }
	         }
                break;
	 case KPAD_MULTI_KEY_TWO:
                press1 = list_head->next->event;
                press2 = list_head->next->next->event;

                if(ms_keypad_multikey2_table_match(press1, press2))
                {
                    if(ms_keypad_multikey2_status_match(press1, press2))
                    {
                         g_kpad_event_list.kmutil2.key1 = press1.key;
                         g_kpad_event_list.kmutil2.key2 = press2.key;
                         g_kpad_event_list.key_multi_type =  KPAD_MULTI_KEY_TWO;
	                  MS_LOGI(MS_DRIVER, "mutil2 match\r\n");
	             }
                }
                break;
	 case KPAD_MULTI_KEY_THREE:
                press1 = list_head->next->event;
                press2 = list_head->next->next->event;
                press3 = list_head->next->next->next->event;

                if(ms_keypad_multikey3_table_match(press1, press2, press3))
                {
                   if(ms_keypad_multikey3_status_match(press1, press2, press3))
                   {
                       g_kpad_event_list.kmutil3.key1 = press1.key;
                       g_kpad_event_list.kmutil3.key2 = press2.key;
                       g_kpad_event_list.kmutil3.key3 = press3.key;
                       g_kpad_event_list.key_multi_type =  KPAD_MULTI_KEY_THREE;
                       MS_LOGI(MS_DRIVER, "mutil3 match\r\n");
	             }
                }
                break;
         default:
                MS_LOGI(MS_DRIVER, "keypad err!\r\n");
    }


    return g_kpad_event_list.key_multi_type;

}


void ms_keypad_signal_map_register(void **table, uint8_t max_row, uint8_t max_col)
{
    uint8_t table_len = max_row * max_col;
	
    if((table == NULL) || (max_row > KPAD_ROW_MAX) || (max_col > KPAD_COL_MAX))
    {
        MS_LOGI(MS_DRIVER, "ms_keypad_signal_map_register err!\r\n");
        return;
    }

    key_signal_table = (uint8_t (*)[KPAD_COL_MAX])table;

    map_row = max_row;
    map_col = max_col;
}

void ms_keypad_mutil2_map_register(void *table, uint8_t max)
{
    if(table == NULL)
    {
        MS_LOGI(MS_DRIVER, "ms_keypad_mutil2_map_register err!\r\n");
        return;
    }

     key_mutil2_table = table;
     mutil2_len = max;

}

void ms_keypad_mutil3_map_register(void *table, uint8_t max)
{
    if(table == NULL)
    {
        MS_LOGI(MS_DRIVER, "ms_keypad_mutil3_map_register err!\r\n");
        return;
    }

    key_mutil3_table = table;
    mutil3_len = max;
}


void KEYSCAN_IRQHandler(void)
{
    kpadevent_type kpadq;
    uint32_t rdata;
    BaseType_t xHigherPriorityTaskWoken = pdTRUE;

    INTERRUPT_DISABLE_IRQ(KEYSCAN_IRQn);
    if(ms_keyscan_hal_get_press_int_status())
    {
        ms_keyscan_hal_clear_press_int_status();
    }
    if(ms_keyscan_hal_get_release_int_status())
    {
        ms_keyscan_hal_clear_release_int_status();
    }
    if(ms_keyscan_hal_get_long_press_int_status())
    {
        ms_keyscan_hal_clear_long_press_int_status();
    }
    if(ms_keyscan_hal_get_press_timeout_int_status())
    {
        ms_keyscan_hal_clear_press_timeout_int_status();
    }

    while(!ms_keypad_fifo_empty())
    {
        rdata = ms_keyscan_hal_get_fifo_rdata();

        kpadq.key.col = KEYSCAN_KPAD_COL(rdata);
        kpadq.key.row = KEYSCAN_KPAD_ROW(rdata);
        kpadq.key_event = KEYSCAN_KEY_EVEN(rdata);

        state_bitmap = ms_keyscan_hal_scan_kpad_state();

        xQueueSendFromISR(keyPadQueue, (void *)(&kpadq), &xHigherPriorityTaskWoken);
    }

    INTERRUPT_ENABLE_IRQ(KEYSCAN_IRQn);
}

/**
 *  keypad tastk init
 */
#define KPADTASK_STACK_DEPTH  1024  // test main stack 2K * 4

static TaskHandle_t kpadtaskHandler;
 
static uint8_t keypad_thread(void)
{
    static kpadevent_type kpdevent;
    uint8_t mutil_type, ret;

    if(xQueueReceive(keyPadQueue, (void *)&kpdevent , 0) == pdPASS)
    {
        mutil_type = ms_keypad_key_event_probe(kpdevent);
        ms_keypad_key_event_process(mutil_type);
	 ret = pdPASS;
    }
    ret = pdFAIL;
}

uint8_t audio_thread(void);
void ms_audio_module_init(void);

static void vWorkQueuetask(void* pvParameters)
{
    uint8_t noidle;
    while(1)
    {
        noidle = keypad_thread();
        noidle += audio_thread();
        if(!noidle)
        {
            vTaskDelay(5);
	 }
    }
}

void keyscan_module_init(kpad_config_type *cfg, keyscan_process_func *kpad_app)
{
    ms_keyscan_init_config(cfg);
    ms_keypad_app_register(kpad_app);

    ms_audio_module_init();

    xTaskCreate((TaskFunction_t)vWorkQueuetask, (const char*)"workqueue",
                (uint16_t)KPADTASK_STACK_DEPTH, (void*)NULL, (UBaseType_t)configMAX_PRIORITIES - 1,
                (TaskHandle_t*)&kpadtaskHandler); 
}
