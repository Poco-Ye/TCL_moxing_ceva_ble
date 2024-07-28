
#include <ms1008.h>
#include <string.h>
#include "ms_uart.h"
#include "unity.h"
#include "uart.h"
#include "pwr.h"
#include "unity_test_runner.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ms_keyscan.h"
#include "ms_audio_adc.h"


#ifdef KEYPAD_TO2
#define KPAD_ROW_MAX        (5)
#define KPAD_COL_MAX         (6)

#define KEY_MUTIL2_MAX (2)
#define KEY_MUTIL3_MAX (1)

static uint8_t  key_table1[KPAD_ROW_MAX][KPAD_COL_MAX] = {
    0xF9, 0x62, 0xC0, 0xF7, 0xC5, 0xD5, //K1 ~ K6
    0xCF, 0xCE, 0xCD, 0xCC, 0xCB, 0xCA, //K7 ~ K12
    0xC9, 0xC8, 0xC7, 0xC6, 0xA9, 0xA8, //K13 ~ K18
    0xA6, 0xA7, 0xD0, 0xD1, 0xD2, 0xD3, //K19 ~ K24
    0xFF, 0x17, 0x1B, 0xD6, 0xD8, 0x0B  //K25 ~ K30
};


static key_mutil2_type key_table2[KEY_MUTIL2_MAX ] = {
    {0x66,  {5, 5}, {5, 6}},              //just test
    {0x22,  {1, 2}, {1, 1}}              //just test
};


static key_mutil3_type key_table3[KEY_MUTIL3_MAX ] = {
    {0x33,  {1, 5}, {1, 4}, {1, 3}}  //just test
};


static kpad_config_type cfg;
static keyscan_process_func kpad_app;

static void app_keypad_just_key_release(key_signal_type key)
{
    MS_LOGI(MS_DRIVER, "\r\njust release app: key map = %#x,  row= %#x, col= %#x,event=%d\r\n", 
		key.map_val,  key.key.row, key.key.col, key.event);
    ms_audio_adc_stop_dma();
}

static void app_keypad_signal_key_process(key_signal_type key)
{
    MS_LOGI(MS_DRIVER, "\r\nsignal press app: key map = %#x,  row= %#x, col= %#x,event=%d\r\n", 
		key.map_val,  key.key.row, key.key.col, key.event);

    ms_audio_adc_start_dma_xfer();
}

static void app_keypad_mutil2_key_process(key_mutil2_type km2)
{
    MS_LOGI(MS_DRIVER, "mutil2 press app:key map = %#x,  row1= %#x, col1= %#x, row2= %#x, col2= %#x\r\n", 
		km2.map_val, km2.key1.row, km2.key1.col, km2.key2.row, km2.key2.col);
}

static void app_keypad_mutil3_key_process(key_mutil3_type km3)
{
    MS_LOGI(MS_DRIVER, "mutil3 press app:key map =%#x,  row1= %#x, col1= %#x, row2= %#x, col2= %#x, row3= %#x, col3= %#x\r\n", 
		km3.map_val, km3.key1.row, km3.key1.col, km3.key2.row, km3.key2.col, km3.key3.row, km3.key3.col);
}
#else
void app_keypad_callback(uint32_t key_value, uint32_t type)
{
    MS_LOGI(MS_DRIVER, "key_value=%#x,type=%d\r\n", key_value, type);
}
#endif



void test_keypad2(void)
{
#ifdef KEYPAD_TO2
    kpad_app.func_release = app_keypad_just_key_release;
    kpad_app.func_signal_press = app_keypad_signal_key_process;
    kpad_app.func_muil2_press = app_keypad_mutil2_key_process;
    kpad_app.func_mutil3_press = app_keypad_mutil3_key_process;

    ms_keypad_signal_map_register((void **)key_table1, 5, 6);
    ms_keypad_mutil2_map_register(key_table2,  KEY_MUTIL2_MAX);
    ms_keypad_mutil3_map_register(key_table3,  KEY_MUTIL3_MAX);

    ms_keyscan_default_config(&cfg);

    keyscan_module_init(&cfg, &kpad_app);

#else
    ms_keyscan_init(app_keypad_callback);
#endif
}




