/**
  * Copyright © 2021 by MooreSilicon. All rights reserved
  * @file  test_ms_queue.c
  * @brief 
  * @author bingrui.chen
  * @date 2022年1月25日
  * @version 1.0
  * @Revision: 
  */


#include <ms_queue.h>
#include <string.h>
#include "ms_uart.h"
#include "ms_clock_hal.h"
#include "ms_pinmux_hal.h"
#include "ms_audio_adc_hal.h"
#include "unity.h"
#include "uart.h"
#include "unity_test_runner.h"


#define TEST_ADC_ITEM_NUM 3
#define TEST_ADC_ITEM_DATA_SIZE    64

typedef struct __TestQueueItem_Type
{
    struct __TestQueueItem_Type *next;
    uint16_t ptr;
    uint16_t len;
}TestQueueItem_Type;


typedef struct
{
    Queue_Type free_list;
    Queue_Type dirty_list;

    TestQueueItem_Type free_items[TEST_ADC_ITEM_NUM];
}TestQueueQueueList_Type;


char buffer[192];

TestQueueQueueList_Type TestQueueQueueList;

void test_queue_item_init(void)
{
    for (uint8_t i = 0; i < TEST_ADC_ITEM_NUM; i++)
    {
        TestQueueQueueList.free_items[i].ptr = TEST_ADC_ITEM_DATA_SIZE * i;
        TestQueueQueueList.free_items[i].len = TEST_ADC_ITEM_DATA_SIZE;
    }

    ms_queue_init_pool(&TestQueueQueueList.free_list, &TestQueueQueueList.free_items[0], sizeof(TestQueueItem_Type),
            TEST_ADC_ITEM_NUM);

    ms_queue_init(&TestQueueQueueList.dirty_list);
}

void test_queue_item_deinit(void)
{
    ms_queue_init(&TestQueueQueueList.dirty_list);
    ms_queue_init(&TestQueueQueueList.free_list);
}

TEST_CASE("queue","test_queue_init", "[Component/Queue]")
{
    test_queue_item_init();

    TEST_ASSERT_EQUAL(TEST_ADC_ITEM_NUM,ms_queue_get_num(&TestQueueQueueList.free_list));
    TEST_ASSERT_EQUAL(0,ms_queue_get_num(&TestQueueQueueList.dirty_list));

    test_queue_item_deinit();
}

TEST_CASE("queue","test_push_back", "[Component/Queue]")
{
    test_queue_item_init();
    for(uint8_t i = 0;  i < TEST_ADC_ITEM_NUM; i++)
    {
        TestQueueItem_Type* item = (TestQueueItem_Type *)ms_queue_pop_front(&TestQueueQueueList.free_list);
        TEST_ASSERT_NOT_NULL(item);
        TEST_ASSERT_EQUAL(TEST_ADC_ITEM_DATA_SIZE,item->len );
        TEST_ASSERT_EQUAL(i*TEST_ADC_ITEM_DATA_SIZE,item->ptr );
        TEST_ASSERT_EQUAL(0,ms_queue_get_num(&TestQueueQueueList.dirty_list));
        ms_queue_push_back(&TestQueueQueueList.dirty_list,(QueueItem_Type *)item);
        TEST_ASSERT_EQUAL(1,ms_queue_get_num(&TestQueueQueueList.dirty_list));
        TestQueueItem_Type* item_dirty = (TestQueueItem_Type *)ms_queue_pop_front(&TestQueueQueueList.dirty_list);
        TEST_ASSERT_NOT_NULL(item_dirty);
        TEST_ASSERT_EQUAL(TEST_ADC_ITEM_DATA_SIZE,item_dirty->len );
        TEST_ASSERT_EQUAL(i*TEST_ADC_ITEM_DATA_SIZE,item_dirty->ptr );
        ms_queue_push_back(&TestQueueQueueList.free_list,(QueueItem_Type *)item_dirty);
    }
    test_queue_item_deinit();

}


TEST_CASE("queue","test_pop_front", "[Component/Queue]")
{
    test_queue_item_init();
    TEST_ASSERT_NOT_NULL( ms_queue_pop_front(&TestQueueQueueList.free_list));
    TEST_ASSERT_NULL( ms_queue_pop_front(&TestQueueQueueList.dirty_list));
    test_queue_item_deinit();
}
