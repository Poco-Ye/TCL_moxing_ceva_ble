/**
  * Copyright © 2021 by MooreSilicon. All rights reserved
  * @file  ms_queue.h
  * @brief 
  * @author bingrui.chen
  * @date 2022年1月25日
  * @version 1.0
  * @Revision: 
  */
#ifndef MS_QUEUE_H_
#define MS_QUEUE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief  Queue Item Structure Definition
 */
typedef struct __QueueItem_Type
{
	struct __QueueItem_Type *next;
}QueueItem_Type;


/**
 * @brief  Queue List Structure Definition
 */
typedef struct 
{
	QueueItem_Type *head;
	QueueItem_Type *tail;
}Queue_Type;

/**
 * @brief  Initial Queue
 * @param[in]  queue:  Pointer to a Queue_Type structure 
 * @retval none
 */
void ms_queue_init(Queue_Type *queue);

/**
 * @brief  Initial Queue with initial parameters
 * @param[in]  queue:  Pointer to a Queue_Type structure 
 * @param[in]  pool:  Pointer to a space of some QueueItem_Type structure 
 * @param[in]  item_size:  the size of a item
 * @param[in]  item_num:  the item numbers in queue
 * @retval none
 */
void ms_queue_init_pool(Queue_Type *list, void *pool, size_t elmt_size, uint32_t elmt_cnt);

/**
 * @brief  Initial Queue with initial parameters
 * @param[in]  queue:  Pointer to a Queue_Type structure 
 * @retval Pointer to a QueueItem_Type structure 
 */
QueueItem_Type *ms_queue_pop_front(Queue_Type *queue);

/**
 * @brief  Initial Queue with initial parameters
 * @param[in]  queue:  Pointer to a Queue_Type structure 
 * @param[in]  item:  Pointer to a QueueItem_Type structure 
 * @retval none 
 */
void ms_queue_push_back(Queue_Type *queue,QueueItem_Type *item);

/**
 * @brief  verify the queue is empty
 * @param[in]  queue:  Pointer to a Queue_Type structure 
 * @retval true of false 
 */
static inline bool ms_queue_is_empty(Queue_Type *queue)
{
    return (queue->head == NULL);
}

/**
 * @brief  Get the item number in the Queue 
 * @param[in]  queue:  Pointer to a Queue_Type structure 
 * @retval true of false 
 */
uint16_t ms_queue_get_num(Queue_Type *queue);

#endif /* MS_QUEUE_H_ */
