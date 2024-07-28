/**
 * Copyright © 2021 by MooreSilicon. All rights reserved
 * @file  ms_queue.c
 * @brief
 * @author bingrui.chen
 * @date 2022年1月25日
 * @version 1.0
 * @Revision:
 */

#include <ms_driver_def.h>
#include "ms_queue.h"

void ms_queue_init(Queue_Type *queue)
{
    queue->head = NULL;
    queue->tail = NULL;
}

void ms_queue_init_pool(Queue_Type *queue, void *pool, size_t elmt_size, uint32_t elmt_cnt)
{
    uint32_t i;
    QueueItem_Type *elt = pool;
    QueueItem_Type *elt_next = NULL;

    // First element
    queue->head = elt;

    // Link elements
    for (i = 0; i < (elmt_cnt - 1); i++)
    {
        elt_next = (QueueItem_Type*) ((uint8_t*) elt + (uint32_t) elmt_size);
        // Link element to next
        elt->next = elt_next;

        // Move to the next element
        elt = elt_next;
    }

    // Last element
    elt->next = NULL;
    queue->tail = elt;
}

void ms_queue_push_back(Queue_Type *queue, QueueItem_Type *queue_node)
{
    CHECK_PTR_NULL(queue);
    CHECK_PTR_NULL(queue_node);

    if (ms_queue_is_empty(queue))
    {
        // queue empty => pushed element is also head
        queue->head = queue_node;
        queue->tail = queue_node;
        queue_node->next = NULL;
    }

    // add element at the end of the queue
    queue_node->next = NULL;
    queue->tail = queue_node;
}

QueueItem_Type* ms_queue_pop_front(Queue_Type *queue)
{
    QueueItem_Type *element = NULL;

    // check if queue is empty
    element = queue->head;
    if (element != NULL)
    {
        // The queue isn't empty : extract the first element
        queue->head = queue->head->next;
        if (queue->head == NULL)
        {
            queue->tail = queue->head;
        }
    }
    return element;
}

uint16_t ms_queue_get_num(Queue_Type *queue)
{
    uint16_t count = 0;
    QueueItem_Type *tmp_queue_hdr = queue->head;

    // browse queue to count number of elements
    while (tmp_queue_hdr != NULL)
    {
        tmp_queue_hdr = tmp_queue_hdr->next;
        count++;
    }

    return count;
}

