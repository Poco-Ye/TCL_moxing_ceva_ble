/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ringbuffer.c
 * @brief
 * @author bingrui.chen
 * @date 2021-12-30
 * @version 1.0
 * @Revision
 */



#include <ringbuffer.h>
#include <string.h>

#define CHECK_NULL_POINTER(x)

/* Return count in buffer.  */
#define RX_CIRC_CNT(head,tail,size) (((head) - (tail)) & ((size)-1))

/* Return space available, 0..size-1.  We always leave one free char
 as a completely full buffer has head == tail, which is the same as
 empty.  */
#define RX_CIRC_SPACE(head,tail,size) RX_CIRC_CNT((tail),((head)+1),(size))

/* Return count up to the end of the buffer.  Carefully avoid
 accessing head and tail more than once, so they can change
 underneath us without returning inconsistent results.  */
static inline int ringbuffer_get_cnt2end(int head, int tail, int size)
{
    int end = (size) - (tail);
    int n = ((head) + end) & ((size) - 1);
    return n < end ? n : end;
}

/* Return space available up to the end of the buffer.  */
static inline int ringbuffer_get_space2end(int head, int tail, int size)
{
    int end = (size) - 1 - (head);
    int n = (end + (tail)) & ((size) - 1);
    return n <= end ? n : end + 1;
}

void ringbuffer_initial(RingBuffer_Type *p_ringbuffer, uint8_t *pbuffer, uint32_t maxSize)
{
    p_ringbuffer->p_buffer = pbuffer;
    p_ringbuffer->max_num = maxSize;
    p_ringbuffer->head = 0;
    p_ringbuffer->tail = 0;
}

uint32_t ringbuffer_get_count(RingBuffer_Type *p_ringbuffer)
{
    return RX_CIRC_CNT(p_ringbuffer->head, p_ringbuffer->tail, p_ringbuffer->max_num);
}

int32_t ringbuffer_push_byte(RingBuffer_Type *p_ringbuffer, uint8_t data)
{
    CHECK_NULL_POINTER(p_ringbuffer);

    p_ringbuffer->p_buffer[p_ringbuffer->head] = data;
    p_ringbuffer->head = (p_ringbuffer->head + 1) & (p_ringbuffer->max_num - 1);

    return 1;
}

int32_t ringbuffer_push_data(RingBuffer_Type *p_ringbuffer, uint8_t *pbuffer, int32_t len)
{
    int32_t space;

    CHECK_NULL_POINTER(p_ringbuffer);CHECK_NULL_POINTER(pbuffer);

    if (p_ringbuffer->max_num < len)
    {
        return -1;
    }

    space = ringbuffer_get_space2end(p_ringbuffer->head, p_ringbuffer->tail, p_ringbuffer->max_num);
    if (space >= len)
    {
        memcpy(p_ringbuffer->p_buffer + p_ringbuffer->head, pbuffer, len);
    }
    else
    {
        memcpy(p_ringbuffer->p_buffer + p_ringbuffer->head, pbuffer, space);
        memcpy(p_ringbuffer->p_buffer, pbuffer + space, len - space);
    }

    p_ringbuffer->head = (p_ringbuffer->head + len) & (p_ringbuffer->max_num - 1);

    return len;
}

int32_t ringbuffer_pop_data(RingBuffer_Type *p_ringbuffer, uint8_t *pbuffer, int32_t len, int32_t head, int32_t tail)
{
    int16_t iCnt = 0;
    int32_t iEnd;
    int32_t count;

    CHECK_NULL_POINTER(p_ringbuffer);CHECK_NULL_POINTER(pbuffer);

    if (head == 0 && tail == 0)
    {
        head = p_ringbuffer->head;
        tail = p_ringbuffer->tail;
    }

    iCnt = RX_CIRC_CNT(head, tail, p_ringbuffer->max_num);

    count = iCnt > len ? len : iCnt;

    memset(pbuffer, 0, count);

    iEnd = ringbuffer_get_cnt2end(head, tail, p_ringbuffer->max_num);
    if (count <= iEnd)
    {
        memcpy(pbuffer, &p_ringbuffer->p_buffer[tail], count);
    }
    else
    {
        memcpy(pbuffer, &p_ringbuffer->p_buffer[tail], iEnd);
        memcpy(pbuffer + iEnd, &p_ringbuffer->p_buffer[0], count - iEnd);
    }

    p_ringbuffer->tail = head;

    return count;
}

