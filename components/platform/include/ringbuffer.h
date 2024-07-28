/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ringbuffer.h
 * @brief
 * @author bingrui.chen
 * @date 2021-12-30
 * @version 1.0
 * @Revision
 */

#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#include <stdint.h>

typedef struct
{
    int32_t head;
    int32_t tail;
    int32_t max_num;
    uint8_t *p_buffer;
} RingBuffer_Type;

/**
 * @brief  Initial Ring Buffer
 * @param[in]   p_ringbuffer: Pointer of the Ring Buffer
 * @param[in]   p_buffer: Pointer of Space in the Ring Buffer
 * @param[in]   maxSize: max size of the Ring Buffer
 * @retval None
 */
extern void ringbuffer_initial(RingBuffer_Type *p_ringbuffer, uint8_t *p_buffer, uint32_t maxSize);

/**
 * @brief  Get Data Count From the Ring Buffer
 * @param[in]   p_ringbuffer: Pointer of the Ring Buffer
 * @retval the valid data number in the ring buffer
 */
extern uint32_t ringbuffer_get_count(RingBuffer_Type *p_ringbuffer);

/**
 * @brief  Push a Byte Into The Ring Buffer
 * @param[in]   p_ringbuffer: Pointer of the Ring Buffer
 * @param[in]   data: data pushed into the Ring Buffer
 * @retval the number of data pushed into the ring buffer
 */
extern int32_t ringbuffer_push_byte(RingBuffer_Type *p_ringbuffer, uint8_t data);

/**
 * @brief  Push Bytes into the Ring Buffer
 * @param[in]   p_ringbuffer: Pointer of the Ring Buffer
 * @param[in]   p_buffer: Pointer of Data Buffer push into the Ring Buffer
 * @param[in]   len: the length of buffer
 * @retval the number of data pushed into the ring buffer
 */
extern int32_t ringbuffer_push_data(RingBuffer_Type *p_ringbuffer, uint8_t *p_buffer, int32_t len);

/**
 * @brief  Pop Data From the Ring Buffer
 * @param[in]   p_ringbuffer: Pointer of the Ring Buffer
 * @param[out]  p_buffer: Pointer of Data Buffer Pop from the Ring Buffer
 * @param[in]   len: the length of buffer
 * @param[in]   head: the start index to pop data from the ring buffer.value 0 is the meaning of the head in the ring buffer
 * @param[in]   tail: the end index to pop data from the ring buffer.value 0 is the meaning of the head in the ring buffer
 * @retval the number of data pop from the ring buffer
 */
extern int32_t ringbuffer_pop_data(RingBuffer_Type *p_ringbuffer, uint8_t *p_buffer, int32_t len, int32_t head,
        int32_t tail);

#endif
