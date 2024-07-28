#ifndef _REG_INTCNTL_H_
#define _REG_INTCNTL_H_

#include <stdint.h>
#include "_reg_intcntl.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_INTCNTL_COUNT 17

#define REG_INTCNTL_DECODING_MASK 0x0000007F

/**
 * @brief IRQ_STATUS register definition
 */
#define INTCNTL_IRQ_STATUS_ADDR   0x00400000
#define INTCNTL_IRQ_STATUS_OFFSET 0x00000000
#define INTCNTL_IRQ_STATUS_INDEX  0x00000000
#define INTCNTL_IRQ_STATUS_RESET  0x00000000
#define INTCNTL_IRQ_STATUS_COUNT  2

__INLINE uint32_t intcntl_irq_status_get(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 1);
    return REG_PL_RD(INTCNTL_IRQ_STATUS_ADDR + reg_idx * 4);
}

/**
 * @brief IRQ_RAW_STATUS register definition
 */
#define INTCNTL_IRQ_RAW_STATUS_ADDR   0x00400008
#define INTCNTL_IRQ_RAW_STATUS_OFFSET 0x00000008
#define INTCNTL_IRQ_RAW_STATUS_INDEX  0x00000002
#define INTCNTL_IRQ_RAW_STATUS_RESET  0x00000000
#define INTCNTL_IRQ_RAW_STATUS_COUNT  2

__INLINE uint32_t intcntl_irq_raw_status_get(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 1);
    return REG_PL_RD(INTCNTL_IRQ_RAW_STATUS_ADDR + reg_idx * 4);
}

/**
 * @brief IRQ_UNMASK_SET register definition
 */
#define INTCNTL_IRQ_UNMASK_SET_ADDR   0x00400010
#define INTCNTL_IRQ_UNMASK_SET_OFFSET 0x00000010
#define INTCNTL_IRQ_UNMASK_SET_INDEX  0x00000004
#define INTCNTL_IRQ_UNMASK_SET_RESET  0x00000000
#define INTCNTL_IRQ_UNMASK_SET_COUNT  2

__INLINE uint32_t intcntl_irq_unmask_get(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 1);
    return REG_PL_RD(INTCNTL_IRQ_UNMASK_SET_ADDR + reg_idx * 4);
}

__INLINE void intcntl_irq_unmask_set(int reg_idx, uint32_t value)
{
    ASSERT_ERR(reg_idx <= 1);
    REG_PL_WR(INTCNTL_IRQ_UNMASK_SET_ADDR + reg_idx * 4, value);
}

/**
 * @brief IRQ_UNMASK_CLEAR register definition
 */
#define INTCNTL_IRQ_UNMASK_CLEAR_ADDR   0x00400018
#define INTCNTL_IRQ_UNMASK_CLEAR_OFFSET 0x00000018
#define INTCNTL_IRQ_UNMASK_CLEAR_INDEX  0x00000006
#define INTCNTL_IRQ_UNMASK_CLEAR_RESET  0x00000000
#define INTCNTL_IRQ_UNMASK_CLEAR_COUNT  2

__INLINE void intcntl_irq_unmask_clear(int reg_idx, uint32_t value)
{
    ASSERT_ERR(reg_idx <= 1);
    REG_PL_WR(INTCNTL_IRQ_UNMASK_CLEAR_ADDR + reg_idx * 4, value);
}

/**
 * @brief IRQ_POLARITY register definition
 */
#define INTCNTL_IRQ_POLARITY_ADDR   0x00400020
#define INTCNTL_IRQ_POLARITY_OFFSET 0x00000020
#define INTCNTL_IRQ_POLARITY_INDEX  0x00000008
#define INTCNTL_IRQ_POLARITY_RESET  0x00000000
#define INTCNTL_IRQ_POLARITY_COUNT  2

__INLINE uint32_t intcntl_irq_polarity_get(int reg_idx)
{
    ASSERT_ERR(reg_idx <= 1);
    return REG_PL_RD(INTCNTL_IRQ_POLARITY_ADDR + reg_idx * 4);
}

__INLINE void intcntl_irq_polarity_set(int reg_idx, uint32_t value)
{
    ASSERT_ERR(reg_idx <= 1);
    REG_PL_WR(INTCNTL_IRQ_POLARITY_ADDR + reg_idx * 4, value);
}

/**
 * @brief IRQ_INDEX register definition
 */
#define INTCNTL_IRQ_INDEX_ADDR   0x00400040
#define INTCNTL_IRQ_INDEX_OFFSET 0x00000040
#define INTCNTL_IRQ_INDEX_INDEX  0x00000010
#define INTCNTL_IRQ_INDEX_RESET  0x00000000

__INLINE uint32_t intcntl_irq_index_get(void)
{
    return REG_PL_RD(INTCNTL_IRQ_INDEX_ADDR);
}


#endif // _REG_INTCNTL_H_

