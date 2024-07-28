#ifndef _REG_COUNTER_H_
#define _REG_COUNTER_H_

#include <stdint.h>
#include "_reg_counter.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_COUNTER_COUNT 4

#define REG_COUNTER_DECODING_MASK 0x0000000F

/**
 * @brief COUNTER_VALUE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00                VALUE   0x0
 * </pre>
 */
#define COUNTER_COUNTER_VALUE_ADDR   0x00404000
#define COUNTER_COUNTER_VALUE_OFFSET 0x00000000
#define COUNTER_COUNTER_VALUE_INDEX  0x00000000
#define COUNTER_COUNTER_VALUE_RESET  0x00000000

__INLINE uint32_t counter_counter_value_get(void)
{
    return REG_PL_RD(COUNTER_COUNTER_VALUE_ADDR);
}

__INLINE void counter_counter_value_set(uint32_t value)
{
    REG_PL_WR(COUNTER_COUNTER_VALUE_ADDR, value);
}

// field definitions
#define COUNTER_VALUE_MASK   ((uint32_t)0xFFFFFFFF)
#define COUNTER_VALUE_LSB    0
#define COUNTER_VALUE_WIDTH  ((uint32_t)0x00000020)

#define COUNTER_VALUE_RST    0x0

__INLINE uint32_t counter_counter_value_value_getf(void)
{
    uint32_t localVal = REG_PL_RD(COUNTER_COUNTER_VALUE_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void counter_counter_value_value_setf(uint32_t value)
{
    ASSERT_ERR((((uint32_t)value << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(COUNTER_COUNTER_VALUE_ADDR, (uint32_t)value << 0);
}

/**
 * @brief COUNTER_RELOAD_VALUE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00               RELOAD   0x0
 * </pre>
 */
#define COUNTER_COUNTER_RELOAD_VALUE_ADDR   0x00404004
#define COUNTER_COUNTER_RELOAD_VALUE_OFFSET 0x00000004
#define COUNTER_COUNTER_RELOAD_VALUE_INDEX  0x00000001
#define COUNTER_COUNTER_RELOAD_VALUE_RESET  0x00000000

__INLINE uint32_t counter_counter_reload_value_get(void)
{
    return REG_PL_RD(COUNTER_COUNTER_RELOAD_VALUE_ADDR);
}

__INLINE void counter_counter_reload_value_set(uint32_t value)
{
    REG_PL_WR(COUNTER_COUNTER_RELOAD_VALUE_ADDR, value);
}

// field definitions
#define COUNTER_RELOAD_MASK   ((uint32_t)0xFFFFFFFF)
#define COUNTER_RELOAD_LSB    0
#define COUNTER_RELOAD_WIDTH  ((uint32_t)0x00000020)

#define COUNTER_RELOAD_RST    0x0

__INLINE uint32_t counter_counter_reload_value_reload_getf(void)
{
    uint32_t localVal = REG_PL_RD(COUNTER_COUNTER_RELOAD_VALUE_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void counter_counter_reload_value_reload_setf(uint32_t reload)
{
    ASSERT_ERR((((uint32_t)reload << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(COUNTER_COUNTER_RELOAD_VALUE_ADDR, (uint32_t)reload << 0);
}

/**
 * @brief COUNTER_EXPIRED_FLAG register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00              EXPIRED   0
 * </pre>
 */
#define COUNTER_COUNTER_EXPIRED_FLAG_ADDR   0x00404008
#define COUNTER_COUNTER_EXPIRED_FLAG_OFFSET 0x00000008
#define COUNTER_COUNTER_EXPIRED_FLAG_INDEX  0x00000002
#define COUNTER_COUNTER_EXPIRED_FLAG_RESET  0x00000000

__INLINE uint32_t counter_counter_expired_flag_get(void)
{
    return REG_PL_RD(COUNTER_COUNTER_EXPIRED_FLAG_ADDR);
}

__INLINE void counter_counter_expired_flag_set(uint32_t value)
{
    REG_PL_WR(COUNTER_COUNTER_EXPIRED_FLAG_ADDR, value);
}

// field definitions
#define COUNTER_EXPIRED_BIT    ((uint32_t)0x00000001)
#define COUNTER_EXPIRED_POS    0

#define COUNTER_EXPIRED_RST    0x0

__INLINE uint8_t counter_counter_expired_flag_expired_getf(void)
{
    uint32_t localVal = REG_PL_RD(COUNTER_COUNTER_EXPIRED_FLAG_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x00000001)) == 0);
    return (localVal >> 0);
}

__INLINE void counter_counter_expired_flag_expired_setf(uint8_t expired)
{
    ASSERT_ERR((((uint32_t)expired << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(COUNTER_COUNTER_EXPIRED_FLAG_ADDR, (uint32_t)expired << 0);
}

/**
 * @brief COUNTER_INT_MASK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00                 MASK   0
 * </pre>
 */
#define COUNTER_COUNTER_INT_MASK_ADDR   0x0040400C
#define COUNTER_COUNTER_INT_MASK_OFFSET 0x0000000C
#define COUNTER_COUNTER_INT_MASK_INDEX  0x00000003
#define COUNTER_COUNTER_INT_MASK_RESET  0x00000000

__INLINE uint32_t counter_counter_int_mask_get(void)
{
    return REG_PL_RD(COUNTER_COUNTER_INT_MASK_ADDR);
}

__INLINE void counter_counter_int_mask_set(uint32_t value)
{
    REG_PL_WR(COUNTER_COUNTER_INT_MASK_ADDR, value);
}

// field definitions
#define COUNTER_MASK_BIT    ((uint32_t)0x00000001)
#define COUNTER_MASK_POS    0

#define COUNTER_MASK_RST    0x0

__INLINE uint8_t counter_counter_int_mask_mask_getf(void)
{
    uint32_t localVal = REG_PL_RD(COUNTER_COUNTER_INT_MASK_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0x00000001)) == 0);
    return (localVal >> 0);
}

__INLINE void counter_counter_int_mask_mask_setf(uint8_t mask)
{
    ASSERT_ERR((((uint32_t)mask << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(COUNTER_COUNTER_INT_MASK_ADDR, (uint32_t)mask << 0);
}


#endif // _REG_COUNTER_H_

