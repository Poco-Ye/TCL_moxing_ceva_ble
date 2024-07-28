#ifndef _REG_GPIO_H_
#define _REG_GPIO_H_

#include <stdint.h>
#include "_reg_gpio.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_GPIO_COUNT 9

#define REG_GPIO_DECODING_MASK 0x0000003F

/**
 * @brief GPIO_OUT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00             GPIO_OUT   0x0
 * </pre>
 */
#define GPIO_GPIO_OUT_ADDR   0x00405000
#define GPIO_GPIO_OUT_OFFSET 0x00000000
#define GPIO_GPIO_OUT_INDEX  0x00000000
#define GPIO_GPIO_OUT_RESET  0x00000000

__INLINE uint32_t gpio_gpio_out_get(void)
{
    return REG_PL_RD(GPIO_GPIO_OUT_ADDR);
}

__INLINE void gpio_gpio_out_set(uint32_t value)
{
    REG_PL_WR(GPIO_GPIO_OUT_ADDR, value);
}

// field definitions
#define GPIO_GPIO_OUT_MASK   ((uint32_t)0xFFFFFFFF)
#define GPIO_GPIO_OUT_LSB    0
#define GPIO_GPIO_OUT_WIDTH  ((uint32_t)0x00000020)

#define GPIO_GPIO_OUT_RST    0x0

__INLINE uint32_t gpio_gpio_out_getf(void)
{
    uint32_t localVal = REG_PL_RD(GPIO_GPIO_OUT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void gpio_gpio_out_setf(uint32_t gpioout)
{
    ASSERT_ERR((((uint32_t)gpioout << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(GPIO_GPIO_OUT_ADDR, (uint32_t)gpioout << 0);
}

/**
 * @brief GPIO_IN register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00              GPIO_IN   0x0
 * </pre>
 */
#define GPIO_GPIO_IN_ADDR   0x00405004
#define GPIO_GPIO_IN_OFFSET 0x00000004
#define GPIO_GPIO_IN_INDEX  0x00000001
#define GPIO_GPIO_IN_RESET  0x00000000

__INLINE uint32_t gpio_gpio_in_get(void)
{
    return REG_PL_RD(GPIO_GPIO_IN_ADDR);
}

// field definitions
#define GPIO_GPIO_IN_MASK   ((uint32_t)0xFFFFFFFF)
#define GPIO_GPIO_IN_LSB    0
#define GPIO_GPIO_IN_WIDTH  ((uint32_t)0x00000020)

#define GPIO_GPIO_IN_RST    0x0

__INLINE uint32_t gpio_gpio_in_getf(void)
{
    uint32_t localVal = REG_PL_RD(GPIO_GPIO_IN_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief GPIO_DIR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00             GPIO_DIR   0x0
 * </pre>
 */
#define GPIO_GPIO_DIR_ADDR   0x00405008
#define GPIO_GPIO_DIR_OFFSET 0x00000008
#define GPIO_GPIO_DIR_INDEX  0x00000002
#define GPIO_GPIO_DIR_RESET  0x00000000

__INLINE uint32_t gpio_gpio_dir_get(void)
{
    return REG_PL_RD(GPIO_GPIO_DIR_ADDR);
}

__INLINE void gpio_gpio_dir_set(uint32_t value)
{
    REG_PL_WR(GPIO_GPIO_DIR_ADDR, value);
}

// field definitions
#define GPIO_GPIO_DIR_MASK   ((uint32_t)0xFFFFFFFF)
#define GPIO_GPIO_DIR_LSB    0
#define GPIO_GPIO_DIR_WIDTH  ((uint32_t)0x00000020)

#define GPIO_GPIO_DIR_RST    0x0

__INLINE uint32_t gpio_gpio_dir_getf(void)
{
    uint32_t localVal = REG_PL_RD(GPIO_GPIO_DIR_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void gpio_gpio_dir_setf(uint32_t gpiodir)
{
    ASSERT_ERR((((uint32_t)gpiodir << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(GPIO_GPIO_DIR_ADDR, (uint32_t)gpiodir << 0);
}

/**
 * @brief GPIO_INTMODE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00         GPIO_INTMODE   0x0
 * </pre>
 */
#define GPIO_GPIO_INTMODE_ADDR   0x0040500C
#define GPIO_GPIO_INTMODE_OFFSET 0x0000000C
#define GPIO_GPIO_INTMODE_INDEX  0x00000003
#define GPIO_GPIO_INTMODE_RESET  0x00000000

__INLINE uint32_t gpio_gpio_intmode_get(void)
{
    return REG_PL_RD(GPIO_GPIO_INTMODE_ADDR);
}

__INLINE void gpio_gpio_intmode_set(uint32_t value)
{
    REG_PL_WR(GPIO_GPIO_INTMODE_ADDR, value);
}

// field definitions
#define GPIO_GPIO_INTMODE_MASK   ((uint32_t)0xFFFFFFFF)
#define GPIO_GPIO_INTMODE_LSB    0
#define GPIO_GPIO_INTMODE_WIDTH  ((uint32_t)0x00000020)

#define GPIO_GPIO_INTMODE_RST    0x0

__INLINE uint32_t gpio_gpio_intmode_getf(void)
{
    uint32_t localVal = REG_PL_RD(GPIO_GPIO_INTMODE_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void gpio_gpio_intmode_setf(uint32_t gpiointmode)
{
    ASSERT_ERR((((uint32_t)gpiointmode << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(GPIO_GPIO_INTMODE_ADDR, (uint32_t)gpiointmode << 0);
}

/**
 * @brief GPIO_INTPOL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00          GPIO_INTPOL   0x0
 * </pre>
 */
#define GPIO_GPIO_INTPOL_ADDR   0x00405010
#define GPIO_GPIO_INTPOL_OFFSET 0x00000010
#define GPIO_GPIO_INTPOL_INDEX  0x00000004
#define GPIO_GPIO_INTPOL_RESET  0x00000000

__INLINE uint32_t gpio_gpio_intpol_get(void)
{
    return REG_PL_RD(GPIO_GPIO_INTPOL_ADDR);
}

__INLINE void gpio_gpio_intpol_set(uint32_t value)
{
    REG_PL_WR(GPIO_GPIO_INTPOL_ADDR, value);
}

// field definitions
#define GPIO_GPIO_INTPOL_MASK   ((uint32_t)0xFFFFFFFF)
#define GPIO_GPIO_INTPOL_LSB    0
#define GPIO_GPIO_INTPOL_WIDTH  ((uint32_t)0x00000020)

#define GPIO_GPIO_INTPOL_RST    0x0

__INLINE uint32_t gpio_gpio_intpol_getf(void)
{
    uint32_t localVal = REG_PL_RD(GPIO_GPIO_INTPOL_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void gpio_gpio_intpol_setf(uint32_t gpiointpol)
{
    ASSERT_ERR((((uint32_t)gpiointpol << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(GPIO_GPIO_INTPOL_ADDR, (uint32_t)gpiointpol << 0);
}

/**
 * @brief GPIO_INTEN register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00           GPIO_INTEN   0x0
 * </pre>
 */
#define GPIO_GPIO_INTEN_ADDR   0x00405014
#define GPIO_GPIO_INTEN_OFFSET 0x00000014
#define GPIO_GPIO_INTEN_INDEX  0x00000005
#define GPIO_GPIO_INTEN_RESET  0x00000000

__INLINE uint32_t gpio_gpio_inten_get(void)
{
    return REG_PL_RD(GPIO_GPIO_INTEN_ADDR);
}

__INLINE void gpio_gpio_inten_set(uint32_t value)
{
    REG_PL_WR(GPIO_GPIO_INTEN_ADDR, value);
}

// field definitions
#define GPIO_GPIO_INTEN_MASK   ((uint32_t)0xFFFFFFFF)
#define GPIO_GPIO_INTEN_LSB    0
#define GPIO_GPIO_INTEN_WIDTH  ((uint32_t)0x00000020)

#define GPIO_GPIO_INTEN_RST    0x0

__INLINE uint32_t gpio_gpio_inten_getf(void)
{
    uint32_t localVal = REG_PL_RD(GPIO_GPIO_INTEN_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void gpio_gpio_inten_setf(uint32_t gpiointen)
{
    ASSERT_ERR((((uint32_t)gpiointen << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(GPIO_GPIO_INTEN_ADDR, (uint32_t)gpiointen << 0);
}

/**
 * @brief GPIO_INTSTATRAW register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00      GPIO_INTSTATRAW   0x0
 * </pre>
 */
#define GPIO_GPIO_INTSTATRAW_ADDR   0x00405018
#define GPIO_GPIO_INTSTATRAW_OFFSET 0x00000018
#define GPIO_GPIO_INTSTATRAW_INDEX  0x00000006
#define GPIO_GPIO_INTSTATRAW_RESET  0x00000000

__INLINE uint32_t gpio_gpio_intstatraw_get(void)
{
    return REG_PL_RD(GPIO_GPIO_INTSTATRAW_ADDR);
}

// field definitions
#define GPIO_GPIO_INTSTATRAW_MASK   ((uint32_t)0xFFFFFFFF)
#define GPIO_GPIO_INTSTATRAW_LSB    0
#define GPIO_GPIO_INTSTATRAW_WIDTH  ((uint32_t)0x00000020)

#define GPIO_GPIO_INTSTATRAW_RST    0x0

__INLINE uint32_t gpio_gpio_intstatraw_getf(void)
{
    uint32_t localVal = REG_PL_RD(GPIO_GPIO_INTSTATRAW_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief GPIO_INTSTATMASKED register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00   GPIO_INTSTATMASKED   0x0
 * </pre>
 */
#define GPIO_GPIO_INTSTATMASKED_ADDR   0x0040501C
#define GPIO_GPIO_INTSTATMASKED_OFFSET 0x0000001C
#define GPIO_GPIO_INTSTATMASKED_INDEX  0x00000007
#define GPIO_GPIO_INTSTATMASKED_RESET  0x00000000

__INLINE uint32_t gpio_gpio_intstatmasked_get(void)
{
    return REG_PL_RD(GPIO_GPIO_INTSTATMASKED_ADDR);
}

// field definitions
#define GPIO_GPIO_INTSTATMASKED_MASK   ((uint32_t)0xFFFFFFFF)
#define GPIO_GPIO_INTSTATMASKED_LSB    0
#define GPIO_GPIO_INTSTATMASKED_WIDTH  ((uint32_t)0x00000020)

#define GPIO_GPIO_INTSTATMASKED_RST    0x0

__INLINE uint32_t gpio_gpio_intstatmasked_getf(void)
{
    uint32_t localVal = REG_PL_RD(GPIO_GPIO_INTSTATMASKED_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief GPIO_INTACK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00          GPIO_INTACK   0x0
 * </pre>
 */
#define GPIO_GPIO_INTACK_ADDR   0x00405020
#define GPIO_GPIO_INTACK_OFFSET 0x00000020
#define GPIO_GPIO_INTACK_INDEX  0x00000008
#define GPIO_GPIO_INTACK_RESET  0x00000000

__INLINE uint32_t gpio_gpio_intack_get(void)
{
    return REG_PL_RD(GPIO_GPIO_INTACK_ADDR);
}

__INLINE void gpio_gpio_intack_set(uint32_t value)
{
    REG_PL_WR(GPIO_GPIO_INTACK_ADDR, value);
}

// field definitions
#define GPIO_GPIO_INTACK_MASK   ((uint32_t)0xFFFFFFFF)
#define GPIO_GPIO_INTACK_LSB    0
#define GPIO_GPIO_INTACK_WIDTH  ((uint32_t)0x00000020)

#define GPIO_GPIO_INTACK_RST    0x0

__INLINE uint32_t gpio_gpio_intack_getf(void)
{
    uint32_t localVal = REG_PL_RD(GPIO_GPIO_INTACK_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void gpio_gpio_intack_setf(uint32_t gpiointack)
{
    ASSERT_ERR((((uint32_t)gpiointack << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(GPIO_GPIO_INTACK_ADDR, (uint32_t)gpiointack << 0);
}


#endif // _REG_GPIO_H_

