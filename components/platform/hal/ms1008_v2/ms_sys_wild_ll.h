/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_sys_wild_ll.h
 * @brief Header file of sys_wild  module.
 * @author haijun.mai
 * @date   2022-03-25
 * @version 1.0
 * @Revision
 */

#ifndef MS_SYS_WILD_LL_H_
#define MS_SYS_WILD_LL_H_

#include <ms1008.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
/*Enable WDT Clock*/
#define __MS_SYS_LL_ENABLE_WDT()            do { \
                                                SET_BIT(SYS_CTRL->CPU_DBG, 1UL<<1);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->CPU_DBG, 1UL<<1);\
                                                UNUSED(tmpreg); \
                                              } while(0U)



/*Enable IR1 Clock*/
#define __MS_SYS_LL_ENABLE_IR1()            do { \
                                                SET_BIT(SYS_CTRL->PERI_SEL, 1UL<<0);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->PERI_SEL, 1UL<<0);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

/*Enable IR1 Clock*/
#define __MS_SYS_LL_ENABLE_IR0()            do { \
                                                CLEAR_BIT(SYS_CTRL->PERI_SEL, 1UL<<0);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->PERI_SEL, 0UL<<0);\
                                                UNUSED(tmpreg); \
                                              } while(0U)




/*Enable TO2 IR1 REQ*/
#define __MS_SYS_LL_ENABLE_IR1_DMAC_I2S1_RX()            do { \
                                                CLEAR_BIT(SYS_CTRL->PERI_SEL, 3UL<<2);\
                                                SET_BIT(SYS_CTRL->PERI_SEL, 1UL<<2);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->PERI_SEL, 1UL<<2);\
                                                UNUSED(tmpreg); \
                                              } while(0U)

/*Enable TO2 IR1 REQ*/
#define __MS_SYS_LL_ENABLE_IR1_DMAC_I2S1_TX()            do { \
                                                CLEAR_BIT(SYS_CTRL->PERI_SEL, 3UL<<2);\
                                                SET_BIT(SYS_CTRL->PERI_SEL, 1UL<<3);\
                                                /* Delay after active the soft reset */\
                                                volatile uint32_t tmpreg = READ_BIT(SYS_CTRL->PERI_SEL, 1UL<<3);\
                                                UNUSED(tmpreg); \
                                              } while(0U)


#ifdef __cplusplus
}
#endif

#endif /* MS_SYS_WILD_LL_H_ */

