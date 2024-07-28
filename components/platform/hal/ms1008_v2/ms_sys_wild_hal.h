/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_sys_wild_hal.h
 * @brief Header file of sys_wild  module.
 * @author haijun.mai
 * @date   2022-03-25
 * @version 1.0
 * @Revision
 */

#ifndef MS_SYS_WILD_HAL_H_
#define MS_SYS_WILD_HAL_H_

#include <ms1008.h>
#include <stdint.h>
#include <stdbool.h>
#include "ms_sys_wild_ll.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MS_SYS_HAL_ENABLE_WDT()               __MS_SYS_LL_ENABLE_WDT()

#define MS_SYS_HAL_ENABLE_IR0()               __MS_SYS_LL_ENABLE_IR0()

#define MS_SYS_HAL_ENABLE_IR1()               __MS_SYS_LL_ENABLE_IR1()

#define MS_SYS_HAL_ENABLE_IR1_DMA_RREQ()    __MS_SYS_LL_ENABLE_IR1_DMAC_I2S1_RX() 
#define MS_SYS_HAL_ENABLE_IR1_DMA_TREQ()    __MS_SYS_LL_ENABLE_IR1_DMAC_I2S1_TX() 
#ifdef __cplusplus
}
#endif

#endif /* MS_SYS_WILD_HAL_H_ */

