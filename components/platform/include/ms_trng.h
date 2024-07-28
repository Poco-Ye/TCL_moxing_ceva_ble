/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file  ms_trng_hal.h
 * @brief
 * @author haijun.mai
 * @date 2022-01-06
 * @version 1.0
 * @Revision
 */

#ifndef MS_TRNG_H_
#define MS_TRNG_H_

#include "ms1008.h"
#include "ms_trng_ll.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif



#define RND_SRC_SEL0         (0)
#define RND_SRC_SEL1         (1)
#define RND_SRC_SEL2         (2)
#define RND_SRC_SEL3         (3)



#define TRNG_ERROR_INVALID_CALLBACK  0x00000001U   /*!< Invalid Callback error  */


typedef struct
{
    uint32_t sample_rate;
    uint32_t rnd_src_sel;
    uint32_t debug_control_sel;
    uint8_t data_x;
} TrngInit_Type;

struct __TrngHandle_Type;

/**
  * @brief  Timer callback handle Structure definition
  */
typedef struct
{
    void (* error_callback)                       (struct __TrngHandle_Type *trng);
    void (* init_callback)                          (struct __TrngHandle_Type *trng);
    void (* deinit_callback)                      (struct __TrngHandle_Type *trng);
    void (* trng_reach_callback)    (struct __TrngHandle_Type *trng);
}TrngCallback_Type;


typedef struct __TrngHandle_Type
{
    Trng_Type           *instance;
    TrngInit_Type                init;/*!< Timer communication parameters      */
    uint32_t                           error_code;         /*!< TIMER Error code*/      
    IRQn_Type                      irq;
    TrngCallback_Type         *p_callback;
}TrngHandle_Type;

extern void ms_trng_enable_cpu_interrupt(TrngHandle_Type *trng);
extern void ms_trng_disable_cpu_interrupt(TrngHandle_Type *trng);
extern uint32_t ms_trng_init(TrngHandle_Type *trng);
extern uint32_t ms_trng_get_data(TrngHandle_Type *trng);
extern void ms_trng_deinit(TrngHandle_Type *trng);
extern void ms_trng_irq_handler(TrngHandle_Type *trng);
extern void ms_trng_clear_interrupt_status(TrngHandle_Type *trng,uint32_t interrupt_clear);
extern void ms_trng_interrupt_mask(TrngHandle_Type *trng,uint32_t interrupt_mask);
extern void ms_trng_interrupt_unmask(TrngHandle_Type *trng,uint32_t interrupt_mask);

#ifdef __cplusplus
}
#endif

#endif /* MS_TRNG_ */

