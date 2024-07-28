/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_ir.h
 * @brief Header file of ir  module.
 * @author haijun.mai
 * @date   2022-04-26
 * @version 1.0
 * @Revision
 */

#ifndef MS_IR0_H_
#define MS_IR0_H_

#include <ms1008.h>
#include "ms_ir0_hal.h"
#include "ms_sys_ctrl_regs.h"
#include <stdint.h>
#include <stdbool.h>
#include <ms_single_dmac.h>
#ifdef __cplusplus
extern "C" {
#endif


#define IR0_TRUE     1
#define IR0_FAULT   0
#define IR0_ERROR_INVALID_CALLBACK  0x00000001U   /*!< Invalid Callback error  */
#define IR0_STATUS_IDLE   0
#define IR0_STATUS_BUSY   1
typedef struct
{
    volatile uint32_t status;
    uint32_t *tx_ptr;  
    uint32_t tx_len;
    uint32_t tx_total_len; 
	
} Ir0Interrupt_Type;

typedef struct {

	uint32_t carry_high_cycle;
	uint32_t carry_low_cycle;
	uint32_t preamble_carry_cnt;
	uint32_t preamble_idle_cnt;
	uint32_t data0_carry_cnt;
	uint32_t data0_idle_cnt;
	uint32_t data1_carry_cnt;
	uint32_t data1_idle_cnt;
	uint32_t end_carry_cnt;
	uint32_t end_idle_cnt;
	uint32_t repeat_carry_cnt;
	uint32_t proto_mode;
  
} Ir0Init_Type;

struct __Ir0Handle_Type;

/**
 * @brief  Ir callback handle Structure definition
 */
typedef struct {
	void (*error_callback)(struct __Ir0Handle_Type *ir);
	void (*init_callback)(struct __Ir0Handle_Type *ir);
	void (*deinit_callback)(struct __Ir0Handle_Type *ir);
	void (*ir_reach_callback)(struct __Ir0Handle_Type *ir);
} Ir0Callback_Type;

typedef struct __Ir0Handle_Type {
	Ir0_Type *instance;
	Ir0Init_Type init;/*!< ir 0 parameters      */
	uint32_t error_code; /*!< ir Error code*/
	Ir0Interrupt_Type interrupt;
	IRQn_Type irq;
	Ir0Callback_Type *p_callback;
} Ir0Handle_Type;

/**
 * @brief enable ir interrupt
 * @param  Ir0Handle_Type *ir: 
 * @retval None
 */
int32_t ms_ir0_enable_cpu_interrupt(Ir0Handle_Type *ir) ;

/**
 * @brief disable ir interrupt
 * @param  Ir0Handle_Type *ir: 
 * @retval None
 */
int32_t ms_ir0_disable_cpu_interrupt(Ir0Handle_Type *ir);

/**
 * @brief ir module init
 * @param  Ir0Handle_Type *ir: 
 * @retval None
 */
int32_t ms_ir0_init(Ir0Handle_Type *ir);


/**
 * @brief   Ir Controller send data
 * @param[in]  Ir0Handle_Type *ir:  Pointer to a IrHandle_Type  structure that contains
 * @param[in]  uint32_t tx_data :the data that will be send by ir controller
 * @param[in]  uint32_t tx_data :the data's size(bit) that will be send by ir controller
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_ir0_send_data(Ir0Handle_Type *ir,uint32_t tx_data,uint32_t size);

/**
 * @brief   Ir Controller repeat send data
 * @param[in]  Ir0Handle_Type *ir:  Pointer to a IrHandle_Type  structure that contains
 * @param[in]  uint32_t tx_data :the data that will be repeat send by ir controller
 * @param[in]  uint32_t tx_data :the data's size(bit) that will be repeat  send by ir controller
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_ir0_send_data_repeat(Ir0Handle_Type *ir,uint32_t tx_data,uint32_t size) ;


/**
 * @brief ir module stop repeat send data
 * @param  Ir0Handle_Type *ir: 
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_ir0_send_data_repeat_stop(Ir0Handle_Type *ir);

/**
 * @brief   Ir interrupt send data
 * @param[in]  Ir0Handle_Type *ir:  Pointer to a IrHandle_Type  structure that contains
 * @param[in]  uint32_t tx_data :the data that will be send by ir controller
 * @param[in]  uint32_t tx_data :the data's size(bit) that will be send by ir controller
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_ir0_int_send_data(Ir0Handle_Type *ir,uint32_t tx_data, uint32_t size);

/**
 * @brief  Deinitial Ir Controller
 * @param[in]  Ir0Handle_Type *ir:  Pointer to a IrHandle_Type  structure that contains
 * @ the configuration information for the specified Ir module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_ir0_deinit(Ir0Handle_Type *ir);

/**
 * @brief unmask ir interrupt
 * @param Ir0Handle_Type* ir: ir regs     
 * @retval None
 */
int32_t  ms_ir0_interrupt_unmask(Ir0Handle_Type* ir);



/**
 * @brief mask ir interrupt
 * @param Ir0Handle_Type* ir: ir regs     
 * @retval None
 */
int32_t  ms_ir0_interrupt_mask(Ir0Handle_Type* ir);



/**
 * @brief register ir handler
 * @param  IrHandle_Type *ir   
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t  ms_ir0_register_handler(Ir0Handle_Type *ir);


/**
 * @brief unregister ir handler
 * @param  none 
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t  ms_ir0_unregister_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* MS_IR_H_ */

