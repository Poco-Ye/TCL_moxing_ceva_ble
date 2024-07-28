/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_ir.h
 * @brief Header file of ir  module.
 * @author haijun.mai
 * @date   2022-03-29
 * @version 1.0
 * @Revision
 */

#ifndef MS_IR_H_
#define MS_IR_H_

#include <ms1008.h>
#include "ms_ir1_hal.h"
#include "ms_sys_ctrl_regs.h"
#include <stdint.h>
#include <stdbool.h>
#include <ms_single_dmac.h>
#ifdef __cplusplus
extern "C" {
#endif


#define IR1_TRUE     1
#define IR1_FAULT   0
#define IR1_ERROR_INVALID_CALLBACK  0x00000001U   /*!< Invalid Callback error  */
#define IR1_STATUS_IDLE   0
#define IR1_STATUS_BUSY   1
typedef struct
{
    volatile uint32_t status;
    uint32_t *tx_ptr;  
    uint32_t tx_len;
    uint32_t tx_total_len; 
	
} Ir1Interrupt_Type;

typedef struct {

	uint32_t carry_high_cycle;
	uint32_t carry_low_cycle;
	uint32_t start_carry_cnt;
	uint32_t start_idle_cnt;

	uint32_t data0_idle_cnt;

	uint32_t data1_idle_cnt;
       uint32_t end_data0_data1_carry_cnt;
	uint32_t end_idle_cnt;
	uint32_t repeat_carry_cnt;

       uint32_t carry_compensation_mode;
	uint32_t proto_mode;
       uint32_t dma_enable; /**/
	uint32_t arbitrary_data_mode;
} Ir1Init_Type;

struct __Ir1Handle_Type;

/**
 * @brief  Ir callback handle Structure definition
 */
typedef struct {
	void (*error_callback)(struct __Ir1Handle_Type *ir);
	void (*init_callback)(struct __Ir1Handle_Type *ir);
	void (*deinit_callback)(struct __Ir1Handle_Type *ir);
	void (*ir_reach_callback)(struct __Ir1Handle_Type *ir);
} Ir1Callback_Type;

typedef struct __Ir1Handle_Type {
	Ir1_Type *instance;
	Ir1Init_Type init;/*!< i2s communication parameters      */
	uint32_t error_code; /*!< i2s Error code*/
	Ir1Interrupt_Type interrupt;
	IRQn_Type irq;
	Ir1Callback_Type *p_callback;
	DmacHandle_Type *hdmatx;            /*!< Pointer to the TX DMA              */
} Ir1Handle_Type;


void ms_ir1_init_dma_tx(DmacHandle_Type **phdmatx);

/**
 * @brief enable ir interrupt
 * @param  Ir_Type *ir: 
 * @retval None
 */
int32_t ms_ir1_enable_cpu_interrupt(Ir1Handle_Type *ir) ;



/**
 * @brief disable ir interrupt
 * @param  Ir_Type *ir: 
 * @retval None
 */
int32_t ms_ir1_disable_cpu_interrupt(Ir1Handle_Type *ir);



/**
 * @brief ir module init
 * @param  IrHandle_Type *ir: 
 * @retval None
 */
int32_t ms_ir1_init(Ir1Handle_Type *ir);




int32_t ms_ir1_send_data(Ir1Handle_Type *ir,uint32_t tx_data,uint32_t size);




int32_t ms_ir1_send_data_repeat(Ir1Handle_Type *ir,uint32_t tx_data,uint32_t size) ;

int32_t ms_ir1_send_data_repeat_stop(Ir1Handle_Type *ir);


/**
 * @brief  Deinitial Ir Controller
 * @param[in]  IrHandle_Type *ir:  Pointer to a IrHandle_Type  structure that contains
 * @ the configuration information for the specified Ir module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t ms_ir1_deinit(Ir1Handle_Type *ir);

/**
 * @brief unmask ir interrupt
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t interrupt_source
 * @retval None
 */
int32_t  ms_ir1_interrupt_unmask(Ir1Handle_Type* ir,uint32_t interrupt_source);



/**
 * @brief mask ir interrupt
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t interrupt_source
 * @retval None
 */
int32_t  ms_ir1_interrupt_mask(Ir1Handle_Type* ir,uint32_t interrupt_source);



int32_t ms_ir1_arbitrary_send_data(Ir1Handle_Type *ir,uint32_t *tx_data, uint32_t size);


int32_t ms_ir1_arbitrary_interrupt_send_data(Ir1Handle_Type *ir,uint32_t *tx_data, uint32_t size);

int32_t ms_ir1_arbitrary_dma_send_data(Ir1Handle_Type *ir,uint32_t *tx_data, uint32_t size) ;

/**
 * @brief register ir handler
 * @param  IrHandle_Type *ir   
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t  ms_ir1_register_handler(Ir1Handle_Type *ir);


/**
 * @brief unregister ir handler
 * @param  none 
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
int32_t  ms_ir1_unregister_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* MS_IR_H_ */

