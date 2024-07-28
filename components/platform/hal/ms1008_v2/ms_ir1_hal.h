
/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_ir_hal.h
 * @brief Header file of ir  module.
 * @author haijun.mai
 * @date   2022-03-08
 * @version 1.0
 * @Revision
 */

#ifndef MS_IR1_HAL_H_
#define MS_IR1_HAL_H_

#include "ms_ir1_ll.h"


/**
 * @brief enable ir controller 
 * @param  Ir_Type* ir: ir regs                    
 * @retval None
 */
#define ms_ir1_tx_enable_hal(ir)     ms_ir1_tx_enable_ll(ir)


/**
 * @brief disable ir controller 
 * @param  Ir_Type* ir: ir regs                    
 * @retval None
 */
#define ms_ir1_tx_disable_hal(ir)     ms_ir1_tx_disable_ll(ir)


/**
 * @brief enable ir dma interface 
 * @param  Ir_Type* ir: ir regs                    
 * @retval None
 */
#define ms_ir1_tx_dma_enable_hal(ir)    ms_ir1_tx_dma_enable_ll(ir)


/**
 * @brief disable ir dma interface  
 * @param  Ir_Type* ir: ir regs                    
 * @retval None
 */
#define ms_ir1_tx_dma_disable_hal(ir)    ms_ir1_tx_dma_disable_ll(ir) 


/**
 * @brief enable ir interrupt
 * @param  Ir_Type* ir: ir regs                    
 * @retval None
 */
#define  ms_ir1_tx_int_enable_hal(ir)    ms_ir1_tx_int_enable_ll(ir)


/**
 * @brief disable ir interruptt 
 * @param  Ir_Type* ir: ir regs                    
 * @retval None
 */
#define  ms_ir1_tx_int_disable_hal(ir)    ms_ir1_tx_int_disable_ll(ir)


/**
 * @brief config ir proto mode
 * @param Ir_Type* ir: ir regs     
 * @param uint8_t proto_mode
 * @param   This parameter can be one of the following values:
 * @param IR_PROCOTOL_MODE_USER_DEFINE
 * @param IR_PROCOTOL_MODE_NEC
 * @param IR_PROCOTOL_MODE_RCA
 * @retval None
 */
#define  ms_ir1_config_proto_mode_hal(ir, proto_mode)     ms_ir1_config_proto_mode_ll(ir, proto_mode)



/**
 * @brief enable ir proto repeat 
 * @param Ir_Type* ir: ir regs     
 * @retval None
 */
#define ms_ir1_proto_repeat_enable_hal(ir)    ms_ir1_proto_repeat_enable_ll(ir)


/**
 * @brief config ir fifo data mode
 * @param Ir_Type* ir: ir regs     
 * @retval None
 */
#define ms_ir1_fifo_data_level_enable_hal(ir)     ms_ir1_fifo_data_level_enable_ll(ir) 


/**
 * @brief config ir fifo data mode
 * @param Ir_Type* ir: ir regs     
 * @retval None
 */
#define ms_ir1_fifo_data_level_disable_hal(ir)     ms_ir1_fifo_data_level_disable_ll(ir)


/**
 * @brief config ir carry compensation mode
 * @param Ir_Type* ir: ir regs     
 * @param  uint32_t carry_compensation_mode
 * @retval None
 */
#define ms_ir1_config_carry_compensation_mode_hal( ir, carry_compensation_mode)   ms_ir1_config_carry_compensation_mode_ll( ir, carry_compensation_mode)


/**
 * @brief config ir tx data length
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t tx_len
 * @retval None
 */
#define ms_ir1_set_tx_length_hal(ir,tx_len)    ms_ir1_set_tx_length_ll(ir,tx_len)


/**
 * @brief config ir tx watermask
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t watermask
 * @retval None
 */
#define ms_ir1_set_tx_watermask_hal(ir, watermask)   ms_ir1_set_tx_watermask_ll(ir, watermask)



/**
 * @brief config ir tx carry high cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t carry_high_cycle
 * @retval None
 */
#define ms_ir1_tx_carry_high_cycle_config_hal(ir, carry_high_cycle)    ms_ir1_tx_carry_high_cycle_config_ll(ir, carry_high_cycle)




/**
 * @brief config ir tx carry low cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t carry_low_cycle
 * @retval None
 */
#define  ms_ir1_tx_carry_low_cycle_config_hal(ir, carry_low_cycle)    ms_ir1_tx_carry_low_cycle_config_ll(ir, carry_low_cycle)



/**
 * @brief config ir tx level cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t tx_level_cycle
 * @retval None
 */
#define ms_ir1_tx_level_cycle_config_hal( ir, tx_level_cycle)    ms_ir1_tx_level_cycle_config_ll( ir, tx_level_cycle)


/**
 * @brief config ir tx level polarity
 * @param Ir_Type* ir: ir regs     
 * @param uint8_t tx_level_cycle
 * @retval None
 */
#define ms_ir1_tx_level_polarity_config_hal(ir, level_polarity)    ms_ir1_tx_level_polarity_config_ll(ir, level_polarity)


/**
 * @brief write ir tx data to data reg
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t tx_data
 * @retval None
 */
#define ms_ir1_tx_data_fill_hal( ir, tx_data)    ms_ir1_tx_data_fill_ll( ir, tx_data)


/**
 * @brief config ir proto start high cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t high_cycle
 * @retval None
 */
#define ms_ir1_config_proto_start_bit_high_cycle_hal(ir, high_cycle)     ms_ir1_config_proto_start_bit_high_cycle_ll(ir, high_cycle)      


/**
 * @brief config ir proto start low cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t low_cycle
 * @retval None
 */
#define ms_ir1_config_proto_start_bit_low_cycle_hal( ir, low_cycle)     ms_ir1_config_proto_start_bit_low_cycle_ll( ir, low_cycle)


/**
 * @brief config ir proto data high cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t high_cycle
 * @retval None
 */
#define ms_ir1_config_proto_data0_bit_low_cycle_hal( ir, high_cycle)     ms_ir1_config_proto_data0_bit_low_cycle_ll( ir, high_cycle)  


/**
 * @brief config ir proto data low cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t low_cycle
 * @retval None
 */
#define  ms_ir1_config_proto_data1_bit_low_cycle_hal(ir, low_cycle)    ms_ir1_config_proto_data1_bit_low_cycle_ll(ir, low_cycle)


/**
 * @brief config ir proto end high cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t high_cycle
 * @retval None
 */
#define  ms_ir1_config_proto_data_and_end_bit_high_cycle_hal( ir, high_cycle)    ms_ir1_config_proto_data_and_end_bit_high_cycle_ll( ir, high_cycle)


/**
 * @brief config ir proto end low cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t low_cycle
 * @retval None
 */
#define  ms_ir1_config_proto_end_bit_low_cycle_hal( ir, low_cycle)      ms_ir1_config_proto_end_bit_low_cycle_ll( ir, low_cycle)    




/**
 * @brief config ir proto end low cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t low_cycle
 * @retval None
 */
#define ms_ir1_config_proto_repeat_end_bit_low_cycle_hal( ir, low_cycle)      ms_ir1_config_proto_repeat_end_bit_low_cycle_ll( ir, low_cycle)


/**
 * @brief unmask ir interrupt
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t interrupt_source
 * @retval None
 */
#define ms_ir1_interrupt_unmask_hal(ir, interrupt_source)    ms_ir1_interrupt_unmask_ll(ir, interrupt_source)



/**
 * @brief mask ir interrupt
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t interrupt_source
 * @retval None
 */
#define  ms_ir1_interrupt_mask_hal(ir, interrupt_source)    ms_ir1_interrupt_mask_ll(ir, interrupt_source)


/**
 * @brief get ir interrupt status
 * @param Ir_Type* ir: ir regs     
 * @retval interrupt status
 */
#define  ms_ir1_get_interrupt_status_hal(ir)     ms_ir1_get_interrupt_status_ll(ir)


/**
 * @brief get ir status
 * @param Ir_Type* ir: ir regs     
 * @retval status
 */
#define  ms_ir1_get_status_hal(ir)    ms_ir1_get_status_ll(ir)



/**
 * @brief flash ir fifo
 * @param Ir_Type* ir: ir regs     
 * @retval none
 */
#define  ms_ir1_flush_fifo_hal(ir)     ms_ir1_flush_fifo_ll(ir)


/**
 * @brief clear ir status
 * @param Ir_Type* ir: ir regs     
 * @retval none
 */
#define   ms_ir1_clear_status_hal( ir, int_source)    ms_ir1_clear_status_ll( ir, int_source)


#endif/* MS_IR_HAL_H_ */