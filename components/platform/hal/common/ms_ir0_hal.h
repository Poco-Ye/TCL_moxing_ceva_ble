/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_ir0_hal.h
 * @brief Header file of ir  module.
 * @author haijun.mai
 * @date   2022-04-25
 * @version 1.0
 * @Revision
 */

#ifndef MS_IR0_HAL_H_
#define MS_IR0_HAL_H_

#include "ms_ir0_ll.h"

/**
 * @brief enable ir0 controller 
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
#define  ms_ir0_tx_enable_hal(ir)           ms_ir0_tx_enable_ll(ir)

/**
 * @brief disable ir0 controller 
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
#define ms_ir0_tx_disable_hal(ir)        ms_ir0_tx_disable_ll(ir)

/**
 * @brief enable ir tx abort
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
#define  ms_ir0_tx_abort_enable_hal(ir)     ms_ir0_tx_abort_enable_ll(ir)  


/**
 * @brief disable ir tx ablrt 
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
#define  ms_ir0_tx_abort_disable_hal(ir)    ms_ir0_tx_abort_disable_ll(ir)


/**
 * @brief enable ir preamble send
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
#define  ms_ir0_tx_preamble_enable_hal(ir)    ms_ir0_tx_preamble_enable_ll(ir)

/**
 * @brief disable ir preamble  send
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
#define  ms_ir0_tx_preamble_disable_hal(ir)  ms_ir0_tx_preamble_disable_ll(ir) 


/**
 * @brief enable ir data send
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
#define ms_ir0_tx_data_enable_hal(ir)   ms_ir0_tx_data_enable_ll(ir)

/**
 * @brief disable ir data send 
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
#define  ms_ir0_tx_data_disable_hal(ir)     ms_ir0_tx_data_disable_ll(ir)


/**
 * @brief enable ir end send
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
#define  ms_ir0_tx_end_enable_hal(ir)       ms_ir0_tx_end_enable_ll(ir)  


/**
 * @brief disable ir end send 
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
#define ms_ir0_tx_end_disable_hal(ir)    ms_ir0_tx_end_disable_ll(ir)

/**
 * @brief enable ir repeat send
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
#define ms_ir0_tx_repeat_enable_hal(ir)    ms_ir0_tx_repeat_enable_ll(ir)

/**
 * @brief disable ir repeat send 
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
#define ms_ir0_tx_repeat_disable_hal(ir)     ms_ir0_tx_repeat_disable_ll(ir)


/**
 * @brief config ir proto mode
 * @param Ir0_Type* ir: ir regs     
 * @param uint8_t proto_mode
 * @param   This parameter can be one of the following values:
 * @param IR0_PROCOTOL_MODE_USER_DEFINE
 * @param IR0_PROCOTOL_MODE_NEC
 * @param IR0_PROCOTOL_MODE_RCA
 * @retval None
 */
#define ms_ir0_config_proto_mode_hal(ir, proto_mode)      ms_ir0_config_proto_mode_ll(ir, proto_mode)


/**
 * @brief config ir tx data number bit
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t tx_len
 * @retval None
 */
#define ms_ir0_set_tx_num_bit_hal(ir, tx_num_bit)       ms_ir0_set_tx_num_bit_ll( ir, tx_num_bit)


/**
 * @brief enable ir interrupt
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
#define  ms_ir0_tx_interrupt_enable_hal( ir)       ms_ir0_tx_interrupt_enable_ll( ir)

/**
 * @brief disable ir interruptt 
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
#define ms_ir0_tx_interrupt_disable_hal( ir)     ms_ir0_tx_interrupt_disable_ll( ir)


/**
 * @brief config ir tx carry high cycle
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t carry_high_cycle
 * @retval None
 */
#define ms_ir0_tx_carry_high_cycle_config_hal( ir,carry_high_cycle)   ms_ir0_tx_carry_high_cycle_config_ll( ir,carry_high_cycle)


/**
 * @brief config ir tx carry low cycle
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t carry_low_cycle
 * @retval None
 */
#define ms_ir0_tx_carry_low_cycle_config_hal(ir, carry_low_cycle)     ms_ir0_tx_carry_low_cycle_config_ll(ir, carry_low_cycle)


/**
 * @brief config ir preamble carry count
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t carry_cnt
 * @retval None
 */
#define ms_ir0_tx_config_preamble_carry_cnt_hal(ir, carry_cnt)     ms_ir0_tx_config_preamble_carry_cnt_ll(ir, carry_cnt) 

/**
 * @brief config ir  preamble idle count
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t idle_cnt
 * @retval None
 */
#define  ms_ir0_tx_config_preamble_idle_cnt_hal( ir, idle_cnt)   ms_ir0_tx_config_preamble_idle_cnt_ll( ir, idle_cnt)


/**
 * @brief config ir  data0 count
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t carry_cnt
 * @retval None
 */
#define  ms_ir0_tx_config_data0_carry_cnt_hal(ir, carry_cnt)   ms_ir0_tx_config_data0_carry_cnt_ll(ir, carry_cnt)


/**
 * @brief config ir   data0 count
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t idle_cnt
 * @retval None
 */
#define ms_ir0_tx_config_data0_idle_cnt_hal( ir, idle_cnt)    ms_ir0_tx_config_data0_idle_cnt_ll( ir, idle_cnt)


/**
 * @brief config ir  data1 count
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t carry_cnt
 * @retval None
 */
#define ms_ir0_tx_config_data1_carry_cnt_hal(ir, carry_cnt)    ms_ir0_tx_config_data1_carry_cnt_ll(ir, carry_cnt)


/**
 * @brief config ir   data1 count
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t idle_cnt
 * @retval None
 */
#define ms_ir0_tx_config_data1_idle_cnt_hal( ir, idle_cnt)    ms_ir0_tx_config_data1_idle_cnt_ll( ir, idle_cnt)  


/**
 * @brief config ir  end carry count
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t carry_cnt
 * @retval None
 */
#define  ms_ir0_tx_config_end_carry_cnt_hal(ir, carry_cnt)     ms_ir0_tx_config_end_carry_cnt_ll(ir, carry_cnt) 

/**
 * @brief config ir end idle count
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t idle_cnt
 * @retval None
 */
#define  ms_ir0_tx_config_end_idle_cnt_hal( ir, idle_cnt)    ms_ir0_tx_config_end_idle_cnt_ll( ir, idle_cnt)


/**
 * @brief config ir end idle count
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t idle_cnt
 * @retval None
 */
#define  ms_ir0_tx_config_repeat_cycle_hal( ir, repeat_cycle)    ms_ir0_tx_config_repeat_cycle_ll( ir, repeat_cycle) 

/**
 * @brief write ir tx data to data reg
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t tx_data
 * @retval None
 */
#define  ms_ir0_tx_data_fill_hal(ir, tx_data)  ms_ir0_tx_data_fill_ll(ir, tx_data)

/**
 * @brief get ir status
 * @param Ir0_Type* ir: ir regs     
 * @retval status
 */
#define ms_ir0_get_status_hal(ir)     ms_ir0_get_status_ll(ir)

/**
 * @brief clear ir done status
 * @param Ir0_Type* ir: ir regs     
 * @retval none
 */
#define ms_ir0_clear_tx_done_status_hal( ir)    ms_ir0_clear_tx_done_status_ll( ir)  

#endif/* MS_IR_HAL_H_ */


