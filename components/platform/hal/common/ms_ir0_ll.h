/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_ir_ll.h
 * @brief Header file of ir0  module.
 * @author haijun.mai
 * @date   2022-04-25
 * @version 1.0
 * @Revision
 */

#ifndef MS_IR0_LL_H_
#define MS_IR0_LL_H_

#include "ms1008.h"
#include "ms_ir0_regs.h"
#include <stdint.h>
#include <stdbool.h>
#include "log.h"

#define IR0_PROCOTOL_MODE_USER_DEFINE    0
#define IR0_PROCOTOL_MODE_NEC    1
#define IR0_PROCOTOL_MODE_RCA    2



/**
 * @brief enable ir0 controller 
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir0_tx_enable_ll(Ir0_Type* ir)
{
	SET_BIT(ir->IR0_CTL,IR0_CTL_TX_EN); 
}

/**
 * @brief disable ir0 controller 
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir0_tx_disable_ll(Ir0_Type* ir)
{
	CLEAR_BIT(ir->IR0_CTL,IR0_CTL_TX_EN); 
}




/**
 * @brief enable ir tx abort
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir0_tx_abort_enable_ll(Ir0_Type* ir)
{
	SET_BIT(ir->IR0_TX_CTL,IR0_TX_CTL_TX_ABORT_EN); 
}

/**
 * @brief disable ir tx abort 
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir0_tx_abort_disable_ll(Ir0_Type* ir)
{
	CLEAR_BIT(ir->IR0_TX_CTL,IR0_TX_CTL_TX_ABORT_EN); 
}




/**
 * @brief enable ir preamble send
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir0_tx_preamble_enable_ll(Ir0_Type* ir)
{
	SET_BIT(ir->IR0_TX_CTL,IR0_TX_CTL_TX_PREAMBLE_EN); 
}

/**
 * @brief disable ir preamble  send
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir0_tx_preamble_disable_ll(Ir0_Type* ir)
{
	CLEAR_BIT(ir->IR0_TX_CTL,IR0_TX_CTL_TX_PREAMBLE_EN); 
}


/**
 * @brief enable ir data send
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir0_tx_data_enable_ll(Ir0_Type* ir)
{
	SET_BIT(ir->IR0_TX_CTL,IR0_TX_CTL_TX_DATA_EN); 
}

/**
 * @brief disable ir data send 
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir0_tx_data_disable_ll(Ir0_Type* ir)
{
	CLEAR_BIT(ir->IR0_TX_CTL,IR0_TX_CTL_TX_DATA_EN); 
}

/**
 * @brief enable ir end send
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir0_tx_end_enable_ll(Ir0_Type* ir)
{
	SET_BIT(ir->IR0_TX_CTL,IR0_TX_CTL_TX_END_EN); 
}

/**
 * @brief disable ir end send 
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir0_tx_end_disable_ll(Ir0_Type* ir)
{
	CLEAR_BIT(ir->IR0_TX_CTL,IR0_TX_CTL_TX_END_EN); 
}

/**
 * @brief enable ir repeat send
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir0_tx_repeat_enable_ll(Ir0_Type* ir)
{
	SET_BIT(ir->IR0_TX_CTL,IR0_TX_CTL_TX_REPEAT_EN); 
}

/**
 * @brief disable ir repeat send 
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir0_tx_repeat_disable_ll(Ir0_Type* ir)
{
	CLEAR_BIT(ir->IR0_TX_CTL,IR0_TX_CTL_TX_REPEAT_EN); 
}


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
static inline void ms_ir0_config_proto_mode_ll(Ir0_Type* ir,uint8_t proto_mode)
{
	MODIFY_REG(ir->IR0_TX_CTL,IR0_TX_CTL_TX_PROTOCOL,((proto_mode<<IR0_TX_CTL_TX_PROTOCOL_POS)&IR0_TX_CTL_TX_PROTOCOL_MASK)); 
}


/**
 * @brief config ir tx data number bit
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t tx_len
 * @retval None
 */
static inline void ms_ir0_set_tx_num_bit_ll(Ir0_Type* ir,uint32_t tx_num_bit)
{
	MODIFY_REG(ir->IR0_TX_CTL,IR0_TX_CTL_TX_DATA_NUM,((tx_num_bit<<IR0_TX_CTL_TX_DATA_NUM_POS)&IR0_TX_CTL_TX_DATA_NUM_MASK)); 
}

/**
 * @brief enable ir interrupt
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir0_tx_interrupt_enable_ll(Ir0_Type* ir)
{
	SET_BIT(ir->IR0_TX_CTL,IR0_TX_CTL_TX_INT_EN); 
}

/**
 * @brief disable ir interruptt 
 * @param  Ir0_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir0_tx_interrupt_disable_ll(Ir0_Type* ir)
{
	CLEAR_BIT(ir->IR0_TX_CTL,IR0_TX_CTL_TX_INT_EN); 
}

/**
 * @brief config ir tx carry high cycle
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t carry_high_cycle
 * @retval None
 */
static inline void ms_ir0_tx_carry_high_cycle_config_ll(Ir0_Type* ir,uint32_t carry_high_cycle)
{
      MODIFY_REG(ir->IR0_TX_CARRY_CFG,IR0_TX_CARRY_CFG_HIGH_CYCLE,((carry_high_cycle<<IR0_TX_CARRY_CFG_HIGH_CYCLE_POS)&IR0_TX_CARRY_CFG_HIGH_CYCLE_MASK)); 
}

/**
 * @brief config ir tx carry low cycle
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t carry_low_cycle
 * @retval None
 */
static inline void ms_ir0_tx_carry_low_cycle_config_ll(Ir0_Type* ir,uint32_t carry_low_cycle)
{
      	MODIFY_REG(ir->IR0_TX_CARRY_CFG,IR0_TX_CARRY_CFG_LOW_CYCLE,((carry_low_cycle<<IR0_TX_CARRY_CFG_LOW_CYCLE_POS)&IR0_TX_CARRY_CFG_LOW_CYCLE_MASK)); 
}	



/**
 * @brief config ir preamble carry count
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t carry_cnt
 * @retval None
 */
static inline void ms_ir0_tx_config_preamble_carry_cnt_ll(Ir0_Type* ir,uint32_t carry_cnt)
{
	MODIFY_REG(ir->IR0_TX_PREAMBLE_CFG,IR0_TX_PREAMBLE_CFG_CARRY_CNT,((carry_cnt<<IR0_TX_PREAMBLE_CFG_CARRY_CNT_POS)&IR0_TX_PREAMBLE_CFG_CARRY_CNT_MASK)); 
}



/**
 * @brief config ir  preamble idle count
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t idle_cnt
 * @retval None
 */
static inline void ms_ir0_tx_config_preamble_idle_cnt_ll(Ir0_Type* ir,uint32_t idle_cnt)
{
	MODIFY_REG(ir->IR0_TX_PREAMBLE_CFG,IR0_TX_PREAMBLE_CFG_IDLE_CNT,((idle_cnt<<IR0_TX_PREAMBLE_CFG_IDLE_CNT_POS)&IR0_TX_PREAMBLE_CFG_IDLE_CNT_MASK)); 
}



/**
 * @brief config ir  data0 count
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t carry_cnt
 * @retval None
 */
static inline void ms_ir0_tx_config_data0_carry_cnt_ll(Ir0_Type* ir,uint32_t carry_cnt)
{
	MODIFY_REG(ir->IR0_TX_DATA0_CFG,IR0_TX_DATA0_CFG_CARRY_CNT,((carry_cnt<<IR0_TX_DATA0_CFG_CARRY_CNT_POS)&IR0_TX_DATA0_CFG_CARRY_CNT_MASK)); 
}



/**
 * @brief config ir   data0 count
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t idle_cnt
 * @retval None
 */
static inline void ms_ir0_tx_config_data0_idle_cnt_ll(Ir0_Type* ir,uint32_t idle_cnt)
{
	MODIFY_REG(ir->IR0_TX_DATA0_CFG,IR0_TX_DATA0_CFG_IDLE_CNT,((idle_cnt<<IR0_TX_DATA0_CFG_IDLE_CNT_POS)&IR0_TX_DATA0_CFG_IDLE_CNT_MASK)); 
}



/**
 * @brief config ir  data1 count
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t carry_cnt
 * @retval None
 */
static inline void ms_ir0_tx_config_data1_carry_cnt_ll(Ir0_Type* ir,uint32_t carry_cnt)
{
	MODIFY_REG(ir->IR0_TX_DATA1_CFG,IR0_TX_DATA1_CFG_CARRY_CNT,((carry_cnt<<IR0_TX_DATA1_CFG_CARRY_CNT_POS)&IR0_TX_DATA1_CFG_CARRY_CNT_MASK)); 
}



/**
 * @brief config ir   data1 count
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t idle_cnt
 * @retval None
 */
static inline void ms_ir0_tx_config_data1_idle_cnt_ll(Ir0_Type* ir,uint32_t idle_cnt)
{
	MODIFY_REG(ir->IR0_TX_DATA1_CFG,IR0_TX_DATA1_CFG_IDLE_CNT,((idle_cnt<<IR0_TX_DATA1_CFG_IDLE_CNT_POS)&IR0_TX_DATA1_CFG_IDLE_CNT_MASK)); 
}



/**
 * @brief config ir  end carry count
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t carry_cnt
 * @retval None
 */
static inline void ms_ir0_tx_config_end_carry_cnt_ll(Ir0_Type* ir,uint32_t carry_cnt)
{
	MODIFY_REG(ir->IR0_TX_END_CFG,IR0_TX_END_CFG_CARRY_CNT,((carry_cnt<<IR0_TX_END_CFG_CARRY_CNT_POS)&IR0_TX_END_CFG_CARRY_CNT_MASK)); 
}



/**
 * @brief config ir end idle count
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t idle_cnt
 * @retval None
 */
static inline void ms_ir0_tx_config_end_idle_cnt_ll(Ir0_Type* ir,uint32_t idle_cnt)
{
	MODIFY_REG(ir->IR0_TX_END_CFG,IR0_TX_END_CFG_IDLE_CNT,((idle_cnt<<IR0_TX_END_CFG_IDLE_CNT_POS)&IR0_TX_END_CFG_IDLE_CNT_MASK)); 
}





/**
 * @brief config ir end idle count
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t idle_cnt
 * @retval None
 */
static inline void ms_ir0_tx_config_repeat_cycle_ll(Ir0_Type* ir,uint32_t repeat_cycle)
{
	MODIFY_REG(ir->IR0_TX_REPEAT_CFG,IR0_TX_REPEAT_CFG_TX_REPEAT_CYCLE,((repeat_cycle<<IR0_TX_REPEAT_CFG_TX_REPEAT_CYCLE_POS)&IR0_TX_REPEAT_CFG_TX_REPEAT_CYCLE_MASK)); 
}


/**
 * @brief write ir tx data to data reg
 * @param Ir0_Type* ir: ir regs     
 * @param uint32_t tx_data
 * @retval None
 */
static inline void ms_ir0_tx_data_fill_ll(Ir0_Type* ir,uint32_t tx_data)
{
	WRITE_REG(ir->IR0_TX_DATA,tx_data);
}





/**
 * @brief get ir status
 * @param Ir0_Type* ir: ir regs     
 * @retval status
 */
static inline uint32_t ms_ir0_get_status_ll(Ir0_Type* ir)
{
	return READ_REG(ir->IR0_TX_STATUS); 
}




/**
 * @brief clear ir done status
 * @param Ir0_Type* ir: ir regs     
 * @retval none
 */
static inline void ms_ir0_clear_tx_done_status_ll(Ir0_Type* ir)
{
	SET_BIT(ir->IR0_TX_STATUS,IR0_TX_STATUS_TX_DONE); 
}




#endif

