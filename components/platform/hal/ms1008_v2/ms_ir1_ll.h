/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_ir_ll.h
 * @brief Header file of ir  module.
 * @author haijun.mai
 * @date   2022-03-28
 * @version 1.0
 * @Revision
 */

#ifndef MS_IR1_LL_H_
#define MS_IR1_LL_H_

#include "ms1008.h"
#include "ms_ir1_regs.h"
#include <stdint.h>
#include <stdbool.h>
#include "log.h"

#define IR1_PROCOTOL_MODE_USER_DEFINE    0
#define IR1_PROCOTOL_MODE_NEC    2
#define IR1_PROCOTOL_MODE_RCA    3

#define IR1_ARBITRAY_DATA_SW_MODE    0
#define IR1_ARBITRAY_DATA_HW_MODE    1

#define IR1_CARRY_COMPENSATION_NO                 0
#define IR1_CARRY_COMPENSATION_HIGH             1
#define IR1_CARRY_COMPENSATION_ALL                2

/**
 * @brief enable ir controller 
 * @param  Ir_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir1_tx_enable_ll(Ir1_Type* ir)
{
	SET_BIT(ir->IR1_TX_CTL,IR1_TX_CTL_TX_EN); 
}

/**
 * @brief disable ir controller 
 * @param  Ir_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir1_tx_disable_ll(Ir1_Type* ir)
{
	CLEAR_BIT(ir->IR1_TX_CTL,IR1_TX_CTL_TX_EN); 
}


/**
 * @brief enable ir dma interface 
 * @param  Ir_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir1_tx_dma_enable_ll(Ir1_Type* ir)
{
	SET_BIT(ir->IR1_TX_CTL,IR1_TX_CTL_DMA_EN); 
}

/**
 * @brief disable ir dma interface  
 * @param  Ir_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir1_tx_dma_disable_ll(Ir1_Type* ir)
{
	CLEAR_BIT(ir->IR1_TX_CTL,IR1_TX_CTL_DMA_EN); 
}



/**
 * @brief enable ir interrupt
 * @param  Ir_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir1_tx_int_enable_ll(Ir1_Type* ir)
{
	SET_BIT(ir->IR1_TX_CTL,IR1_TX_CTL_INT_EN); 
}

/**
 * @brief disable ir interruptt 
 * @param  Ir_Type* ir: ir regs                    
 * @retval None
 */
static inline void ms_ir1_tx_int_disable_ll(Ir1_Type* ir)
{
	CLEAR_BIT(ir->IR1_TX_CTL,IR1_TX_CTL_INT_EN); 
}


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
static inline void ms_ir1_config_proto_mode_ll(Ir1_Type* ir,uint8_t proto_mode)
{
	MODIFY_REG(ir->IR1_TX_CTL,IR1_TX_CTL_PROTO_MODE,((proto_mode<<IR1_TX_CTL_PROTO_MODE_POS)&IR1_TX_CTL_PROTO_MODE_MASK)); 
}



/**
 * @brief enable ir proto repeat 
 * @param Ir_Type* ir: ir regs     
 * @retval None
 */
static inline void ms_ir1_proto_repeat_enable_ll(Ir1_Type* ir)
{
	SET_BIT(ir->IR1_TX_CTL,IR1_TX_CTL_PROTO_REPEAT_EN); 
}

/**
 * @brief config ir fifo data mode
 * @param Ir_Type* ir: ir regs     
 * @retval None
 */
static inline void ms_ir1_fifo_data_level_enable_ll(Ir1_Type* ir)
{
	SET_BIT(ir->IR1_TX_CTL,IR1_TX_CTL_FIFO_DATA_MODE); 
}




/**
 * @brief config ir fifo data mode
 * @param Ir_Type* ir: ir regs     
 * @retval None
 */
static inline void ms_ir1_fifo_data_level_disable_ll(Ir1_Type* ir)
{
	CLEAR_BIT(ir->IR1_TX_CTL,IR1_TX_CTL_FIFO_DATA_MODE); 
}


/**
 * @brief config ir carry compensation mode
 * @param Ir_Type* ir: ir regs     
 * @param  uint32_t carry_compensation_mode
 * @retval None
 */
static inline void ms_ir1_config_carry_compensation_mode_ll(Ir1_Type* ir,uint32_t carry_compensation_mode)
{
	MODIFY_REG(ir->IR1_TX_CTL,IR1_TX_CTL_FIFO_CARRY_COMP_MODE,(carry_compensation_mode<<IR1_TX_CTL_FIFO_CARRY_COMP_MODE_POS)&IR1_TX_CTL_FIFO_CARRY_COMP_MODE_MASK); 
}





/**
 * @brief config ir tx data length
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t tx_len
 * @retval None
 */
static inline void ms_ir1_set_tx_length_ll(Ir1_Type* ir,uint32_t tx_len)
{
	MODIFY_REG(ir->IR1_TX_LENGTH,IR1_TX_LENGTH_CFG,((tx_len<<IR1_TX_LENGTH_CFG_POS)&IR1_TX_LENGTH_CFG_MASK)); 
}

/**
 * @brief config ir tx watermask
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t watermask
 * @retval None
 */
static inline void ms_ir1_set_tx_watermask_ll(Ir1_Type* ir,uint32_t watermask)
{
	MODIFY_REG(ir->IR1_FIFO_CFG,IR1_FIFO_CFG_FIFO_WATERMASK,((watermask<<IR1_FIFO_CFG_FIFO_WATERMASK_POS)&IR1_FIFO_CFG_FIFO_WATERMASK_MASK)); 
}

/**
 * @brief config ir tx carry high cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t carry_high_cycle
 * @retval None
 */
static inline void ms_ir1_tx_carry_high_cycle_config_ll(Ir1_Type* ir,uint32_t carry_high_cycle)
{
MODIFY_REG(ir->IR1_TX_CARRY_CFG,IR1_TX_CARRY_CFG_CARRY_HIGH_CYCLE,((carry_high_cycle<<IR1_TX_CARRY_CFG_CARRY_HIGH_CYCLE_POS)&IR1_TX_CARRY_CFG_CARRY_HIGH_CYCLE_MASK)); 
}




/**
 * @brief config ir tx carry low cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t carry_low_cycle
 * @retval None
 */
static inline void ms_ir1_tx_carry_low_cycle_config_ll(Ir1_Type* ir,uint32_t carry_low_cycle)
{
      	MODIFY_REG(ir->IR1_TX_CARRY_CFG,IR1_TX_CARRY_CFG_CARRY_LOW_CYCLE,((carry_low_cycle<<IR1_TX_CARRY_CFG_CARRY_LOW_CYCLE_POS)&IR1_TX_CARRY_CFG_CARRY_LOW_CYCLE_MASK)); 
}	

/**
 * @brief config ir tx level cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t tx_level_cycle
 * @retval None
 */
static inline void ms_ir1_tx_level_cycle_config_ll(Ir1_Type* ir,uint32_t tx_level_cycle)
{
      // MS_LOGI(MS_DRIVER, "\r\ntx_level_cycle = %x\n",tx_level_cycle);
	WRITE_REG(ir->IR1_TX_LEVEL_CFG, tx_level_cycle);
	//MODIFY_REG(ir->IR1_TX_LEVEL_CFG,IR1_TX_LEVEL_CFG_LEVEL_CYCLE,((tx_level_cycle<<IR1_TX_LEVEL_CFG_LEVEL_CYCLE_POS)&IR1_TX_LEVEL_CFG_LEVEL_CYCLE_MASK)); 
	//MS_LOGI(MS_DRIVER, "\r\nIR1_TX_LEVEL_CFG(%x) = %x\n",&ir->IR1_TX_LEVEL_CFG,ir->IR1_TX_LEVEL_CFG);
      

}


/**
 * @brief config ir tx level polarity
 * @param Ir_Type* ir: ir regs     
 * @param uint8_t tx_level_cycle
 * @retval None
 */
static inline void ms_ir1_tx_level_polarity_config_ll(Ir1_Type* ir,uint8_t level_polarity)
{
	MODIFY_REG(ir->IR1_TX_LEVEL_CFG,IR1_TX_LEVEL_CFG_LEVEL_POL_SEL,((level_polarity<<IR1_TX_LEVEL_CFG_LEVEL_POL_SEL_POS)&IR1_TX_LEVEL_CFG_LEVEL_POL_SEL_MASK)); 
}



/**
 * @brief write ir tx data to data reg
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t tx_data
 * @retval None
 */
static inline void ms_ir1_tx_data_fill_ll(Ir1_Type* ir,uint32_t tx_data)
{
	WRITE_REG(ir->IR1_PROTO_TX_DATA,tx_data);
}




/**
 * @brief config ir proto start high cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t high_cycle
 * @retval None
 */
static inline void ms_ir1_config_proto_start_bit_high_cycle_ll(Ir1_Type* ir,uint32_t high_cycle)
{
	MODIFY_REG(ir->IR1_PROTO_START_H,IR1_PROTO_START_H_CFG,((high_cycle<<IR1_PROTO_START_H_CFG_POS)&IR1_PROTO_START_H_CFG_MASK)); 
}



/**
 * @brief config ir proto start low cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t low_cycle
 * @retval None
 */
static inline void ms_ir1_config_proto_start_bit_low_cycle_ll(Ir1_Type* ir,uint32_t low_cycle)
{
	MODIFY_REG(ir->IR1_PROTO_START_L,IR1_PROTO_START_L_CFG,((low_cycle<<IR1_PROTO_START_L_CFG_POS)&IR1_PROTO_START_L_CFG_MASK)); 
}

/**
 * @brief config ir proto data high cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t high_cycle
 * @retval None
 */
static inline void ms_ir1_config_proto_data0_bit_low_cycle_ll(Ir1_Type* ir,uint32_t high_cycle)
{
	MODIFY_REG(ir->IR1_PROTO_DATA_L,IR1_PROTO_DATA_L_PROTO_DATA0_L,((high_cycle<<IR1_PROTO_DATA_L_PROTO_DATA0_L_POS)&IR1_PROTO_DATA_L_PROTO_DATA0_L_MASK)); 
}



/**
 * @brief config ir proto data low cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t low_cycle
 * @retval None
 */
static inline void ms_ir1_config_proto_data1_bit_low_cycle_ll(Ir1_Type* ir,uint32_t low_cycle)
{
	MODIFY_REG(ir->IR1_PROTO_DATA_L,IR1_PROTO_DATA_L_PROTO_DATA1_L,((low_cycle<<IR1_PROTO_DATA_L_PROTO_DATA1_L_POS)&IR1_PROTO_DATA_L_PROTO_DATA1_L_MASK)); 
}


/**
 * @brief config ir proto end high cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t high_cycle
 * @retval None
 */
static inline void ms_ir1_config_proto_data_and_end_bit_high_cycle_ll(Ir1_Type* ir,uint32_t high_cycle)
{
	MODIFY_REG(ir->IR1_PROTO_DATA_END_H,IR1_PROTO_DATA_END_H_CFG,((high_cycle<<IR1_PROTO_DATA_END_H_CFG_POS)&IR1_PROTO_DATA_END_H_CFG_MASK)); 
}



/**
 * @brief config ir proto end low cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t low_cycle
 * @retval None
 */
static inline void ms_ir1_config_proto_end_bit_low_cycle_ll(Ir1_Type* ir,uint32_t low_cycle)
{
	MODIFY_REG(ir->IR1_PROTO_END_L,IR1_PROTO_END_L_CFG,((low_cycle<<IR1_PROTO_END_L_CFG_POS)&IR1_PROTO_END_L_CFG_MASK)); 
}



/**
 * @brief config ir proto end low cycle
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t low_cycle
 * @retval None
 */
static inline void ms_ir1_config_proto_repeat_end_bit_low_cycle_ll(Ir1_Type* ir,uint32_t low_cycle)
{
	MODIFY_REG(ir->IR1_PROTO_REP_END_L,IR1_PROTO_REP_END_L_CFG,((low_cycle<<IR1_PROTO_REP_END_L_CFG_POS)&IR1_PROTO_REP_END_L_CFG_MASK)); 
}


/**
 * @brief unmask ir interrupt
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t interrupt_source
 * @retval None
 */
static inline void ms_ir1_interrupt_unmask_ll(Ir1_Type* ir,uint32_t interrupt_source)
{
	CLEAR_BIT(ir->IR1_INT_MASK,(interrupt_source&IR1_INT_MASK_ALL_MASK)); 
}

/**
 * @brief mask ir interrupt
 * @param Ir_Type* ir: ir regs     
 * @param uint32_t interrupt_source
 * @retval None
 */
static inline void ms_ir1_interrupt_mask_ll(Ir1_Type* ir,uint32_t interrupt_source)
{
	SET_BIT(ir->IR1_INT_MASK,(interrupt_source&IR1_INT_MASK_ALL_MASK)); 
}



/**
 * @brief get ir interrupt status
 * @param Ir_Type* ir: ir regs     
 * @retval interrupt status
 */
static inline uint32_t ms_ir1_get_interrupt_status_ll(Ir1_Type* ir)
{
	return READ_REG(ir->IR1_INT_STATUS)&IR1_INT_STATUS_ALL; 
}


/**
 * @brief get ir status
 * @param Ir_Type* ir: ir regs     
 * @retval status
 */
static inline uint32_t ms_ir1_get_status_ll(Ir1_Type* ir)
{
	return READ_REG(ir->IR1_STATUS); 
}


/**
 * @brief flash ir fifo
 * @param Ir_Type* ir: ir regs     
 * @retval none
 */
static inline void ms_ir1_flush_fifo_ll(Ir1_Type* ir)
{
	SET_BIT(ir->IR1_FLUSH,IR1_FLUSH_CFG);  
}



/**
 * @brief clear ir status
 * @param Ir_Type* ir: ir regs     
 * @retval none
 */
static inline void ms_ir1_clear_status_ll(Ir1_Type* ir,uint32_t int_source)
{
	CLEAR_BIT(ir->IR1_STATUS,int_source); 
}


#endif

