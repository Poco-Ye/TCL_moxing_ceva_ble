/*
 * ir.h
 *
 *  Created on: 2021年12月24日
 *      Author: haijun.mai
 */
#ifdef IR1_TO2
#ifndef IR1_H_
#define IR1_H_

#include "ms1008.h"
#include "ms_ir1.h"


int32_t ir1_nec_init(void);

int32_t ir1_nec_send_data(uint32_t tx_data);

int32_t ir1_nec_send_data_repeat(uint32_t tx_data);

int32_t ir1_nec_send_data_repeat_stop();

int32_t ir1_rca_init(void);

int32_t ir1_rca_send_data(uint32_t tx_data);



int32_t ir1_deinit(void);


int32_t ir1_arbitrary_init(void);

int32_t ir1_arbitrary_send_data(uint32_t * tx_data,uint32_t size) ;

int32_t ir1_arbitrary_int_send_data(uint32_t * tx_data,uint32_t size);

int32_t ir1_arbitrary_dma_send_data(uint32_t * tx_data,uint32_t size) ;

int32_t ir1_config_sw_mode(void);

int32_t ir1_arbitrary_sw_init(void);

int32_t ir1_arbitrary_all_com_init(void);



#define TCL_RCA_USER_CODE 0x0

#define IR_DEV_ADDR 0x5a
	
void  ir_nec_init();
void  ir_nec_deinit();
void ir_nec_send_frame(int8_t addr_code,  int8_t cmd_code);
void ir_nec_send_frame_repeat(int8_t addr_code,  int8_t cmd_code);
void ir_nec_send_frame_repeat_stop();

void  ir_rca_init();
void  ir_rca_deinit();
void ir_rca_send_frame(int8_t user_code,  int8_t key_code);
void ir_rca_send_frame_repeat(int8_t user_code,  int8_t key_code);
void ir_rca_send_frame_repeat_stop();

#endif /* IR_H_ */
#endif