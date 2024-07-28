/*
 * ir.h
 *
 *  Created on: 2021年12月24日
 *      Author: haijun.mai
 */

#ifndef IR0_H_
#define IR0_H_

#include "ms1008.h"
#include "ms_ir0.h"


int32_t ir0_nec_init(void);

int32_t ir0_nec_send_data(uint32_t tx_data);

int32_t ir0_nec_send_data_repeat(uint32_t tx_data);

int32_t ir0_nec_send_data_repeat_stop();

int32_t ir0_nec_int_send_data(uint32_t tx_data);

int32_t ir0_rca_init(void);

int32_t ir0_rca_send_data(uint32_t tx_data);

int32_t ir0_rca_send_data_repeat(uint32_t tx_data);

int32_t ir0_rca_send_data_repeat_stop();

int32_t ir0_rca_int_send_data(uint32_t tx_data);

int32_t ir0_deinit(void);

#if 0

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
#endif
#endif /* IR_H_ */
