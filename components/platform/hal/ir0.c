/*
 * ir.c
 *
 *  Created on: 2021年12月24日
 *      Author: haijun.mai
 */

#include <ms_clock_hal.h>
#include "ms_ir0.h"
#include <stddef.h>
#include "ms_pinmux_hal.h"
#include <string.h>
#include "gpio.h"
#include "log.h"

Ir0Handle_Type ir0_handle;



#define IR0_NEC_DATA_NUM                                                   (31)

#define  IR0_CLK_CYCLE                                                             (0.04166)  /*24MHZ    1000000/24000000 = 0.04166us*/
#define  IR0_NEC_CARRY_CYCLE                                              ( 26.3) /*  1000000/38000  =26.31578947368421 us*/

#define  IR0_NEC_CARRY_CNT                                                 (IR0_NEC_CARRY_CYCLE/IR0_CLK_CYCLE)
#define  IR0_NEC_CARRY_HIGH_CYCLE                                   (IR0_NEC_CARRY_CNT/3)
#define  IR0_NEC_CARRY_LOW_CYCLE                                    (IR0_NEC_CARRY_CNT-IR0_NEC_CARRY_HIGH_CYCLE)

#define  IR0_NEC_START_CARRY_TIME_LEN                           (9000)/*9000 us*/
#define  IR0_NEC_PREAMBLE_CARRY_CNT                              (IR0_NEC_START_CARRY_TIME_LEN/IR0_NEC_CARRY_CYCLE)
#define  IR0_NEC_START_IDLE_TIME_LEN                             (4500)/*4500 us*/
#define  IR0_NEC_PREAMBLE_IDLE_CNT                                      (IR0_NEC_START_IDLE_TIME_LEN/IR0_CLK_CYCLE)

#define  IR0_NEC_DATA0_CARRY_TIME_LEN                         (562.5)/*562.5 us*/
#define  IR0_NEC_DATA0_CARRY_CNT                                     (IR0_NEC_DATA0_CARRY_TIME_LEN/IR0_NEC_CARRY_CYCLE)
#define  IR0_NEC_DATA0_IDLE_TIME_LEN                            (562.5)/*562.5 us*/
#define  IR0_NEC_DATA0_IDLE_CNT                                     (IR0_NEC_DATA0_IDLE_TIME_LEN/IR0_CLK_CYCLE)

#define  IR0_NEC_DATA1_CARRY_TIME_LEN                         (562.5)/*562.5 us*/
#define  IR0_NEC_DATA1_CARRY_CNT                                   (IR0_NEC_DATA1_CARRY_TIME_LEN/IR0_NEC_CARRY_CYCLE)
#define  IR0_NEC_DATA1_IDLE_TIME_LEN                           (1687.5)/*1687.5 us*/
#define  IR0_NEC_DATA1_IDLE_CNT                                     (IR0_NEC_DATA1_IDLE_TIME_LEN/IR0_CLK_CYCLE)

#define  IR0_NEC_END_CARRY_TIME_LEN                             (562.5)/*562.5 us*/
#define  IR0_NEC_END_CARRY_CNT                                       (IR0_NEC_END_CARRY_TIME_LEN/IR0_NEC_CARRY_CYCLE)
#define  IR0_NEC_END_IDLE_TIME_LEN                                (41937.5)/*110000- 9000-4500-16x(1125+2250)+562.5= 41937.5 us*/
#define  IR0_NEC_END_IDLE_CNT                                          (IR0_NEC_END_IDLE_TIME_LEN/IR0_CLK_CYCLE)

#define  IR0_NEC_REPEAT_CARRY_TIME_LEN                        (110000)/*98187.5 us*//*110000- 9000-2250-+562.5= 98187.5 us*/
#define  IR0_NEC_REPEAT_CARRY_CNT                                 (IR0_NEC_REPEAT_CARRY_TIME_LEN/IR0_CLK_CYCLE)




#define IR0_RCA_DATA_NUM                                                  (23)


#define  IR0_RCA_CARRY_CYCLE                                              ( 26.3) /*  1000000/38000  =26.31578947368421 us*/

#define  IR0_RCA_CARRY_CNT                                                 (IR0_RCA_CARRY_CYCLE/IR0_CLK_CYCLE)
#define  IR0_RCA_CARRY_HIGH_CYCLE                                   (IR0_RCA_CARRY_CNT/3)
#define  IR0_RCA_CARRY_LOW_CYCLE                                    (IR0_RCA_CARRY_CNT-IR0_RCA_CARRY_HIGH_CYCLE)

#define  IR0_RCA_START_CARRY_TIME_LEN                           (4000)/*4000 us*/
#define  IR0_RCA_PREAMBLE_CARRY_CNT                               (IR0_RCA_START_CARRY_TIME_LEN/IR0_RCA_CARRY_CYCLE)
#define  IR0_RCA_START_IDLE_TIME_LEN                             (4000)/*4000 us*/
#define  IR0_RCA_PREAMBLE_IDLE_CNT                                 (IR0_RCA_START_IDLE_TIME_LEN/IR0_CLK_CYCLE)


#define  IR0_RCA_DATA0_CARRY_TIME_LEN                           (500)/*500 us*/
#define  IR0_RCA_DATA0_CARRY_CNT                                     (IR0_RCA_DATA0_CARRY_TIME_LEN/IR0_RCA_CARRY_CYCLE)
#define  IR0_RCA_DATA0_IDLE_TIME_LEN                              (1000)/*1000 us*/
#define  IR0_RCA_DATA0_IDLE_CNT                                        (IR0_RCA_DATA0_IDLE_TIME_LEN/IR0_CLK_CYCLE)

#define  IR0_RCA_DATA1_CARRY_TIME_LEN                           (500)/*500 us*/
#define  IR0_RCA_DATA1_CARRY_CNT                                     (IR0_RCA_DATA1_CARRY_TIME_LEN/IR0_RCA_CARRY_CYCLE)
#define  IR0_RCA_DATA1_IDLE_TIME_LEN                              (2000)/*2000 us*/
#define  IR0_RCA_DATA1_IDLE_CNT                                        (IR0_RCA_DATA1_IDLE_TIME_LEN/IR0_CLK_CYCLE)

#define  IR0_RCA_END_CARRY_TIME_LEN                                 (500)/*500 us*/
#define  IR0_RCA_END_CARRY_CNT                                          (IR0_RCA_END_CARRY_TIME_LEN/IR0_RCA_CARRY_CYCLE)
#define  IR0_RCA_END_IDLE_TIME_LEN                                   (8500)/*8500 us*/
#define  IR0_RCA_END_IDLE_CNT                                             (IR0_RCA_END_IDLE_TIME_LEN/IR0_CLK_CYCLE)

#define  IR0_RCA_REAPEAT_CARRY_CNT                                   (1)

void ir0_reach_callback(Ir0Handle_Type *ir)
{
    CHECK_PTR_NULL(ir);

     MS_LOGI(MS_DRIVER, "\r\n ir0 call int!\n");
}





void ir0_init_calllback(Ir0Handle_Type *ir)
{

     CHECK_PTR_NULL(ir);

 
    /*config the ir pin mux*/
    //ms_pinmux_hal_set_pinmux(PAD13,PIN13_IR);
   ms_pinmux_hal_set_pinmux(PAD16,PIN16_IR);
    
    /*config the ir clock*/
    MS_CLOCK_HAL_CLK_ENABLE_IR();

    /*config the spi0 pre div*/
    ms_clock_hal_set_ir_div(4);

    /*mask ir interrupt*/
    ms_ir0_interrupt_mask(ir);

    /*enable the interrupt*/
    ms_ir0_enable_cpu_interrupt(ir) ;

   /*register ir interrupt handler*/
   ms_ir0_register_handler(ir);  
	
}


void ir0_deinit_calllback(Ir0Handle_Type *ir)
{
    CHECK_PTR_NULL(ir);

     /*config ir pin mux as default*/
    //ms_pinmux_hal_set_pinmux(PAD13,PIN13_GPIO_13);
     ms_pinmux_hal_set_pinmux(PAD16,PIN16_GPIO_16);

    /*mask ir interrupt*/	 
    ms_ir0_interrupt_mask( ir);
	
    /*disable the interrupt*/
    ms_ir0_disable_cpu_interrupt(ir);

    /*close the ir clock*/
    MS_CLOCK_HAL_CLK_DISABLE_IR();
 
    /*unregister ir0 interrupt handler */	
    ms_ir0_unregister_handler();

}


Ir0Callback_Type  ir0_callback =
{
       .init_callback = ir0_init_calllback,
	.deinit_callback = ir0_deinit_calllback,
	.ir_reach_callback = ir0_reach_callback,
};


int32_t ir0_nec_init(void)
{
    memset((uint8_t *)&ir0_handle, 0, sizeof(ir0_handle));

    ir0_handle.instance = IR0;
    ir0_handle.init.proto_mode = IR0_PROCOTOL_MODE_NEC;

    ir0_handle.init.carry_high_cycle = IR0_NEC_CARRY_HIGH_CYCLE;
    ir0_handle.init.carry_low_cycle = IR0_NEC_CARRY_LOW_CYCLE;
	
    ir0_handle.init.preamble_carry_cnt = IR0_NEC_PREAMBLE_CARRY_CNT;
    ir0_handle.init.preamble_idle_cnt = IR0_NEC_PREAMBLE_IDLE_CNT;
	
    ir0_handle.init.data0_carry_cnt = IR0_NEC_DATA0_CARRY_CNT;
    ir0_handle.init.data0_idle_cnt = IR0_NEC_DATA0_IDLE_CNT;
	
    ir0_handle.init.data1_carry_cnt = IR0_NEC_DATA1_CARRY_CNT;
    ir0_handle.init.data1_idle_cnt = IR0_NEC_DATA1_IDLE_CNT;
	
    ir0_handle.init.end_carry_cnt = IR0_NEC_END_CARRY_CNT;
    ir0_handle.init.end_idle_cnt = IR0_NEC_END_IDLE_CNT;

     ir0_handle.init.repeat_carry_cnt = IR0_NEC_REPEAT_CARRY_CNT;
    ir0_handle.p_callback = &ir0_callback;
    ir0_handle.irq = IR_IRQn;

	
    ms_ir0_init(&ir0_handle);

 

    return STATUS_SUCCESS;
}




int32_t ir0_nec_send_data(uint32_t tx_data) 
{
 
    ms_ir0_send_data(&ir0_handle, tx_data,IR0_NEC_DATA_NUM); 
    return STATUS_SUCCESS;
}

int32_t ir0_nec_send_data_repeat(uint32_t tx_data) 
{
  
   
    ms_ir0_send_data_repeat(&ir0_handle,tx_data,IR0_NEC_DATA_NUM); 
    return STATUS_SUCCESS;
}

int32_t ir0_nec_send_data_repeat_stop() 
{
    ms_ir0_send_data_repeat_stop(&ir0_handle); 
    return STATUS_SUCCESS;
}


int32_t ir0_nec_int_send_data(uint32_t tx_data) 
{
  
    ms_ir0_int_send_data(&ir0_handle, tx_data,IR0_NEC_DATA_NUM); 
    return STATUS_SUCCESS;
}




int32_t ir0_rca_init(void)
{
    memset((uint8_t *)&ir0_handle, 0, sizeof(ir0_handle));

    ir0_handle.instance = IR0;
    ir0_handle.init.proto_mode = IR0_PROCOTOL_MODE_RCA;

    ir0_handle.init.carry_high_cycle = IR0_RCA_CARRY_HIGH_CYCLE;
    ir0_handle.init.carry_low_cycle = IR0_RCA_CARRY_LOW_CYCLE;
	
    ir0_handle.init.preamble_carry_cnt = IR0_RCA_PREAMBLE_CARRY_CNT;
    ir0_handle.init.preamble_idle_cnt = IR0_RCA_PREAMBLE_IDLE_CNT;

    ir0_handle.init.data0_carry_cnt = IR0_RCA_DATA0_CARRY_CNT;
    ir0_handle.init.data0_idle_cnt = IR0_RCA_DATA0_IDLE_CNT;

    ir0_handle.init.data1_carry_cnt = IR0_RCA_DATA1_CARRY_CNT;
    ir0_handle.init.data1_idle_cnt = IR0_RCA_DATA1_IDLE_CNT;

    ir0_handle.init.end_carry_cnt =  IR0_RCA_END_CARRY_CNT;
    ir0_handle.init.end_idle_cnt = IR0_RCA_END_IDLE_CNT;

    ir0_handle.init.repeat_carry_cnt = IR0_RCA_REAPEAT_CARRY_CNT;

    ir0_handle.p_callback = &ir0_callback;
    ir0_handle.irq = IR_IRQn;

  
    ms_ir0_init(&ir0_handle);

    return STATUS_SUCCESS;
}



int32_t ir0_rca_send_data(uint32_t tx_data) 
{
 
    ms_ir0_send_data(&ir0_handle, tx_data,IR0_RCA_DATA_NUM); 
    return STATUS_SUCCESS;
}

int32_t ir0_rca_send_data_repeat(uint32_t tx_data) 
{
  
    ms_ir0_send_data_repeat(&ir0_handle,tx_data,IR0_RCA_DATA_NUM); 
    return STATUS_SUCCESS;
}

int32_t ir0_rca_send_data_repeat_stop() 
{
    ms_ir0_send_data_repeat_stop(&ir0_handle); 
    return STATUS_SUCCESS;
}

int32_t ir0_rca_int_send_data(uint32_t tx_data) 
{
  
    ms_ir0_int_send_data(&ir0_handle, tx_data,IR0_RCA_DATA_NUM); 
    return STATUS_SUCCESS;
}



int32_t ir0_deinit(void)
{
	ms_ir0_deinit(&ir0_handle);
	memset((uint8_t *)&ir0_handle, 0, sizeof(ir0_handle));
	return STATUS_SUCCESS;
}





//////////FOR APP USE////////////////////





#if 0



uint8_t ms_shift_char(uint8_t data)
{
    data = (data <<4) | (data >> 4);
    data = ((data <<2) &0xcc) |((data >>2)&0x33);
    data = ((data <<1) &0xaa) |((data >>1)&0x55);
    return data;
}

uint8_t ms_shift_4bit(uint8_t data)
{
    int  i            = 0;
    int8_t temp = 0;
    for(i = 0; i <4 ;i++)
    {
        temp   = temp << 1;
        temp |= (data >> i) & 0x01;
    }
   return temp;

}




void  ir_nec_init()
{
    ir0_nec_init();
}
void  ir_nec_deinit()
{
    ir0_deinit();
}




void ir_nec_send_frame(int8_t addr_code,  int8_t cmd_code)
{
    int32_t ir_tx_data;

    ir_tx_data = (((~cmd_code)&0xff)<<24) |  (((cmd_code)&0xff)<<16)  | (((~addr_code)&0xff)<<8) | addr_code;
   
    ir0_nec_send_data(ir_tx_data);
}




void ir_nec_send_frame_repeat(int8_t addr_code,  int8_t cmd_code)
{
    int32_t ir_tx_data;

    ir_tx_data = (((~cmd_code)&0xff)<<24) | (((cmd_code)&0xff)<<16) | (((~addr_code)&0xff)<<8) | addr_code;
    MS_LOGI(MS_DRIVER, "\r\n nec repeat send\n");
    ir0_nec_send_data_repeat(ir_tx_data);
 
}




void ir_nec_send_frame_repeat_stop() 
{
     ir0_nec_send_data_repeat_stop();
}




void  ir_rca_init()
{
     ir0_rca_init();
     MS_LOGI(MS_DRIVER, "\r\n ir rca init\n");
}
void  ir_rca_deinit()
{
    ir0_deinit();
    MS_LOGI(MS_DRIVER, "\r\n ir nec init\n");
}

void ir_rca_send_frame(int8_t user_code,  int8_t key_code)
{

    int32_t ir_tx_data;

    key_code  = ms_shift_char(key_code);
    user_code = ms_shift_4bit(user_code);
    	
   
   //  ir_tx_data = (((~key_code)&0xff)<<16) | (((~user_code) & 0xf)<<12) | (key_code<<4) | (user_code & 0xf)  ;	
    ir_tx_data = (((key_code)&0xff)<<16) | (((user_code) & 0xf)<<12) | (((~key_code)&0xff)<<4) | ((~user_code) & 0xf)  ;
   
	
     ir0_rca_send_data(ir_tx_data);
	
}

void ir_rca_send_frame_repeat(int8_t user_code,  int8_t key_code)
{

    int32_t ir_tx_data;

    key_code  = ms_shift_char(key_code);
    user_code = ms_shift_4bit(user_code);
    	
   
    // ir_tx_data = (((~key_code)&0xff)<<16) | (((~user_code) & 0xf)<<12) | (key_code<<4) | (user_code & 0xf)  ;	
     ir_tx_data = (((key_code)&0xff)<<16) | (((user_code) & 0xf)<<12) | (((~key_code)&0xff)<<4) | ((~user_code) & 0xf)  ;

//	   ir_tx_data = 0x00f380c7;


    // ir1_rca_send_data_repeat(ir_tx_data);
	
}



void ir_rca_send_frame_repeat_stop() 
{
     ir0_rca_send_data_repeat_stop();
}


#endif









