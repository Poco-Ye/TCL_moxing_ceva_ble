/*
 * ir.c
 *
 *  Created on: 2021年12月24日
 *      Author: haijun.mai
 */
#ifdef IR1_TO2
#include <ms_clock_hal.h>
#include "ms_ir1.h"
#include <stddef.h>
#include "ms_pinmux_hal.h"
#include <string.h>
#include "gpio.h"
#include "log.h"

Ir1Handle_Type ir1_handle;



#define IR1_NEC_DATA_NUM                                                   (31)

#define  IR1_CLK_FREQ                                                             (0.04166)  /*24MHZ    1000000/24000000 = 0.04166us*/
#define  IR1_NEC_CARRY_CYCLE                                              ( 26.3) /*  1000000/38000  =26.31578947368421 us*/

#define  IR1_NEC_CARRY_CNT                                                 (IR1_NEC_CARRY_CYCLE/IR1_CLK_FREQ)
#define  IR1_NEC_CARRY_HIGH_CYCLE                                   (IR1_NEC_CARRY_CNT/3)
#define  IR1_NEC_CARRY_LOW_CYCLE                                    (IR1_NEC_CARRY_CNT-IR1_NEC_CARRY_HIGH_CYCLE)

#define  IR1_NEC_START_CARRY_TIME_LEN                           (9000)/*9000 us*/
#define  IR1_NEC_START_CARRY_CNT                                    (IR1_NEC_START_CARRY_TIME_LEN/IR1_CLK_FREQ)
#define  IR1_NEC_START_IDLE_TIME_LEN                             (4500)/*4500 us*/
#define  IR1_NEC_START_IDLE_CNT                                      (IR1_NEC_START_IDLE_TIME_LEN/IR1_CLK_FREQ)

#define IR1_NEC_END_DATA0_DATA1_CARRY_TIME_LEN     (562.5)
#define IR1_NEC_END_DATA0_DATA1_CARRY_CNT                (IR1_NEC_END_DATA0_DATA1_CARRY_TIME_LEN/IR1_CLK_FREQ)

#define  IR1_NEC_DATA0_IDLE_TIME_LEN                            (562.5)/*562.5 us*/
#define  IR1_NEC_DATA0_IDLE_CNT                                     (IR1_NEC_DATA0_IDLE_TIME_LEN/IR1_CLK_FREQ)
#define  IR1_NEC_DATA1_IDLE_TIME_LEN                           (1687.5)/*1687.5 us*/
#define  IR1_NEC_DATA1_IDLE_CNT                                     (IR1_NEC_DATA1_IDLE_TIME_LEN/IR1_CLK_FREQ)

#define  IR1_NEC_END_IDLE_TIME_LEN                                 (41937.5)/*110000- 9000-4500-16x(1125+2250)+562.5= 41937.5 us*/
#define  IR1_NEC_END_IDLE_CNT                                          (IR1_NEC_END_IDLE_TIME_LEN/IR1_CLK_FREQ)

#define  IR1_NEC_REPEAT_CARRY_TIME_LEN                        (98187.5)/*98187.5 us*//*110000- 9000-2250-+562.5= 98187.5 us*/
#define  IR1_NEC_REPEAT_CARRY_CNT                                 (IR1_NEC_REPEAT_CARRY_TIME_LEN/IR1_CLK_FREQ)




#define IR1_RCA_DATA_NUM                                                  (23)


#define  IR1_RCA_CARRY_CYCLE                                              ( 26.3) /*  1000000/38000  =26.31578947368421 us*/

#define  IR1_RCA_CARRY_CNT                                                 (IR1_RCA_CARRY_CYCLE/IR1_CLK_FREQ)
#define  IR1_RCA_CARRY_HIGH_CYCLE                                   (IR1_RCA_CARRY_CNT/3)
#define  IR1_RCA_CARRY_LOW_CYCLE                                    (IR1_RCA_CARRY_CNT-IR1_RCA_CARRY_HIGH_CYCLE)

#define  IR1_RCA_START_CARRY_TIME_LEN                           (4000)/*4000 us*/
#define  IR1_RCA_START_CARRY_CNT                                    (IR1_RCA_START_CARRY_TIME_LEN/IR1_CLK_FREQ)
#define  IR1_RCA_START_CARRY_TIME_LEN                          (4000)/*4000 us*/
#define  IR1_RCA_START_IDLE_CNT                                      (IR1_RCA_START_CARRY_TIME_LEN/IR1_CLK_FREQ)

#define IR1_RCA_END_DATA0_DATA1_CARRY_TIME_LEN     (500)
#define IR1_RCA_END_DATA0_DATA1_CARRY_CNT                (IR1_RCA_END_DATA0_DATA1_CARRY_TIME_LEN/IR1_CLK_FREQ)

#define  IR1_RCA_DATA0_IDLE_TIME_LEN                            (1000)/*1000 us*/
#define  IR1_RCA_DATA0_IDLE_CNT                                     (IR1_RCA_DATA0_IDLE_TIME_LEN/IR1_CLK_FREQ)
#define  IR1_RCA_DATA1_IDLE_TIME_LEN                           (2000)/*2000 us*/
#define  IR1_RCA_DATA1_IDLE_CNT                                     (IR1_RCA_DATA1_IDLE_TIME_LEN/IR1_CLK_FREQ)

#define  IR1_RCA_END_IDLE_TIME_LEN                                 (8500)/*8500 us*/
#define  IR1_RCA_END_IDLE_CNT                                          (IR1_RCA_END_IDLE_TIME_LEN/IR1_CLK_FREQ)



#define  IR1_ARBITRARY_CARRY_CYCLE                                ( 26.3) /*  1000000/38000  =26.31578947368421 us*/

#define  IR1_ARBITRARY_CARRY_CNT                                  (IR1_RCA_CARRY_CYCLE/IR1_CLK_FREQ)
#define  IR1_ARBITRARY_CARRY_HIGH_CYCLE                      (IR1_RCA_CARRY_CNT/3)
#define  IR1_ARBITRARY_CARRY_LOW_CYCLE                        (IR1_RCA_CARRY_CNT-IR1_RCA_CARRY_HIGH_CYCLE)



void ir1_reach_calllback(Ir1Handle_Type *ir)
{
     //MS_LOGI(MS_DRIVER, "\r\n ir int!\n");
     ;
}





void ir1_init_calllback(Ir1Handle_Type *ir)
{
    /*config the ir pin mux*/
    //ms_pinmux_hal_set_pinmux(PAD13,PIN13_IR);
   ms_pinmux_hal_set_pinmux(PAD16,PIN16_IR);
    
    /*config the ir clock*/
    MS_CLOCK_HAL_CLK_ENABLE_IR();
   // MS_CLOCK_HAL_CLK_ENABLE_IR_SAMPLE();
    
    /*config the spi0 pre div*/
    ms_clock_hal_set_ir_div(4);

    ms_ir1_interrupt_mask( ir, IR1_INT_MASK_ALL);

   // ms_ir1_interrupt_unmask(ir,IR1_INT_MASK_TX_DONE ); 
    
    /*enable the interrupt*/
    ms_ir1_enable_cpu_interrupt(ir) ;

   ms_ir1_register_handler(ir);

             /*dma tx init*/
   ms_ir1_init_dma_tx(&ir->hdmatx);
   
   if (ir->hdmatx)
   {
        ir->hdmatx->parent = ir;
   }
	
}


void ir1_deinit_calllback(Ir1Handle_Type *ir)
{

     /*config ir pin mux as default*/
    ms_pinmux_hal_set_pinmux(PAD13,PIN13_GPIO_13);

    ms_ir1_interrupt_mask( ir, IR1_INT_MASK_ALL);
    /*disable the interrupt*/
    ms_ir1_disable_cpu_interrupt(ir);

    /*close the ir clock*/
    //MS_CLOCK_HAL_CLK_DISABLE_IR_SAMPLE();
    MS_CLOCK_HAL_CLK_DISABLE_IR();
   ms_ir1_unregister_handler();

     if (ir->hdmatx)
    {
        ms_dmac_mgmt_free(ir->hdmatx);
    }
}


Ir1Callback_Type  ir1_callback =
{
       .init_callback = ir1_init_calllback,
	.deinit_callback = ir1_deinit_calllback,
	.ir_reach_callback = ir1_reach_calllback,
};


int32_t ir1_nec_init(void)
{
    memset((uint8_t *)&ir1_handle, 0, sizeof(ir1_handle));

    ir1_handle.instance = IR1;
    ir1_handle.init.proto_mode = IR1_PROCOTOL_MODE_NEC;


    ir1_handle.init.carry_high_cycle = IR1_NEC_CARRY_HIGH_CYCLE;
    ir1_handle.init.carry_low_cycle = IR1_NEC_CARRY_LOW_CYCLE;
    ir1_handle.init.start_carry_cnt = IR1_NEC_START_CARRY_CNT;
    ir1_handle.init.start_idle_cnt = IR1_NEC_START_IDLE_CNT;
    ir1_handle.init.data0_idle_cnt = IR1_NEC_DATA0_IDLE_CNT;
    ir1_handle.init.data1_idle_cnt = IR1_NEC_DATA1_IDLE_CNT;
    ir1_handle.init.end_data0_data1_carry_cnt = IR1_NEC_END_DATA0_DATA1_CARRY_CNT;
    ir1_handle.init.repeat_carry_cnt = IR1_NEC_REPEAT_CARRY_CNT;
    ir1_handle.init.end_idle_cnt = IR1_NEC_END_IDLE_CNT;
	
    ir1_handle.init.carry_compensation_mode = IR1_CARRY_COMPENSATION_NO;

    ir1_handle.p_callback = &ir1_callback;
    ir1_handle.init.dma_enable = IR1_TRUE;
    ir1_handle.irq = IR1_IRQn;

	
    ms_ir1_init(&ir1_handle);

 

    return STATUS_SUCCESS;
}




int32_t ir1_nec_send_data(uint32_t tx_data) 
{
    ms_ir1_send_data(&ir1_handle, tx_data,IR1_NEC_DATA_NUM); 
    return STATUS_SUCCESS;
}

int32_t ir1_nec_send_data_repeat(uint32_t tx_data) 
{
    ms_ir1_send_data_repeat(&ir1_handle,tx_data,IR1_NEC_DATA_NUM); 
    return STATUS_SUCCESS;
}

int32_t ir1_nec_send_data_repeat_stop() 
{
    ms_ir1_send_data_repeat_stop(&ir1_handle); 
    return STATUS_SUCCESS;
}






int32_t ir1_rca_init(void)
{
    memset((uint8_t *)&ir1_handle, 0, sizeof(ir1_handle));

    ir1_handle.instance = IR1;
    ir1_handle.init.proto_mode = IR1_PROCOTOL_MODE_RCA;



    ir1_handle.init.carry_high_cycle = IR1_RCA_CARRY_HIGH_CYCLE;
    ir1_handle.init.carry_low_cycle = IR1_RCA_CARRY_LOW_CYCLE;
    ir1_handle.init.start_carry_cnt = IR1_RCA_START_CARRY_CNT;
    ir1_handle.init.start_idle_cnt = IR1_RCA_START_IDLE_CNT;
    ir1_handle.init.data0_idle_cnt = IR1_RCA_DATA0_IDLE_CNT;
    ir1_handle.init.data1_idle_cnt = IR1_RCA_DATA1_IDLE_CNT;
    ir1_handle.init.end_idle_cnt = IR1_RCA_END_IDLE_CNT;
    ir1_handle.init.end_data0_data1_carry_cnt = IR1_RCA_END_DATA0_DATA1_CARRY_CNT; 


	
    ir1_handle.init.carry_compensation_mode = IR1_CARRY_COMPENSATION_NO;

    ir1_handle.p_callback = &ir1_callback;
    ir1_handle.init.dma_enable = IR1_TRUE;
    ir1_handle.irq = IR1_IRQn;

  
    ms_ir1_init(&ir1_handle);

    return STATUS_SUCCESS;
}



int32_t ir1_rca_send_data(uint32_t tx_data) 
{
    ms_ir1_send_data(&ir1_handle, tx_data,IR1_RCA_DATA_NUM); 
    return STATUS_SUCCESS;
}

int32_t ir1_rca_send_data_repeat_stop() 
{
    ms_ir1_send_data_repeat_stop(&ir1_handle); 
    return STATUS_SUCCESS;
}


int32_t ir1_arbitrary_init(void)
{
    memset((uint8_t *)&ir1_handle, 0, sizeof(ir1_handle));

    ir1_handle.instance = IR1;
    ir1_handle.init.proto_mode = IR1_PROCOTOL_MODE_USER_DEFINE;
    ir1_handle.init.carry_high_cycle = IR1_ARBITRARY_CARRY_HIGH_CYCLE;
    ir1_handle.init.carry_low_cycle = IR1_ARBITRARY_CARRY_LOW_CYCLE;
    ir1_handle.init.arbitrary_data_mode = IR1_ARBITRAY_DATA_HW_MODE;
    ir1_handle.init.carry_compensation_mode =  IR1_CARRY_COMPENSATION_HIGH;
    ir1_handle.p_callback = &ir1_callback;
    ir1_handle.init.dma_enable = IR1_TRUE;
    ir1_handle.irq = IR1_IRQn;
	  /*select ir1*/

    ms_ir1_init(&ir1_handle);

    return STATUS_SUCCESS;
}

int32_t ir1_arbitrary_sw_init(void)
{
    memset((uint8_t *)&ir1_handle, 0, sizeof(ir1_handle));

    ir1_handle.instance = IR1;
    ir1_handle.init.proto_mode = IR1_PROCOTOL_MODE_USER_DEFINE;
    ir1_handle.init.carry_high_cycle = IR1_ARBITRARY_CARRY_HIGH_CYCLE;
    ir1_handle.init.carry_low_cycle = IR1_ARBITRARY_CARRY_LOW_CYCLE;
    ir1_handle.init.arbitrary_data_mode = IR1_ARBITRAY_DATA_SW_MODE;
    ir1_handle.init.carry_compensation_mode = IR1_CARRY_COMPENSATION_NO;
  
    ir1_handle.p_callback = &ir1_callback;
    ir1_handle.init.dma_enable = IR1_TRUE;
    ir1_handle.irq = IR1_IRQn;

    ms_ir1_init(&ir1_handle);

    return STATUS_SUCCESS;
}

int32_t ir1_arbitrary_all_com_init(void)
{
    memset((uint8_t *)&ir1_handle, 0, sizeof(ir1_handle));

    ir1_handle.instance = IR1;
    ir1_handle.init.proto_mode = IR1_PROCOTOL_MODE_USER_DEFINE;
    ir1_handle.init.carry_high_cycle = IR1_ARBITRARY_CARRY_HIGH_CYCLE;
    ir1_handle.init.carry_low_cycle = IR1_ARBITRARY_CARRY_LOW_CYCLE;
    ir1_handle.init.arbitrary_data_mode = IR1_ARBITRAY_DATA_HW_MODE;
    ir1_handle.init.carry_compensation_mode =  IR1_CARRY_COMPENSATION_ALL;
    ir1_handle.p_callback = &ir1_callback;
    ir1_handle.init.dma_enable = IR1_TRUE;
    ir1_handle.irq = IR1_IRQn;

    ms_ir1_init(&ir1_handle);

    return STATUS_SUCCESS;
}




int32_t ir1_arbitrary_send_data(uint32_t * tx_data,uint32_t size) 
{
    ms_ir1_arbitrary_send_data(&ir1_handle, tx_data,size); 
    return STATUS_SUCCESS;
}



int32_t ir1_arbitrary_int_send_data(uint32_t * tx_data,uint32_t size) 
{
    ms_ir1_arbitrary_interrupt_send_data(&ir1_handle, tx_data,size); 
    return STATUS_SUCCESS;
}


int32_t ir1_arbitrary_dma_send_data(uint32_t * tx_data,uint32_t size) 
{
    ms_ir1_arbitrary_dma_send_data(&ir1_handle, tx_data,size); 
    return STATUS_SUCCESS;
}


int32_t ir1_deinit(void)
{
	ms_ir1_deinit(&ir1_handle);
	memset((uint8_t *)&ir1_handle, 0, sizeof(ir1_handle));
	return STATUS_SUCCESS;
}





//////////FOR APP USE////////////////////









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
    ir1_nec_init();
}
void  ir_nec_deinit()
{
    ir1_deinit();
}




void ir_nec_send_frame(int8_t addr_code,  int8_t cmd_code)
{
    int32_t ir_tx_data;

    ir_tx_data = (((~cmd_code)&0xff)<<24) |  (((cmd_code)&0xff)<<16)  | (((~addr_code)&0xff)<<8) | addr_code;
   
    ir1_nec_send_data(ir_tx_data);
}




void ir_nec_send_frame_repeat(int8_t addr_code,  int8_t cmd_code)
{
    int32_t ir_tx_data;

    ir_tx_data = (((~cmd_code)&0xff)<<24) | (((cmd_code)&0xff)<<16) | (((~addr_code)&0xff)<<8) | addr_code;
    MS_LOGI(MS_DRIVER, "\r\n nec repeat send\n");
    ir1_nec_send_data_repeat(ir_tx_data);
 
}




void ir_nec_send_frame_repeat_stop() 
{
     ir1_nec_send_data_repeat_stop();
}




void  ir_rca_init()
{
     ir1_rca_init();
     MS_LOGI(MS_DRIVER, "\r\n ir rca init\n");
}
void  ir_rca_deinit()
{
    ir1_deinit();
    MS_LOGI(MS_DRIVER, "\r\n ir nec init\n");
}

void ir_rca_send_frame(int8_t user_code,  int8_t key_code)
{

    int32_t ir_tx_data;

    key_code  = ms_shift_char(key_code);
    user_code = ms_shift_4bit(user_code);
    	
   
   //  ir_tx_data = (((~key_code)&0xff)<<16) | (((~user_code) & 0xf)<<12) | (key_code<<4) | (user_code & 0xf)  ;	
    ir_tx_data = (((key_code)&0xff)<<16) | (((user_code) & 0xf)<<12) | (((~key_code)&0xff)<<4) | ((~user_code) & 0xf)  ;
   
	
     ir1_rca_send_data(ir_tx_data);
	
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
     ir1_rca_send_data_repeat_stop();
}



void  ir_arbitrary_init()
{

}

void  ir_arbitrary_deinit()
{

}




#endif






