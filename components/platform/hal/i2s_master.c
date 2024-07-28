/*
 * i2s_master.c
 *
 *  Created on: 2021年12月24日
 *      Author: haijun.mai
 */

#include <ms_clock_hal.h>
#include "ms_i2s.h"
#include <stddef.h>
#include "ms_pinmux_hal.h"
#include <string.h>
#include "gpio.h"
#include "log.h"

I2sHandle_Type i2s_master_handle[2];


void ms_i2s0_master_init_dma_tx(DmacHandle_Type **phdmatx)
{
    //    *phdmatx = NULL;
    DmacHandle_Type *hdmatx;
    hdmatx = ms_dmac_mgmt_alloc();
    *phdmatx = hdmatx;

    hdmatx->init.dst_tr_width = DMAC_XFER_WIDTH_32BITS;
    hdmatx->init.dst_addr_mode = DMAC_ADDRESS_MODE_NOC;
    hdmatx->init.dst_peri_type = DMAC_I2S0_TX;
    hdmatx->init.dst_msize = DMAC_MSIZE_1;
    hdmatx->init.src_peri_type = DMAC_MEM;
    hdmatx->init.src_tr_width = DMAC_XFER_WIDTH_32BITS;
    hdmatx->init.src_addr_mode = DMAC_ADDRESS_MODE_INC;
    hdmatx->init.src_msize = DMAC_MSIZE_1;
    ms_dmac_init(hdmatx);
}

 void ms_i2s0_master_init_dma_rx(DmacHandle_Type **phdmarx)
{
    //    *phdmatx = NULL;
    DmacHandle_Type *hdmarx;
    hdmarx = ms_dmac_mgmt_alloc();
    *phdmarx = hdmarx;
   
    hdmarx->init.dst_tr_width = DMAC_XFER_WIDTH_32BITS;
    hdmarx->init.dst_addr_mode = DMAC_ADDRESS_MODE_INC;
    hdmarx->init.dst_peri_type = DMAC_MEM;
    hdmarx->init.dst_msize = DMAC_MSIZE_1;
    hdmarx->init.src_peri_type = DMAC_I2S0_RX;
    hdmarx->init.src_tr_width = DMAC_XFER_WIDTH_32BITS;
    hdmarx->init.src_addr_mode =DMAC_ADDRESS_MODE_NOC ;
    hdmarx->init.src_msize = DMAC_MSIZE_1;
    ms_dmac_init(hdmarx);
}





void i2s0_master_reach_calllback(I2sHandle_Type *i2s_master)
{
     MS_LOGI(MS_DRIVER, "\r\n i2s0 master int!\n");
}





void i2s0master_init_calllback(I2sHandle_Type *i2s_master)
{
   /*config the spi0 pin mux*/
   

    /*config the i2s0*/
    MS_CLOCK_HAL_CLK_ENABLE_I2S0();

      /*config the spi0 pre div*/
    ms_clock_hal_set_i2s0_div(i2s_master->init.i2s_div);


     /*config spi0 pin mux*/
#if   defined ( __MS1008_V2 )
    ms_pinmux_hal_set_pinmux(PAD22,PIN22_I2S0_SCLK);
    ms_pinmux_hal_set_pinmux(PAD23,PIN23_I2S0_LRCK);
    ms_pinmux_hal_set_pinmux(PAD24,PIN24_I2S0_DI);
    ms_pinmux_hal_set_pinmux(PAD8,PIN08_I2S0_MCLK);
    ms_pinmux_hal_set_pinmux(PAD9,PIN09_I2S0_DO);
#else
    ms_pinmux_hal_set_pinmux(PAD0,PIN00_I2S0_SCLK);
    ms_pinmux_hal_set_pinmux(PAD1,PIN01_I2S0_LRCK);
    ms_pinmux_hal_set_pinmux(PAD2,PIN02_I2S0_DI);
    ms_pinmux_hal_set_pinmux(PAD3,PIN03_I2S0_MCLK);
    ms_pinmux_hal_set_pinmux(PAD4,PIN04_I2S0_DO);
#endif

      /*dma tx init*/
    ms_i2s0_master_init_dma_tx(&i2s_master->hdmatx);
    
    if (i2s_master->hdmatx)
    {
        i2s_master->hdmatx->parent = i2s_master;
    }
  
    /*dma rx init*/
    ms_i2s0_master_init_dma_rx(&i2s_master->hdmarx);

    if (i2s_master->hdmarx)
    {
        i2s_master->hdmarx->parent = i2s_master;
    }
    /*enable the interrupt*/

   ms_i2s_enable_cpu_interrupt(i2s_master) ;
}


void i2s0_master_deinit_calllback(I2sHandle_Type *i2s_master)
{

     /*config spi0 pin mux as default*/
#if   defined ( __MS1008_V2 )
    ms_pinmux_hal_set_pinmux(PAD22,PIN22_GPIO_22);
    ms_pinmux_hal_set_pinmux(PAD23,PIN23_GPIO_23);
    ms_pinmux_hal_set_pinmux(PAD24,PIN24_GPIO_24);
    ms_pinmux_hal_set_pinmux(PAD8,PIN08_GPIO_8);
    ms_pinmux_hal_set_pinmux(PAD9,PIN09_GPIO_9);
#else
    ms_pinmux_hal_set_pinmux(PAD0,PIN00_GPIO_0);
    ms_pinmux_hal_set_pinmux(PAD1,PIN01_GPIO_1);
    ms_pinmux_hal_set_pinmux(PAD2,PIN02_GPIO_2);
    ms_pinmux_hal_set_pinmux(PAD3,PIN03_GPIO_3);
    ms_pinmux_hal_set_pinmux(PAD4,PIN04_GPIO_4);
#endif

  

    /*disable the interrupt*/
    ms_i2s_disable_cpu_interrupt(i2s_master);

    /*close the spi master clock*/
    MS_CLOCK_HAL_CLK_DISABLE_I2S0();

    
    if (i2s_master->hdmarx)
    {
        ms_dmac_mgmt_free(i2s_master->hdmarx);
    }

    if (i2s_master->hdmatx)
    {
        ms_dmac_mgmt_free(i2s_master->hdmatx);
    }
}


I2sCallback_Type  i2s0_master_callback =
{
       .init_callback = i2s0master_init_calllback,
	.deinit_callback = i2s0_master_deinit_calllback,
	.i2s_reach_callback = i2s0_master_reach_calllback,
};


int32_t i2s0_master_init(void)
{
    memset((uint8_t *)&i2s_master_handle[0], 0, sizeof(i2s_master_handle[0]));
  
    i2s_master_handle[0].instance = I2S0;
    i2s_master_handle[0].dma_instance = I2S0_DMA;
    i2s_master_handle[0].init.role = I2S_MASTER;
    i2s_master_handle[0].init.wss = CLOCK_CYCLES_32;
    i2s_master_handle[0].init.sclkg = SCLK_CLOCK_CYCLES_16; 
    i2s_master_handle[0].init.i2s_div = 4;
	
    i2s_master_handle[0].init.i2s_tx_block_en = I2S_TRUE;
    i2s_master_handle[0].init.i2s_tx_channel_en = I2S_TRUE;
    i2s_master_handle[0].init.i2s_tx_dma_en = I2S_TRUE;
    i2s_master_handle[0].init.tx_data_valid_len = I2S_WORDSIZE_24bit;
    i2s_master_handle[0].init.tx_fifo_trigger_level = I2S_FIFO_TRIGGERL_LEVEL_11;
    

    i2s_master_handle[0].init.i2s_rx_block_en = I2S_TRUE;
    i2s_master_handle[0].init.i2s_rx_channel_en = I2S_TRUE;
    i2s_master_handle[0].init.i2s_rx_dma_en = I2S_TRUE;
    i2s_master_handle[0].init.rx_data_valid_len = I2S_WORDSIZE_24bit;
    i2s_master_handle[0].init.rx_fifo_trigger_level = I2S_FIFO_TRIGGERL_LEVEL_11;
    

	
    i2s_master_handle[0].p_callback = &i2s0_master_callback;
    i2s_master_handle[0].irq = I2S0_IRQn;
	
    ms_i2s_master_init(&i2s_master_handle[0]);

    return STATUS_SUCCESS;
}




int32_t i2s0_master_send( uint32_t* left_chan_data, uint32_t* right_chan_data, uint32_t len)
{
    ms_i2s_master_send_data(&i2s_master_handle[0], left_chan_data, right_chan_data, len);
    return STATUS_SUCCESS;
}



int32_t i2s0_master_receive(  uint32_t *left_chan_data, uint32_t *right_chan_data, uint32_t len)
{ 
    ms_i2s_master_receive_data(&i2s_master_handle[0],  left_chan_data, right_chan_data,  len);
    return STATUS_SUCCESS;
}


int32_t i2s0_master_int_send( uint32_t* left_chan_data, uint32_t* right_chan_data, uint32_t len)
{
    ms_i2s_master_int_send_data(&i2s_master_handle[0], left_chan_data, right_chan_data, len);
    return STATUS_SUCCESS;
}



int32_t i2s0_master_int_receive(  uint32_t *left_chan_data, uint32_t *right_chan_data, uint32_t len)
{ 
    ms_i2s_master_int_receive_data(&i2s_master_handle[0],  left_chan_data, right_chan_data,  len);
    return STATUS_SUCCESS;
}


int32_t i2s0_master_dma_send( uint32_t* send_data, uint32_t len)
{
    ms_i2s_master_dma_send_data(&i2s_master_handle[0], send_data, len);
    return STATUS_SUCCESS;
}



int32_t i2s0_master_dma_receive(  uint32_t *receive_data, uint32_t len)
{ 
    ms_i2s_master_dma_receive_data(&i2s_master_handle[0],  receive_data, len);
    return STATUS_SUCCESS;
}



int32_t i2s0_master_deinit(void)
{
	ms_i2s_master_deinit(&i2s_master_handle[0]);
	memset((uint8_t *)&i2s_master_handle[0], 0, sizeof(i2s_master_handle[0]));
	return STATUS_SUCCESS;
}

void I2S0_IRQHandler(void)
{
    ms_i2s_master_irq_handler(&i2s_master_handle[0]);
}





void ms_i2s1_master_init_dma_tx(DmacHandle_Type **phdmatx)
{
    //    *phdmatx = NULL;
    DmacHandle_Type *hdmatx;
    hdmatx = ms_dmac_mgmt_alloc();
    *phdmatx = hdmatx;

    hdmatx->init.dst_tr_width = DMAC_XFER_WIDTH_32BITS;
    hdmatx->init.dst_addr_mode = DMAC_ADDRESS_MODE_NOC;
    hdmatx->init.dst_peri_type = DMAC_I2S1_TX;
    hdmatx->init.dst_msize = DMAC_MSIZE_1;
    hdmatx->init.src_peri_type = DMAC_MEM;
    hdmatx->init.src_tr_width = DMAC_XFER_WIDTH_32BITS;
    hdmatx->init.src_addr_mode = DMAC_ADDRESS_MODE_INC;
    hdmatx->init.src_msize = DMAC_MSIZE_1;
    ms_dmac_init(hdmatx);
}

 void ms_i2s1_master_init_dma_rx(DmacHandle_Type **phdmarx)
{
    //    *phdmatx = NULL;
    DmacHandle_Type *hdmarx;
    hdmarx = ms_dmac_mgmt_alloc();
    *phdmarx = hdmarx;

    hdmarx->init.dst_tr_width = DMAC_XFER_WIDTH_32BITS;
    hdmarx->init.dst_addr_mode = DMAC_ADDRESS_MODE_INC;
    hdmarx->init.dst_peri_type = DMAC_MEM;
    hdmarx->init.dst_msize = DMAC_MSIZE_1;
    hdmarx->init.src_peri_type = DMAC_I2S1_RX;
    hdmarx->init.src_tr_width = DMAC_XFER_WIDTH_32BITS;
    hdmarx->init.src_addr_mode =DMAC_ADDRESS_MODE_NOC ;
    hdmarx->init.src_msize = DMAC_MSIZE_1;
    ms_dmac_init(hdmarx);
}






void i2s1_master_reach_calllback(I2sHandle_Type *i2s_master)
{
     MS_LOGI(MS_DRIVER, "\r\nspi master int!\n");
}




void i2s1master_init_calllback(I2sHandle_Type *i2s_master)
{
   /*config the i2s1 pin mux*/
   

    /*config the i2s1*/
    MS_CLOCK_HAL_CLK_ENABLE_I2S1();

      /*config the i2s1 pre div*/
    ms_clock_hal_set_i2s1_div(i2s_master->init.i2s_div);


     /*config i2s1 pin mux*/
#if   defined ( __MS1008_V2 )
    ms_pinmux_hal_set_pinmux(PAD15,PIN15_I2S1_SCLK);
    ms_pinmux_hal_set_pinmux(PAD16,PIN16_I2S1_LRCK);
    ms_pinmux_hal_set_pinmux(PAD17,PIN17_I2S1_DI);
    ms_pinmux_hal_set_pinmux(PAD18,PIN18_I2S1_MCLK);
    ms_pinmux_hal_set_pinmux(PAD19,PIN19_I2S1_DO);
#else
    ms_pinmux_hal_set_pinmux(PAD5,PIN05_I2S1_SCLK);
    ms_pinmux_hal_set_pinmux(PAD6,PIN06_I2S1_LRCK);
    ms_pinmux_hal_set_pinmux(PAD7,PIN07_I2S1_DI);
    ms_pinmux_hal_set_pinmux(PAD8,PIN08_I2S1_MCLK);
    ms_pinmux_hal_set_pinmux(PAD9,PIN09_I2S1_DO);
#endif
          /*dma tx init*/
    ms_i2s1_master_init_dma_tx(&i2s_master->hdmatx);
    
    if (i2s_master->hdmatx)
    {
        i2s_master->hdmatx->parent = i2s_master;
    }
  
    /*dma rx init*/
    ms_i2s1_master_init_dma_rx(&i2s_master->hdmarx);

    if (i2s_master->hdmarx)
    {
        i2s_master->hdmarx->parent = i2s_master;
    }
	
    /*enable the interrupt*/
    ms_i2s_enable_cpu_interrupt(i2s_master) ;
}



void i2s1_master_deinit_calllback(I2sHandle_Type *i2s_master)
{

     /*config i2s1 pin mux as default*/
#if   defined ( __MS1008_V2 )
    ms_pinmux_hal_set_pinmux(PAD15,PIN15_GPIO_15);
    ms_pinmux_hal_set_pinmux(PAD16,PIN16_GPIO_16);
    ms_pinmux_hal_set_pinmux(PAD17,PIN17_GPIO_17);
    ms_pinmux_hal_set_pinmux(PAD18,PIN18_GPIO_18);
    ms_pinmux_hal_set_pinmux(PAD19,PIN19_GPIO_19);
#else
    ms_pinmux_hal_set_pinmux(PAD5,PIN05_GPIO_5);
    ms_pinmux_hal_set_pinmux(PAD6,PIN06_GPIO_6);
    ms_pinmux_hal_set_pinmux(PAD7,PIN07_GPIO_7);
    ms_pinmux_hal_set_pinmux(PAD8,PIN08_GPIO_8);
    ms_pinmux_hal_set_pinmux(PAD9,PIN09_GPIO_9);
#endif

    /*disable the interrupt*/
    ms_i2s_disable_cpu_interrupt(i2s_master);

    /*close the spi master clock*/
    MS_CLOCK_HAL_CLK_DISABLE_I2S1();

      
    if (i2s_master->hdmarx)
    {
        ms_dmac_mgmt_free(i2s_master->hdmarx);
    }

    if (i2s_master->hdmatx)
    {
        ms_dmac_mgmt_free(i2s_master->hdmatx);
    }
	
}



I2sCallback_Type  i2s1_master_callback =
{
       .init_callback = i2s1master_init_calllback,
	.deinit_callback = i2s1_master_deinit_calllback,
	.i2s_reach_callback = i2s1_master_reach_calllback,
};

int32_t i2s1_master_init(void)
{
    memset((uint8_t *)&i2s_master_handle[1], 0, sizeof(i2s_master_handle[1]));
  
    i2s_master_handle[1].instance = I2S1;
    i2s_master_handle[1].dma_instance = I2S1_DMA;
    i2s_master_handle[1].init.role = I2S_MASTER;
    i2s_master_handle[1].init.wss = CLOCK_CYCLES_32;
    i2s_master_handle[1].init.sclkg = SCLK_CLOCK_CYCLES_16; 
    i2s_master_handle[1].init.i2s_div = 4;
	
    i2s_master_handle[1].init.i2s_tx_block_en = I2S_TRUE;
    i2s_master_handle[1].init.i2s_tx_channel_en = I2S_TRUE;
    i2s_master_handle[1].init.i2s_tx_dma_en = I2S_TRUE;
    i2s_master_handle[1].init.tx_data_valid_len = I2S_WORDSIZE_24bit;
    i2s_master_handle[1].init.tx_fifo_trigger_level = I2S_FIFO_TRIGGERL_LEVEL_11;
    

    i2s_master_handle[1].init.i2s_rx_block_en = I2S_TRUE;
    i2s_master_handle[1].init.i2s_rx_channel_en = I2S_TRUE;
    i2s_master_handle[1].init.i2s_rx_dma_en = I2S_TRUE;
    i2s_master_handle[1].init.rx_data_valid_len = I2S_WORDSIZE_24bit;
    i2s_master_handle[1].init.rx_fifo_trigger_level = I2S_FIFO_TRIGGERL_LEVEL_11;
    

	
    i2s_master_handle[1].p_callback = &i2s1_master_callback;
    i2s_master_handle[1].irq = I2S1_IRQn;
	
    ms_i2s_master_init(&i2s_master_handle[1]);

    return STATUS_SUCCESS;
}




int32_t i2s1_master_send( uint32_t* left_chan_data, uint32_t* right_chan_data, uint32_t len)
{
    ms_i2s_master_send_data(&i2s_master_handle[1], left_chan_data, right_chan_data, len);
    return STATUS_SUCCESS;
}



int32_t i2s1_master_receive(  uint32_t *left_chan_data, uint32_t *right_chan_data, uint32_t len)
{ 
    ms_i2s_master_receive_data(&i2s_master_handle[1],  left_chan_data, right_chan_data,  len);
    return STATUS_SUCCESS;
}


int32_t i2s1_master_int_send( uint32_t* left_chan_data, uint32_t* right_chan_data, uint32_t len)
{
    ms_i2s_master_int_send_data(&i2s_master_handle[1], left_chan_data, right_chan_data, len);
    return STATUS_SUCCESS;
}



int32_t i2s1_master_int_receive(  uint32_t *left_chan_data, uint32_t *right_chan_data, uint32_t len)
{ 
    ms_i2s_master_int_receive_data(&i2s_master_handle[1],  left_chan_data, right_chan_data,  len);
    return STATUS_SUCCESS;
}


int32_t i2s1_master_dma_send( uint32_t* send_data, uint32_t len)
{
    ms_i2s_master_dma_send_data(&i2s_master_handle[1], send_data, len);
    return STATUS_SUCCESS;
}



int32_t i2s1_master_dma_receive(  uint32_t *receive_data, uint32_t len)
{ 
    ms_i2s_master_dma_receive_data(&i2s_master_handle[1],  receive_data, len);
    return STATUS_SUCCESS;
}

int32_t i2s1_master_deinit(void)
{
	ms_i2s_master_deinit(&i2s_master_handle[1]);
	memset((uint8_t *)&i2s_master_handle[1], 0, sizeof(i2s_master_handle[1]));
	return STATUS_SUCCESS;
}



void I2S1_IRQHandler(void)
{
    ms_i2s_master_irq_handler(&i2s_master_handle[1]);
}






