/*
 * spi_slaver.c
 *
 *  Created on: 2021年12月24日
 *      Author: haijun.mai
 */

#include <ms_clock_hal.h>
#include "ms_spi_slave.h"
#include <stddef.h>
#include "ms_pinmux_hal.h"
#include <string.h>
#include "gpio.h"
#include "log.h"

SpiSlaveHandle_Type spi_slave_handle;



int32_t spi_slave_cs_set_low()
{
    gpio_out_put_low_level(PAD3);
    return STATUS_SUCCESS;
}


int32_t spi_slave_cs_set_high()
{
    gpio_out_put_high_level( PAD3);
    return STATUS_SUCCESS;
}



void spi_slave_reach_calllback(SpiSlaveHandle_Type *spi_slave)
{
     //MS_LOGI(MS_DRIVER, "\r\nspi slave int!\n");
}



void spi_slave_init_calllback(SpiSlaveHandle_Type *spi_slave)
{
   /*config the spi0 pin mux*/
   

    /*config the spi1*/
    MS_CLOCK_HAL_CLK_ENABLE_SPI1();

      /*config the spi1 pre div*/
    ms_clock_hal_set_spi1_div(4);


     /*config spi1 pin mux*/
    ms_pinmux_hal_set_pinmux(PAD6,PIN06_SPI1_CLK);
    ms_pinmux_hal_set_pinmux(PAD7,PIN07_SPI1_CS);
    ms_pinmux_hal_set_pinmux(PAD16,PIN16_SPI1_TXD);
    ms_pinmux_hal_set_pinmux(PAD17,PIN17_SPI1_RXD);

	
    /*enable the interrupt*/
   ms_spi_slave_enable_cpu_interrupt(spi_slave);
}

void spi_slave_deinit_calllback(SpiSlaveHandle_Type *spi_slave)
{

     /*config spi0 pin mux as default*/
    ms_pinmux_hal_set_pinmux(PAD6,PIN06_GPIO_6);
    ms_pinmux_hal_set_pinmux(PAD7,PIN07_GPIO_7);
    ms_pinmux_hal_set_pinmux(PAD16,PIN16_GPIO_16);
    ms_pinmux_hal_set_pinmux(PAD17,PIN17_GPIO_17);


    /*disable the interrupt*/
    ms_spi_slave_disable_cpu_interrupt(spi_slave);

    /*close the spi slave clock*/
    MS_CLOCK_HAL_CLK_DISABLE_SPI1();
}

SpiSlaveCallback_Type  spi_slave_callback =
{
       .init_callback = spi_slave_init_calllback,
	.deinit_callback = spi_slave_deinit_calllback,
	.spi_reach_callback = spi_slave_reach_calllback,
};

int32_t spi_slave_init(void)
{
    memset((uint8_t *)&spi_slave_handle, 0, sizeof(spi_slave_handle));
  
    spi_slave_handle.instance = SPI1;
	
    spi_slave_handle.init.role = MS_SPI_SLAVE;
    spi_slave_handle.init.protocol_frame_format = MOTOROLA_SPI;
    spi_slave_handle.init.frame_forma= STD_SPI_FRF;/*Standard SPI Frame Format*/
	
    spi_slave_handle.init.cpha = SCPOL_SCLK_LOW;
    spi_slave_handle.init.cpol= SCPH_SCPH_MIDDLE;
    spi_slave_handle.init.frame_size = FRAME_08BITS;
    //spi_slave_handle.init.sck_div = 4;
    spi_slave_handle.init.transmit_fifo_threshold = 0;
    spi_slave_handle.init.mode_transfer = TRANSMIT_AND_RECEIVE;

    spi_slave_handle.init.sste = MS_SPI_SLAVE_SLECT_GOGGLE_DISABLE;
	 
    //spi_slave_handle.init.data_frames_number = 0;
    spi_slave_handle.init.sample_delay = 0;
    spi_slave_handle.init.rx_threshold_level = 0;
    
	
	
    spi_slave_handle.p_callback = &spi_slave_callback;
    spi_slave_handle.irq = SPI1_IRQn;
	
    ms_spi_slave_init(&spi_slave_handle);

    return STATUS_SUCCESS;
}


int32_t spi_slave_send(const uint8_t *data, uint16_t size, uint32_t timeout)
{
    ms_spi_slave_send(&spi_slave_handle, data,  size,  timeout);
    return STATUS_SUCCESS;
}



int32_t spi_slave_receive( uint8_t *rx_data, uint16_t size, uint32_t timeout)
{
    ms_spi_slave_receive(&spi_slave_handle, rx_data,  size,  timeout);
    return STATUS_SUCCESS;
}


int32_t spi_slave_interrupt_send(const uint8_t *data, uint16_t size, uint32_t timeout)
{
    ms_spi_slave_int_send(&spi_slave_handle, data,  size,  timeout);
    return STATUS_SUCCESS;
}


int32_t spi_slave_interrupt_receive( uint8_t *rx_data, uint16_t size, uint32_t timeout)
{
    ms_spi_slave_int_receive(&spi_slave_handle, rx_data,  size,  timeout);
    return STATUS_SUCCESS;
}


int32_t spi_slave_deinit(void)
{
	ms_spi_slave_deinit(&spi_slave_handle);
	memset((uint8_t *)&spi_slave_handle, 0, sizeof(spi_slave_handle));
	return STATUS_SUCCESS;
}

void SPI1_IRQHandler(void)
{
    ms_spi_slave_irq_handler(&spi_slave_handle);
}


