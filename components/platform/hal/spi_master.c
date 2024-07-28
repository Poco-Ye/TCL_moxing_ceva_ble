/*
 * spi_master.c
 *
 *  Created on: 2021年12月24日
 *      Author: haijun.mai
 */

#include <ms_clock_hal.h>
#include "ms_spi_master.h"
#include <stddef.h>
#include "ms_pinmux_hal.h"
#include <string.h>
#include "gpio.h"
#include "log.h"

SpiMasterHandle_Type spi_master_handle;



int32_t spi_master_cs_set_low()
{

    gpio_out_put_low_level(PAD3);

    return STATUS_SUCCESS;
}


int32_t spi_master_cs_set_high()
{

    gpio_out_put_high_level( PAD3);

    return STATUS_SUCCESS;
}



void spi_master_reach_calllback(SpiMasterHandle_Type *spi_master)
{
     MS_LOGI(MS_DRIVER, "\r\nspi master int!\n");
}



void spi_master_init_calllback(SpiMasterHandle_Type *spi_master)
{
   /*config the spi0 pin mux*/
   

    /*config the spi0*/
    MS_CLOCK_HAL_CLK_ENABLE_SPI0();

      /*config the spi0 pre div*/
    ms_clock_hal_set_spi0_div(2);


     /*config spi0 pin mux*/

    ms_pinmux_hal_set_pinmux(PAD2,PIN02_SPI0_CLK);
    ms_pinmux_hal_set_pinmux(PAD3,PIN03_GPIO_3);
    //ms_pinmux_hal_set_pinmux(PAD3,PIN03_SPI0_CS);
    ms_pinmux_hal_set_pinmux(PAD4,PIN04_SPI0_TXD);
    ms_pinmux_hal_set_pinmux(PAD5,PIN05_SPI0_RXD);
    gpio_out_init(PAD3);

     spi_master_cs_set_high();
	
    /*enable the interrupt*/
   ms_spi_master_enable_cpu_interrupt(spi_master);
}

void spi_master_deinit_calllback(SpiMasterHandle_Type *spi_master)
{

     /*config spi0 pin mux as default*/

    ms_pinmux_hal_set_pinmux(PAD2,PIN02_GPIO_2);
    ms_pinmux_hal_set_pinmux(PAD3,PIN03_GPIO_3);
    ms_pinmux_hal_set_pinmux(PAD4,PIN04_GPIO_4);
    ms_pinmux_hal_set_pinmux(PAD5,PIN05_GPIO_5);
    gpio_deinit(PAD3);


    /*disable the interrupt*/
    ms_spi_master_disable_cpu_interrupt(spi_master);

    /*close the spi master clock*/
    MS_CLOCK_HAL_CLK_DISABLE_SPI0();
}

SpiMasterCallback_Type  spi_master_callback =
{
       .init_callback = spi_master_init_calllback,
	.deinit_callback = spi_master_deinit_calllback,
	.spi_reach_callback = spi_master_reach_calllback,
};

int32_t spi_master_init(void)
{
    memset((uint8_t *)&spi_master_handle, 0, sizeof(spi_master_handle));
  
    spi_master_handle.instance = SPI0;
	
    spi_master_handle.init.role = MS_SPI_MASTER;
    spi_master_handle.init.protocol_frame_format = MOTOROLA_SPI;
    spi_master_handle.init.frame_forma= STD_SPI_FRF;/*Standard SPI Frame Format*/
	
    spi_master_handle.init.cpha = SCPOL_SCLK_LOW;
    spi_master_handle.init.cpol= SCPH_SCPH_MIDDLE;
    spi_master_handle.init.frame_size = FRAME_08BITS;
    spi_master_handle.init.sck_div = 4;
    spi_master_handle.init.transmit_fifo_threshold = 0;
    spi_master_handle.init.mode_transfer = TRANSMIT_AND_RECEIVE;

    spi_master_handle.init.sste = MS_SPI_SLAVE_SLECT_GOGGLE_DISABLE;
	 
    spi_master_handle.init.data_frames_number = 0;
    spi_master_handle.init.sample_delay = 0;
    spi_master_handle.init.rx_threshold_level = 0;
    
	
	
    spi_master_handle.p_callback = &spi_master_callback;
    spi_master_handle.irq = SPI0_IRQn;
	
    ms_spi_master_init(&spi_master_handle);

    return STATUS_SUCCESS;
}


int32_t spi_master_send(const uint8_t *data, uint16_t size, uint32_t timeout)
{
    ms_spi_master_send(&spi_master_handle, data,  size,  timeout);
    return STATUS_SUCCESS;
}



int32_t spi_master_receive( uint8_t *rx_data, uint16_t size, uint32_t timeout)
{
    ms_spi_master_receive(&spi_master_handle, rx_data,  size,  timeout);
    return STATUS_SUCCESS;
}


int32_t spi_master_interrupt_send(const uint8_t *data, uint16_t size, uint32_t timeout)
{
    ms_spi_master_int_send(&spi_master_handle, data,  size,  timeout);
    return STATUS_SUCCESS;
}


int32_t spi_master_interrupt_receive( uint8_t *rx_data, uint16_t size, uint32_t timeout)
{
    ms_spi_master_int_receive(&spi_master_handle, rx_data,  size,  timeout);
    return STATUS_SUCCESS;
}


int32_t spi_master_deinit(void)
{
	ms_spi_master_deinit(&spi_master_handle);
	memset((uint8_t *)&spi_master_handle, 0, sizeof(spi_master_handle));
	return STATUS_SUCCESS;
}

void SPI0_IRQHandler(void)
{
    ms_spi_master_irq_handler(&spi_master_handle);
}


















