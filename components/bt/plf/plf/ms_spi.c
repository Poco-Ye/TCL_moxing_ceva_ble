

#include "NUCLEI_N.h"
#include "ms_spi.h"
//#include "ms_sys_ctrl.h"
//#include "ms_gpio.h"
//#include "ms_pinmux.h"
#include "ms_clock_hal.h"
#include "spi_master.h"
#include "ms_pinmux_hal.h"
#include "gpio.h"


//extern void NVIC_EnableIRQ(IRQn_Type IRQn);
//extern void NVIC_DisableIRQ(IRQn_Type IRQn);

ms_spi_callback_func g_ms_spi_callback_handler[MS_SPI_NUM] = {0};

uint32_t getSpixbaseViaIdx(uint8_t spi_idx);

void ms_spi_struct_init(ms_spi_dev_t * init_struct)
{
    init_struct->config.freq = 1000000; // 1M
    init_struct->config.mode = SPI_ROLE_MASTER;
    init_struct->priv = NULL;
}


void SPI0_master_config(ms_spi_dev_t * dev)
{

    /* spi config structrue init */
    ms_spi_struct_init(dev);
    dev->port = MS_SPI0_INDEX;
    /* initialize spi */
    dev->config.data_frame_size = 31;
    ms_spi_init(dev);
}


void ms_spi_interrupt_config(uint32_t spi_base, uint8_t spi_interrupt, uint8_t new_state)
{

#if 0
    if(new_state == ENABLE)
        SPIx->IMSC |= spi_interrupt;
    else
        SPIx->IMSC &= ~spi_interrupt;
#endif
}

void ms_spi_cs_init(void)
{
/*
	//enable gpio clk
      set_clk_enable(GPIO_RC_CLK, 1);  
      set_ctrl_clk_enable(GPIO_CTRL_CLK, 1);
     //set gpio3 out put as cs
     set_gpio_portA_direction(GPIO_BIT_3, GPIO_DIR_OUTPUT); */  // pyxue
     /*config spi0 pin mux*/

    ms_pinmux_hal_set_pinmux(PAD2,PIN02_SPI0_CLK);
    ms_pinmux_hal_set_pinmux(PAD3,PIN03_GPIO_3);
   // ms_pinmux_hal_set_pinmux(PAD3,PIN03_SPI0_CS);
    ms_pinmux_hal_set_pinmux(PAD4,PIN04_SPI0_TXD);
    ms_pinmux_hal_set_pinmux(PAD5,PIN05_SPI0_RXD);

    gpio_out_init(PAD3);     
     
}

int32_t ms_spi_dma_config(ms_spi_dev_t * spi,uint8_t dma_tx_rx_sel,uint8_t new_state)
{
    uint32_t spi_base = getSpixbaseViaIdx(spi->port); 
    if(SPI_UNVALID_ADDR == spi_base)
    {
		    return -1;
    }
    reg_SPI_DMACR   spi_dmacr   ;
    reg_SPI_DMATDLR spi_dmatdlr ;
    reg_SPI_DMARDLR spi_dmardlr ;
	
	  spi_dmatdlr.reg_value = inl(spi_base+SPI_DMATDLR_OFFSET);
    spi_dmatdlr.DMATDL      = 0;
    outl(spi_base+SPI_DMATDLR_OFFSET, spi_dmatdlr.reg_value);   // enable TX DMA level
    
    spi_dmardlr.reg_value = inl(spi_base+SPI_DMARDLR_OFFSET);
    spi_dmardlr.DMARDL      = 0;
    outl(spi_base+SPI_DMARDLR_OFFSET, spi_dmardlr.reg_value);   // enable RX DMA level

    spi_dmacr.reg_value = inl(spi_base+SPI_DMACR_OFFSET);	

    if(new_state == ENABLE)
    {
        spi_dmacr.RDMAE         = 1;
        spi_dmacr.TDMAE         = 1;
    }
    else
    {
        spi_dmacr.RDMAE         = 0;
        spi_dmacr.TDMAE         = 0;
    }	
    outl(spi_base+SPI_DMACR_OFFSET, spi_dmacr.reg_value);       // enable TX/RX DMA	
#if 0
    if(new_state == ENABLE)
    {
        SPIx->DMA_CR |= dma_tx_rx_sel;
    }
    else
    {
        SPIx->DMA_CR &= ~dma_tx_rx_sel;
    }
#endif
    return 0;
}

void ms_spi_cmd(uint32_t spi_base, uint8_t new_state)
{
#if 0
    if(new_state == ENABLE)
        SPIx->CR1 |= (0x1<<1);
    else
        SPIx->CR1 &= ~(0x1<<1);
#endif
}

int32_t ms_spi_struct_check(ms_spi_dev_t * spi)
{
    if(NULL == spi)
    {
        return -1;
    }
    if(0 == spi->config.freq)
    {
        return -1;
    }
    return 0;
}

int32_t ms_spi_init(ms_spi_dev_t * spi)
{
    uint32_t spi_base;
    reg_SPI_SSIENR  spi_ssienr  ;
    reg_SPI_CTRLR0  spi_ctrlr0  ;
    reg_SPI_CTRLR1  spi_ctrlr1  ;
    reg_SPI_BAUDR   spi_baudr   ;
    reg_SPI_TXFTLR  spi_txftlr  ;
    reg_SPI_RXFTLR  spi_rxftlr  ;
    reg_SPI_IMR     spi_imr     ;
    reg_SPI_SER     spi_ser     ;
    //reg_SPI_DMACR   spi_dmacr   ;

   //  set_clk_enable(SPI0_CLK, 1);  pyxue
   //  set_ctrl_clk_enable(SPI0_CTRL_CLK, 1);  //pyxue
       MS_CLOCK_HAL_CLK_ENABLE_SPI0();
	   ms_clock_hal_set_spi0_div(2);

    if(0 > ms_spi_struct_check(spi))	
    {
        return -1;
    }
   	
    spi_base = getSpixbaseViaIdx(spi->port); 
    if(SPI_UNVALID_ADDR == spi_base)
    {
        return -1;
    }

    spi_ssienr.reg_value    = inl(spi_base+SPI_SSIENR_OFFSET);  // disable spi
    spi_ssienr.SSI_EN       = 0x0;
    outl(spi_base+SPI_SSIENR_OFFSET, spi_ssienr.reg_value);     // disable spi

    /* set frame format */
    spi_ctrlr0.reg_value    = inl(spi_base+SPI_CTRLR0_OFFSET);
    spi_ctrlr0.DFS_32       = spi->config.data_frame_size;                             // data frame
   // spi_ctrlr0.DFS_32       = 0x1f;                             // data frame
    spi_ctrlr0.SSTE         = 0x1;                              //
    outl(spi_base+SPI_CTRLR0_OFFSET, spi_ctrlr0.reg_value);
	
    spi_ctrlr1.reg_value    = inl(spi_base+SPI_CTRLR1_OFFSET);
    spi_ctrlr1.NDF          = 0x05;                             // number of data frame
    outl(spi_base+SPI_CTRLR1_OFFSET, spi_ctrlr1.reg_value);
	
    spi_baudr.reg_value     = inl(spi_base+SPI_BAUDR_OFFSET);
    spi_baudr.SCKDV         = SYSTEM_CLOCK/spi->config.freq;   // baud rate
    outl(spi_base+SPI_BAUDR_OFFSET, spi_baudr.reg_value);

    spi_txftlr.reg_value    = inl(spi_base+SPI_TXFTLR_OFFSET);
    spi_txftlr.TFT          = 0x11;                             // tx fifo depth
    outl(spi_base+SPI_TXFTLR_OFFSET, spi_txftlr.reg_value);

    spi_rxftlr.reg_value    = inl(spi_base+SPI_RXFTLR_OFFSET);
    spi_rxftlr.RFT          = 0x11;                             // rx fifo depth
    outl(spi_base+SPI_RXFTLR_OFFSET, spi_rxftlr.reg_value);

    spi_imr.reg_value       = inl(spi_base+SPI_IMR_OFFSET);
    spi_imr.TXEIM           = 0x0;
    spi_imr.TXOIM           = 0x0;
    spi_imr.RXUIM           = 0x0;
    spi_imr.RXOIM           = 0x0;
    spi_imr.RXFIM           = 0x0;
    spi_imr.MSTIM           = 0x0;
    outl(spi_base+SPI_IMR_OFFSET, spi_imr.reg_value);           // disable interrupt

    spi_ser.reg_value       = inl(spi_base+SPI_SER_OFFSET);
    spi_ser.SER             = 0x1;
    outl(spi_base+SPI_SER_OFFSET, spi_ser.reg_value);           // slave number
    if(spi->config.mode == SPI_ROLE_MASTER)
    {
        outl(SPI1_BASE_ADDR+SPI_SSIENR_OFFSET, 0x00000000);
        spi_ctrlr0.reg_value    = inl(spi_base+SPI_CTRLR0_OFFSET);
        spi_ctrlr0.SLV_OE    = 1;
        outl(SPI1_BASE_ADDR+SPI_CTRLR0_OFFSET, spi_ctrlr0.reg_value);    //slave output disable
        outl(SPI1_BASE_ADDR+SPI_SSIENR_OFFSET, 0x00000001);
    }
    else
    {
        outl(SPI1_BASE_ADDR+SPI_SSIENR_OFFSET, 0x00000000);
        spi_ctrlr0.reg_value    = inl(spi_base+SPI_CTRLR0_OFFSET);
        spi_ctrlr0.SLV_OE    = 0;			
        outl(SPI1_BASE_ADDR+SPI_CTRLR0_OFFSET, spi_ctrlr0.reg_value);
    
        outl(SPI1_BASE_ADDR+SPI_SSIENR_OFFSET, 0x00000001);
    }
    // spi_dmacr.reg_value     = inl(spi_base+SPI_DMACR_OFFSET);
    // spi_dmacr.RDMAE         = 0x1;
    // spi_dmacr.TDMAE         = 0x1;
    // outl(spi_base+SPI_DMACR_OFFSET, spi_dmacr.reg_value);       // enable TX/RX DMA

    spi_ssienr.reg_value    = inl(spi_base+SPI_SSIENR_OFFSET);  
    spi_ssienr.SSI_EN       = 0x1;
    outl(spi_base+SPI_SSIENR_OFFSET, spi_ssienr.reg_value);     // enable spi
	
    if(spi->priv)
    {
        //enable cm4 interrupt
        if(MS_SPI0_INDEX == spi->port)
        {
            //NVIC_EnableIRQ(SPI0_IRQn);
            ECLIC_EnableIRQ(SPI0_IRQn);
        }
        else
        {
            ECLIC_EnableIRQ(SPI1_IRQn);			
        }
        g_ms_spi_callback_handler[spi->port] = (ms_spi_callback_func)(spi->priv);
    }

    return 0;
}

/**
 * De-initialises a SPI interface
 *
 *
 * @param[in]  spi the SPI device to be de-initialised
 *
 * @return  0 : on success, -1 : if an error occurred
 */
int32_t ms_spi_deinit(ms_spi_dev_t *spi)
{
    uint32_t spi_base;

    if(NULL == spi)
    {
        return -1;
    }

	spi_base = getSpixbaseViaIdx(spi->port); 
    if(SPI_UNVALID_ADDR == spi_base)
    {
		return -1;
    }

    //disable all spi interrupt

    //disable all spi config


    //disable cm4 interrupt
    if(MS_SPI0_INDEX == spi->port)
    {
        //NVIC_DisableIRQ(SPI0_IRQn);
        ECLIC_DisableIRQ(SPI0_IRQn);
    }
    else
    {
        //NVIC_DisableIRQ(SPI1_IRQn);
        ECLIC_DisableIRQ(SPI1_IRQn);
    }

    g_ms_spi_callback_handler[spi->port] = NULL;
    return 0;
}

int32_t ms_spi_send(ms_spi_dev_t *spi, const uint32_t *data, uint16_t size, uint32_t timeout)
{
    reg_SPI_SR  spi_sr  ;
    uint32_t snd_d;

    uint32_t spi_base = getSpixbaseViaIdx(spi->port);
    if(SPI_UNVALID_ADDR == spi_base)
    {
         return -1;
    }

   spi_sr.reg_value = inl(spi_base+SPI_SR_OFFSET);
    while(spi_sr.RFNE)
    {
          inl(spi_base+SPI_DRX_OFFSET);
          spi_sr.reg_value = inl(spi_base+SPI_SR_OFFSET);
    }

    while(size)
    {
        //wait till tx fifo is not full
       
       do{
            spi_sr.reg_value = inl(spi_base+SPI_SR_OFFSET);
        }while(!spi_sr.TFE);
	 snd_d = *data;
	 data++;
        outl(spi_base+SPI_DRX_OFFSET, snd_d);
	 size -= 1;
    }

    do{
            spi_sr.reg_value = inl(spi_base+SPI_SR_OFFSET);
    }while(spi_sr.BUSY);

    return 0;
}

int32_t ms_spi_recv(ms_spi_dev_t *spi, uint32_t *rx_data, uint16_t size, uint32_t timeout)
{
    reg_SPI_SR  spi_sr  ;
    uint32_t rcv_d;

    uint32_t spi_base = getSpixbaseViaIdx(spi->port);
    if(SPI_UNVALID_ADDR == spi_base)
    {
		return -1;
    }
	
    while(size)
    {
        //wait till rx fifo is not empty,
        do{
            spi_sr.reg_value = inl(spi_base+SPI_SR_OFFSET);
        }while(!spi_sr.RFNE);

        rcv_d = inl(spi_base+SPI_DRX_OFFSET);
	size -= 1;
	 *rx_data = rcv_d;
	 rx_data += 1;
    }
    return 0;
}

static void ms_spi_cs_low(void)
{

spi_master_cs_set_low();

/*	int value = 0;
	//gpio 3 low
	value = get_gpio_portA_input();
	value &= ~GPIO_BIT_3;
	set_gpio_portA_output(value);
	delay(10);  */

}

static void ms_spi_cs_high(void)
{
   spi_master_cs_set_high();
/*	int value = 0;
	 delay(10);
	//gpio 3 high
	value = get_gpio_portA_input();
	value |= GPIO_BIT_3;
	set_gpio_portA_output(value); */

}

void ms_spi_cs_enable(ms_spi_dev_t *rf_spi0)
{
    ms_spi_cs_low();
}

void ms_spi_cs_disable(ms_spi_dev_t *rf_spi0)
{
    ms_spi_cs_high();
}

uint32_t getSpixbaseViaIdx(uint8_t spi_idx)
{
    switch(spi_idx){
        case MS_SPI0_INDEX:
            return SPI0_BASE_ADDR;
        case MS_SPI1_INDEX:
            return SPI1_BASE_ADDR;
        default:
            return SPI_UNVALID_ADDR;
    }
}

void SPIX_IRQHandler(uint8_t spi_idx) {
    reg_SPI_IMR imr;
    reg_SPI_ISR isr;
    if(spi_idx >= MS_SPI_NUM)
        return;
	
    uint32_t spi_base = getSpixbaseViaIdx(spi_idx);
    isr.reg_value = inl(spi_base + SPI_ISR_OFFSET);

    //set interrupt disable 
    imr.reg_value = inl(spi_base + SPI_IMR_OFFSET);
    if(isr.TXEIS ==1) {
        //outl(SPI_DEBUG2+0+100*spi, 0x00000001 <<0);
        imr.TXEIM = 0;
    }
    else if(isr.TXOIS == 1) {
        //outl(SPI_DEBUG2+4+100*spi, 0x00000001 <<1);
        imr.TXOIM = 0;
    }
    else if(isr.RXUIS == 1) {
        //outl(SPI_DEBUG2+8+100*spi, 0x00000001 <<2);
        imr.RXUIM = 0;
    }
    else if(isr.RXOIS == 1) {
        //outl(SPI_DEBUG2+12+100*spi, 0x00000001 <<3);
        imr.RXOIM = 0;
    }
    else if(isr.RXFIS == 1) {
        //outl(SPI_DEBUG2+16+100*spi, 0x00000001 <<4);
        imr.RXFIM = 0;
    }
    outl(spi_base + SPI_IMR_OFFSET, imr.reg_value);
}

/*
void SPI0_IRQHandler()
{
    SPIX_IRQHandler(MS_SPI0_INDEX);
}

void SPI1_IRQHandler()
{
    SPIX_IRQHandler(MS_SPI1_INDEX);
}
*/
void ms_spi_set_callback(uint8_t spi_idx,ms_spi_callback_func func)
{
    if(spi_idx >= MS_SPI_NUM)
        return;
    g_ms_spi_callback_handler[spi_idx] = func;

}
