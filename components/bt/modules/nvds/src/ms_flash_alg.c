/***********************************************************************/
/*  This file is part of the ARM Toolchain package                     */
/*  Copyright (c) 2010 Keil - An ARM Company. All rights reserved.     */
/***********************************************************************/
/*                                                                     */
/*  FlashDev.C:  Flash Programming Functions adapted                   */
/*               for New Device 256kB Flash                            */
/*                                                                     */
/***********************************************************************/
#include <stdlib.h>
#include <string.h>
#include "ms.h"
#include "system_ms.h"
#include "ms_common.h"
#include "ms_flash_alg.h"
//#include "ms_clk_ctrl.h"
#include "atiny_log.h"

unsigned int flash_calibrate_cr = 0;


static unsigned int qspi_indac_tri_addr;
static unsigned int qspi_indac_tri_addr_last;

// wait STIG command done
FLASH_DRIVER_SEG void qspi_stig_wait_done(void) 
{
    reg_QSPI_FLASH_CMD_CTRL qspi_flash_cmd_ctrl;
    do {
        qspi_flash_cmd_ctrl.reg_value = inl(QSPI_FLASH_CMD_CTRL);
    } while(qspi_flash_cmd_ctrl.STIG_EXC_P);
}

// wait WIP
FLASH_DRIVER_SEG void qspi_stig_wait_wip(void) 
{
    reg_QSPI_FLASH_CMD_CTRL     qspi_flash_cmd_ctrl ;
    reg_QSPI_FLASH_CMD_RDATA_L  qspi_flash_cmd_rdata;

    qspi_flash_cmd_ctrl.reg_value   = 0;
    qspi_flash_cmd_ctrl.CMD_OPCODE  = FLASH_READ_STATUS;
    qspi_flash_cmd_ctrl.R_DATA_EN   = 1;
    qspi_flash_cmd_ctrl.R_BYTE_NUM  = 0;
    qspi_flash_cmd_ctrl.EXC_CMD     = 1;
    do {
        outl(QSPI_FLASH_CMD_CTRL, qspi_flash_cmd_ctrl.reg_value);
        qspi_stig_wait_done();
        qspi_flash_cmd_rdata.reg_value = inl(QSPI_FLASH_CMD_RDATA_L);
    } while (qspi_flash_cmd_rdata.reg_value & 0x01);
}


// send single command (e.g. RESET) by STIG
FLASH_DRIVER_SEG void qspi_stig_single_command(FLASH_CMD_TYPE cmd) {
    reg_QSPI_FLASH_CMD_CTRL     qspi_flash_cmd_ctrl;

    qspi_flash_cmd_ctrl.reg_value   = 0;
    qspi_flash_cmd_ctrl.CMD_OPCODE  = cmd;
    qspi_flash_cmd_ctrl.EXC_CMD     = 1;

    outl(QSPI_FLASH_CMD_CTRL, qspi_flash_cmd_ctrl.reg_value);
    qspi_stig_wait_done();
}

// write status register (e.g. enable QE)
FLASH_DRIVER_SEG void qspi_stig_write_status(unsigned int status, unsigned int size) {
    reg_QSPI_FLASH_CMD_CTRL     qspi_flash_cmd_ctrl ;

    outl(QSPI_FLASH_CMD_WDATA_L, status);

    qspi_flash_cmd_ctrl.reg_value   = 0;
    qspi_flash_cmd_ctrl.CMD_OPCODE  = FLASH_WRITE_STATUS;
    qspi_flash_cmd_ctrl.W_DATA_EN   = 1;
    qspi_flash_cmd_ctrl.W_BYTE_NUM  = size - 1;
    qspi_flash_cmd_ctrl.EXC_CMD     = 1;
    outl(QSPI_FLASH_CMD_CTRL, qspi_flash_cmd_ctrl.reg_value);
    qspi_stig_wait_done();
    delay(10);   // wait 2us (wait for WIP arise)
}

FLASH_DRIVER_SEG UINT32   qspi_stig_read_spi_flash_Identification(void) 
{
    reg_QSPI_FLASH_CMD_CTRL     qspi_flash_cmd_ctrl ;
    reg_QSPI_FLASH_CMD_RDATA_L  qspi_flash_cmd_rdata;

    qspi_flash_cmd_ctrl.reg_value   = 0;
    qspi_flash_cmd_ctrl.CMD_OPCODE  = 0x9f;
    qspi_flash_cmd_ctrl.R_DATA_EN   = 1;
    qspi_flash_cmd_ctrl.R_BYTE_NUM  = 2;
    qspi_flash_cmd_ctrl.EXC_CMD     = 1;

    
    outl(QSPI_FLASH_CMD_CTRL, qspi_flash_cmd_ctrl.reg_value);

	 
    qspi_stig_wait_done();
   
	
    qspi_flash_cmd_rdata.reg_value = inl(QSPI_FLASH_CMD_RDATA_L)&0xffffff;
    return qspi_flash_cmd_rdata.reg_value;
   
}




FLASH_DRIVER_SEG UINT32   qspi_stig_rwrite_byte(UINT32 write_addr,char data) 
{
	reg_QSPI_FLASH_CMD_CTRL     qspi_flash_cmd_ctrl ;
	

	//write enable 
	qspi_stig_single_command(FLASH_WRITE_ENABLE);   // enable write_enable
	
	//write addr 
	outl(QSPI_FLASH_CMD_ADR, write_addr);

	//write data
	outl(QSPI_FLASH_CMD_WDATA_L, data);

	qspi_flash_cmd_ctrl.reg_value   = 0;
	qspi_flash_cmd_ctrl.CMD_OPCODE  = FLASH_PAGE_PROGRAM;
	qspi_flash_cmd_ctrl.ADR_EN = 1;
	qspi_flash_cmd_ctrl.ADR_BYTE_NUM = 2;
	qspi_flash_cmd_ctrl.W_DATA_EN = 1;
	qspi_flash_cmd_ctrl.W_BYTE_NUM = 0;
	qspi_flash_cmd_ctrl.EXC_CMD     = 1;
	

	outl(QSPI_FLASH_CMD_CTRL, qspi_flash_cmd_ctrl.reg_value);

	qspi_stig_wait_done();


	qspi_stig_wait_wip();
	return 0;

}


// config DAC/INDAC spi mode
FLASH_DRIVER_SEG void qspi_instr_rdwr_config(QSPI_MODE_TYPE mode) 
{
    reg_QSPI_DEV_RDINSTR_CFG    qspi_dev_rdinstr_cfg;
    reg_QSPI_DEV_WRINSTR_CFG    qspi_dev_wrinstr_cfg;

    switch (mode) {
    case QSPI_QPI_MODE:
        //qspi_stig_single_command(FLASH_ENABLE_QPI);
        qspi_stig_single_command(FLASH_WRITE_ENABLE);   // enable write_enable
        qspi_stig_write_status((1<<9), 2);              // enable data quad-mode
        qspi_stig_wait_wip();

        qspi_dev_rdinstr_cfg.reg_value      = 0;
#if 0
        qspi_dev_rdinstr_cfg.R_OPCODE       = 0x6B;     // GD Quad Output Fast Read
        qspi_dev_rdinstr_cfg.INSTR_TRAN_TYPE= 0x0;      // spi opcode
        qspi_dev_rdinstr_cfg.ADR_TRAN_TYPE  = 0x0;      // spi address
        qspi_dev_rdinstr_cfg.DATA_TRAN_TYPE = 0x2;      // qpi data
        qspi_dev_rdinstr_cfg.DUMMY_CLOCK_NUM= 8;
#else
        qspi_dev_rdinstr_cfg.R_OPCODE       = 0xEB;     // GD Quad Output Fast Read
        qspi_dev_rdinstr_cfg.INSTR_TRAN_TYPE= 0x0;      // spi opcode
        qspi_dev_rdinstr_cfg.ADR_TRAN_TYPE  = 0x2;      // spi address
        qspi_dev_rdinstr_cfg.DATA_TRAN_TYPE = 0x2;      // qpi data
        qspi_dev_rdinstr_cfg.DUMMY_CLOCK_NUM= 4;        // dump 4cycle
        qspi_dev_rdinstr_cfg.MODE_BIT_EN    = 1;        // enable mode bit
        outl(QSPI_MODE_BIT, 0x00);                      // mode bit = 0x00
#endif
        outl(QSPI_DEV_RDINSTR_CFG, qspi_dev_rdinstr_cfg.reg_value);

        qspi_dev_wrinstr_cfg.reg_value      = 0;
        qspi_dev_wrinstr_cfg.W_OPCODE       = 0x32;     // GD Quad Page Program
        qspi_dev_wrinstr_cfg.ADR_TRAN_TYPE  = 0x0;      // spi address
        qspi_dev_wrinstr_cfg.DATA_TRAN_TYPE = 0x2;      // qpi data
        outl(QSPI_DEV_WRINSTR_CFG, qspi_dev_wrinstr_cfg.reg_value);
        break;
    case QSPI_DPI_MODE:
    default:    // QSPI_SPI_MODE
        qspi_stig_single_command(FLASH_WRITE_ENABLE);   // enable write_enable
        qspi_stig_write_status((0<<9), 2);              // disable data quad-mode
        qspi_stig_wait_wip();

        qspi_dev_rdinstr_cfg.reg_value      = QSPI_DEV_RDINSTR_CFG_RESET_VALUE;
        qspi_dev_wrinstr_cfg.reg_value      = QSPI_DEV_WRINSTR_CFG_RESET_VALUE;
        outl(QSPI_DEV_RDINSTR_CFG, qspi_dev_rdinstr_cfg.reg_value);
        outl(QSPI_DEV_WRINSTR_CFG, qspi_dev_wrinstr_cfg.reg_value);
        break;
    }
}




// for sector-erase and block-erase
FLASH_DRIVER_SEG void qspi_stig_erase(FLASH_CMD_TYPE cmd, unsigned int addr) 
{
    reg_QSPI_FLASH_CMD_CTRL     qspi_flash_cmd_ctrl ;
    reg_QSPI_FLASH_CMD_ADR      qspi_flash_cmd_adr  ;

    // send enable write
    qspi_stig_single_command(FLASH_WRITE_ENABLE);

    // send erase command
    qspi_flash_cmd_adr.CMD_ADR      = addr;
    outl(QSPI_FLASH_CMD_ADR, qspi_flash_cmd_adr.reg_value);

    qspi_flash_cmd_ctrl.reg_value   = 0;
    qspi_flash_cmd_ctrl.CMD_OPCODE  = cmd;
    qspi_flash_cmd_ctrl.ADR_EN      = 1;
    qspi_flash_cmd_ctrl.ADR_BYTE_NUM= 3 - 1;
    qspi_flash_cmd_ctrl.EXC_CMD     = 1;
    outl(QSPI_FLASH_CMD_CTRL,qspi_flash_cmd_ctrl.reg_value);
    qspi_stig_wait_done();

    // wait erase done, sector erase ~= 40ms
    qspi_stig_wait_wip();
}

// DAC mode init
FLASH_DRIVER_SEG void qspi_dac_init(void) {
    reg_QSPI_CFG            qspi_cfg;

    qspi_stig_single_command(FLASH_RESET_ENABLE);
    qspi_stig_single_command(FLASH_RESET);

    //qspi_cfg.reg_value              = inl(QSPI_CFG);
    qspi_cfg.reg_value              = QSPI_CFG_RESET_VALUE;
    qspi_cfg.QSPI_EN                = 1;    // enable qspi (default)
    qspi_cfg.PER_SEL                = 0x0;  // select CSN0 (default)
    qspi_cfg.AHB_ADR_REMAP_EN       = 0;    // no need remap (default): 0x1000000 (AHB) -> 0x000000 (flash)
    qspi_cfg.BAUD_RATE_CFG          = 1;    // baud rate = ref_clk / 4
    outl(QSPI_CFG, qspi_cfg.reg_value);
}
// DAC word read (flash_addr = base + offset)
// exclude indac address [tri_addr , tri_addr_last]
FLASH_DRIVER_SEG __inline UINT32 qspi_dac_word_rd (UINT32 flash_addr) {
    return inl(flash_addr);
}
// DAC word write (flash_addr = base + offset)
// exclude indac address [tri_addr , tri_addr_last]
FLASH_DRIVER_SEG __inline void qspi_dac_word_wr (UINT32 flash_addr, UINT32 word) {
    outl(flash_addr, word);
}

// INDAC word read (fifo read !!!!)
FLASH_DRIVER_SEG unsigned int qspi_indac_word_rd (void) {
    return inl(qspi_indac_tri_addr);
}
// INDAC word write (fifo write !!!!)
FLASH_DRIVER_SEG void qspi_indac_word_wr (unsigned int word) {
    outl(qspi_indac_tri_addr, word);
}
// INDAC trigger setting 
FLASH_DRIVER_SEG void qspi_indac_set_tri_addr (UINT32 tri_addr, UINT32 tri_range) {
    qspi_indac_tri_addr      = tri_addr;
    qspi_indac_tri_addr_last = tri_addr + (2 << tri_range);
    outl(QSPI_IND_AHB_TRIGGER, tri_addr);
    outl(QSPI_IND_RANGE_WIDTH, tri_range);      // 2^tri_range (byte)
}
// INDAC read (flash_addr = offset)
FLASH_DRIVER_SEG void qspi_indac_rd (unsigned int flash_addr, unsigned int rd_byte) {
    reg_QSPI_IND_RD_CTRL        qspi_ind_rd_ctrl;

    outl(QSPI_IND_RD_ADR,flash_addr);
    outl(QSPI_IND_RD_BYTE, rd_byte);

    qspi_ind_rd_ctrl.reg_value = 0;
    qspi_ind_rd_ctrl.IND_READ_START = 1;
    outl(QSPI_IND_RD_CTRL, qspi_ind_rd_ctrl.reg_value);
}

// INDAC write (flash_addr = offset)
FLASH_DRIVER_SEG void qspi_indac_wr_cfg (unsigned int flash_addr, unsigned int wr_byte) {
    reg_QSPI_IND_WR_CTRL        qspi_ind_wr_ctrl;

    outl(QSPI_IND_WR_ADR, flash_addr);
    outl(QSPI_IND_WR_BYTE, wr_byte);

    qspi_ind_wr_ctrl.reg_value = 0;
    qspi_ind_wr_ctrl.IND_WRITE_START = 1;
    outl(QSPI_IND_WR_CTRL,qspi_ind_wr_ctrl.reg_value);
}


FLASH_DRIVER_SEG int ms_flash_alg_modify_clk(void)
{

    //ms_set_high_freq_clk_source(XCLK_SRC_PLL);
    //ms_set_ahb_clk_source(HCLK_SRC_PLL_OR_OSC16M); // ahb_src_clk from pll 96M
    //ms_set_clk_div(AHB_DIV,  2 - 1);               // qspi_clk = ahb_src_clk / 3
   // ms_set_clk_div(QSPI_DIV, 1 - 1);               // qspi_clk = ahb_src_clk / 1

    return 0;
}


FLASH_DRIVER_SEG int ms_flash_alg_init ()
{
    qspi_dac_init();
	
    qspi_instr_rdwr_config(QSPI_QPI_MODE);       // Config flash Mode
	
    return (0);                       
}

/*
 *  cmd: CHIP_ERASE_CMD or SECTOR_ERASE_CMD or BLOCK32_ERASE_CMD or BLOCK64_ERASE_CMD
 *  adr: not used for CHIP_ERASE_CMD
 */
FLASH_DRIVER_SEG int ms_flash_alg_erase(unsigned int cmd, unsigned int address) {
	
    // Erase the sector size
    qspi_stig_erase(cmd, address);

    return (0);                                  // Finished without Errors
}
unsigned long AlignWord(unsigned char * pucData)
{
    //
    // Align the bytes in the buffer into a 32-bit value.
    //
    return((pucData[3] << 24) | (pucData[2] << 16) | (pucData[1] << 8) |
           (pucData[0]));
}


/*
 *  Program  in Flash Memory
 *    Parameter:      address:   Page Start Address
 *                    sz:    Page Size
 *                    buf:   Data
 *    Return Value:   0 - OK,  1 - Failed
 */
FLASH_DRIVER_SEG int ms_flash_alg_programpage(unsigned int address, unsigned int sz, unsigned char *buf)
{
    unsigned int write_word = 0;
    int remain_sz = (int)sz;
	
    qspi_indac_set_tri_addr(address, 8);
	  // INDAC write:ulcount shoud be one page size
    qspi_indac_wr_cfg(address - 0x20000000, sz);

    //if need check flash status is idle
    //qspi_stig_wait_wip();
	
    while(0 < remain_sz)
    {
			  //transform char data to word
        write_word = AlignWord(buf);
			
        // Program the next word.
        qspi_indac_word_wr(write_word);        

        // Increment to the next word.
        buf += 4;
        remain_sz -= 4;		
    }

    //if need check flash status is idle		
	  qspi_stig_wait_wip();
		
    return(0);
}


#if 0
FLASH_DRIVER_SEG int ms_flash_alg_program(unsigned int address, unsigned int size, unsigned char *buf)
{
  unsigned int i = 0;
   
   for(i=0;i<size;i++)
  {
        qspi_stig_rwrite_byte(address+i,buf[i]);
   }
    return(0);
}

 #endif

 FLASH_DRIVER_SEG int ms_flash_alg_program(unsigned int address, unsigned int size, unsigned char *buf)
{
  unsigned int i = 0;
  //dbg_print("ms_flash_alg_program,%x\r\n",buf[0]);
   for(i=0;i<size;i++)
  {
        // *(volatile unsigned char *)(address+i) = buf[i];
	  outb(address+i,buf[i]);
   }

  //dbg_print("ms_flash_alg_program,write:%x\r\n",inb(address));
    return(0);
}

FLASH_DRIVER_SEG int ms_flash_stig_write8byte(unsigned int address, unsigned int size, unsigned char *buf)
{
    reg_QSPI_FLASH_CMD_CTRL     qspi_flash_cmd_ctrl ;
    reg_QSPI_FLASH_CMD_ADR      qspi_flash_cmd_adr  ;

    unsigned int i = 0;
    uint32_t flash_addr = address - FLASH_BASE_ADDR;
    uint32_t write_addr =  flash_addr;
    uint8_t* write_data = buf;
    
    for(i=0;i<size;i+=8)
    {
        write_addr = flash_addr + i;
        // send enable write
        qspi_stig_single_command(FLASH_WRITE_ENABLE);
        
        // send erase command
        qspi_flash_cmd_adr.CMD_ADR      = write_addr;
        outl(QSPI_FLASH_CMD_ADR, qspi_flash_cmd_adr.reg_value);

        outl(QSPI_FLASH_CMD_WDATA_L, *(uint32_t *)write_data);
        outl(QSPI_FLASH_CMD_WDATA_H, *(uint32_t *)(write_data+4));
        
        qspi_flash_cmd_ctrl.reg_value   = 0;
        qspi_flash_cmd_ctrl.CMD_OPCODE  = FLASH_PAGE_PROGRAM;
        qspi_flash_cmd_ctrl.ADR_EN      = 1;
        qspi_flash_cmd_ctrl.ADR_BYTE_NUM= 3 - 1;
        qspi_flash_cmd_ctrl.W_BYTE_NUM  = 7;
        qspi_flash_cmd_ctrl.W_DATA_EN   = 1;
        qspi_flash_cmd_ctrl.EXC_CMD     = 1;
        outl(QSPI_FLASH_CMD_CTRL,qspi_flash_cmd_ctrl.reg_value);
        qspi_stig_wait_done();
        
        // wait erase done, sector erase ~= 40ms
        qspi_stig_wait_wip();
        write_data += 8;

    }


     return(0);
}


FLASH_DRIVER_SEG int ms_flash_stig_write(unsigned int address, unsigned int size, unsigned char *buf)
{
    reg_QSPI_FLASH_CMD_CTRL     qspi_flash_cmd_ctrl ;
    reg_QSPI_FLASH_CMD_ADR      qspi_flash_cmd_adr  ;

    unsigned int i = 0;
    uint32_t flash_addr = address - FLASH_BASE_ADDR;
    uint32_t write_addr =  flash_addr;
    uint8_t* write_data = buf;
    
    for(i=0;i<size;i+=1)
    {
        write_addr = flash_addr + i;
        // send enable write
        qspi_stig_single_command(FLASH_WRITE_ENABLE);
        
        // send erase command
        qspi_flash_cmd_adr.CMD_ADR      = write_addr;
        outl(QSPI_FLASH_CMD_ADR, qspi_flash_cmd_adr.reg_value);

        outl(QSPI_FLASH_CMD_WDATA_L, *write_data);
        //outl(QSPI_FLASH_CMD_WDATA_H, *(uint32_t *)(write_data+4));
        
        qspi_flash_cmd_ctrl.reg_value   = 0;
        qspi_flash_cmd_ctrl.CMD_OPCODE  = FLASH_PAGE_PROGRAM;
        qspi_flash_cmd_ctrl.ADR_EN      = 1;
        qspi_flash_cmd_ctrl.ADR_BYTE_NUM= 3 - 1;
        qspi_flash_cmd_ctrl.W_BYTE_NUM  = 0;
        qspi_flash_cmd_ctrl.W_DATA_EN   = 1;
        qspi_flash_cmd_ctrl.EXC_CMD     = 1;
        outl(QSPI_FLASH_CMD_CTRL,qspi_flash_cmd_ctrl.reg_value);
        qspi_stig_wait_done();
        
        // wait erase done, sector erase ~= 40ms
        qspi_stig_wait_wip();
        write_data += 1;

    }


     return(0);
}

FLASH_DRIVER_SEG int ms_flash_stig_read8byte(unsigned int address, unsigned int size, unsigned char *buf)
{
    reg_QSPI_FLASH_CMD_CTRL     qspi_flash_cmd_ctrl ;
    reg_QSPI_FLASH_CMD_ADR      qspi_flash_cmd_adr  ;

    unsigned int i = 0;
    uint32_t flash_addr = address - FLASH_BASE_ADDR;
    uint32_t read_addr =  flash_addr;

    uint8_t* read_data = buf;
    
    for(i=0;i<size;i+=8)
    {
        read_addr = flash_addr + i;
        // send enable write
        qspi_stig_single_command(FLASH_WRITE_ENABLE);
        
        // send erase command
        
        qspi_flash_cmd_adr.CMD_ADR      = read_addr;
        outl(QSPI_FLASH_CMD_ADR, qspi_flash_cmd_adr.reg_value);

        
        qspi_flash_cmd_ctrl.reg_value   = 0;
        qspi_flash_cmd_ctrl.CMD_OPCODE  = FLASH_READ_DATA;
        qspi_flash_cmd_ctrl.ADR_EN      = 1;
        qspi_flash_cmd_ctrl.ADR_BYTE_NUM= 3 - 1;
        qspi_flash_cmd_ctrl.R_BYTE_NUM  = 7;
        qspi_flash_cmd_ctrl.R_DATA_EN   = 1;
        qspi_flash_cmd_ctrl.EXC_CMD     = 1;
        outl(QSPI_FLASH_CMD_CTRL,qspi_flash_cmd_ctrl.reg_value);
        qspi_stig_wait_done();


        *(uint32_t *)read_data = inl(QSPI_FLASH_CMD_RDATA_L);
        *(uint32_t *)(read_data+4) = inl(QSPI_FLASH_CMD_RDATA_H);       
        // wait erase done, sector erase ~= 40ms
        qspi_stig_wait_wip();
        read_data += 8;
    }

}


FLASH_DRIVER_SEG int ms_flash_stig_read(unsigned int address, unsigned int size, unsigned char *buf)
{
    reg_QSPI_FLASH_CMD_CTRL     qspi_flash_cmd_ctrl ;
    reg_QSPI_FLASH_CMD_ADR      qspi_flash_cmd_adr  ;

    unsigned int i = 0;
    uint32_t flash_addr = address - FLASH_BASE_ADDR;
    uint32_t read_addr =  flash_addr;

    uint8_t* read_data = buf;
    
    for(i=0;i<size;i+=1)
    {
        read_addr = flash_addr + i;
        // send enable write
        qspi_stig_single_command(FLASH_WRITE_ENABLE);
        
        // send erase command
        
        qspi_flash_cmd_adr.CMD_ADR      = read_addr;
        outl(QSPI_FLASH_CMD_ADR, qspi_flash_cmd_adr.reg_value);

        
        qspi_flash_cmd_ctrl.reg_value   = 0;
        qspi_flash_cmd_ctrl.CMD_OPCODE  = FLASH_READ_DATA;
        qspi_flash_cmd_ctrl.ADR_EN      = 1;
        qspi_flash_cmd_ctrl.ADR_BYTE_NUM= 3 - 1;
        qspi_flash_cmd_ctrl.R_BYTE_NUM  = 0;
        qspi_flash_cmd_ctrl.R_DATA_EN   = 1;
        qspi_flash_cmd_ctrl.EXC_CMD     = 1;
        outl(QSPI_FLASH_CMD_CTRL,qspi_flash_cmd_ctrl.reg_value);
        qspi_stig_wait_done();


        *read_data = (uint8_t)inl(QSPI_FLASH_CMD_RDATA_L);
      
        // wait erase done, sector erase ~= 40ms
        qspi_stig_wait_wip();
        read_data += 1;
    }

}


FLASH_DRIVER_SEG int ms_flash_alg_read_id(){

    return(0);
}

