#ifndef QSPI_REG__H
#define QSPI_REG__H

    #include "global.h"

    #define   QSPI_CFG                  ( QSPI_BASE_ADDR + QSPI_CFG_OFFSET                  )
    #define   QSPI_DEV_RDINSTR_CFG      ( QSPI_BASE_ADDR + QSPI_DEV_RDINSTR_CFG_OFFSET      )
    #define   QSPI_DEV_WRINSTR_CFG      ( QSPI_BASE_ADDR + QSPI_DEV_WRINSTR_CFG_OFFSET      )
    #define   QSPI_DEV_DELAY            ( QSPI_BASE_ADDR + QSPI_DEV_DELAY_OFFSET            )
    #define   QSPI_RD_CAPTURE           ( QSPI_BASE_ADDR + QSPI_RD_CAPTURE_OFFSET           )
    #define   QSPI_DEV_SIZE             ( QSPI_BASE_ADDR + QSPI_DEV_SIZE_OFFSET             )
    #define   QSPI_SRAM_PARTITION       ( QSPI_BASE_ADDR + QSPI_SRAM_PARTITION_OFFSET       )
    #define   QSPI_IND_AHB_TRIGGER      ( QSPI_BASE_ADDR + QSPI_IND_AHB_TRIGGER_OFFSET      )
    #define   QSPI_DMA_PERIPH_CFG       ( QSPI_BASE_ADDR + QSPI_DMA_PERIPH_CFG_OFFSET       )
    #define   QSPI_REMAP_ADR            ( QSPI_BASE_ADDR + QSPI_REMAP_ADR_OFFSET            )
    #define   QSPI_MODE_BIT             ( QSPI_BASE_ADDR + QSPI_MODE_BIT_OFFSET             )
    #define   QSPI_SRAM_FILL_LEVEL      ( QSPI_BASE_ADDR + QSPI_SRAM_FILL_LEVEL_OFFSET      )
    #define   QSPI_TX_THRESH            ( QSPI_BASE_ADDR + QSPI_TX_THRESH_OFFSET            )
    #define   QSPI_RX_THRESH            ( QSPI_BASE_ADDR + QSPI_RX_THRESH_OFFSET            )
    #define   QSPI_WRITE_COMP_CTRL      ( QSPI_BASE_ADDR + QSPI_WRITE_COMP_CTRL_OFFSET      )
    #define   QSPI_MAX_NUM_POLLS        ( QSPI_BASE_ADDR + QSPI_MAX_NUM_POLLS_OFFSET        )
    #define   QSPI_INT_STATUS           ( QSPI_BASE_ADDR + QSPI_INT_STATUS_OFFSET           )
    #define   QSPI_INT_MASK             ( QSPI_BASE_ADDR + QSPI_INT_MASK_OFFSET             )
    #define   QSPI_WP_L                 ( QSPI_BASE_ADDR + QSPI_WP_L_OFFSET                 )
    #define   QSPI_WP_H                 ( QSPI_BASE_ADDR + QSPI_WP_H_OFFSET                 )
    #define   QSPI_WP_CTRL              ( QSPI_BASE_ADDR + QSPI_WP_CTRL_OFFSET              )
    #define   QSPI_IND_RD_CTRL          ( QSPI_BASE_ADDR + QSPI_IND_RD_CTRL_OFFSET          )
    #define   QSPI_IND_RD_WATERMARK     ( QSPI_BASE_ADDR + QSPI_IND_RD_WATERMARK_OFFSET     )
    #define   QSPI_IND_RD_ADR           ( QSPI_BASE_ADDR + QSPI_IND_RD_ADR_OFFSET           )
    #define   QSPI_IND_RD_BYTE          ( QSPI_BASE_ADDR + QSPI_IND_RD_BYTE_OFFSET          )
    #define   QSPI_IND_WR_CTRL          ( QSPI_BASE_ADDR + QSPI_IND_WR_CTRL_OFFSET          )
    #define   QSPI_IND_WR_WATERMARK     ( QSPI_BASE_ADDR + QSPI_IND_WR_WATERMARK_OFFSET     )
    #define   QSPI_IND_WR_ADR           ( QSPI_BASE_ADDR + QSPI_IND_WR_ADR_OFFSET           )
    #define   QSPI_IND_WR_BYTE          ( QSPI_BASE_ADDR + QSPI_IND_WR_BYTE_OFFSET          )
    #define   QSPI_IND_RANGE_WIDTH      ( QSPI_BASE_ADDR + QSPI_IND_RANGE_WIDTH_OFFSET      )
    #define   QSPI_FLASH_CMD_MEM        ( QSPI_BASE_ADDR + QSPI_FLASH_CMD_MEM_OFFSET        )
    #define   QSPI_FLASH_CMD_CTRL       ( QSPI_BASE_ADDR + QSPI_FLASH_CMD_CTRL_OFFSET       )
    #define   QSPI_FLASH_CMD_ADR        ( QSPI_BASE_ADDR + QSPI_FLASH_CMD_ADR_OFFSET        )
    #define   QSPI_FLASH_CMD_RDATA_L    ( QSPI_BASE_ADDR + QSPI_FLASH_CMD_RDATA_L_OFFSET    )
    #define   QSPI_FLASH_CMD_RDATA_H    ( QSPI_BASE_ADDR + QSPI_FLASH_CMD_RDATA_H_OFFSET    )
    #define   QSPI_FLASH_CMD_WDATA_L    ( QSPI_BASE_ADDR + QSPI_FLASH_CMD_WDATA_L_OFFSET    )
    #define   QSPI_FLASH_CMD_WDATA_H    ( QSPI_BASE_ADDR + QSPI_FLASH_CMD_WDATA_H_OFFSET    )
    #define   QSPI_FLASH_STATUS         ( QSPI_BASE_ADDR + QSPI_FLASH_STATUS_OFFSET         )
    #define   QSPI_MODULE_ID            ( QSPI_BASE_ADDR + QSPI_MODULE_ID_OFFSET            )

    #define   QSPI_CFG_OFFSET            0x0
    #define   QSPI_CFG_RESET_VALUE       0x80780081
    #define   QSPI_CFG_MASK_W            0x01ffff87
    #define   QSPI_CFG_MASK              0x81ffff87
    typedef union {
        struct {
            UINT32 QSPI_EN:                1;  
            UINT32 QSPI_CPOL:              1;  
            UINT32 QSPI_CPHA:              1;  
            UINT32 reserved_0:             4;  //RO
            UINT32 DIRAC_EN:               1;  
            UINT32 Legacy_EN:              1;  
            UINT32 PER_SEL_DEC:            1;  
            UINT32 PER_SEL:                4;  
            UINT32 WP_EN:                  1;  
            UINT32 DMA_EN:                 1;  
            UINT32 AHB_ADR_REMAP_EN:       1;  
            UINT32 XIP_NEXT:               1;  
            UINT32 XIP_NOW:                1;  
            UINT32 BAUD_RATE_CFG:          4;  
            UINT32 AHB_DEC_EN:             1;  
            UINT32 DTR_EN:                 1;  //pose and neg sampling data
            UINT32 reserved_1:             6;  //RO
            UINT32 RE_TIME_ST:             1;  //RO
        };
        UINT32 reg_value;
    } reg_QSPI_CFG;

    #define   QSPI_DEV_RDINSTR_CFG_OFFSET           0x4
    #define   QSPI_DEV_RDINSTR_CFG_RESET_VALUE      0x00000003
    #define   QSPI_DEV_RDINSTR_CFG_MASK             0x1f1337ff
    typedef union {
        struct {
            UINT32 R_OPCODE:                8; 
            UINT32 INSTR_TRAN_TYPE:         2; 
            UINT32 DDR_EN:                  1; 
            UINT32 reserved_0:              1; //ro
            UINT32 ADR_TRAN_TYPE:           2; 
            UINT32 reserved_1:              2; //ro
            UINT32 DATA_TRAN_TYPE:          2; 
            UINT32 reserved_2:              2; //ro
            UINT32 MODE_BIT_EN:             1; 
            UINT32 reserved_3:              3; //ro
            UINT32 DUMMY_CLOCK_NUM:         5; 
            UINT32 reserved_4:              3; //ro
        };
        UINT32 reg_value;
    } reg_QSPI_DEV_RDINSTR_CFG;
    
    #define   QSPI_DEV_WRINSTR_CFG_OFFSET          0x8
    #define   QSPI_DEV_WRINSTR_CFG_RESET_VALUE     0x00000002
    #define   QSPI_DEV_WRINSTR_CFG_MASK            0x1f033eff
    typedef union {
        struct {
            UINT32 W_OPCODE:               8; 
            UINT32 WEL_DISABLE:            1; 
            UINT32 reserved_0:             3; //ro
            UINT32 ADR_TRAN_TYPE:          2; 
            UINT32 reserved_1:             2; //ro
            UINT32 DATA_TRAN_TYPE:         2; 
            UINT32 reserved_2:             6; //ro
            UINT32 DUMMY_CLOCK_NUM:        5; 
            UINT32 reserved_3:             3; //ro
        };
        UINT32 reg_value;
    } reg_QSPI_DEV_WRINSTR_CFG;

    #define   QSPI_DEV_DELAY_OFFSET          0xc 
    #define   QSPI_DEV_DELAY_RESET_VALUE     0x00000000
    #define   QSPI_DEV_DELAY_MASK            0xffffffff

    typedef union {
        struct {
            UINT32 CSSOT:                   8; //
            UINT32 CSEOT:                   8; //
            UINT32 CSDADS:                  8; //
            UINT32 CSDA:                    8; //
        };
        UINT32 reg_value;
    } reg_QSPI_DEV_DELAY;

    #define   QSPI_RD_CAPTURE_OFFSET           0x10
    #define   QSPI_RD_CAPTURE_RESET_VALUE      0x00000001
    #define   QSPI_RD_CAPTURE_MASK             0x000f003f
    typedef union {
        struct {
            UINT32 ADAPT_LOOPBACK:          1;  
            UINT32 DELAY_NUM:               4;  
            UINT32 SAMP_EDGE:               1;  
            UINT32 reserved_0:              10; //ro
            UINT32 DELAY_TRAN:              4;  
            UINT32 reserved_1:              12; //ro
        };
        UINT32 reg_value;
    } reg_QSPI_RD_CAPTURE;
   
    #define   QSPI_DEV_SIZE_OFFSET           0x14
    #define   QSPI_DEV_SIZE_RESET_VALUE      0x00001002
    #define   QSPI_DEV_SIZE_MASK             0x1fffffff
    typedef union {
        struct {
            UINT32 ADR_B_NUM:               4;  
            UINT32 PAGE_NUM:                12;  
            UINT32 BLOCK_NUM:               5;  
            UINT32 CS0_SIZE:                2;  
            UINT32 CS1_SIZE:                2;  
            UINT32 CS2_SIZE:                2;  
            UINT32 CS3_SIZE:                2;  
            UINT32 reserved_0:              3; //ro
        };
        UINT32 reg_value;
    } reg_QSPI_DEV_SIZE;

    #define   QSPI_SRAM_PARTITION_OFFSET           0x18
    #define   QSPI_SRAM_PARTITION_RESET_VALUE      0x00000080
    #define   QSPI_SRAM_PARTITION_MASK             0x000000ff
    typedef union {
        struct {
            UINT32 SRAM_PARTITION:          8;  
            UINT32 reserved_0:              24; //ro
        };
        UINT32 reg_value;
    } reg_QSPI_SRAM_PARTITION;
    
    #define   QSPI_IND_AHB_TRIGGER_OFFSET           0x1c
    #define   QSPI_IND_AHB_TRIGGER_RESET_VALUE      0x00000000
    #define   QSPI_IND_AHB_TRIGGER_MASK             0xffffffff
    typedef union {
        struct {
            UINT32 IND_AHB_ADR_TRIGGER:     32;  
        };
        UINT32 reg_value;
    } reg_QSPI_IND_AHB_TRIGGER;
 
    #define   QSPI_DMA_PERIPH_CFG_OFFSET           0x20
    #define   QSPI_DMA_PERIPH_CFG_RESET_VALUE      0x00000000
    #define   QSPI_DMA_PERIPH_CFG_MASK             0x00000fff
    typedef union {
        struct {
            UINT32 SING_TYPE_REQ:           4; 
            UINT32 reserved_0:              4; //(R)
            UINT32 BURST_TYPE_REQ:          4; 
            UINT32 reserved_1:              20; //(R)
        };
        UINT32 reg_value;
    } reg_QSPI_DMA_PERIPH_CFG;
   
    #define   QSPI_REMAP_ADR_OFFSET           0x24
    #define   QSPI_REMAP_ADR_RESET_VALUE      0x00000002
    #define   QSPI_REMAP_ADR_MASK             0x00000007
    typedef union {
        struct {
            UINT32 REMAP_ADR:               32;
        };
        UINT32 reg_value;
    } reg_QSPI_REMAP_ADR;
   
    #define   QSPI_MODE_BIT_OFFSET           0x28
    #define   QSPI_MODE_BIT_RESET_VALUE      0x00000000
    #define   QSPI_MODE_BIT_MASK             0x000000ff
    typedef union {
        struct {
            UINT32 MODE_BIT:                8;
            UINT32 reserved_0:              24; //(R)
        };
        UINT32 reg_value;
    } reg_QSPI_MODE_BIT;
    
    #define   QSPI_SRAM_FILL_LEVEL_OFFSET           0x2c
    #define   QSPI_SRAM_FILL_LEVEL_RESET_VALUE      0x00000000
    #define   QSPI_SRAM_FILL_LEVEL_MASK             0x0000ffff
    typedef union {
        struct {
            UINT32 SRAM_RFILL_LEVEL:        16; //R
            UINT32 SRAM_WFILL_LEVEL:        16; //R
        };
        UINT32 reg_value;
    } reg_QSPI_SRAM_FILL_LEVEL;
   
    #define   QSPI_TX_THRESH_OFFSET           0x30
    #define   QSPI_TX_THRESH_RESET_VALUE      0x00000001
    #define   QSPI_TX_THRESH_MASK             0x0000001f
    typedef union {
        struct {
            UINT32 TX_THRESH:               5; 
            UINT32 reserved_0:              27; //(R)
        };
        UINT32 reg_value;
    } reg_QSPI_TX_THRESH;
  
    #define   QSPI_RX_THRESH_OFFSET           0x34
    #define   QSPI_RX_THRESH_RESET_VALUE      0x00000001
    #define   QSPI_RX_THRESH_MASK             0x0000001f
    typedef union {
        struct {
            UINT32 RX_THRESH:               5; 
            UINT32 reserved_0:              27; //(R)
        };
        UINT32 reg_value;
    } reg_QSPI_RX_THRESH;

    #define   QSPI_WRITE_COMP_CTRL_OFFSET           0x38
    #define   QSPI_WRITE_COMP_CTRL_RESET_VALUE      0x00010005
    #define   QSPI_WRITE_COMP_CTRL_MASK             0xffffe7ff
    typedef union {
        struct {
            UINT32 OPCODE:                  8;  
            UINT32 POLLING_BIT_INDEX:       3;  
            UINT32 reserved_0:              2; //ro
            UINT32 POLLING_POLAR:           1;  
            UINT32 POLLING_DISABLE:         1;  
            UINT32 POLLING_EN_END:          1;  
            UINT32 POLLING_REP_DELAY:       8;  
        };
        UINT32 reg_value;
    } reg_QSPI_WRITE_COMP_CTRL;
   
    #define   QSPI_MAX_NUM_POLLS_OFFSET           0x3c
    #define   QSPI_MAX_NUM_POLLS_RESET_VALUE      0xffffffff
    #define   QSPI_MAX_NUM_POLLS_MASK             0xffffffff
    typedef union {
        struct {
            UINT32 MAX_NUM_POLLS:           32; 
        };
        UINT32 reg_value;
    } reg_QSPI_MAX_NUM_POLLS;

    #define   QSPI_INT_STATUS_OFFSET           0x40
    #define   QSPI_INT_STATUS_RESET_VALUE      0x00000000
    #define   QSPI_INT_STATUS_MASK             0x000007ff
    typedef union {
        struct {
            UINT32 MODE_FAIL:               1;  //rwc
            UINT32 Underflow_detected:      1;  
            UINT32 last_ind_completed:      1; //ro
            UINT32 ind_noaccept:            1;  
            UINT32 WP_ATTEMPT:              1;  
            UINT32 ILLEGAL_AHB_ACCESS:      1;  
            UINT32 IND_WATERMARK:           1;  
            UINT32 Receive_overflow:        1;  
            UINT32 TX_FIFO_NOT_FULL:        1;  
            UINT32 TX_FIFO_FULL:            1;  
            UINT32 RX_FIFO_NOT_EMPTY:       1;  
            UINT32 RX_FIFO_FULL:            1;  
            UINT32 IND_READ:                1;  
            UINT32 MAXI_POLLS_FULL:         1;  
            UINT32 STIG_REQUEST_INT:        1;  
            UINT32 reserved_0:              17; //ro
        };
        UINT32 reg_value;
    } reg_QSPI_INT_STATUS;
    
    #define   QSPI_INT_MASK_OFFSET           0x44
    #define   QSPI_INT_MASK_RESET_VALUE      0x00000000
    #define   QSPI_INT_MASK_MASK             0x00007fff
    typedef union {
        struct {
            UINT32 INT_MASK:        15; 
            UINT32 reserved_0:      17; //ro
        };
        UINT32 reg_value;
    } reg_QSPI_INT_MASK;
    
    #define   QSPI_WP_L_OFFSET           0x50
    #define   QSPI_WP_L_RESET_VALUE      0x00000000
    #define   QSPI_WP_L_MASK             0xffffffff
    typedef union {
        struct {
            UINT32 WP_L:            32; 
        };
        UINT32 reg_value;
    } reg_QSPI_WP_L;
    
    #define   QSPI_WP_H_OFFSET           0x54
    #define   QSPI_WP_H_RESET_VALUE      0x00000000
    #define   QSPI_WP_H_MASK             0xffffffff
    typedef union {
        struct {
            UINT32 WP_H:            32; 
        };
        UINT32 reg_value;
    } reg_QSPI_WP_H;
    
    #define   QSPI_WP_CTRL_OFFSET        0x58
    #define   QSPI_WP_CTRL_RESET_VALUE   0x00000000
    #define   QSPI_WP_CTRL_MASK          0x00000003
    typedef union {
        struct {
            UINT32 WP_INV:          1;
            UINT32 WP_EN:           1; 
            UINT32 reserved_0:      30; 
        };
        UINT32 reg_value;
    } reg_QSPI_WP_CTRL;

    
    #define   QSPI_IND_RD_CTRL_OFFSET        0x60
    #define   QSPI_IND_RD_CTRL_RESET_VALUE   0x00000000
    #define   QSPI_IND_RD_CTRL_MASK_W        0x0000005b
    #define   QSPI_IND_RD_CTRL_MASK          0x000000ff
    typedef union {
        struct {
            UINT32 IND_READ_START:          1; //only W
            UINT32 IND_READ_CANCEL:         1; //only W
            UINT32 IND_READ_STATUS:         1; //only R 
            UINT32 SRAM_FULL:               1; 
            UINT32 IND_READ_TWO:            1; //only R
            UINT32 IND_COMPLETION_STATUS:   1; 
            UINT32 IND_COMPLETION_NUM:      1; //only R 
            UINT32 reserved_0:              24; 
        };
        UINT32 reg_value;
    } reg_QSPI_IND_RD_CTRL;
    
    #define   QSPI_IND_RD_WATERMARK_OFFSET        0x64
    #define   QSPI_IND_RD_WATERMARK_RESET_VALUE   0x00000000
    #define   QSPI_IND_RD_WATERMARK_MASK          0xffffffff
    typedef union {
        struct {
            UINT32 IND_WATERMARK:           32; 
        };
        UINT32 reg_value;
    } reg_QSPI_IND_RD_WATERMARK;
    
    #define   QSPI_IND_RD_ADR_OFFSET        0x68
    #define   QSPI_IND_RD_ADR_RESET_VALUE   0x00000000
    #define   QSPI_IND_RD_ADR_MASK          0xffffffff
    typedef union {
        struct {
            UINT32 IND_RD_ADR:              32; 
        };
        UINT32 reg_value;
    } reg_QSPI_IND_RD_ADR;
    
    #define   QSPI_IND_RD_BYTE_OFFSET       0x6c
    #define   QSPI_IND_RD_BYTE_RESET_VALUE  0x00000000
    #define   QSPI_IND_RD_BYTE_MASK         0xffffffff
    typedef union {
        struct {
            UINT32 IND_RD_BYTE:             32; 
        };
        UINT32 reg_value;
    } reg_QSPI_IND_RD_BYTE;
    
    #define   QSPI_IND_WR_CTRL_OFFSET        0x70
    #define   QSPI_IND_WR_CTRL_RESET_VALUE   0x00000000
    #define   QSPI_IND_WR_CTRL_MASK_W        0x0000005b
    #define   QSPI_IND_WR_CTRL_MASK          0x000000ff
    typedef union {
        struct {
            UINT32 IND_WRITE_START:         1; //only W
            UINT32 IND_WRITE_CANCEL:        1; //only W
            UINT32 IND_WRITE_STATUS:        1; //only R 
            UINT32 reserved_0:              1; 
            UINT32 IND_WRITE_TWO:           1; //only R
            UINT32 IND_COMPLETION_STATUS:   1; 
            UINT32 IND_COMPLETION_NUM:      1; //only R 
            UINT32 reserved_1:              24; 
        };
        UINT32 reg_value;
    } reg_QSPI_IND_WR_CTRL;

    #define   QSPI_IND_WR_WATERMARK_OFFSET        0x74
    #define   QSPI_IND_WR_WATERMARK_RESET_VALUE   0x00000000
    #define   QSPI_IND_WR_WATERMARK_MASK          0xffffffff
    typedef union {
        struct {
            UINT32 IND_WATERMARK:       32; 
        };
        UINT32 reg_value;
    } reg_IND_WR_WATERMARK;
    
    #define   QSPI_IND_WR_ADR_OFFSET        0x78
    #define   QSPI_IND_WR_ADR_RESET_VALUE   0x00000000
    #define   QSPI_IND_WR_ADR_MASK          0xffffffff
    typedef union {
        struct {
            UINT32 IND_WR_ADR:          32; 
        };
        UINT32 reg_value;
    } reg_QSPI_IND_WR_ADR;
    
    #define   QSPI_IND_WR_BYTE_OFFSET       0x7c
    #define   QSPI_IND_WR_BYTE_RESET_VALUE  0x00000000
    #define   QSPI_IND_WR_BYTE_MASK         0xffffffff
    typedef union {
        struct {
            UINT32 IND_WR_BYTE:         32; 
        };
        UINT32 reg_value;
    } reg_QSPI_IND_WR_BYTE;


    #define   QSPI_IND_RANGE_WIDTH_OFFSET        0x80
    #define   QSPI_IND_RANGE_WIDTH_RESET_VALUE   0x00000004
    #define   QSPI_IND_RANGE_WIDTH_MASK          0x0000000f
    typedef union {
        struct {
            UINT32 IND_RANGE_WIDTH:     4; 
            UINT32 reserved_0:          28; 
        };
        UINT32 reg_value;
    } reg_QSPI_IND_RANGE_WIDTH;


    #define   QSPI_FLASH_CMD_MEM_OFFSET        0x8c
    #define   QSPI_FLASH_CMD_MEM_RESET_VALUE   0x00000000
    #define   QSPI_FLASH_CMD_MEM_MASK_W        0x1ffb0001
    #define   QSPI_FLASH_CMD_MEM_MASK          0x1ffbff03
    typedef union {
        struct {
            UINT32 MEM_request_T:       1; //only W
            UINT32 MEM_request_P:       1; //only R 
            UINT32 reserved_0:          6; 
            UINT32 MEM_RDATA:           8; //only R
            UINT32 MEM_RBYTES:          3;  
            UINT32 reserved_1:          1; 
            UINT32 MEM_ADR:             9;  
            UINT32 reserved_2:          3; 
        };
        UINT32 reg_value;
    } reg_QSPI_FLASH_CMD_MEM;
    
    #define   QSPI_FLASH_CMD_CTRL_OFFSET        0x90
    #define   QSPI_FLASH_CMD_CTRL_RESET_VALUE   0x00000000
    #define   QSPI_FLASH_CMD_CTRL_MASK_W        0xffffff0d
    #define   QSPI_FLASH_CMD_CTRL_MASK          0xffffff0f
    typedef union {
        struct {
            UINT32 EXC_CMD:             1; //only W
            UINT32 STIG_EXC_P:          1; //only R 
            UINT32 STIG_MEM_EN:         1; 
            UINT32 reserved_0:          4; 
            UINT32 DUMMY_NUM:           5; 
            UINT32 W_BYTE_NUM:          3;  
            UINT32 W_DATA_EN:           1;  
            UINT32 ADR_BYTE_NUM:        2; 
            UINT32 MODE_BIT_EN:         1;  
            UINT32 ADR_EN:              1;  
            UINT32 R_BYTE_NUM:          3; 
            UINT32 R_DATA_EN:           1;  
            UINT32 CMD_OPCODE:          8; 
        };
        UINT32 reg_value;
    } reg_QSPI_FLASH_CMD_CTRL;
    
    #define   QSPI_FLASH_CMD_ADR_OFFSET        0x94
    #define   QSPI_FLASH_CMD_ADR_RESET_VALUE   0x00000000
    #define   QSPI_FLASH_CMD_ADR_MASK          0xffffffff
    typedef union {
        struct {
            UINT32 CMD_ADR:             32; 
        };
        UINT32 reg_value;
    } reg_QSPI_FLASH_CMD_ADR;
   
    #define   QSPI_FLASH_CMD_RDATA_L_OFFSET        0xA0   //only R
    #define   QSPI_FLASH_CMD_RDATA_L_RESET_VALUE   0x00000000
    #define   QSPI_FLASH_CMD_RDATA_L_MASK          0xffffffff
    typedef union {
        struct {
            UINT32 CMD_RDATA_L:         32; 
        };
        UINT32 reg_value;
    } reg_QSPI_FLASH_CMD_RDATA_L;

    #define   QSPI_FLASH_CMD_RDATA_H_OFFSET        0xA4   //only R
    #define   QSPI_FLASH_CMD_RDATA_H_RESET_VALUE   0x00000000
    #define   QSPI_FLASH_CMD_RDATA_H_MASK          0xffffffff
    typedef union {
        struct {
            UINT32 CMD_RDATA_H:         32; 
        };
        UINT32 reg_value;
    } reg_QSPI_FLASH_CMD_RDATA_H;
    
    #define   QSPI_FLASH_CMD_WDATA_L_OFFSET        0xA8   
    #define   QSPI_FLASH_CMD_WDATA_L_RESET_VALUE   0x00000000
    #define   QSPI_FLASH_CMD_WDATA_L_MASK          0xffffffff
    typedef union {
        struct {
            UINT32 CMD_WDATA_L:         32; 
        };
        UINT32 reg_value;
    } reg_QSPI_FLASH_CMD_WDATA_L;

    #define   QSPI_FLASH_CMD_WDATA_H_OFFSET        0xAC   
    #define   QSPI_FLASH_CMD_WDATA_H_RESET_VALUE   0x00000000
    #define   QSPI_FLASH_CMD_WDATA_H_MASK          0xffffffff
    typedef union {
        struct {
            UINT32 CMD_WDATA_H:         32; 
        };
        UINT32 reg_value;
    } reg_QSPI_FLASH_CMD_WDATA_H;


    #define   QSPI_FLASH_STATUS_OFFSET        0xb0
    #define   QSPI_FLASH_STATUS_RESET_VALUE   0x00000000 
    #define   QSPI_FLASH_STATUS_MASK_W        0x000f0000
    #define   QSPI_FLASH_STATUS_MASK          0x000f01ff
    typedef union {
        struct {
            UINT32 FLASH_STATUS:         8;  //only R
            UINT32 POLL_STATUS:          1;  //only R
            UINT32 reserved_0:           7; 
            UINT32 DUM_NUM:              4;  
            UINT32 reserved_1:           12; 
        };
        UINT32 reg_value;
    } reg_QSPI_FLASH_STATUS;
    
    #define   QSPI_MODULE_ID_OFFSET        0xfc  //only R
    #define   QSPI_MODULE_ID_RESET_VALUE   0x01000103 
    #define   QSPI_MODULE_ID_MASK          0xffffffff
    typedef union {
        struct {
            UINT32 CFG_ID:               2;  
            UINT32 reserved_0:           6; 
            UINT32 REVISION_ID:          16;  
            UINT32 FIX_NUM_ID:           8; 
        };
        UINT32 reg_value;
    } reg_QSPI_MODULE_ID;
     

#endif
