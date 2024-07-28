#ifndef MS_SPI_REG__H
#define MS_SPI_REG__H

    #include "global.h"

   // #define SPI0_BASE_ADDR 0x40013000
   // #define SPI1_BASE_ADDR 0x40013400

    #define   SPI_CTRLR0_OFFSET         0x0
    #define   SPI_CTRLR0_RST_VALUE      0x1070000    //SSI_CTRLR0_RST_21
    #define   SPI_CTRLR0_MASK           0x011ffff0
    typedef union {
        struct {
            UINT32 DFS:            4; //`define SSI_MAX_XFER_SIZE 32 R
            UINT32 FRF:            2; //`define SSI_HC_FRF 0 R/W (can't 2'b11),else hold
            UINT32 SCPH:           1; //`define SSI_HC_FRF 0 R/W
            UINT32 SCPOL:          1; //`define SSI_HC_FRF 0 R/W
            UINT32 TMOD:           2;
            UINT32 SLV_OE:         1;
            UINT32 SRL:            1;
            UINT32 CFS:            4;
            UINT32 DFS_32:         5; //`define SSI_MAX_XFER_SIZE 32 R/W must >=3,else hold
            UINT32 SPI_FRF:        2; //`define SSI_SPI_MODE 0 only r
            UINT32 RSVD_CTRLR0_23: 1;
            UINT32 SSTE:           1; //SSI_SCPH0_SSTOGGLE 1 R/W
            UINT32 RSVD_CTRLR0:    7;
        };
        UINT32 reg_value;
    } reg_SPI_CTRLR0;

    #define   SPI_CTRLR1_OFFSET         0x4
    #define   SPI_CTRLR1_RST_VALUE      0x0
    #define   SPI_CTRLR1_MASK           0x0000ffff
    typedef union {
        struct {
            UINT32 NDF:            16;
            UINT32 RSVD_CTRLR1:    16;
        };
        UINT32 reg_value;
    } reg_SPI_CTRLR1;

    #define   SPI_SSIENR_OFFSET         0x8
    #define   SPI_SSIENR_RST_VALUE      0x0
    #define   SPI_SSIENR_MASK           0x00000001
    typedef union {
        struct {
            UINT32 SSI_EN:          1;
            UINT32 RSVD_SSIENR:     31;
        };
        UINT32 reg_value;
    } reg_SPI_SSIENR;

    #define   SPI_MWCR_OFFSET           0xc
    #define   SPI_MWCR_RST_VALUE        0x0
    #define   SPI_MWCR_MASK             0x00000007
    typedef union {
        struct {
            UINT32 MWMOD:           1;
            UINT32 MDD:             1;
            UINT32 MHS:             1;
            UINT32 RSVD_MWCR:       29;
        };
        UINT32 reg_value;
    } reg_SPI_MWCR;

    #define   SPI_SER_OFFSET            0x10
    #define   SPI_SER_RST_VALUE         0x0
    #define   SPI_SER_MASK              0x00000001
    typedef union {
        struct {
            UINT32 SER:             1;
            UINT32 RSVR_SER:        31;
        };
        UINT32 reg_value;
    } reg_SPI_SER;

    #define   SPI_BAUDR_OFFSET          0x14
    #define   SPI_BAUDR_RST_VALUE       0x0
    #define   SPI_BAUDR_MASK            0x0000ffff
    typedef union {
        struct {
            UINT32 SCKDV:           16;
            UINT32 RSVD_BAUDR:      16;
        };
        UINT32 reg_value;
    } reg_SPI_BAUDR;

    #define   SPI_TXFTLR_OFFSET         0x18
    #define   SPI_TXFTLR_RST_VALUE      0x0
    #define   SPI_TXFTLR_MASK           0x0000003f
    typedef union {
        struct {
            UINT32 TFT:             6;
            UINT32 RSVD_TXFTLR:     26;
        };
        UINT32 reg_value;
    } reg_SPI_TXFTLR;

    #define   SPI_RXFTLR_OFFSET         0x1c
    #define   SPI_RXFTLR_RST_VALUE      0x0
    #define   SPI_RXFTLR_MASK           0x0000003f
    typedef union {
        struct {
            UINT32 RFT:             6;
            UINT32 RSVD_RXFTLR:     26;
        };
        UINT32 reg_value;
    } reg_SPI_RXFTLR;

    #define   SPI_TXFLR_OFFSET          0x20
    #define   SPI_TXFLR_RST_VALUE       0x0
    #define   SPI_TXFLR_MASK            0x0000003f
    typedef union {
        struct {
            UINT32 TXTFL:           6;
            UINT32 RSVD_TXFLR:      26;
        };
        UINT32 reg_value;
    } reg_SPI_TXFLR;

    #define   SPI_RXFLR_OFFSET          0x24
    #define   SPI_RXFLR_RST_VALUE       0x0
    #define   SPI_RXFLR_MASK            0x0000003f
    typedef union {
        struct {
            UINT32 RXTFL:           6;
            UINT32 RSVD_RXFLR:      26;
        };
        UINT32 reg_value;
    } reg_SPI_RXFLR;

    #define   SPI_SR_OFFSET             0x28
    #define   SPI_SR_RST_VALUE          0x6   //only read
    #define   SPI_SR_MASK               0x0000007f
    typedef union {
        struct {
            UINT32 BUSY:            1;
            UINT32 TFNF:            1;
            UINT32 TFE:             1;
            UINT32 RFNE:            1;
            UINT32 RFF:             1;
            UINT32 TXE:             1;
            UINT32 DCOL:            1;
            UINT32 RSVD_SR:         25;
        };
        UINT32 reg_value;
    } reg_SPI_SR;

    #define   SPI_IMR_OFFSET            0x2c
    #define   SPI_IMR_RST_VALUE         0x3f
    #define   SPI_IMR_RST_VALUE1        0x1f
    #define   SPI_IMR_MASK              0x0000003f
    #define   SPI_IMR_MASK1             0x0000001f
    typedef union {
        struct {
            UINT32 TXEIM:           1;
            UINT32 TXOIM:           1;
            UINT32 RXUIM:           1;
            UINT32 RXOIM:           1;
            UINT32 RXFIM:           1;
            UINT32 MSTIM:           1;
            UINT32 RSVD_IMR:        26;
        };
        UINT32 reg_value;
    } reg_SPI_IMR;

    #define   SPI_ISR_OFFSET            0x30
    #define   SPI_ISR_RST_VALUE         0x00        //only read
    #define   SPI_ISR_MASK              0x0000003f
    typedef union {
        struct {
            UINT32 TXEIS:           1;
            UINT32 TXOIS:           1;
            UINT32 RXUIS:           1;
            UINT32 RXOIS:           1;
            UINT32 RXFIS:           1;
            UINT32 MSTIS:           1;
            UINT32 RSVD_ISR:        26;
        };
        UINT32 reg_value;
    } reg_SPI_ISR;

    #define   SPI_RISR_OFFSET           0x34
    #define   SPI_RISR_RST_VALUE        0x00      //only read
    #define   SPI_RISR_MASK             0x0000003f
    typedef union {
        struct {
            UINT32 TXEIR:           1;
            UINT32 TXOIR:           1;
            UINT32 RXUIR:           1;
            UINT32 RXOIR:           1;
            UINT32 RXFIR:           1;
            UINT32 MSTIR:           1;
            UINT32 RSVD_RISR:       26;
        };
        UINT32 reg_value;
    } reg_SPI_RISR;

    #define   SPI_TXOICR_OFFSET         0x38
    #define   SPI_TXOICR_RST_VALUE      0x00     //only read
    #define   SPI_TXOICR_MASK           0x00000001
    typedef union {
        struct {
            UINT32 TXOICR:          1;
            UINT32 RSVD_TXOICR:     31;
        };
        UINT32 reg_value;
    } reg_SPI_TXOICR;

    #define   SPI_RXOICR_OFFSET         0x3c
    #define   SPI_RXOICR_RST_VALUE      0x00    //only read
    #define   SPI_RXOICR_MASK           0x00000001
    typedef union {
        struct {
            UINT32 RXOICR:          1;
            UINT32 RSVD_RXOICR:     31;
        };
        UINT32 reg_value;
    } reg_SPI_RXOICR;

    #define   SPI_RXUICR_OFFSET         0x40
    #define   SPI_RXUICR_RST_VALUE      0x00    //only read
    #define   SPI_RXUICR_MASK           0x00000001
    typedef union {
        struct {
            UINT32 RXUICR:          1;
            UINT32 RSVD_RXUICR:     31;
        };
        UINT32 reg_value;
    } reg_SPI_RXUICR;

    #define   SPI_MSTICR_OFFSET         0x44
    #define   SPI_MSTICR_RST_VALUE      0x00   //only read
    #define   SPI_MSTICR_MASK           0x00000001
    typedef union {
        struct {
            UINT32 MSTICR:          1;
            UINT32 RSVD_MSTICR:     31;
        };
        UINT32 reg_value;
    } reg_SPI_MSTICR;

    #define   SPI_ICR_OFFSET            0x48
    #define   SPI_ICR_RST_VALUE         0x00  //only read
    #define   SPI_ICR_MASK              0x00000001
    typedef union {
        struct {
            UINT32 ICR:             1;
            UINT32 RSVD_ICR:        31;
        };
        UINT32 reg_value;
    } reg_SPI_ICR;

    #define   SPI_DMACR_OFFSET          0x4c
    #define   SPI_DMACR_RST_VALUE       0x00
    #define   SPI_DMACR_MASK            0x00000003
    typedef union {
        struct {
            UINT32 RDMAE:           1;
            UINT32 TDMAE:           1;
            UINT32 RSVD_DMACR:      30;
        };
        UINT32 reg_value;
    } reg_SPI_DMACR;

    #define   SPI_DMATDLR_OFFSET        0x50
    #define   SPI_DMATDLR_RST_VALUE     0x00
    #define   SPI_DMATDLR_MASK          0x0000003f
    typedef union {
        struct {
            UINT32 DMATDL:          6;
            UINT32 RSVD_DMATDLR:    26;
        };
        UINT32 reg_value;
    } reg_SPI_DMATDLR;

    #define   SPI_DMARDLR_OFFSET        0x54
    #define   SPI_DMARDLR_RST_VALUE     0x00
    #define   SPI_DMARDLR_MASK          0x0000003f
    typedef union {
        struct {
            UINT32 DMARDL:          6;
            UINT32 RSVD_DMARDLR:    26;
        };
        UINT32 reg_value;
    } reg_SPI_DMARDLR;

    #define   SPI_IDR_OFFSET            0x58
    #define   SPI_IDR_RST_VALUE         0xffffffff  
    #define   SPI_IDR_RST_VALUE1        0xfffffff0  //only read
    #define   SPI_IDR_MASK              0xffffffff
    typedef union {
        struct {
            UINT32 IDCODE:          32;
        };
        UINT32 reg_value;
    } reg_SPI_IDR;

    #define   SPI_VERSION_ID_OFFSET     0x5c
    #define   SPI_VERSION_ID_RST_VALUE  0x3430322a    //only read
    #define   SPI_VERSION_ID_MASK       0xffffffff
    typedef union {
        struct {
            UINT32 COMP_VERSION:    32;
        };
        UINT32 reg_value;
    } reg_SPI_VERSION_ID;

    #define   SPI_DRX_OFFSET            0x60
    #define   SPI_DRX_RST_VALUE         0x00    
    #define   SPI_DRX_MASK              0xffffffff
    typedef union {
        struct {
            UINT32 DR:              16;
            UINT32 RSVD_DR:         16;
        };
        UINT32 reg_value;
    } reg_SPI_DRX;

    #define   SPI_RX_SAMPLE_DLY_OFFSET     0xf0
    #define   SPI_RX_SAMPLE_DLY_RST_VALUE  0x00    
    #define   SPI_RX_SAMPLE_DLY_MASK       0x000000ff
    typedef union {
        struct {
            UINT32 RSD:                  8;
            UINT32 RSVD_RX_SAMPLE_DLY:   24;
        };
        UINT32 reg_value;
    } reg_SPI_RX_SAMPLE_DLY;

    #define   SPI_CTRLR00_OFFSET         0xf4
    #define   SPI_CTRLR00_RST_VALUE      0x200    
    #define   SPI_CTRLR00_MASK           0x0007fb3f
    typedef union {
        struct {
            UINT32 TRANS_TYPE:      2;
            UINT32 ADDR_L:          4;
            UINT32 RSVD_CTRLR00_67: 2;
            UINT32 INST_L:          2;
            UINT32 RSVD_CTRLR00_10: 1;
            UINT32 WAIT_CYCLES:     5;
            UINT32 SPI_DDR_EN:      1;
            UINT32 INST_DDR_EN:     1;
            UINT32 SPI_RXDS_EN:     1;
            UINT32 RSVD_CTRLR00:    13;
        };
        UINT32 reg_value;
    } reg_SPI_SPI_CTRLR00;

    #define   SPI_TXD_DRIVE_EDGE_OFFSET     0xf8
    #define   SPI_TXD_DRIVE_EDGE_RST_VALUE  0x00    
    #define   SPI_TXD_DRIVE_EDGE_MASK       0x000000ff
    typedef union {
        struct {
            UINT32 TDE:                  8;
            UINT32 RSVD_TXD_DRIVE_EDGE:  24;
        };
        UINT32 reg_value;
    } reg_SPI_TXD_DRIVE_EDGE;

    #define   SPI_RSVD_OFFSET            0xfc
    #define   SPI_RSVD_RST_VALUE         0x00      //only read
    #define   SPI_RSVD_MASK              0x00000000
    typedef union {
        struct {
            UINT32 RSVD:            32;
        };
        UINT32 reg_value;
    } reg_SPI_RSVD;


    


#endif
