/**
 ****************************************************************************************
 *
 * @file spi.c
 *
 * @brief SPI driver
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup SPI
 * @{
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "arch.h"
//#include "plf.h"

#if (PLF_SPI)

//#include "plf.h"         // plf functions
#include "spi.h"         // spi functions
//#include "intcntl.h"     // interrupt controller functions
#include "reg_spi.h"     // spi register definitions

/*
 * DEFINES
 ****************************************************************************************
 */

/// Disable mask
#define   INTERRUPT_MASK_ALL_DISABLED    0x00000000

/// Dummy byte
#define   SPI_DUMMY_BYTE        (0x00)

//ms_spi_dev_t rf_spi0;
/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

///SPI module mode: master or slave
enum SPI_MODE
{
    SPI_SLAVE = 0,
    SPI_MASTER = 1
};

///SPI Input clock select
enum SPI_INPUT_CLK_SEL
{
    SPI_INPUT_CLK_SEL_0 = 0,
    SPI_INPUT_CLK_SEL_1 = 1,
    SPI_INPUT_CLK_SEL_2 = 2,
    SPI_INPUT_CLK_SEL_3 = 3
};

///SPI Bus activity
enum SPI_ACTIVITY
{
    SPI_IDLE = 0,
    SPI_ACTIVE = 1
};

///SPI Bus fault detection
enum SPI_MODE_FAULT
{
    SPI_IGNORE = 0,
    SPI_DETECT = 1
};

///SPI SCK polarity
enum SPI_POLARITY
{
    SPI_IDLE_LOW = 0,
    SPI_IDLE_HIGH = 1
};

///SPI SCK phase
enum SPI_PHASE
{
    SPI_1_ST_EDGE = 0,
    SPI_2_ND_EDGE = 1
};

///SPI Bit ordering
enum SPI_BIT_ORDERING
{
    SPI_MSB_FIRST = 0,
    SPI_LSB_FIRST = 1
};

///SPI Bi-directional mode
enum SPI_DIR_MODE
{
    SPI_UNIDIRECTIONAL = 0,
    SPI_BIDIRECTIONAL = 1
};

///SPI Direction
enum SPI_DIRECTION
{
    SPI_INPUT = 0,
    SPI_OUTPUT = 1
};


/*
 * STRUCT DEFINITIONS
 *****************************************************************************************
 */

/// SPI channel parameters, holding data used for asynchronous R/W data transactions
struct spi_txrxchannel
{
    uint32_t  size_wr;
    uint32_t  size_rd;
    uint8_t  *bufptr;
    void     (*callback)(void);
};

///Structure defining SPI environment parameters
struct spi_env_tag
{
    struct spi_txrxchannel tx;
    struct spi_txrxchannel rx;
};


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
///SPI environment
static struct spi_env_tag spi_env;



/*
 * LOCAL FUNCTION DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Receives data from SPI RX FIFO.
 ****************************************************************************************
 */
static void spi_receive_data(void);

/**
 ****************************************************************************************
 * @brief Transmit data to SPI TX FIFO.
 ****************************************************************************************
 */
static void spi_transmit_data(void);



/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

static void spi_receive_data(void)
{
    // Read RX FIFO
    while (spi_spi_status_rx_fifo_not_empty_getf())
    {
        // Read the received data in the FIFO
        *spi_env.rx.bufptr = spi_spi_rx_data_rx_data_getf();

        // Decrement size
        spi_env.rx.size_rd--;

        // Increment buffer
        spi_env.rx.bufptr++;
    }

    // Check if all expected data have been received
    if (spi_env.rx.size_rd > 0)
    {
        // Send dummy bytes to receive data
        while ((spi_env.rx.size_wr > 0) && (!spi_spi_status_rx_fifo_half_full_getf()) && spi_spi_status_tx_fifo_half_empty_getf())
        {
            // Fill dummy byte
            spi_spi_tx_data_tx_data_setf(SPI_DUMMY_BYTE);

            // Decrement size
            spi_env.rx.size_wr--;
        }
   }
    else
    {
        void (*callback)(void) = spi_env.rx.callback;

        // Reset RX parameters
        spi_env.rx.bufptr = NULL;
        spi_env.rx.callback = NULL;

        // Call end of reception callback
        if(callback != NULL)
        {
            callback();
        }
    }
}

static void spi_transmit_data(void)
{
    // Flush RX FIFO
    while (spi_spi_status_rx_fifo_not_empty_getf())
    {
        // Flush the received data in the FIFO
        spi_spi_rx_data_rx_data_getf();

        // Decrement size
        spi_env.tx.size_rd--;
    }

    // Check if all expected data have been sent
    if (spi_env.tx.size_rd > 0)
    {
        // Fill TX data into FIFO
        while ((spi_env.tx.size_wr > 0) && (!spi_spi_status_rx_fifo_half_full_getf()) && spi_spi_status_tx_fifo_half_empty_getf())
        {
            // Write the TX data into the FIFO
            spi_spi_tx_data_tx_data_setf(*spi_env.tx.bufptr);

            // Decrement size
            spi_env.tx.size_wr--;

            // Increment buffer
            spi_env.tx.bufptr++;
        }
    }
    else
    {
        void (*callback)(void) = spi_env.tx.callback;

        // Reset TX parameters
        spi_env.tx.bufptr = NULL;
        spi_env.tx.callback = NULL;

        // Call end of transmission callback
        if(callback != NULL)
        {
            callback();
        }
    }
}


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void spi_init(void)
{
    //uint32_t clock;

    /*
     * Set SPI default baudrate
     */
    //spi_spi_clk_div_set(plf_spi_divider());
   
 // Enable SPI
    spi_spi_config_spi_enable_setf(SPI_ENABLED);

    // Mask all interrupts in SPI component
    spi_spi_mask_set(INTERRUPT_MASK_ALL_DISABLED);

    #ifndef CFG_ROM
    // Enable SPI interrupt
//    intcntl_enable_irq(INTCNTL_SPI_RX);
    #endif //CFG_ROM

    //Configure SPI environment
    spi_env.rx.bufptr = NULL;
    spi_env.rx.size_rd = 0;
    spi_env.rx.size_wr = 0;
    spi_env.rx.callback = NULL;
    spi_env.tx.bufptr = NULL;
    spi_env.tx.size_rd = 0;
    spi_env.tx.size_wr = 0;
    spi_env.tx.callback = NULL;

    // Flush RX FIFO
    while (spi_spi_status_rx_fifo_not_empty_getf())
    {
        // Flush the received data in the FIFO
        spi_spi_rx_data_rx_data_getf();
    }
    // Enable RX FIFO interrupt to detect incoming bytes
    spi_spi_mask_rx_fifo_not_empty_mask_setf(SPI_ENABLED);
}

void spi_read(uint8_t *bufptr, uint32_t size, void (*rx_callback)(void))
{
    //Store environment parameters
    spi_env.rx.bufptr = bufptr;
    spi_env.rx.size_wr = size;
    spi_env.rx.size_rd = size;
    spi_env.rx.callback = rx_callback;

    // Start data reception
    spi_receive_data();
}

void spi_write(uint8_t *bufptr, uint32_t size, void (*tx_callback)(void))
{
    //Store environment parameters
    spi_env.tx.bufptr = bufptr;
    spi_env.tx.size_wr = size;
    spi_env.tx.size_rd = size;
    spi_env.tx.callback = tx_callback;

    // Start data transmission
    spi_transmit_data();
}

void spi_interrupt_mode(uint8_t mode)
{
    if(mode)
    {
        // Enable SPI interrupt
       // intcntl_enable_irq(INTCNTL_SPI_RX);
    }
    else
    {
        // Disable SPI interrupt
      //  intcntl_disable_irq(INTCNTL_SPI_RX);
    }
}

void spi_poll(void)
{
    // RX or TX
    if(spi_env.rx.size_rd > 0)
    {
        spi_receive_data();
    }
    else if(spi_env.tx.size_rd > 0)
    {
        spi_transmit_data();
    }
}

void spi_isr(void)
{
    // RX or TX
    if(spi_env.rx.size_rd > 0)
    {
        spi_receive_data();
    }
    else if(spi_env.tx.size_rd > 0)
    {
        spi_transmit_data();
    }
}

#endif //PLF_SPI

/// @} SPI

