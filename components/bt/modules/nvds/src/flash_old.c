/**
 ****************************************************************************************
 *
 * @file flash.c
 *
 * @brief Definition of the Flash re-mapping API.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup FLASH
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <string.h>

#include "flash_old.h"

//#include "co_math.h"

#include "arch.h"

#include "gpio.h"
#include "ms_flash_alg.h"

#if PLF_SPI
#include "spi.h"       // spi functions
#endif //PLF_SPI

#ifndef CFG_ROM
#include "dbg.h"
#endif //CFG_ROM
#include "atiny_log.h"
#include "co_math.h"     // math operations
#include "co_error.h"

/*
 * DEFINES
 ****************************************************************************************
 */

///Flash Write protect pin : GPIO 9
#define FLASH_WVPP_PIN                (GPIO_PIN_9)
///Flash chip select pin: GPIO 8
#define FLASH_SPI_CS_PIN              (GPIO_PIN_8)

///Flash manufacturer ID: Numonyx
#define FLASH_MAN_ID_M25P128          0x20
///Flash memory type
#define FLASH_MEM_TYPE_M25P128        0x20
///Flash memory capacity
#define FLASH_MEM_CAPA_M25P128        0x18

///Flash manufacturer ID: GigaDevice
#define FLASH_MAN_ID_GD25LQ80C          0xc8
///Flash memory type
#define FLASH_MEM_TYPE_GD25LQ80C        0x65
///Flash memory capacity
#define FLASH_MEM_CAPA_GD25LQ80C        0x13

///Flash memory size: 128 MBytes
//#define FLASH_MEMORY_SIZE             0x08000000

///Flash memory sector size; 256 KBytes
//#define FLASH_SECT_SIZE               0x40000

///Flash memory size: 512 KBytes
#define FLASH_MEMORY_SIZE             0x20061000 //0x20041000

///Flash memory sector size; 4 KBytes
#define FLASH_SECT_SIZE               0x01000

///Flash memory page size: 256 Bytes
#define FLASH_PAGE_SIZE               0x100

///Flash sector Mask
#define FLASH_SECT_MASK               0xFFFF0000
///Flash page mask
#define FLASH_PAGE_MASK               0xFFFFFF00


//Flash instruction set
///Write Enable:                    0000 0110 06h 0 0 0
#define FLASH_WREN      0x06
/// Write Disable :                 0000 0100 04h 0 0 0
#define FLASH_WRDI      0x04
///Read Identification :            1001 1111 9Fh 0 0 1 to 3
#define FLASH_RDID      0x9F
///Read Status Register :           0000 0101 05h 0 0 1 to inf
#define FLASH_RDSR      0x05
///Write Status Register :          0000 0001 01h 0 0 1
#define FLASH_WRSR      0x01
///Read Data Bytes:                 0000 0011 03h 3 0 1 to inf
#define FLASH_READ      0x03
///Read Data Bytes at Higher Speed: 0000 1011 0Bh 3 1 1 to inf
#define FLASH_FAST_READ 0x0B
///Page Program :                   0000 0010 02h 3 0 1 to 256
#define FLASH_PP        0x02
///Sector Erase :                   1101 1000 D8h 3 0 0
#define FLASH_SE        0xD8
///Bulk Erase :                     1100 0111 C7h 0 0 0
#define FLASH_BE        0xC7

//Status register fields
#define WIP_BIT         0
#define WEL_BIT         1
#define BP0_BIT         2
#define BP1_BIT         3
#define BP2_BIT         4
#define SRWD_BIT        7


/** Flash memory protection threshold
 * All addresses below threshold can not be written or erased) */
#define FLASH_PROTECTION_THR      0x00480000

#define FAKE            ((void *) 1)

/*
 * MACROS
 ****************************************************************************************
 */
/**
 * @brief Macro to send a command to flash
 * @param cmd   Command to send
 */
#define SEND_COMMAND(cmd)                                                    \
    flash_spi_transfer_state = FLASH_SPI_TRANSFER_ONGOING;                             \
    spi_write((uint8_t*) &cmd, sizeof(cmd), &flash_spi_transfer_callback);


/**
 * @brief Macro to send data to flash
 * @param size   Size of data to send
 * @param pData  Pointer on data to send
 */
#define SEND_DATA(size, pData)                                              \
    flash_spi_transfer_state = FLASH_SPI_TRANSFER_ONGOING;                             \
    spi_write(pData, size, &flash_spi_transfer_callback);


/**
 * @brief Macro to receive a result from flash
 * @param res   Result to receive
 */
#define RECEIVE_RESULT(res)                                                 \
    flash_spi_transfer_state = FLASH_SPI_TRANSFER_ONGOING;                             \
    spi_read((uint8_t*) &res, sizeof(res), &flash_spi_transfer_callback);

/**
 * @brief Macro to receive data from flash
 * @param size   Size of data to receive
 * @param pData  Pointer on data to receive
 */
#define RECEIVE_DATA(size, pData)                                           \
    flash_spi_transfer_state = FLASH_SPI_TRANSFER_ONGOING;                             \
    spi_read(pData, size, &flash_spi_transfer_callback);


/**
 * @brief Macro to poll SPI transfer status
 * Status is expected to be updated by transfer callback when transaction
 * is completed.
 */
#define POLL_TRANSFER_STATUS() \
    while(flash_spi_transfer_state) \
    {\
        spi_poll(); \
        ASSERT_ERR(flash_spi_transfer_state <= FLASH_SPI_TRANSFER_ONGOING); \
    } \


/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

/// Flash states
enum FLASH_STATE
{
    IDLE,
    IDENTIFY,
    ERASE,
    WRITE,
    READ,
};

/// Flash identify states
enum FLASH_IDENTIFY_STATE
{
    IDENTIFY_IDLE,
    IDENTIFY_RDID,
    IDENTIFY_RES,
};

/// Flash erase states
enum FLASH_ERASE_STATE
{
    ERASE_IDLE,
    ERASE_WREN,
    ERASE_SE,
    ERASE_RDST,
    ERASE_RES,
};

/// Flash write states
enum FLASH_WRITE_STATE
{
    WRITE_IDLE,
    WRITE_WREN,
    WRITE_PP,
    WRITE_DATA,
    WRITE_RDST,
    WRITE_RES,
};

/// Flash read states
enum FLASH_READ_STATE
{
    READ_IDLE,
    READ_RD,
    READ_DATA,
};

///Flash-SPI transfer states
enum FLASH_SPI_TRANSFER_STATE
{
    FLASH_SPI_TRANSFER_DONE = 0,
    FLASH_SPI_TRANSFER_ONGOING  = 1
};

///Flash-SPI polling states
enum FLASH_SPI_POLLING_STATE
{
    FLASH_SPI_POLLING_INACTIVE = 0,
    FLASH_SPI_POLLING_ACTIVE   = 1
};


/*
 * STRUCT DEFINITIONS
 ****************************************************************************************
 */

///Flash Rad ID command format
struct cmd_read_id
{
    uint8_t cmd;
};

///Flash Read Status command format
struct cmd_read_status
{
    uint8_t cmd;
};

///Flash Sector Erase command format
struct cmd_sector_erase
{
    uint8_t cmd;
    uint8_t address[3];
};

///Flash Write Enable command format
struct cmd_write_enable
{
    uint8_t cmd;
};

///Flash Write status command format
struct cmd_write_status
{
    uint8_t cmd;
    uint8_t status;
};

///Flash page program command format
struct cmd_page_program
{
    uint8_t cmd;
    uint8_t address[3];
};

///Flash Read Data command format
struct cmd_read_data
{
    uint8_t cmd;
    uint8_t address[3];
};

///Flash Rad ID result format
struct res_read_id
{
    uint8_t man_id;
    uint8_t mem_type;
    uint8_t mem_capacity;
};

///Flash Rasd Status result format
struct res_read_status
{
    uint8_t status;
};

///Structure defining FLASH environment parameters
struct flash_env_tag
{
    /// Address in Flash memory for the current operation
    uint32_t address;
    /// Remaining size for the current operation
    uint32_t size;
    /// Address in RAM memory for the current operation
    uint8_t* bufptr;

    /// Callback for the current operation
    void     (*callback)(void);

    /// Current operation or state
    uint8_t state;

    /// Current operation substate
    uint8_t substate;

    /// Buffer for temporary result storing
    union
    {
        struct res_read_status res_read_status;
        struct res_read_id res_read_id;
    } result;
};


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

///Current Flash-SPI transfer state
static volatile enum FLASH_SPI_TRANSFER_STATE  flash_spi_transfer_state;
///Current Flash-SPI polling state
static enum FLASH_SPI_POLLING_STATE  flash_spi_polling_state;

///FLASH environment
static struct flash_env_tag flash_env;


/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief   Disable flash write protection.
 * This function performs command sequence to disable write protection in flash.
 ****************************************************************************************
 */
static void flash_disable_protection(void);

/**
 ****************************************************************************************
 * @brief   Identify flash sequencer.
 * This function performs command sequence to identify the flash device.
 ****************************************************************************************
 */
static void flash_identify_seq();

/**
 ****************************************************************************************
 * @brief   Erase flash sequencer.
 * This function performs command sequence to erase sectors in flash.
 * Full sector will be erased.
 ****************************************************************************************
 */
static void flash_erase_seq(void);

/**
 ****************************************************************************************
 * @brief   Program flash sequencer.
 *
 * This function performs command sequence to program data in flash.
 * Write starts at specified address. If data stream exceeds page boundary, it continues
 * the beginning of the page.
 * Page size is 256B.
 ****************************************************************************************
 */
static void flash_write_seq(void);

/**
 ****************************************************************************************
 * @brief   Read flash sequencer.
 *
 * This function performs command sequence to read data from flash.
 * Read starts at specified address. It data stream exceeds memory boundary, it continues
 * the beginning of the memory.
 ****************************************************************************************
 */
static void flash_read_seq(void);

/**
 ****************************************************************************************
 * @brief   Manage flash operations.
 *
 * This function manage the current flash operation.
 ****************************************************************************************
 */
static void flash_manage(void);

/**
 ****************************************************************************************
 * @brief   Flash SPI end of transfer callback.
 *
 * This function is called each time a read or write transfer has been completed.
 * It switches global transfer status flag to TRANSFER_DONE.
 ****************************************************************************************
 */
//static void flash_spi_transfer_callback(void);

/**
 ****************************************************************************************
 * @brief   Finish the current operation.
 *
 * This function polls the SPI and performs the current procedure until completed.
 ****************************************************************************************
 */
static void flash_perform_operation(void);

/**
 ****************************************************************************************
 * @brief   Finish the current operation with SPI IRQ handling.
 *
 * This function waits SPI IRQs and performs the current procedure until completed.
 *
 * Note: shall not be called under critical section
 ****************************************************************************************
 */
static void flash_perform_operation_irq(void);


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

static void flash_disable_protection(void)
{
  // Disable SPI interrupts
    spi_interrupt_mode(SPI_DISABLED);

    //1. Send flash write enable command
    qspi_stig_single_command(FLASH_WREN);
    qspi_stig_wait_wip(); 
   

    //2. Send flash write status register command
    qspi_stig_write_status(0,2);
    qspi_stig_wait_wip(); 


    // Re-enable SPI interrupts
    spi_interrupt_mode(SPI_ENABLED);
}

static void flash_identify_seq()
{
   uint32_t flash_id = 0;
    // Check substate
    switch(flash_env.substate)
    {
        
        case IDENTIFY_IDLE:
        {
        

            //1. Send flash read id command
	     flash_id = qspi_stig_read_spi_flash_Identification();
	     //dbg_print("flash_identify_seq:%x\r\n",flash_id);
	      flash_env.result.res_read_id.man_id =  (flash_id)&0xff ;
             flash_env.result.res_read_id.mem_type= (flash_id>>8)&0xff ;
             flash_env.result.res_read_id.mem_capacity = (flash_id>>16)&0xff ;

            // Move substate
            flash_env.substate = IDENTIFY_RDID;
            DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);
        }
        break;

        case IDENTIFY_RDID:
        {
            //2. Receive flash read id result

            // Move substate
            flash_env.substate = IDENTIFY_RES;
            DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);
        }
        break;

        case IDENTIFY_RES:
        {

        

	       void  (*callback)(void)  = flash_env.callback;
            // Check results
            if ((flash_env.result.res_read_id.man_id       == FLASH_MAN_ID_M25P128) &&
                (flash_env.result.res_read_id.mem_type     == FLASH_MEM_TYPE_M25P128) &&
                (flash_env.result.res_read_id.mem_capacity == FLASH_MEM_CAPA_M25P128))
            {
                *flash_env.bufptr = FLASH_TYPE_NUMONYX_M25P128;
            }
	     else   if ((flash_env.result.res_read_id.man_id       == FLASH_MAN_ID_GD25LQ80C) &&
                (flash_env.result.res_read_id.mem_type     == FLASH_MEM_TYPE_GD25LQ80C) &&
                (flash_env.result.res_read_id.mem_capacity == FLASH_MEM_CAPA_GD25LQ80C))
	     {
                 *flash_env.bufptr = FLASH_TYPE_GIGADEVICE_GD25LQ80C;
		
	     }
            else
            {
                *flash_env.bufptr = FLASH_TYPE_UNKNOWN;
            }

            // Clear substate
            flash_env.substate = IDENTIFY_IDLE;
            DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);

            // Clear state
            flash_env.state = IDLE;
            DBG_SWDIAG(FLASH, STATE, flash_env.state);

            // Clear parameters
            flash_env.bufptr = NULL;
            flash_env.size = 0;
            flash_env.callback = NULL;
	       // Call end of operation callback
            if(callback != NULL && callback != FAKE)
            {
                callback();
            }

        
        }
        break;

        default:
        {
            ASSERT_INFO(0, flash_env.state, flash_env.substate);
        }
        break;
    }
}

static void flash_erase_seq()
{
	//dbg_print("flash_erase_seq,%x,%x\r\n",flash_env.substate,flash_env.address);

   // Check substate
    switch(flash_env.substate)
    {
        case ERASE_IDLE:
        {
            //1. Send flash write enable command
        
	      qspi_stig_single_command(FLASH_WREN);
             qspi_stig_wait_wip(); 

            // Move substate
            flash_env.substate = ERASE_WREN;
            DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);
        }
        break;

        case ERASE_WREN:
        {
            //2. Send flash sector erase command
          
	     qspi_stig_erase(FLASH_SE , flash_env.address);

            // Update parameters
            if (flash_env.size > FLASH_SECT_SIZE)
                flash_env.size -= FLASH_SECT_SIZE;
            else
                flash_env.size = 0;
            flash_env.address += FLASH_SECT_SIZE;

            // Move substate
            flash_env.substate = ERASE_SE;
            DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);
        }
        break;

        case ERASE_SE:
        {
         
          
            //4. Send flash read status command
            qspi_stig_wait_wip(); 

            // Move substate
            flash_env.substate = ERASE_RDST;
            DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);
        }
        break;

        case ERASE_RDST:
        {
            //5. Poll flash status register
         
            // Move substate
            flash_env.substate = ERASE_RES;
            DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);
        }
        break;

        case ERASE_RES:
        {
             // If data remaining to write, restart the algorithm to write next page
                if(flash_env.size > 0)
                {
            
                    // Move substate
                    flash_env.substate = ERASE_WREN;
                    DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);
                }
                // Else the write operation is finished
                else
                {
                     void  (*callback)(void)  = flash_env.callback;

                 
                    // Clear substate
                    flash_env.substate = ERASE_IDLE;
                    DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);

                    // Clear state
                    flash_env.state = IDLE;
                    DBG_SWDIAG(FLASH, STATE, flash_env.state);

                    // Clear parameters
                    flash_env.bufptr = NULL;
                    flash_env.size = 0;
                    flash_env.callback = NULL;

		      // Call end of operation callback
                    if(callback != NULL && callback != FAKE)
                    {
                        callback();
                    }

                 
                }

              
                
            
        }
        break;

        default:
        {
            ASSERT_INFO(0, flash_env.state, flash_env.substate);
        }
        break;
    }
	 //dbg_print("flash_erase_seq end\r\n");
}

static void flash_write_seq()
{
	//dbg_print("flash_write_seq,%x\r\n",flash_env.substate);

  // Check substate
    switch(flash_env.substate)
    {
        case WRITE_IDLE:
        {
            //1. Send flash write enable command
             qspi_stig_single_command(FLASH_WREN);
             qspi_stig_wait_wip(); 

            // Move substate
            flash_env.substate = WRITE_WREN;
            DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);
        }
        break;

        case WRITE_WREN:
        {
            //2. Send flash page program command
            // Move substate
            flash_env.substate = WRITE_PP;
            DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);
        }
        break;

        case WRITE_PP:
        {
            // Determine number of bytes to write in current page
            uint8_t size = co_min(flash_env.size, (((flash_env.address & FLASH_PAGE_MASK) + FLASH_PAGE_SIZE) - flash_env.address));
          
            //3. Send data
	        // ms_flash_alg_programpage(flash_env.address, size, flash_env.bufptr);
		    //ms_flash_alg_program(flash_env.address, size, flash_env.bufptr);//ok
		    ms_flash_stig_write(flash_env.address, size, flash_env.bufptr);
            //  memcpy((unsigned char *)flash_env.address, (unsigned  char *)( flash_env.bufptr), size);//ok
            // Update parameters
            flash_env.size -= size;
            flash_env.bufptr += size;
            flash_env.address += size;

            // Move substate
            flash_env.substate = WRITE_DATA;
            DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);
        }
        break;

        case WRITE_DATA:
        {
        
              qspi_stig_wait_wip(); 
            // Move substate
            flash_env.substate = WRITE_RDST;
            DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);
        }
        break;

        case WRITE_RDST:
        {
            //5. Poll flash status register
            // Move substate
            flash_env.substate = WRITE_RES;
            DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);
        }
        break;

        case WRITE_RES:
        {
               

                // If data remaining to write, restart the algorithm to write next page
                if(flash_env.size > 0)
                {
              
                    // Move substate
                    flash_env.substate = WRITE_WREN;
                    DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);
                }
                // Else the write operation is finished
                else
                {
                 
                     void  (*callback)(void)  = flash_env.callback;
                  
                    // Clear substate
                    flash_env.substate = WRITE_IDLE;
                    DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);

                    // Clear state
                    flash_env.state = IDLE;
                    DBG_SWDIAG(FLASH, STATE, flash_env.state);

                    // Clear parameters
                    flash_env.bufptr = NULL;
                    flash_env.size = 0;
                    flash_env.callback = NULL;

		       // Call end of operation callback
                    if(callback != NULL && callback != FAKE)
                    {
                        callback();
                    }

            
                }
           
        }
        break;

        default:
        {
            ASSERT_INFO(0, flash_env.state, flash_env.substate);
        }
        break;
    }
}

static void flash_read_seq()
{
   // Check substate
    switch(flash_env.substate)
    {
        case READ_IDLE:
        {
	    
            //1. Send flash read data command
            // Move substate
            flash_env.substate = READ_RD;
            DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);
        }
        break;

        case READ_RD:
        {
	    
            //2. Receive data
	        //memcpy((unsigned char *)flash_env.bufptr, (unsigned  char *)( flash_env.address), flash_env.size);
            ms_flash_stig_read(flash_env.address, flash_env.size, flash_env.bufptr);
            // Move substate
            flash_env.substate = READ_DATA;
            DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);
        }
        break;

        case READ_DATA:
        {
    
       
             void  (*callback)(void)  = flash_env.callback;

            // Move substate
            flash_env.substate = READ_IDLE;
            DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);

            // Clear state
            flash_env.state = IDLE;
            DBG_SWDIAG(FLASH, STATE, flash_env.state);

            // Clear parameters
            flash_env.bufptr = NULL;
            flash_env.size = 0;
            flash_env.callback = NULL;

	     // Call end of operation callback
            if(callback != NULL && callback != FAKE)
            {
                callback();
            }
       
        }
        break;

        default:
        {
            ASSERT_INFO(0, flash_env.state, flash_env.substate);
        }
        break;
    }
}

static void flash_manage(void)
{
    DBG_SWDIAG(FLASH, MANAGE, 1);

    GLOBAL_INT_DISABLE();
	// dbg_print("flash_manage,%x\r\n",flash_env.state);

    switch(flash_env.state)
    {
        case IDENTIFY:
        {
            // Identification procedure
            flash_identify_seq();
        }
        break;

        case ERASE:
        {
            // Erase procedure
            flash_erase_seq();
        }
        break;

        case WRITE:
        {
            // Write procedure
            flash_write_seq();
        }
        break;

        case READ:
        {
            // Read procedure
            flash_read_seq();
        }
        break;

        default:
        {

        }
        break;
    }

    GLOBAL_INT_RESTORE();

    DBG_SWDIAG(FLASH, MANAGE, 0);
}

#if 0
static void flash_spi_transfer_callback(void)
{
    DBG_SWDIAG(FLASH, CALLBACK, 1);

    // Clear transfer state
    flash_spi_transfer_state = FLASH_SPI_TRANSFER_DONE;

    if (flash_spi_polling_state == FLASH_SPI_POLLING_INACTIVE)
    {
        flash_manage();
    }

    DBG_SWDIAG(FLASH, CALLBACK, 0);
}

#endif
static void flash_perform_operation(void)
{
	//dbg_print("flash_perform_operation,%x\r\n",flash_env.state);

    // Disable SPI interrupts
    spi_interrupt_mode(SPI_DISABLED);

    // Set active polling state
    flash_spi_polling_state = FLASH_SPI_POLLING_ACTIVE;

    do
    {
        // Process SPI transfer completion
        flash_manage();

        // Wait SPI transfer completion
        POLL_TRANSFER_STATUS();
    } while(flash_env.state != IDLE);

    // Set inactive polling state
    flash_spi_polling_state = FLASH_SPI_POLLING_INACTIVE;

    // Re-enable SPI interrupts
    spi_interrupt_mode(SPI_ENABLED);
}

static void flash_perform_operation_irq(void)
{
    // Set active polling state
    flash_spi_polling_state = FLASH_SPI_POLLING_ACTIVE;

    do
    {
        // Process SPI transfer completion
        flash_manage();

        // Wait SPI transfer completion
        while(flash_spi_transfer_state);
    }while(flash_env.state != IDLE);

    // Set active polling state
    flash_spi_polling_state = FLASH_SPI_POLLING_INACTIVE;
}


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void flash_init_old(void)
{
    // Initialize environment
    memset(&flash_env, 0, sizeof(flash_env));

    // Initialize spi transfer status flag
    flash_spi_transfer_state = FLASH_SPI_TRANSFER_DONE;

    // Disable flash write protection
    flash_disable_protection();

    // Initialize state
    flash_env.state = IDLE;
    DBG_SWDIAG(FLASH, STATE, flash_env.state);
}

uint8_t flash_identify(uint8_t* id, void (*callback)(void))
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    do
    {
        //  Check state
        if(flash_env.state != IDLE)
            break;

        status = CO_ERROR_NO_ERROR;

        // Set state
        flash_env.state = IDENTIFY;
        DBG_SWDIAG(FLASH, STATE, flash_env.state);

        // Initiate sub state
        flash_env.substate = IDENTIFY_IDLE;
        DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);

        // Store parameters
        flash_env.bufptr = id;
        flash_env.callback = callback;

        if(callback == FAKE)
        {
            flash_perform_operation_irq();
        }
        else if(callback == NULL)
        {
            // Perform the operation completely
            flash_perform_operation();
        }
        else
        {
            // Sequence entry
            flash_manage();
        }

    } while(0);

    return (status);
}

uint8_t flash_erase_old(uint8_t flash_type, uint32_t offset, uint32_t size, void (*callback)(void))
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

	 //dbg_print("flash_erase_old:offset%x,size:%x,flash_env.address:%x\r\n",offset,size,flash_env.address);

    do
    {
        //  Check flash type
        if((flash_type != FLASH_TYPE_NUMONYX_M25P128) && (flash_type != FLASH_TYPE_GIGADEVICE_GD25LQ80C))
            break;

        //  Avoid address overflow
        if(offset+size > FLASH_MEMORY_SIZE)
            break;

        //  Check state
        if(flash_env.state != IDLE)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        status = CO_ERROR_NO_ERROR;

        // Set state
        flash_env.state = ERASE;
        DBG_SWDIAG(FLASH, STATE, flash_env.state);

        // Initiate sub state
        flash_env.substate = ERASE_IDLE;
        DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);

        // Align to previous sector boundary
        flash_env.address = offset & FLASH_SECT_MASK;
        flash_env.size = size + (offset - flash_env.address);
        flash_env.callback = callback;

		  //dbg_print("flash_erase_old:%x,%x\r\n",offset,flash_env.address);

        if(callback == FAKE)
        {
            flash_perform_operation_irq();
        }
        else if(callback == NULL)
        {
            // Perform the operation completely
            flash_perform_operation();
        }
        else
        {
            // Sequence entry
            flash_manage();
        }

    } while(0);

    return (status);
}

uint8_t flash_write_old(uint8_t flash_type, uint32_t offset, uint32_t length, uint8_t *buffer, void (*callback)(void))
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;
	 //dbg_print("flash_write_old,%x,%x,offset:%x,length:%x\r\n",callback,flash_type,offset,length);

    do
    {
        //  Check flash type
        if((flash_type != FLASH_TYPE_NUMONYX_M25P128) && (flash_type != FLASH_TYPE_GIGADEVICE_GD25LQ80C))
            break;

        //  Avoid address overflow
        if(offset+length > FLASH_MEMORY_SIZE)
            break;

        //  Check state
        if(flash_env.state != IDLE)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        status = CO_ERROR_NO_ERROR;

        // Set state
        flash_env.state = WRITE;
        DBG_SWDIAG(FLASH, STATE, flash_env.state);

        // Initiate sub state
        flash_env.substate = WRITE_IDLE;
        DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);

        // Store parameters
        flash_env.address = offset;
        flash_env.size = length;
        flash_env.bufptr = buffer;
        flash_env.callback = callback;


        if(callback == FAKE)
        {
            flash_perform_operation_irq();
        }
        else if(callback == NULL)
        {
            // Perform the operation completely
            flash_perform_operation();
        }
        else
        {
            // Sequence entry
            flash_manage();
        }

    } while(0);

    return (status);
}

uint8_t flash_read_old(uint8_t flash_type, uint32_t offset, uint32_t length, uint8_t *buffer, void (*callback)(void))
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    do
    {
        //  Check flash type
        if((flash_type != FLASH_TYPE_NUMONYX_M25P128) && (flash_type != FLASH_TYPE_GIGADEVICE_GD25LQ80C))
            break;

        //  Avoid address overflow
        if(offset + length > FLASH_MEMORY_SIZE)
            break;

        //  Check state
        if(flash_env.state != IDLE)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        status = CO_ERROR_NO_ERROR;

        // Set READ state
        flash_env.state = READ;
        DBG_SWDIAG(FLASH, STATE, flash_env.state);

        // Set READ_IDLE sub state
        flash_env.substate = READ_IDLE;
        DBG_SWDIAG(FLASH, SUBSTATE, flash_env.substate);

        // Store parameters
        flash_env.address = offset;
        flash_env.size = length;
        flash_env.bufptr = buffer;
        flash_env.callback = callback;


        if(callback == FAKE)
        {
            flash_perform_operation_irq();
        }
        else if(callback == NULL)
        {
            // Perform the operation completely
            flash_perform_operation();
        }
        else
        {
            // Sequence entry
            flash_manage();
        }

    } while(0);

    return (status);
}


/// @} FLASH
