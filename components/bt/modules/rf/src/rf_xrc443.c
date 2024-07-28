/*!
    \file    RF_pir815.c
    \brief   rf pir815 source file

    \version 2021-01-12, V1.0.0, firmware for XRD011S
*/

/*
    Copyright (c) 2020, ACTT Inc.

*/


/*****************************************************************************************
* @addtogroup RF_pir815
* @ingroup RF
* @brief Pir815 Radio Driver
*
* This is the driver block for pir815 radio
* @{
*****************************************************************************************/

/*****************************************************************************************
 * INCLUDE FILES
 ****************************************************************************************/
#include "rwip_config.h"        // SYD SW configuration

#include <string.h>             // for memcpy
#include "co_utils.h"           // common utility definition
#include "co_math.h"            // common math functions
#include "co_endian.h"          // endian definitions
#include "rf.h"                 // RF interface

#include "rwip.h"               // for RF API structure definition

#include "em_map.h"

#if (BLE_EMB_PRESENT)
#include "reg_blecore.h"        // ble core registers
#include "reg_em_ble_cs.h"      // control structure definitions
#endif //BLE_EMB_PRESENT

#if (BT_EMB_PRESENT)
#include "reg_btcore.h"         // bt core registers
#include "reg_em_bt_cs.h"       // control structure definitions
#endif //BT_EMB_PRESENT

#include "ms_spi.h"
#include "atiny_log.h"
#include <stdio.h>
#include "atiny_log.h"
#include "ms_common.h"
//#include "systick_delay.h"
#include "mdm.h"
#include "ms_section.h"
#include "log.h"

// TX max power
#define PIR_TXPWR_MAX       0

static ms_spi_dev_t rf_spi0;

#define SPI_RD  (1<<31)
#define SPI_WR  (0<<31)

RAM_FUNCTION static uint32_t rf_xrc443_reg_rd (uint32_t addr)
{
       uint32_t ret = 0;
#ifndef WIN32
	uint32_t cmd_snd = 0;
		
	cmd_snd = ((addr & 0x7fff) << 16) | (SPI_RD);

	  wait_nop(200);
      ms_spi_send(&rf_spi0, &cmd_snd, 1, 0);
	  wait_nop(200);
      ms_spi_recv(&rf_spi0, &ret, 1, 0);
#endif // WIN32

    return (uint32_t)ret;
}


RAM_FUNCTION static void rf_xrc443_reg_wr (uint32_t addr, uint32_t value)
{
#ifndef WIN32
	uint32_t wr = 0;

    wr |= value & 0xffff;
    wr |= ((addr & 0x7fff) << 16) | (SPI_WR); //high byte

  ms_spi_send(&rf_spi0, &wr, 1, 0);
#endif // WIN32
}

RAM_FUNCTION static int8_t rf_xrc443_txpwrdbm_get(uint8_t txpwr_idx, uint8_t modulation)
{
    return (PIR_TXPWR_MAX);
}

RAM_FUNCTION static uint8_t rf_xrc443_txpwr_cs_get(int8_t txpwr_dbm, uint8_t high)
{
    return (PIR_TXPWR_MAX);
}

RAM_FUNCTION static void rf_xrc443_sleep(void)
{
    ble_deepslcntl_set(ble_deepslcntl_get() |
                      BLE_DEEP_SLEEP_ON_BIT |    // RW BLE Core sleep
                      BLE_RADIO_SLEEP_EN_BIT |   // Radio sleep
                      BLE_OSC_SLEEP_EN_BIT);     // Oscillator sleep   
}

RAM_FUNCTION static void rf_xrc443_reset(void)
{

}

static void rf_xrc443_force_agc_enable(bool en)
{}

RAM_FUNCTION static uint8_t rf_lna_level_to_gain(uint8_t lna_gain_lv)
{
    uint8_t lna_gain;
    switch (lna_gain_lv)
    {
    case 0x0:
        lna_gain = 6;
        break;
    case 0x1:
        lna_gain = 28;
        break;
    case 0x2:
        lna_gain = 40;
        break;
    case 0x3:
        lna_gain = 56;
        break;
    case 0x7:
    default:
        lna_gain = 67;
    }
    return lna_gain;
}

RAM_FUNCTION static int8_t rf_xrc443_rssi_convert(uint8_t rssi_reg)
{
    uint8_t lna_gain_lv;
    uint8_t loss_after_shife;

    lna_gain_lv = (*(volatile uint32_t *)(0x4001100C))>>8;
    lna_gain_lv &= 0x07;
    loss_after_shife = (0) ? 2 : 4; //Todo, how to get the modem mode of 1M/2M after pdu reception?
    //LE_PRINTF("%d, %d\r\n", rssi_reg, rf_lna_level_to_gain(lna_gain_lv));
    return (int8_t)((int16_t)rssi_reg - (int16_t)(84 + rf_lna_level_to_gain(lna_gain_lv) + 3 - 4 - loss_after_shife - 13));
}

void spi_config_rf_mode(void)
{
#define  CONFIG_SPI_MODE_OFFSET   (0xB8)
#define  CONFIG_SPI_RF_MODE  (1 << 6)

    UINT32 reg_get_value;
    UINT32 reg_set_value;

    reg_get_value = inl(SYS_CTRL_BASE_ADDR + CONFIG_SPI_MODE_OFFSET);
    reg_set_value = reg_get_value | CONFIG_SPI_RF_MODE;
    outl(SYS_CTRL_BASE_ADDR + CONFIG_SPI_MODE_OFFSET,  reg_set_value);
}

void rf_init(struct rwip_rf_api *api)
{
    uint16_t  n;
	//uint32_t rf_reg;
    
    // Initialize the RF driver API structure
    api->reg_rd = rf_xrc443_reg_rd;
    api->reg_wr = rf_xrc443_reg_wr;
    api->txpwr_dbm_get = rf_xrc443_txpwrdbm_get;
    api->txpwr_cs_get = rf_xrc443_txpwr_cs_get;
    api->txpwr_max = PIR_TXPWR_MAX;
    api->sleep = rf_xrc443_sleep;
    api->reset = rf_xrc443_reset;
    api->force_agc_enable = rf_xrc443_force_agc_enable;
    api->rssi_convert = rf_xrc443_rssi_convert;
    
    rf_spi0.config.mode = SPI_ROLE_MASTER;
    rf_spi0.priv = NULL;
    rf_spi0.port = MS_SPI0_INDEX;
    /* initialize spi */

#ifndef WIN32
	spi_config_rf_mode();
    SPI0_master_config(&rf_spi0);
#endif
    //mdm_init(0);
    // RF config

#define CONF_SIZE (18)

    rfconf conf[CONF_SIZE] = {
        	{0x04, 0x1800},
        	{0x15, 0x05e0},
        	{0x28, 0x007d},
        	{0x14, 0xa43c},
        	{0x13, 0x1d1d},
        	{0x2c, 0x3e15},
        	{0x10, 0x18fb},
        	{0x120, 0x0609},
        	{0x121, 0x1378},
        	{0x00, 0x1cab},
        	{0x1a, 0x027f},
        	{0x1b, 0xcf3f},
        	{0x1c, 0x280b},
        	{0x1f, 0x5ae7},
        	{0x24, 0x9c94},
        	{0x2e, 0xfb68},
        	{0x101, 0x2002},
        	{0x2f, 0x30af}
    };

    for(n = 0; n < CONF_SIZE; n++)
    {
    rf_xrc443_reg_wr(conf[n].add, conf[n].value);
	//rf_reg = rf_xrc443_reg_rd(conf[n].add);
	//MS_LOGI(MS_DRIVER, "read rf data %x \r\n",rf_reg);
    }

}
///@} RF_PIR815ddd

