/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_pwr.h
 * @brief
 * @author che.jiang
 * @date 2021-12-30
 * @version 1.0
 * @Revision
 */
#ifndef MS_PWR_H_
#define MS_PWR_H_

#include <ms_pwr_hal.h>
#include <stdbool.h>



/**
 * @brief  PWR Sleep Mode
 */
typedef enum
{
    PWR_MODE_POWERDOWN = 0,
    PWR_MODE_DEEPSLEEP,
    PWR_MODE_SLEEP,
    PWR_MODE_ACTIVE,
    PWR_MODE_IDLE,
}PWR_MODE_Type;

/**
 * @brief  PWR wakeup source select type
 */
typedef enum
{
    PWR_WAKEUP_SEL_RTC_TIMER    = 0x00000000U,   /*!< GPIO wakeup source            */
    PWR_WAKEUP_SEL_GPIO_PORT    = 0x00000002,    /*!< RTC Timer wakeup source       */
    PWR_WAKEUP_SEL_LP_UART      = 0x00000004,    /*!< low power uart wakeup source  */
    PWR_WAKEUP_SEL_BLE_TIMER    = 0x00000008,    /*!< BLE Timer wakeup source  */
}PWR_WAKEUP_SEL_Type;

/**
 * @brief  PWR wakeup state
 */
typedef enum {
    PWR_WAKEUP_IOMAX_STATE      = 0x00000010,   /*!< GPIO wakeup state            */
    PWR_WAKEUP_UART2_STATE      = 0x00000020,   /*!< LPUART wakeup state          */
    PWR_WAKEUP_RTC_STATE        = 0x00000040,   /*!< RTC wakeup state             */
    PWR_WAKEUP_BLE_STATE        = 0x00000100,   /*!< BLE wakeup state             */
} PWR_WAKEUP_STATE;


/**
 * @brief  PWR run flash control command
 */
typedef struct __PWR_Retention_cmd
{
    uint32_t   addr;                      /*!< Reg Address                    */
    uint32_t   value;                     /*!< Reg Value                      */
    uint32_t   wait_cnt;                  /*!< run set reg Value, wait count  */
    uint32_t   check_sum;                 /*!< check sum                      */
} PWR_Retention_cmd;

/**
 * @brief  PWR flash config  data
 */
typedef struct __PWR_Retention_Data
{
    uint8_t     header[4];                /*!< "MSFR"                        */
    uint32_t    magic;                    /*!< Magic number                  */
    uint16_t    cmd_number;               /*!< commander number              */
    uint8_t     is_valid;                 /*!< cmd valid flag,0-unvalid,1-valid */
    uint8_t     un_used;                  /*!< not used                      */
    PWR_Retention_cmd *p_cmd;             /*!< comand pointer                */
} PWR_Retention_Data;

/**
 * @brief  PWR Configuration Structure definition
 */
typedef struct __PWR_Config_Type
{
    PWR_MODE_Type pwr_mode;              /*!< pwr config mode                 */
    PWR_WAKEUP_SEL_Type ds_wkup_sel;     /*!< pwr ds wakeup source select     */
    PWR_WAKEUP_SEL_Type slp_wkup_sel;    /*!< pwr sleep wakeup source select  */
    uint32_t pull_type_p00_p15;          /*!< pwr pad wakeup  select          */
    uint32_t pull_type_p16_p26;          /*!< pwr pad wakeup  select          */
    uint32_t rtc_sleep_time_val;         /*!< pwr rtc wakeup  time            */
    int32_t (*pwr_before_sleep_callback)(); /*!< pwr user callback before sleep  */
    int32_t (*pwr_after_wakeup_callback)(); /*!< pwr user callback after wakeup  */

    /*!< pwr flash enter sleep callback  */
    int32_t (*pwr_flash_before_sleep_callback)(uint32_t addr, uint32_t *size);
    /*!< pwr gpio enter sleep callback , set gpio mode as the wakeup source */
    int32_t (*pwr_gpio_before_sleep_callback)(uint8_t pin, uint8_t dir, uint8_t level);
    
}PWR_Config_Type;

/**
 * @brief  PWR hanler Structure definition
 */
typedef struct __PWRHandle_Type
{
    SysCtrlRegs_Type  *sysctl_instance;  /*!< SYSCTL registers base address   */
    PWR_Config_Type pwr_config;          /*!< PWR Initial Config              */
} PWRHandle_Type;


/**
 * @brief This function initial pwr module  .
 * @param  PWRHandle_Type *hpwr : Pointer to  pwr module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
extern int32_t ms_pwr_init(PWRHandle_Type *hpwr);

/**
 * @brief This function deinitial pwr module  .
 * @param  PWRHandle_Type *hpwr : Pointer to  pwr module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
extern int32_t ms_pwr_deinit(PWRHandle_Type *hpwr);

/**
 * @brief This function run pwr enter ds/slp/pd mode   .
 * @param  PWRHandle_Type *hpwr : Pointer to  pwr module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
extern int32_t ms_pwr_enter(PWRHandle_Type *hpwr);

/**
 * @brief This function run pwr exit from ds/slp/pd mode   .
 * @param  PWRHandle_Type *hpwr : Pointer to  pwr module.
 * @retval STATUS_SUCCESS or STATUS_ERROR
 */
extern int32_t ms_pwr_exit(PWRHandle_Type *hpwr);

/**
 * @brief This function get pre sleep mode from ds/slp/pd mode   .
 * @param  PWRHandle_Type *hpwr : Pointer to  pwr module.
 * @retval pre sleep mode
 */
extern int32_t ms_pwr_get_pre_mode(PWRHandle_Type *hpwr);

/**
 * @brief This function config  sleep  ds/slp/pd mode   .
 * @param  PWR_MODE_Type pwr_mode : sleep mode.
 * @retval pre sleep mode
 */
extern int32_t  ms_pwr_config_sleep_mode(PWR_MODE_Type pwr_mode);


/**
 * @brief This function config  wakeup  gpio/ble timer/rtc timer/lpuart source   .
 * @param  PWR_WAKEUP_SEL_Type wakeup_sel : wakeup source.
 * @retval pre sleep mode
 */
extern int32_t  ms_pwr_config_wkup_sel(PWR_WAKEUP_SEL_Type wakeup_sel);

/**
 * @brief This function register user enter sleep mode  .
 * @param  PWRHandle_Type *hpwr : Pointer to  pwr module.
 * @retval pre sleep mode
 */
extern void ms_pwr_register_user_sleep_callback(int32_t (*sleep_cb)());

/**
 * @brief This function register user wakeup function  .
 * @param  PWRHandle_Type *hpwr : Pointer to  pwr module.
 * @retval pre sleep mode
 */
extern void ms_pwr_register_user_wakeup_callback(int32_t (*wakeup_cb)());

/**
 * @brief This function register gpio enter sleep mode  .
 * @param  PWRHandle_Type *hpwr : Pointer to  pwr module.
 * @retval pre sleep mode
 */
extern void ms_pwr_register_gpio_sleep_callback(int32_t (*sleep_cb)(uint8_t pin, uint8_t dir, uint8_t level));

/**
 * @brief This function register flash enter sleep mode  .
 * @param  PWRHandle_Type *hpwr : Pointer to  pwr module.
 * @retval pre sleep mode
 */
extern void ms_pwr_register_flash_sleep_callback(int32_t (*sleep_cb)());

/**
 * @brief This function do wakeup irq handler.
 * @param  PWRHandle_Type *hpwr : Pointer to  pwr module.
 * @retval pre sleep mode
 */
extern void ms_pwr_wakeup_irq_handler(PWRHandle_Type *hpwr);
#endif /* MS_PWR_H_ */
