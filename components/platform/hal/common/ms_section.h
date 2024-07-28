/**
  * Copyright © 2021 by MooreSilicon. All rights reserved
  * @file  ms_section.h
  * @brief section
  * @author che.jiang
  * @date 2022年1月14日
  * @version 1.0
  * @Revision:
  */
#ifndef _MS_SECTION_H_
#define _MS_SECTION_H_

/*flash const data or flash function */
#define APP_FLASH_TEXT_SECTION     __attribute__((section(".flash.text")))
#define POWER_SETTING_SECTION     __attribute__((section(".power.setting")))
#define APP_FLASH_RODATA_SECTION   __attribute__((section(".flash.rodata")))

/* app encryption */


/*  global variable or ram function,  data and code on (default) */
#define RAM_DATA_SECTION    __attribute__((section(".data")))  
#define RAM_BSS_SECTION     __attribute__((section(".bss"))) 

#define RAM_RWIP_CONST_SECTION    __attribute__((section(".rwip.const")))  
#define RAM_RWIP_BSS_SECTION    __attribute__((section(".rwip.bss")))  
#define RAM_RWIP_DATA_SECTION    __attribute__((section(".rwip.data"))) 
#define RAM_RWIP_FUNCTION           __attribute__((section(".rwip.func")))
//#define RAM_RWIP_FUNCTION           __attribute__((section(".func.rom")))
#define RAM_FUNCTION           __attribute__((section(".func")))  
#define RAM_SLEEP_FUNCTION   __attribute__((section(".func")))


/*  retention global variable or ram function,  data and code on (default) */
#define RETENTION_RAM_DATA_SECTION    __attribute__((section(".retram.data")))  /*!<retention ram  */
#define RETENTION_RAM_FUNCTION        __attribute__((section(".func_retram")))  /*!< retention ram */
#define BOOTROM_FUNCTION        __attribute__((section(".func.rom")))  /*!< retention ram */

/** @} */ /* End of group APP_SECTION */


#endif /* _MS_SECTION_H_ */
