/**
****************************************************************************************
*
* @file rwip.c
*
* @brief RW IP SW main module
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup RW IP SW main module
 * @ingroup ROOT
 * @brief The RW IP SW main module.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // RW SW configuration

#include <string.h>          // for mem* functions
#include <stdio.h>
#include "arch.h"            // Platform architecture definition
#include "compiler.h"
#include "co_version.h"      // version information
#include "co_utils.h"

#include "rwip.h"            // RW definitions
#include "rwip_int.h"        // RW internal definitions

#if (NVDS_SUPPORT)
#include "nvds.h"         // NVDS definitions
#endif // NVDS_SUPPORT

#if (BT_EMB_PRESENT)
#include "rwbt.h"            // rwbt definitions
#endif //BT_EMB_PRESENT

#if (BLE_EMB_PRESENT)
#include "rwble.h"           // rwble definitions
#endif //BLE_EMB_PRESENT

#if (HOST_PRESENT)
#include "hl.h"              // BLE HL definitions
#endif // (HOST_PRESENT)

#if (BLE_GAF_PRESENT)
#include "gaf_inc.h"         // Isochronous Layers Definitions
#endif //(BLE_GAF_PRESENT)

#if (APP_PRESENT)
#include "app.h"             // Application definitions
#endif // (APP_PRESENT)

#include "led.h"             // led definitions

#if (BT_EMB_PRESENT)
#include "ld.h"
#endif //BT_EMB_PRESENT

#if (AUDIO_SYNC_SUPPORT)
#include "audio_sync.h"
#endif // (AUDIO_SYNC_SUPPORT)

#if (DISPLAY_SUPPORT)
#include "display.h"         // display definitions
#include "co_utils.h"        // toolbox
#include "plf.h"             // platform definition
#if (BT_EMB_PRESENT)
#include "reg_btcore.h"
#endif // (BT_EMB_PRESENT)
#if (BLE_EMB_PRESENT)
#include "reg_blecore.h"
#endif // (BLE_EMB_PRESENT)
#if (BT_DUAL_MODE)
#include "reg_ipcore.h"
#endif // (BT_DUAL_MODE)
#endif //DISPLAY_SUPPORT

#if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
#include "sch_arb.h"            // Scheduling Arbiter
#include "sch_prog.h"           // Scheduling Programmer
#include "sch_plan.h"           // Scheduling Planner
#include "sch_slice.h"          // Scheduling Slicer
#include "sch_alarm.h"          // Scheduling Alarm
#endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

#if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
#include "rf.h"              // RF definitions
#endif //BT_EMB_PRESENT || BLE_EMB_PRESENT

#if (H4TL_SUPPORT)
#include "h4tl.h"
#endif //H4TL_SUPPORT

#if (AHI_TL_SUPPORT)
#include "ahi.h"
#endif //AHI_TL_SUPPORT

#if (HCI_PRESENT)
#include "hci.h"             // HCI definition
#endif //HCI_PRESENT

#include "ke.h"              // kernel definition
#include "ke_event.h"        // kernel event
#include "ke_timer.h"        // definitions for timer
#include "ke_mem.h"          // kernel memory manager

#include "co_djob.h"         // Common DJob
#include "co_time.h"         // Common Time
#include "co_math.h"         // For address alignement macros
#if(CO_BUF_PRESENT)
#include "co_buf.h"          // Common Buffers
#endif//(CO_BUF_PRESENT)

#if (ECC_P256_SUPPORT)
#include "ecc_p256.h"        // ECC P256 library
#endif // (ECC_P256_SUPPORT)

#include "aes.h"             // For AES functions

#if (BLE_EMB_PRESENT)
#include "reg_blecore.h"        // ble core registers
#endif //BLE_EMB_PRESENT

#if (BT_EMB_PRESENT)
#include "reg_btcore.h"         // bt core registers
#endif //BT_EMB_PRESENT

#if (BLE_EMB_PRESENT && BLE_ISO_PRESENT)
#include "dma.h"             // HW accelerator for memory copy
#endif // (BLE_EMB_PRESENT && BLE_ISO_PRESENT)

#include "dbg.h"             // debug definition

#include "log.h"


/*
 * DEFINES & CONSTANT DECLARATIONS
 ****************************************************************************************
 */

#if (DISPLAY_SUPPORT)
///Table of HW image names for display
extern const char* plf_type[];
extern const char* rf_type[];

/// FW type display line
#if (BT_EMB_PRESENT && BLE_EMB_PRESENT)
#define FW_TYPE_DISPLAY   "FW: BTDM split emb"
#elif (BT_EMB_PRESENT)
#define FW_TYPE_DISPLAY   "FW: BT split emb"
#elif (BLE_EMB_PRESENT && BLE_HOST_PRESENT)
#define FW_TYPE_DISPLAY   "FW: BLE full"
#elif (BLE_EMB_PRESENT)
#define FW_TYPE_DISPLAY   "FW: BLE split emb"
#else
#define FW_TYPE_DISPLAY   "FW: ROM"
#endif // BT_EMB_PRESENT / BLE_EMB_PRESENT / BLE_HOST_PRESENT
#endif //DISPLAY_SUPPORT

 // Heap header size is 12 bytes
 #define RWIP_HEAP_HEADER             (12 / sizeof(uint32_t))
 // ceil(len/sizeof(uint32_t)) + RWIP_HEAP_HEADER
 #define RWIP_CALC_HEAP_LEN(len)      ((((len) + (sizeof(uint32_t) - 1)) / sizeof(uint32_t)) + RWIP_HEAP_HEADER)
 // compute size of the heap block in bytes
 #define RWIP_CALC_HEAP_LEN_IN_BYTES(len)  (RWIP_CALC_HEAP_LEN(len) * sizeof(uint32_t))


/*
 * STRUCT DEFINITIONS
 ****************************************************************************************
 */
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
/// Local supported commands
const struct rwip_prio rwip_priority[RWIP_PRIO_IDX_MAX]={
    #if (BT_EMB_PRESENT)
    {RWIP_PRIO_ACL_DFT,                   RWIP_INCR_ACL_DFT},
    {RWIP_PRIO_ACL_ACT,                   RWIP_INCR_ACL_ACT},
    {RWIP_PRIO_ACL_RSW,                   RWIP_INCR_ACL_RSW},
    {RWIP_PRIO_ACL_SNIFF_DFT,             RWIP_INCR_ACL_SNIFF_DFT},
    {RWIP_PRIO_ACL_SNIFF_TRANS,           RWIP_INCR_ACL_SNIFF_TRANS},
    #if MAX_NB_SYNC
    {RWIP_PRIO_SCO_DFT,                   RWIP_INCR_SCO_DFT},
    #endif //MAX_NB_SYNC
    {RWIP_PRIO_BCST_DFT,                  RWIP_INCR_BCST_DFT},
    {RWIP_PRIO_BCST_ACT,                  RWIP_INCR_BCST_ACT},
    {RWIP_PRIO_CSB_RX_DFT,                RWIP_INCR_CSB_RX_DFT},
    {RWIP_PRIO_CSB_TX_DFT,                RWIP_INCR_CSB_TX_DFT},
    {RWIP_PRIO_INQ_DFT,                   RWIP_INCR_INQ_DFT},
    {RWIP_PRIO_ISCAN_DFT,                 RWIP_INCR_ISCAN_DFT},
    {RWIP_PRIO_PAGE_DFT,                  RWIP_INCR_PAGE_DFT},
    {RWIP_PRIO_PAGE_1ST_PKT,              RWIP_INCR_PAGE_1ST_PKT},
    {RWIP_PRIO_PCA_DFT,                   RWIP_INCR_PCA_DFT},
    {RWIP_PRIO_PSCAN_DFT,                 RWIP_INCR_PSCAN_DFT},
    {RWIP_PRIO_PSCAN_1ST_PKT,             RWIP_INCR_PSCAN_1ST_PKT},
    {RWIP_PRIO_SSCAN_DFT,                 RWIP_INCR_SSCAN_DFT},
    {RWIP_PRIO_STRAIN_DFT,                RWIP_INCR_STRAIN_DFT},
    #endif // #if (BT_EMB_PRESENT)
    #if (BLE_EMB_PRESENT)
    {RWIP_PRIO_SCAN_DFT,                  RWIP_INCR_SCAN_DFT},
    {RWIP_PRIO_AUX_RX_DFT,                RWIP_INCR_AUX_RX_DFT},
    {RWIP_PRIO_PER_ADV_RX_DFT,            RWIP_INCR_PER_ADV_RX_DFT},
    {RWIP_PRIO_PER_ADV_RX_ESTAB,          RWIP_INCR_PER_ADV_RX_ESTAB},
    {RWIP_PRIO_INIT_DFT,                  RWIP_INCR_INIT_DFT},
    {RWIP_PRIO_CONNECT_DFT,               RWIP_INCR_CONNECT_DFT},
    {RWIP_PRIO_CONNECT_ACT,               RWIP_INCR_CONNECT_ACT},
    {RWIP_PRIO_CONNECT_ESTAB,             RWIP_INCR_CONNECT_ESTAB},
    {RWIP_PRIO_CONNECT_INSTANT,           RWIP_INCR_CONNECT_INSTANT},
    {RWIP_PRIO_ADV_DFT,                   RWIP_INCR_ADV_DFT},
    {RWIP_PRIO_ADV_HDC_DFT,               RWIP_INCR_ADV_HDC_PRIO_DFT},
    {RWIP_PRIO_ADV_AUX_DFT,               RWIP_INCR_ADV_AUX_DFT},
    {RWIP_PRIO_PER_ADV_DFT,               RWIP_INCR_PER_ADV_DFT},
    {RWIP_PRIO_RPA_RENEW_DFT,             RWIP_INCR_RPA_RENEW_DFT},
    #if (BLE_CIS)
    {RWIP_PRIO_M_CIS_DFT,                 RWIP_INCR_M_CIS_DFT},
    {RWIP_PRIO_S_CIS_DFT,                 RWIP_INCR_S_CIS_DFT},
    #endif // (BLE_CIS)
    #if (BLE_BIS)
    {RWIP_PRIO_M_BIS_DFT,                 RWIP_INCR_M_BIS_DFT},
    {RWIP_PRIO_S_BIS_DFT,                 RWIP_INCR_S_BIS_DFT},
    {RWIP_PRIO_BIS_SCAN_DFT,              RWIP_INCR_BIS_SCAN_DFT},
    #endif // (BLE_BIS)
    #endif // #if (BLE_EMB_PRESENT)
};
#endif//#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
#if (RW_WLAN_COEX || RW_MWS_COEX)
const uint8_t rwip_coex_cfg[RWIP_COEX_CFG_MAX]={
    #if (BT_EMB_PRESENT)
    [RWIP_COEX_MSSWITCH_IDX] = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS) | (RWIP_SAM_DIS << RWIP_SAMEN_POS)),
    [RWIP_COEX_SNIFFATT_IDX] = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS) | (RWIP_SAM_DIS << RWIP_SAMEN_POS)),
    [RWIP_COEX_PAGE_IDX]     = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTEN << RWIP_DNABORT_POS) | (RWIP_SAM_DIS << RWIP_SAMEN_POS)),
    [RWIP_COEX_PSCAN_IDX]    = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS) | (RWIP_SAM_DIS << RWIP_SAMEN_POS)),
    [RWIP_COEX_INQ_IDX]      = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS) | (RWIP_SAM_DIS << RWIP_SAMEN_POS)),
    [RWIP_COEX_INQRES_IDX]   = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS) | (RWIP_SAM_DIS << RWIP_SAMEN_POS)),
    [RWIP_COEX_SCORSVD_IDX]  = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS) | (RWIP_SAM_DIS << RWIP_SAMEN_POS)),
    [RWIP_COEX_BCAST_IDX]    = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS) | (RWIP_SAM_DIS << RWIP_SAMEN_POS)),
    [RWIP_COEX_CONNECT_IDX]  = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS) | (RWIP_SAM_EN << RWIP_SAMEN_POS)),
    #endif // #if (BT_EMB_PRESENT)
    #if (BLE_EMB_PRESENT)
    [RWIP_COEX_CON_IDX]     = (uint8_t)((RWIP_PTI_TXDIS << RWIP_TXBSY_POS)  | (RWIP_PTI_RXDIS << RWIP_RXBSY_POS)  | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS)),
    [RWIP_COEX_CON_DATA_IDX]= (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS)  | (RWIP_PTI_RXEN << RWIP_RXBSY_POS)  | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS)),
    [RWIP_COEX_ADV_IDX]     = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS)  | (RWIP_PTI_RXDIS << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS)),
    [RWIP_COEX_SCAN_IDX]    = (uint8_t)((RWIP_PTI_TXDIS << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS)  | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS)),
    [RWIP_COEX_INIT_IDX]    = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS)  | (RWIP_PTI_RXEN << RWIP_RXBSY_POS)  | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS)),
    #endif // #if (BLE_EMB_PRESENT)
};
#endif //(RW_WLAN_COEX || RW_MWS_COEX)
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// RF API
struct rwip_rf_api         rwip_rf;
/// Parameters API
struct rwip_param_api      rwip_param;

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
/// Programming delay, margin for programming the baseband in advance of each activity (in half-slots)
uint8_t rwip_prog_delay;
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)

/// Heap definitions - use uint32 to ensure that memory blocks are 32bits aligned.
/// Memory allocated for environment variables
uint32_t rwip_heap_env[RWIP_CALC_HEAP_LEN(RWIP_HEAP_ENV_SIZE)];
#if (HOST_PRESENT)
/// Memory allocated for profiles
uint32_t rwip_heap_profile[RWIP_CALC_HEAP_LEN(RWIP_HEAP_PROFILE_SIZE)];
#endif // (HOST_PRESENT)
/// Memory allocated for kernel messages
uint32_t rwip_heap_msg[RWIP_CALC_HEAP_LEN(RWIP_HEAP_MSG_SIZE)];
/// Non Retention memory block
uint32_t rwip_heap_non_ret[RWIP_CALC_HEAP_LEN(RWIP_HEAP_NON_RET_SIZE)];
/// IP reset state variable (@see enum rwip_init_type)
__STATIC uint8_t rwip_rst_state = RWIP_PRE_INIT;

#if(CO_BUF_PRESENT)
/// Big Buffer Pool
uint32_t rwip_buf_pool_big[(CO_BUF_BIG_POOL_SIZE >> 2)];
/// Small Buffer Pool
uint32_t rwip_buf_pool_small[(CO_BUF_SMALL_POOL_SIZE >> 2)];
#endif // (CO_BUF_PRESENT)

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (!NVDS_SUPPORT)
__STATIC uint8_t rwip_param_dummy_get(uint8_t param_id, uint8_t * lengthPtr, uint8_t *buf)
{
    return (PARAM_FAIL);
}
__STATIC uint8_t rwip_param_dummy_set(uint8_t param_id, uint8_t length, uint8_t *buf)
{
    return (PARAM_FAIL);
}
__STATIC uint8_t rwip_param_dummy_del(uint8_t param_id)
{
    return (PARAM_FAIL);
}
#endif // (!NVDS_SUPPORT)

#if (DISPLAY_SUPPORT)
/**
 ****************************************************************************************
 * @brief Display device configuration
 *
 * This function adds graphical display
 ****************************************************************************************
 */
__STATIC void display_add_config(void)
{
    struct plf_version plfversion;
    struct plf_version plfversion_unkn;

    // Get platform version, date, time ...
    plf_read_version(&plfversion);

    /************************************************************************/
    /*                              FW TYPE                                 */
    /************************************************************************/
    {
        char scr_fw_type[DISPLAY_LINE_SIZE+1];
        uint8_t scr_id = display_screen_alloc();
        display_screen_insert(scr_id, 0);

        memset(&plfversion_unkn, 0, sizeof(plfversion_unkn));

        if((plfversion.ip_type == 0) || (memcmp(&plfversion, &plfversion_unkn, sizeof(plfversion_unkn)) == 0))
            sprintf(scr_fw_type,"HW Unknown");
        else if(plfversion.plf_type == 0)
            sprintf(scr_fw_type,"HW: Backup Image");
        else if(plfversion.ip_type == 1)
            sprintf(scr_fw_type,"HW: %s40 %s %2x",plf_type[plfversion.plf_type],rf_type[plfversion.rf_type],plfversion.version);
        else if(plfversion.ip_type == 7)
            sprintf(scr_fw_type,"HW: %s41 %s %2x",plf_type[plfversion.plf_type],rf_type[plfversion.rf_type],plfversion.version);
        else if(plfversion.ip_type == 8)
            sprintf(scr_fw_type,"HW: %s42 %s %2x",plf_type[plfversion.plf_type],rf_type[plfversion.rf_type],plfversion.version);
        else if(plfversion.ip_type == 9)
            sprintf(scr_fw_type,"HW: %s50 %s %2x",plf_type[plfversion.plf_type],rf_type[plfversion.rf_type],plfversion.version);
        else if(plfversion.ip_type == 10)
            sprintf(scr_fw_type,"HW: %s51 %s %2x",plf_type[plfversion.plf_type],rf_type[plfversion.rf_type],plfversion.version);
        else if(plfversion.ip_type == 11)
            sprintf(scr_fw_type,"HW: %s52 %s %2x",plf_type[plfversion.plf_type],rf_type[plfversion.rf_type],plfversion.version);
        else if(plfversion.ip_type == 0xbb)
            sprintf(scr_fw_type,"HW: %s52+DS %s %2x",plf_type[plfversion.plf_type],rf_type[plfversion.rf_type],plfversion.version);
        else
            sprintf(scr_fw_type,"HW Unknown %x", plfversion.ip_type);

        display_screen_set(scr_id, NULL, scr_fw_type , FW_TYPE_DISPLAY);
    }

    /************************************************************************/
    /*                             FW VERSION                               */
    /************************************************************************/
    {
        uint8_t byte;
        char line[DISPLAY_LINE_SIZE+1];
        uint8_t scr_id = display_screen_alloc();
        display_screen_insert(scr_id, 0);
        byte = RWBT_SW_VERSION_MAJOR;
        co_bytes_to_string(&line[0], &byte, 1);
        line[2] = '.';
        byte = RWBT_SW_VERSION_MINOR;
        co_bytes_to_string(&line[3], &byte, 1);
        line[5] = '.';
        byte = RWBT_SW_VERSION_BUILD;
        co_bytes_to_string(&line[6], &byte, 1);
        line[8] = '.';
        byte = RWBT_SW_VERSION_SUB_BUILD;
        co_bytes_to_string(&line[9], &byte, 1);
        line[11] = '\0';
        display_screen_set(scr_id, NULL, "FW version:", line);
    }

    /************************************************************************/
    /*                              FW TIME                                 */
    /************************************************************************/
    {
        char line[DISPLAY_LINE_SIZE+1];
        uint8_t scr_id = display_screen_alloc();
        display_screen_insert(scr_id, 0);

        /* Build the FW type screen with:
         *  - type
         *  - build date "Mmm dd yyyy"
         *  - build time "hh:mm:ss"
         */
        strncpy(line, __DATE__, 7);
        strncpy(line+7, __TIME__, 8);
        line[DISPLAY_LINE_SIZE] = '0';
        display_screen_set(scr_id, NULL, "FW date:", line);
    }

    /************************************************************************/
    /*                            FPGA VERSION                              */
    /* The HW FPGA version w.xy.0z.v is coded is coded as below :           */
    /*	w : IP SIG release  )(09 for 5.0)                                   */
    /*   x : processor type(0: Cortus, 1 : RISCV)                           */
    /*   y : RW IP(1 :BLE, 2 : BT, 3 : BTDM)                                */
    /*   z : RF type(1: Ripple, 3 : Calypso, 4 : Icytrx_v2, 5 : btipt, 6:aura50)*/
    /*   v : synthesis build version                                        */
    /************************************************************************/
    {
        char line[DISPLAY_LINE_SIZE + 1];
        uint8_t xy = (plfversion.cpu_type << 4) + plfversion.plf_type;
        uint8_t scr_id = display_screen_alloc();
        display_screen_insert(scr_id, 0);
        co_bytes_to_string(&line[0], &plfversion.ip_type, 1);
        line[2] = '.';
        co_bytes_to_string(&line[3], &xy, 1);
        line[5] = '.';
        co_bytes_to_string(&line[6], &plfversion.rf_type, 1);
        line[8] = '.';
        co_bytes_to_string(&line[9], &plfversion.version, 1);
        line[11] = '\0';
        display_screen_set(scr_id, NULL, "FPGA version:", line);
    }

    /************************************************************************/
    /*                           FPGA DATE/TIME                             */
    /************************************************************************/
    {
        char* ptr;
        uint8_t value;
        uint8_t digit;
        char line[DISPLAY_LINE_SIZE+1];
        uint8_t scr_id = display_screen_alloc();
        display_screen_insert(scr_id, 0);

        ptr = line;
        // Month
        value = plfversion.month;
        digit = (value/10) + 48;
        *(ptr++) = (char)digit;
        digit = (value - 10*(value/10)) + 48;
        *(ptr++) = (char)digit;
        *(ptr++) = '_';
        // Day
        value = plfversion.day;
        digit = (value/10) + 48;
        *(ptr++) = (char)digit;
        digit = (value - 10*(value/10)) + 48;
        *(ptr++) = (char)digit;
        *(ptr++) = ' ';
        // Hours
        value = plfversion.hour;
        digit = (value/10) + 48;
        *(ptr++) = (char)digit;
        digit = (value - 10*(value/10)) + 48;
        *(ptr++) = (char)digit;
        *(ptr++) = ':';
        // Minutes
        value = plfversion.minute;
        digit = (value/10) + 48;
        *(ptr++) = (char)digit;
        digit = (value - 10*(value/10)) + 48;
        *(ptr++) = (char)digit;
        *(ptr++) = ':';
        // Seconds
        value = plfversion.second;
        digit = (value/10) + 48;
        *(ptr++) = (char)digit;
        digit = (value - 10*(value/10)) + 48;
        *(ptr++) = (char)digit;
        *(ptr++) = '\0';
        display_screen_set(scr_id, NULL, "FPGA Date:", line);
    }

    #if (BT_EMB_PRESENT)
    /************************************************************************/
    /*                           BT HW VERSION                              */
    /************************************************************************/
    {
        uint8_t byte;
        char line[DISPLAY_LINE_SIZE+1];
        uint8_t scr_id = display_screen_alloc();
        display_screen_insert(scr_id, 0);
        byte = bt_version_typ_getf();
        co_bytes_to_string(&line[0], &byte, 1);
        line[2] = '.';
        byte = bt_version_rel_getf();
        co_bytes_to_string(&line[3], &byte, 1);
        line[5] = '.';
        byte = bt_version_upg_getf();
        co_bytes_to_string(&line[6], &byte, 1);
        line[8] = '.';
        byte = bt_version_build_getf();
        co_bytes_to_string(&line[9], &byte, 1);
        line[11] = '\0';
        display_screen_set(scr_id, NULL, "BT HW version:", line);
    }
    #endif // (BT_EMB_PRESENT)

    #if (BLE_EMB_PRESENT)
    /************************************************************************/
    /*                           BLE HW VERSION                              */
    /************************************************************************/
    {
        uint8_t byte;
        char line[DISPLAY_LINE_SIZE+1];
        uint8_t scr_id = display_screen_alloc();
        display_screen_insert(scr_id, 0);
        byte = ble_version_typ_getf();
        co_bytes_to_string(&line[0], &byte, 1);
        line[2] = '.';
        byte = ble_version_rel_getf();
        co_bytes_to_string(&line[3], &byte, 1);
        line[5] = '.';
        byte = ble_version_upg_getf();
        co_bytes_to_string(&line[6], &byte, 1);
        line[8] = '.';
        byte = ble_version_build_getf();
        co_bytes_to_string(&line[9], &byte, 1);
        line[11] = '\0';
        display_screen_set(scr_id, NULL, "BLE HW version:", line);
    }
    #endif // (BLE_EMB_PRESENT)

    #if (BT_DUAL_MODE)
    /************************************************************************/
    /*                           DM HW VERSION                              */
    /************************************************************************/
    {
        uint8_t byte;
        char line[DISPLAY_LINE_SIZE+1];
        uint8_t scr_id = display_screen_alloc();
        display_screen_insert(scr_id, 0);
        byte = ip_version_typ_getf();
        co_bytes_to_string(&line[0], &byte, 1);
        line[2] = '.';
        byte = ip_version_rel_getf();
        co_bytes_to_string(&line[3], &byte, 1);
        line[5] = '.';
        byte = ip_version_upg_getf();
        co_bytes_to_string(&line[6], &byte, 1);
        line[8] = '.';
        byte = ip_version_build_getf();
        co_bytes_to_string(&line[9], &byte, 1);
        line[11] = '\0';
        display_screen_set(scr_id, NULL, "DM HW version:", line);
    }
    #endif // (BT_DUAL_MODE)

    /************************************************************************/
    /*                            DEVICE NAME                               */
    /************************************************************************/
    {
        uint8_t dev_name_length = PARAM_LEN_DEVICE_NAME;
        uint8_t dev_name_data[PARAM_LEN_DEVICE_NAME];
        uint8_t scr_id = display_screen_alloc();
        display_screen_insert(scr_id, 0);

        if(rwip_param.get(PARAM_ID_DEVICE_NAME, &dev_name_length, dev_name_data) == PARAM_OK)
        {
            // Put end of line
            if(dev_name_length > 16)
            {
                dev_name_length = 16;
            }
            dev_name_data[dev_name_length] = '\0';
        }
        else
        {
            dev_name_data[0] = '\0';
        }
        display_screen_set(scr_id, NULL, "Device name:", (char*)dev_name_data);
    }

    /************************************************************************/
    /*                              BD ADDRESS                              */
    /************************************************************************/
    {
        char scr_bd_ad[DISPLAY_LINE_SIZE+1];
        uint8_t bd_ad_length = PARAM_LEN_BD_ADDRESS;
        uint8_t bd_ad_data[PARAM_LEN_BD_ADDRESS];
        uint8_t scr_id = display_screen_alloc();
        display_screen_insert(scr_id, 0);

        strcpy(scr_bd_ad, "0x");
        if(rwip_param.get(PARAM_ID_BD_ADDRESS, &bd_ad_length, bd_ad_data) == PARAM_OK)
        {
            // Encode to ASCII
            for(int i = 0; i < PARAM_LEN_BD_ADDRESS; i++)
            {
                uint8_t digit = bd_ad_data[PARAM_LEN_BD_ADDRESS-1-i]>>4;
                digit += (digit < 10) ? 48:55;
                *(scr_bd_ad+2+2*i) = (char)digit;
                digit = bd_ad_data[PARAM_LEN_BD_ADDRESS-1-i]&0xF;
                digit += (digit < 10) ? 48:55;
                *(scr_bd_ad+2+2*i+1) = (char)digit;
            }
        }
        scr_bd_ad[14] = '\0';
        display_screen_set(scr_id, NULL, "BD Address:", scr_bd_ad);
    }

    #if (PLF_UART)
    /************************************************************************/
    /*                            UART BAUDRATE                             */
    /************************************************************************/
    {
        int i = 0;
        char line[DISPLAY_LINE_SIZE+1];
        uint8_t uart_length = PARAM_LEN_UART_BAUDRATE;
        uint32_t baudrate = 921600;
        uint8_t scr_id = display_screen_alloc();
        display_screen_insert(scr_id, 0);

        // Get UART baudrate from NVDS
        if(rwip_param.get(PARAM_ID_UART_BAUDRATE, &uart_length, (uint8_t*) &baudrate) == PARAM_OK)
        {
            if(baudrate > 3500000 || baudrate < 9600)
            {
                baudrate = 921600;
            }
        }
        else
        {
            baudrate = 921600;
        }

        // Display UART baudrate on screen
        strcpy(line, "        bps");
        while (baudrate > 0)
        {
            line[6-i++] = (baudrate - (10*(baudrate / 10))) + 48;
            baudrate = baudrate / 10;
        }
        display_screen_set(scr_id, NULL, "UART baudrate:", line);
    }
    #endif //PLF_UART

    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /************************************************************************/
    /*                               RF BOARD                               */
    /************************************************************************/
    {
        uint8_t scr_id = display_screen_alloc();
        display_screen_insert(scr_id, 0);

        #if defined(RF_BOARD_ID_READ)
        {
            char line[DISPLAY_LINE_SIZE+1];
            // Read board ID in platform
            uint16_t rf_id = plf_read_rf_board_id();
            // Add screen to display RF board type
            strcpy(line, RF_BOARD_STR);
            line[8] = (rf_id/10) + 48;
            line[9] = (rf_id - (10*(rf_id/10))) + 48;
            line[10] = '\0';
            display_screen_set(scr_id, NULL, "RF board:", line);
        }
        #else // !defined(RF_BOARD_ID_READ)
        display_screen_set(scr_id, NULL, "RF board:", RF_BOARD_STR);
        #endif // !defined(RF_BOARD_ID_READ)
    }
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

    /************************************************************************/
    /*                               CPU TYPE                               */
    /************************************************************************/
    {
        uint8_t scr_id = display_screen_alloc();
        display_screen_insert(scr_id, 0);

        if (plfversion.cpu_type == 0)
            display_screen_set(scr_id, NULL, "CPU type:", "Cortus APS3");
        else if (plfversion.cpu_type == 1)
            display_screen_set(scr_id, NULL, "CPU type:", "RISC-V");
        else
            display_screen_set(scr_id, NULL, "CPU type:", "Unknown");
    }

    // Start with FW type screen
    display_start(0);
}
#endif //DISPLAY_SUPPORT

/**
 ****************************************************************************************
 * @brief IP initialization of PRNG
 *
 * The method below shall be replaced by platform-specific code to seed the PRNG using
 * a TRNG physical source with at least 20 bits of entropy.
 *
 ****************************************************************************************
 */
void rwip_rand_init()
{
    uint32_t seed;
    rwip_time_t current_time = rwip_time_get();

    seed =  current_time.hs;
    seed += current_time.hus;

    co_random_init(seed);
}

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void rwip_init(uint32_t error)
{
    #if (NVDS_SUPPORT)
    // Point to NVDS for parameters get/set
    rwip_param.get = nvds_get;
    rwip_param.set = nvds_put;
    rwip_param.del = nvds_del;
    #else // (NVDS_SUPPORT)
    // !! Need to point to some parameter configuration system
    ASSERT_WARN(GAIA_SUPPORT, 0, 0);
    rwip_param.get = rwip_param_dummy_get;
    rwip_param.set = rwip_param_dummy_set;
    rwip_param.del = rwip_param_dummy_del;
    #endif // (NVDS_SUPPORT)

    MS_LOGI(MS_DRIVER, "rwip_init enter\r\n");

    // Initialize kernel
    ke_init();

    MS_LOGI(MS_DRIVER, "ke_init\r\n");

    // Initialize memory heap used by kernel.
    // Memory allocated for environment variables
    ke_mem_init(KE_MEM_ENV,           (uint8_t*)rwip_heap_env,     RWIP_CALC_HEAP_LEN_IN_BYTES(RWIP_HEAP_ENV_SIZE));
    #if (HOST_PRESENT)
    // Memory allocated for Attribute database
    ke_mem_init(KE_MEM_PROFILE,       (uint8_t*)rwip_heap_profile, RWIP_CALC_HEAP_LEN_IN_BYTES(RWIP_HEAP_PROFILE_SIZE));
    #endif // (HOST_PRESENT)
    // Memory allocated for kernel messages
    ke_mem_init(KE_MEM_KE_MSG,        (uint8_t*)rwip_heap_msg,     RWIP_CALC_HEAP_LEN_IN_BYTES(RWIP_HEAP_MSG_SIZE));
    // Non Retention memory block
    ke_mem_init(KE_MEM_NON_RETENTION, (uint8_t*)rwip_heap_non_ret, RWIP_CALC_HEAP_LEN_IN_BYTES(RWIP_HEAP_NON_RET_SIZE));
    MS_LOGI(MS_DRIVER, "ke_mem_init\r\n");

    // IP initialization
        MS_LOGI(MS_DRIVER, "IP initialization\r\n");
    rwip_rst_state = RWIP_INIT;

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    #if (RW_DEBUG)
    // Initialize the debug process
    MS_LOGI(MS_DRIVER, "rwip_rst_state %d\r\n",rwip_rst_state);
    dbg_init(rwip_rst_state);
	    MS_LOGI(MS_DRIVER, "dbg_init\r\n");
    #endif //(RW_DEBUG)
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

    // Initialize RF
    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    rf_init(&rwip_rf);
    #endif //BT_EMB_PRESENT || BLE_EMB_PRESENT


    // Initialize random process
    rwip_rand_init();

    #if (DISPLAY_SUPPORT)
    // Initialize display module
    display_init();

    // Add some configuration information to display
    display_add_config();
    #endif //DISPLAY_SUPPORT

    #if (ECC_P256_SUPPORT)
    // Initialize Diffie Hellman Elliptic Curve Algorithm
    ecc_init(rwip_rst_state);
    #endif // (ECC_P256_SUPPORT)

    // Initialize H4TL
    #if (H4TL_SUPPORT)
    #if (H4TL_NB_CHANNEL > 1)
    h4tl_init(1, rwip_eif_get(1));
    #endif // (H4TL_NB_CHANNEL > 1)
    h4tl_init(0, rwip_eif_get(0));
    #endif //(H4TL_SUPPORT)

    #if (HCI_PRESENT)
    // Initialize the HCI
    hci_initialize(rwip_rst_state);
    #endif //HCI_PRESENT

    #if (AHI_TL_SUPPORT)
    // Initialize the Application Host Interface
    ahi_init();
    #endif //AHI_TL_SUPPORT

    #if (HOST_PRESENT)
    // Initialize BLE Host stack
    hl_initialize(rwip_rst_state);

    #if (BLE_GAF_PRESENT)
    gaf_init(rwip_rst_state);
    #endif //(BLE_GAF_PRESENT)
    #endif //(HOST_PRESENT)

    #if (BT_EMB_PRESENT)
    // Initialize BT
    rwbt_init();

    // Initialize channel assessment environment
    memset(&rwip_env.ch_assess_bt, 0, sizeof(struct rwip_ch_assess_data_bt));
    #endif //BT_EMB_PRESENT

    #if (BLE_EMB_PRESENT)
    // Initialize BLE
    rwble_init(rwip_rst_state);
	MS_LOGI(MS_DRIVER, "rwble_init\r\n");

    // Initialize channel assessment environment
    memset(&rwip_env.ch_assess_ble, 0, sizeof(struct rwip_ch_assess_data_ble));
    #endif //BLE_EMB_PRESENT

    // Initialize IP core driver
    rwip_driver_init(rwip_rst_state);

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    #if (RW_WLAN_COEX)
    // WLAN COEX enabled immediate
    rwip_wlcoex_set(1);
    #endif //(RW_WLAN_COEX)
    #if (RW_MWS_COEX)
    // MWS COEX disabled until Ext Frame Configuration
    rwip_mwscoex_set(0);
    #endif //(RW_MWS_COEX)
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

    #if (BT_EMB_PRESENT || (BLE_EMB_PRESENT && !BLE_HOST_PRESENT))
    // If FW initializes due to FW reset, send the message to Host
    if(error != RESET_NO_ERROR)
    {
        if(error == RESET_TO_ROM || error == RESET_AND_LOAD_FW)
        {
            // Send platform reset command complete if requested by user
            dbg_platform_reset_complete(error);
        }
        else
        {
            // Allocate a message structure for hardware error event
            struct hci_hw_err_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_HW_ERR_EVT_CODE, hci_hw_err_evt);

            // Fill the HW error code
            switch(error)
            {
                case RESET_MEM_ALLOC_FAIL: evt->hw_code = CO_ERROR_HW_MEM_ALLOC_FAIL; break;
                default: ASSERT_INFO(0, error, 0); break;
            }

            // Send the message
            hci_send_2_host(evt);
        }
    }
    #endif //(BT_EMB_PRESENT || (BLE_EMB_PRESENT && !BLE_HOST_PRESENT))

    // initialize Common block module
    co_djob_initialize(rwip_rst_state);
    co_time_init(rwip_rst_state);
    #if (CO_BUF_PRESENT)
    co_buf_init(rwip_rst_state, rwip_buf_pool_big, rwip_buf_pool_small);
    #endif // (CO_BUF_PRESENT)

    /*
     ************************************************************************************
     * Application initialization
     ************************************************************************************
     */
    #if (APP_PRESENT)
    // Initialize APP
    app_init();
    #endif //APP_PRESENT
	MS_LOGI(MS_DRIVER, "app_init\r\n");

    // Move to IP first reset state
    rwip_rst_state = RWIP_1ST_RST;

    #if (!(BLE_HOST_PRESENT|| BT_HOST_PRESENT) && (BLE_EMB_PRESENT || BT_EMB_PRESENT))
    // Make a full initialization in split-emb mode
    rwip_reset();
		MS_LOGI(MS_DRIVER, "rwip_reset\r\n");
    #endif // (!(BLE_HOST_PRESENT|| BT_HOST_PRESENT) && (BLE_EMB_PRESENT || BT_EMB_PRESENT))
}

void rwip_reset(void)
{
    // Disable interrupts until reset procedure is completed
    GLOBAL_INT_DISABLE();

    // remove all timers
    ke_timer_flush();
    // initialize Common block module
    co_djob_initialize(rwip_rst_state);
    co_time_init(rwip_rst_state);

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    #if (RW_DEBUG)
    // Reset dbg
    dbg_init(rwip_rst_state);
    #endif //(RW_DEBUG)
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

    #if (ECC_P256_SUPPORT)
    // Reset Diffie Hellman Elliptic Curve Algorithm
    ecc_init(rwip_rst_state);
    #endif // (ECC_P256_SUPPORT)

    #if (HCI_PRESENT)
    // Reset the HCI
    hci_initialize(rwip_rst_state);
    #endif //HCI_PRESENT

    #if (HOST_PRESENT)
    // Initialize BLE Host stack
    hl_initialize(rwip_rst_state);

    #if (BLE_GAF_PRESENT)
    gaf_init(rwip_rst_state);
    #endif // (BLE_GAF_PRESENT)
    #endif // (HOST_PRESENT)

    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    // Initialize IP core driver
    rwip_driver_init(rwip_rst_state);
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

    #if (BT_EMB_PRESENT)
    if (rwip_rst_state == RWIP_RST)
    {
        // Reset BT
        rwbt_reset();

        // Initialize channel assessment environment
        memset(&rwip_env.ch_assess_bt, 0, sizeof(struct rwip_ch_assess_data_bt));
    }
    #endif //BT_EMB_PRESENT

    #if (BLE_EMB_PRESENT)
    // Reset BLE
    rwble_init(rwip_rst_state);
    // Initialize channel assessment environment
    memset(&rwip_env.ch_assess_ble, 0, sizeof(struct rwip_ch_assess_data_ble));
    #endif //BLE_EMB_PRESENT

    // Reset AES
    aes_init(rwip_rst_state);

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    // Reset Scheduling blocks
    sch_arb_init(rwip_rst_state);
    sch_prog_init(rwip_rst_state);
    sch_plan_init(rwip_rst_state);
    sch_alarm_init(rwip_rst_state);
    sch_slice_init(rwip_rst_state);
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    #if (RW_WLAN_COEX)
    // WLAN COEX enabled immediate
    rwip_wlcoex_set(1);
    #endif //(RW_WLAN_COEX)
    #if (RW_MWS_COEX)
    // MWS COEX disabled until Ext Frame Configuration
    rwip_mwscoex_set(0);
    #endif //(RW_MWS_COEX)
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

    // clean-up buffers
    #if (CO_BUF_PRESENT)
    co_buf_init(rwip_rst_state, rwip_buf_pool_big, rwip_buf_pool_small);
    #endif // (CO_BUF_PRESENT)

    //Clear all message pending
    ke_flush();

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    if (rwip_rst_state == RWIP_RST)
    {
        // Reset the RF
        rwip_rf.reset();
    }
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

    #if (AUDIO_SYNC_SUPPORT)
    audio_sync_init();
    #endif //(AUDIO_SYNC_SUPPORT)

    #if (DISPLAY_SUPPORT)
    // Restart display module
    display_resume();
    #endif //DISPLAY_SUPPORT

    // Move to normal IP reset state
    rwip_rst_state = RWIP_RST;

    // Restore interrupts once reset procedure is completed
    GLOBAL_INT_RESTORE();
}

void rwip_schedule(void)
{
    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    // If system is waking up, delay the handling up to the Bluetooth clock is available and corrected
    if ((rwip_env.prevent_sleep & RW_WAKE_UP_ONGOING) == 0)
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    {
        // schedule all pending events
        ke_event_schedule();
    }
}

#if (BT_EMB_PRESENT)
#if PCA_SUPPORT
bool rwip_pca_clock_dragging_only(void)
{
#if (BLE_EMB_PRESENT)
    return rwble_activity_ongoing_check();
#else
    return false;
#endif // BLE_EMB_PRESENT
}
#endif // PCA_SUPPORT
#endif // BT_EMB_PRESENT

#if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
#if (RW_MWS_COEX)
void rwip_mwscoex_set(bool en)
{
    // Set MWS Coexistence interface accordingly. Leave MWS WCI Interface disabled.
    #if (BT_EMB_PRESENT)
    bt_coexifcntl0_mwswci_en_setf(0);
    bt_coexifcntl0_mwscoex_en_setf(en);
    #if (BLE_EMB_PRESENT)
    // Valid in Dual Mode only
    ble_coexifcntl0_mwswci_en_setf(0);
    ble_coexifcntl0_mwscoex_en_setf(en);
    #endif // BLE_EMB_PRESENT
    #endif // BT_EMB_PRESENT
}
#endif // RW_MWS_COEX
#if (RW_WLAN_COEX)
void rwip_wlcoex_set(bool en)
{
    // Set WLAN Coexistence interface accordingly, and bt/ble sync pulse generation.
    #if (BT_EMB_PRESENT)
    bt_coexifcntl0_syncgen_en_setf(en);
    bt_coexifcntl0_wlancoex_en_setf(en);
    #endif // BT_EMB_PRESENT
    #if (BLE_EMB_PRESENT)
    ble_coexifcntl0_syncgen_en_setf(en);
    ble_coexifcntl0_wlancoex_en_setf(en);
    #endif // BLE_EMB_PRESENT
}
#endif // RW_WLAN_COEX
#endif // (BT_EMB_PRESENT || BLE_EMB_PRESENT)

#if RW_DEBUG
void rwip_assert(const char * file, int line, int param0, int param1, uint8_t type)
{
    #if (!HOST_PRESENT)
    if (rwip_rst_state > RWIP_PRE_INIT)
    {
        uint16_t file_len = strlen(file);
        struct hci_dbg_assert_evt *evt = KE_MSG_ALLOC_DYN(HCI_DBG_EVT, 0, 0, hci_dbg_assert_evt, sizeof(struct hci_dbg_assert_evt) + CO_ALIGN4_HI(file_len+1));
        evt->subcode = HCI_DBG_ASSERT_EVT_SUBCODE;
        evt->type = type;
        evt->line = line;
        evt->param0 = param0;
        evt->param1 = param1;
        strcpy((char *) evt->file, file);

        // Lib function strlen() has been found to use 32-bit memory accesses, so can perform read-access a few bytes beyond end of string,
        // which may not be allocated/initialized in kernel memory. Permission updated here for strlen() in hci_dbg_assert_evt_pkupk.
        DBG_MEM_INIT(&evt->file[file_len], CO_ALIGN4_HI(file_len+1) - file_len);

        hci_send_2_host(evt);
    }
    #endif //(!HOST_PRESENT)
}
#endif //RW_DEBUG

#if (BLE_EMB_PRESENT && BLE_ISO_PRESENT)
void rwip_iso_data_transfer (void* p_dst_addr, const void* p_src_addr, uint16_t size)
{
    // Use DMA to copy buffer
    dma_copy(DMA_CHAN_ISOAL, p_dst_addr, p_src_addr, size);
}
#endif // (BLE_EMB_PRESENT && BLE_ISO_PRESENT)

///@} RW
