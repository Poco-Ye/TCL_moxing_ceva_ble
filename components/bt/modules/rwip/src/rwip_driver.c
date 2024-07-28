/**
****************************************************************************************
*
* @file rwip_driver.c
*
* @brief RW IP Driver SW module used to manage common IP features.
*
* Copyright (C) RivieraWaves 2009-2015
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
#include "rwip.h"            // RW definitions
#include "rwip_int.h"        // RW internal definitions
#include "arch.h"            // Platform architecture definition
#include "co_bt.h"           // Bluetooth defines

#if (NVDS_SUPPORT)
#include "nvds.h"            // NVDS definitions
#endif // NVDS_SUPPORT

#if (H4TL_SUPPORT)
#include "h4tl.h"            // H4TL definition
#endif //H4TL_SUPPORT

#include "dbg.h"             // debug definition
#include "ke_mem.h"          // for AES client management

#include "ke.h"              // To check if a kernel event is programmed
#include "ke_event.h"

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
#include "sch_alarm.h"       // for the half slot target ISR
#include "sch_arb.h"         // for the half us target ISR
#include "sch_prog.h"        // for the fifo/clock ISRs
#include "led.h"
#include "reg_ipcore.h"
#if (BLE_EMB_PRESENT)
#include "reg_ipcore_bts.h"  // ISO Bluetooth timestamp registers
#endif // (BLE_EMB_PRESENT)
#include "aes.h"             // AES result function

#if (BLE_EMB_PRESENT)
#include "rwble.h"           // for sleep and wake-up specific functions
#include "lld.h"             // for AES encryption handler
#endif // (BLE_EMB_PRESENT)
#if (BT_EMB_PRESENT)
#include "rwbt.h"            // for sleep and wake-up specific functions
#include "ld.h"              // for clock interrupt handler
#include "reg_btcore.h"      // for PCA cntl
#endif // (BT_EMB_PRESENT)"
#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)


#if (HOST_PRESENT && (!EMB_PRESENT))
#include "timer.h"
#endif // (HOST_PRESENT && (!EMB_PRESENT))

#include "co_math.h"         // min/max macros
#include "co_utils.h"        // Clock Diff

#include "atiny_log.h"
#include "ms_common.h"
#include "reg_blecore.h"
#include "reg_em_et.h"
#include "ms_section.h"

#ifdef PM_DEBUG_BLE_DEEP_SLEEP
extern int ms_sys_check_if_enter_sleep();
#endif
/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// RW SW environment
struct rwip_env_tag        rwip_env;


/*
 * CONSTANT DEFINITION
 ****************************************************************************************
 */

#if (BLE_EMB_PRESENT && BT_EMB_PRESENT)
/// Table mapping BLE channel index to/from BLE physical channels, for determining corresponding BT physical channels
const uint8_t ble_phy_to_ch_idx[40] = {37,0,1,2,3,4,5,6,7,8,9,10,38,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,39};
const uint8_t ble_ch_to_phy_idx[40] = {1,2,3,4,5,6,7,8,9,10,11,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,0,12,39};
#endif // (BLE_EMB_PRESENT && BT_EMB_PRESENT)

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
/**
 ****************************************************************************************
 * @brief Converts a duration in lp cycles into a duration in half us.
 *
 * The function converts a duration in lp cycles into a duration is half us, according to the
 * low power clock frequency (32768Hz or 32000Hz).
 *
 * To do this the following formula are applied:
 *
 *   Tus = (x*30.517578125)*2 = (30*x + x/2 + x/64 + x/512)*2 = (61*x + (x*8 + x)/256) for a 32.768kHz clock or
 *   Tus = (x*31.25)*2        = (31*x + x/4) * 2              = (62*x + x/2)           for a 32kHz clock
 *
 * @param[in]     lpcycles    duration in lp cycles
 * @param[in|out] error_corr  Insert and retrieve error created by truncating the LP Cycle Time to a half us (32kHz: 1/2 half us | 32.768kHz: 1/256 half-us)
 *
 * @return duration in half us
 ****************************************************************************************
 */
__STATIC uint32_t rwip_lpcycles_2_hus(uint32_t lpcycles, uint32_t *error_corr)
{
    uint32_t res;

    // Sanity check: The number of lp cycles should not be too high to avoid overflow
    ASSERT_ERR(lpcycles < 2000000);

    #if (HZ32000)
    // Compute the sleep duration in us - case of a 32kHz clock and insert previous computed error
    *error_corr = lpcycles + *error_corr;
    // get the truncated value
    res = *error_corr >> 1;
    // retrieve new inserted error
    *error_corr = *error_corr - (res << 1);
    // finish computation
    res = 62 * lpcycles + res;
    #else //HZ32000
    // Compute the sleep duration in half us - case of a 32.768kHz clock and insert previous computed error
    *error_corr = (lpcycles << 3) + lpcycles + *error_corr;
    // get the truncated value
    res = *error_corr >> 8;
    // retrieve new inserted error
    *error_corr = *error_corr - (res << 8);
    // finish computation
    res = 61 * lpcycles + res;
    #endif //HZ32000

    return(res);
}

/**
 ****************************************************************************************
 * @brief Converts a duration in half slots into a number of low power clock cycles.
 * The function converts a duration in half slots into a number of low power clock cycles.
 * Sleep clock runs at either 32768Hz or 32000Hz, so this function divides the value in
 * slots by 10.24 or 10 depending on the case.
 * To do this the following formulae are applied:
 *
 *   N = x * 10.24 = (1024 * x)/100 for a 32.768kHz clock or
 *   N = x * 10                     for a 32kHz clock
 *
 * @param[in] hs_cnt    The value in half slot count
 *
 * @return The number of low power clock cycles corresponding to the slot count
 *
 ****************************************************************************************
 */
__STATIC uint32_t rwip_slot_2_lpcycles(uint32_t hs_cnt)
{
    uint32_t lpcycles;

    #if HZ32000
    // Sanity check: The number of slots should not be too high to avoid overflow
    ASSERT_ERR(hs_cnt < (0xFFFFFFFF / 10));

    // Compute the low power clock cycles - case of a 32kHz clock
    lpcycles = hs_cnt * 10;
    #else //HZ32000
    // Sanity check: The number of slots should not be too high to avoid overflow
    ASSERT_ERR(hs_cnt < (0xFFFFFFFF >> 10));

    // Compute the low power clock cycles - case of a 32.768kHz clock
    lpcycles = (hs_cnt << 10)/100;
    #endif //HZ32000

    return(lpcycles);
}



/**
 ****************************************************************************************
 * @brief Converts a duration in us into a duration in lp cycles.
 *
 * The function converts a duration in us into a duration is lp cycles, according to the
 * low power clock frequency (32768Hz or 32000Hz).
 *
 * @param[in] us    duration in us
 *
 * @return duration in lpcycles
 ****************************************************************************************
 */
__STATIC uint32_t rwip_us_2_lpcycles(uint32_t us)
{
    uint32_t lpcycles;

    #if (HZ32000)
    // Compute the low power clock cycles - case of a 32kHz clock
    lpcycles = ((us * 32) + (999)) / 1000;
    #else //HZ32000
    // Compute the low power clock cycles - case of a 32.768kHz clock
    lpcycles = ((us * 32768) + (999999)) / 1000000;
    #endif //HZ32000

    return(lpcycles);
}

/**
 ****************************************************************************************
 * @brief Handles the Alarm timer target
 ****************************************************************************************
 */
__STATIC void rwip_timer_alarm_handler(void)
{
    // disable the timer driver
    rwip_env.timer_alarm_target.hs = RWIP_INVALID_TARGET_TIME;
    ip_intcntl1_timestamptgt2intmsk_setf(0);

    // call the default half slot call-back
    sch_alarm_timer_isr();
}

/**
 ****************************************************************************************
 * @brief Handles the Arbiter timer target
 ****************************************************************************************
 */
__STATIC void rwip_timer_arb_handler(void)
{
    // disable the timer driver
    rwip_env.timer_arb_target.hs = RWIP_INVALID_TARGET_TIME;
    ip_intcntl1_timestamptgt1intmsk_setf(0);

    // call the default half slot call-back
    sch_arb_event_start_isr();
}

#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

/**
 ****************************************************************************************
 * @brief Handles the Common timer target
 ****************************************************************************************
 */
__STATIC void rwip_timer_co_handler(void)
{
    // disable the timer driver
    rwip_env.timer_co_target.hs = RWIP_INVALID_TARGET_TIME;

    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    ip_intcntl1_timestamptgt3intmsk_setf(0);
    #elif (BLE_HOST_PRESENT)
    // Stop timer
    timer_set_timeout(0, NULL);
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

    // Mark that Common timer is over
    ke_event_set(KE_EVENT_TIMER);
}
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)

/**
 ****************************************************************************************
 * @brief Handles crypto event (to provide results out of interrupt context
 ****************************************************************************************
 */
__STATIC void rwip_crypt_evt_handler(void)
{
    uint8_t aes_result[KEY_LEN];

    // Clear event
    ke_event_clear(KE_EVENT_AES_END);

    // Load AES result
    em_rd(aes_result, EM_ENC_OUT_OFFSET, KEY_LEN);

    // inform AES result handler
    aes_result_handler(CO_ERROR_NO_ERROR, aes_result);
}

/**
 ****************************************************************************************
 * @brief Handles crypto interrupt
 ****************************************************************************************
 */
__STATIC void rwip_crypt_isr_handler(void)
{
    // Prevent going to deep sleep during encryption
    rwip_prevent_sleep_clear(RW_CRYPT_ONGOING);

    // Clear interrupt mask
    ip_intcntl1_cryptintmsk_setf(0);

    // mark that AES is done
    ke_event_set(KE_EVENT_AES_END);
}

/**
 ****************************************************************************************
 * @brief Handles Software requested interrupt
 ****************************************************************************************
 */
__STATIC void rwip_sw_int_handler(void)
{
    // Disable interrupt
    ip_intcntl1_swintmsk_setf(0);

    // call the SW interrupt handler
    sch_arb_sw_isr();
}

/**
 ****************************************************************************************
 * @brief Wake-up from Core sleep.
 *
 * Compute and apply the clock correction according to duration of the deep sleep.
 ****************************************************************************************
 */
__STATIC void rwip_wakeup(void)
{
    uint16_t fintetime_correction;
    // duration in half us
    uint32_t dur_hus;
    // duration in half slot
    uint32_t dur_hslot;
    // Get the number of low power sleep period
    uint32_t slp_period = ip_deepslstat_get();

    DBG_SWDIAG(SLEEP, SLEEP, 0);

    led_set(6);
    led_reset(2);

    // Sleep is over now
    rwip_prevent_sleep_clear(RW_DEEP_SLEEP);

    // Prevent going to deep sleep until a slot interrupt is received
    rwip_prevent_sleep_set(RW_WAKE_UP_ONGOING);

    // Compensate the base time counter and fine time counter by the number of slept periods
    dur_hus = rwip_lpcycles_2_hus(slp_period, &(rwip_env.sleep_acc_error));
    // Compute the sleep duration (based on number of low power clock cycles)
    dur_hslot = dur_hus / HALF_SLOT_SIZE;

    // retrieve halfslot sleep duration
    fintetime_correction = (HALF_SLOT_SIZE-1) - (dur_hus - dur_hslot*HALF_SLOT_SIZE);

    // The correction values are then deduced from the sleep duration in us
    ip_clkncntcorr_pack(/*absdelta*/ 1, /*clkncntcorr*/ dur_hus / HALF_SLOT_SIZE);

    // The correction values are then deduced from the sleep duration in us
    ip_finecntcorr_setf(fintetime_correction);

    #if (BLE_ISO_HW_PRESENT)
    // set the correction as an absolute delta in microseconds
    ip_isocntcntl_isocorrmode_setf(1);
    ip_isocntcorr_setf((dur_hus >> 1));
    ip_isocntcorr_hus_setf((dur_hus & 0x1));
    #endif // (BLE_ISO_HW_PRESENT)

    // Start the correction
    ip_deepslcntl_deep_sleep_corr_en_setf(1);

    // Enable the RWBT slot interrupt
    ip_intcntl1_clknintsrmsk_setf(0);
    ip_intcntl1_clknintmsk_setf(1);
    ip_intack1_clear(IP_CLKNINTACK_BIT);

    #if (H4TL_SUPPORT)
    // Restart the flow on the TL
    h4tl_start();
    #endif //H4TL_SUPPORT

    TRC_REQ_WAKEUP();
}



/**
 ****************************************************************************************
 * @brief Restore the core processing after the clock correction
 *
 * Enable the core and check if some timer target has been reached.
 ****************************************************************************************
 */
__STATIC void rwip_wakeup_end(void)
{
    DBG_SWDIAG(SLEEP, WAKEUP_END, 1);

    // get current time
    rwip_time_t current_time = rwip_time_get();

    // Disable clock interrupt
    ip_intcntl1_clknintmsk_setf(0);

    if(rwip_env.timer_arb_target.hs != RWIP_INVALID_TARGET_TIME)
    {
        // check if arbiter timer target is reach
        if(CLK_LOWER_EQ(rwip_env.timer_arb_target.hs, current_time.hs))
        {
            rwip_timer_arb_handler();
        }
    }

    if(rwip_env.timer_alarm_target.hs != RWIP_INVALID_TARGET_TIME)
    {
        // check if alarm timer target is reach
        if(CLK_LOWER_EQ(rwip_env.timer_alarm_target.hs, current_time.hs))
        {
            rwip_timer_alarm_handler();
        }
    }

    if(rwip_env.timer_co_target.hs != RWIP_INVALID_TARGET_TIME)
    {
        // check if common target is reached
        if(CLK_LOWER_EQ(rwip_env.timer_co_target.hs, current_time.hs))
        {
            rwip_timer_co_handler();
        }
    }

    // Wake up is complete now, so we allow the deep sleep again
    rwip_prevent_sleep_clear(RW_WAKE_UP_ONGOING);


    led_reset(6);

    DBG_SWDIAG(SLEEP, WAKEUP_END, 0);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)
rwip_time_t rwip_time_get(void)
{
    rwip_time_t res;

    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    //Sample the base time count
    ip_slotclk_samp_setf(1); // liujin mark
    while (ip_slotclk_samp_getf());
    // get base time and offset - must be read atomically
    GLOBAL_INT_DISABLE();
    res.hs  = ip_slotclk_sclk_getf();
	//MS_LOGI(MS_DRIVER, "res.hs time get %d", res.hs);
    res.hus = HALF_SLOT_INV(ip_finetimecnt_get());
    #if (BLE_ISO_HW_PRESENT)
    res.bts = ip_isocntsamp_get();
    #else  // (!BLE_ISO_HW_PRESENT)
    {
        // compute time in half microseconds between last sampled time and BT time provided in parameters
        int32_t clk_diff_hus       = CLK_DIFF(rwip_env.last_samp_time.hs, res.hs) * HALF_SLOT_SIZE;
        clk_diff_hus              += (int32_t)res.hus - (int32_t)rwip_env.last_samp_time.hus + rwip_env.samp_hus_residual;
        rwip_env.samp_hus_residual = (clk_diff_hus & 0x1);
        res.bts                    = rwip_env.last_samp_time.bts + (clk_diff_hus >> 1);
    }
    #endif // (BLE_ISO_HW_PRESENT)
    GLOBAL_INT_RESTORE();
    #elif (BLE_HOST_PRESENT)
    // get base time (1us unit)
    res.bts = timer_get_time();
    rwip_bts_to_bt_time(res.bts, &res.hs, &res.hus);
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

    // keep last sampled time to perform a clock conversion
    rwip_env.last_samp_time = res;


    return res;
}

#if (BT_EMB_PRESENT)
void rwip_time_set(uint32_t clock)
{
    GLOBAL_INT_DISABLE();
    #if !(BLE_ISO_HW_PRESENT)
    // Sample current BTS time
    uint32_t samp_bts;
    rwip_time_get();
    samp_bts = rwip_env.last_samp_time.bts;
    #endif // !(BLE_ISO_HW_PRESENT)

    // update clock
    ip_slotclk_pack(IP_SAMP_RST, 1 /* clk_upd */, clock & 0x0FFFFFFF);
    while(ip_slotclk_clkn_upd_getf());

    #if !(BLE_ISO_HW_PRESENT)
    // Get time again to sample new clockN
    rwip_time_get();
    // restore previous sampled BTS clock value
    rwip_env.last_samp_time.bts = samp_bts;
    #endif // !(BLE_ISO_HW_PRESENT)

    // Inform co_timer module to restart timer after clock correction
    ke_event_set(KE_EVENT_TIMER);
    GLOBAL_INT_RESTORE();
}

void rwip_time_adj(int16_t clk_adj_us)
{
    #if !(BLE_ISO_HW_PRESENT)
    // Sample current BTS time
    uint32_t samp_bts;
    rwip_time_get();
    samp_bts = rwip_env.last_samp_time.bts;
    #endif // !(BLE_ISO_HW_PRESENT)

    // Write down the clock us adjustment, cut sign extension to 11 bit register size
    bt_pcacntl1_clock_shift_setf(clk_adj_us & 0x7FF);
    bt_pcacntl1_clock_shift_en_setf(1);
    while(bt_pcacntl1_clock_shift_en_getf());

    #if !(BLE_ISO_HW_PRESENT)
    // Get time again to sample new clockN
    rwip_time_get();
    // restore previous sampled BTS clock value
    rwip_env.last_samp_time.bts = samp_bts;
    #endif // !(BLE_ISO_HW_PRESENT)
}
#endif // (BT_EMB_PRESENT)

uint8_t rwip_sleep(void)
{
    uint8_t sleep_res = RWIP_ACTIVE;

    DBG_SWDIAG(SLEEP, FUNC, 1);

    DBG_SWDIAG(SLEEP, ALGO, 0);

    do
    {
        #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
        int32_t sleep_duration;
        rwip_time_t target;
        #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

        /************************************************************************
         **************            CHECK KERNEL EVENTS             **************
         ************************************************************************/
        // Check if some kernel processing is ongoing (during wakeup, kernel events are not processed)
        if (((rwip_env.prevent_sleep & RW_WAKE_UP_ONGOING) == 0) && !ke_sleep_check())
            break;
            
        //ATINY_LOG(LOG_INFO,"prevent_sleep=0x%x !",rwip_env.prevent_sleep);

        // Processor sleep can be enabled
        sleep_res = RWIP_CPU_SLEEP;

        DBG_SWDIAG(SLEEP, ALGO, 1);

        /************************************************************************
         **************               Check flags                  **************
         ************************************************************************/

        // First check if no pending procedure prevent from going to sleep
        if (rwip_env.prevent_sleep != 0)
            break;

        #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
        DBG_SWDIAG(SLEEP, ALGO, 2);

        /************************************************************************
         **************       Get the soonest timer expiry         **************
         ************************************************************************/

        target.hs = RWIP_INVALID_TARGET_TIME;
        target.hus = 0;

        // check if Common timer is active
        if(rwip_env.timer_co_target.hs != RWIP_INVALID_TARGET_TIME)
        {
            target = rwip_env.timer_co_target;
        }

        // check if alarm timer is active
        if(rwip_env.timer_alarm_target.hs != RWIP_INVALID_TARGET_TIME)
        {
            if((target.hs == RWIP_INVALID_TARGET_TIME) || CLK_GREATER_THAN_HUS(target.hs, target.hus, rwip_env.timer_alarm_target.hs, rwip_env.timer_alarm_target.hus))
            {
                target = rwip_env.timer_alarm_target;
            }
        }

        // check if arbiter timer is active
        if(rwip_env.timer_arb_target.hs != RWIP_INVALID_TARGET_TIME)
        {
            rwip_time_t target_arb;
            uint32_t hs = rwip_env.timer_arb_target.hs;
            int16_t hus = rwip_env.timer_arb_target.hus;

            // Remove margin, as the programming at wake-up is safe from other ISRs
            hs += rwip_prog_delay;
            hus -= ((IP_PREFETCHABORT_TIME_US << 1) + SLEEP_PROG_MARGIN);
            if(hus < 0)
            {
                uint8_t nb_hs = 1 + ((-hus)/HALF_SLOT_SIZE);
                hs -= nb_hs;
                hus += nb_hs * HALF_SLOT_SIZE;
            }
            target_arb.hs = (hs & RWIP_MAX_CLOCK_TIME);
            target_arb.hus = hus;

            if((target.hs == RWIP_INVALID_TARGET_TIME) || CLK_GREATER_THAN_HUS(target.hs, target.hus, target_arb.hs, target_arb.hus))
            {
                target = target_arb;
            }
        }

        DBG_SWDIAG(SLEEP, ALGO, 3);

        /************************************************************************
         **************          Compute sleep duration            **************
         ************************************************************************/
        //ATINY_LOG(LOG_INFO,"target:hs=%d hus=%d!",target.hs, target.hus);

        if(target.hs != RWIP_INVALID_TARGET_TIME)
        {
            // Get current time
            rwip_time_t current_time = rwip_time_get();

            if(CLK_LOWER_EQ_HUS(target.hs, target.hus, current_time.hs, current_time.hus))
                break;

            // Compute sleep duration
            sleep_duration = rwip_slot_2_lpcycles(CLK_DIFF(current_time.hs, target.hs));
            if(target.hus < current_time.hus)
            {
                sleep_duration -= rwip_us_2_lpcycles((current_time.hus - target.hus) >> 1);
            }
            else
            {
                sleep_duration += rwip_us_2_lpcycles((target.hus - current_time.hus) >> 1);
            }

            // Take a margin for clock correction and SW processing at enter sleep / wake-up
            sleep_duration -= (SLEEP_ENTER_FUNC_MARGIN + SLEEP_WAKEUP_FUNC_MARGIN + SLEEP_WAKEUP_CLOCK_CORR_MARGIN);

            // Sleep duration must ensure wake-up delay of the system
            if(sleep_duration < ((int32_t)(rwip_env.lp_cycle_wakeup_delay + 1)))
                break;
                
            //ATINY_LOG(LOG_INFO,"current_time:hs=%d hus=%d,sleep_duration =%d,lp_cycle_wakeup_delay=%d,sleep_dur_max=%d!",
                //current_time.hs, current_time.hus, sleep_duration, rwip_env.lp_cycle_wakeup_delay, rwip_env.sleep_dur_max);

            // Ensure not exceeding the maximum duration
            sleep_duration = co_min(sleep_duration, rwip_env.sleep_dur_max);
            //sleep_duration = rwip_env.sleep_dur_max;
        }
        else
        {
            // Set the maximum sleep duration
            sleep_duration = rwip_env.sleep_dur_max;
        }

        DBG_SWDIAG(SLEEP, ALGO, 4);

        #if (H4TL_SUPPORT)
        /************************************************************************
         **************                 CHECK TL                   **************
         ************************************************************************/
        // Try to switch off TL
        if (!h4tl_stop())
        {
            sleep_res = RWIP_ACTIVE;
            break;
        }
        #endif //H4TL_SUPPORT

        DBG_SWDIAG(SLEEP, FUNC, 0);
#ifdef PM_DEBUG_BLE_DEEP_SLEEP
	    if(0 == ms_sys_check_if_enter_sleep())
        {
            sleep_res = RWIP_CPU_SLEEP;
            break;
        }
#endif
        sleep_res = RWIP_DEEP_SLEEP;

        TRC_REQ_SLEEP();
        
        ATINY_LOG(LOG_INFO,"sleep_res=%d sleep_duration=%d!\r\n",sleep_res,sleep_duration);

        /************************************************************************
         **************          PROGRAM CORE DEEP SLEEP           **************
         ************************************************************************/

        // Program wake-up time
        ip_deepslwkup_set(sleep_duration);

        led_set(2);

        DBG_SWDIAG(SLEEP, SLEEP, 1);

        // Prevent re-entering sleep, until it effectively sleeps and wakes up
        rwip_prevent_sleep_set(RW_DEEP_SLEEP);

        /************************************************************************
         **************               SWITCH OFF RF                **************
         ************************************************************************/
        rwip_rf.sleep();
        #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

    } while(0);

    if(sleep_res != RWIP_DEEP_SLEEP)
    {
        DBG_SWDIAG(SLEEP, FUNC, 0);
    }
    return sleep_res;
}


void rwip_driver_init(uint8_t init_type)
{

    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)

    switch (init_type)
    {
        case RWIP_INIT:
        {
            // Register AES event
            ke_event_callback_set(KE_EVENT_AES_END, &rwip_crypt_evt_handler);

            // ensure that we will never enter in deep sleep
            rwip_prevent_sleep_set(RW_PLF_DEEP_SLEEP_DISABLED);


        }
        break;

        case RWIP_RST:
        {
            // Do nothing
        }
        // No break

        case RWIP_1ST_RST:
        {
            uint8_t length;
            uint8_t sleep_enable = 0;
            uint8_t ext_wakeup_enable;
            #if (BT_DUAL_MODE)
            uint8_t diag_cfg[PARAM_LEN_DIAG_DM_HW];
            #endif // (BT_DUAL_MODE)

            // initialize environment
            rwip_env.prevent_sleep      = 0;
            // clear target timer
            rwip_env.timer_co_target.hs    = RWIP_INVALID_TARGET_TIME;
            rwip_env.timer_alarm_target.hs = RWIP_INVALID_TARGET_TIME;
            rwip_env.timer_arb_target.hs   = RWIP_INVALID_TARGET_TIME;

            // Reset the IP core
            ip_rwdmcntl_master_soft_rst_setf(1);
            while(ip_rwdmcntl_master_soft_rst_getf());

            // Enable default common interrupts
            ip_intcntl1_set(IP_FIFOINTMSK_BIT | IP_CRYPTINTMSK_BIT | IP_SWINTMSK_BIT | IP_SLPINTMSK_BIT);

            // And acknowledge any possible pending ones
            ip_intack1_clear(0xFFFFFFFF);


            #if (BLE_EMB_PRESENT && BT_EMB_PRESENT)
            // Enable BTDM-specific interrupts
            ip_intcntl0_set(IP_ERRORINTMSK_BIT);
            // And acknowledge any possible pending ones
            ip_intack0_clear(0xFFFFFFFF);
            #endif //(BLE_EMB_PRESENT && BT_EMB_PRESENT)

            #if (BT_DUAL_MODE)
            // Read diagport configuration from NVDS
            length = PARAM_LEN_DIAG_DM_HW;
            if(rwip_param.get(PARAM_ID_DIAG_DM_HW, &length, diag_cfg) == PARAM_OK)
            {
                ip_diagcntl_pack(1, diag_cfg[3], 1, diag_cfg[2], 1, diag_cfg[1], 1, diag_cfg[0]);
            }
            else
            {
                ip_diagcntl_set(0);
            }
            #endif // (BT_DUAL_MODE)
#if BLE_SLEEP_NVDS_SUPPORT 
            // Activate deep sleep feature if enabled in NVDS and in reset mode
            length = PARAM_LEN_SLEEP_ENABLE;
            sleep_enable = 1;
            if(rwip_param.set(PARAM_ID_SLEEP_ENABLE, length, &sleep_enable) != PARAM_OK)
            {
                ATINY_LOG(LOG_ERR,"PARAM_ID_SLEEP_ENABLE failed!");
            }
            // Set max sleep duration depending on wake-up mode
            length  = PARAM_LEN_EXT_WAKEUP_ENABLE;
            ext_wakeup_enable = 1;
            if(rwip_param.set(PARAM_ID_EXT_WAKEUP_ENABLE, length, &ext_wakeup_enable) != PARAM_OK)
            {
                ATINY_LOG(LOG_ERR,"PARAM_ID_EXT_WAKEUP_ENABLE failed!");
            }

#endif
            // Activate deep sleep feature if enabled in NVDS and in reset mode
            length = PARAM_LEN_SLEEP_ENABLE;
            if(rwip_param.get(PARAM_ID_SLEEP_ENABLE, &length, &sleep_enable) != PARAM_OK)
            {
                sleep_enable = 0;
            }
            //ATINY_LOG(LOG_INFO,"sleep_enable=%d length=%d!",sleep_enable,length);                        

            // check is sleep is enabled
            if(sleep_enable != 0)
            {
                uint16_t twext, twosc, twrm;
#if BLE_SLEEP_NVDS_SUPPORT                 
                length  = PARAM_LEN_RM_WAKEUP_TIME;
                twrm = SLEEP_OSC_EXT_WAKEUP_DELAY;
                if(rwip_param.set(PARAM_ID_RM_WAKEUP_TIME, length, (uint8_t *)&twrm) != PARAM_OK)
                {
                    ATINY_LOG(LOG_ERR,"PARAM_ID_RM_WAKEUP_TIME failed!");
                }
                length  = PARAM_LEN_OSC_WAKEUP_TIME;
                twosc = SLEEP_OSC_EXT_WAKEUP_DELAY;
                if(rwip_param.set(PARAM_ID_OSC_WAKEUP_TIME, length, (uint8_t *)&twosc) != PARAM_OK)
                {
                    ATINY_LOG(LOG_ERR,"PARAM_ID_OSC_WAKEUP_TIME failed!");
                }    
                length  = PARAM_LEN_EXT_WAKEUP_TIME;
                twext = SLEEP_OSC_EXT_WAKEUP_DELAY;
                if(rwip_param.set(PARAM_ID_EXT_WAKEUP_TIME, length, (uint8_t *)&twext) != PARAM_OK)
                {
                    ATINY_LOG(LOG_ERR,"PARAM_ID_EXT_WAKEUP_TIME failed!");
                }  
#endif

                // Set max sleep duration depending on wake-up mode
                if(rwip_param.get(PARAM_ID_EXT_WAKEUP_ENABLE, &length, &ext_wakeup_enable) != PARAM_OK)
                {
                    ext_wakeup_enable = 0;
                }
                rwip_env.ext_wakeup_enable = (ext_wakeup_enable != 0) ? true : false;

                // Initialize sleep parameters
                rwip_env.sleep_acc_error   = 0;

                // Get TWrm from NVDS
                length = sizeof(uint16_t);
                if (rwip_param.get(PARAM_ID_RM_WAKEUP_TIME, &length, (uint8_t*)&twrm) != PARAM_OK)
                {
                    // Set default values : 625 us
                    twrm = SLEEP_RM_WAKEUP_DELAY;
                }

                // Get TWosc from NVDS
                length = sizeof(uint16_t);
                if (rwip_param.get(PARAM_ID_OSC_WAKEUP_TIME, &length, (uint8_t*)&twosc) != PARAM_OK)
                {
                    // Set default values : 5 ms
                    twosc = SLEEP_OSC_NORMAL_WAKEUP_DELAY;
                }

                // Get TWext from NVDS
                length = sizeof(uint16_t);
                if (rwip_param.get(PARAM_ID_EXT_WAKEUP_TIME, &length, (uint8_t*)&twext) != PARAM_OK)
                {
                    // Set default values : 5 ms
                    twext = SLEEP_OSC_EXT_WAKEUP_DELAY;
                }

                twrm  = rwip_us_2_lpcycles(twrm);
                twosc = rwip_us_2_lpcycles(twosc);
                twext = rwip_us_2_lpcycles(twext);

                // Program register
                ip_enbpreset_pack(twext, twosc, twrm);

                // Configure wake up delay to the highest parameter
                twext = co_max(twext,twrm);
                twext = co_max(twext,twosc);

                // Store wake-up delay in lp cycles
                rwip_env.lp_cycle_wakeup_delay = twext;

                // Store wake-up delay in lp cycles
                //rwip_env.sleep_dur_max = (rwip_env.ext_wakeup_enable) ? MAX_SLEEP_DURATION_EXTERNAL_WAKEUP : MAX_SLEEP_DURATION_PERIODIC_WAKEUP;
                rwip_env.sleep_dur_max = (rwip_env.ext_wakeup_enable) ? TCL_SLEEP_DURATION_TIME: MAX_SLEEP_DURATION_PERIODIC_WAKEUP;
                //ATINY_LOG(LOG_INFO,"ext_wakeup_enable=%d,sleep_dur_max=%d",rwip_env.ext_wakeup_enable,rwip_env.sleep_dur_max);
                // Set the external wakeup parameter
                ip_deepslcntl_extwkupdsb_setf(!rwip_env.ext_wakeup_enable);
            }
            else
            {
                // ensure that we will never enter in deep sleep
                rwip_prevent_sleep_set(RW_PLF_DEEP_SLEEP_DISABLED);
            }

            // Set prefetch time and anticipated prefetch abort time
            //ip_timgencntl_pack((IP_PREFETCHABORT_TIME_US << 1), (IP_PREFETCH_TIME_US << 1));
				ip_timgencntl_pack((IP_PREFETCHABORT_TIME_US << 1), (IP_PREFETCH_TIME_US << 1)+256);

            // Get programming delay from parameters
            length = PARAM_LEN_PROG_DELAY;
            if(rwip_param.get(PARAM_ID_PROG_DELAY, &length, &rwip_prog_delay) != PARAM_OK)
            {
                rwip_prog_delay = IP_PROG_DELAY_DFT;
            }

            // Initialize the sleep clock drift
            length = PARAM_LEN_LPCLK_DRIFT ;
            if (rwip_param.get(PARAM_ID_LPCLK_DRIFT, &length, (uint8_t *) &rwip_env.sleep_clock_drift) != PARAM_OK)
            {
                // If no value is set in NVDS, set maximum value allowed by the standard (BTDM:250ppm / BLE:500ppm)
                #if BT_EMB_PRESENT
                rwip_env.sleep_clock_drift = BT_MAX_DRIFT_SLEEP;
                #else // BT_EMB_PRESENT
                rwip_env.sleep_clock_drift = BLE_MAX_DRIFT_SLEEP;
                #endif // BT_EMB_PRESENT
            }

            // Initialize the active clock drift
            length = PARAM_LEN_ACTCLK_DRIFT ;
            if (rwip_param.get(PARAM_ID_ACTCLK_DRIFT, &length, &rwip_env.active_clock_drift) != PARAM_OK)
            {
                // If no value is set in NVDS, set maximum value allowed by the standard (BTDM:20ppm / BLE:50ppm)
                #if BT_EMB_PRESENT
                rwip_env.active_clock_drift = BT_MAX_DRIFT_ACTIVE;
                #else // BT_EMB_PRESENT
                rwip_env.active_clock_drift = BLE_MAX_DRIFT_ACTIVE;
                #endif // BT_EMB_PRESENT
            }

            // Deduce & save the SCA from the drift
            if (rwip_env.sleep_clock_drift < 21)
                rwip_env.sleep_clock_accuracy = SCA_20PPM;
            else if (rwip_env.sleep_clock_drift < 31)
                rwip_env.sleep_clock_accuracy = SCA_30PPM;
            else if (rwip_env.sleep_clock_drift < 51)
                rwip_env.sleep_clock_accuracy = SCA_50PPM;
            else if (rwip_env.sleep_clock_drift < 76)
                rwip_env.sleep_clock_accuracy = SCA_75PPM;
            else if (rwip_env.sleep_clock_drift < 101)
                rwip_env.sleep_clock_accuracy = SCA_100PPM;
            else if (rwip_env.sleep_clock_drift < 151)
                rwip_env.sleep_clock_accuracy = SCA_150PPM;
            else if (rwip_env.sleep_clock_drift < 251)
                rwip_env.sleep_clock_accuracy = SCA_250PPM;
            else
                rwip_env.sleep_clock_accuracy = SCA_500PPM;

            // Initialize channel assessment enable flag
            length = PARAM_LEN_CH_ASS_EN;
            if (rwip_param.get(PARAM_ID_CH_ASS_EN, &length, (uint8_t *) &rwip_env.ch_ass_en) != PARAM_OK)
            {
                // If no value is set in persistent storage enable the feature by default
                rwip_env.ch_ass_en = true;
            }

        }
        break;

        default:
        {
            // Do nothing
        }
        break;
    }

    #elif (BLE_HOST_PRESENT)
    // initialize environment
    rwip_env.prevent_sleep     = 0;
    rwip_env.timer_co_target.hs = RWIP_INVALID_TARGET_TIME;
    // enable timer
    timer_enable(true);
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)
}

void rwip_prevent_sleep_set(uint32_t prv_slp_bit)
{
    GLOBAL_INT_DISABLE();
    rwip_env.prevent_sleep |= prv_slp_bit;
    GLOBAL_INT_RESTORE();
}

void rwip_prevent_sleep_clear(uint32_t prv_slp_bit)
{
    GLOBAL_INT_DISABLE();
    rwip_env.prevent_sleep &= ~prv_slp_bit;
    GLOBAL_INT_RESTORE();
}


#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
uint16_t rwip_current_drift_get(void)
{
    // Initialize with the active clock drift
    uint16_t current_drift = rwip_env.active_clock_drift;

    do
    {
        #if BT_EMB_PRESENT
        if(rwip_env.prevent_sleep & (RW_CSB_NOT_LPO_ALLOWED | RW_BT_ACTIVE_MODE))
            break;
        #endif // BT_EMB_PRESENT

        #if BLE_EMB_PRESENT
        if(rwip_env.prevent_sleep & (RW_BLE_ACTIVE_MODE))
            break;
        #endif // BLE_EMB_PRESENT

        // If device can sleep, use the sleep clock drift
        current_drift = rwip_env.sleep_clock_drift;

    } while (0);

    return current_drift;
}

uint16_t rwip_max_drift_get(void)
{
    return rwip_env.sleep_clock_drift;
}

uint8_t rwip_sca_get(void)
{
    return rwip_env.sleep_clock_accuracy;
}
#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

void rwip_timer_co_set(uint32_t target_bts)
{
    GLOBAL_INT_DISABLE();

    rwip_time_t target;

    // convert target time to bluetooth time target
    rwip_bts_to_bt_time(target_bts, &(target.hs), &(target.hus));

    // save target time
    rwip_env.timer_co_target = target;

    #if (EMB_PRESENT)

    ASSERT_ERR(target.hs <= RWIP_MAX_CLOCK_TIME);
    ASSERT_ERR(target.hus <= HALF_SLOT_TIME_MAX);

    // set the abs timeout in HW
    ip_clkntgt3_set(target.hs);
    ip_hmicrosectgt3_set(HALF_SLOT_TIME_MAX - target.hus);

    // if timer is not enabled, it is possible that the irq is raised
    // due to a spurious value, so ack it before
    ip_intack1_timestamptgt3intack_clearf(1);
    ip_intcntl1_timestamptgt3intmsk_setf(1);

    #else // (HOST_PRESENT)
    timer_set_timeout(target_bts, rwip_timer_co_handler);
    #endif // (EMB_PRESENT)

    GLOBAL_INT_RESTORE();
}

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
void rwip_timer_alarm_set(uint32_t target, uint32_t half_us_delay)
{
    // save target time
    rwip_env.timer_alarm_target.hs = target;
    rwip_env.timer_alarm_target.hus = half_us_delay;

    if (target != RWIP_INVALID_TARGET_TIME)
    {
        ASSERT_INFO(half_us_delay < HALF_SLOT_SIZE, half_us_delay, 0);

        // set the abs timeout in HW
        ip_clkntgt2_setf(target);
        ip_hmicrosectgt2_setf(HALF_SLOT_TIME_MAX - half_us_delay);

        // if timer is not enabled, it is possible that the irq is raised
        // due to a spurious value, so ack it before
        ip_intack1_timestamptgt2intack_clearf(1);
        ip_intcntl1_timestamptgt2intmsk_setf(1);
    }
    else
    {
        // disable timer irq
        ip_intcntl1_timestamptgt2intmsk_setf(0);
    }
}

void rwip_timer_arb_set(uint32_t target, uint32_t half_us_delay)
{
    // save target time
    rwip_env.timer_arb_target.hs = target;
    rwip_env.timer_arb_target.hus = half_us_delay;

    if (target != RWIP_INVALID_TARGET_TIME)
    {
        ASSERT_INFO(half_us_delay < HALF_SLOT_SIZE, half_us_delay, 0);

        // set the abs timeout in HW
        ip_clkntgt1_setf(target);
        ip_hmicrosectgt1_setf(HALF_SLOT_TIME_MAX - half_us_delay);

        // if timer is not enabled, it is possible that the irq is raised
        // due to a spurious value, so ack it before
        ip_intack1_timestamptgt1intack_clearf(1);
        ip_intcntl1_timestamptgt1intmsk_setf(1);
    }
    else
    {
        // disable timer irq
        ip_intcntl1_timestamptgt1intmsk_setf(0);
    }
}

void rwip_aes(const uint8_t *key, const uint8_t* val, bool cipher)
{
    // Prevent going to deep sleep during encryption
    rwip_prevent_sleep_set(RW_CRYPT_ONGOING);

    // Copy data to EM buffer
    em_wr(val, EM_ENC_IN_OFFSET, KEY_LEN);
    GLOBAL_INT_DISABLE();

    // copy the key in the register dedicated for the encryption
    ip_aeskey31_0_set(  co_read32p(&(key[0])));
    ip_aeskey63_32_set( co_read32p(&(key[4])));
    ip_aeskey95_64_set( co_read32p(&(key[8])));
    ip_aeskey127_96_set(co_read32p(&(key[12])));

    // Set the pointer on the data to encrypt.
    ip_aesptr_setf(EM_ENC_IN_OFFSET >> 2);

    // enable crypt interrupt (and clear a previous interrupt if needed)
    ip_intack1_cryptintack_clearf(1);
    ip_intcntl1_cryptintmsk_setf(1);

    // Start encryption
    ip_aescntl_pack(/*aesmode*/cipher ? 0 : 1, /*aesstart*/1);

    GLOBAL_INT_RESTORE();
}

void rwip_sw_int_req(void)
{
    // enable SW interrupt (and clear a previous interrupt if needed)
    ip_intack1_swintack_clearf(1);
    ip_intcntl1_swintmsk_setf(1);
    // start the SW interrupt
    ip_rwdmcntl_swint_req_setf(1);
}



#if 1

#include "reg_em_ble_cs.h"
#include "em_map.h"

//add by liujin for rwip irq cnt1
void RWIP_IRQHandler(void)
{
	 
    DBG_SWDIAG(ISR, RWIP, 1);

    // Check interrupt status and call the appropriate handlers
    uint32_t irq_stat      = ip_intstat1_get();
   // uint32_t irq_stat0      = ip_intstat0_get();
	 // g_irq_stat = irq_stat;
   //  MS_LOGI(MS_DRIVER,"irq_stat:%x,error:%x\r\n", irq_stat,irq_stat0);

    // General purpose timer interrupt - half us accuracy
    if (irq_stat & IP_TIMESTAMPTGT1INTSTAT_BIT)
    {
        DBG_SWDIAG(IP_ISR, ARB_TIMER, 1);
        DBG_SWDIAG(BI, START, 1);
		  
        // Clear the interrupt
        ip_intack1_timestamptgt1intack_clearf(1);

        // handles arbiter timer target
        rwip_timer_arb_handler();

        DBG_SWDIAG(BI, START, 0);
        DBG_SWDIAG(IP_ISR, ARB_TIMER, 0);
    }

    // General purpose timer interrupt - half us accuracy
    if (irq_stat & IP_TIMESTAMPTGT2INTSTAT_BIT)
    {
        DBG_SWDIAG(IP_ISR, ALARM_TIMER, 1);

        // Clear the interrupt
        ip_intack1_timestamptgt2intack_clearf(1);

        // handles alarm timer target
        rwip_timer_alarm_handler();

        DBG_SWDIAG(IP_ISR, ALARM_TIMER, 0);
    }

    // Clock
    if (irq_stat & IP_CLKNINTSTAT_BIT) // clock interrupt
    {
        DBG_SWDIAG(IP_ISR, CLKNINT, 1);

        // Ack clock interrupt
        ip_intack1_clknintack_clearf(1);

        if(rwip_env.prevent_sleep & RW_WAKE_UP_ONGOING)
        {
            // Handle end of wake-up
            rwip_wakeup_end();
        }
        #if (BT_EMB_PRESENT)
        else // BT uses clock IRQ to program ACL frames
        {
            // Call Scheduling Programmer
            sch_prog_clk_isr();
        }
        #endif //BT_EMB_PRESENT

        DBG_SWDIAG(IP_ISR, CLKNINT, 0);
    }

    // General purpose timer interrupt - half us accuracy
    if (irq_stat & IP_TIMESTAMPTGT3INTSTAT_BIT)
    {
        DBG_SWDIAG(IP_ISR, CO_TIMER, 1);
        // Clear the interrupt
        ip_intack1_timestamptgt3intack_clearf(1);

        // handles common timer target
        rwip_timer_co_handler();

        DBG_SWDIAG(IP_ISR, CO_TIMER, 0);
    }

    // SW interrupt
    if (irq_stat & IP_SWINTSTAT_BIT)
    {
        DBG_SWDIAG(IP_ISR, SWINT, 1);
        // Clear the interrupt
        ip_intack1_swintack_clearf(1);

        // call SW interrupt handler
        rwip_sw_int_handler();

        DBG_SWDIAG(IP_ISR, SWINT, 0);
    }

    // FIFO
    if (irq_stat & IP_FIFOINTSTAT_BIT) // FIFO interrupt
    {
        DBG_SWDIAG(IP_ISR, FIFOINT, 1);

        // Call scheduling programmer
        sch_prog_fifo_isr();

        // Ack FIFO interrupt
        ip_intack1_fifointack_clearf(1);

        DBG_SWDIAG(IP_ISR, FIFOINT, 0);
    }

    if (irq_stat & IP_SLPINTSTAT_BIT)
    {
        DBG_SWDIAG(IP_ISR, SLPINT, 1);

        // ack Sleep wakeup interrupt
        ip_intack1_slpintack_clearf(1);

        // Handle wake-up
        rwip_wakeup();

        DBG_SWDIAG(IP_ISR, SLPINT, 0);
    }

    // Encryption interrupt
    if (irq_stat & IP_CRYPTINTSTAT_BIT)
    {
        DBG_SWDIAG(IP_ISR, CRYPTINT, 1);

        ip_intack1_cryptintack_clearf(1);

        // call the crypto ISR handler
        rwip_crypt_isr_handler();

        DBG_SWDIAG(IP_ISR, CRYPTINT, 0);
    }

    // General purpose timer interrupt - half slot accuracy
    if (irq_stat & IP_FINETGTINTSTAT_BIT)
    {
        // Clear the interrupt
        ip_intack1_finetgtintack_clearf(1);

        // Finetgt unused
    }

    DBG_SWDIAG(ISR, RWIP, 0);
}
#endif



#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

#if (BLE_EMB_PRESENT && BT_EMB_PRESENT)
void rwip_btdm_isr(void)
{
    DBG_SWDIAG(ISR, RWIP, 1);

    // Check BTDN interrupt status and call the appropriate handlers
    uint32_t irq_stat      = ip_intstat0_get();

    // Error interrupt
    if (irq_stat & IP_ERRORINTSTAT_BIT)
    {
        // Clear the interrupt
        ip_intack0_errorintack_clearf(1);

        ASSERT_INFO(0, ip_errortypestat_get(), 0);
    }

    DBG_SWDIAG(ISR, RWIP, 0);
}
#endif //(BLE_EMB_PRESENT && BT_EMB_PRESENT)


uint32_t rwip_bt_time_to_bts(uint32_t hs, uint16_t hus)
{
    // compute time in half microseconds between last sampled time and BT time provided in parameters
    int32_t clk_diff_hus = CLK_DIFF(rwip_env.last_samp_time.hs, hs) * HALF_SLOT_SIZE;
    clk_diff_hus        += (int32_t)hus - (int32_t)rwip_env.last_samp_time.hus;

    // compute BTS time in microseconds
    return (rwip_env.last_samp_time.bts + (clk_diff_hus >> 1));
}


void rwip_bts_to_bt_time(uint32_t bts, uint32_t* p_hs, uint16_t* p_hus)
{
    uint32_t clk_diff_hus;
    int32_t clk_diff_us = (int32_t) (bts - rwip_env.last_samp_time.bts);

    *p_hus = rwip_env.last_samp_time.hus;
    *p_hs  = rwip_env.last_samp_time.hs;

    // check if clock difference is positive or negative
    if(clk_diff_us > 0)
    {
        clk_diff_hus = (clk_diff_us << 1);
        *p_hus += CO_MOD(clk_diff_hus, HALF_SLOT_SIZE);
        *p_hs  += (clk_diff_hus / HALF_SLOT_SIZE);

        if(*p_hus >= HALF_SLOT_SIZE)
        {
            *p_hus -= HALF_SLOT_SIZE;
            *p_hs  += 1;
        }
    }
    else
    {
        uint32_t diff_hus;
        clk_diff_hus = (rwip_env.last_samp_time.bts - bts) << 1;
        diff_hus = CO_MOD(clk_diff_hus, HALF_SLOT_SIZE);

        if(*p_hus < diff_hus)
        {
            *p_hus += HALF_SLOT_SIZE;
            *p_hs  -= 1;
        }

        *p_hus -= diff_hus;
        *p_hs  -= (clk_diff_hus / HALF_SLOT_SIZE);
    }

    // wrap clock
    *p_hs = (*p_hs & RWIP_MAX_CLOCK_TIME);
}

#if (BLE_EMB_PRESENT)
void rwip_channel_assess_ble(uint8_t channel, uint8_t rx_status, uint8_t rssi_cs, uint32_t timestamp, bool connected)
{
    if(channel < DATA_CHANNEL_NB)
    {
        // Increment/decrement counter
        if(rx_status == RWIP_RX_OK)
        {
            if(rwip_env.ch_assess_ble.level[channel] < BLE_CH_ASSESS_COUNT_MAX)
            {
                rwip_env.ch_assess_ble.level[channel]++;
            }
        }
        else if(rwip_rf.rssi_convert(rssi_cs) > rwip_rf.rssi_interf_thr)
        {
            if(rwip_env.ch_assess_ble.level[channel] > BLE_CH_ASSESS_COUNT_MIN)
            {
                rwip_env.ch_assess_ble.level[channel]--;
            }
        }

        // Store timestamp
        rwip_env.ch_assess_ble.timestamp[channel] = timestamp;

        // Share information with BT if present
        #if (BT_EMB_PRESENT)
        channel = 2*ble_ch_to_phy_idx[channel];

        // Update two BT channels
        for (uint8_t j = 0; j < 2; j++)
        {
            channel += j;

            // Increment/decrement counter
            if(rx_status == RWIP_RX_OK)
            {
                if(rwip_env.ch_assess_bt.level[channel] < BT_AFH_ASSESS_COUNT_MAX)
                {
                    rwip_env.ch_assess_bt.level[channel]++;
                }
            }
            else if(rwip_rf.rssi_convert(rssi_cs) > rwip_rf.rssi_interf_thr)
            {
                if(rwip_env.ch_assess_bt.level[channel] > BT_AFH_ASSESS_COUNT_MIN)
                {
                    rwip_env.ch_assess_bt.level[channel]--;
                }
            }

            // Store timestamp
            rwip_env.ch_assess_bt.timestamp[channel] = timestamp;
        }
        #endif //BT_EMB_PRESENT
    }
    else
    {
        ASSERT_ERR(0);
    }
}

struct rwip_ch_assess_data_ble* rwip_ch_assess_data_ble_get(void)
{
    return &rwip_env.ch_assess_ble;
}

bool rwip_update_ch_map_with_ch_assess_ble(uint8_t ch_idx, uint8_t* ch_map)
{
    struct rwip_ch_assess_data_ble* ch_assess = rwip_ch_assess_data_ble_get();
    uint32_t clock = lld_read_clock();
    uint8_t bit_pos = ch_idx & 0x7;

    // Check if channel assessment is used to build channel map
    if(rwip_ch_ass_en_get())
    {
        // Channel assessment is considered only if a packet has been received recently
        if(CLK_SUB(clock, ch_assess->timestamp[ch_idx]) < 2*BLE_CH_ASSESS_VALID_TO)
        {
            // Controller assesses the channel as bad
            if (ch_assess->level[ch_idx] <= BLE_CH_ASSESS_COUNT_THR_BAD)
            {
                *ch_map &= ~(1 << bit_pos);
                return true;
            }
        }
        else
        {
            // Clear assessment level
            ch_assess->level[ch_idx] = 0;

            // Re-activate a channel in case it has not been assessed from a long time
            if(CLK_SUB(clock, ch_assess->timestamp[ch_idx]) > 2*BLE_CH_REASSESS_TO)
            {
                *ch_map |= (1 << bit_pos);
                return true;
            }
        }
    }
    return false;
}
#endif //BLE_EMB_PRESENT

#if (BT_EMB_PRESENT)
void rwip_channel_assess_bt(uint8_t channel, uint8_t rx_status, uint8_t rssi_cs, uint32_t timestamp)
{
    if(channel < AFH_NB_CHANNEL_MAX)
    {
        // Increment/decrement counter
        if(rx_status == RWIP_RX_OK)
        {
            if(rwip_env.ch_assess_bt.level[channel] < BT_AFH_ASSESS_COUNT_MAX)
            {
                rwip_env.ch_assess_bt.level[channel]++;
            }
        }
        else if(rwip_rf.rssi_convert(rssi_cs) > rwip_rf.rssi_interf_thr)
        {
            if(rwip_env.ch_assess_bt.level[channel] > BT_AFH_ASSESS_COUNT_MIN)
            {
                rwip_env.ch_assess_bt.level[channel]--;
            }
        }

        // Store timestamp
        rwip_env.ch_assess_bt.timestamp[channel] = timestamp;

        // Share information with BLE if present
        #if (BLE_EMB_PRESENT)
        channel = ble_phy_to_ch_idx[channel/2];

        // Update one BLE channel
        if(channel < DATA_CHANNEL_NB)
        {
            // Increment/decrement counter
            if(rx_status == RWIP_RX_OK)
            {
                if(rwip_env.ch_assess_ble.level[channel] < BLE_CH_ASSESS_COUNT_MAX)
                {
                    rwip_env.ch_assess_ble.level[channel]++;
                }
            }
            else if(rwip_rf.rssi_convert(rssi_cs) > rwip_rf.rssi_interf_thr)
            {
                if(rwip_env.ch_assess_ble.level[channel] > BLE_CH_ASSESS_COUNT_MIN)
                {
                    rwip_env.ch_assess_ble.level[channel]--;
                }
            }

            // Store timestamp
            rwip_env.ch_assess_ble.timestamp[channel] = timestamp;
        }
        #endif //BLE_EMB_PRESENT
    }
    else
    {
        ASSERT_ERR(0);
    }
}

struct rwip_ch_assess_data_bt* rwip_ch_assess_data_bt_get(void)
{
    return &rwip_env.ch_assess_bt;
}

bool rwip_update_ch_map_with_ch_assess_bt(uint8_t ch_idx, struct chnl_map* peer_ch_class,
                                          uint8_t* ch_map)
{
    // Get local channel assessment
    struct rwip_ch_assess_data_bt* ch_assess = rwip_ch_assess_data_bt_get();
    uint32_t clock = ld_read_clock();
    int j = 0;

    uint8_t bit_pos = ch_idx & 0x7;
    uint8_t byte_idx = ch_idx >> 3;

    // Check local assessment if enabled
    if(rwip_ch_ass_en_get())
    {
        // Channel assessment is considered only if a packet has been received recently
        if(CLK_SUB(clock, ch_assess->timestamp[ch_idx]) < 2*BT_AFH_ASSESS_VALID_TO)
        {
            // Controller assesses the channel as bad
            if (ch_assess->level[ch_idx] <= BT_AFH_ASSESS_COUNT_THR_BAD)
            {
                *ch_map &= ~(1 << bit_pos);
                return true;
            }
        }
        else
        {
            // Clear assessment level
            ch_assess->level[ch_idx] = 0;
        }
    }


    // Check channel classification from the peer devices
    for(j = 0 ; j < MAX_NB_ACTIVE_ACL ; j++)
    {
        if (((peer_ch_class[j].map[byte_idx] >> (bit_pos & 0x06)) & 0x3) == AFH_CH_CLASS_BAD)
        {
            *ch_map &= ~(1 << bit_pos);
            break;
        }
    }

    // Re-activate a channel in case it has not been assessed from a long time
    // Only if the channel has not been explicitly indicated as bad from one slave classification
    if(j == MAX_NB_ACTIVE_ACL)
    {
        if(CLK_SUB(clock, ch_assess->timestamp[ch_idx]) > 2*BT_AFH_REASSESS_TO)
        {
            *ch_map |= (1 << bit_pos);
            return true;
        }
    }

    return false;
}

#endif //BT_EMB_PRESENT

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
void rwip_ch_ass_en_set(bool ch_ass_en)
{
    rwip_env.ch_ass_en = ch_ass_en;
}

bool rwip_ch_ass_en_get(void)
{
    return rwip_env.ch_ass_en;
}

#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)
///@} RW
