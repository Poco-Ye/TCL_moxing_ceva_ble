/**
 ****************************************************************************************
 * @file co_time.c
 *
 * @brief Common Time Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup CO_TIME
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "co_time.h"      // Common time module
#include "arch.h"         // for defines
#include "rwip.h"         // To get system time
#include "co_utils.h"     // to get clock macros
#include "ke_event.h"     // kernel event used to handle time in a background task


/*
 * MACROS
 ****************************************************************************************
 */

/// Convert time in milliseconds to microseconds
#define CO_TIME_MS_TO_US(time) ((time) * (CO_TIME_1_MS_TO_US))

/*
 * DEFINES
 ****************************************************************************************
 */


/// 1 Milliseconds in half microseconds
#define CO_TIME_1_MS_TO_US              (1000)

/// Minimum delay 1 ms
#define CO_TIME_DELAY_MIN                (1)
/// Minimum period 8388608 ms ~= 2,3 hours
#define CO_TIME_PERIOD_MAX               (0x7FFFFF)
/// Maximum delay to reprogram system timer (15 min in microseconds)
#define CO_TIME_SYS_TIMER_DELAY_US_MAX   (900000000)
/// Maximum delay to reprogram system timer (15 min in milliseconds)
#define CO_TIME_SYS_TIMER_DELAY_MS_MAX   (900000)



/// Timer structure bit field description
enum co_time_timer_bf
{
    /// Expiration time [32-39] part (in milliseconds)
    CO_TIME_EXP_TIME_MS_MSB_LSB                 = 0,
    CO_TIME_EXP_TIME_MS_MSB_MASK                = 0x000000FF,
    /// Timer period (in milliseconds)
    CO_TIME_PERIOD_LSB                          = 8,
    CO_TIME_PERIOD_MASK                         = 0x7fffff00,
    /// Timer programmed
    CO_TIME_PROG_POS                            = 31,
    CO_TIME_PROG_BIT                            = 0x80000000,
};


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Timer environment structure
typedef struct co_time_env_
{
    /// First timer in the list sorted according to elapse time.
    co_time_timer_t*    p_first;

    /// Last sampled Bluetooth timestamp system time (microseconds counter)
    uint32_t            last_samp_bts_time;
    /// Last sampled remaining microseconds (between 0 and 1000)
    uint16_t            last_samp_us_time;
    /// Last sampled time in milliseconds
    co_time_t           last_samp_time;
} co_time_env_t;

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Time environment structure
__STATIC co_time_env_t co_time_env;

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Update of System timer
 *
 * Here it is considered that device time has been already sampled
 ****************************************************************************************
 */
__STATIC void co_time_sys_timer_update(void)
{
    DBG_FUNC_ENTER(co_time_sys_timer_update);
    // get current time - consider it has been already sampled
    co_time_t time            = co_time_env.last_samp_time;
    uint32_t  delay_us        = CO_TIME_SYS_TIMER_DELAY_US_MAX;
    co_time_timer_t* p_timer  = co_time_env.p_first;

    // Check if a timer is present
    if(p_timer != NULL)
    {
        uint32_t delay_ms_lsb = p_timer->exp_time_ms_lsb;
        uint8_t  delay_ms_msb = GETF(p_timer->timer_bf, CO_TIME_EXP_TIME_MS_MSB);

        // check if timer is in the past looking at MSB
        if(delay_ms_msb < time.ms_msb)
        {
            delay_us = 0;
        }
        else
        {
            // update timer MSB
            delay_ms_msb -= time.ms_msb;

            // check if timer is in the past looking at LSB
            if((delay_ms_msb == 0) && (delay_ms_lsb < time.ms_lsb))
            {
                delay_us = 0;
            }
            else
            {
                // check if LSB part wrap
                if(delay_ms_lsb < time.ms_lsb)
                {
                    delay_ms_msb -= 1;
                }

                // update delay LSB part
                delay_ms_lsb -= time.ms_lsb;

                // check if next timer delay is less that max system timer delay
                if((delay_ms_msb == 0) && (delay_ms_lsb < CO_TIME_SYS_TIMER_DELAY_MS_MAX))
                {
                    delay_us = CO_TIME_MS_TO_US(delay_ms_lsb);
                }
            }
        }
    }

    // if delay is one milliseconds or less, consider to trigger it now
    if(delay_us <= CO_TIME_1_MS_TO_US)
    {
        ke_event_set(KE_EVENT_TIMER);
    }
    // compute when next timer should be programmed
    else
    {
        uint32_t target_bts = co_time_env.last_samp_bts_time + delay_us;

        rwip_timer_co_set(target_bts);
    }
    DBG_FUNC_EXIT(co_time_sys_timer_update);
}

/**
 ****************************************************************************************
 * @brief Extract timer form timer queue
 *
 * @param[in] p_timer       Pointer to the timer structure.
 *
 * @return True if system timer must be updated, false otherwise
 ****************************************************************************************
 */
__STATIC bool co_time_timer_extract(co_time_timer_t* p_timer)
{
    bool update_sys_timer = false;
    SETB(p_timer->timer_bf, CO_TIME_PROG, false);

    co_time_timer_t* p_prev_timer = NULL;
    co_time_timer_t* p_cur_timer  = co_time_env.p_first;

    // find timer into timer list
    while(p_cur_timer != NULL)
    {
        // timer found
        if(p_cur_timer == p_timer)
        {
            // timer at begin of the list
            if(p_prev_timer == NULL)
            {
                co_time_env.p_first = p_cur_timer->p_next;
                update_sys_timer   = true;
            }
            // or into the list
            else
            {
                p_prev_timer->p_next = p_cur_timer->p_next;
            }

            break;
        }

        p_prev_timer = p_cur_timer;
        p_cur_timer  = p_cur_timer->p_next;
    }

    p_timer->p_next = NULL;

    return (update_sys_timer);
}


/**
 ****************************************************************************************
 * @brief Insert timer into timer queue
 *
 * @param[in] p_timer  Pointer to the timer structure.
 * @param[in] reprog   True: if it's a timer re-program, False otherwise
 *
 * @return True if system timer must be updated, false otherwise
 ****************************************************************************************
 */
__STATIC bool co_time_timer_insert(co_time_timer_t* p_timer)
{
    bool update_sys_timer = false;
    co_time_timer_t* p_prev_timer = NULL;
    co_time_timer_t* p_cur_timer;

    // check if timer must be first extract from timer list.
    if(GETB(p_timer->timer_bf, CO_TIME_PROG))
    {
        update_sys_timer = co_time_timer_extract(p_timer);
    }

    SETB(p_timer->timer_bf, CO_TIME_PROG, true);

    p_cur_timer  = co_time_env.p_first;

    // find timer into timer list
    while(p_cur_timer != NULL)
    {
        // check if timer to insert elapse before current timer
        //    New elapse time MSB part < Current elapse time MSB part
        if(   GETF(p_timer->timer_bf, CO_TIME_EXP_TIME_MS_MSB) < GETF(p_cur_timer->timer_bf, CO_TIME_EXP_TIME_MS_MSB)
           // MSB part equals and LSB part < Current elapse time LSB part
           || (   (GETF(p_timer->timer_bf, CO_TIME_EXP_TIME_MS_MSB) == GETF(p_cur_timer->timer_bf, CO_TIME_EXP_TIME_MS_MSB)
               && (p_timer->exp_time_ms_lsb < p_cur_timer->exp_time_ms_lsb))))
        {
            // insertion point found
            break;
        }

        p_prev_timer = p_cur_timer;
        p_cur_timer  = p_cur_timer->p_next;
    }


    // timer to insert at beginning of the list
    if(p_prev_timer == NULL)
    {
        co_time_env.p_first = p_timer;
        update_sys_timer   = true;
    }
    // or into the list (before current)
    else
    {
        p_prev_timer->p_next = p_timer;
    }

    p_timer->p_next = p_cur_timer;

    return (update_sys_timer);
}

/**
 ****************************************************************************************
 * @brief Program a timer
 *
 * Here it is considered that device time has been already sampled
 *
 * @param[in] p_timer       Pointer to the timer structure.
 * @param[in] delay_ms_lsb  Duration before expiration of the timer [0-31] part (in milliseconds)
 * @param[in] delay_ms_msb  Duration before expiration of the timer [32-39] part (in milliseconds)
 ****************************************************************************************
 */
__STATIC void co_time_timer_prog(co_time_timer_t* p_timer, uint32_t delay_ms_lsb, uint8_t delay_ms_msb)
{
   // get current time
    co_time_t exp_time = co_time_get();

    // detect milliseconds LSB wrap
    if((exp_time.ms_lsb + delay_ms_lsb) < exp_time.ms_lsb)
    {
        // to increase milliseconds MSB part
        exp_time.ms_msb += 1;
    }

    // update milliseconds LSB part
    exp_time.ms_lsb += delay_ms_lsb;

    // update milliseconds MSB part
    exp_time.ms_msb += delay_ms_msb;

    p_timer->exp_time_ms_lsb = exp_time.ms_lsb;
    SETF(p_timer->timer_bf, CO_TIME_EXP_TIME_MS_MSB, exp_time.ms_msb);

    // program timer
    if(co_time_timer_insert(p_timer))
    {
        // update system timer
        co_time_sys_timer_update();
    }
}

/**
 ****************************************************************************************
 * @brief Handle RWIP timer trigger.
 ****************************************************************************************
 */
__STATIC void co_timer_handler(void)
{

    // Clear event
    ke_event_clear(KE_EVENT_TIMER);

    // sample time
    co_time_t cur_time = co_time_get();

    while(co_time_env.p_first != NULL)
    {
        // pick first timer
        co_time_timer_t* p_timer  = co_time_env.p_first;

        // Check if a timer is present and if it has expired
        if(   (GETF(p_timer->timer_bf, CO_TIME_EXP_TIME_MS_MSB) < cur_time.ms_msb)
              // LSB part comparison
           || (   (GETF(p_timer->timer_bf, CO_TIME_EXP_TIME_MS_MSB) == cur_time.ms_msb)
               && (p_timer->exp_time_ms_lsb <= cur_time.ms_lsb)))
        {
            uint32_t period_ms = GETF(p_timer->timer_bf, CO_TIME_PERIOD);

            // remove timer from the list
            co_time_env.p_first = p_timer->p_next;

            // mark timer un-programmed
            SETB(p_timer->timer_bf, CO_TIME_PROG, false);

            // check if timer is a periodic timer, if yes reprogram it
            if(period_ms > 0)
            {
                // detect milliseconds LSB wrap
                if((p_timer->exp_time_ms_lsb + period_ms) < p_timer->exp_time_ms_lsb)
                {
                    // to increase milliseconds MSB part
                    SETF(p_timer->timer_bf, CO_TIME_EXP_TIME_MS_MSB, GETF(p_timer->timer_bf, CO_TIME_EXP_TIME_MS_MSB) + 1);
                }

                // update milliseconds LSB part
                p_timer->exp_time_ms_lsb += period_ms;

                // reprogram timer
                co_time_timer_insert(p_timer);
            }
            else
            {
                p_timer->p_next = NULL;
            }

            // execute timer callback
            if(p_timer->cb != NULL)
            {
                p_timer->cb(p_timer->p_env);
            }
        }
        else
        {
            // stop loop
            break;
        }

        // sample time for next loop
        cur_time = co_time_get();
    }

    // Update system timer for clock update and possibly new timer trigger
    co_time_sys_timer_update();
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

void co_time_init(uint8_t init_type)
{
    switch (init_type)
    {
        case RWIP_INIT:
        {
            // Initialize environment variable
            co_time_env.p_first               = NULL;
            co_time_env.last_samp_time.ms_lsb = 0;
            co_time_env.last_samp_time.ms_msb = 0;
            co_time_env.last_samp_us_time     = 0;
            co_time_env.last_samp_bts_time    = rwip_time_get().bts;

            // Register System timer handler
            ke_event_callback_set(KE_EVENT_TIMER, &co_timer_handler);

            // program next timer
            co_time_sys_timer_update();
        }
        break;

        case RWIP_1ST_RST:
        case RWIP_RST:
        {
            // Clear all timers
            co_time_env.p_first = NULL;
            // update HW timer
            co_time_get();
            // program next timer
            co_time_sys_timer_update();
        }
        break;

        default: { /* Do nothing */ } break;
    }
}

/*
 ****************************************************************************************
 * Time and timer functions
 ****************************************************************************************
 */

co_time_t co_time_get(void)
{
    uint32_t cur_bts_time = rwip_time_get().bts;
    uint32_t clk_diff_ms;

    // compute time in half microseconds between last sampled time and BT time provided in parameters
    // update of the time must be done at least every 15 minutes to ensure that this value is always positive
    uint32_t clk_diff_us = cur_bts_time - co_time_env.last_samp_bts_time;

    // half microseconds value must be positive
    ASSERT_ERR(clk_diff_us < 0x80000000);

    clk_diff_ms          = clk_diff_us / CO_TIME_1_MS_TO_US;
    clk_diff_us         -= clk_diff_ms * CO_TIME_1_MS_TO_US;

    // update half us counter
    co_time_env.last_samp_us_time += clk_diff_us;

    // check if half us counter must wrap
    if(co_time_env.last_samp_us_time >= CO_TIME_1_MS_TO_US)
    {
        co_time_env.last_samp_us_time -= CO_TIME_1_MS_TO_US;
        clk_diff_ms                    += 1;
    }

    // detect milliseconds counter LSB wrap
    if((co_time_env.last_samp_time.ms_lsb + clk_diff_ms) < co_time_env.last_samp_time.ms_lsb)
    {
        // to increase milliseconds counter MSB part
        co_time_env.last_samp_time.ms_msb += 1;
    }

    // update milliseconds counter LSB part
    co_time_env.last_samp_time.ms_lsb += clk_diff_ms;

    // update last sampled value
    co_time_env.last_samp_bts_time = cur_bts_time;

    // return current time
    return co_time_env.last_samp_time;
}

void co_time_compensate(uint32_t delta_time_ms_lsb, uint8_t delta_time_ms_msb)
{
    // detect milliseconds counter LSB wrap
    if((co_time_env.last_samp_time.ms_lsb + delta_time_ms_lsb) < co_time_env.last_samp_time.ms_lsb)
    {
        // to increase milliseconds counter MSB part
        co_time_env.last_samp_time.ms_msb += 1;
    }

    // update milliseconds counter LSB part
    co_time_env.last_samp_time.ms_lsb += delta_time_ms_lsb;

    // update milliseconds counter MSB part
    co_time_env.last_samp_time.ms_msb += delta_time_ms_msb;

    // check if some timer must be updated.
    ke_event_set(KE_EVENT_TIMER);
}

void co_time_timer_init(co_time_timer_t* p_timer, co_time_timer_cb cb, void* p_env)
{
    // initialize callback and environment
    p_timer->p_next   = NULL;
    p_timer->cb       = cb;
    p_timer->p_env    = p_env;
    p_timer->timer_bf = 0;
}

void co_time_timer_set(co_time_timer_t* p_timer, uint32_t delay_ms)
{
    DBG_FUNC_ENTER(co_time_timer_set);
    // Disable period
    SETF(p_timer->timer_bf, CO_TIME_PERIOD, 0);

    if (delay_ms < CO_TIME_DELAY_MIN)
    {
        delay_ms = CO_TIME_DELAY_MIN;
    }

    // program timer
    co_time_timer_prog(p_timer, delay_ms, 0);
    DBG_FUNC_EXIT(co_time_timer_set);
}

void co_time_timer_long_set(co_time_timer_t* p_timer, uint32_t delay_ms_lsb, uint8_t delay_ms_msb)
{
    // Disable period
    SETF(p_timer->timer_bf, CO_TIME_PERIOD, 0);

    if ((delay_ms_msb == 0) && (delay_ms_lsb < CO_TIME_DELAY_MIN))
    {
        delay_ms_lsb = CO_TIME_DELAY_MIN;
    }

    // program timer
    co_time_timer_prog(p_timer, delay_ms_lsb, delay_ms_msb);
}

void co_time_timer_periodic_set(co_time_timer_t* p_timer, uint32_t period_ms)
{
    // put period in supported range
    if(period_ms > CO_TIME_PERIOD_MAX)
    {
        period_ms = CO_TIME_PERIOD_MAX;
    }
    else if (period_ms < CO_TIME_DELAY_MIN)
    {
        period_ms = CO_TIME_DELAY_MIN;
    }

    // set period value
    SETF(p_timer->timer_bf, CO_TIME_PERIOD, period_ms);

    // program timer
    co_time_timer_prog(p_timer, period_ms, 0);
}

void co_time_timer_stop(co_time_timer_t* p_timer)
{
    // check if timer is present in
    if(GETB(p_timer->timer_bf, CO_TIME_PROG))
    {
        // extract timer
        if(co_time_timer_extract(p_timer))
        {
            // if system timer must be updated, sample time and update system timer
            co_time_get();
            co_time_sys_timer_update();
        }
    }
}


/// @} CO_TIME

