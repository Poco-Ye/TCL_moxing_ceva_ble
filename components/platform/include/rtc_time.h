/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  uart.h
 * @brief
 * @author che.jiang
 * @date 2021年12月24日
 * @version 1.0
 * @Revision
 */
#ifndef RTC_TIME_H_
#define RTC_TIME_H_

#include "ms1008.h"

/*
 * RTC time
 */
typedef struct
{
    uint8_t sec;         /* DEC format:value range from 0 to 59, BCD format:value range from 0x00 to 0x59 */
    uint8_t min;         /* DEC format:value range from 0 to 59, BCD format:value range from 0x00 to 0x59 */
    uint8_t hr;          /* DEC format:value range from 0 to 23, BCD format:value range from 0x00 to 0x23 */
    uint8_t weekday;     /* DEC format:value range from 1 to  7, BCD format:value range from 0x01 to 0x07 */
    uint8_t date;        /* DEC format:value range from 1 to 31, BCD format:value range from 0x01 to 0x31 */
    uint8_t month;       /* DEC format:value range from 1 to 12, BCD format:value range from 0x01 to 0x12 */
    uint8_t year;        /* DEC format:value range from 0 to 99, BCD format:value range from 0x00 to 0x99 */
} rtc_time_t;

extern void rtc_time_init(void);
extern void rtc_time_deinit(void);
extern int32_t rtc_set_time(const rtc_time_t *time);
extern int32_t rtc_get_time(rtc_time_t *time);
extern void rtc_time_register_alarm_config(uint32_t alarm_time, void (*callback)());

#endif /* RTC_TIME_H_ */
