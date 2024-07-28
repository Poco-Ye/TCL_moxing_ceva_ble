#ifndef _REG_RTC_H_
#define _REG_RTC_H_

#include <stdint.h>
#include "_reg_rtc.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_RTC_COUNT 17

#define REG_RTC_DECODING_MASK 0x0000001F

/**
 * @brief SEC register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  06:04               TENSEC   0x0
 *  03:00                  SEC   0x0
 * </pre>
 */
#define RTC_SEC_ADDR   0x00000000
#define RTC_SEC_OFFSET 0x00000000
#define RTC_SEC_INDEX  0x00000000
#define RTC_SEC_RESET  0x00000000

__INLINE uint8_t rtc_sec_get(void)
{
    return REG_RTC_RD(RTC_SEC_ADDR);
}

__INLINE void rtc_sec_set(uint8_t value)
{
    REG_RTC_WR(RTC_SEC_ADDR, value);
}

// field definitions
#define RTC_TENSEC_MASK   ((uint8_t)0x00000070)
#define RTC_TENSEC_LSB    4
#define RTC_TENSEC_WIDTH  ((uint8_t)0x00000003)
#define RTC_SEC_MASK      ((uint8_t)0x0000000F)
#define RTC_SEC_LSB       0
#define RTC_SEC_WIDTH     ((uint8_t)0x00000004)

#define RTC_TENSEC_RST    0x0
#define RTC_SEC_RST       0x0

__INLINE void rtc_sec_pack(uint8_t tensec, uint8_t sec)
{
    ASSERT_ERR((((uint8_t)tensec << 4) & ~((uint8_t)0x00000070)) == 0);
    ASSERT_ERR((((uint8_t)sec << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_SEC_ADDR,  ((uint8_t)tensec << 4) | ((uint8_t)sec << 0));
}

__INLINE void rtc_sec_unpack(uint8_t* tensec, uint8_t* sec)
{
    uint8_t localVal = REG_RTC_RD(RTC_SEC_ADDR);

    *tensec = (localVal & ((uint8_t)0x00000070)) >> 4;
    *sec = (localVal & ((uint8_t)0x0000000F)) >> 0;
}

__INLINE uint8_t rtc_sec_tensec_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_SEC_ADDR);
    return ((localVal & ((uint8_t)0x00000070)) >> 4);
}

__INLINE void rtc_sec_tensec_setf(uint8_t tensec)
{
    ASSERT_ERR((((uint8_t)tensec << 4) & ~((uint8_t)0x00000070)) == 0);
    REG_RTC_WR(RTC_SEC_ADDR, (REG_RTC_RD(RTC_SEC_ADDR) & ~((uint8_t)0x00000070)) | ((uint8_t)tensec << 4));
}

__INLINE uint8_t rtc_sec_sec_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_SEC_ADDR);
    return ((localVal & ((uint8_t)0x0000000F)) >> 0);
}

__INLINE void rtc_sec_sec_setf(uint8_t sec)
{
    ASSERT_ERR((((uint8_t)sec << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_SEC_ADDR, (REG_RTC_RD(RTC_SEC_ADDR) & ~((uint8_t)0x0000000F)) | ((uint8_t)sec << 0));
}

/**
 * @brief MIN register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  06:04               TENMIN   0x0
 *  03:00                  MIN   0x0
 * </pre>
 */
#define RTC_MIN_ADDR   0x00000001
#define RTC_MIN_OFFSET 0x00000001
#define RTC_MIN_INDEX  0x00000001
#define RTC_MIN_RESET  0x00000000

__INLINE uint8_t rtc_min_get(void)
{
    return REG_RTC_RD(RTC_MIN_ADDR);
}

__INLINE void rtc_min_set(uint8_t value)
{
    REG_RTC_WR(RTC_MIN_ADDR, value);
}

// field definitions
#define RTC_TENMIN_MASK   ((uint8_t)0x00000070)
#define RTC_TENMIN_LSB    4
#define RTC_TENMIN_WIDTH  ((uint8_t)0x00000003)
#define RTC_MIN_MASK      ((uint8_t)0x0000000F)
#define RTC_MIN_LSB       0
#define RTC_MIN_WIDTH     ((uint8_t)0x00000004)

#define RTC_TENMIN_RST    0x0
#define RTC_MIN_RST       0x0

__INLINE void rtc_min_pack(uint8_t tenmin, uint8_t min)
{
    ASSERT_ERR((((uint8_t)tenmin << 4) & ~((uint8_t)0x00000070)) == 0);
    ASSERT_ERR((((uint8_t)min << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_MIN_ADDR,  ((uint8_t)tenmin << 4) | ((uint8_t)min << 0));
}

__INLINE void rtc_min_unpack(uint8_t* tenmin, uint8_t* min)
{
    uint8_t localVal = REG_RTC_RD(RTC_MIN_ADDR);

    *tenmin = (localVal & ((uint8_t)0x00000070)) >> 4;
    *min = (localVal & ((uint8_t)0x0000000F)) >> 0;
}

__INLINE uint8_t rtc_min_tenmin_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_MIN_ADDR);
    return ((localVal & ((uint8_t)0x00000070)) >> 4);
}

__INLINE void rtc_min_tenmin_setf(uint8_t tenmin)
{
    ASSERT_ERR((((uint8_t)tenmin << 4) & ~((uint8_t)0x00000070)) == 0);
    REG_RTC_WR(RTC_MIN_ADDR, (REG_RTC_RD(RTC_MIN_ADDR) & ~((uint8_t)0x00000070)) | ((uint8_t)tenmin << 4));
}

__INLINE uint8_t rtc_min_min_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_MIN_ADDR);
    return ((localVal & ((uint8_t)0x0000000F)) >> 0);
}

__INLINE void rtc_min_min_setf(uint8_t min)
{
    ASSERT_ERR((((uint8_t)min << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_MIN_ADDR, (REG_RTC_RD(RTC_MIN_ADDR) & ~((uint8_t)0x0000000F)) | ((uint8_t)min << 0));
}

/**
 * @brief HOUR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     06             HOURTYPE   0
 *  05:04              TENHOUR   0x0
 *  03:00                 HOUR   0x0
 * </pre>
 */
#define RTC_HOUR_ADDR   0x00000002
#define RTC_HOUR_OFFSET 0x00000002
#define RTC_HOUR_INDEX  0x00000002
#define RTC_HOUR_RESET  0x00000000

__INLINE uint8_t rtc_hour_get(void)
{
    return REG_RTC_RD(RTC_HOUR_ADDR);
}

__INLINE void rtc_hour_set(uint8_t value)
{
    REG_RTC_WR(RTC_HOUR_ADDR, value);
}

// field definitions
#define RTC_HOURTYPE_BIT    ((uint8_t)0x00000040)
#define RTC_HOURTYPE_POS    6
#define RTC_TENHOUR_MASK    ((uint8_t)0x00000030)
#define RTC_TENHOUR_LSB     4
#define RTC_TENHOUR_WIDTH   ((uint8_t)0x00000002)
#define RTC_HOUR_MASK       ((uint8_t)0x0000000F)
#define RTC_HOUR_LSB        0
#define RTC_HOUR_WIDTH      ((uint8_t)0x00000004)

#define RTC_HOURTYPE_RST    0x0
#define RTC_TENHOUR_RST     0x0
#define RTC_HOUR_RST        0x0

__INLINE void rtc_hour_pack(uint8_t hourtype, uint8_t tenhour, uint8_t hour)
{
    ASSERT_ERR((((uint8_t)hourtype << 6) & ~((uint8_t)0x00000040)) == 0);
    ASSERT_ERR((((uint8_t)tenhour << 4) & ~((uint8_t)0x00000030)) == 0);
    ASSERT_ERR((((uint8_t)hour << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_HOUR_ADDR,  ((uint8_t)hourtype << 6) | ((uint8_t)tenhour << 4) | ((uint8_t)hour << 0));
}

__INLINE void rtc_hour_unpack(uint8_t* hourtype, uint8_t* tenhour, uint8_t* hour)
{
    uint8_t localVal = REG_RTC_RD(RTC_HOUR_ADDR);

    *hourtype = (localVal & ((uint8_t)0x00000040)) >> 6;
    *tenhour = (localVal & ((uint8_t)0x00000030)) >> 4;
    *hour = (localVal & ((uint8_t)0x0000000F)) >> 0;
}

__INLINE uint8_t rtc_hour_hourtype_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_HOUR_ADDR);
    return ((localVal & ((uint8_t)0x00000040)) >> 6);
}

__INLINE void rtc_hour_hourtype_setf(uint8_t hourtype)
{
    ASSERT_ERR((((uint8_t)hourtype << 6) & ~((uint8_t)0x00000040)) == 0);
    REG_RTC_WR(RTC_HOUR_ADDR, (REG_RTC_RD(RTC_HOUR_ADDR) & ~((uint8_t)0x00000040)) | ((uint8_t)hourtype << 6));
}

__INLINE uint8_t rtc_hour_tenhour_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_HOUR_ADDR);
    return ((localVal & ((uint8_t)0x00000030)) >> 4);
}

__INLINE void rtc_hour_tenhour_setf(uint8_t tenhour)
{
    ASSERT_ERR((((uint8_t)tenhour << 4) & ~((uint8_t)0x00000030)) == 0);
    REG_RTC_WR(RTC_HOUR_ADDR, (REG_RTC_RD(RTC_HOUR_ADDR) & ~((uint8_t)0x00000030)) | ((uint8_t)tenhour << 4));
}

__INLINE uint8_t rtc_hour_hour_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_HOUR_ADDR);
    return ((localVal & ((uint8_t)0x0000000F)) >> 0);
}

__INLINE void rtc_hour_hour_setf(uint8_t hour)
{
    ASSERT_ERR((((uint8_t)hour << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_HOUR_ADDR, (REG_RTC_RD(RTC_HOUR_ADDR) & ~((uint8_t)0x0000000F)) | ((uint8_t)hour << 0));
}

/**
 * @brief DAY register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  02:00                  DAY   0x0
 * </pre>
 */
#define RTC_DAY_ADDR   0x00000003
#define RTC_DAY_OFFSET 0x00000003
#define RTC_DAY_INDEX  0x00000003
#define RTC_DAY_RESET  0x00000000

__INLINE uint8_t rtc_day_get(void)
{
    return REG_RTC_RD(RTC_DAY_ADDR);
}

__INLINE void rtc_day_set(uint8_t value)
{
    REG_RTC_WR(RTC_DAY_ADDR, value);
}

// field definitions
#define RTC_DAY_MASK   ((uint8_t)0x00000007)
#define RTC_DAY_LSB    0
#define RTC_DAY_WIDTH  ((uint8_t)0x00000003)

#define RTC_DAY_RST    0x0

__INLINE uint8_t rtc_day_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_DAY_ADDR);
    ASSERT_ERR((localVal & ~((uint8_t)0x00000007)) == 0);
    return (localVal >> 0);
}

__INLINE void rtc_day_setf(uint8_t day)
{
    ASSERT_ERR((((uint8_t)day << 0) & ~((uint8_t)0x00000007)) == 0);
    REG_RTC_WR(RTC_DAY_ADDR, (uint8_t)day << 0);
}

/**
 * @brief DATE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  05:04              TENDATE   0x0
 *  03:00                 DATE   0x0
 * </pre>
 */
#define RTC_DATE_ADDR   0x00000004
#define RTC_DATE_OFFSET 0x00000004
#define RTC_DATE_INDEX  0x00000004
#define RTC_DATE_RESET  0x00000000

__INLINE uint8_t rtc_date_get(void)
{
    return REG_RTC_RD(RTC_DATE_ADDR);
}

__INLINE void rtc_date_set(uint8_t value)
{
    REG_RTC_WR(RTC_DATE_ADDR, value);
}

// field definitions
#define RTC_TENDATE_MASK   ((uint8_t)0x00000030)
#define RTC_TENDATE_LSB    4
#define RTC_TENDATE_WIDTH  ((uint8_t)0x00000002)
#define RTC_DATE_MASK      ((uint8_t)0x0000000F)
#define RTC_DATE_LSB       0
#define RTC_DATE_WIDTH     ((uint8_t)0x00000004)

#define RTC_TENDATE_RST    0x0
#define RTC_DATE_RST       0x0

__INLINE void rtc_date_pack(uint8_t tendate, uint8_t date)
{
    ASSERT_ERR((((uint8_t)tendate << 4) & ~((uint8_t)0x00000030)) == 0);
    ASSERT_ERR((((uint8_t)date << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_DATE_ADDR,  ((uint8_t)tendate << 4) | ((uint8_t)date << 0));
}

__INLINE void rtc_date_unpack(uint8_t* tendate, uint8_t* date)
{
    uint8_t localVal = REG_RTC_RD(RTC_DATE_ADDR);

    *tendate = (localVal & ((uint8_t)0x00000030)) >> 4;
    *date = (localVal & ((uint8_t)0x0000000F)) >> 0;
}

__INLINE uint8_t rtc_date_tendate_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_DATE_ADDR);
    return ((localVal & ((uint8_t)0x00000030)) >> 4);
}

__INLINE void rtc_date_tendate_setf(uint8_t tendate)
{
    ASSERT_ERR((((uint8_t)tendate << 4) & ~((uint8_t)0x00000030)) == 0);
    REG_RTC_WR(RTC_DATE_ADDR, (REG_RTC_RD(RTC_DATE_ADDR) & ~((uint8_t)0x00000030)) | ((uint8_t)tendate << 4));
}

__INLINE uint8_t rtc_date_date_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_DATE_ADDR);
    return ((localVal & ((uint8_t)0x0000000F)) >> 0);
}

__INLINE void rtc_date_date_setf(uint8_t date)
{
    ASSERT_ERR((((uint8_t)date << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_DATE_ADDR, (REG_RTC_RD(RTC_DATE_ADDR) & ~((uint8_t)0x0000000F)) | ((uint8_t)date << 0));
}

/**
 * @brief MONTH register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  06:04             TENMONTH   0x0
 *  03:00                MONTH   0x0
 * </pre>
 */
#define RTC_MONTH_ADDR   0x00000005
#define RTC_MONTH_OFFSET 0x00000005
#define RTC_MONTH_INDEX  0x00000005
#define RTC_MONTH_RESET  0x00000000

__INLINE uint8_t rtc_month_get(void)
{
    return REG_RTC_RD(RTC_MONTH_ADDR);
}

__INLINE void rtc_month_set(uint8_t value)
{
    REG_RTC_WR(RTC_MONTH_ADDR, value);
}

// field definitions
#define RTC_TENMONTH_MASK   ((uint8_t)0x00000070)
#define RTC_TENMONTH_LSB    4
#define RTC_TENMONTH_WIDTH  ((uint8_t)0x00000003)
#define RTC_MONTH_MASK      ((uint8_t)0x0000000F)
#define RTC_MONTH_LSB       0
#define RTC_MONTH_WIDTH     ((uint8_t)0x00000004)

#define RTC_TENMONTH_RST    0x0
#define RTC_MONTH_RST       0x0

__INLINE void rtc_month_pack(uint8_t tenmonth, uint8_t month)
{
    ASSERT_ERR((((uint8_t)tenmonth << 4) & ~((uint8_t)0x00000070)) == 0);
    ASSERT_ERR((((uint8_t)month << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_MONTH_ADDR,  ((uint8_t)tenmonth << 4) | ((uint8_t)month << 0));
}

__INLINE void rtc_month_unpack(uint8_t* tenmonth, uint8_t* month)
{
    uint8_t localVal = REG_RTC_RD(RTC_MONTH_ADDR);

    *tenmonth = (localVal & ((uint8_t)0x00000070)) >> 4;
    *month = (localVal & ((uint8_t)0x0000000F)) >> 0;
}

__INLINE uint8_t rtc_month_tenmonth_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_MONTH_ADDR);
    return ((localVal & ((uint8_t)0x00000070)) >> 4);
}

__INLINE void rtc_month_tenmonth_setf(uint8_t tenmonth)
{
    ASSERT_ERR((((uint8_t)tenmonth << 4) & ~((uint8_t)0x00000070)) == 0);
    REG_RTC_WR(RTC_MONTH_ADDR, (REG_RTC_RD(RTC_MONTH_ADDR) & ~((uint8_t)0x00000070)) | ((uint8_t)tenmonth << 4));
}

__INLINE uint8_t rtc_month_month_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_MONTH_ADDR);
    return ((localVal & ((uint8_t)0x0000000F)) >> 0);
}

__INLINE void rtc_month_month_setf(uint8_t month)
{
    ASSERT_ERR((((uint8_t)month << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_MONTH_ADDR, (REG_RTC_RD(RTC_MONTH_ADDR) & ~((uint8_t)0x0000000F)) | ((uint8_t)month << 0));
}

/**
 * @brief YEAR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:04              TENYEAR   0x0
 *  03:00                 YEAR   0x0
 * </pre>
 */
#define RTC_YEAR_ADDR   0x00000006
#define RTC_YEAR_OFFSET 0x00000006
#define RTC_YEAR_INDEX  0x00000006
#define RTC_YEAR_RESET  0x00000000

__INLINE uint8_t rtc_year_get(void)
{
    return REG_RTC_RD(RTC_YEAR_ADDR);
}

__INLINE void rtc_year_set(uint8_t value)
{
    REG_RTC_WR(RTC_YEAR_ADDR, value);
}

// field definitions
#define RTC_TENYEAR_MASK   ((uint8_t)0x000000F0)
#define RTC_TENYEAR_LSB    4
#define RTC_TENYEAR_WIDTH  ((uint8_t)0x00000004)
#define RTC_YEAR_MASK      ((uint8_t)0x0000000F)
#define RTC_YEAR_LSB       0
#define RTC_YEAR_WIDTH     ((uint8_t)0x00000004)

#define RTC_TENYEAR_RST    0x0
#define RTC_YEAR_RST       0x0

__INLINE void rtc_year_pack(uint8_t tenyear, uint8_t year)
{
    ASSERT_ERR((((uint8_t)tenyear << 4) & ~((uint8_t)0x000000F0)) == 0);
    ASSERT_ERR((((uint8_t)year << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_YEAR_ADDR,  ((uint8_t)tenyear << 4) | ((uint8_t)year << 0));
}

__INLINE void rtc_year_unpack(uint8_t* tenyear, uint8_t* year)
{
    uint8_t localVal = REG_RTC_RD(RTC_YEAR_ADDR);

    *tenyear = (localVal & ((uint8_t)0x000000F0)) >> 4;
    *year = (localVal & ((uint8_t)0x0000000F)) >> 0;
}

__INLINE uint8_t rtc_year_tenyear_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_YEAR_ADDR);
    return ((localVal & ((uint8_t)0x000000F0)) >> 4);
}

__INLINE void rtc_year_tenyear_setf(uint8_t tenyear)
{
    ASSERT_ERR((((uint8_t)tenyear << 4) & ~((uint8_t)0x000000F0)) == 0);
    REG_RTC_WR(RTC_YEAR_ADDR, (REG_RTC_RD(RTC_YEAR_ADDR) & ~((uint8_t)0x000000F0)) | ((uint8_t)tenyear << 4));
}

__INLINE uint8_t rtc_year_year_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_YEAR_ADDR);
    return ((localVal & ((uint8_t)0x0000000F)) >> 0);
}

__INLINE void rtc_year_year_setf(uint8_t year)
{
    ASSERT_ERR((((uint8_t)year << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_YEAR_ADDR, (REG_RTC_RD(RTC_YEAR_ADDR) & ~((uint8_t)0x0000000F)) | ((uint8_t)year << 0));
}

/**
 * @brief SEC_ALARM0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     07      SEC_MASK_ALARM0   0
 *  06:04        TENSEC_ALARM0   0x0
 *  03:00           SEC_ALARM0   0x0
 * </pre>
 */
#define RTC_SEC_ALARM0_ADDR   0x00000007
#define RTC_SEC_ALARM0_OFFSET 0x00000007
#define RTC_SEC_ALARM0_INDEX  0x00000007
#define RTC_SEC_ALARM0_RESET  0x00000000

__INLINE uint8_t rtc_sec_alarm0_get(void)
{
    return REG_RTC_RD(RTC_SEC_ALARM0_ADDR);
}

__INLINE void rtc_sec_alarm0_set(uint8_t value)
{
    REG_RTC_WR(RTC_SEC_ALARM0_ADDR, value);
}

// field definitions
#define RTC_SEC_MASK_ALARM0_BIT    ((uint8_t)0x00000080)
#define RTC_SEC_MASK_ALARM0_POS    7
#define RTC_TENSEC_ALARM0_MASK     ((uint8_t)0x00000070)
#define RTC_TENSEC_ALARM0_LSB      4
#define RTC_TENSEC_ALARM0_WIDTH    ((uint8_t)0x00000003)
#define RTC_SEC_ALARM0_MASK        ((uint8_t)0x0000000F)
#define RTC_SEC_ALARM0_LSB         0
#define RTC_SEC_ALARM0_WIDTH       ((uint8_t)0x00000004)

#define RTC_SEC_MASK_ALARM0_RST    0x0
#define RTC_TENSEC_ALARM0_RST      0x0
#define RTC_SEC_ALARM0_RST         0x0

__INLINE void rtc_sec_alarm0_pack(uint8_t secmaskalarm0, uint8_t tensecalarm0, uint8_t secalarm0)
{
    ASSERT_ERR((((uint8_t)secmaskalarm0 << 7) & ~((uint8_t)0x00000080)) == 0);
    ASSERT_ERR((((uint8_t)tensecalarm0 << 4) & ~((uint8_t)0x00000070)) == 0);
    ASSERT_ERR((((uint8_t)secalarm0 << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_SEC_ALARM0_ADDR,  ((uint8_t)secmaskalarm0 << 7) | ((uint8_t)tensecalarm0 << 4) | ((uint8_t)secalarm0 << 0));
}

__INLINE void rtc_sec_alarm0_unpack(uint8_t* secmaskalarm0, uint8_t* tensecalarm0, uint8_t* secalarm0)
{
    uint8_t localVal = REG_RTC_RD(RTC_SEC_ALARM0_ADDR);

    *secmaskalarm0 = (localVal & ((uint8_t)0x00000080)) >> 7;
    *tensecalarm0 = (localVal & ((uint8_t)0x00000070)) >> 4;
    *secalarm0 = (localVal & ((uint8_t)0x0000000F)) >> 0;
}

__INLINE uint8_t rtc_sec_alarm0_sec_mask_alarm0_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_SEC_ALARM0_ADDR);
    return ((localVal & ((uint8_t)0x00000080)) >> 7);
}

__INLINE void rtc_sec_alarm0_sec_mask_alarm0_setf(uint8_t secmaskalarm0)
{
    ASSERT_ERR((((uint8_t)secmaskalarm0 << 7) & ~((uint8_t)0x00000080)) == 0);
    REG_RTC_WR(RTC_SEC_ALARM0_ADDR, (REG_RTC_RD(RTC_SEC_ALARM0_ADDR) & ~((uint8_t)0x00000080)) | ((uint8_t)secmaskalarm0 << 7));
}

__INLINE uint8_t rtc_sec_alarm0_tensec_alarm0_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_SEC_ALARM0_ADDR);
    return ((localVal & ((uint8_t)0x00000070)) >> 4);
}

__INLINE void rtc_sec_alarm0_tensec_alarm0_setf(uint8_t tensecalarm0)
{
    ASSERT_ERR((((uint8_t)tensecalarm0 << 4) & ~((uint8_t)0x00000070)) == 0);
    REG_RTC_WR(RTC_SEC_ALARM0_ADDR, (REG_RTC_RD(RTC_SEC_ALARM0_ADDR) & ~((uint8_t)0x00000070)) | ((uint8_t)tensecalarm0 << 4));
}

__INLINE uint8_t rtc_sec_alarm0_sec_alarm0_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_SEC_ALARM0_ADDR);
    return ((localVal & ((uint8_t)0x0000000F)) >> 0);
}

__INLINE void rtc_sec_alarm0_sec_alarm0_setf(uint8_t secalarm0)
{
    ASSERT_ERR((((uint8_t)secalarm0 << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_SEC_ALARM0_ADDR, (REG_RTC_RD(RTC_SEC_ALARM0_ADDR) & ~((uint8_t)0x0000000F)) | ((uint8_t)secalarm0 << 0));
}

/**
 * @brief MIN_ALARM0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     07      MIN_MASK_ALARM0   0
 *  06:04        TENMIN_ALARM0   0x0
 *  03:00           MIN_ALARM0   0x0
 * </pre>
 */
#define RTC_MIN_ALARM0_ADDR   0x00000008
#define RTC_MIN_ALARM0_OFFSET 0x00000008
#define RTC_MIN_ALARM0_INDEX  0x00000008
#define RTC_MIN_ALARM0_RESET  0x00000000

__INLINE uint8_t rtc_min_alarm0_get(void)
{
    return REG_RTC_RD(RTC_MIN_ALARM0_ADDR);
}

__INLINE void rtc_min_alarm0_set(uint8_t value)
{
    REG_RTC_WR(RTC_MIN_ALARM0_ADDR, value);
}

// field definitions
#define RTC_MIN_MASK_ALARM0_BIT    ((uint8_t)0x00000080)
#define RTC_MIN_MASK_ALARM0_POS    7
#define RTC_TENMIN_ALARM0_MASK     ((uint8_t)0x00000070)
#define RTC_TENMIN_ALARM0_LSB      4
#define RTC_TENMIN_ALARM0_WIDTH    ((uint8_t)0x00000003)
#define RTC_MIN_ALARM0_MASK        ((uint8_t)0x0000000F)
#define RTC_MIN_ALARM0_LSB         0
#define RTC_MIN_ALARM0_WIDTH       ((uint8_t)0x00000004)

#define RTC_MIN_MASK_ALARM0_RST    0x0
#define RTC_TENMIN_ALARM0_RST      0x0
#define RTC_MIN_ALARM0_RST         0x0

__INLINE void rtc_min_alarm0_pack(uint8_t minmaskalarm0, uint8_t tenminalarm0, uint8_t minalarm0)
{
    ASSERT_ERR((((uint8_t)minmaskalarm0 << 7) & ~((uint8_t)0x00000080)) == 0);
    ASSERT_ERR((((uint8_t)tenminalarm0 << 4) & ~((uint8_t)0x00000070)) == 0);
    ASSERT_ERR((((uint8_t)minalarm0 << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_MIN_ALARM0_ADDR,  ((uint8_t)minmaskalarm0 << 7) | ((uint8_t)tenminalarm0 << 4) | ((uint8_t)minalarm0 << 0));
}

__INLINE void rtc_min_alarm0_unpack(uint8_t* minmaskalarm0, uint8_t* tenminalarm0, uint8_t* minalarm0)
{
    uint8_t localVal = REG_RTC_RD(RTC_MIN_ALARM0_ADDR);

    *minmaskalarm0 = (localVal & ((uint8_t)0x00000080)) >> 7;
    *tenminalarm0 = (localVal & ((uint8_t)0x00000070)) >> 4;
    *minalarm0 = (localVal & ((uint8_t)0x0000000F)) >> 0;
}

__INLINE uint8_t rtc_min_alarm0_min_mask_alarm0_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_MIN_ALARM0_ADDR);
    return ((localVal & ((uint8_t)0x00000080)) >> 7);
}

__INLINE void rtc_min_alarm0_min_mask_alarm0_setf(uint8_t minmaskalarm0)
{
    ASSERT_ERR((((uint8_t)minmaskalarm0 << 7) & ~((uint8_t)0x00000080)) == 0);
    REG_RTC_WR(RTC_MIN_ALARM0_ADDR, (REG_RTC_RD(RTC_MIN_ALARM0_ADDR) & ~((uint8_t)0x00000080)) | ((uint8_t)minmaskalarm0 << 7));
}

__INLINE uint8_t rtc_min_alarm0_tenmin_alarm0_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_MIN_ALARM0_ADDR);
    return ((localVal & ((uint8_t)0x00000070)) >> 4);
}

__INLINE void rtc_min_alarm0_tenmin_alarm0_setf(uint8_t tenminalarm0)
{
    ASSERT_ERR((((uint8_t)tenminalarm0 << 4) & ~((uint8_t)0x00000070)) == 0);
    REG_RTC_WR(RTC_MIN_ALARM0_ADDR, (REG_RTC_RD(RTC_MIN_ALARM0_ADDR) & ~((uint8_t)0x00000070)) | ((uint8_t)tenminalarm0 << 4));
}

__INLINE uint8_t rtc_min_alarm0_min_alarm0_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_MIN_ALARM0_ADDR);
    return ((localVal & ((uint8_t)0x0000000F)) >> 0);
}

__INLINE void rtc_min_alarm0_min_alarm0_setf(uint8_t minalarm0)
{
    ASSERT_ERR((((uint8_t)minalarm0 << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_MIN_ALARM0_ADDR, (REG_RTC_RD(RTC_MIN_ALARM0_ADDR) & ~((uint8_t)0x0000000F)) | ((uint8_t)minalarm0 << 0));
}

/**
 * @brief HOUR_ALARM0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     07     HOUR_MASK_ALARM0   0
 *     06      HOURTYPE_ALARM0   0
 *  05:04       TENHOUR_ALARM0   0x0
 *  03:00          HOUR_ALARM0   0x0
 * </pre>
 */
#define RTC_HOUR_ALARM0_ADDR   0x00000009
#define RTC_HOUR_ALARM0_OFFSET 0x00000009
#define RTC_HOUR_ALARM0_INDEX  0x00000009
#define RTC_HOUR_ALARM0_RESET  0x00000000

__INLINE uint8_t rtc_hour_alarm0_get(void)
{
    return REG_RTC_RD(RTC_HOUR_ALARM0_ADDR);
}

__INLINE void rtc_hour_alarm0_set(uint8_t value)
{
    REG_RTC_WR(RTC_HOUR_ALARM0_ADDR, value);
}

// field definitions
#define RTC_HOUR_MASK_ALARM0_BIT    ((uint8_t)0x00000080)
#define RTC_HOUR_MASK_ALARM0_POS    7
#define RTC_HOURTYPE_ALARM0_BIT     ((uint8_t)0x00000040)
#define RTC_HOURTYPE_ALARM0_POS     6
#define RTC_TENHOUR_ALARM0_MASK     ((uint8_t)0x00000030)
#define RTC_TENHOUR_ALARM0_LSB      4
#define RTC_TENHOUR_ALARM0_WIDTH    ((uint8_t)0x00000002)
#define RTC_HOUR_ALARM0_MASK        ((uint8_t)0x0000000F)
#define RTC_HOUR_ALARM0_LSB         0
#define RTC_HOUR_ALARM0_WIDTH       ((uint8_t)0x00000004)

#define RTC_HOUR_MASK_ALARM0_RST    0x0
#define RTC_HOURTYPE_ALARM0_RST     0x0
#define RTC_TENHOUR_ALARM0_RST      0x0
#define RTC_HOUR_ALARM0_RST         0x0

__INLINE void rtc_hour_alarm0_pack(uint8_t hourmaskalarm0, uint8_t hourtypealarm0, uint8_t tenhouralarm0, uint8_t houralarm0)
{
    ASSERT_ERR((((uint8_t)hourmaskalarm0 << 7) & ~((uint8_t)0x00000080)) == 0);
    ASSERT_ERR((((uint8_t)hourtypealarm0 << 6) & ~((uint8_t)0x00000040)) == 0);
    ASSERT_ERR((((uint8_t)tenhouralarm0 << 4) & ~((uint8_t)0x00000030)) == 0);
    ASSERT_ERR((((uint8_t)houralarm0 << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_HOUR_ALARM0_ADDR,  ((uint8_t)hourmaskalarm0 << 7) | ((uint8_t)hourtypealarm0 << 6) | ((uint8_t)tenhouralarm0 << 4) | ((uint8_t)houralarm0 << 0));
}

__INLINE void rtc_hour_alarm0_unpack(uint8_t* hourmaskalarm0, uint8_t* hourtypealarm0, uint8_t* tenhouralarm0, uint8_t* houralarm0)
{
    uint8_t localVal = REG_RTC_RD(RTC_HOUR_ALARM0_ADDR);

    *hourmaskalarm0 = (localVal & ((uint8_t)0x00000080)) >> 7;
    *hourtypealarm0 = (localVal & ((uint8_t)0x00000040)) >> 6;
    *tenhouralarm0 = (localVal & ((uint8_t)0x00000030)) >> 4;
    *houralarm0 = (localVal & ((uint8_t)0x0000000F)) >> 0;
}

__INLINE uint8_t rtc_hour_alarm0_hour_mask_alarm0_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_HOUR_ALARM0_ADDR);
    return ((localVal & ((uint8_t)0x00000080)) >> 7);
}

__INLINE void rtc_hour_alarm0_hour_mask_alarm0_setf(uint8_t hourmaskalarm0)
{
    ASSERT_ERR((((uint8_t)hourmaskalarm0 << 7) & ~((uint8_t)0x00000080)) == 0);
    REG_RTC_WR(RTC_HOUR_ALARM0_ADDR, (REG_RTC_RD(RTC_HOUR_ALARM0_ADDR) & ~((uint8_t)0x00000080)) | ((uint8_t)hourmaskalarm0 << 7));
}

__INLINE uint8_t rtc_hour_alarm0_hourtype_alarm0_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_HOUR_ALARM0_ADDR);
    return ((localVal & ((uint8_t)0x00000040)) >> 6);
}

__INLINE void rtc_hour_alarm0_hourtype_alarm0_setf(uint8_t hourtypealarm0)
{
    ASSERT_ERR((((uint8_t)hourtypealarm0 << 6) & ~((uint8_t)0x00000040)) == 0);
    REG_RTC_WR(RTC_HOUR_ALARM0_ADDR, (REG_RTC_RD(RTC_HOUR_ALARM0_ADDR) & ~((uint8_t)0x00000040)) | ((uint8_t)hourtypealarm0 << 6));
}

__INLINE uint8_t rtc_hour_alarm0_tenhour_alarm0_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_HOUR_ALARM0_ADDR);
    return ((localVal & ((uint8_t)0x00000030)) >> 4);
}

__INLINE void rtc_hour_alarm0_tenhour_alarm0_setf(uint8_t tenhouralarm0)
{
    ASSERT_ERR((((uint8_t)tenhouralarm0 << 4) & ~((uint8_t)0x00000030)) == 0);
    REG_RTC_WR(RTC_HOUR_ALARM0_ADDR, (REG_RTC_RD(RTC_HOUR_ALARM0_ADDR) & ~((uint8_t)0x00000030)) | ((uint8_t)tenhouralarm0 << 4));
}

__INLINE uint8_t rtc_hour_alarm0_hour_alarm0_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_HOUR_ALARM0_ADDR);
    return ((localVal & ((uint8_t)0x0000000F)) >> 0);
}

__INLINE void rtc_hour_alarm0_hour_alarm0_setf(uint8_t houralarm0)
{
    ASSERT_ERR((((uint8_t)houralarm0 << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_HOUR_ALARM0_ADDR, (REG_RTC_RD(RTC_HOUR_ALARM0_ADDR) & ~((uint8_t)0x0000000F)) | ((uint8_t)houralarm0 << 0));
}

/**
 * @brief DAY_ALARM0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     07      DAY_MASK_ALARM0   0
 *  02:00           DAY_ALARM0   0x0
 * </pre>
 */
#define RTC_DAY_ALARM0_ADDR   0x0000000A
#define RTC_DAY_ALARM0_OFFSET 0x0000000A
#define RTC_DAY_ALARM0_INDEX  0x0000000A
#define RTC_DAY_ALARM0_RESET  0x00000000

__INLINE uint8_t rtc_day_alarm0_get(void)
{
    return REG_RTC_RD(RTC_DAY_ALARM0_ADDR);
}

__INLINE void rtc_day_alarm0_set(uint8_t value)
{
    REG_RTC_WR(RTC_DAY_ALARM0_ADDR, value);
}

// field definitions
#define RTC_DAY_MASK_ALARM0_BIT    ((uint8_t)0x00000080)
#define RTC_DAY_MASK_ALARM0_POS    7
#define RTC_DAY_ALARM0_MASK        ((uint8_t)0x00000007)
#define RTC_DAY_ALARM0_LSB         0
#define RTC_DAY_ALARM0_WIDTH       ((uint8_t)0x00000003)

#define RTC_DAY_MASK_ALARM0_RST    0x0
#define RTC_DAY_ALARM0_RST         0x0

__INLINE void rtc_day_alarm0_pack(uint8_t daymaskalarm0, uint8_t dayalarm0)
{
    ASSERT_ERR((((uint8_t)daymaskalarm0 << 7) & ~((uint8_t)0x00000080)) == 0);
    ASSERT_ERR((((uint8_t)dayalarm0 << 0) & ~((uint8_t)0x00000007)) == 0);
    REG_RTC_WR(RTC_DAY_ALARM0_ADDR,  ((uint8_t)daymaskalarm0 << 7) | ((uint8_t)dayalarm0 << 0));
}

__INLINE void rtc_day_alarm0_unpack(uint8_t* daymaskalarm0, uint8_t* dayalarm0)
{
    uint8_t localVal = REG_RTC_RD(RTC_DAY_ALARM0_ADDR);

    *daymaskalarm0 = (localVal & ((uint8_t)0x00000080)) >> 7;
    *dayalarm0 = (localVal & ((uint8_t)0x00000007)) >> 0;
}

__INLINE uint8_t rtc_day_alarm0_day_mask_alarm0_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_DAY_ALARM0_ADDR);
    return ((localVal & ((uint8_t)0x00000080)) >> 7);
}

__INLINE void rtc_day_alarm0_day_mask_alarm0_setf(uint8_t daymaskalarm0)
{
    ASSERT_ERR((((uint8_t)daymaskalarm0 << 7) & ~((uint8_t)0x00000080)) == 0);
    REG_RTC_WR(RTC_DAY_ALARM0_ADDR, (REG_RTC_RD(RTC_DAY_ALARM0_ADDR) & ~((uint8_t)0x00000080)) | ((uint8_t)daymaskalarm0 << 7));
}

__INLINE uint8_t rtc_day_alarm0_day_alarm0_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_DAY_ALARM0_ADDR);
    return ((localVal & ((uint8_t)0x00000007)) >> 0);
}

__INLINE void rtc_day_alarm0_day_alarm0_setf(uint8_t dayalarm0)
{
    ASSERT_ERR((((uint8_t)dayalarm0 << 0) & ~((uint8_t)0x00000007)) == 0);
    REG_RTC_WR(RTC_DAY_ALARM0_ADDR, (REG_RTC_RD(RTC_DAY_ALARM0_ADDR) & ~((uint8_t)0x00000007)) | ((uint8_t)dayalarm0 << 0));
}

/**
 * @brief SEC_ALARM1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     07      SEC_MASK_ALARM1   0
 *  06:04        TENSEC_ALARM1   0x0
 *  03:00           SEC_ALARM1   0x0
 * </pre>
 */
#define RTC_SEC_ALARM1_ADDR   0x0000000B
#define RTC_SEC_ALARM1_OFFSET 0x0000000B
#define RTC_SEC_ALARM1_INDEX  0x0000000B
#define RTC_SEC_ALARM1_RESET  0x00000000

__INLINE uint8_t rtc_sec_alarm1_get(void)
{
    return REG_RTC_RD(RTC_SEC_ALARM1_ADDR);
}

__INLINE void rtc_sec_alarm1_set(uint8_t value)
{
    REG_RTC_WR(RTC_SEC_ALARM1_ADDR, value);
}

// field definitions
#define RTC_SEC_MASK_ALARM1_BIT    ((uint8_t)0x00000080)
#define RTC_SEC_MASK_ALARM1_POS    7
#define RTC_TENSEC_ALARM1_MASK     ((uint8_t)0x00000070)
#define RTC_TENSEC_ALARM1_LSB      4
#define RTC_TENSEC_ALARM1_WIDTH    ((uint8_t)0x00000003)
#define RTC_SEC_ALARM1_MASK        ((uint8_t)0x0000000F)
#define RTC_SEC_ALARM1_LSB         0
#define RTC_SEC_ALARM1_WIDTH       ((uint8_t)0x00000004)

#define RTC_SEC_MASK_ALARM1_RST    0x0
#define RTC_TENSEC_ALARM1_RST      0x0
#define RTC_SEC_ALARM1_RST         0x0

__INLINE void rtc_sec_alarm1_pack(uint8_t secmaskalarm1, uint8_t tensecalarm1, uint8_t secalarm1)
{
    ASSERT_ERR((((uint8_t)secmaskalarm1 << 7) & ~((uint8_t)0x00000080)) == 0);
    ASSERT_ERR((((uint8_t)tensecalarm1 << 4) & ~((uint8_t)0x00000070)) == 0);
    ASSERT_ERR((((uint8_t)secalarm1 << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_SEC_ALARM1_ADDR,  ((uint8_t)secmaskalarm1 << 7) | ((uint8_t)tensecalarm1 << 4) | ((uint8_t)secalarm1 << 0));
}

__INLINE void rtc_sec_alarm1_unpack(uint8_t* secmaskalarm1, uint8_t* tensecalarm1, uint8_t* secalarm1)
{
    uint8_t localVal = REG_RTC_RD(RTC_SEC_ALARM1_ADDR);

    *secmaskalarm1 = (localVal & ((uint8_t)0x00000080)) >> 7;
    *tensecalarm1 = (localVal & ((uint8_t)0x00000070)) >> 4;
    *secalarm1 = (localVal & ((uint8_t)0x0000000F)) >> 0;
}

__INLINE uint8_t rtc_sec_alarm1_sec_mask_alarm1_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_SEC_ALARM1_ADDR);
    return ((localVal & ((uint8_t)0x00000080)) >> 7);
}

__INLINE void rtc_sec_alarm1_sec_mask_alarm1_setf(uint8_t secmaskalarm1)
{
    ASSERT_ERR((((uint8_t)secmaskalarm1 << 7) & ~((uint8_t)0x00000080)) == 0);
    REG_RTC_WR(RTC_SEC_ALARM1_ADDR, (REG_RTC_RD(RTC_SEC_ALARM1_ADDR) & ~((uint8_t)0x00000080)) | ((uint8_t)secmaskalarm1 << 7));
}

__INLINE uint8_t rtc_sec_alarm1_tensec_alarm1_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_SEC_ALARM1_ADDR);
    return ((localVal & ((uint8_t)0x00000070)) >> 4);
}

__INLINE void rtc_sec_alarm1_tensec_alarm1_setf(uint8_t tensecalarm1)
{
    ASSERT_ERR((((uint8_t)tensecalarm1 << 4) & ~((uint8_t)0x00000070)) == 0);
    REG_RTC_WR(RTC_SEC_ALARM1_ADDR, (REG_RTC_RD(RTC_SEC_ALARM1_ADDR) & ~((uint8_t)0x00000070)) | ((uint8_t)tensecalarm1 << 4));
}

__INLINE uint8_t rtc_sec_alarm1_sec_alarm1_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_SEC_ALARM1_ADDR);
    return ((localVal & ((uint8_t)0x0000000F)) >> 0);
}

__INLINE void rtc_sec_alarm1_sec_alarm1_setf(uint8_t secalarm1)
{
    ASSERT_ERR((((uint8_t)secalarm1 << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_SEC_ALARM1_ADDR, (REG_RTC_RD(RTC_SEC_ALARM1_ADDR) & ~((uint8_t)0x0000000F)) | ((uint8_t)secalarm1 << 0));
}

/**
 * @brief MIN_ALARM1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     07      MIN_MASK_ALARM1   0
 *  06:04        TENMIN_ALARM1   0x0
 *  03:00           MIN_ALARM1   0x0
 * </pre>
 */
#define RTC_MIN_ALARM1_ADDR   0x0000000C
#define RTC_MIN_ALARM1_OFFSET 0x0000000C
#define RTC_MIN_ALARM1_INDEX  0x0000000C
#define RTC_MIN_ALARM1_RESET  0x00000000

__INLINE uint8_t rtc_min_alarm1_get(void)
{
    return REG_RTC_RD(RTC_MIN_ALARM1_ADDR);
}

__INLINE void rtc_min_alarm1_set(uint8_t value)
{
    REG_RTC_WR(RTC_MIN_ALARM1_ADDR, value);
}

// field definitions
#define RTC_MIN_MASK_ALARM1_BIT    ((uint8_t)0x00000080)
#define RTC_MIN_MASK_ALARM1_POS    7
#define RTC_TENMIN_ALARM1_MASK     ((uint8_t)0x00000070)
#define RTC_TENMIN_ALARM1_LSB      4
#define RTC_TENMIN_ALARM1_WIDTH    ((uint8_t)0x00000003)
#define RTC_MIN_ALARM1_MASK        ((uint8_t)0x0000000F)
#define RTC_MIN_ALARM1_LSB         0
#define RTC_MIN_ALARM1_WIDTH       ((uint8_t)0x00000004)

#define RTC_MIN_MASK_ALARM1_RST    0x0
#define RTC_TENMIN_ALARM1_RST      0x0
#define RTC_MIN_ALARM1_RST         0x0

__INLINE void rtc_min_alarm1_pack(uint8_t minmaskalarm1, uint8_t tenminalarm1, uint8_t minalarm1)
{
    ASSERT_ERR((((uint8_t)minmaskalarm1 << 7) & ~((uint8_t)0x00000080)) == 0);
    ASSERT_ERR((((uint8_t)tenminalarm1 << 4) & ~((uint8_t)0x00000070)) == 0);
    ASSERT_ERR((((uint8_t)minalarm1 << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_MIN_ALARM1_ADDR,  ((uint8_t)minmaskalarm1 << 7) | ((uint8_t)tenminalarm1 << 4) | ((uint8_t)minalarm1 << 0));
}

__INLINE void rtc_min_alarm1_unpack(uint8_t* minmaskalarm1, uint8_t* tenminalarm1, uint8_t* minalarm1)
{
    uint8_t localVal = REG_RTC_RD(RTC_MIN_ALARM1_ADDR);

    *minmaskalarm1 = (localVal & ((uint8_t)0x00000080)) >> 7;
    *tenminalarm1 = (localVal & ((uint8_t)0x00000070)) >> 4;
    *minalarm1 = (localVal & ((uint8_t)0x0000000F)) >> 0;
}

__INLINE uint8_t rtc_min_alarm1_min_mask_alarm1_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_MIN_ALARM1_ADDR);
    return ((localVal & ((uint8_t)0x00000080)) >> 7);
}

__INLINE void rtc_min_alarm1_min_mask_alarm1_setf(uint8_t minmaskalarm1)
{
    ASSERT_ERR((((uint8_t)minmaskalarm1 << 7) & ~((uint8_t)0x00000080)) == 0);
    REG_RTC_WR(RTC_MIN_ALARM1_ADDR, (REG_RTC_RD(RTC_MIN_ALARM1_ADDR) & ~((uint8_t)0x00000080)) | ((uint8_t)minmaskalarm1 << 7));
}

__INLINE uint8_t rtc_min_alarm1_tenmin_alarm1_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_MIN_ALARM1_ADDR);
    return ((localVal & ((uint8_t)0x00000070)) >> 4);
}

__INLINE void rtc_min_alarm1_tenmin_alarm1_setf(uint8_t tenminalarm1)
{
    ASSERT_ERR((((uint8_t)tenminalarm1 << 4) & ~((uint8_t)0x00000070)) == 0);
    REG_RTC_WR(RTC_MIN_ALARM1_ADDR, (REG_RTC_RD(RTC_MIN_ALARM1_ADDR) & ~((uint8_t)0x00000070)) | ((uint8_t)tenminalarm1 << 4));
}

__INLINE uint8_t rtc_min_alarm1_min_alarm1_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_MIN_ALARM1_ADDR);
    return ((localVal & ((uint8_t)0x0000000F)) >> 0);
}

__INLINE void rtc_min_alarm1_min_alarm1_setf(uint8_t minalarm1)
{
    ASSERT_ERR((((uint8_t)minalarm1 << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_MIN_ALARM1_ADDR, (REG_RTC_RD(RTC_MIN_ALARM1_ADDR) & ~((uint8_t)0x0000000F)) | ((uint8_t)minalarm1 << 0));
}

/**
 * @brief HOUR_ALARM1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     07     HOUR_MASK_ALARM1   0
 *     06      HOURTYPE_ALARM1   0
 *  05:04       TENHOUR_ALARM1   0x0
 *  03:00          HOUR_ALARM1   0x0
 * </pre>
 */
#define RTC_HOUR_ALARM1_ADDR   0x0000000D
#define RTC_HOUR_ALARM1_OFFSET 0x0000000D
#define RTC_HOUR_ALARM1_INDEX  0x0000000D
#define RTC_HOUR_ALARM1_RESET  0x00000000

__INLINE uint8_t rtc_hour_alarm1_get(void)
{
    return REG_RTC_RD(RTC_HOUR_ALARM1_ADDR);
}

__INLINE void rtc_hour_alarm1_set(uint8_t value)
{
    REG_RTC_WR(RTC_HOUR_ALARM1_ADDR, value);
}

// field definitions
#define RTC_HOUR_MASK_ALARM1_BIT    ((uint8_t)0x00000080)
#define RTC_HOUR_MASK_ALARM1_POS    7
#define RTC_HOURTYPE_ALARM1_BIT     ((uint8_t)0x00000040)
#define RTC_HOURTYPE_ALARM1_POS     6
#define RTC_TENHOUR_ALARM1_MASK     ((uint8_t)0x00000030)
#define RTC_TENHOUR_ALARM1_LSB      4
#define RTC_TENHOUR_ALARM1_WIDTH    ((uint8_t)0x00000002)
#define RTC_HOUR_ALARM1_MASK        ((uint8_t)0x0000000F)
#define RTC_HOUR_ALARM1_LSB         0
#define RTC_HOUR_ALARM1_WIDTH       ((uint8_t)0x00000004)

#define RTC_HOUR_MASK_ALARM1_RST    0x0
#define RTC_HOURTYPE_ALARM1_RST     0x0
#define RTC_TENHOUR_ALARM1_RST      0x0
#define RTC_HOUR_ALARM1_RST         0x0

__INLINE void rtc_hour_alarm1_pack(uint8_t hourmaskalarm1, uint8_t hourtypealarm1, uint8_t tenhouralarm1, uint8_t houralarm1)
{
    ASSERT_ERR((((uint8_t)hourmaskalarm1 << 7) & ~((uint8_t)0x00000080)) == 0);
    ASSERT_ERR((((uint8_t)hourtypealarm1 << 6) & ~((uint8_t)0x00000040)) == 0);
    ASSERT_ERR((((uint8_t)tenhouralarm1 << 4) & ~((uint8_t)0x00000030)) == 0);
    ASSERT_ERR((((uint8_t)houralarm1 << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_HOUR_ALARM1_ADDR,  ((uint8_t)hourmaskalarm1 << 7) | ((uint8_t)hourtypealarm1 << 6) | ((uint8_t)tenhouralarm1 << 4) | ((uint8_t)houralarm1 << 0));
}

__INLINE void rtc_hour_alarm1_unpack(uint8_t* hourmaskalarm1, uint8_t* hourtypealarm1, uint8_t* tenhouralarm1, uint8_t* houralarm1)
{
    uint8_t localVal = REG_RTC_RD(RTC_HOUR_ALARM1_ADDR);

    *hourmaskalarm1 = (localVal & ((uint8_t)0x00000080)) >> 7;
    *hourtypealarm1 = (localVal & ((uint8_t)0x00000040)) >> 6;
    *tenhouralarm1 = (localVal & ((uint8_t)0x00000030)) >> 4;
    *houralarm1 = (localVal & ((uint8_t)0x0000000F)) >> 0;
}

__INLINE uint8_t rtc_hour_alarm1_hour_mask_alarm1_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_HOUR_ALARM1_ADDR);
    return ((localVal & ((uint8_t)0x00000080)) >> 7);
}

__INLINE void rtc_hour_alarm1_hour_mask_alarm1_setf(uint8_t hourmaskalarm1)
{
    ASSERT_ERR((((uint8_t)hourmaskalarm1 << 7) & ~((uint8_t)0x00000080)) == 0);
    REG_RTC_WR(RTC_HOUR_ALARM1_ADDR, (REG_RTC_RD(RTC_HOUR_ALARM1_ADDR) & ~((uint8_t)0x00000080)) | ((uint8_t)hourmaskalarm1 << 7));
}

__INLINE uint8_t rtc_hour_alarm1_hourtype_alarm1_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_HOUR_ALARM1_ADDR);
    return ((localVal & ((uint8_t)0x00000040)) >> 6);
}

__INLINE void rtc_hour_alarm1_hourtype_alarm1_setf(uint8_t hourtypealarm1)
{
    ASSERT_ERR((((uint8_t)hourtypealarm1 << 6) & ~((uint8_t)0x00000040)) == 0);
    REG_RTC_WR(RTC_HOUR_ALARM1_ADDR, (REG_RTC_RD(RTC_HOUR_ALARM1_ADDR) & ~((uint8_t)0x00000040)) | ((uint8_t)hourtypealarm1 << 6));
}

__INLINE uint8_t rtc_hour_alarm1_tenhour_alarm1_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_HOUR_ALARM1_ADDR);
    return ((localVal & ((uint8_t)0x00000030)) >> 4);
}

__INLINE void rtc_hour_alarm1_tenhour_alarm1_setf(uint8_t tenhouralarm1)
{
    ASSERT_ERR((((uint8_t)tenhouralarm1 << 4) & ~((uint8_t)0x00000030)) == 0);
    REG_RTC_WR(RTC_HOUR_ALARM1_ADDR, (REG_RTC_RD(RTC_HOUR_ALARM1_ADDR) & ~((uint8_t)0x00000030)) | ((uint8_t)tenhouralarm1 << 4));
}

__INLINE uint8_t rtc_hour_alarm1_hour_alarm1_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_HOUR_ALARM1_ADDR);
    return ((localVal & ((uint8_t)0x0000000F)) >> 0);
}

__INLINE void rtc_hour_alarm1_hour_alarm1_setf(uint8_t houralarm1)
{
    ASSERT_ERR((((uint8_t)houralarm1 << 0) & ~((uint8_t)0x0000000F)) == 0);
    REG_RTC_WR(RTC_HOUR_ALARM1_ADDR, (REG_RTC_RD(RTC_HOUR_ALARM1_ADDR) & ~((uint8_t)0x0000000F)) | ((uint8_t)houralarm1 << 0));
}

/**
 * @brief DAY_ALARM1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     07      DAY_MASK_ALARM1   0
 *  02:00           DAY_ALARM1   0x0
 * </pre>
 */
#define RTC_DAY_ALARM1_ADDR   0x0000000E
#define RTC_DAY_ALARM1_OFFSET 0x0000000E
#define RTC_DAY_ALARM1_INDEX  0x0000000E
#define RTC_DAY_ALARM1_RESET  0x00000000

__INLINE uint8_t rtc_day_alarm1_get(void)
{
    return REG_RTC_RD(RTC_DAY_ALARM1_ADDR);
}

__INLINE void rtc_day_alarm1_set(uint8_t value)
{
    REG_RTC_WR(RTC_DAY_ALARM1_ADDR, value);
}

// field definitions
#define RTC_DAY_MASK_ALARM1_BIT    ((uint8_t)0x00000080)
#define RTC_DAY_MASK_ALARM1_POS    7
#define RTC_DAY_ALARM1_MASK        ((uint8_t)0x00000007)
#define RTC_DAY_ALARM1_LSB         0
#define RTC_DAY_ALARM1_WIDTH       ((uint8_t)0x00000003)

#define RTC_DAY_MASK_ALARM1_RST    0x0
#define RTC_DAY_ALARM1_RST         0x0

__INLINE void rtc_day_alarm1_pack(uint8_t daymaskalarm1, uint8_t dayalarm1)
{
    ASSERT_ERR((((uint8_t)daymaskalarm1 << 7) & ~((uint8_t)0x00000080)) == 0);
    ASSERT_ERR((((uint8_t)dayalarm1 << 0) & ~((uint8_t)0x00000007)) == 0);
    REG_RTC_WR(RTC_DAY_ALARM1_ADDR,  ((uint8_t)daymaskalarm1 << 7) | ((uint8_t)dayalarm1 << 0));
}

__INLINE void rtc_day_alarm1_unpack(uint8_t* daymaskalarm1, uint8_t* dayalarm1)
{
    uint8_t localVal = REG_RTC_RD(RTC_DAY_ALARM1_ADDR);

    *daymaskalarm1 = (localVal & ((uint8_t)0x00000080)) >> 7;
    *dayalarm1 = (localVal & ((uint8_t)0x00000007)) >> 0;
}

__INLINE uint8_t rtc_day_alarm1_day_mask_alarm1_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_DAY_ALARM1_ADDR);
    return ((localVal & ((uint8_t)0x00000080)) >> 7);
}

__INLINE void rtc_day_alarm1_day_mask_alarm1_setf(uint8_t daymaskalarm1)
{
    ASSERT_ERR((((uint8_t)daymaskalarm1 << 7) & ~((uint8_t)0x00000080)) == 0);
    REG_RTC_WR(RTC_DAY_ALARM1_ADDR, (REG_RTC_RD(RTC_DAY_ALARM1_ADDR) & ~((uint8_t)0x00000080)) | ((uint8_t)daymaskalarm1 << 7));
}

__INLINE uint8_t rtc_day_alarm1_day_alarm1_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_DAY_ALARM1_ADDR);
    return ((localVal & ((uint8_t)0x00000007)) >> 0);
}

__INLINE void rtc_day_alarm1_day_alarm1_setf(uint8_t dayalarm1)
{
    ASSERT_ERR((((uint8_t)dayalarm1 << 0) & ~((uint8_t)0x00000007)) == 0);
    REG_RTC_WR(RTC_DAY_ALARM1_ADDR, (REG_RTC_RD(RTC_DAY_ALARM1_ADDR) & ~((uint8_t)0x00000007)) | ((uint8_t)dayalarm1 << 0));
}

/**
 * @brief CONTROL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     06        WRITE_PROTECT   0
 *     02      ONEHZ_OUTPUT_EN   0
 *     01        ALARM_INT_EN1   0
 *     00        ALARM_INT_EN0   0
 * </pre>
 */
#define RTC_CONTROL_ADDR   0x0000000F
#define RTC_CONTROL_OFFSET 0x0000000F
#define RTC_CONTROL_INDEX  0x0000000F
#define RTC_CONTROL_RESET  0x00000000

__INLINE uint8_t rtc_control_get(void)
{
    return REG_RTC_RD(RTC_CONTROL_ADDR);
}

__INLINE void rtc_control_set(uint8_t value)
{
    REG_RTC_WR(RTC_CONTROL_ADDR, value);
}

// field definitions
#define RTC_WRITE_PROTECT_BIT      ((uint8_t)0x00000040)
#define RTC_WRITE_PROTECT_POS      6
#define RTC_ONEHZ_OUTPUT_EN_BIT    ((uint8_t)0x00000004)
#define RTC_ONEHZ_OUTPUT_EN_POS    2
#define RTC_ALARM_INT_EN1_BIT      ((uint8_t)0x00000002)
#define RTC_ALARM_INT_EN1_POS      1
#define RTC_ALARM_INT_EN0_BIT      ((uint8_t)0x00000001)
#define RTC_ALARM_INT_EN0_POS      0

#define RTC_WRITE_PROTECT_RST      0x0
#define RTC_ONEHZ_OUTPUT_EN_RST    0x0
#define RTC_ALARM_INT_EN1_RST      0x0
#define RTC_ALARM_INT_EN0_RST      0x0

__INLINE void rtc_control_pack(uint8_t writeprotect, uint8_t onehzoutputen, uint8_t alarminten1, uint8_t alarminten0)
{
    ASSERT_ERR((((uint8_t)writeprotect << 6) & ~((uint8_t)0x00000040)) == 0);
    ASSERT_ERR((((uint8_t)onehzoutputen << 2) & ~((uint8_t)0x00000004)) == 0);
    ASSERT_ERR((((uint8_t)alarminten1 << 1) & ~((uint8_t)0x00000002)) == 0);
    ASSERT_ERR((((uint8_t)alarminten0 << 0) & ~((uint8_t)0x00000001)) == 0);
    REG_RTC_WR(RTC_CONTROL_ADDR,  ((uint8_t)writeprotect << 6) | ((uint8_t)onehzoutputen << 2) | ((uint8_t)alarminten1 << 1) | ((uint8_t)alarminten0 << 0));
}

__INLINE void rtc_control_unpack(uint8_t* writeprotect, uint8_t* onehzoutputen, uint8_t* alarminten1, uint8_t* alarminten0)
{
    uint8_t localVal = REG_RTC_RD(RTC_CONTROL_ADDR);

    *writeprotect = (localVal & ((uint8_t)0x00000040)) >> 6;
    *onehzoutputen = (localVal & ((uint8_t)0x00000004)) >> 2;
    *alarminten1 = (localVal & ((uint8_t)0x00000002)) >> 1;
    *alarminten0 = (localVal & ((uint8_t)0x00000001)) >> 0;
}

__INLINE uint8_t rtc_control_write_protect_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_CONTROL_ADDR);
    return ((localVal & ((uint8_t)0x00000040)) >> 6);
}

__INLINE void rtc_control_write_protect_setf(uint8_t writeprotect)
{
    ASSERT_ERR((((uint8_t)writeprotect << 6) & ~((uint8_t)0x00000040)) == 0);
    REG_RTC_WR(RTC_CONTROL_ADDR, (REG_RTC_RD(RTC_CONTROL_ADDR) & ~((uint8_t)0x00000040)) | ((uint8_t)writeprotect << 6));
}

__INLINE uint8_t rtc_control_onehz_output_en_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_CONTROL_ADDR);
    return ((localVal & ((uint8_t)0x00000004)) >> 2);
}

__INLINE void rtc_control_onehz_output_en_setf(uint8_t onehzoutputen)
{
    ASSERT_ERR((((uint8_t)onehzoutputen << 2) & ~((uint8_t)0x00000004)) == 0);
    REG_RTC_WR(RTC_CONTROL_ADDR, (REG_RTC_RD(RTC_CONTROL_ADDR) & ~((uint8_t)0x00000004)) | ((uint8_t)onehzoutputen << 2));
}

__INLINE uint8_t rtc_control_alarm_int_en1_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_CONTROL_ADDR);
    return ((localVal & ((uint8_t)0x00000002)) >> 1);
}

__INLINE void rtc_control_alarm_int_en1_setf(uint8_t alarminten1)
{
    ASSERT_ERR((((uint8_t)alarminten1 << 1) & ~((uint8_t)0x00000002)) == 0);
    REG_RTC_WR(RTC_CONTROL_ADDR, (REG_RTC_RD(RTC_CONTROL_ADDR) & ~((uint8_t)0x00000002)) | ((uint8_t)alarminten1 << 1));
}

__INLINE uint8_t rtc_control_alarm_int_en0_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_CONTROL_ADDR);
    return ((localVal & ((uint8_t)0x00000001)) >> 0);
}

__INLINE void rtc_control_alarm_int_en0_setf(uint8_t alarminten0)
{
    ASSERT_ERR((((uint8_t)alarminten0 << 0) & ~((uint8_t)0x00000001)) == 0);
    REG_RTC_WR(RTC_CONTROL_ADDR, (REG_RTC_RD(RTC_CONTROL_ADDR) & ~((uint8_t)0x00000001)) | ((uint8_t)alarminten0 << 0));
}

/**
 * @brief STATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     01                IRQF1   0
 *     00                IRQF0   0
 * </pre>
 */
#define RTC_STATUS_ADDR   0x00000010
#define RTC_STATUS_OFFSET 0x00000010
#define RTC_STATUS_INDEX  0x00000010
#define RTC_STATUS_RESET  0x00000000

__INLINE uint8_t rtc_status_get(void)
{
    return REG_RTC_RD(RTC_STATUS_ADDR);
}

// field definitions
#define RTC_IRQF1_BIT    ((uint8_t)0x00000002)
#define RTC_IRQF1_POS    1
#define RTC_IRQF0_BIT    ((uint8_t)0x00000001)
#define RTC_IRQF0_POS    0

#define RTC_IRQF1_RST    0x0
#define RTC_IRQF0_RST    0x0

__INLINE void rtc_status_unpack(uint8_t* irqf1, uint8_t* irqf0)
{
    uint8_t localVal = REG_RTC_RD(RTC_STATUS_ADDR);

    *irqf1 = (localVal & ((uint8_t)0x00000002)) >> 1;
    *irqf0 = (localVal & ((uint8_t)0x00000001)) >> 0;
}

__INLINE uint8_t rtc_status_irqf1_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_STATUS_ADDR);
    return ((localVal & ((uint8_t)0x00000002)) >> 1);
}

__INLINE uint8_t rtc_status_irqf0_getf(void)
{
    uint8_t localVal = REG_RTC_RD(RTC_STATUS_ADDR);
    return ((localVal & ((uint8_t)0x00000001)) >> 0);
}


#endif // _REG_RTC_H_

