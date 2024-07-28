/*
 * ms_driver_def.h
 *
 *  Created on: 2021年12月22日
 *      Author: bingrui.chen
 */

#ifndef MS_DRIVER_DEF_H_
#define MS_DRIVER_DEF_H_

#include "log.h"

#define STATUS_SUCCESS   0
#define STATUS_ERROR    -1
#define STATUS_TIMEOUT  -2
#define STATUS_BUSY  -3

#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */

#if defined ( __GNUC__ ) && !defined (__CC_ARM) /* GNU Compiler */
#ifndef __weak
#define __weak   __attribute__((weak))
#endif /* __weak */
#ifndef __packed
#define __packed __attribute__((__packed__))
#endif /* __packed */
#endif /* __GNUC__ */

/* Macro to get variable aligned on 4-bytes, for __ICCARM__ the directive "#pragma data_alignment=4" must be used instead */
#if defined ( __GNUC__ ) && !defined (__CC_ARM) /* GNU Compiler */
#ifndef __ALIGN_END
#define __ALIGN_END    __attribute__ ((aligned (4)))
#endif /* __ALIGN_END */
#ifndef __ALIGN_BEGIN
#define __ALIGN_BEGIN
#endif /* __ALIGN_BEGIN */
#else
#ifndef __ALIGN_END
#define __ALIGN_END
#endif /* __ALIGN_END */
#ifndef __ALIGN_BEGIN
#if defined   (__CC_ARM)      /* ARM Compiler */
#define __ALIGN_BEGIN    __align(4)
#elif defined (__ICCARM__)    /* IAR Compiler */
#define __ALIGN_BEGIN
#endif /* __CC_ARM */
#endif /* __ALIGN_BEGIN */
#endif /* __GNUC__ */

/*
#define CHECK_PTR_NULL(ptr)  do{if(ptr == NULL)return ;}while(0)

#define CHECK_PTR_NULL_RET(ptr,ret)  do{if(ptr == NULL){return ret;}}while(0)


#define CHECK_VAL_TRUE(value)  do{if((value))return;}while(0)

#define CHECK_VAL_TRUE_RET(value,ret)  do{if((value))return (ret);}while(0)
*/

#define CHECK_PTR_NULL(ptr)  MS_ASSERT_PTR(ptr, "pointer error")

#define CHECK_PTR_NULL_RET(ptr,ret)  MS_ASSERT_PTR_RETURN(ptr, "pointer error",ret)

//#define CHECK_VAL_TRUE(value)  MS_ASSERT_VAL(!value, "value error")

//#define CHECK_VAL_TRUE_RET(value,ret)  MS_ASSERT_VAL_RETURN(!value, "value error", ret)


#endif /* MS_DRIVER_DEF_H_ */
