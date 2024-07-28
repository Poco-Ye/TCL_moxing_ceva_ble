/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  log.h
 * @brief
 * @author pengyu.xue
 * @date 2021-1-10
 * @version 1.0
 * @Revision
 */

/*
 *  examples:
 * 	    MS_LOGE( MS_DRIVER, "this is message E %s\r\n", string );
 *      MS_LOGW( MS_DRIVER, "this is message W %d\r\n", int );
 *	    MS_LOGI( MS_DRIVER, "this is message I %d\r\n", int );
 *	    MS_LOGD( MS_DRIVER, "this is message D %s\r\n", string );
 *	    MS_LOGV( MS_DRIVER, "this is message V %d\r\n", int );
 *	    MS_ASSERT_VAL(byte == 5, "byte should be 5\r\n");
 *	    MS_ASSERT_PTR(pointer, "pointer is illegal\r\n");
 *	    MS_FUNCTION_ENTER(MS_DRIVER, "function name");
 *	    MS_FUNCTION_LEAVE(MS_DRIVER, "function name");
 *	    MS_FUNCTION_RET_STR(MS_DRIVER, "function name", string);
 *	    MS_FUNCTION_RET_INT(MS_DRIVER, "function name", int)
 */


#ifndef __MS_LOG_H__
#define __MS_LOG_H__

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>



#ifdef __cplusplus
extern "C" {
#endif


#define DEBUG_TRACE_FUNCTIONE
#define DEBUG_USE_ASSERT
#define DEBUG_TRACE_LINE

/**
 * @brief log level definition 
 *
 */
typedef enum {
    MS_LOG_NONE,       /* No log*/
    MS_LOG_ERROR,      /* fatal error. reset in release version, also support storing in flash */
    MS_LOG_WARNING,    /* general error and warning  message, */
    MS_LOG_INFO,       /* general information for normal flow */
    MS_LOG_DEBUG,      /* extra debug information */
    MS_LOG_VERBOSE     /* verbose debug message, only used in SW develop stage */
} LogLevel_Type;


/**
 * @brief component name for log level definition 
 *
 */
typedef enum {
    MS_DRIVER,      
    MS_BLUETOOTH,   
    MS_FREERTOS, 
    MS_POWERMANAGE,
    MS_OTA,    
    MS_COMPONENT_MAX
} ComponentName_Type;

extern uint32_t componentdbg_level[MS_COMPONENT_MAX] ;    /* should init with configure tool*/


/* function pointer to the real pintf, can be  vsprintf etc*/
typedef int (*vprintf_like_t)(const char *, va_list);

/**
 * @brief set the log level according to tag
 * @param tag Tag of the log entries to enable. Must uint_tbe a non-NULL zero terminated string.
 *            Value "*" resets log level for all tags to the given value.
 * @param level  Selects log level to enable. Only logs at this and lower verbosity
 * levels will be shown.
 */
void log_level_set(ComponentName_Type comp, LogLevel_Type level);

/**
 * @brief Set function used to output log entries
 * By default, the vsprintf is used and UART is the destination 
 *
 * @param func new Function used for output. typically vsprintf.
 * @return func old Function used for output.
 */
void log_set_vprintf(vprintf_like_t func);


/**
 * @brief write log message
 *
 * Other component need to use MS_LOGE, MS_LOGW, MS_LOGI, MS_LOGD, MS_LOGV macros..
 * These macros will finally call to this write function.
 * should not be used in interrupt service routine.
 */
void ms_log_write(LogLevel_Type level, ComponentName_Type compo, const char* format, ...);


/**
 * @brief timestamp used in log output
 * using system time stamp 
 *
 * @return timestamp, in milliseconds
 */
uint32_t log_timestamp();


#define MS_LOGE( component, format, ... ) MS_LOG_RUNTIME_LEVEL(MS_LOG_ERROR,   component, format, ##__VA_ARGS__ )
#define MS_LOGW( component, format, ... ) MS_LOG_RUNTIME_LEVEL(MS_LOG_WARNING,    component, format, ##__VA_ARGS__ )
#define MS_LOGI( component, format, ... ) MS_LOG_RUNTIME_LEVEL(MS_LOG_INFO,    component, format, ##__VA_ARGS__)
#define MS_LOGD( component, format, ... ) MS_LOG_RUNTIME_LEVEL(MS_LOG_DEBUG,   component, format, ##__VA_ARGS__)
#define MS_LOGV( component, format, ... ) MS_LOG_RUNTIME_LEVEL(MS_LOG_VERBOSE, component, format, ##__VA_ARGS__)


#define MS_LOG_RUNTIME_LEVEL(level, compo, format, ...) do {               \
        if (componentdbg_level[compo] >= level ) MS_LOG_LEVEL(level, compo, format, ##__VA_ARGS__); \
    } while(0)
    
    
/* support time stamp or not */
#ifdef CONFIG_LOG_TIMESTAMP_SUPPORT
#define MS_LOG_LEVEL(level, component, format, ...) do {                     \
        if (level==MS_LOG_ERROR )             { ms_log_write(MS_LOG_ERROR,      component,  log_timestamp(),format, ##__VA_ARGS__); } \
        else if (level==MS_LOG_WARNING )      { ms_log_write(MS_LOG_WARNING,    component,  log_timestamp(),format, ##__VA_ARGS__); } \
        else if (level==MS_LOG_DEBUG )        { ms_log_write(MS_LOG_DEBUG,      component,  log_timestamp(),format, ##__VA_ARGS__); } \
        else if (level==MS_LOG_VERBOSE )      { ms_log_write(MS_LOG_VERBOSE,    component,  log_timestamp(),format, ##__VA_ARGS__); } \
        else                                  { ms_log_write(MS_LOG_INFO,       component,  log_timestamp(),format, ##__VA_ARGS__); } \
    } while(0)
#else
#define MS_LOG_LEVEL(level, component, format, ...) do {                     \
        if (level==MS_LOG_ERROR )             { ms_log_write(MS_LOG_ERROR,      component, format, ##__VA_ARGS__); } \
        else if (level==MS_LOG_WARNING )      { ms_log_write(MS_LOG_WARNING,    component, format, ##__VA_ARGS__); } \
        else if (level==MS_LOG_DEBUG )        { ms_log_write(MS_LOG_DEBUG,      component, format, ##__VA_ARGS__); } \
        else if (level==MS_LOG_VERBOSE )      { ms_log_write(MS_LOG_VERBOSE,    component, format, ##__VA_ARGS__); } \
        else                                  { ms_log_write(MS_LOG_INFO,       component, format, ##__VA_ARGS__); } \
    } while(0)
#endif //CONFIG_LOG_TIMESTAMP_SUPPORT


/* DEBUG use assert */
#ifdef DEBUG_USE_ASSERT

    bool AssertCheckPointer(void* ptr);

    #ifdef DEBUG_TRACE_LINE
        #define MAX_ASSERT_FLEN		50
		
		// deal with failure condition
        void AssertMsg(const char* message, const char* filename, uint32_t line);

        #define MS_ASSERT_VAL(_bool_, _msg_)    if (!(_bool_)) { AssertMsg(_msg_, __FILE__, (uint32_t)__LINE__);  }
		#define MS_ASSERT_VAL_RETURN(_bool_, _msg_, _ret_)    if (!(_bool_)) { AssertMsg(_msg_, __FILE__, (uint32_t)__LINE__);  return _ret_;}
        #define MS_ASSERT_PTR(_ptr_, _msg_)	if (!AssertCheckPointer(_ptr_)) { AssertMsg(_msg_, __FILE__, (uint32_t)__LINE__);  }
        #define MS_ASSERT_PTR_RETURN(_ptr_, _msg_, _ret_)	if (!AssertCheckPointer(_ptr_)) { AssertMsg(_msg_, __FILE__, (uint32_t)__LINE__);  return _ret_;}
    #else		/* not define DEBUG_TRACE_LINE */
		// deal with failure condition
        void AssertMsg(const char* message);

        #define MS_ASSERT_VAL(_bool_, _msg_)    if (!(_bool_)) { AssertMsg(_msg_);  }
		#define MS_ASSERT_VAL_RETURN(_bool_, _msg_, _ret_)    if (!(_bool_)) { AssertMsg(_msg_);  return _ret_;}		
        #define MS_ASSERT_PTR(_ptr_, _msg_)	if (!AssertCheckPointer(_ptr_)) { AssertMsg(_msg_); }
        #define MS_ASSERT_PTR_RETURN(_ptr_, _msg_, _ret_)	if (!AssertCheckPointer(_ptr_)) { AssertMsg(_msg_); return _ret_;}		
    #endif		/* end of OS DEBUG_TRACE_LINE */

#else		/* not define DEBUG_USE_ASSERT */
    #define MS_ASSERT_VAL(_bool_, _msg_)
    #define MS_ASSERT_VAL_RETURN(_bool_, _msg_, _ret_)	
    #define MS_ASSERT_PTR(_ptr_, _msg_)
    #define MS_ASSERT_PTR_RETURN(_ptr_, _msg_, _ret_)	
#endif   // end of DEBUG_USE_ASSERT


/* DEBUG use function trace  */
#ifdef DEBUG_TRACE_FUNCTIONE
    void FuncEnter(ComponentName_Type compo, const  char* funcname);
    void FuncLeave(ComponentName_Type compo, const  char* funcname);
    void FuncLeaveS(ComponentName_Type compo, const  char* funcname, const  char* retval);
    void FuncLeaveD(ComponentName_Type compo, const  char* funcname, uint32_t retval);

    #define MS_FUNCTION_ENTER(component,_funcname_)	            FuncEnter(component,_funcname_)
    #define MS_FUNCTION_LEAVE(component,_funcname_)             FuncLeave(component,_funcname_)
    #define MS_FUNCTION_RET_STR(component,_funcname_, _retval_) FuncLeaveS(component,_funcname_, _retval_)
    #define MS_FUNCTION_RET_INT(component,_funcname_, _retval_)	FuncLeaveD(component,_funcname_, _retval_)
#else    // not define DEBUG_TRACE_FUNCTIONE
    #define MS_FUNCTION_ENTER(component,_funcname_)
    #define MS_FUNCTION_LEAVE(component,_funcname_)
    #define MS_FUNCTION_RET_STR(component,_funcname_, _retval_)
    #define MS_FUNCTION_RET_INT(component,_funcname_, _retval_)
#endif    // end of define DEBUG_TRACE_FUNCTIONE


/* DEBUG output to file,  to be added */

#ifdef __cplusplus
}
#endif

#endif 
