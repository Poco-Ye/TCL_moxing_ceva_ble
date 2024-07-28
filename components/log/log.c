/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  log.c
 * @brief log management
 * @author pengyu.xue
 * @date 2021-1-10
 * @version 1.0
 * @Revision
 */

#include "log.h"
#include "uart.h"
#include "module_base_address.h"
#include "string.h"

#define DBG_TRACE_LOGLEN 512  // max debug out put
static unsigned char gb_printf_buf[DBG_TRACE_LOGLEN];

uint32_t componentdbg_level[MS_COMPONENT_MAX] = {5,5,5,5,5};    /* should init with configure tool*/



void ms_assert_write(const char* format,...);



/* log print with time stamp, to be added*/
uint32_t log_timestamp(void)
{
    return 0;
 //   return system time stamp;
}


/* multitask log need, to be added*/

void ms_log_impl_lock()
{
	// check the global mutex exit or not

	    // create the gloabl mutex if not exit

	// take the gloabl mutex  xSemaphoreTake

    return;
}

int ms_log_impl_lock_timeout()
{
	// check the global mutex exit or not

	    // create the gloabl mutex if not exit

	// take the gloabl mutex with  little timou setting
    return 1;
}

void ms_log_impl_unlock()
{
	// release
  // xSemaphoreGive(gloabl mutex);
    return;
}



/* trace in to uart0. default */
int uart_print_trace(const  char *format, va_list args )
{
	
    uint32_t len;

 //   va_list args;
 //   va_start(args, message);
    len = vsprintf((char*)gb_printf_buf, (const char*)format, args);
 //   va_end(args);
    
//    if(len >=  DBG_TRACE_LOGLEN)
//    		ms_assert_write("warning gb_printf_buf over flow %s\r\n", message);
    uart0_write(gb_printf_buf, len);
    return 0;
}


/* trace record in file system (log partition)   To be added*/
int fs_print_trace(const  char * format, va_list args)
{
	
}

/* trace in to PC. PC tool will analyse and display   To be added */
int pc_print_trace(const  char *format, va_list args)
{
	
}


/* should be configured later by PC confgiure tool*/
static vprintf_like_t ms_log_print_func = &uart_print_trace;

/* after PC configure, called during system start up stage to set the trace direction*/
void log_set_vprintf(vprintf_like_t func)
{
    ms_log_impl_lock();
    ms_log_print_func = func;
    ms_log_impl_unlock();
    return ;
}




void log_level_set(ComponentName_Type comp, LogLevel_Type level)
{
    ms_log_impl_lock();  // multitask lock
    componentdbg_level[comp] = level;  // run time 还是 global
    ms_log_impl_unlock();
}


void ms_log_write(LogLevel_Type level, ComponentName_Type compo, const char* format, ...)
{

    /* check should be output or not*/
//    if(componentdbg_level[compo] < level)
//    	return;
   // char timetick[10];
	
    if (!ms_log_impl_lock_timeout())
    {
        return;
    }

    va_list args;

   

	switch (compo)
	{
		case MS_DRIVER:     uart0_write("[DRIVER]", 8); break;
		case MS_BLUETOOTH:     uart0_write("[BT]", 4); break;
		case MS_FREERTOS:     uart0_write("[OS]", 4); break;
		case MS_POWERMANAGE:     uart0_write("[PM]", 4); break;
		case MS_OTA:     uart0_write("[OTA]", 5); break;
		default:  uart0_write("[DRIVER]", 8); break;	
	}
    
	
	switch (level)
	{
		case MS_LOG_ERROR:     uart0_write("ERROR:", 6); break;
		case MS_LOG_WARNING:     uart0_write("WARNING:", 8); break;
		case MS_LOG_INFO:     uart0_write("INFO:", 5); break;
		case MS_LOG_DEBUG:     uart0_write("DEBUG:", 6); break;
		case MS_LOG_VERBOSE:     uart0_write("VERBOSE:", 8); break;	
		default: uart0_write("INFO:", 5); break;	
	} 

	//sprintf(timetick, "%d",ms_sys_timer_get_systick());
    //uart0_write("time tick:", 10);
    //uart0_write(timetick, strlen(timetick));


    va_start(args, format);

    (*ms_log_print_func)(format, args);
    va_end(args);
    ms_log_impl_unlock();
}


void ms_assert_write(const char* format,...)
{		

    if (!ms_log_impl_lock_timeout())
    {
        return;
    }
	/* in final release, assert need to trigger HW reset, to be added*/
    va_list list;
    va_start(list, format);

    (*ms_log_print_func)(format, list);
    va_end(list);
    ms_log_impl_unlock();
}




/*DEBUG use assert**/ 
#ifdef DEBUG_USE_ASSERT
bool AssertCheckPointer(void* ptr)
{
    if ((((uint32_t)ptr >= RWIP_RAM_BASE_ADDR) && ((uint32_t)ptr <= RAM_ADDR_END)) ||
        (((uint32_t)ptr >= FLASH_BASE_ADDR) && ((uint32_t)ptr <= FLASH_BASE_ADDR_END)) ||
        (((uint32_t)ptr >= RET_RAM_BASE_ADDR) && ((uint32_t)ptr <= RET_RAM_BASE_ADDR_END)) ||
        (((uint32_t)ptr >= ROM_BASE_ADDR) && ((uint32_t)ptr <= ROM_BASE_ADDR_END))  ||  
	    (((uint32_t)ptr >= 0x40000000) && ((uint32_t)ptr <= 0x41000000)) )  // register address space
        return true;
    else
    {
		ms_assert_write("AssertCheckPointer failed, ptr is:%d\r\n", (uint32_t)ptr);
        return false;
    }
}


#ifdef DEBUG_TRACE_LINE
void AssertMsg(const char* message, const char* filename, uint32_t line)
{

    ms_assert_write("---------------------ASSERT-----------------\r\n");
    ms_assert_write("AssertDispMsg:  %s, file: %s, line: %d\r\n", message, filename, line);
    return;		/* assert occur */
	
}
#else
void AssertMsg(const char* message)
{

    ms_assert_write("---------------------ASSERT-----------------\r\n");
    ms_assert_write("AssertDispMsg: %s\r\n", message);
    return;		/* assert occur */
	
}
#endif   /* not define DEBUG_TRACE_LINE */

#endif  /* not define DEBUG_USE_ASSERT */





/*DEBUG use function trace 
	function trace only support uart trace
	Important: BT satck related trace need to use DBG_SWDIAG which will direct to EM
**/ 
#ifdef DEBUG_TRACE_FUNCTIONE
void FuncEnter(ComponentName_Type compo, const  char* funcname)
{
//	uart_print_trace("Enter function: %s\r\n",funcname);
    if(componentdbg_level[compo] == MS_LOG_VERBOSE)
        ms_log_write(MS_LOG_VERBOSE,  compo, "Enter function: %s\r\n", funcname);
    return;
}

void FuncLeave(ComponentName_Type compo, const  char* funcname)
{
//	uart_print_trace("Enter function: %s\r\n",funcname);
    if(componentdbg_level[compo] == MS_LOG_VERBOSE)
        ms_log_write(MS_LOG_VERBOSE,  compo, "Leave function: %s\r\n", funcname);
    return;
}

void FuncLeaveS(ComponentName_Type compo, const  char* funcname, const  char* retval)
{
    if(componentdbg_level[compo] == MS_LOG_VERBOSE)
        ms_log_write(MS_LOG_VERBOSE,  compo, "Leave function: %s, return value %s\r\n", funcname, retval);
    return;
}

void FuncLeaveD(ComponentName_Type compo, const  char* funcname, uint32_t retval)
{
    if(componentdbg_level[compo] == MS_LOG_VERBOSE)
        ms_log_write(MS_LOG_VERBOSE,  compo, "Leave function: %s, return value %d\r\n", funcname, retval);
    return;
}
#endif  /* not define DEBUG_TRACE_FUNCTIONE */
