#ifdef WIN32

#include "arch.h"
#include "stdbool.h"
#include "rwip.h"
#include "string.h"

#undef TRACE
extern int win_trace(const char* psz, ...);
#define TRACE(psz, ...) win_trace(psz, ##__VA_ARGS__)
#define PATH_CHAR '\\'

uint32_t componentdbg_level[3] = {5 ,5, 5 };    /* should init with configure tool*/

void RWIP_IRQHandler(void);
extern void rwip_isr()
{
	RWIP_IRQHandler();
}

_declspec(naked) extern int __wrap_printf(const char* psz, ...)
{
	__asm { jmp win_trace }
}


int atiny_get_log_level(void)
{
	return 0; // 0x7FFF;
}

const char * atiny_get_log_level_name(int log_level)
{
	return "UNKOWN";
}

void platform_reset(uint32_t error)
{

}

void  ms_keyscan_init(void (*func)())
{

}

void ms_log_write(int level, int compo, const char* format, ...)
{
	const char**p = (const char**)&format;
	__wrap_printf(p[0], p[1], p[2], p[3], p[4], p[5], p[6]);
}


uint32_t ECLIC_GetEnableIRQ(int IRQn)
{
	return 0;
}


void led_set(uint8_t led_number)
{

}

void led_reset(uint8_t led_number)
{

}

uint32_t ms_sys_check_if_enter_sleep()
{
	return 1;
}

void dsleep_timer_start(void)
{

}

void dsleep_timer_restart(void)
{
}

typedef struct
{
	bool is_allowed_to_enter_sleep;  /* to indicate whether to allow to enter sleep or not */
} ms_sleep_global_data;

extern ms_sleep_global_data g_sleep_data = { 0 };


uint32_t aux_adc_top_init(void)
{
	return 0;
}

uint32_t aux_adc_top_get_data(void)
{
	return 0;
}

uint16_t gatvs_send_report(uint8_t conidx, uint16_t att_idx, void* p_buf)
{
	return 0;
}

void gatvs_audio_stop(void)
{
}

#endif // WIN32