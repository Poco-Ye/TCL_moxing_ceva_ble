/*
 * timer.c
 *
 *  Created on: 2021年12月24日
 *      Author: haijun.mai
 */

#include <ms_clock_hal.h>
#include "ms_timer.h"
#include <stddef.h>
#include "log.h"

#define TIMER_MAX_NUMBER    8

TimerHandle_Type timer_handle[TIMER_MAX_NUMBER];


int32_t timer0_interrupt_enable(void)
{
	/*enable timer interrupt*/
       ms_timer_int_enable( &timer_handle[0]);
	return STATUS_SUCCESS;
}

int32_t timer1_interrupt_enable(void)
{
	/*enable timer interrupt*/
       ms_timer_int_enable( &timer_handle[1]);
	return STATUS_SUCCESS;
}

int32_t timer2_interrupt_enable(void)
{
	/*enable timer interrupt*/
       ms_timer_int_enable( &timer_handle[2]);
	return STATUS_SUCCESS;
}

int32_t timer3_interrupt_enable(void)
{
	/*enable timer interrupt*/
       ms_timer_int_enable( &timer_handle[3]);
	return STATUS_SUCCESS;
}

int32_t timer4_interrupt_enable(void)
{
	/*enable timer interrupt*/
       ms_timer_int_enable( &timer_handle[4]);
	return STATUS_SUCCESS;
}

int32_t timer5_interrupt_enable(void)
{
	/*enable timer interrupt*/
       ms_timer_int_enable( &timer_handle[5]);
	return STATUS_SUCCESS;
}

int32_t timer6_interrupt_enable(void)
{
	/*enable timer interrupt*/
       ms_timer_int_enable( &timer_handle[6]);
	return STATUS_SUCCESS;
}

int32_t timer7_interrupt_enable(void)
{
	/*enable timer interrupt*/
       ms_timer_int_enable( &timer_handle[7]);
	return STATUS_SUCCESS;
}

int32_t timer0_interrupt_disable(void)
{
	/*disable timer interrupt*/
       ms_timer_int_disable( &timer_handle[0]);
	return STATUS_SUCCESS;
}

int32_t timer1_interrupt_disable(void)
{
	/*disable timer interrupt*/
       ms_timer_int_disable( &timer_handle[1]);
	return STATUS_SUCCESS;
}

int32_t timer2_interrupt_disable(void)
{
	/*disable timer interrupt*/
       ms_timer_int_disable( &timer_handle[2]);
	return STATUS_SUCCESS;
}

int32_t timer3_interrupt_disable(void)
{
	/*disable timer interrupt*/
       ms_timer_int_disable( &timer_handle[3]);
	return STATUS_SUCCESS;
}

int32_t timer4_interrupt_disable(void)
{
	/*disable timer interrupt*/
       ms_timer_int_disable( &timer_handle[4]);
	return STATUS_SUCCESS;
}

int32_t timer5_interrupt_disable(void)
{
	/*disable timer interrupt*/
       ms_timer_int_disable( &timer_handle[5]);
	return STATUS_SUCCESS;
}

int32_t timer6_interrupt_disable(void)
{
	/*disable timer interrupt*/
       ms_timer_int_disable( &timer_handle[6]);
	return STATUS_SUCCESS;
}

int32_t timer7_interrupt_disable(void)
{
	/*disable timer interrupt*/
       ms_timer_int_disable( &timer_handle[7]);
	return STATUS_SUCCESS;
}



void timer0_reach_calllback(TimerHandle_Type *tim)
{
    MS_LOGI(MS_DRIVER, "\r\n timer0 int!\n");
    ms_timer_clear_timer_interrupt_status(tim);
    //timer0_interrupt_disable();	
}
void timer1_reach_calllback(TimerHandle_Type *tim)
{
    MS_LOGI(MS_DRIVER, "\r\n timer1 int!\n");
    ms_timer_clear_timer_interrupt_status(tim);
    //timer1_interrupt_disable();	
}

void timer2_reach_calllback(TimerHandle_Type *tim)
{
    MS_LOGI(MS_DRIVER, "\r\n timer2 int!\n");
    ms_timer_clear_timer_interrupt_status(tim);
    //timer2_interrupt_disable();	
}

void timer3_reach_calllback(TimerHandle_Type *tim)
{
    MS_LOGI(MS_DRIVER, "\r\n timer3 int!\n");
    ms_timer_clear_timer_interrupt_status(tim);
    //timer3_interrupt_disable();	
}

void timer4_reach_calllback(TimerHandle_Type *tim)
{
    MS_LOGI(MS_DRIVER, "\r\n timer4 int!\n");
    ms_timer_clear_timer_interrupt_status(tim);
    //timer4_interrupt_disable();	
}

void timer5_reach_calllback(TimerHandle_Type *tim)
{
    MS_LOGI(MS_DRIVER, "\r\n timer5 int!\n");
    ms_timer_clear_timer_interrupt_status(tim);
    //timer5_interrupt_disable();	
}

void timer6_reach_calllback(TimerHandle_Type *tim)
{
     MS_LOGI(MS_DRIVER, "\r\n timer6 int!\n");
     ms_timer_clear_timer_interrupt_status(tim);
     //timer6_interrupt_disable();	
}

void timer7_reach_calllback(TimerHandle_Type *tim)
{
    MS_LOGI(MS_DRIVER, "\r\n timer7 int!\n");
    ms_timer_clear_timer_interrupt_status(tim);
    //timer7_interrupt_disable();	
}




void timer0_init_calllback(TimerHandle_Type *tim)
{
    /*config the timer clock*/
   MS_CLOCK_HAL_CLK_ENABLE_TIMERx(0);
	
    ms_clock_hal_set_timer0_div(0);
	
    ms_clock_hal_peripheral_clk_div_toggle(TIMER0_DIV_TOG);

    /*disable timer interrupt*/
    ms_timer_int_disable( tim);
	
    /*enable the cpu interrupt*/
    ms_timer_enable_cpu_interrupt(tim);
    
}

void timer1_init_calllback(TimerHandle_Type *tim)
{
    /*config the timer clock*/
   MS_CLOCK_HAL_CLK_ENABLE_TIMERx(1);
    ms_clock_hal_set_timer1_div(0);
	
    ms_clock_hal_peripheral_clk_div_toggle(TIMER1_DIV_TOG);
	
   /*disable timer interrupt*/
    ms_timer_int_disable( tim);
	
     /*enable the cpu interrupt*/
     ms_timer_enable_cpu_interrupt(tim);
    
}

void timer2_init_calllback(TimerHandle_Type *tim)
{
    /*config the timer clock*/
   MS_CLOCK_HAL_CLK_ENABLE_TIMERx(2);
    ms_clock_hal_set_timer2_div(0);
	
    ms_clock_hal_peripheral_clk_div_toggle(TIMER2_DIV_TOG);

    /*disable timer interrupt*/
    ms_timer_int_disable( tim);
  
    /*enable the cpu interrupt*/
     ms_timer_enable_cpu_interrupt(tim);
   
}

void timer3_init_calllback(TimerHandle_Type *tim)
{
    /*config the timer clock*/
   MS_CLOCK_HAL_CLK_ENABLE_TIMERx(3);
    ms_clock_hal_set_timer3_div(0);
	
    ms_clock_hal_peripheral_clk_div_toggle(TIMER3_DIV_TOG);

  /*disable timer interrupt*/
    ms_timer_int_disable( tim);

     /*enable the cpu interrupt*/
     ms_timer_enable_cpu_interrupt(tim);
   
}

void timer4_init_calllback(TimerHandle_Type *tim)
{
    /*config the timer clock*/
   MS_CLOCK_HAL_CLK_ENABLE_TIMERx(4);
    ms_clock_hal_set_timer4_div(0);
	
    ms_clock_hal_peripheral_clk_div_toggle(TIMER4_DIV_TOG);

    /*disable timer interrupt*/
    ms_timer_int_disable( tim);

     /*enable the cpu interrupt*/
     ms_timer_enable_cpu_interrupt(tim);
    
}

void timer5_init_calllback(TimerHandle_Type *tim)
{
    /*config the timer clock*/
   MS_CLOCK_HAL_CLK_ENABLE_TIMERx(5);
    ms_clock_hal_set_timer5_div(0);
	
    ms_clock_hal_peripheral_clk_div_toggle(TIMER5_DIV_TOG);
	
    /*disable timer interrupt*/
    ms_timer_int_disable( tim);

    /*enable the cpu interrupt*/
    ms_timer_enable_cpu_interrupt(tim);
    
}

void timer6_init_calllback(TimerHandle_Type *tim)
{
    /*config the timer clock*/
   MS_CLOCK_HAL_CLK_ENABLE_TIMERx(6);
    ms_clock_hal_set_timer6_div(0);
	
    ms_clock_hal_peripheral_clk_div_toggle(TIMER6_DIV_TOG);

   /*disable timer interrupt*/
    ms_timer_int_disable( tim);

    /*enable the cpu interrupt*/
    ms_timer_enable_cpu_interrupt(tim);
    
}

void timer7_init_calllback(TimerHandle_Type *tim)
{
    /*config the timer clock*/
    MS_CLOCK_HAL_CLK_ENABLE_TIMERx(7);
	
    ms_clock_hal_set_timer7_div(0);
	
    ms_clock_hal_peripheral_clk_div_toggle(TIMER7_DIV_TOG);

    /*disable timer interrupt*/
    ms_timer_int_disable( tim);


     /*enable the cpu interrupt*/
     ms_timer_enable_cpu_interrupt(tim);
    
}


void timer0_deinit_calllback(TimerHandle_Type *tim)
{
    /*disable the interrupt*/
    ms_timer_disable_cpu_interrupt(tim);

    /*config the timer clock*/
    MS_CLOCK_HAL_CLK_DISABLE_TIMERx(0);

      if(!ms_timer_get_int_mask_hal())	
    {
           /*enable the interrupt*/
         ms_timer_disable_cpu_interrupt(tim);
    }
}


void timer1_deinit_calllback(TimerHandle_Type *tim)
{
    /*disable the interrupt*/
    ms_timer_disable_cpu_interrupt(tim);

    /*config the timer clock*/
    MS_CLOCK_HAL_CLK_DISABLE_TIMERx(1);

       if(!ms_timer_get_int_mask_hal())	
    {
           /*enable the interrupt*/
         ms_timer_disable_cpu_interrupt(tim);
    }
}

void timer2_deinit_calllback(TimerHandle_Type *tim)
{
    /*disable the interrupt*/
    ms_timer_disable_cpu_interrupt(tim);

    /*config the timer clock*/
    MS_CLOCK_HAL_CLK_DISABLE_TIMERx(2);

      if(!ms_timer_get_int_mask_hal())	
    {
           /*enable the interrupt*/
         ms_timer_disable_cpu_interrupt(tim);
    }
}

void timer3_deinit_calllback(TimerHandle_Type *tim)
{
    /*disable the interrupt*/
    ms_timer_disable_cpu_interrupt(tim);

    /*config the timer clock*/
    MS_CLOCK_HAL_CLK_DISABLE_TIMERx(3);

      if(!ms_timer_get_int_mask_hal())	
    {
           /*enable the interrupt*/
         ms_timer_disable_cpu_interrupt(tim);
    }
}


void timer4_deinit_calllback(TimerHandle_Type *tim)
{
    /*disable the interrupt*/
    ms_timer_disable_cpu_interrupt(tim);

    /*config the timer clock*/
    MS_CLOCK_HAL_CLK_DISABLE_TIMERx(4);

       if(!ms_timer_get_int_mask_hal())	
    {
           /*enable the interrupt*/
         ms_timer_disable_cpu_interrupt(tim);
    }
}

void timer5_deinit_calllback(TimerHandle_Type *tim)
{
    /*disable the interrupt*/
    ms_timer_disable_cpu_interrupt(tim);

    /*config the timer clock*/
    MS_CLOCK_HAL_CLK_DISABLE_TIMERx(5);

       if(!ms_timer_get_int_mask_hal())	
    {
           /*enable the interrupt*/
         ms_timer_disable_cpu_interrupt(tim);
    }
}


void timer6_deinit_calllback(TimerHandle_Type *tim)
{
    /*disable the interrupt*/
    ms_timer_disable_cpu_interrupt(tim);

    /*config the timer clock*/
    MS_CLOCK_HAL_CLK_DISABLE_TIMERx(6);

       if(!ms_timer_get_int_mask_hal())	
    {
           /*enable the interrupt*/
         ms_timer_disable_cpu_interrupt(tim);
    }
}

void timer7_deinit_calllback(TimerHandle_Type *tim)
{
    /*disable the interrupt*/
    ms_timer_disable_cpu_interrupt(tim);

    /*config the timer clock*/
    MS_CLOCK_HAL_CLK_DISABLE_TIMERx(7);

      if(!ms_timer_get_int_mask_hal())	
    {
           /*enable the interrupt*/
         ms_timer_disable_cpu_interrupt(tim);
    }
}



TimerCallback_Type  timer0_callback =
{
       .init_callback = timer0_init_calllback,
	.deinit_callback = timer0_deinit_calllback,
	.timer_reach_callback = timer0_reach_calllback,
};

TimerCallback_Type  timer1_callback =
{
       .init_callback = timer1_init_calllback,
	.deinit_callback = timer1_deinit_calllback,
	.timer_reach_callback = timer1_reach_calllback,
};

TimerCallback_Type  timer2_callback =
{
       .init_callback = timer2_init_calllback,
	.deinit_callback = timer2_deinit_calllback,
	.timer_reach_callback = timer2_reach_calllback,
};

TimerCallback_Type  timer3_callback =
{
       .init_callback = timer3_init_calllback,
	.deinit_callback = timer3_deinit_calllback,
	.timer_reach_callback = timer3_reach_calllback,
};

TimerCallback_Type  timer4_callback =
{
       .init_callback = timer4_init_calllback,
	.deinit_callback = timer4_deinit_calllback,
	.timer_reach_callback = timer4_reach_calllback,
};

TimerCallback_Type  timer5_callback =
{
       .init_callback = timer5_init_calllback,
	.deinit_callback = timer5_deinit_calllback,
	.timer_reach_callback = timer5_reach_calllback,
};

TimerCallback_Type  timer6_callback =
{
       .init_callback = timer6_init_calllback,
	.deinit_callback = timer6_deinit_calllback,
	.timer_reach_callback = timer6_reach_calllback,
};


TimerCallback_Type  timer7_callback =
{
       .init_callback = timer7_init_calllback,
	.deinit_callback = timer7_deinit_calllback,
	.timer_reach_callback = timer7_reach_calllback,
};


int32_t timer0_init(void)
{
    timer_handle[0].instance = TIMER0;
    timer_handle[0].instance_conmon = TIMER;
	  
    timer_handle[0].init.period = 0x00100000;
    timer_handle[0].init.reload_mode = TIMER_USER_DEFINED;
  
    timer_handle[0].p_callback = &timer0_callback;
    timer_handle[0].irq = TIMER_IRQn;
	
    ms_timer_init(&timer_handle[0]);

    return STATUS_SUCCESS;
}


int32_t timer1_init(void)
{
    timer_handle[1].instance = TIMER1;
    timer_handle[1].instance_conmon = TIMER;
	  
    timer_handle[1].init.period = 0x00200000;
    timer_handle[1].init.reload_mode = TIMER_USER_DEFINED;
  
    timer_handle[1].p_callback = &timer1_callback;
    timer_handle[1].irq = TIMER_IRQn;
	
    ms_timer_init(&timer_handle[1]);
    return STATUS_SUCCESS;
}


int32_t timer2_init(void)
{
    timer_handle[2].instance = TIMER2;
    timer_handle[2].instance_conmon = TIMER;
	  
    timer_handle[2].init.period = 0x00300000;
    timer_handle[2].init.reload_mode = TIMER_USER_DEFINED;
  
    timer_handle[2].p_callback = &timer2_callback;
    timer_handle[2].irq = TIMER_IRQn;
	
    ms_timer_init(&timer_handle[2]);
    return STATUS_SUCCESS;
}



int32_t timer3_init(void)
{
    timer_handle[3].instance = TIMER3;
    timer_handle[3].instance_conmon = TIMER;
	  
    timer_handle[3].init.period = 0x00400000;
    timer_handle[3].init.reload_mode = TIMER_USER_DEFINED;
  
    timer_handle[3].p_callback = &timer3_callback;
    timer_handle[3].irq = TIMER_IRQn;
	
    ms_timer_init(&timer_handle[3]);
    return STATUS_SUCCESS;
}



int32_t timer4_init(void)
{
    timer_handle[4].instance = TIMER4;
    timer_handle[4].instance_conmon = TIMER;
	  
    timer_handle[4].init.period = 0x00500000;
    timer_handle[4].init.reload_mode = TIMER_USER_DEFINED;
  
    timer_handle[4].p_callback = &timer4_callback;
    timer_handle[4].irq = TIMER_IRQn;
	
    ms_timer_init(&timer_handle[4]);
    return STATUS_SUCCESS;
}




int32_t timer5_init(void)
{
    timer_handle[5].instance = TIMER5;
    timer_handle[5].instance_conmon = TIMER;
	  
    timer_handle[5].init.period = 0x00600000;
    timer_handle[5].init.reload_mode = TIMER_USER_DEFINED;
  
    timer_handle[5].p_callback = &timer5_callback;
    timer_handle[5].irq = TIMER_IRQn;
	
    ms_timer_init(&timer_handle[5]);
    return STATUS_SUCCESS;
}



int32_t timer6_init(void)
{
    timer_handle[6].instance = TIMER6;
    timer_handle[6].instance_conmon = TIMER;
	  
    timer_handle[6].init.period = 0x00700000;
    timer_handle[6].init.reload_mode = TIMER_USER_DEFINED;
  
    timer_handle[6].p_callback = &timer6_callback;
    timer_handle[6].irq = TIMER_IRQn;
	
    ms_timer_init(&timer_handle[6]);
    return STATUS_SUCCESS;
}



int32_t timer7_init(void)
{
    timer_handle[7].instance = TIMER7;
    timer_handle[7].instance_conmon = TIMER;
	  
    timer_handle[7].init.period = 0x00800000;
    timer_handle[7].init.reload_mode = TIMER_USER_DEFINED;
  
    timer_handle[7].p_callback = &timer7_callback;
    timer_handle[7].irq = TIMER_IRQn;
	
    ms_timer_init(&timer_handle[7]);
    return STATUS_SUCCESS;
}




int32_t timer0_start(void)
{
	ms_timer_start(&timer_handle[0]);
       return STATUS_SUCCESS;
}

int32_t timer1_start(void)
{
    ms_timer_start(&timer_handle[1]);
     return STATUS_SUCCESS;
}

int32_t timer2_start(void)
{
	ms_timer_start(&timer_handle[2]);
	return STATUS_SUCCESS;
}

int32_t timer3_start(void)
{
    ms_timer_start(&timer_handle[3]);
    return STATUS_SUCCESS;
}

int32_t timer4_start(void)
{
	ms_timer_start(&timer_handle[4]);
	return STATUS_SUCCESS;
}

int32_t timer5_start(void)
{
	ms_timer_start(&timer_handle[5]);
	return STATUS_SUCCESS;
}

int32_t timer6_start(void)
{
	ms_timer_start(&timer_handle[6]);
	return STATUS_SUCCESS;
}

int32_t timer7_start(void)
{
	ms_timer_start(&timer_handle[7]);
	return STATUS_SUCCESS;
}


int32_t timer0_stop(void)
{
	ms_timer_stop(&timer_handle[0]);
	return STATUS_SUCCESS;
}

int32_t timer1_stop(void)
{
	ms_timer_stop(&timer_handle[1]);
	return STATUS_SUCCESS;
}

int32_t timer2_stop(void)
{
	ms_timer_stop(&timer_handle[2]);
	return STATUS_SUCCESS;
}

int32_t timer3_stop(void)
{
	ms_timer_stop(&timer_handle[3]);
	return STATUS_SUCCESS;
}

int32_t timer4_stop(void)
{
	ms_timer_stop(&timer_handle[4]);
	return STATUS_SUCCESS;
}

int32_t timer5_stop(void)
{
	ms_timer_stop(&timer_handle[5]);
	return STATUS_SUCCESS;
}

int32_t timer6_stop(void)
{
	ms_timer_stop(&timer_handle[6]);
	return STATUS_SUCCESS;
}

int32_t timer7_stop(void)
{
	ms_timer_stop(&timer_handle[7]);
	return STATUS_SUCCESS;
}



int32_t timer0_relaod_value(int32_t reload_value)
{
	timer_handle[0].init.period = reload_value;
	ms_timer_reload(&timer_handle[0]);
	return STATUS_SUCCESS;
}

int32_t timer1_relaod_value(int32_t reload_value)
{
	timer_handle[1].init.period = reload_value;
	ms_timer_reload(&timer_handle[1]);
	return STATUS_SUCCESS;
}

int32_t timer2_relaod_value(int32_t reload_value)
{
	timer_handle[2].init.period = reload_value;
	ms_timer_reload(&timer_handle[2]);
	return STATUS_SUCCESS;
}

int32_t timer3_relaod_value(int32_t reload_value)
{
	timer_handle[3].init.period = reload_value;
	ms_timer_reload(&timer_handle[3]);
	return STATUS_SUCCESS;
}

int32_t timer4_relaod_value(int32_t reload_value)
{
	timer_handle[4].init.period = reload_value;
	ms_timer_reload(&timer_handle[4]);
	return STATUS_SUCCESS;
}

int32_t timer5_relaod_value(int32_t reload_value)
{
	timer_handle[5].init.period = reload_value;
	ms_timer_reload(&timer_handle[5]);
	return STATUS_SUCCESS;
}

int32_t timer6_relaod_value(int32_t reload_value)
{
	timer_handle[6].init.period = reload_value;
	ms_timer_reload(&timer_handle[6]);
	return STATUS_SUCCESS;
}

int32_t timer7_relaod_value(int32_t reload_value)
{
	timer_handle[7].init.period = reload_value;
	ms_timer_reload(&timer_handle[7]);
	return STATUS_SUCCESS;
}



int32_t timer0_set_mode(int32_t reload_mode)
{
	timer_handle[0].init.reload_mode= reload_mode;
	ms_timer_set_mode(&timer_handle[0]);
	return STATUS_SUCCESS;
}

int32_t timer1_set_mode(int32_t reload_mode)
{
	timer_handle[1].init.reload_mode= reload_mode;
	ms_timer_set_mode(&timer_handle[1]);
	return STATUS_SUCCESS;
}

int32_t timer2_set_mode(int32_t reload_mode)
{
	timer_handle[2].init.reload_mode= reload_mode;
	ms_timer_set_mode(&timer_handle[2]);
	return STATUS_SUCCESS;
}
int32_t timer3_set_mode(int32_t reload_mode)
{
	timer_handle[3].init.reload_mode= reload_mode;
	ms_timer_set_mode(&timer_handle[3]);
	return STATUS_SUCCESS;
}
int32_t timer4_set_mode(int32_t reload_mode)
{
	timer_handle[4].init.reload_mode= reload_mode;
	ms_timer_set_mode(&timer_handle[4]);
	return STATUS_SUCCESS;
}
int32_t timer5_set_mode(int32_t reload_mode)
{
	timer_handle[5].init.reload_mode= reload_mode;
	ms_timer_set_mode(&timer_handle[5]);
	return STATUS_SUCCESS;
}
int32_t timer6_set_mode(int32_t reload_mode)
{
	timer_handle[6].init.reload_mode= reload_mode;
	ms_timer_set_mode(&timer_handle[6]);
	return STATUS_SUCCESS;
}

int32_t timer7_set_mode(int32_t reload_mode)
{
	timer_handle[7].init.reload_mode= reload_mode;
	ms_timer_set_mode(&timer_handle[7]);
	return STATUS_SUCCESS;
}


uint32_t timer0_get_curvalue(void)
{
	return ms_timer_get_curvalue(&timer_handle[0]);
}

uint32_t timer1_get_curvalue(void)
{
	return ms_timer_get_curvalue(&timer_handle[1]);
}


uint32_t timer2_get_curvalue(void)
{
	return ms_timer_get_curvalue(&timer_handle[2]);
}

uint32_t timer3_get_curvalue(void)
{
	return ms_timer_get_curvalue(&timer_handle[3]);
}

uint32_t timer4_get_curvalue(void)
{
	return ms_timer_get_curvalue(&timer_handle[4]);
}

uint32_t timer5_get_curvalue(void)
{
	return ms_timer_get_curvalue(&timer_handle[5]);
}

uint32_t timer6_get_curvalue(void)
{
	return ms_timer_get_curvalue(&timer_handle[6]);
}

uint32_t timer7_get_curvalue(void)
{
	return ms_timer_get_curvalue(&timer_handle[7]);
}

int32_t timer0_get_timers_raw_status()
{
    return ms_timer_get_timers_raw_status(&timer_handle[0],TIMER0_TIS_STATUS);
}

int32_t timer1_get_timers_raw_status()
{
    return ms_timer_get_timers_raw_status(&timer_handle[1],TIMER1_TIS_STATUS);
}

int32_t timer2_get_timers_raw_status()
{
    return ms_timer_get_timers_raw_status(&timer_handle[2],TIMER2_TIS_STATUS);
}

int32_t timer3_get_timers_raw_status()
{
    return ms_timer_get_timers_raw_status(&timer_handle[3],TIMER3_TIS_STATUS);
}

int32_t timer4_get_timers_raw_status()
{
    return ms_timer_get_timers_raw_status(&timer_handle[4],TIMER4_TIS_STATUS);
}

int32_t timer5_get_timers_raw_status()
{
    return ms_timer_get_timers_raw_status(&timer_handle[5],TIMER5_TIS_STATUS);
}

int32_t timer6_get_timers_raw_status()
{
    return ms_timer_get_timers_raw_status(&timer_handle[6],TIMER6_TIS_STATUS);
}

int32_t timer7_get_timers_raw_status()
{
    return ms_timer_get_timers_raw_status(&timer_handle[7],TIMER7_TIS_STATUS);
}




int32_t timer0_deinit(void)
{
	ms_timer_deinit(&timer_handle[0]);
	return STATUS_SUCCESS;
}

int32_t timer1_deinit(void)
{
	ms_timer_deinit(&timer_handle[1]);
	return STATUS_SUCCESS;
}

int32_t timer2_deinit(void)
{
	ms_timer_deinit(&timer_handle[2]);
	return STATUS_SUCCESS;
}

int32_t timer3_deinit(void)
{
	ms_timer_deinit(&timer_handle[3]);
	return STATUS_SUCCESS;
}

int32_t timer4_deinit(void)
{
	ms_timer_deinit(&timer_handle[4]);
	return STATUS_SUCCESS;
}

int32_t timer5_deinit(void)
{
	ms_timer_deinit(&timer_handle[5]);
	return STATUS_SUCCESS;
}


int32_t timer6_deinit(void)
{
	ms_timer_deinit(&timer_handle[6]);
	return STATUS_SUCCESS;
}

int32_t timer7_deinit(void)
{
	ms_timer_deinit(&timer_handle[7]);
	return STATUS_SUCCESS;
}
void TIMER_IRQHandler(void)
{
    uint8_t value = 0 ;
    uint8_t i = 0 ;
    
    value = ms_timer_get_timers_interrupt_status();
    for(i=0;i<TIMER_MAX_NUMBER;i++)
    {
        if(value&(0x1UL<<i))
        {
            ms_timer_irq_handler(&timer_handle[i]);
        }
    }
}
