/**
 * Copyright © 2021 by MooreSilicon.All rights reserved
 * @file  ms_syc_ctrl_interrupt.h
 * @brief
 * @author bingrui.chen
 * @date 2021-12-21
 * @version 1.0
 * @Revision
 */
#ifndef MS_INTERRUPT_H_
#define MS_INTERRUPT_H_

#include "NUCLEI_N.h"

/** @defgroup INTERRUPT_INTERFACE Interrupt Interface definition
 * @{
 */

#define INTERRUPT_ENABLE_GLOBAL_IRQ()           __enable_irq()
#define INTERRUPT_DISABLE_GLOBAL_IRQ()          __disable_irq()

#define INTERRUPT_ENABLE_IRQ(irq_n) 			do{  __enable_irq();ECLIC_EnableIRQ(irq_n); } while(0)
#define INTERRUPT_DISABLE_IRQ(irq_n)			ECLIC_DisableIRQ(irq_n)
#define INTERRUPT_GET_ENABLE_IRQ(irq_n) 		ECLIC_GetEnableIRQ(irq_n)
#define INTERRUPT_GET_PENDING_IRQ(irq_n)		ECLIC_GetPendingIRQ(irq_n)
#define INTERRUPT_SET_PENDING_IRQ(irq_n)		ECLIC_SetPendingIRQ(irq_n)
#define INTERRUPT_CLR_PENDING_IRQ(irq_n)		ECLIC_ClearPendingIRQ(irq_n)
#define INTERRUPT_SET_TRIG_IRQ(irq_n,trig)		ECLIC_SetTrigIRQ(irq_n,trig)
#define INTERRUPT_GET_TRIG_IRQ(irq_n)			ECLIC_GetTrigIRQ(irq_n)
#define INTERRUPT_SET_SHV_IRQ(irq_n,shv)		ECLIC_SetShvIRQ(irq_n,shv)
#define INTERRUPT_GET_SHV_IRQ(irq_n)			ECLIC_GetShvIRQ(irq_n)
#define INTERRUPT_SET_CTRL_IRQ(irq_n,intctrl)	ECLIC_SetCtrlIRQ(irq_n,intctrl)
#define INTERRUPT_GET_CTRL_IRQ(irq_n)			ECLIC_GetCtrlIRQ(irq_n)
#define INTERRUPT_SET_LEVEL_IRQ(irq_n,level)	ECLIC_SetLevelIRQ(irq_n,level)
#define INTERRUPT_GET_LEVEL_IRQ(irq_n)			ECLIC_GetLevelIRQ(irq_n)
#define INTERRUPT_SET_PRIORITY_IRQ(irq_n,pri)	ECLIC_SetPriorityIRQ(irq_n,pri)
#define INTERRUPT_GET_PRIORITY_IRQ(irq_n)		ECLIC_GetPriorityIRQ(irq_n)
#define INTERRUPT_SET_VECTOR(irq_n,vector)		ECLIC_SetVector(irq_n,vector)
#define INTERRUPT_GET_VECTOR(irq_n)				ECLIC_GetVector(irq_n)

/**
 * @}
 */


#endif /* MS_INTERRUPT_H_ */
