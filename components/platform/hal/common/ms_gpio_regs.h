/**
 * Copyright ? 2021 by MooreSilicon.All rights reserved
 * @file   ms_gpio_regs.h
 * @brief Header file of gpio  module.
 * @author haijun.mai
 * @date   2022-01-05
 * @version 1.0
 * @Revision
 */

#ifndef MS_GPIO_REGS_H_
#define MS_GPIO_REGS_H_

#include <ms1008.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_PORTA      0
#define GPIO_PORTB      1
#define GPIO_PORTC      2
#define GPIO_PORTD      3

typedef struct
{
    volatile uint32_t PXDR; //Port X(A-D) data register
    volatile uint32_t PXDDR; //Port X(A-D) Data Direction Register
    volatile uint32_t PXCTL; //Port X(A-D) data source register
} GpioPortx_Type;

typedef struct
{
    GpioPortx_Type PORTX[4];
    volatile uint32_t INTEN; //Interrupt enable register
    volatile uint32_t INTMASK; //Interrupt mask register
    volatile uint32_t INTTYPELEVEL; //Interrupt level
    volatile uint32_t INTPOLARITY; //Interrupt polarity
    volatile uint32_t INTSTATUS; //Interrupt status
    volatile uint32_t RAWINTSTATUS; //Raw interrupt status
    volatile uint32_t DEBOUNCE; //Debounce enable
    volatile uint32_t PAEOI; //Port A clear interrupt register
    volatile uint32_t EXTPX[4]; //External port X(A-D) register
} Gpio_Type;

/*Port A data register Bit Define*/
#define GPIO_PADR_GPIO_SWPORTA_DR_POS               					                       (0UL)
#define GPIO_PADR_GPIO_SWPORTA_DR_MASK      							                (0xFFUL << GPIO_PADR_GPIO_SWPORTA_DR_POS)
#define GPIO_PADR_GPIO_SWPORTA_DR	                                                                        (GPIO_PADR_GPIO_SWPORTA_DR_MASK)

/*Port A Data Direction Register Bit Define*/
#define GPIO_PADDR_GPIO_SWPORTA_DDR_POS               					                 (0UL)
#define GPIO_PADDR_GPIO_SWPORTA_DDR_MASK      							          (0xFFUL << GPIO_PADDR_GPIO_SWPORTA_DDR_POS)
#define GPIO_PADDR_GPIO_SWPORTA_DDR	                                                                         (GPIO_PADDR_GPIO_SWPORTA_DDR_MASK)

/*Port A data source register Bit Define*/
#define GPIO_PACTL_GPIO_SWPORTA_CTL_POS               					                       (0UL)
#define GPIO_PACTL_GPIO_SWPORTA_CTL_MASK      							         (0xFFUL << GPIO_PACTL_GPIO_SWPORTA_CTL_POS)
#define GPIO_PACTL_GPIO_SWPORTA_CTL	                                                                        (GPIO_PACTL_GPIO_SWPORTA_CTL_MASK)

/*Port B data register Bit Define*/
#define GPIO_PBDR_GPIO_SWPORTB_DR_POS               					                       (0UL)
#define GPIO_PBDR_GPIO_SWPORTB_DR_MASK      							                (0xFFUL << GPIO_PBDR_GPIO_SWPORTB_DR_POS)
#define GPIO_PBDR_GPIO_SWPORTB_DR	                                                                        (GPIO_PBDR_GPIO_SWPORTB_DR_MASK)

/*Port B Data Direction Register Bit Define*/
#define GPIO_PBDDR_GPIO_SWPORTB_DDR_POS               					                 (0UL)
#define GPIO_PBDDR_GPIO_SWPORTB_DDR_MASK      							          (0xFFUL << GPIO_PBDDR_GPIO_SWPORTB_DDR_POS)
#define GPIO_PBDDR_GPIO_SWPORTB_DDR	                                                                         (GPIO_PBDDR_GPIO_SWPORTB_DDR_MASK)

/*Port B data source register Bit Define*/
#define GPIO_PBCTL_GPIO_SWPORTB_CTL_POS               					                       (0UL)
#define GPIO_PBCTL_GPIO_SWPORTB_CTL_MASK      							                (0xFFUL << GPIO_PBCTL_GPIO_SWPORTB_CTL_POS)
#define GPIO_PBCTL_GPIO_SWPORTB_CTL	                                                                        (GPIO_PBCTL_GPIO_SWPORTB_CTL_MASK)

/*Port C data register Bit Define*/
#define GPIO_PCDR_GPIO_SWPORTC_DR_POS               					                       (0UL)
#define GPIO_PCDR_GPIO_SWPORTC_DR_MASK      							                (0xFFUL << GPIO_PCDR_GPIO_SWPORTC_DR_POS)
#define GPIO_PCDR_GPIO_SWPORTC_DR	                                                                        (GPIO_PCDR_GPIO_SWPORTC_DR_MASK)

/*Port C Data Direction Register Bit Define*/
#define GPIO_PCDDR_GPIO_SWPORTC_DDR_POS               					                 (0UL)
#define GPIO_PCDDR_GPIO_SWPORTC_DDR_MASK      							          (0xFFUL << GPIO_PCDDR_GPIO_SWPORTC_DDR_POS)
#define GPIO_PCDDR_GPIO_SWPORTC_DDR	                                                                         (GPIO_PCDDR_GPIO_SWPORTC_DDR_MASK)

/*Port C data source register Bit Define*/
#define GPIO_PCCTL_GPIO_SWPORTC_CTL_POS               					                       (0UL)
#define GPIO_PCCTL_GPIO_SWPORTC_CTL_MASK      							                (0xFFUL << GPIO_PCCTL_GPIO_SWPORTC_CTL_POS)
#define GPIO_PCCTL_GPIO_SWPORTC_CTL	                                                                        (GPIO_PCCTL_GPIO_SWPORTC_CTL_MASK)

/*Port D data register Bit Define*/
#define GPIO_PDDR_GPIO_SWPORTD_DR_POS               					                       (0UL)
#define GPIO_PDDR_GPIO_SWPORTD_DR_MASK      							                (0xFFUL << GPIO_PDDR_GPIO_SWPORTD_DR_POS)
#define GPIO_PDDR_GPIO_SWPORTD_DR	                                                                        (GPIO_PDDR_GPIO_SWPORTD_DR_MASK)

/*Port D Data Direction Register Bit Define*/
#define GPIO_PDDDR_GPIO_SWPORTD_DDR_POS               					                 (0UL)
#define GPIO_PDDDR_GPIO_SWPORTD_DDR_MASK      							          (0xFFUL << GPIO_PDDDR_GPIO_SWPORTD_DDR_POS)
#define GPIO_PDDDR_GPIO_SWPORTD_DDR	                                                                         (GPIO_PDDDR_GPIO_SWPORTD_DDR_MASK)

/*Port D data source register Bit Define*/
#define GPIO_PDCTL_GPIO_SWPORTD_CTL_POS               					                       (0UL)
#define GPIO_PDCTL_GPIO_SWPORTD_CTL_MASK      							         (0xFFUL << GPIO_PDCTL_GPIO_SWPORTD_CTL_POS)
#define GPIO_PDCTL_GPIO_SWPORTD_CTL	                                                                        (GPIO_PDCTL_GPIO_SWPORTD_CTL_MASK)

/*Interrupt enable register Bit Define*/
#define GPIO_INTEN_GPIO_INTEN_POS               					                             (0UL)
#define GPIO_INTEN_GPIO_INTEN_MASK      							                      (0xFFUL << GPIO_INTEN_GPIO_INTEN_POS)
#define GPIO_INTEN_GPIO_INTEN	                                                                                     (GPIO_INTEN_GPIO_INTEN_MASK)

/*Interrupt mask register Bit Define*/
#define GPIO_INTMASK_GPIO_INTMASK_POS               					                       (0UL)
#define GPIO_INTMASK_GPIO_INTMASK_MASK      							                (0xFFUL << GPIO_INTMASK_GPIO_INTMASK_POS)
#define GPIO_INTMASK_GPIO_INTMASK	                                                                        (GPIO_INTMASK_GPIO_INTMASK_MASK)

/*Interrupt level Bit Define*/
#define GPIO_INTTYPELEVEL_GPIO_INTTYPE_LEVEL_POS               					  (0UL)
#define GPIO_INTTYPELEVEL_GPIO_INTTYPE_LEVEL_MASK      					         (0xFFUL << GPIO_INTTYPELEVEL_GPIO_INTTYPE_LEVEL_POS)
#define GPIO_INTTYPELEVEL_GPIO_INTTYPE_LEVEL	                                                          (GPIO_INTTYPELEVEL_GPIO_INTTYPE_LEVEL_MASK)

/*Interrupt polarity Bit Define*/
#define GPIO_INTPOLARITY_GPIO_INT_POLARITY_POS               					         (0UL)
#define GPIO_INTPOLARITY_GPIO_INT_POLARITY_MASK      							  (0xFFUL << GPIO_INTPOLARITY_GPIO_INT_POLARITY_POS)
#define GPIO_INTPOLARITY_GPIO_INT_POLARITY	                                                          (GPIO_INTPOLARITY_GPIO_INT_POLARITY_MASK)

/*Interrupt status Bit Define*/
#define GPIO_INTSTATUS_GPIO_INTSTATUS_POS               					                (0UL)
#define GPIO_INTSTATUS_GPIO_INTSTATUS_MASK      							         (0xFFUL << GPIO_INTSTATUS_GPIO_INTSTATUS_POS)
#define GPIO_INTSTATUS_GPIO_INTSTATUS	                                                                 (GPIO_INTSTATUS_GPIO_INTSTATUS_MASK)

/*Raw interrupt status Bit Define*/
#define GPIO_RAWINTSTATUS_GPIO_RAW_INTSTATUS_POS               					  (0UL)
#define GPIO_RAWINTSTATUS_GPIO_RAW_INTSTATUS_MASK      						  (0xFFUL << GPIO_RAWINTSTATUS_GPIO_RAW_INTSTATUS_POS)
#define GPIO_RAWINTSTATUS_GPIO_RAW_INTSTATUS	                                                   (GPIO_RAWINTSTATUS_GPIO_RAW_INTSTATUS_MASK)

/*Debounce enable Bit Define*/
#define GPIO_DEBOUNCE_GPIO_DEBOUNCE_POS               					                 (0UL)
#define GPIO_DEBOUNCE_GPIO_DEBOUNCE_MASK      							          (0xFFUL << GPIO_DEBOUNCE_GPIO_DEBOUNCE_POS)
#define GPIO_DEBOUNCE_GPIO_DEBOUNCE	                                                                         (GPIO_DEBOUNCE_GPIO_DEBOUNCE_MASK)

/*Port A clear interrupt register Bit Define*/
#define GPIO_PAEOI_GPIO_PORTA_EOI_POS               					                       (0UL)
#define GPIO_PAEOI_GPIO_PORTA_EOI_MASK      							                (0xFFUL << GPIO_PAEOI_GPIO_PORTA_EOI_POS)
#define GPIO_PAEOI_GPIO_PORTA_EOI	                                                                               (GPIO_PAEOI_GPIO_PORTA_EOI_MASK)

/*External port A register Bit Define*/
#define GPIO_EXTPA_GPIO_EXT_PORTA_POS               					                       (0UL)
#define GPIO_EXTPA_GPIO_EXT_PORTA_MASK      							                (0xFFUL << GPIO_EXTPA_GPIO_EXT_PORTA_POS)
#define GPIO_EXTPA_GPIO_EXT_PORTA	                                                                               (GPIO_EXTPA_GPIO_EXT_PORTA_MASK)

/*External port B register Bit Define*/
#define GPIO_EXTPB_GPIO_EXT_PORTB_POS               					                       (0UL)
#define GPIO_EXTPB_GPIO_EXT_PORTB_MASK      							                (0xFFUL << GPIO_EXTPB_GPIO_EXT_PORTB_POS)
#define GPIO_EXTPB_GPIO_EXT_PORTB	                                                                               (GPIO_EXTPB_GPIO_EXT_PORTB_MASK)

/*External port C register Bit Define*/
#define GPIO_EXTPC_GPIO_EXT_PORTC_POS               					                       (0UL)
#define GPIO_EXTPC_GPIO_EXT_PORTC_MASK      							                (0xFFUL << GPIO_EXTPC_GPIO_EXT_PORTC_POS)
#define GPIO_EXTPC_GPIO_EXT_PORTC	                                                                               (GPIO_EXTPC_GPIO_EXT_PORTC_MASK)

/*External port D register Bit Define*/
#define GPIO_EXTPD_GPIO_EXT_PORTD_POS               					                       (0UL)
#define GPIO_EXTPD_GPIO_EXT_PORTD_MASK      							                (0xFFUL << GPIO_EXTPD_GPIO_EXT_PORTD_POS)
#define GPIO_EXTPD_GPIO_EXT_PORTD	                                                                               (GPIO_EXTPD_GPIO_EXT_PORTD_MASK)

#ifdef __cplusplus
}
#endif

#endif /* MS_GPIO_H_ */

