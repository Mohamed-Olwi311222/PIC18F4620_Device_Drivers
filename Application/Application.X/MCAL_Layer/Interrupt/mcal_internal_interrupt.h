/* 
 * File:   mcal_internal_interrupt.h
 * Author: Mohamed olwi
 *
 * Created on 01 August 2024, 06:52
 */

#ifndef MCAL_INTERNAL_INTERRUPT_H
#define	MCAL_INTERNAL_INTERRUPT_H
/*----------------------------Header Files------------------------------------*/
#include "mcal_interrupt_config.h"

/*----------------------------Macros Declarations-----------------------------*/

/*----------------------------Macros Functions Declarations-------------------*/
#if ADC_INTERRUPT_FEATURE == INTERRUPT_FEATURE_ENABLE
/**
 * Clears the interrupt enable for the ADC module
 */
#define ADC_INTERRUPT_DISABLE()                     (PIE1bits.ADIE = 0)
/**
 * Sets the interrupt enable for the ADC module
 */
#define ADC_INTERRUPT_ENABLE()                      (PIE1bits.ADIE = 1)
/**
 * Clears the interrupt flag for the ADC module
 */
#define ADC_INTERRUPT_FLAG_BIT_CLEAR()             (PIR1bits.ADIF = 0)
#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
/**
 * ADC HIGH priority
 */
#define ADC_HIGH_PRIORITY()                        (IPR1bits.ADIP = 1)
/**
 * ADC LOW priority
 */
#define ADC_LOW_PRIORITY()                         (IPR1bits.ADIP = 0)
#endif
#endif

/*----------------------------DataTypes---------------------------------------*/

/*----------------------------Function Prototypes-----------------------------*/
#endif	/* MCAL_INTERNAL_INTERRUPT_H */

