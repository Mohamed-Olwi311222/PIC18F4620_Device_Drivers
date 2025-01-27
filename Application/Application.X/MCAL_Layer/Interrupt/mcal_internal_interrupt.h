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
/*----------------------------Macros Functions Declarations-------------------*/
/*----------------ADC Interrupts----------------*/
#if ADC_INTERRUPT_FEATURE == INTERRUPT_FEATURE_ENABLE
/**
 * Clears the interrupt enable for the ADC module
 */
#define ADC_INTERRUPT_DISABLE()                       (PIE1bits.ADIE = 0)
/**
 * Sets the interrupt enable for the ADC module
 */
#define ADC_INTERRUPT_ENABLE()                        (PIE1bits.ADIE = 1)
/**
 * Clears the interrupt flag for the ADC module
 */
#define ADC_INTERRUPT_FLAG_BIT_CLEAR()                (PIR1bits.ADIF = 0)
#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
/**
 * ADC HIGH priority
 */
#define ADC_HIGH_PRIORITY()                           (IPR1bits.ADIP = 1)
/**
 * ADC LOW priority
 */
#define ADC_LOW_PRIORITY()                            (IPR1bits.ADIP = 0)
#endif
#endif

/*----------------TIMER0 Interrupt--------------*/
#if TIMER0_INTERRUPT_FEATURE == INTERRUPT_FEATURE_ENABLE
/**
 * Clears the interrupt enable for the TIMER0 module
 */
#define TIMER0_INTERRUPT_DISABLE()                    (INTCONbits.T0IE = 0)
/**
 * Sets the interrupt enable for the TIMER0 module
 */
#define TIMER0_INTERRUPT_ENABLE()                     (INTCONbits.T0IE = 1)
/**
 * Clears the interrupt flag for the TIMER0 module
 */
#define TIMER0_INTERRUPT_FLAG_BIT_CLEAR()             (INTCONbits.T0IF = 0)
#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
/**
 * TIMER0 HIGH priority
 */
#define TIMER0_HIGH_PRIORITY()                        (INTCON2bits.TMR0IP = 1)
/**
 * TIMER0 LOW priority
 */
#define TIMER0_LOW_PRIORITY()                         (INTCON2bits.TMR0IP = 0)
#endif
#endif

/*----------------TIMER1 Interrupt--------------*/
#if TIMER1_INTERRUPT_FEATURE == INTERRUPT_FEATURE_ENABLE
/**
 * Clears the interrupt enable for the TIMER1 module
 */
#define TIMER1_INTERRUPT_DISABLE()                    (PIE1bits.TMR1IE = 0)
/**
 * Sets the interrupt enable for the TIMER1 module
 */
#define TIMER1_INTERRUPT_ENABLE()                     (PIE1bits.TMR1IE = 1)
/**
 * Clears the interrupt flag for the TIMER1 module
 */
#define TIMER1_INTERRUPT_FLAG_BIT_CLEAR()             (PIR1bits.TMR1IF = 0)
#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
/**
 * TIMER1 HIGH priority
 */
#define TIMER1_HIGH_PRIORITY()                        (IPR1bits.TMR1IP = 1)
/**
 * TIMER1 LOW priority
 */
#define TIMER1_LOW_PRIORITY()                         (IPR1bits.TMR1IP = 0)
#endif
#endif

/*----------------TIMER2 Interrupt--------------*/
#if TIMER2_INTERRUPT_FEATURE == INTERRUPT_FEATURE_ENABLE
/**
 * Clears the interrupt enable for the TIMER2 module
 */
#define TIMER2_INTERRUPT_DISABLE()                    (PIE1bits.TMR2IE = 0)
/**
 * Sets the interrupt enable for the TIMER2 module
 */
#define TIMER2_INTERRUPT_ENABLE()                     (PIE1bits.TMR2IE = 1)
/**
 * Clears the interrupt flag for the TIMER2 module
 */
#define TIMER2_INTERRUPT_FLAG_BIT_CLEAR()             (PIR1bits.TMR2IF = 0)
#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
/**
 * TIMER2 HIGH priority
 */
#define TIMER2_HIGH_PRIORITY()                        (IPR1bits.TMR2IP = 1)
/**
 * TIMER2 LOW priority
 */
#define TIMER2_LOW_PRIORITY()                         (IPR1bits.TMR2IP = 0)
#endif
#endif

/*----------------TIMER3 Interrupt--------------*/
#if TIMER3_INTERRUPT_FEATURE == INTERRUPT_FEATURE_ENABLE
/**
 * Clears the interrupt enable for the TIMER3 module
 */
#define TIMER3_INTERRUPT_DISABLE()                    (PIE2bits.TMR3IE = 0)
/**
 * Sets the interrupt enable for the TIMER3 module
 */
#define TIMER3_INTERRUPT_ENABLE()                     (PIE2bits.TMR3IE = 1)
/**
 * Clears the interrupt flag for the TIMER3 module
 */
#define TIMER3_INTERRUPT_FLAG_BIT_CLEAR()             (PIR2bits.TMR3IF = 0)
#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
/**
 * TIMER3 HIGH priority
 */
#define TIMER3_HIGH_PRIORITY()                        (IPR2bits.TMR3IP = 1)
/**
 * TIMER3 LOW priority
 */
#define TIMER3_LOW_PRIORITY()                         (IPR2bits.TMR3IP = 0)
#endif
#endif

/*----------------CCP1 Interrupt----------------*/
#if CCP1_INTERRUPT_FEATURE == INTERRUPT_FEATURE_ENABLE
/**
 * Clears the interrupt enable for the CCP1 module
 */
#define CCP1_INTERRUPT_DISABLE()                        (PIE1bits.CCP1IE = 0)
/**
 * Sets the interrupt enable for the CCP1 module
 */
#define CCP1_INTERRUPT_ENABLE()                         (PIE1bits.CCP1IE = 1)
/**
 * Clears the interrupt flag for the CCP1 module
 */
#define CCP1_INTERRUPT_FLAG_BIT_CLEAR()                 (PIR1bits.CCP1IF = 0)
#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
/**
 * CCP1 HIGH priority
 */
#define CCP1_HIGH_PRIORITY()                            (IPR1bits.CCP1IP = 1)
/**
 * CCP1 LOW priority
 */
#define CCP1_LOW_PRIORITY()                             (IPR1bits.CCP1IP = 0)
#endif
#endif

/*----------------CCP2 Interrupt----------------*/
#if CCP2_INTERRUPT_FEATURE == INTERRUPT_FEATURE_ENABLE
/**
 * Clears the interrupt enable for the CCP2 module
 */
#define CCP2_INTERRUPT_DISABLE()                        (PIE2bits.CCP2IE = 0)
/**
 * Sets the interrupt enable for the CCP2 module
 */
#define CCP2_INTERRUPT_ENABLE()                         (PIE2bits.CCP2IE = 1)
/**
 * Clears the interrupt flag for the CCP2 module
 */
#define CCP2_INTERRUPT_FLAG_BIT_CLEAR()                 (PIR2bits.CCP2IF = 0)
#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
/**
 * CCP2 HIGH priority
 */
#define CCP2_HIGH_PRIORITY()                            (IPR2bits.CCP2IP = 1)
/**
 * CCP2 LOW priority
 */
#define CCP2_LOW_PRIORITY()                             (IPR2bits.CCP2IP = 0)
#endif
#endif
/*----------------EUSART Interrupt--------------*/
/*---Transmit Mode---*/
#if EUSART_TRANSMIT_INTERRUPT_FEATURE == INTERRUPT_FEATURE_ENABLE
/**
 * Clears the interrupt enable for the EUSART transmit mode
 */
#define EUSART_TRANSMIT_INTERRUPT_DISABLE()             (PIE1bits.TX1IE = 0)
/**
 * Sets the interrupt enable for the EUSART transmit mode
 */
#define EUSART_TRANSMIT_INTERRUPT_ENABLE()              (PIE1bits.TX1IE = 1)
#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
/**
 * EUSART transmit mode HIGH priority
 */
#define EUSART_TRANSMIT_HIGH_PRIORITY()                 (IPR1bits.TX1IP = 1)
/**
 * EUSART transmit mode LOW priority
 */
#define EUSART_TRANSMIT_LOW_PRIORITY()                  (IPR1bits.TX1IP = 0)
#endif
#endif

/*---Receive Mode----*/
#if EUSART_RECEIVE_INTERRUPT_FEATURE == INTERRUPT_FEATURE_ENABLE
/**
 * Clears the interrupt enable for the EUSART receive mode
 */
#define EUSART_RECEIVE_INTERRUPT_DISABLE()              (PIE1bits.RC1IE = 0)
/**
 * Sets the interrupt enable for the EUSART receive mode
 */
#define EUSART_RECEIVE_INTERRUPT_ENABLE()               (PIE1bits.RC1IE = 1)
/**
 * Clears the interrupt flag for the EUSART receive mode
 */
#define EUSART_RECEIVE_INTERRUPT_FLAG_BIT_CLEAR()       (PIR1bits.RC1IF = 0)
#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
/**
 * EUSART receive mode HIGH priority
 */
#define EUSART_RECEIVE_HIGH_PRIORITY()                  (IPR1bits.RC1IP = 1)
/**
 * EUSART receive mode LOW priority
 */
#define EUSART_RECEIVE_LOW_PRIORITY()                   (IPR1bits.RC1IP = 0)
#endif
#endif

/*----------------SPI Interrupt--------------*/
#if SPI_INTERRUPT_FEATURE == INTERRUPT_FEATURE_ENABLE
/**
 * Clears the interrupt enable for the SPI Module
 */
#define SPI_INTERRUPT_DISABLE()                         (PIE1bits.SSPIE = 0)
/**
 * Sets the interrupt enable for the SPI Module
 */
#define SPI_INTERRUPT_ENABLE()                          (PIE1bits.SSPIE = 1)
/**
 * Clears the Interrupt flag for the SPI Module
 */
#define SPI_INTERRUPT_FLAG_BIT_CLEAR()                  (PIR1bits.SSPIF = 0)
#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
/**
 * SPI Module HIGH priority
 */
#define SPI_HIGH_PRIORITY()                             (IPR1bits.SSPIP = 1)
/**
 * SPI Module LOW priority
 */
#define SPI_LOW_PRIORITY()                              (IPR1bits.SSPIP = 0)
#endif
#endif

/*----------------I2C Interrupt--------------*/
#if I2C_INTERRUPT_FEATURE == INTERRUPT_FEATURE_ENABLE
/**
 * Clears the interrupt enable for the I2C Module
 */
#define I2C_INTERRUPT_DISABLE()                         (PIE1bits.SSPIE = 0)
/**
 * Sets the interrupt enable for the I2C Module
 */
#define I2C_INTERRUPT_ENABLE()                          (PIE1bits.SSPIE = 1)
/**
 * Clears the Interrupt flag for the I2C Module
 */
#define I2C_INTERRUPT_FLAG_BIT_CLEAR()                  (PIR1bits.SSPIF = 0)
#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
/**
 * I2C Module HIGH priority
 */
#define I2C_HIGH_PRIORITY()                             (IPR1bits.SSPIP = 1)
/**
 * I2C Module LOW priority
 */
#define I2C_LOW_PRIORITY()                              (IPR1bits.SSPIP = 0)
#endif
#endif

#endif	/* MCAL_INTERNAL_INTERRUPT_H */

