/* 
 * File:   mcal_interrupt_config.h
 * Author: Mohamed olwi
 *
 * Created on 01 August 2024, 06:51
 */

#ifndef MCAL_INTERRUPT_CONFIG_H
#define	MCAL_INTERRUPT_CONFIG_H
/*----------------------------Header Files------------------------------------*/
#include "xc.h"
#include "mcal_interrupt_gen_cfg.h"
#include "../mcal_std_types.h"
#include "../GPIO/hal_gpio.h"
/*----------------------------Macros Declarations-----------------------------*/

#define INTERRUPT_ENABLE                                1 /*Interrupt is enabled*/                   
#define INTERRUPT_DISABLE                               0 /*Interrupt is disabled*/
#define INTERRUPT_OCCUR                                 1 /*Interrupt has occured*/
#define INTERRUPT_NOT_OCCUR                             0 /*Interrupt hasnt occured*/
#define INTERRUPT_PRIORITY_ENABLE                       1 /*Interrupt Priority is enabled*/
#define INTERRUPT_PRIORITY_DISABLE                      1 /*Interrupt Priority is disabled*/
#define RBx_FLAG_TRUE                                   1 /*True Flag to the ISR*/
#define RBx_FLAG_FALSE                                  0 /*False Flag to the ISR*/
#define SPI_MASTER_MODE                                 2 /* A macro to chose the mode of the SPI Master ISR */
#define SPI_SLAVE_SEND_MODE                             1 /* A macro to chose the mode of the SPI Slave Send ISR */
#define SPI_SLAVE_RECEIVE_MODE                          0 /* A macro to chose the mode of the SPI Slave Receive ISR */
#define _I2C_START_COND_INTERRUPT                0 /* An indicator that the start condition happened */
#define _I2C_STOP_COND_INTERRUPT                 1 /* An indicator that the stop condition happened */
#define _I2C_ADDRESS_SENT_INTERRUPT              2 /* An indicator that the address has been sent */
#define _I2C_TRANSMIT_INTERRUPT                  3 /* An indicator that the transmit happened */
#define _I2C_RECEIVE_INTERRUPT                   4 /* An indicator that the receive happened */
#define _I2C_OPERATION_INTERRUPT                 5 /* An indicator that the current operation is done */
/*----------------------------Macros Functions Declarations-------------------*/

#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
/**
 * ENABLE priority levels on interrupts
 */
#define INTERRUPT_PRIORITY_levels_ENABLE()              (RCONbits.IPEN = 1)
/**
 * DISABLE priority levels on interrupts
 */
#define INTERRUPT_PRIORITY_levels_DISABLE()             (RCONbits.IPEN = 0)

/**
 * ENABLE LOW priority global interrupts
 */
#define INTERRUPT_Global_interrupt_LOW_ENABLE()         (INTCONbits.GIEL = 1)
/**
 * ENABLE LOW priority global interrupts
 */
#define INTERRUPT_Global_interrupt_LOW_DISABLE()        (INTCONbits.GIEL = 0)

/**
 * ENABLE HIGH priority global interrupts
 */
#define INTERRUPT_Global_interrupt_HIGH_ENABLE()        (INTCONbits.GIEH = 1)
/**
 * DISABLE HIGH priority global interrupts
 */
#define INTERRUPT_Global_interrupt_HIGH_DISABLE()       (INTCONbits.GIEH = 0)

#else
/**
 * ENABLE Periheral interrupts
 */
#define INTERRUPT_Peripheral_interrupt_ENABLE()         (INTCONbits.PEIE = 1)
/**
 * DISABLE Periheral interrupts
 */
#define INTERRUPT_Peripheral_interrupt_DISABLE()        (INTCONbits.PEIE = 0)
/**
 * ENABLE global interrupts
 */
#define INTERRUPT_Global_interrupt_ENABLE()             (INTCONbits.GIE = 1)
/**
 * DISABLE global interrupts
 */
#define INTERRUPT_Global_interrupt_DISABLE()            (INTCONbits.GIE = 0)
#endif
/*----------------------------DataTypes---------------------------------------*/
typedef void (*INTERRUPT_HANDLER) (void); /*Interrupt handler for callback functions*/

#if SPI_INTERRUPT_FEATURE == INTERRUPT_FEATURE_ENABLE
/* Extern variable to choose the correct SPI Slave function mode */
volatile uint8 SPI_MODE;
#endif 
#if I2C_INTERRUPT_FEATURE == INTERRUPT_FEATURE_ENABLE
/* variable to choose the correct I2C Master function mode */
volatile uint8 I2C_INTERRUPT_TYPE;
#endif 
/**
 * an enum accessible by all interrupts for controling the priority levels
 */
typedef enum
{
    INTERRUPT_LOW_PRIORITY = 0,                         /*Interrupt low priority*/
    INTERRUPT_HIGH_PRIORITY = 1,                        /*Interrupt High priority*/
}interrupt_priority_cfg;
/*----------------------------Function Prototypes-----------------------------*/
#endif	/* MCAL_INTERRUPT_CONFIG_H */

