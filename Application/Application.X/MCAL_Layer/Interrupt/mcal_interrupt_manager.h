/* 
 * File:   mcal_interrupt_manager.h
 * Author: Mohamed olwi
 *
 * Created on 01 August 2024, 06:53
 */

#ifndef MCAL_INTERRUPT_MANAGER_H
#define	MCAL_INTERRUPT_MANAGER_H
/*----------------------------Header Files-----------------------------------------------------------------*/
#include "mcal_interrupt_config.h"
/*----------------------------Function Prototypes----------------------------------------------------------*/

/*--------------------------------INTx ISRS------------------------------------*/
/**
 * @brief: The interrupt service routine of INT0, will be called if INT0 interrupt has been raised
 */
void INT0_ISR(void);
/**
 * @brief: The interrupt service routine of INT1, will be called if INT0 interrupt has been raised
 */
void INT1_ISR(void);
/**
 * @brief: The interrupt service routine of INT2, will be called if INT0 interrupt has been raised
 */
void INT2_ISR(void);
/*--------------------------------RBx ISRS-------------------------------------*/

/**
 * @brief The interrupt service routine of RB4, will be called if RB4 on change interrupt has been raised depending on
 * the source of the calling
 * @param source the source of the calling from the interrupt manager
 */
void RB4_ISR(uint8 source);

/**
 * @brief The interrupt service routine of RB5, will be called if RB4 on change interrupt has been raised depending on
 * the source of the calling
 * @param source the source of the calling from the interrupt manager
 */
void RB5_ISR(uint8 source);

/**
 * @brief The interrupt service routine of RB6, will be called if RB4 on change interrupt has been raised depending on
 * the source of the calling
 * @param source the source of the calling from the interrupt manager
 */
void RB6_ISR(uint8 source);

/**
 * @brief The interrupt service routine of RB7, will be called if RB4 on change interrupt has been raised depending on
 * the source of the calling
 * @param source the source of the calling from the interrupt manager
 */
void RB7_ISR(uint8 source);
/*--------------------------------Internal Interrupts ISRS---------------------*/
/**
 * @brief the interrupt service routine of ADC module, will be called if the conversion is done
 */
void ADC_ISR(void);

/**
 * @brief the interrupt service routine of timer0 module, will be called if the overflow happened
 */
void TIMER0_ISR(void);

/**
 * @brief the interrupt service routine of timer1 module, will be called if the overflow happened
 */
void TIMER1_ISR(void);

/**
 * @brief the interrupt service routine of timer2 module, will be called if the TMR2 matches with PR2
 */
void TIMER2_ISR(void);

/**
 * @brief the interrupt service routine of timer3 module, will be called if the overflow happened
 */
void TIMER3_ISR(void);

/**
 * @brief the interrupt service routine of CCP1 module
 */
void CCP1_ISR(void);

/**
 * @brief the interrupt service routine of CCP2 module
 */
void CCP2_ISR(void);

/**
 * @brief the interrupt service routine of EUSART Transmit mode
 */
void EUSART_TX_ISR(void);

/**
 * @brief the interrupt service routine of EUSART Receive mode
 */
void EUSART_RX_ISR(void);

/**
 * @brief the interrupt service routine of SPI Module Master Mode
 */
void SPI_MASTER_ISR(void);
#endif	/* MCAL_INTERRUPT_MANAGER_H */

