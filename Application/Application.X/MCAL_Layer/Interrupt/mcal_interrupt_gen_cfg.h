/* 
 * File:   mcal_interrupt_gen_cfg.h
 * Author: Mohamed olwi
 *
 * Created on 01 August 2024, 06:51
 */

#ifndef MCAL_INTERRUPT_GEN_CFG_H
#define	MCAL_INTERRUPT_GEN_CFG_H
/*----------------------------Macros Declarations-----------------------------*/
#define INTERRUPT_FEATURE_ENABLE                            1                           /* Interrupt feature enable */
#define INTERRUPT_FEATURE_DISABLE                           0                           /* Interrupt feature disable */
#define INTERRUPT_PRIORITY_LEVELS_ENABLE                    INTERRUPT_FEATURE_ENABLE    /* Priority feature enable for preconfig */

/*----------------External Interrupts----------------*/
#define EXTERNAL_INTERRUPT_ENABLE                           INTERRUPT_FEATURE_DISABLE   /* Enable all external interrupts */
/*--------INTx--------*/
#define EXTERNAL_INTERRUPT_INTx_FEATURE_ENABLE              INTERRUPT_FEATURE_DISABLE   /* External interrupt enable for preconfig */
/*--------RBX---------*/
#define EXTERNAL_INTERRUPT_ONChange_FEATURE_ENABLE          INTERRUPT_FEATURE_DISABLE   /* On change interrupt enable for preconfig */
/*----------------END External Interrupts------------*/

/*----------------Internal Interrupts----------------*/
/*--------ADC Module--------*/
#define ADC_INTERRUPT_FEATURE                               INTERRUPT_FEATURE_DISABLE   /* ADC interrupt enable for preconfig */        

/*--------TIMER0 Module-----*/
#define TIMER0_INTERRUPT_FEATURE                            INTERRUPT_FEATURE_DISABLE   /* Timer0 interrupt enable for preconfig */        

/*--------TIMER1 Module-----*/
#define TIMER1_INTERRUPT_FEATURE                            INTERRUPT_FEATURE_DISABLE   /* Timer1 interrupt enable for preconfig */        

/*--------TIMER2 Module-----*/
#define TIMER2_INTERRUPT_FEATURE                            INTERRUPT_FEATURE_DISABLE   /* Timer2 interrupt enable for preconfig */        

/*--------TIMER3 Module-----*/
#define TIMER3_INTERRUPT_FEATURE                            INTERRUPT_FEATURE_DISABLE    /* Timer3 interrupt enable for preconfig */        

/*--------CCP1 Module-------*/
#define CCP1_INTERRUPT_FEATURE                              INTERRUPT_FEATURE_DISABLE   /* CCP1 interrupt enable for preconfig */        

/*--------CCP2 Module-------*/
#define CCP2_INTERRUPT_FEATURE                              INTERRUPT_FEATURE_DISABLE    /* CCP2 interrupt enable for preconfig */        

/*--------EUSART Module-----*/ 
#define EUSART_RECEIVE_INTERRUPT_FEATURE                    INTERRUPT_FEATURE_DISABLE     /* EUSART Receive mode interrupt enable for preconfig */        
#define EUSART_TRANSMIT_INTERRUPT_FEATURE                   INTERRUPT_FEATURE_DISABLE     /* EUSART Transmit mode interrupt enable for preconfig */        
#define EUSART_ERROR_INTERRUPTS_FEATURE                     INTERRUPT_FEATURE_DISABLE     /* EUSART Errors interrupts enable for preconfig */   
#define EUSART_FERR_INTERRUPT_FEATURE                       INTERRUPT_FEATURE_DISABLE     /* EUSART Framing error interrupt enable for preconfig */
#define EUSART_OERR_INTERRUPT_FEATURE                       INTERRUPT_FEATURE_DISABLE     /* EUSART Overrun error interrupt enable for preconfig */

/*--------SPI Module--------*/ 
#define SPI_INTERRUPT_FEATURE                               INTERRUPT_FEATURE_DISABLE     /* SPI Interrupt on receive/transmit enable for preconfig */
/*--------I2C Module--------*/ 
#define I2C_INTERRUPT_FEATURE                               INTERRUPT_FEATURE_DISABLE     /* I2C Interrupt on receive/transmit enable for preconfig */
/*----------------END Internal Interrupts------------*/

/*----------------------------END Macros Declarations-------------------------*/


#endif	/* MCAL_INTERRUPT_GEN_CFG_H */

