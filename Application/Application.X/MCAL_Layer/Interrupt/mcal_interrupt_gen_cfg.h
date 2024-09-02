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
#define ADC_INTERRUPT_FEATURE                               INTERRUPT_FEATURE_ENABLE    /* ADC interrupt enable for preconfig */        
/*----------------END Internal Interrupts------------*/

/*----------------------------END Macros Declarations-------------------------*/


#endif	/* MCAL_INTERRUPT_GEN_CFG_H */

