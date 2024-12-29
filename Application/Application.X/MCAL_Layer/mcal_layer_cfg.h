/* 
 * File:   mcal_layer_cfg.h
 * Author: Mohamed olwi
 *
 * Created on 18 October 2024, 03:07
 */

#ifndef MCAL_LAYER_CFG_H
#define	MCAL_LAYER_CFG_H
/*----------------------------Macros Declarations-----------------------------*/
#define MCAL_ENABLED                 1
#define MCAL_DISABLED                0
#define GPIO_MODULE_ENABLE           MCAL_ENABLED                               /* Precompile Config for GPIO Module to enable/disable it */
#define CCP1_MODULE_ENABLE           MCAL_DISABLED                              /* Precompile Config for CCP1 Module to enable/disable it */
#define CCP2_MODULE_ENABLE           MCAL_DISABLED                              /* Precompile Config for CCP2 Module to enable/disable it */
#define TIMER0_MODULE_ENABLE         MCAL_DISABLED                              /* Precompile Config for TIMER0 Module to enable/disable it */
#define TIMER1_MODULE_ENABLE         MCAL_DISABLED                              /* Precompile Config for TIMER1 Module to enable/disable it */
#define TIMER2_MODULE_ENABLE         MCAL_DISABLED                              /* Precompile Config for TIMER2 Module to enable/disable it */
#define TIMER3_MODULE_ENABLE         MCAL_DISABLED                              /* Precompile Config for TIMER3 Module to enable/disable it */
#define ADC_MODULE_ENABLE            MCAL_DISABLED                              /* Precompile Config for ADC Module to enable/disable it */
#define EEPROM_MODULE_ENABLE         MCAL_DISABLED                              /* Precompile Config for EEPROM Module to enable/disable it */
#define EUSART_MODULE_ENABLE         MCAL_DISABLED                              /* Precompile Config for EUSART Module to enable/disable it */
#define SPI_MODULE_ENABLE            MCAL_DISABLED                              /* Precompile Config for SPI Module to enable/disable it */
#endif	/* MCAL_LAYER_CFG_H */

