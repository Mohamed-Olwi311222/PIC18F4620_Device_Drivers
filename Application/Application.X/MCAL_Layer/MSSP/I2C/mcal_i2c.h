/* 
 * File:   mcal_i2c.h
 * Author: Mohamed olwi
 *
 * Created on 31 December 2024, 19:43
 */

#ifndef MCAL_I2C_H
#define	MCAL_I2C_H
/*----------------------------Header Files------------------------------------*/
#include "../../mcal_std_types.h"
#include "../../GPIO/hal_gpio.h"
#include <xc.h>
#include "../../Interrupt/mcal_internal_interrupt.h"
#include "../../Interrupt/mcal_interrupt_manager.h"
#include "../../mcal_layer_cfg.h"
/*----------------------------Macros Declarations-----------------------------*/
/*==================SSPSTAT REG================*/
/*----------SMP Bit-----------*/

/*----------CKE Bit-----------*/

/*----------P Bit-------------*/

/*----------S Bit-------------*/

/*----------R/W Bit-----------*/
/*---Slave---*/
#define _I2C_SLAVE_READ_MODE                                 1 /* In Slave mode Read */
#define _I2C_SLAVE_WRITE_MODE                                0 /* In Slave mode Write */
/*---Master---*/
#define _I2C_MASTER_TRANSMIT_IN_PROGRESS                     1 /* In Master mode Transmit is in progress */
#define _I2C_MASTER_TRANSMIT_IN_PROGRESS                     0 /* In Master mode Transmit is not in progress */
/*----------UA Bit------------*/
#define _I2C_SLAVE_ADDRESS_REQ_UPDATE                        1 /* Indicates that the user needs to update the address in the SSPADD register */
#define _I2C_SLAVE_ADDRESS_NOT_REQ_UPDATE                    0 /* Address does not need to be updated */
/*----------BF Bit------------*/
#define _I2C_RECEIVE_BUFFER_FULL                             1 /* Receive complete, SSPBUF is full */
#define _I2C_RECEIVE_BUFFER_EMPTY                            0 /* Receive not complete, SSPBUF is empty */
/*----------------------------Macros Functions Declarations-------------------*/

/*----------------------------DataTypes---------------------------------------*/

/*----------------------------Function Prototypes-----------------------------*/
#endif	/* MCAL_I2C_H */

