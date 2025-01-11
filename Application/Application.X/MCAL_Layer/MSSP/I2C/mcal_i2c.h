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
#define _I2C_SLEW_RATE_CONTROL_DISABLE                       1 /* Slew rate control disabled for Standard Speed mode (100 kHz) */
#define _I2C_SLEW_RATE_CONTROL_ENABLE                        0 /* Slew rate control enabled for High-Speed mode (400 kHz) */
/*----------CKE Bit-----------*/
#define _I2C_SMBUS_ENABLE                                    1 /* Enable SMBus specific inputs */
#define _I2C_SMBUS_DISABLE                                   0 /* Disable SMBus specific inputs */
/*----------D/A Bit-----------*/
/*---Slave---*/
#define _I2C_DATA_COMM                                       1 /* Indicates that the last byte received or transmitted was data */
#define _I2C_ADDR_COMM                                       0 /* Indicates that the last byte received or transmitted was address */
/*----------P Bit-------------*/
#define _I2C_STOP_BIT_DETECTED                               1 /* Indicates that a Stop bit has been detected last */
#define _I2C_STOP_BIT_NOT_DETECTED                           0 /* Stop bit was not detected last */
/*----------S Bit-------------*/
#define _I2C_START_BIT_DETECTED                              1 /* Indicates that a Start bit has been detected last */
#define _I2C_START_BIT_NOT_DETECTED                          0 /* Start bit was not detected last */
/*----------R/W Bit-----------*/
/*---Slave---*/
#define _I2C_SLAVE_READ_MODE                                 1 /* In Slave mode Read */
#define _I2C_SLAVE_WRITE_MODE                                0 /* In Slave mode Write */
/*---Master---*/
#define _I2C_MASTER_TRANSMIT_IN_PROGRESS                     1 /* In Master mode Transmit is in progress */
#define _I2C_MASTER_TRANSMIT_IN_PROGRESS                     0 /* In Master mode Transmit is not in progress */
/*----------UA Bit------------*/
/*---Slave---*/
#define _I2C_SLAVE_ADDRESS_REQ_UPDATE                        1 /* Indicates that the user needs to update the address in the SSPADD register */
#define _I2C_SLAVE_ADDRESS_NOT_REQ_UPDATE                    0 /* Address does not need to be updated */
/*----------BF Bit------------*/
#define _I2C_RECEIVE_BUFFER_FULL                             1 /* Receive complete, SSPBUF is full */
#define _I2C_RECEIVE_BUFFER_EMPTY                            0 /* Receive not complete, SSPBUF is empty */
/*==================SSPCON1 REG================*/
/*----------WCOL Bit----------*/
/*---Master---*/
#define _I2C_MASTER_WRITE_COLLISION                          1 /* A write to the SSPBUF was attempted while the I2C conditions were not valid for a transmission to be started(must be cleared in software) */
#define _I2C_MASTER_WRITE_NO_COLLISION                       0 /* No collision */
/*---Slave---*/
#define _I2C_SLAVE_WRITE_COLLISION                           1 /* The SSPBUF register is written while it is still transmitting the previous word (must be cleared in software) */
#define _I2C_SLAVE_WRITE_NO_COLLISION                        0 /* No collision */
/*----------SSPOV Bit---------*/
#define _I2C_RECEIVE_OVERFLOW                                1 /* A new byte is received while the SSPBUF register is still holding the previous data */
#define _I2C_RECEIVE_NO_OVERFLOW                             0 /* No Overflow */
/*----------SSPEN Bit---------*/
#define _I2C_ENABLE_SERIAL_PORT                              1 /* Enables the serial port and configures the SDA and SCL pins as the serial port pins  */
#define _I2C_DISABLE_SERIAL_PORT                             0 /* Disables serial port and configures these pins as I/O port pins */
/*----------CKP Bit-----------*/
/*---Slave---*/
#define _I2C_SLAVE_RELEASE_CLK                               1 /* Releases clock */
#define _I2C_SLAVE_HOLD_CLK_LOW                              0 /* Holds clock low (clock stretch), used to ensure data setup time */
/*==================SSPCON2 REG================*/
/*----------CGEN Bit----------*/
/*---Slave---*/
#define _I2C_SLAVE_GENERAL_CALL_ENABLE                       1 /* Enables interrupt when a general call address (0000h) is received in the SSPSR */
#define _I2C_SLAVE_GENERAL_CALL_DISABLE                      0 /* General call address disabled */
/*----------ACKSTAT Bit-------*/
/*---Master---*/
#define _I2C_SLAVE_ACK_NOT_RECEIVED                          1 /* Acknowledge was not received from slave */
#define _I2C_SLAVE_ACK_RECEIVED                              0 /* Acknowledge was received from slave */
/*----------ACKDT Bit---------*/
/*---Master---*/
#define _I2C_MASTER_NACK_RECEIVED_DATA                       1 /* Not Acknowledge */
#define _I2C_MASTER_ACK_RECEIVED_DATA                        0 /* Acknowledge */
/*----------ACKEN Bit---------*/
/*---Master---*/
#define _I2C_MASTER_ACK_SEQUENCE_ENABLE                      1 /* Initiates Acknowledge sequence on SDA and SCL pins and transmit ACKDT data bit. Automatically cleared by hardware. */
#define _I2C_MASTER_ACK_SEQUENCE_DISABLE                     0 /* Acknowledge sequence Idle */
/*----------RCEN Bit----------*/
/*---Master---*/
#define _I2C_MASTER_RECEIVE_ENABLE                           1 /* Enables Receive mode for I2C */
#define _I2C_MASTER_RECEIVE_DISABLE                          0 /* Receive Idle */
/*----------PEN Bit-----------*/
/*---Master---*/
#define _I2C_MASTER_SEND_STOP_COND                           1 /* Initiates Stop condition on SDA and SCL pins. Automatically cleared by hardware. */
#define _I2C_MASTER_STOP_COND_IDLE                           0 /* Stop condition Idle */
/*----------RSEN Bit----------*/
/*---Master---*/
#define _I2C_MASTER_SEND_REPEATED_START_COND                 1 /* Initiates Repeated Start condition on SDA and SCL pins. Automatically cleared by hardware. */
#define _I2C_MASTER_REPEATED_START_COND_IDLE                 0 /* Repeated Start condition Idle */
/*----------SEN Bit-----------*/
/*---Master---*/
#define _I2C_MASTER_SEND_START_COND                          1 /* Initiates Start condition on SDA and SCL pins. Automatically cleared by hardware. */
#define _I2C_MASTER_START_COND_IDLE                          0 /* Start condition Idle */
/*---Slave---*/
#define _I2C_SLAVE_CLK_STRETCHING_ENABLE                     1 /* Clock stretching is enabled for both slave transmit and slave receive (stretch enabled) */
#define _I2C_SLAVE_CLK_STRETCHING_DISABLE                    0 /* Clock stretching is disabled */
/*----------------------------Macros Functions Declarations-------------------*/

/*----------------------------DataTypes---------------------------------------*/
/**
 * @brief: An enum for selecting I2C modes
 */
typedef enum
{
    I2C_SLAVE_MODE_10_BIT_ADDR_START_STOP_INTERRUPTS_ON = 0x0F,
    I2C_SLAVE_MODE_10_BIT_ADDR_START_STOP_INTERRUPTS_OFF = 0x07,
    I2C_SLAVE_MODE_7_BIT_ADDR_START_STOP_INTERRUPTS_ON = 0x0E,
    I2C_SLAVE_MODE_7_BIT_ADDR_START_STOP_INTERRUPTS_OFF = 0x06, 
    I2C_FIRMWARE_CONTROLLER_MASTER_MODE = 0x0B,
    I2C_MASTER_MODE
} i2c_mode_t;
/*----------------------------Function Prototypes-----------------------------*/
#endif	/* MCAL_I2C_H */

