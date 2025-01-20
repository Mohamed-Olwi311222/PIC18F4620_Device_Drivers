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
#define _I2C_MASTER_TRANSMIT_NOT_IN_PROGRESS                 1 /* In Master mode Transmit is in progress */
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
/*==================SSPSTAT REG================*/
/*----------SMP Bit-----------*/
/**
 * Disable Slew rate control disabled for Standard Speed mode (100 kHz)
 */
#define I2C_SLEW_RATE_CONTROL_DISABLE_CONFIG()               (SSPSTATbits.SMP = _I2C_SLEW_RATE_CONTROL_DISABLE)
/**
 * Enable Slew rate control enabled for High-Speed mode (400 kHz) 
 */
#define I2C_SLEW_RATE_CONTROL_ENABLE_CONFIG()                (SSPSTATbits.SMP =  _I2C_SLEW_RATE_CONTROL_ENABLE)
/*----------CKE Bit-----------*/
/**
 * Enable SMBus specific inputs 
 */
#define I2C_SMBUS_ENABLE_CONFIG()                            (SSPSTATbits.CKE = _I2C_SMBUS_ENABLE)
/**
 * Disable SMBus specific inputs 
 */
#define I2C_SMBUS_DISABLE_CONFIG()                           (SSPSTATbits.CKE = _I2C_SMBUS_DISABLE)
/*----------D/A Bit-----------*/
/*---Slave---*/
/**
 * Read the type of the last byte received or transmitted and store it in an address given
 */
#define I2C_SLAVE_DATA_TYPE_CONFIG(__ADDR)                   (*__ADDR = SSPSTATbits.D_A)
/*----------P Bit-------------*/
/**
 * Read the status of the Stop bit and store it in an address given 
 */
#define I2C_READ_STOP_BIT_STATUS_CONFIG(__ADDR)              (*__ADDR = SSPSTATbits.P)
/*----------S Bit-------------*/
/**
 * Read the status of the Start bit and store it in an address given 
 */
#define I2C_READ_START_BIT_STATUS_CONFIG(__ADDR)             (*__ADDR = SSPSTATbits.S)
/*----------R/W Bit-----------*/
/*---Slave---*/
/**
 * Read the type of the operation of the slave and store it in an address given
 */
#define I2C_SLAVE_OPERATION_MODE_TYPE_CONFIG(__ADDR)         (*__ADDR = SSPSTATbits.R_W)
/*---Master---*/
/**
 * Read the status of the master transmission and store it in an address given
 */
#define I2C_MASTER_TRANSMISSION_STATUS_CONFIG(__ADDR)        (*__ADDR = SSPSTATbits.R_W)
/*----------UA Bit------------*/
/*---Slave---*/
/**
 * Read the status of the update address bit and store it in an address given
 */
#define I2C_SLAVE_UPDATE_ADDR_STATUS_CONFIG(__ADDR)          (*__ADDR = SSPSTATbits.UA)
/*----------BF Bit------------*/
/**
 * Read the Status of the I2C buffer register and store it in an address given
 */
#define I2C_READ_BUFFER_STATUS_CONFIG(__ADDR)                (*__ADDR = SSPSTATbits.BF)
/*==================SSPCON1 REG================*/
/*----------WCOL Bit----------*/
/*---Master---*/
/**
 * Read the Status of the I2C Master Write collision and store it in an address given
 */
#define I2C_MASTER_READ_WRITE_COL_STATUS_CONFIG(__ADDR)      (*__ADD = SSPCON1bits.WCOL)
/**
 * Clear the Status of the I2C Master Write collision
 */
#define I2C_MASTER_CLEAR_WRITE_COL_STATUS_CONFIG()           (SSPCON1bits.WCOL = _I2C_MASTER_WRITE_NO_COLLISION)
/*---Slave---*/
/**
 * Read the Status of the I2C Slave Write collision and store it in an address given
 */
#define I2C_SLAVE_READ_WRITE_COL_STATUS_CONFIG(__ADDR)       (*__ADD = SSPCON1bits.WCOL)
/**
 * Clear the Status of the I2C Slave Write collision bit
 */
#define I2C_SLAVE_CLEAR_WRITE_COL_STATUS_CONFIG()            (SSPCON1bits.WCOL = _I2C_SLAVE_WRITE_NO_COLLISION)
/*----------SSPOV Bit---------*/
/**
 * Read the Status of receive overflow bit and store it in an address given
 */
#define I2C_READ_RECEIVE_OVERFLOW_STATUS_CONFIG(__ADDR)      (*__ADD = SSPCON1bits.SSPOV)
/**
 * Clear the Status of the I2C receive overflow bit
 */
#define I2C_SLAVE_CLEAR_OVERFLOW_STATUS_CONFIG()             (SSPCON1bits.SSPOV = _I2C_RECEIVE_NO_OVERFLOW)
/*----------SSPEN Bit---------*/
/**
 * Enable the I2C serial port and configures the SDA and SCL pins as the serial port pins 
 */
#define I2C_SERIAL_PORT_ENABLE_CONFIG()                      (SSPCON1bits.SSPEN = _I2C_ENABLE_SERIAL_PORT)
/**
 * Disable the I2C serial port and configures the SDA and SCL pin as I/O port pins
 */
#define I2C_SERIAL_PORT_DISABLE_CONFIG()                     (SSPCON1bits.SSPEN = _I2C_DISABLE_SERIAL_PORT)
/*----------CKP Bit-----------*/
/*---Slave---*/
/**
 * Release the CLK given to the Slave
 */
#define I2C_SLAVE_RELEASE_CLK_CONFIG()                       (SSPCON1bits.CKP = _I2C_SLAVE_RELEASE_CLK)
/**
 * Holds the CLK Low(clock stretch) given to the Slave
 */
#define I2C_SLAVE_HOLD_CLK_CONFIG()                          (SSPCON1bits.CKP = _I2C_SLAVE_HOLD_CLK_LOW)
/*----------SSPM3:SSPM0 Bits--*/
/**
 * Set the I2C operation mode @ref i2c_mode_t
 */
#define I2C_SET_OPERATION_MODE(__MODE)                       (SSPCON1bits.SSPM = __MODE)
/*==================SSPCON2 REG================*/
/*----------CGEN Bit----------*/
/*---Slave---*/
/**
 * Enable General Call interrupt in I2C Slave Mode
 */
#define I2C_SLAVE_ENABLE_GENERAL_CALL_INTERRUPT_CONFIG()     (SSPCON2bits.GCEN = _I2C_SLAVE_GENERAL_CALL_ENABLE)
/**
 * Disable General Call interrupt in I2C Slave Mode
 */
#define I2C_SLAVE_DISABLE_GENERAL_CALL_INTERRUPT_CONFIG()    (SSPCON2bits.GCEN = _I2C_SLAVE_GENERAL_CALL_DISABLE)
/*----------ACKSTAT Bit-------*/
/*---Master---*/
/**
 * Read the ACK from the slave in I2C Master Transmit Mode
 */
#define I2C_MASTER_TRANSMIT_READ_ACK_STATUS_CONFIG(__ADDR)   (*__ADDR = SSPCON2bits.ACKSTAT)
/*----------ACKDT Bit---------*/
/*---Master---*/
/**
 * Set the ACK to be send after initiation to the slave in I2C Master Receive Mode
 */
#define I2C_MASTER_RECEIVE_SET_ACK_CONFIG()                  (SSPCON2bits.ACKDT = _I2C_MASTER_ACK_RECEIVED_DATA)
/**
 * Set the NACK to be send after initiation to the slave in I2C Master Receive Mode
 */
#define I2C_MASTER_RECEIVE_SET_NACK_CONFIG()                 (SSPCON2bits.ACKDT = _I2C_MASTER_NACK_RECEIVED_DATA)
/*----------ACKEN Bit---------*/
/*---Master---*/
/**
 * Initiates Acknowledge sequence on SDA and SCL pins and transmit ACKDT data bit.
 * Automatically cleared by hardware. 
 */
#define I2C_MASTER_RECEIVE_SEND_ACK_NACK_CONFIG()            (SSPCON2bits.ACKEN = _I2C_MASTER_ACK_SEQUENCE_ENABLE)
/*----------RCEN Bit----------*/
/*---Master---*/
/**
 * Enable I2C Master Receive Mode
 */
#define I2C_MASTER_ENABLE_RECEIVE_MODE_CONFIG()              (SSPCON2bits.RCEN = _I2C_MASTER_RECEIVE_ENABLE)
/**
 * Disable I2C Master Receive Mode
 */
#define I2C_MASTER_DISABLE_RECEIVE_MODE_CONFIG()             (SSPCON2bits.RCEN = _I2C_MASTER_RECEIVE_DISABLE)
/*----------PEN Bit-----------*/
/*---Master---*/
/**
 * Initiates Stop condition on SDA and SCL pins.
 * Automatically cleared by hardware. 
 */
#define I2C_MASTER_SEND_STOP_COND_CONFIG()                   (SSPCON2bits.PEN = _I2C_MASTER_SEND_STOP_COND)
/*----------RSEN Bit----------*/
/*---Master---*/
/**
 * Initiates Repeated Start condition on SDA and SCL pins.
 * Automatically cleared by hardware. 
 */
#define I2C_MASTER_SEND_REPEATED_START_CONFIG()              (SSPCON2bits.PEN = _I2C_MASTER_SEND_REPEATED_START_COND)
/*----------SEN Bit-----------*/
/*---Master---*/
/**
 * Initiates Start condition on SDA and SCL pins.
 * Automatically cleared by hardware. 
 */
#define I2C_MASTER_SEND_START_CONFIG()                       (SSPCON2bits.PEN = _I2C_MASTER_SEND_START_COND)
/*---Slave---*/
/**
 * Clock stretching is enabled for both slave transmit and slave receive (stretch enabled)
 */
#define I2C_SLAVE_ENABLE_CLK_STRETCH_CONFIG()                (SSPCON2bits.SEN = _I2C_SLAVE_CLK_STRETCHING_ENABLE)
/**
 * Clock stretching is disabled for both slave transmit and slave receive (stretch disabled)
 */
#define I2C_SLAVE_DISABLE_CLK_STRETCH_CONFIG()               (SSPCON2bits.SEN = _I2C_SLAVE_CLK_STRETCHING_DISABLE)
/*==================SSPADD REG=================*/
/**
 * Set the I2C Slave Address
 */
#define I2C_SLAVE_SET_ADDR_CONFIG(__SLAVE_ADDR)              (SSPADD = __SLAVE_ADDR)
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
    I2C_MASTER_MODE = 0x08
} i2c_mode_t;

typedef struct
{
#if I2C_INTERRUPT_FEATURE == INTERRUPT_ENABLE
    INTERRUPT_HANDLER i2c_interrupt;
#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
    interrupt_priority_cfg i2c_interrupt_priority;
#endif
#endif 
    i2c_mode_t i2c_mode;
    uint16 i2c_slave_mode_addr;
    uint8 i2c_slave_general_call_enable : 1;
    uint8 i2c_master_receive_enable : 1;
    uint8 i2c_smbus_enable : 1;
    uint8 i2c_slew_rate_control : 1;
    uint8 RESERVED : 4;
} i2c_t;
/*----------------------------Function Prototypes-----------------------------*/
/**
 * @brief: Initialize the I2C module
 * @param i2c_obj the I2C module object
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType i2c_init(const i2c_t *const i2c_obj);
#endif	/* MCAL_I2C_H */

