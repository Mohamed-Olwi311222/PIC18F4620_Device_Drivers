/* 
 * File:   mcal_spi.h
 * Author: Mohamed olwi
 *
 * Created on 12 December 2024, 00:36
 */

#ifndef MCAL_SPI_H
#define	MCAL_SPI_H
/*----------------------------Header Files------------------------------------*/
#include "../../mcal_std_types.h"
#include "../../GPIO/hal_gpio.h"
#include <xc.h>
#include "../../Interrupt/mcal_internal_interrupt.h"
#include "../../mcal_layer_cfg.h"
/*----------------------------Macros Declarations-----------------------------*/
#define SPI_DUMMY_DATA                                    0xFF /* Dummy data to send in master mode to initiate the CLK line */
/*==================SSPSTAT REG================*/
/*----------SMP Bit-----------*/
#define _SPI_SAMPLE_INPUT_END                                1 /* Input data sampled at end of data output time */
#define _SPI_SAMPLE_INPUT_MIDDLE                             0 /* Input data sampled at middle of data output time */                  
/*----------CKE Bit-----------*/
#define _SPI_TRANSITION_ACTIVE_IDLE                          1 /* Transmit occurs on transition from active to Idle clock state */
#define _SPI_TRANSITION_IDLE_ACTIVE                          0 /* Transmit occurs on transition from Idle to active clock state */
/*----------BF Bit------------*/
#define _SPI_RECEIVE_BUFFER_FULL                             1 /* Receive complete, SSPBUF is full */
#define _SPI_RECEIVE_BUFFER_EMPTY                            0 /* Receive not complete, SSPBUF is empty */
/*==================SSPCON1 REG================*/
/*----------WCOL Bit----------*/
#define _SPI_WRITE_COLLISION                                 1 /* The SSPxBUF register is written while it is still transmitting the previous word (must be cleared in software) */
#define _SPI_WRITE_NO_COLLISION                              0 /* No collision */
/*----------SSPOV Bit---------*/
#define _SPI_SLAVE_RECEIVE_OVERFLOW                          1 /* A new byte is received while the SSPBUF register is still holding the previous data */
#define _SPI_SLAVE_RECEIVE_NO_OVERFLOW                       0 /* No Overflow */
/*----------SSPEN Bit---------*/
#define _SPI_ENABLE_SERIAL_PORT                              1 /* Enables serial port and configures SCK, SDO, SDI and SS as serial port pins */
#define _SPI_DISABLE_SERIAL_PORT                             0 /* Disables serial port and configures these pins as I/O port pins */
/*----------CKP Bit-----------*/
#define _SPI_IDLE_STATE_HIGH                                 1 /* Idle state for clock is a high level */
#define _SPI_IDLE_STATE_LOW                                  0 /* Idle state for clock is a low level */
/*----------------------------Macros Functions Declarations-------------------*/
/*==================SSPSTAT REG================*/
/*----------SMP Bit-----------*/
/**
 * Sample the input data at the end of data output time
 */
#define SPI_SAMPLE_INPUT_END_CONFIG()                        (SSPSTATbits.SMP = _SPI_SAMPLE_INPUT_END)
/**
 * Sample the input data at the middle of data output time
 * @Note: SPI slave mode must be at the middle(SMP must be cleared when SPI is used in Slave mode.)
 */
#define SPI_SAMPLE_INPUT_MIDDLE_CONFIG()                     (SSPSTATbits.SMP = _SPI_SAMPLE_INPUT_MIDDLE)
/*----------CKE Bit-----------*/
/**
 * Set the transmission on transition of active to idle state
 */
#define SPI_TRANSITION_ACTIVE_IDLE_CONFIG()                  (SSPSTATbits.CKE = _SPI_TRANSITION_ACTIVE_IDLE)
/**
 * Set the transmission on transition of idle to active state
 */
#define SPI_TRANSITION_IDLE_ACTIVE_CONFIG()                  (SSPSTATbits.CKE = _SPI_TRANSITION_IDLE_ACTIVE)
/*----------BF Bit------------*/
/**
 * Read the Status of the SPI buffer register and store it in an address given
 */
#define SPI_READ_BUFFER_STATUS_CONFIG(__ADD)                 (*__ADDR = SSPSTATbits.BF)
/*==================SSPCON1 REG================*/
/*----------WCOL Bit----------*/
/**
 * Read the status of the SPI write collision and store it in an address given
 */
#define SPI_READ_COLLISION_STATUS_CONFIG(__ADD)              (*__ADDR = SSPCON1bits.WCOL)
/**
 * Clear the status of the SPI write collision
 */
#define SPI_CLEAR_COLLISION_STATUS_CONFIG()                  (SSPCON1bits.WCOL = _SPI_WRITE_NO_COLLISION)
/*----------SSPOV Bit---------*/
/**
 * Read the status of the SPI slave mode receive overflow and store it in an address given
 */
#define SPI_SLAVE_READ_RECEIVE_OVERFLOW_STATUS_CONFIG(__ADD) (*__ADDR = SSPCON1bits.SSPOV) 
/**
 * Clear the status of the SPI slave mode receive overflow 
 */
#define SPI_SLAVE_CLEAR_RECEIVE_OVERFLOW_STATUS_CONFIG()     (SSPCON1bits.SSPOV = _SPI_SLAVE_RECEIVE_NO_OVERFLOW) 
/*----------SSPEN Bit---------*/
/**
 * Enable the SPI serial port and configures SCK, SDO, SDI and SS as serial port pins 
 */
#define SPI_SERIAL_PORT_ENABLE_CONFIG()                      (SSPCON1bits.SSPEN = _SPI_ENABLE_SERIAL_PORT)
/**
 * Enable the SPI serial port and configures SCK, SDO, SDI and SS as I/O port pins
 */
#define SPI_SERIAL_PORT_DISABLE_CONFIG()                     (SSPCON1bits.SSPEN = _SPI_DISABLE_SERIAL_PORT)
/*----------CKP Bit-----------*/
/**
 * Configures the idle state for the clock to be high
 */
#define SPI_IDLE_STATE_HIGH_CONFIG()                         (SSPCON1bits.CKP = _SPI_IDLE_STATE_HIGH)
/**
 * Configures the idle state for the clock to be low
 */
#define SPI_IDLE_STATE_LOW_CONFIG()                          (SSPCON1bits.CKP = _SPI_IDLE_STATE_LOW)
/*----------SSPM3:SSPM0 Bits--*/
/**
 * Set the SPI operation mode @ref spi_mode_select_t
 */
#define SPI_SET_OPERATION_MODE(__MODE)                       (SSPCON1bits.SSPM = __MODE)
/*----------------------------DataTypes---------------------------------------*/
/**
 * @brief: An enum for selecting SPI modes
 * Note1: FOSC_x means FOSC/x and TMR2_2 is TMR2 output/2
 */
typedef enum
{
    SPI_MASTER_MODE_CLK_FOSC_4,
    SPI_MASTER_MODE_CLK_FOSC_16,
    SPI_MASTER_MODE_CLK_FOSC_64,
    SPI_MASTER_MODE_CLK_TMR2_2,
    SPI_SLAVE_MODE_SS_ENABLED,
    SPI_SLAVE_MODE_SS_DISABLED
} spi_mode_select_t;
/**
 * struct spi_t - A Struct for SPI 
 * @spi_interrupt: The interrupt handler for SPI Module
 * @spi_interrupt_priority: The priority of the interrupt of SPI Module
 * @spi_mode: The Operating Mode of the SPI Module @ref spi_mode_select_t
 * @idle_clk_polarity: The Clock Polarity of the idle state of the SPI Module
 * @clk_transition_edge: The Clock transition edge of the SPI Module
 */
typedef struct
{
#if SPI_INTERRUPT_FEATURE == INTERRUPT_ENABLE
    INTERRUPT_HANDLER spi_interrupt;
#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
    interrupt_priority_cfg spi_interrupt_priority;
#endif
#endif
    spi_mode_select_t spi_mode;
    uint8 idle_clk_polarity : 1;
    uint8 data_input_sampe_phase : 1;
    uint8 clk_transition_edge : 1;
    uint8 RESERVED : 5;
} spi_t;
/*----------------------------Function Prototypes-----------------------------*/
/**
 * @brief: Initialize the SPI module
 * @param spi_obj the SPI module object
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType spi_init(const spi_t *const spi_obj);
/**
 * @brief: Deinitialize the SPI module
 * @param spi_obj the SPI module object
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType spi_deinit(const spi_t *const spi_obj);
/**
 * @brief: Send Data using Master Mode SPI Module
 * @param spi_obj the SPI module object
 * @param slave_ss_pin the slave select pin to send data to its Slave SPI Module
 * @param data the data to send
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType spi_master_send_data(const spi_t *const spi_obj, 
                                    const pin_config_t *const slave_ss_pin,
                                     const uint8 data);
/**
 * @brief: Receive Data using Master Mode SPI Module
 * @param spi_obj the SPI module object
 * @param slave_ss_pin the slave select pin to send data to its Slave SPI Module
 * @param data the address to save the data read
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType spi_master_receive_data(const spi_t *const spi_obj, 
                                    const pin_config_t *const slave_ss_pin,
                                     uint8 *const data);
/**
 * @brief: Send Data using Slave Mode SPI Module
 * @param spi_obj the SPI module object
 * @param data the data to send
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType spi_slave_send_data(const spi_t *const spi_obj, const uint8 data);
/**
 * @brief: Receive Data using Slave Mode SPI Module
 * @param spi_obj the SPI module object
 * @param data the address to save the data read
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType spi_slave_receive_data(const spi_t *const spi_obj, uint8 *const data);
#endif	/* MCAL_SPI_H */

