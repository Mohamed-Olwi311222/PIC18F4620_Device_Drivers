#include "mcal_i2c.h"
/*---------------Static Data types----------------------------------------------*/
#if I2C_INTERRUPT_FEATURE == INTERRUPT_FEATURE_ENABLE
static INTERRUPT_HANDLER spi_interrupt_handler = NULL; /* A pointer to the callback function when an interrupt is raised */
#endif
/* SCL (Serial clock) pin */
static pin_config_t SCL_pin = {.port = PORTC_INDEX, .pin = GPIO_PIN3};
/* SDA (Serial Data) pin */
static pin_config_t SDA_pin = {.port = PORTC_INDEX, .pin = GPIO_PIN4};
/* The first 7 bits of the Slave address*/
static uint8 slave_low_byte_addr = ZERO_INIT;
/* The 2 MSBs of the 10-bit Slave Address */
static uint8 slave_high_byte_addr = ZERO_INIT;
/*---------------Static Data types End------------------------------------------*/

/*---------------Static Helper functions declerations---------------------------*/
static inline Std_ReturnType configure_i2c_pins(void);
static inline void configure_i2c_slew_rate(const i2c_t *const i2c_obj);
static inline void configure_i2c_smbus_specific_inputs(const i2c_t *const i2c_obj);
static inline Std_ReturnType configure_i2c_mode(const i2c_t *const i2c_obj);
/*----SLAVE Helper functions----*/
static inline Std_ReturnType configure_i2c_slave_mode(const i2c_t *const i2c_obj);
static inline Std_ReturnType select_i2c_slave_mode(const i2c_t *const i2c_obj);
/*----MASTER Helper functions---*/
static inline Std_ReturnType configure_i2c_master_mode(const i2c_t *const i2c_obj);
static inline Std_ReturnType select_i2c_master_mode(const i2c_t *const i2c_obj);
static inline Std_ReturnType i2c_master_set_speed(const i2c_t *const i2c_obj);
static inline Std_ReturnType i2c_master_send_start_cond(void);
static inline Std_ReturnType i2c_master_send_stop_cond(void);
static inline Std_ReturnType i2c_send_data(const uint8 data, 
                                           uint8 *const slave_ack);
static inline Std_ReturnType i2c_send_address(const uint8 slave_addr, 
                                              const uint8 mode,
                                              uint8 *const slave_ack);
static inline Std_ReturnType i2c_receive_data(uint8 *const data, 
                                           uint8 *const expected_data);
static inline void poll_i2c_interrupt_flag(void);
static inline void clear_i2c_interrupt_flag(void);
static inline uint8 i2c_interrupt_flag(void);
static inline void poll_i2c_sen_bit(void);
static inline void poll_i2c_pen_bit(void);
/*----Interrupt helper functions----*/

/*---------------Static Helper functions declerations End-----------------------*/
/**
 * @brief: Initialize the I2C module
 * @param i2c_obj the I2C module object
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType i2c_init(const i2c_t *const i2c_obj)
{
    Std_ReturnType return_status = E_OK;
    
    if (NULL == i2c_obj)
    {
        return_status = E_NOT_OK;
    }
    else
    {
        /* Disable Serial Port */
        I2C_SERIAL_PORT_DISABLE_CONFIG();
        /* Configure the I2C Pins */
        if (E_NOT_OK == configure_i2c_pins())
        {
            /* Error in configuring the i2c pins */
            return_status = E_NOT_OK; 
        }
        /* Enable and Select I2C Module depending on the mode selected */
        if (E_NOT_OK == configure_i2c_mode(i2c_obj))
        {
            /* Error in configuring the i2c mode */
            return_status = E_NOT_OK;
        }
        /* Enable or disable SMBUS specific inputs */
        configure_i2c_smbus_specific_inputs(i2c_obj);
        /* Enable or disable Slew rate control */
        configure_i2c_slew_rate(i2c_obj);
        /* Enable Serial Port */
        I2C_SERIAL_PORT_ENABLE_CONFIG();
    }
    return (return_status);
}
/**
 * @brief: Send data using master transmitter to a 7-bit slave address
 * @param i2c_obj the I2C module object
 * @param slave_addr the address of the slave to send to it
 * @param data the data to send to the slaved
 * @param slave_ack the address to store the slave acknowledge
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType i2c_master_transmit_data_7_bit_addr(const i2c_t *const i2c_obj, 
                                        const uint8 slave_addr, 
                                        const uint8 data,
                                        uint8 *const slave_ack)
{
    Std_ReturnType return_status = E_OK;
    
    if (NULL == i2c_obj ||0x0000 == slave_addr)
    {
        return_status = E_NOT_OK;
    }
    else
    {
        /* Send Start Condition */
        if (i2c_master_send_start_cond() == E_OK)
        {
            /* Send slave address */
            return_status = i2c_send_address(slave_addr, _I2C_SLAVE_WRITE_MODE, slave_ack);
            if (_I2C_SLAVE_ACK_NOT_RECEIVED == *slave_ack)
            {
                /* Ack not received from the slave */
                return_status = E_NOT_OK;
            }
            else
            {
                /* Send data if a slave acknowledge its address */
                return_status = i2c_send_data(data, slave_ack);
                if (_I2C_SLAVE_ACK_NOT_RECEIVED == *slave_ack)
                {
                    /* Ack not received from the slave */
                    return_status = E_NOT_OK;
                }
            }
            /* Send Stop Condition */
            if (i2c_master_send_stop_cond() == E_NOT_OK)
            {
                /* failed to send stop condition */
                return_status = E_NOT_OK;
            }
        }
        else
        {
            /* failed to send start condition */
            return_status = E_NOT_OK;
        }
    }
    return (return_status);
}
/**
 * @brief: Receive data using master receiver of a 7-bit slave address
 * @param i2c_obj the I2C module object
 * @param slave_addr the address of the slave to receive from it
 * @param data the address to store the data received
 * @param expected_data the expected data to be received (NULL if none)
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType i2c_master_receive_data_7_bit_addr(const i2c_t *const i2c_obj, 
                                        const uint8 slave_addr, 
                                        uint8 *const data,
                                        uint8 *const expected_data)
{
    Std_ReturnType return_status = E_OK;
    
    if (NULL == i2c_obj ||0x0000 == slave_addr)
    {
        return_status = E_NOT_OK;
    }
    else
    {
        /* Send Start Condition */
        if (i2c_master_send_start_cond() == E_OK)
        {
            return_status = i2c_send_address(slave_addr, _I2C_SLAVE_READ_MODE, NULL);
            /* Send slave address */
            if (_I2C_SLAVE_ACK_NOT_RECEIVED == return_status)
            {
                /* Ack not received from the slave */
                return_status = E_NOT_OK;
            }
            else
            {
                /* Receive data if a slave acknowledge its address */
                if (_I2C_MASTER_NACK_RECEIVED_DATA == i2c_receive_data(data, expected_data))
                {
                    /* Data isn't as expected */
                    return_status = E_NOT_OK;
                }
            }
            /* Send Stop Condition */
            if (i2c_master_send_stop_cond() == E_NOT_OK)
            {
                /* failed to send stop condition */
                return_status = E_NOT_OK;
            }
        }
        else
        {
            /* failed to send start condition */
            return_status = E_NOT_OK;
        }
    }
    return (return_status); 
}
/**
 * @brief: Send data using slave transmitter to a master
 * @param i2c_obj the I2C module object
 * @param data the data to send to the master
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType i2c_slave_transmit_data_7_bit_addr(const i2c_t *const i2c_obj, 
                                        const uint8 data)
{
    Std_ReturnType return_status = E_OK;
    
    if (NULL == i2c_obj)
    {
        return_status = E_NOT_OK;
    }
    else
    {  
        /* Wait for the address match */
        poll_i2c_interrupt_flag();
        
        if (_I2C_SLAVE_READ_MODE == SSPSTATbits.R_W)
        {  
            /* Address is in read mode */
            /* Send the data given */
            SSPBUF = data;     
            /* Wait until master reads the data */
            poll_i2c_interrupt_flag();
        } 
    }
       return (return_status);
}
/**
 * @brief: Receive data using slave receiver from a master
 * @param i2c_obj the I2C module object
 * @param data the address to store the data received
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType i2c_slave_receive_data_7_bit_addr(const i2c_t *const i2c_obj, uint8 *const data)
{
    Std_ReturnType return_status = E_OK;
    
    if (NULL == i2c_obj || NULL == data)
    {
        return_status = E_NOT_OK;
    }
    else
    {
        poll_i2c_interrupt_flag();
        I2C_SLAVE_HOLD_CLK_CONFIG();
        if (_I2C_SLAVE_WRITE_MODE == SSPSTATbits.RW)
        {            
            /* Read the data */
            *data = SSPBUF;
        }
        I2C_SLAVE_RELEASE_CLK_CONFIG();
    }
    return (return_status);
}
/*---------------Static Helper functions definitions----------------------------*/
/**
 * @brief Read the status of the I2C interrupt flag
 * @return the status of the I2C interrupt flag
 */
static inline uint8 i2c_interrupt_flag(void)
{
    return PIR1bits.SSPIF;
}
/**
 * @brief Clear the I2C interrupt flag
 */
static inline void clear_i2c_interrupt_flag(void)
{
    PIR1bits.SSPIF = 0;
}
/**
 * @brief poll the I2C interrupt flag to wait for the current operation to end
 */
static inline void poll_i2c_interrupt_flag(void)
{
    /* Wait for the transmission to complete */
    while (!i2c_interrupt_flag());
    /* Clear the interrupt flag for the next operation */
    clear_i2c_interrupt_flag(); 
}
/**
 * @brief poll the SEN bit to wait until the start condition is sent
 */
static inline void poll_i2c_sen_bit(void)
{
    /* Wait until the start condition is sent */
    while (!(_I2C_MASTER_START_COND_IDLE == SSPCON2bits.SEN));
    /* Clear Interrupt flag */
    clear_i2c_interrupt_flag();
}
/**
 * @brief poll the PEN bit to wait until the stop condition is sent
 */
static inline void poll_i2c_pen_bit(void)
{
    /* Wait Until the stop condition is sent */
    while (!(_I2C_MASTER_STOP_COND_IDLE == SSPCON2bits.PEN)); 
    /* Clear Interrupt flag */
    clear_i2c_interrupt_flag();
}
/**
 * @brief: Send the first byte data + R/W bit
 * @param slave_addr the address of the slave to send
 * @param mode the mode needed read or write
 * @param slave_ack the address to store the slave acknowledge
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType i2c_send_address(const uint8 slave_addr, 
                                              const uint8 mode,
                                              uint8 *const slave_ack)
{
    Std_ReturnType return_status = E_OK;
    uint8 first_byte = slave_addr;               /* First byte to transmit (addr + RW) */
    
    /* Shift the slave address for the R/W bit to be LSB */
    first_byte = (uint8)((uint16)(first_byte << 1));
    if (_I2C_SLAVE_WRITE_MODE == mode)
    {
        /* Clear the 0 bit in the address to indicate a write */
        CLEAR_BIT(first_byte, 0);
    }
    else
    {
        /* Set the 0 bit in the address to indicate a read */
        SET_BIT(first_byte, 0);
    }
    /* Send the slave address and the R/W bit */
    SSPBUF = first_byte;
    /* Wait until transmit stops */
    poll_i2c_interrupt_flag();
    
    /* Check the slave ACK */
    while (_I2C_SLAVE_ACK_NOT_RECEIVED == SSPCON2bits.ACKSTAT);
    if (NULL != slave_ack )
    {
        I2C_MASTER_TRANSMIT_READ_ACK_STATUS_CONFIG(slave_ack); 
    }
    return (return_status);
}
/**
 * @brief: Send data using I2C Module
 * @param data the data to send
 * @param slave_ack the address to store the slave acknowledge
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType i2c_send_data(const uint8 data, 
                                           uint8 *const slave_ack)
{
    Std_ReturnType return_status = E_OK;

    /* Send data to the slave */
    SSPBUF = data;
    /* Wait until transmit stops */
    poll_i2c_interrupt_flag();
    
    while (_I2C_SLAVE_ACK_NOT_RECEIVED == SSPCON2bits.ACKSTAT);
    /* Check the slave ACK */
    if (NULL != slave_ack)
    {
        I2C_MASTER_TRANSMIT_READ_ACK_STATUS_CONFIG(slave_ack); 
    }
    return (return_status);
}
/**
 * @brief: Receive data using I2C Module
 * @param data the address to store the received data
 * @param expected_data the expected data to be received (NULL if none)
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType i2c_receive_data(uint8 *const data, 
                                           uint8 *const expected_data)
{
    Std_ReturnType return_status = E_OK;
    
    /* Enable receive mode (will be cleared by hardware)*/
    I2C_MASTER_ENABLE_RECEIVE_MODE_CONFIG();
    /* Wait for reception */
    poll_i2c_interrupt_flag();
    /* Read the data received from the slave */
    *data = SSPBUF;
    if (NULL != expected_data && expected_data == data)
    {
        /* The expected data is received */
        I2C_MASTER_RECEIVE_SET_ACK_CONFIG();
    }
    else if (NULL != expected_data && expected_data != data)
    {
        /* The expected data is not received */
        I2C_MASTER_RECEIVE_SET_NACK_CONFIG();
        return_status = E_NOT_OK;
    }
    /* Send the ACK/NACK bit */
    I2C_MASTER_RECEIVE_SEND_ACK_NACK_CONFIG();
    return (return_status);
}
/**
 * @brief: Send the start condition in master mode
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType i2c_master_send_start_cond(void)
{
    Std_ReturnType return_status = E_OK;
    
    /* Send start condition */
    I2C_MASTER_SEND_START_CONFIG();
    /* Wait until the start condition is sent */
    poll_i2c_sen_bit();
    
    /* Check if the start condition is sent successfully */
    if (_I2C_START_BIT_DETECTED == SSPSTATbits.S)
    {
        /* Start Condition is sent */
        return_status = E_OK;
    }
    else
    {
        /* Start Condition is not sent */
        return_status = E_NOT_OK;
    }
    return (return_status);
}
/**
 * @brief: Send the stop condition in master mode
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType i2c_master_send_stop_cond(void)
{
    Std_ReturnType return_status = E_OK;
    
    /* Send Stop Condition */
    I2C_MASTER_SEND_STOP_COND_CONFIG();
    poll_i2c_pen_bit();
    
    /* Check if the Stop condition is sent successfully */
    if (_I2C_STOP_BIT_DETECTED == SSPSTATbits.P)
    {
        /* Stop Condition is sent */
        return_status = E_OK;
    }
    else
    {
        /* Stop Condition is not sent */
        return_status = E_NOT_OK;
    }
    return (return_status);
}
/**
 * @brief: Configure the SDA pin and SCL pin to be both inputs
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType configure_i2c_pins(void)
{
    Std_ReturnType return_status = E_OK;

    /* Configure the SCL Pin to be input */
    SCL_pin.direction = GPIO_DIRECTION_INPUT;
    if (gpio_pin_direction_initialize(&SCL_pin) == E_NOT_OK)
    {
        /* Error in configuring the SCL pin */
        return_status = E_NOT_OK;
    }
    
    /* Configure the SDA Pin to be input */
    SDA_pin.direction = GPIO_DIRECTION_INPUT;
    if (gpio_pin_direction_initialize(&SDA_pin) == E_NOT_OK)
    {
        /* Error in configuring the SDA pin */
        return_status = E_NOT_OK;
    }
    return (return_status);
}
/**
 * @brief: Configure the Slew rate of the I2C Module
 * @param i2c_obj the I2C module object
 */
static inline void configure_i2c_slew_rate(const i2c_t *const i2c_obj)
{
    switch (i2c_obj->i2c_slew_rate_control)
    {
        /* Slew rate control enabled for High-Speed mode (400 kHz) */
        case _I2C_SLEW_RATE_CONTROL_ENABLE:
            I2C_SLEW_RATE_CONTROL_ENABLE_CONFIG();
            break;
            
        /* Slew rate control disabled for Standard Speed mode (100 kHz) */
        case _I2C_SLEW_RATE_CONTROL_DISABLE:
            I2C_SLEW_RATE_CONTROL_DISABLE_CONFIG();
            break;
    }  
}
/**
 * @brief: Configure the SMBUS Specific inputs of the I2C Module
 * @param i2c_obj the I2C module object
 */
static inline void configure_i2c_smbus_specific_inputs(const i2c_t *const i2c_obj)
{
    switch (i2c_obj->i2c_smbus_enable)
    {
        /* SMBUS Specific inputs is enabled */
        case _I2C_SMBUS_ENABLE:
            I2C_SMBUS_ENABLE_CONFIG();
            break;
            
        /* SMBUS Specific inputs is disable */
        case _I2C_SMBUS_DISABLE:
            I2C_SMBUS_DISABLE_CONFIG();
            break;
    }
}
/**
 * @brief: Configure the I2C depending on the Mode bits
 * @param i2c_obj the I2C module object
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType configure_i2c_mode(const i2c_t *const i2c_obj)
{
    Std_ReturnType return_status = E_NOT_OK;
    
    if (I2C_FIRMWARE_CONTROLLER_MASTER_MODE == i2c_obj->i2c_mode || 
        I2C_MASTER_MODE ==i2c_obj->i2c_mode )
    {
        /* Configure Master Mode if selected */
        return_status = configure_i2c_master_mode(i2c_obj);
    }
    else
    {
        /* Configure Slave Mode if selected */
        return_status = configure_i2c_slave_mode(i2c_obj);  
    }
    return (return_status);
}
/*---------------SLAVE Helper functions-----------------------------------------*/
/**
 * @brief: Configure the I2C Slave Mode
 * @param i2c_obj the I2C module object
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType configure_i2c_slave_mode(const i2c_t *const i2c_obj)
{
    Std_ReturnType return_status = E_OK;
    
    /* Enable/Disable General Call interrupt */
    if (_I2C_SLAVE_GENERAL_CALL_ENABLE == i2c_obj->i2c_slave_general_call_enable)
    {
        I2C_SLAVE_ENABLE_GENERAL_CALL_INTERRUPT_CONFIG();
    }
    else
    {
        I2C_SLAVE_DISABLE_GENERAL_CALL_INTERRUPT_CONFIG(); 
    }

    /* Set the slave mode selected by the user */
    if (select_i2c_slave_mode(i2c_obj) == E_NOT_OK)
    {
        /* Error in setting the mode */
        return_status = E_NOT_OK; 
    }
    
    /* Enable Clock Stretching */
    I2C_SLAVE_ENABLE_CLK_STRETCH_CONFIG();
    
    return (return_status);
}
/**
 * @brief: Select the Operating I2C Slave Mode
 * @param i2c_obj the I2C module object
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType select_i2c_slave_mode(const i2c_t *const i2c_obj)
{
    Std_ReturnType return_status = E_OK;
    i2c_mode_t slave_mode = ZERO_INIT;

    /* Select the I2C Slave Mode */
    switch(i2c_obj->i2c_mode)
    {
        /* 10-bit Addr Modes */    
        case I2C_SLAVE_MODE_10_BIT_ADDR_START_STOP_INTERRUPTS_ON:
            slave_mode = I2C_SLAVE_MODE_10_BIT_ADDR_START_STOP_INTERRUPTS_ON;
            /* Set the 2 MSBs of the 10-bit Slave address */
            slave_high_byte_addr = (uint8)(i2c_obj->i2c_slave_mode_addr) >> 8;
            break;
        case I2C_SLAVE_MODE_10_BIT_ADDR_START_STOP_INTERRUPTS_OFF:
            slave_mode = I2C_SLAVE_MODE_10_BIT_ADDR_START_STOP_INTERRUPTS_OFF;
            /* Set the 2 MSBs of the 10-bit Slave address */
            slave_high_byte_addr = (uint8)(i2c_obj->i2c_slave_mode_addr) >> 8;
            break;
            
        /* 7-bit Addr Modes */    
        case I2C_SLAVE_MODE_7_BIT_ADDR_START_STOP_INTERRUPTS_ON:
            slave_mode = I2C_SLAVE_MODE_7_BIT_ADDR_START_STOP_INTERRUPTS_ON;
            break;
        case I2C_SLAVE_MODE_7_BIT_ADDR_START_STOP_INTERRUPTS_OFF:
            slave_mode = I2C_SLAVE_MODE_7_BIT_ADDR_START_STOP_INTERRUPTS_OFF;
            break;
            
        default:
            return_status = E_NOT_OK;
            break;
    }
    /* Set the 7 bit address of the slave */
    slave_low_byte_addr = (uint8)(i2c_obj->i2c_slave_mode_addr);
    /* If no error happened, set the SSPM3:SSPM0 bits */
    if (E_OK == return_status)
    {
        I2C_SET_OPERATION_MODE(slave_mode);

    }
    /* Load the address of the slave to the SSPADD */
    SSPADD = slave_low_byte_addr;
    return (return_status);
}
/*---------------MASTER Helper functions----------------------------------------*/
/**
 * @brief: Configure the I2C Master Mode
 * @param i2c_obj the I2C module object
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType configure_i2c_master_mode(const i2c_t *const i2c_obj)
{
    Std_ReturnType return_status = E_OK;
    
    /* Set the Master mode selected by the user */
    if (select_i2c_master_mode(i2c_obj) == E_NOT_OK)
    {
        /* Error in setting the mode */
        return_status = E_NOT_OK; 
    }
    
    /* Set the Speed of the I2C */
    if (i2c_master_set_speed(i2c_obj) == E_NOT_OK)
    {
        /* Error in setting the Speed of the I2C */
        return_status = E_NOT_OK; 
    }
    return (return_status);
}
/**
 * @brief: Set the chosen i2c master mode
 * @param i2c_obj the I2C module object
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType select_i2c_master_mode(const i2c_t *const i2c_obj)
{
    Std_ReturnType return_status = E_OK;
    uint8 current_master_mode = ZERO_INIT;
    
    switch (i2c_obj->i2c_mode)
    {
        /* I2C Master mode, clock = FOSC/(4 * (SSPADD + 1)) */
        case I2C_MASTER_MODE:
            current_master_mode = I2C_MASTER_MODE;
            break;
            
        /* I2C Firmware Controlled Master mode (Slave Idle) */
        case I2C_FIRMWARE_CONTROLLER_MASTER_MODE:
            current_master_mode = I2C_MASTER_MODE;
            break;    
            
        default:
            return_status = E_NOT_OK;
            break;
    }
    /* If no error happened, set the SSPM3:SSPM0 bits */
    if (E_OK == return_status)
    {
        I2C_SET_OPERATION_MODE(current_master_mode);
    }
    return (return_status);

}
/**
 * @brief: Set the Speed of the I2C (Master Mode Only)
 * @param i2c_obj
 * @return 
 */
static inline Std_ReturnType i2c_master_set_speed(const i2c_t *const i2c_obj)
{
    Std_ReturnType return_status = E_OK;
    uint32 i2c_clk = ZERO_INIT;
    
    /* Set the i2c_clk with the user chosen i2c speed */
    switch (i2c_obj->i2c_master_speed)
    {
        case I2C_100KB_STANDARD_MODE:
            i2c_clk = (uint32)(I2C_100KB_STANDARD_MODE * 1000ul);
            break;
            
        case I2C_400KB_FAST_MODE:
            i2c_clk = (uint32)(I2C_400KB_FAST_MODE * 1000ul);
            break;
            
        case I2C_1MB_FAST_MODE_PLUS:
            i2c_clk = (uint32)(I2C_1MB_FAST_MODE_PLUS * 1000ul);
            break;
            
        default:
            /* Unsupported Speed */
            return_status = E_NOT_OK;
            break;
    }
    /* Set the Speed of the I2C Communication Protocol */
    SSPADD = (uint8)(((_XTAL_FREQ) / (4 * i2c_clk)) - 1);
    
    return (return_status);
}
/*---------------Interrupt Helper functions-------------------------------------*/


/*---------------Static Helper functions definitions End------------------------*/
