#include "mcal_i2c.h"
/*---------------Static Data types----------------------------------------------*/
#if I2C_MODULE_ENABLE == MCAL_ENABLED
#if I2C_INTERRUPT_FEATURE == INTERRUPT_FEATURE_ENABLE
static INTERRUPT_HANDLER i2c_interrupt_handler = NULL;              /* A pointer to the callback function when an interrupt is raised */
static uint8 data_to_transmit[_I2C_TRANSMIT_MAX_LEN] = {ZERO_INIT};
static uint8 tx_idx = ZERO_INIT;
static uint8 tx_queue_len = ZERO_INIT;
static uint8 *slave_ack_nack = NULL;
static uint8 *data_to_receive = NULL;
static uint8 *expected_data_to_receive = NULL;
static uint8 i2c_current_mode = _I2C_START_COND_INTERRUPT;
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
#if I2C_INTERRUPT_FEATURE == INTERRUPT_DISABLE
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
#endif
/*----Interrupt helper functions----*/
#if I2C_INTERRUPT_FEATURE == INTERRUPT_ENABLE
static inline Std_ReturnType configure_i2c_interrupt(const i2c_t *const i2c_obj);
static inline Std_ReturnType set_i2c_interrupt_handler(INTERRUPT_HANDLER i2c_interrupt);
static inline void i2c_master_send_address(void);
static inline void i2c_send_data(void);
static inline void i2c_read_SSPBUF(void);
#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
static inline void configure_i2c_interrupt_priority(interrupt_priority_cfg i2c_interrupt_priority);
#endif
#endif 
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
        /* Configure interrupt and its priority */
#if I2C_INTERRUPT_FEATURE == INTERRUPT_ENABLE
        configure_i2c_interrupt(i2c_obj);
#endif 
        /* Enable or disable SMBUS specific inputs */
        configure_i2c_smbus_specific_inputs(i2c_obj);
        /* Enable or disable Slew rate control */
        configure_i2c_slew_rate(i2c_obj);
        /* Enable Serial Port */
        I2C_SERIAL_PORT_ENABLE_CONFIG();
    }
    return (return_status);
}
#if I2C_INTERRUPT_FEATURE == INTERRUPT_DISABLE
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
                if (E_NOT_OK == i2c_receive_data(data, expected_data))
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
#else
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
    
    if (NULL == i2c_obj)
    {
        return_status = E_NOT_OK;
    }
    else
    {
        if (tx_queue_len < _I2C_TRANSMIT_MAX_LEN)
        {
            /* Set the data to transmit */
            data_to_transmit[tx_queue_len] = data;
            tx_queue_len++;
        }
        else if (tx_queue_len == _I2C_TRANSMIT_MAX_LEN)
        {
            tx_queue_len = 0;
        }
        /* Set the address to send the data to */
        slave_low_byte_addr = (uint8)(slave_addr << 1);
        CLEAR_BIT(slave_low_byte_addr, 0);
        /* Set the slave_ack_nack pointer to the current address given */
        slave_ack_nack = slave_ack;
        /* Set the i2c_current_mode to be used in the interrupts */
        i2c_current_mode = _I2C_TRANSMIT_INTERRUPT;
        /* Send start condition */
        I2C_MASTER_SEND_START_CONFIG();
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
                                        uint8 *expected_data)
{
    Std_ReturnType return_status = E_OK;
    
    if (NULL == i2c_obj)
    {
        return_status = E_NOT_OK;
    }
    else
    {
        /* Set the address to send the data to */
        slave_low_byte_addr = (uint8)(slave_addr << 1);
        SET_BIT(slave_low_byte_addr, 0);
        /* Set the pointer data_to_recieve to the address of @data to change it directly */
        data_to_receive = data;
        /* Set the pointer expected_data_to_receive to the address of @expected_data to change it directly */
        expected_data_to_receive = expected_data;
        /* Send start condition */
        if (_I2C_START_COND_INTERRUPT == i2c_current_mode)
        {
            /* Set the i2c_current_mode to be used in the interrupts */
            i2c_current_mode = _I2C_RECEIVE_INTERRUPT;
            I2C_MASTER_SEND_START_CONFIG();
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
        /* Set the data to transmit */
        if (tx_queue_len < _I2C_TRANSMIT_MAX_LEN)
        {
            data_to_transmit[_I2C_TRANSMIT_MAX_LEN - tx_queue_len] = data;
            tx_queue_len++;
        }
        else if (tx_queue_len == _I2C_TRANSMIT_MAX_LEN)
        {
            tx_queue_len = 0;
        }
        
        if (I2C_SLAVE_MODE_7_BIT_ADDR_START_STOP_INTERRUPTS_ON == i2c_obj->i2c_mode 
                && _I2C_START_COND_INTERRUPT == i2c_current_mode)
        {
            /* Set the i2c_current_mode to be used in the interrupts */
            i2c_current_mode = _I2C_TRANSMIT_INTERRUPT;
        }
        else if ((I2C_SLAVE_MODE_7_BIT_ADDR_START_STOP_INTERRUPTS_OFF == i2c_obj->i2c_mode)
                &&(_I2C_STOP_COND_INTERRUPT == i2c_current_mode || _I2C_START_COND_INTERRUPT == i2c_current_mode))
        {
            /* Set the i2c_current_mode to be used in the interrupts */
            i2c_current_mode = _I2C_ADDRESS_SENT_INTERRUPT;
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
Std_ReturnType i2c_slave_receive_data_7_bit_addr(const i2c_t *const i2c_obj, 
                                                uint8 *const data)
{
    Std_ReturnType return_status = E_OK;
    
    if (NULL == i2c_obj)
    {
        return_status = E_NOT_OK;
    }
    else
    {
        /* Set the pointer data_to_recieve to the address of @data to change it directly */
        data_to_receive = data;
        /* Send start condition */
        if (_I2C_START_COND_INTERRUPT == i2c_current_mode)
        {
            /* Set the i2c_current_mode to be used in the interrupts */
            i2c_current_mode = _I2C_RECEIVE_INTERRUPT;
        }
    }
    return (return_status);
}
#endif
/*---------------Static Helper functions definitions----------------------------*/
#if I2C_INTERRUPT_FEATURE == INTERRUPT_DISABLE
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
#else
/**
 * @brief: Send address when start condition happens in I2C Master Transmit mode
 */
static inline void i2c_master_send_address(void)
{
    SSPBUF = slave_low_byte_addr;
}
/**
 * @brief: Send data when start condition happens in I2C Transmit mode
 */
static inline void i2c_send_data(void)
{
    SSPBUF = data_to_transmit[tx_idx];
    tx_idx++;
    tx_queue_len--;
    if (tx_idx == _I2C_TRANSMIT_MAX_LEN)
    {
        tx_idx = 0;
    }
}
/*
 * @brief: Read the SSPBUF in I2C Master Receive mode
 */
static inline void i2c_read_SSPBUF(void)
{
    if (NULL != data_to_receive)
    {
        *data_to_receive = SSPBUF;
    }
}
/*
 * @brief: Send ACK in I2C Master Receive mode
 */
static inline void i2c_send_ack(void)
{
    /* Check if data pointers are valid */
    if (expected_data_to_receive && data_to_receive)
    {
        /* Compare expected data with received data */
        if (*expected_data_to_receive == *data_to_receive)
        {
            /* Data as expected, send NACK to end the communication */
            I2C_MASTER_RECEIVE_SET_NACK_CONFIG();
        }
        else
        {
            /* Data not as expected, send ACK to request another byte */
            I2C_MASTER_RECEIVE_SET_ACK_CONFIG();
        }
    }
    else
    {
        /* Default behavior: NACK to end communication */
        I2C_MASTER_RECEIVE_SET_NACK_CONFIG();
    }

    /* Send the ACK/NACK bit */
    I2C_MASTER_RECEIVE_SEND_ACK_NACK_CONFIG();
}

/**
 * @brief the interrupt service routine of I2C Module Master Mode
 * @param interrupt_type the interrupt type that has happened
 */
void I2C_MASTER_ISR(const uint8 interrupt_type)
{
    /* Clear the I2C interrupt flag */
    I2C_INTERRUPT_FLAG_BIT_CLEAR();

    switch (interrupt_type)
    {
        /* The interrupt source is the start cond */
        case _I2C_START_COND_INTERRUPT:
            /* Send slave address */
            i2c_master_send_address();
            /* Change the next source to the address sent interrupt */
            I2C_INTERRUPT_TYPE = _I2C_ADDRESS_SENT_INTERRUPT;
            break;
            
        /* The interrupt source is the address sent */    
        case _I2C_ADDRESS_SENT_INTERRUPT:
            /*If I2C master transmit mode is on, transmit the data */
            if (i2c_current_mode == _I2C_TRANSMIT_INTERRUPT)
            {
                /* Send the data */
                i2c_send_data();
            }
            else if (i2c_current_mode == _I2C_RECEIVE_INTERRUPT)
            {
                /* Enable Receive Mode */
                I2C_MASTER_ENABLE_RECEIVE_MODE_CONFIG();
            }
            /* Change the next source to the ending operation interrupt */
            I2C_INTERRUPT_TYPE = _I2C_OPERATION_INTERRUPT;
            break;

        /* The interrupt source is the ending operation interrupt */    
        case _I2C_OPERATION_INTERRUPT:
            if (i2c_current_mode == _I2C_RECEIVE_INTERRUPT)
            {
                /* Wait until SSPBUF is full */
                while (_I2C_RECEIVE_BUFFER_EMPTY == SSPSTATbits.BF);
                /* Read the data */
                i2c_read_SSPBUF();
                /* If I2C master receive mode is on, ack data */
                i2c_send_ack();
                /* Wait until it is sent */
                while (_I2C_MASTER_ACK_SEQUENCE_ENABLE == SSPCON2bits.ACKEN);
            }
            /* Send Stop Condition */
            I2C_MASTER_SEND_STOP_COND_CONFIG();

            /* Change the next source to the Stop condition interrupt */
            I2C_INTERRUPT_TYPE = _I2C_STOP_COND_INTERRUPT;
            break;
            
        case _I2C_STOP_COND_INTERRUPT:
            /* Change the next source to be the start condition to indicate a new comms */
            i2c_current_mode = _I2C_START_COND_INTERRUPT;
            I2C_INTERRUPT_TYPE = _I2C_START_COND_INTERRUPT;
            break;
    }

    /* Call the interrupt handler if set */
    if (i2c_interrupt_handler)
    {
        i2c_interrupt_handler();
    }
}

/**
 * @brief The interrupt service routine for I2C Module in Slave Mode
 * @param interrupt_type The interrupt type that has occurred
 */
void I2C_SLAVE_ISR(const uint8 interrupt_type)
{
    /* Clear the I2C interrupt flag */
    I2C_INTERRUPT_FLAG_BIT_CLEAR();
    
    switch (interrupt_type)
    {
        /* The interrupt source is a start condition */
        case _I2C_START_COND_INTERRUPT:
            /* Reset mode and prepare for address reception */
            i2c_current_mode = _I2C_ADDRESS_SENT_INTERRUPT;
            I2C_INTERRUPT_TYPE = _I2C_ADDRESS_SENT_INTERRUPT;
            break;
            
        /* The interrupt source is an address sent interrupt */    
        case _I2C_ADDRESS_SENT_INTERRUPT:
            /* Read the SSPBUF to clear BF flag and prevent overflow */
            if (_I2C_SLAVE_READ_MODE == SSPSTATbits.RW || 
                _I2C_SLAVE_WRITE_MODE == SSPSTATbits.RW)
            {
                volatile uint8 dummy_data = SSPBUF;
                /* Release the CLK */
                I2C_SLAVE_RELEASE_CLK_CONFIG();
            }
            
            /* Determine mode based on R/W bit */
            if (_I2C_SLAVE_READ_MODE == SSPSTATbits.RW)  
            {
                /* Master requests data (Slave Transmit Mode) */
                i2c_current_mode = _I2C_TRANSMIT_INTERRUPT;
                i2c_send_data();  
            }
            /* Move to operation phase */
            I2C_INTERRUPT_TYPE = _I2C_OPERATION_INTERRUPT;
            break;
            
        case _I2C_OPERATION_INTERRUPT:
            /* Master is sending data (Slave Receive Mode) */
            /* Check for data reception */
            if (_I2C_RECEIVE_BUFFER_FULL == SSPSTATbits.BF)
            {
                /* Read received data */
                i2c_read_SSPBUF();
            }
            /* Reset communication mode for new transmissions */
            I2C_INTERRUPT_TYPE = _I2C_STOP_COND_INTERRUPT;
            break;
            
        /* The interrupt source is a stop condition */
        case _I2C_STOP_COND_INTERRUPT:
            /* Reset communication mode for new transmissions */
            i2c_current_mode = _I2C_START_COND_INTERRUPT;
            I2C_INTERRUPT_TYPE = _I2C_START_COND_INTERRUPT;
            break;
    }

    /* Call the registered interrupt handler (if available) */
    if (i2c_interrupt_handler)
    {
        i2c_interrupt_handler();
    }
    /* Release the CLK */
    I2C_SLAVE_RELEASE_CLK_CONFIG();
}

#endif
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
    SSPADD = (uint8)(slave_low_byte_addr << 1);
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
#if I2C_INTERRUPT_FEATURE == INTERRUPT_ENABLE
/**
 * @brief: Configure the I2C interrupts
 * @param i2c_obj the I2C module object
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType configure_i2c_interrupt(const i2c_t *const i2c_obj)
{
    Std_ReturnType ret = E_OK;
    
    /* Disable Interrupt before configuring */
    I2C_INTERRUPT_DISABLE();
    
    /* Enable priority levels */
#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
    INTERRUPT_PRIORITY_levels_ENABLE();
    INTERRUPT_Global_interrupt_LOW_ENABLE();
    INTERRUPT_Global_interrupt_HIGH_ENABLE();
    /* Configure the I2C Module Interrupt priority */
    configure_i2c_interrupt_priority(i2c_obj->i2c_interrupt_priority);
#else
    /* If the interrupt priority is disabled then enable the peripheral interrupt
    and global interrupts */
        INTERRUPT_Peripheral_interrupt_ENABLE();
        INTERRUPT_Global_interrupt_ENABLE();
#endif
    /* Clear Interrupt flag */
    I2C_INTERRUPT_FLAG_BIT_CLEAR();
    
    /* Configure the interrupt handler */
    ret = set_i2c_interrupt_handler(i2c_obj->i2c_interrupt);
    
    /* Enable I2C interrupt */
    I2C_INTERRUPT_ENABLE();
    
    return (ret);
}
/**
 * @brief: Set the interrupt handler of the i2c module
 * @param i2c_interrupt the interrupt handler to call when the interrupt flag is set
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType set_i2c_interrupt_handler(INTERRUPT_HANDLER i2c_interrupt)
{
    Std_ReturnType ret = E_OK;
    
    /* Set the interrupt handler if it isn't null */
    if (i2c_interrupt)
    {
        i2c_interrupt_handler = i2c_interrupt;
    }
    else
    {
        ret = E_NOT_OK;
    }
    return (ret); 
}

#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
/**
 * @brief Configure the I2C Interrupt priority depending on the user input
 * @param i2c_interrupt_priority the inputted interrupt priority from the user
 */
static inline void configure_i2c_interrupt_priority(interrupt_priority_cfg i2c_interrupt_priority)
{
    switch (i2c_interrupt_priority)
    {
        /* High Priority */
        case INTERRUPT_HIGH_PRIORITY:
            I2C_HIGH_PRIORITY();
            break;
        /* Low Priority */
        case INTERRUPT_LOW_PRIORITY:
            I2C_LOW_PRIORITY();
            break;
    }
}
#endif
#endif
#endif
/*---------------Static Helper functions definitions End------------------------*/
