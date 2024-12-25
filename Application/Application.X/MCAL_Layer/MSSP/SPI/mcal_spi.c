/* 
 * File:   mcal_spi.c
 * Author: Mohamed olwi
 *
 * Created on 12 December 2024, 00:36
 */
#include "mcal_spi.h"

/*---------------Static Data types----------------------------------------------*/

/*---------------Static Data types End------------------------------------------*/

/*---------------Static Helper functions declerations---------------------------*/
static inline void spi_slave_mode_init(const spi_t *const spi_obj);
static inline void spi_master_mode_init(const spi_t *const spi_obj);
static inline uint8 is_spi_buffer_full(void);
static inline uint8 is_spi_write_collision(void);
static inline void clear_spi_interrupt_flag(void);
static inline void clear_spi_write_collision(void);
static inline uint8 spi_interrupt_flag(void);
static inline void clear_spi_receive_overflow(void);
static inline uint8 read_spi_buffer(void);
/*---------------Static Helper functions declerations End-----------------------*/

/**
 * @brief: Initialize the SPI module
 * @param spi_obj the SPI module object
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType spi_init(const spi_t *const spi_obj)
{
    Std_ReturnType ret = E_OK;
    /* SDO (Serial Data Out) pin */
    static const pin_config_t SDO_pin = {.direction = GPIO_DIRECTION_OUTPUT, .port = PORTC_INDEX, .pin = GPIO_PIN5};
    
    if (NULL == spi_obj)
    {
        ret = E_NOT_OK;
    }
    else
    {
        /* Disable SPI */
        SPI_SERIAL_PORT_DISABLE_CONFIG();
        /* Configure the SPI Mode */
        SPI_SET_OPERATION_MODE(spi_obj->spi_mode);
        /* Configure CLK Edge */
        if ( _SPI_TRANSITION_ACTIVE_IDLE == spi_obj->clk_edg)
        {
            SPI_TRANSITION_ACTIVE_IDLE_CONFIG();
        }
        else
        {
            SPI_TRANSITION_IDLE_ACTIVE_CONFIG();
        }
        /* Configure CLK Polarity */
        if (_SPI_IDLE_STATE_HIGH == spi_obj->clk_polarity)
        {
            SPI_IDLE_STATE_HIGH_CONFIG();
        }
        else
        {
            SPI_IDLE_STATE_LOW_CONFIG();
        }
        /* Configure Master/Slave mode SCK pin data direction and Sample time*/
        if (SPI_SLAVE_MODE_SS_ENABLED == spi_obj->spi_mode || 
                SPI_SLAVE_MODE_SS_DISABLED == spi_obj->spi_mode)
        {
            spi_slave_mode_init(spi_obj);
        }
        else
        {
            spi_master_mode_init(spi_obj);
        }
        /* Configure SDO pin */
        if (E_OK != gpio_pin_direction_initialize(&SDO_pin)) ret = E_NOT_OK;
        /* Enable SPI */
        SPI_SERIAL_PORT_ENABLE_CONFIG();
    }
    return (ret);
}
/**
 * @brief: Initialize the SPI Slave mode sample time and SCK pin 
 * @param spi_obj the SPI module object
 */
static inline void spi_slave_mode_init(const spi_t *const spi_obj)
{
    /* SCK (Serial Clock) pin */
    static const pin_config_t SCK_pin = {.port = PORTC_INDEX, .pin = GPIO_PIN3, .direction = GPIO_DIRECTION_INPUT};
    /* SS pin control enabled */
    static const pin_config_t SS_pin = {.port = PORTA_INDEX, .pin = GPIO_PIN5, .direction = GPIO_DIRECTION_INPUT};

    /* Configure Slave mode Sample time */
    /* SMP must be cleared when SPI is used in Slave mode(sample at middle) */
    SPI_SAMPLE_INPUT_MIDDLE_CONFIG();
    /* Configure SCK pin */
    /* SCK (Slave mode) must have TRISC<3> bit set (input) */
    gpio_pin_direction_initialize(&SCK_pin);
    /* Configure SS pin if it is enabled */
    if (SPI_SLAVE_MODE_SS_ENABLED == spi_obj->spi_mode)
    {
        gpio_pin_direction_initialize(&SS_pin);
    }
}
/**
 * @brief: Initialize the SPI Master mode sample time and SCK pin
 * @param spi_obj the SPI module object
 */
static inline void spi_master_mode_init(const spi_t *const spi_obj)
{
    /* SCK (Serial Clock) pin */
    static const pin_config_t SCK_pin = {.port = PORTC_INDEX, .pin = GPIO_PIN3, .direction = GPIO_DIRECTION_OUTPUT};
    /* Configure Master mode Sample time */
    if (_SPI_SAMPLE_INPUT_MIDDLE == spi_obj->data_input_sampe_phase)
    {
        /* Input data sampled at middle of data output time */
        SPI_SAMPLE_INPUT_MIDDLE_CONFIG();
    }
    else
    {
        /* Input data sampled at end of data output time */
        SPI_SAMPLE_INPUT_END_CONFIG();
    }
    /* Configure SCK pin */
    /* SCK (Master mode) must have TRISC<3> bit cleared */
    gpio_pin_direction_initialize(&SCK_pin);
}
/**
 * @brief: Deinitialize the SPI module
 * @param spi_obj the SPI module object
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType spi_deinit(const spi_t *const spi_obj)
{
    Std_ReturnType ret = E_OK;
    
    if (NULL == spi_obj)
    {
        ret = E_NOT_OK;
    }
    else
    {
        /* Disable SPI Module */
        SPI_SERIAL_PORT_DISABLE_CONFIG();
    } 
    return (ret);
}
/**
 * @brief Read the status of the SPI buffer
 * @return the status of the BF flag in SSPSTAT register
 */
static inline uint8 is_spi_buffer_full(void)
{
    return SSPSTATbits.BF;
}
/**
 * @brief Poll and read the SPI Buffer
 * @return the SPI Buffer
 */
static inline uint8 read_spi_buffer(void)
{
    while (!is_spi_buffer_full());
    return SSPBUF;
}
/**
 * @brief Read the status of the Write Collision of SPI Buffer
 * @return the status of the WCOL flag in the SSPSTAT register
 */
static inline uint8 is_spi_write_collision(void)
{
    return SSPCON1bits.WCOL;
}
/**
 * @brief Read the status of the SPI interrupt flag
 * @return the status of the SPI interrupt flag
 */
static inline uint8 spi_interrupt_flag(void)
{
    return PIR1bits.SSPIF;
}
/**
 * @brief Clear the SPI interrupt flag
 */
static inline void clear_spi_interrupt_flag(void)
{
    PIR1bits.SSPIF = 0;
}
/**
 * @brief Clear the Write Collision flag
 */
static inline void clear_spi_write_collision(void)
{
    SSPCON1bits.WCOL = _SPI_WRITE_NO_COLLISION;
}
/**
 * @brief Clear the SPI Receive Overflow
 */
static inline void clear_spi_receive_overflow(void)
{
    SSPCON1bits.SSPOV = _SPI_SLAVE_RECEIVE_NO_OVERFLOW;
}
/**
 * @brief poll the SPI interrupt flag to wait for the current operation to end
 */
static inline void poll_spi_interrupt_flag(void)
{
    /* Wait for the transmission to complete */
    while (!spi_interrupt_flag());
    /* Clear the interrupt flag for the next operation */
    clear_spi_interrupt_flag(); 
}
/**
 * @brief: Send Data using Master Mode SPI Module
 * @note: It use Polling mechanism to send the data(Polling BF flag)
 * @param spi_obj the SPI module object
 * @param slave_ss_pin the slave select pin to send data to its Slave SPI Module
 * @param data the data to send
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType spi_master_send_data(const spi_t *const spi_obj, 
                                    const pin_config_t *const slave_ss_pin,
                                     const uint8 data)
{
    Std_ReturnType ret = E_OK;
    
    /* Only Master Mode */
    if (NULL == spi_obj || 
        SPI_SLAVE_MODE_SS_ENABLED == spi_obj->spi_mode ||
        SPI_SLAVE_MODE_SS_DISABLED == spi_obj->spi_mode)
    {
        ret = E_NOT_OK;
    }
    else
    {
        /* Select the Slave SPI to send to it */
        if (NULL != slave_ss_pin) ret = gpio_pin_write_logic(slave_ss_pin, GPIO_LOW);
        /* Write To the SSPBUF register to send data */
        SSPBUF = data;
        /* Check the Write Collision Status */
        if (_SPI_WRITE_COLLISION == SSPCON1bits.WCOL)
        {
            /* Collision is detected */
            ret = E_NOT_OK;
            /* Clear the WCOL bit to continue SPI operations */
            clear_spi_write_collision();
        }
        /* Wait for the transmission to complete */
        poll_spi_interrupt_flag();
        /* Send Acknowledgement signal */
        SSPBUF = SPI_ACKNOWLEDGEMENT;
        /* Wait for the transmission to complete */
        poll_spi_interrupt_flag();
        /* Deselect the chosen Slave SPI */
        if (NULL != slave_ss_pin) ret = gpio_pin_write_logic(slave_ss_pin, GPIO_HIGH);
    } 
    return (ret);   
}
/**
 * @brief: Receive Data using Master Mode SPI Module
 * @param spi_obj the SPI module object
 * @param slave_ss_pin the slave select pin to send data to its Slave SPI Module
 * @param data the address to save the data read
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType spi_master_receive_data(const spi_t *const spi_obj, 
                                    const pin_config_t *const slave_ss_pin,
                                     uint8 *const data)
{
    Std_ReturnType ret = E_OK;
    
    /* Only Master Mode */
    if (NULL == spi_obj || 
        SPI_SLAVE_MODE_SS_ENABLED == spi_obj->spi_mode ||
        SPI_SLAVE_MODE_SS_DISABLED == spi_obj->spi_mode)
    {
        ret = E_NOT_OK;
    }
    else
    {
        /* Select the Slave SPI to receive from it */
        if (NULL != slave_ss_pin) ret = gpio_pin_write_logic(slave_ss_pin, GPIO_LOW);
        /* Send Dummy Data to initiates the CLK */
        SSPBUF = SPI_DUMMY_DATA;
        /* Wait for the transmission to complete */
        poll_spi_interrupt_flag();
        /* Poll the BF Bit to wait until any read/write operation is done and Read the SPI Buffer */
        *data = read_spi_buffer();
        if (SPI_DUMMY_DATA == *data)
        {
            ret = E_NOT_OK;
        }
        else
        {
            /* Send Acknowledgement signal */
            SSPBUF = SPI_ACKNOWLEDGEMENT;
            /* Wait for the transmission to complete */
            poll_spi_interrupt_flag();
        }
        /* Deselect the chosen Slave SPI */
        if (NULL != slave_ss_pin) ret = gpio_pin_write_logic(slave_ss_pin, GPIO_HIGH);
    } 
    return (ret);   
}
/**
 * @brief: Send Data using Slave Mode SPI Module
 * @param spi_obj the SPI module object
 * @param data the data to send
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType spi_slave_send_data(const spi_t *const spi_obj, const uint8 data)
{
    Std_ReturnType ret = E_OK;
    uint8 dummy = ZERO_INIT;
    
    if (NULL == spi_obj)
    {
        ret = E_NOT_OK;
    }
    else
    {
        /* Only Slave Mode */
        if (SPI_SLAVE_MODE_SS_ENABLED == spi_obj->spi_mode || 
                SPI_SLAVE_MODE_SS_DISABLED == spi_obj->spi_mode)
        {
            /* Wait for the transmission to complete */
            while (!spi_interrupt_flag());
            /* Clear the interrupt flag for the next operation */
            clear_spi_interrupt_flag();
            /* Read SSPBUF to avoid Overflow */
            dummy = SSPBUF;
            clear_spi_receive_overflow();
            /* Write To the SSPBUF register to send data */
            SSPBUF = data;
            /* Wait for the transmission to complete */
            poll_spi_interrupt_flag();
            if (_SPI_WRITE_COLLISION == SSPCON1bits.WCOL)
            {
                /* Collision is detected */
                ret = E_NOT_OK;
                /* Clear the WCOL bit to continue SPI operations */
                clear_spi_write_collision();
            }
            /* Send Acknowledgement signal */
            SSPBUF = SPI_ACKNOWLEDGEMENT;
            /* Wait for the transmission to complete */
            poll_spi_interrupt_flag();
        }
    } 
    return (ret);  
}
/**
 * @brief: Receive Data using Slave Mode SPI Module
 * @param spi_obj the SPI module object
 * @param data the address to save the data read
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType spi_slave_receive_data(const spi_t *const spi_obj, uint8 *const data)
{
    Std_ReturnType ret = E_OK;
    uint8 dummy = ZERO_INIT;

    /* Only Master Mode */
    if (NULL == spi_obj)
    {
        ret = E_NOT_OK;
    }
    else
    {
        /* Only Slave Mode */
        if (SPI_SLAVE_MODE_SS_ENABLED == spi_obj->spi_mode || 
                SPI_SLAVE_MODE_SS_DISABLED == spi_obj->spi_mode)
        {
            /* Poll the BF Bit to wait until any read/write operation is done and 
             * Read the SPI Buffer to avoid Overflowing */
            dummy = read_spi_buffer();
            clear_spi_receive_overflow();
            /* Read the SSPBUF register */
            *data = SSPBUF;
            if (SPI_DUMMY_DATA == *data)
            {
                ret = E_NOT_OK;
            }
            else
            {
                /* Send Acknowledgement signal */
                SSPBUF = SPI_ACKNOWLEDGEMENT;
                /* Wait for the transmission to complete */
                poll_spi_interrupt_flag();
            }
        }
    }  
    return (ret);
}
