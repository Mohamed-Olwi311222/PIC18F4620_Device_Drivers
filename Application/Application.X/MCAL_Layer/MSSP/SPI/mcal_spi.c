/* 
 * File:   mcal_spi.c
 * Author: Mohamed olwi
 *
 * Created on 12 December 2024, 00:36
 */
#include "mcal_spi.h"
#if SPI_MODULE_ENABLE == MCAL_ENABLED
/*---------------Static Data types----------------------------------------------*/
#if SPI_INTERRUPT_FEATURE == INTERRUPT_FEATURE_ENABLE
static INTERRUPT_HANDLER spi_interrupt_handler = NULL; /* A pointer to the callback function when an interrupt is raised */
static pin_config_t *ss_pin = NULL;                    /* A pointer to the ss pin of the other mcu in the master interrupt mode */
static uint8 *data_received = NULL;                    /* A pointer to the received data in SPI using interrupt */
static uint8 slave_data_received = ZERO_INIT;          /* A Variable to read data before reading/writing to SSPBUF */
static uint8 *slave_data_sent = NULL;                  /* A pointer to the sent data in SPI Slave Mode using interrupt */
#endif
/*---------------Static Data types End------------------------------------------*/

/*---------------Static Helper functions declerations---------------------------*/
static inline void configure_spi_clk(const spi_t *const spi_obj);
static inline Std_ReturnType configure_spi_pins(const spi_t *const spi_obj);
static inline void spi_slave_mode_init(const spi_t *const spi_obj);
static inline void spi_master_mode_init(const spi_t *const spi_obj);
static inline uint8 is_spi_buffer_full(void);
static inline uint8 is_spi_write_collision(void);
static inline void clear_spi_interrupt_flag(void);
static inline void clear_spi_write_collision(void);
static inline uint8 spi_interrupt_flag(void);
static inline void clear_spi_receive_overflow(void);
static inline uint8 read_spi_buffer(void);
/*----Interrupt helper functions----*/
static inline Std_ReturnType configure_spi_interrupt(const spi_t *const spi_obj);
static inline Std_ReturnType set_spi_interrupt_handler(INTERRUPT_HANDLER spi_interrupt);
static inline void configure_spi_interrupt_priority(interrupt_priority_cfg spi_interrupt_priority);
static void inline SPI_SLAVE_RECEIVE_ISR(void);
static void inline SPI_SLAVE_SEND_ISR(void);
/*---------------Static Helper functions declerations End-----------------------*/

/**
 * @brief: Initialize the SPI module
 * @param spi_obj the SPI module object
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType spi_init(const spi_t *const spi_obj)
{
    Std_ReturnType ret = E_OK;
    
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
        /* Configure the SPI Clock polarity and idle state */
        configure_spi_clk(spi_obj);
        /* Configure SPI pins and Sample time of Master/Slave Modes */
        if (E_OK != configure_spi_pins(spi_obj)) ret = E_NOT_OK;
#if SPI_INTERRUPT_FEATURE == INTERRUPT_ENABLE
        /* Configure the SPI Interrupt */
        if (E_OK != configure_spi_interrupt(spi_obj)) ret = E_NOT_OK;
#endif
        /* Enable SPI */
        SPI_SERIAL_PORT_ENABLE_CONFIG();
    }
    return (ret);
}
/**
 * @brief Configure the SPI Clock polarity and the data transition edge
 * @param spi_obj the SPI module object
 */
static inline void configure_spi_clk(const spi_t *const spi_obj)
{
    /* Configure CLK Edge */
    if ( _SPI_TRANSITION_ACTIVE_IDLE == spi_obj->clk_transition_edge)
    {
        SPI_TRANSITION_ACTIVE_IDLE_CONFIG();
    }
    else
    {
        SPI_TRANSITION_IDLE_ACTIVE_CONFIG();
    }
    
    /* Configure CLK Polarity */
    if (_SPI_IDLE_STATE_HIGH == spi_obj->idle_clk_polarity)
    {
        SPI_IDLE_STATE_HIGH_CONFIG();
    }
    else
    {
        SPI_IDLE_STATE_LOW_CONFIG();
    }  
}
/**
 * @brief Configure SPI Pins and Sample time of Master/Slave modes
 * @param spi_obj the SPI module object
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType configure_spi_pins(const spi_t *const spi_obj)
{
    Std_ReturnType ret = E_OK;
    /* SDO (Serial Data Out) pin */
    static const pin_config_t SDO_pin = {.direction = GPIO_DIRECTION_OUTPUT, .port = PORTC_INDEX, .pin = GPIO_PIN5};
    
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
#if SPI_INTERRUPT_FEATURE == INTERRUPT_ENABLE
        /* Disable SPI Interrupt */
        SPI_INTERRUPT_DISABLE();
#endif      
        /* Disable SPI Module */
        SPI_SERIAL_PORT_DISABLE_CONFIG();
    } 
    return (ret);
}
/**
 * @brief Clear the Write Collision flag
 */
static inline void clear_spi_write_collision(void)
{
    SSPCON1bits.WCOL = _SPI_WRITE_NO_COLLISION;
}
#if SPI_INTERRUPT_FEATURE == INTERRUPT_ENABLE
/**
 * @brief Configure the interrupt feature of the SPI Module
 * @param spi_obj the SPI module object
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType configure_spi_interrupt(const spi_t *const spi_obj)
{
    Std_ReturnType ret = E_OK;
    
    /* Disable Interrupt before configuring */
    SPI_INTERRUPT_DISABLE();
    
#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
    /* Enable priority levels */
    INTERRUPT_PRIORITY_levels_ENABLE();
    INTERRUPT_Global_interrupt_LOW_ENABLE();
    INTERRUPT_Global_interrupt_HIGH_ENABLE();
    /* Configure the SPI Module Interrupt priority */
    configure_spi_interrupt_priority(spi_obj->spi_interrupt_priority);
#else
    /* If the interrupt priority is disabled then enable the peripheral interrupt
    and global interrupts */
        INTERRUPT_Peripheral_interrupt_ENABLE();
        INTERRUPT_Global_interrupt_ENABLE();
#endif
    
    /* Clear Interrupt flag */
    SPI_INTERRUPT_FLAG_BIT_CLEAR();
    
    /* Configure the interrupt hadler */
    ret = set_spi_interrupt_handler(spi_obj->spi_interrupt);
    
    /* Enable SPI interrupt */
    SPI_INTERRUPT_ENABLE();
    
    return (ret);
}
/**
 * @brief Set the interrupt handler of the SPI Module
 * @param spi_interrupt the interrupt handler to call when the interrupt flag is set
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType set_spi_interrupt_handler(INTERRUPT_HANDLER spi_interrupt)
{
    Std_ReturnType ret = E_OK;
    
    /* Set the interrupt handler if it isn't null */
    if (spi_interrupt)
    {
        spi_interrupt_handler = spi_interrupt;
    }
    else
    {
        ret = E_NOT_OK;
    }
    return (ret);
}
/**
 * @brief the interrupt service routine of SPI Module Master Mode
 */
void SPI_MASTER_ISR(void)
{
    /* Pull up the SS pin of the slave high to end the comms */
    if (NULL != ss_pin)
    {
        (void) gpio_pin_write_logic(ss_pin, GPIO_HIGH);
    }
    /* Read the data in SPI Master receive Mode */
    if (NULL != data_received)
    {
        *data_received = SSPBUF;
    }
    /* Clear the SPI interrupt flag */
    SPI_INTERRUPT_FLAG_BIT_CLEAR();
    if (NULL != spi_interrupt_handler)
    {
        /* Call the interrupt handler */
        spi_interrupt_handler();
    }
}
/**
 * @brief the interrupt service routine of SPI Module Slave Mode
 * @param mode the mode of the SPI Slave function mode
 */
void SPI_SLAVE_ISR(uint8 mode)
{
    /* Call the Correct Working mode (sending/receiving) */
    if (SPI_SLAVE_SEND_MODE == mode)
    {
        SPI_SLAVE_SEND_ISR();
    }
    else if (SPI_SLAVE_RECEIVE_MODE == mode)
    {
        SPI_SLAVE_RECEIVE_ISR();
    }
    /* Clear the SPI interrupt flag */
    SPI_INTERRUPT_FLAG_BIT_CLEAR();
    /* Call the interrupt handler */
    if (NULL != spi_interrupt_handler)
    {
        spi_interrupt_handler();
    }
}
/**
 * @brief the interrupt service routine of SPI Module Slave Receive Mode
 */
static void inline SPI_SLAVE_RECEIVE_ISR(void)
{
    /* Read the data in SPI Slave receive Mode */
    if (NULL != data_received)
    {
        *data_received = SSPBUF;
    }
    /** Make the pointer point to null to make the functionality of receiving
     *  independent of each other
     */
    data_received = NULL;
}
/**
 * @brief the interrupt service routine of SPI Module Slave Send Mode
 */
static void inline SPI_SLAVE_SEND_ISR(void)
{
    /* Read the Data to avoid Receive Overflow */
    slave_data_received = SSPBUF;
    /* Send the data in SPI Slave Send Mode */
    if (NULL != slave_data_sent)
    {
        SSPBUF = *slave_data_sent;
        if (_SPI_WRITE_COLLISION == SSPCON1bits.WCOL)
        {
            /* Collision is detected */
            /* Clear the WCOL bit to continue SPI operations */
            clear_spi_write_collision();
        }
    }
    /** Make the pointer point to null to make the functionality of sending
     *  independent of each other
     */
    slave_data_sent = NULL;
}
#if INTERRUPT_PRIORITY_LEVELS_ENABLE == INTERRUPT_FEATURE_ENABLE
/**
 * @brief Configure the SPI Interrupt priority depending on the user input
 * @param spi_interrupt_priority the inputted interrupt priority from the user
 */
static inline void configure_spi_interrupt_priority(interrupt_priority_cfg spi_interrupt_priority)
{
    switch (spi_interrupt_priority)
    {
        /* High Priority */
        case INTERRUPT_HIGH_PRIORITY:
            SPI_HIGH_PRIORITY();
            break;
        /* Low Priority */
        case INTERRUPT_LOW_PRIORITY:
            SPI_LOW_PRIORITY();
            break;
    }
}
#endif
/**
 * @brief: Send Data using Master Mode SPI Module
 * @note: It use Polling mechanism to send the data(Polling BF flag)
 * @param spi_obj the SPI module object
 * @param slave_ss_pin the slave select pin to send data to its Slave SPI Module
 *                     (can be null if no ss pin is used)
 * @param data the data to send
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType inline spi_master_send_data(const spi_t *const spi_obj, 
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
        /* Point to the given SS pin of the slave */
        ss_pin = (pin_config_t *)slave_ss_pin;
        /* Select the Slave SPI to send to it */
        if (NULL != ss_pin) ret = gpio_pin_write_logic(ss_pin, GPIO_LOW);
        /* Choose the Correct SPI Mode to call the correct ISR */
        SPI_MODE = SPI_MASTER_MODE;
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
    } 
    return (ret);   
}
/**
 * @brief: Receive Data using Master Mode SPI Module
 * @param spi_obj the SPI module object
 * @param slave_ss_pin the slave select pin to receive data from the Slave SPI Module
 *                     (can be null if no ss pin is used)
 * @param data the address to save the data read
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType inline spi_master_receive_data(const spi_t *const spi_obj, 
                                    const pin_config_t *const slave_ss_pin,
                                     uint8 *const data)
{
    Std_ReturnType ret = E_OK;
    
    /* Only Master Mode */
    if (NULL == spi_obj || 
        NULL == data    ||
        SPI_SLAVE_MODE_SS_ENABLED == spi_obj->spi_mode ||
        SPI_SLAVE_MODE_SS_DISABLED == spi_obj->spi_mode)
    {
        ret = E_NOT_OK;
    }
    else
    {
        /* Point to the given SS pin of the slave */
        ss_pin = (pin_config_t *)slave_ss_pin;
        /* Select the Slave SPI to send to it */
        if (NULL != ss_pin) ret = gpio_pin_write_logic(ss_pin, GPIO_LOW);
        /* Choose the Correct SPI Mode to call the correct ISR */
        SPI_MODE = SPI_MASTER_MODE;
        /* Make the static pointer points to the returned address so when
           the ISR is called the data_received(which is @data) is updated */
        data_received = data;
        /* Check the Write Collision Status */
        if (_SPI_WRITE_COLLISION == SSPCON1bits.WCOL)
        {
            /* Collision is detected */
            ret = E_NOT_OK;
            /* Clear the WCOL bit to continue SPI operations */
            clear_spi_write_collision();
        }
    } 
    return (ret);   
}
/**
 * @brief: Send Data using Slave Mode SPI Module
 * @param spi_obj the SPI module object
 * @param data the data to send
 * @return E_OK if success otherwise E_NOT_OK
 */
Std_ReturnType inline spi_slave_send_data(const spi_t *const spi_obj, const uint8 data)
{
    Std_ReturnType ret = E_OK;
    
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
            /* Make the Current mode is SPI Slave Send to call the correct ISR */
            SPI_MODE = SPI_SLAVE_SEND_MODE;
            slave_data_sent = (uint8 *)&data;
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
Std_ReturnType inline spi_slave_receive_data(const spi_t *const spi_obj, uint8 *const data)
{
    Std_ReturnType ret = E_OK;

    if (NULL == spi_obj || NULL == data)
    {
        ret = E_NOT_OK;
    }
    else
    {
        /* Only Slave Mode */
        if (SPI_SLAVE_MODE_SS_ENABLED == spi_obj->spi_mode || 
                SPI_SLAVE_MODE_SS_DISABLED == spi_obj->spi_mode)
        {
            /* Make the Current mode is SPI Slave receive to call the correct ISR */
            SPI_MODE = SPI_SLAVE_RECEIVE_MODE;
            data_received = data;
        }
    }  
    return (ret);
}
#else
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
 *                     (can be null if no ss pin is used)
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
        /* Wait for the transmission to complete */
        poll_spi_interrupt_flag();
        /* Check the Write Collision Status */
        if (_SPI_WRITE_COLLISION == SSPCON1bits.WCOL)
        {
            /* Collision is detected */
            ret = E_NOT_OK;
            /* Clear the WCOL bit to continue SPI operations */
            clear_spi_write_collision();
        }
        /* Deselect the chosen Slave SPI */
        if (NULL != slave_ss_pin) ret = gpio_pin_write_logic(slave_ss_pin, GPIO_HIGH);
    } 
    return (ret);   
}
/**
 * @brief: Receive Data using Master Mode SPI Module
 * @param spi_obj the SPI module object
 * @param slave_ss_pin the slave select pin to receive data from the Slave SPI Module
 *                     (can be null if no ss pin is used)
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
        NULL == data    ||
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
            /* Wait until the CLK Initiate by the Master */
            dummy = read_spi_buffer();
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

    if (NULL == spi_obj || NULL == data)
    {
        ret = E_NOT_OK;
    }
    else
    {
        /* Only Slave Mode */
        if (SPI_SLAVE_MODE_SS_ENABLED == spi_obj->spi_mode || 
                SPI_SLAVE_MODE_SS_DISABLED == spi_obj->spi_mode)
        {
            /* Read the SPI Buffer to avoid Overflowing */
            dummy = read_spi_buffer();
            clear_spi_receive_overflow();
            /* Read the SSPBUF register */
            *data = read_spi_buffer();
        }
    }  
    return (ret);
}
#endif
#endif