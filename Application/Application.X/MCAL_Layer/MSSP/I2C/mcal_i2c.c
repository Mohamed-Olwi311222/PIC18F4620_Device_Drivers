#include "mcal_i2c.h"
/*---------------Static Data types----------------------------------------------*/
#if I2C_INTERRUPT_FEATURE == INTERRUPT_FEATURE_ENABLE
static INTERRUPT_HANDLER spi_interrupt_handler = NULL; /* A pointer to the callback function when an interrupt is raised */
#endif
/* SCK (Serial Clock) pin */
static const pin_config_t SCK_pin = {.port = PORTC_INDEX, .pin = GPIO_PIN3};
/* SDI (Serial Data) pin */
static const pin_config_t SDI_pin = {.port = PORTC_INDEX, .pin = GPIO_PIN4};
/*---------------Static Data types End------------------------------------------*/

/*---------------Static Helper functions declerations---------------------------*/
static inline void configure_i2c_slew_rate(const i2c_t *const i2c_obj);
static inline void configure_i2c_smbus_specific_inputs(const i2c_t *const i2c_obj);
static inline Std_ReturnType configure_i2c_mode(const i2c_t *const i2c_obj);
static inline Std_ReturnType configure_master_mode(const i2c_t *const i2c_obj);
static inline Std_ReturnType configure_slave_mode(const i2c_t *const i2c_obj);
static inline Std_ReturnType select_slave_mode(const i2c_t *const i2c_obj);
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
        return_status = configure_master_mode(i2c_obj);
    }
    else
    {
        /* Configure Slave Mode if selected */
        return_status = configure_slave_mode(i2c_obj);  
    }
    return (return_status);
}
/**
 * @brief: Configure the I2C Master Mode
 * @param i2c_obj the I2C module object
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType configure_master_mode(const i2c_t *const i2c_obj)
{
    Std_ReturnType return_status = E_NOT_OK;
    
    return (return_status);
}
/**
 * @brief: Configure the I2C Slave Mode
 * @param i2c_obj the I2C module object
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType configure_slave_mode(const i2c_t *const i2c_obj)
{
    Std_ReturnType return_status = E_OK;
    
    /* Configure the SCK Pin to be input */
    SCK_pin.direction = GPIO_DIRECTION_INPUT;
    if (gpio_pin_direction_initialize(&SCK_pin) == E_NOT_OK)
    {
        /* Error in configuring the SCK pin */
        return_status = E_NOT_OK;
    }
    
    /* Configure the SDI Pin to be input */
    SDI_pin.direction = GPIO_DIRECTION_INPUT;
    if (gpio_pin_direction_initialize(&SDI_pin) == E_NOT_OK)
    {
        /* Error in configuring the SDI pin */
        return_status = E_NOT_OK;
    }
    
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
    if (select_slave_mode(i2c_obj) == E_NOT_OK)
    {
        /* Error in setting the mode */
        return_status = E_NOT_OK; 
    }
    
    /* Set the address of the slave mode */
    I2C_SLAVE_SET_ADDR_CONFIG(i2c_obj->i2c_slave_mode_addr);
    return (return_status);
}
/**
 * @brief: Select the Operating I2C Slave Mode
 * @param i2c_obj the I2C module object
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType select_slave_mode(const i2c_t *const i2c_obj)
{
    Std_ReturnType return_status = E_OK;
    i2c_mode_t slave_mode = ZERO_INIT;

    /* Select the I2C Slave Mode */
    switch(i2c_obj->i2c_mode)
    {
        /* 10-bit Addr Modes */    
        case I2C_SLAVE_MODE_10_BIT_ADDR_START_STOP_INTERRUPTS_ON:
            slave_mode = I2C_SLAVE_MODE_10_BIT_ADDR_START_STOP_INTERRUPTS_ON;
            break;
        case I2C_SLAVE_MODE_10_BIT_ADDR_START_STOP_INTERRUPTS_OFF:
            slave_mode = I2C_SLAVE_MODE_10_BIT_ADDR_START_STOP_INTERRUPTS_OFF;
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
    /* If no error happened, set the SSPM3:SSPM0 bits */
    if (E_OK == return_status)
    {
        I2C_SET_OPERATION_MODE(slave_mode);
    }
    return (return_status);
}
