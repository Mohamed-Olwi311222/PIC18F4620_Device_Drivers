#include "mcal_i2c.h"
/*---------------Static Data types----------------------------------------------*/
#if I2C_INTERRUPT_FEATURE == INTERRUPT_FEATURE_ENABLE
static INTERRUPT_HANDLER spi_interrupt_handler = NULL; /* A pointer to the callback function when an interrupt is raised */
#endif
/* SCK (Serial Clock) pin */
static pin_config_t SCK_pin = {.port = PORTC_INDEX, .pin = GPIO_PIN3};
/* SDI (Serial Data) pin */
static pin_config_t SDI_pin = {.port = PORTC_INDEX, .pin = GPIO_PIN4};
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
/*---------------Static Helper functions definitions----------------------------*/
/**
 * @brief: Configure the SCK pin and SDI pin to be both inputs
 * @return E_OK if success otherwise E_NOT_OK
 */
static inline Std_ReturnType configure_i2c_pins(void)
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
    
    /* Enable or disable master receive */
    if (_I2C_MASTER_RECEIVE_ENABLE == i2c_obj->i2c_master_receive_enable)
    {
        /* Enable Master Receive Mode */
        I2C_MASTER_ENABLE_RECEIVE_MODE_CONFIG();
    }
    else
    {
        /* Disable Master Receive Mode */
        I2C_MASTER_DISABLE_RECEIVE_MODE_CONFIG();
    }
    return (return_status);
}
/**
 * 
 * @param i2c_obj
 * @return 
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
