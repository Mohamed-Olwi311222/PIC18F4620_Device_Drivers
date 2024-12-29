/*
 * File:   app.c
 * Author: Mohamed olwi
 *
 * Created on April 1, 2024, 4:43 PM
 */
#include "../../../../ecu_test_init.h"
/*------------------------Master-----------------------------------------------*/

int master_mcu(void)
{  
    Std_ReturnType ret = E_OK;
    uint8 data_mm = 0;
    pin_config_t ss_pin = {
      .port = PORTA_INDEX,
      .direction = GPIO_DIRECTION_OUTPUT,
      .logic = GPIO_HIGH,
      .pin = GPIO_PIN5
    };
    spi_t spi_obj = {
      .spi_mode = SPI_MASTER_MODE_CLK_FOSC_4,
      .data_input_sampe_phase = _SPI_SAMPLE_INPUT_MIDDLE,
      .idle_clk_polarity = _SPI_IDLE_STATE_LOW,
      .clk_transition_edge = _SPI_TRANSITION_IDLE_ACTIVE,
      .spi_interrupt = NULL,
      .spi_interrupt_priority = INTERRUPT_HIGH_PRIORITY
    };
    
    ret |= spi_init(&spi_obj);
    ret = gpio_pin_initialize(&ss_pin);
 
    TRISDbits.RD0 = 0;

    while (1)
    {
        spi_master_send_data(&spi_obj, &ss_pin, 'A');
        __delay_ms(500);
        spi_master_receive_data(&spi_obj, &ss_pin, &data_mm);
        if ('B' == data_mm)
        {
            LATDbits.LATD0 = 1;
        }
    }
    return (ret);
}
