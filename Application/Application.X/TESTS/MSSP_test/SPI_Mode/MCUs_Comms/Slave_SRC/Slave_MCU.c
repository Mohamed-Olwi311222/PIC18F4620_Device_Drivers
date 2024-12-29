/*
 * File:   app.c
 * Author: Mohamed olwi
 *
 * Created on April 1, 2024, 4:43 PM
 */
#include "../../../../ecu_test_init.h"
/*------------------------SLAVE-----------------------------------------------*/

int slave_mcu(void)
{  
    Std_ReturnType ret = E_OK;
    uint8 data_mm = 0;
    spi_t spi_obj = {
      .spi_mode = SPI_SLAVE_MODE_SS_ENABLED,
      .data_input_sampe_phase = _SPI_SAMPLE_INPUT_MIDDLE,
      .idle_clk_polarity = _SPI_IDLE_STATE_LOW,
      .clk_transition_edge = _SPI_TRANSITION_IDLE_ACTIVE,
      .spi_interrupt = NULL,
      .spi_interrupt_priority = INTERRUPT_HIGH_PRIORITY
    };
    ret |= spi_init(&spi_obj);
 
    TRISDbits.RD0 = 0;
    while (1)
    {
        spi_slave_receive_data(&spi_obj, &data_mm);
        if ('A' == data_mm)
        {
            LATDbits.LATD0 = 1;
            spi_slave_send_data(&spi_obj, 'B');
            __delay_ms(500);
        }
    }
    return (ret);
}
