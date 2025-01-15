# PIC18F4620_Device_Drivers

Welcome to **PIC18F4620_Device_Drivers**! This repository contains my custom implementation of device drivers for the PIC18F4620 microcontroller, along with some ECU (Electronic Control Unit) drivers. These drivers are designed to help you interface and manage various peripherals of the PIC18F4620 MCU for embedded applications.

## Overview

The PIC18F4620 is a versatile microcontroller from Microchip's 8-bit family, known for its rich set of peripherals and suitability for embedded system applications. This repository aims to provide reusable and reliable driver implementations that facilitate development and make interfacing with hardware components simpler and more efficient.

## Features
- **Peripheral Drivers**:
  - Timer modules (Timer0, Timer1, Timer2, Timer3)
  - EEPROM module (electrically erasable programmable read-only memory)
  - External INTx and PORTB On-Change interrupts
  - ADC (Analog-to-Digital Converter)
  - PWM (Pulse Width Modulation) for CCP modules
  - UART (Universal Asynchronous Receiver-Transmitter)
  - I2C and SPI communication protocols
- **ECU Drivers**:
  - LCD display driver
  - Keypad interface driver
  - LED and button control
  - Relay driver
  - Seven Segment driver
  - DC Motor driver

## Project Structure

```
PIC18F4620_Device_Drivers/
|-- Application/Application.X                   # Source files for driver implementations and Header files for driver functions and structures
|-- Application/Application.X/TESTS             # Sample projects showcasing driver usage
|-- Proteus_Projects/                           # For Proteus Projects design
|-- README.md                                   # Project overview and information (this file)
```

## Getting Started

### Prerequisites
- **MPLAB X IDE**: Ensure you have the latest version of MPLAB X installed.
- **XC8 Compiler**: The drivers are written for the XC8 compiler. Ensure the appropriate version is installed and configured (recommended v2.46).
- **PICkit or Similar Programmer**: Required for uploading the compiled code to the PIC18F4620.

### Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/Mohamed-Olwi311222/PIC18F4620_Device_Drivers.git
   ```
2. Open the project in MPLAB X IDE and set up the build configuration.
3. Include the header files directories in your project path.

### Example Usage
Refer to the `"Application/Application.X/TESTS"` directory for sample code demonstrating how to:
- Configure and use the ADC for analog sensor readings.
- Set up PWM channels for motor control.
- Send and receive data using UART communication.

```c
#include "MCAL_Layer/ADC/hal_adc.h"
#include "MCAL_Layer/Timers/Timer0/hal_timer0.h"

void main(void) {
    // Initialize Timer0
    timer0_init(&timer0_obj);
    
    // Initialize ADC
    adc_init(&adc_obj);
    
    // Main loop
    while (1) {
        // Your code here
    }
}
```

## Contributing
Contributions are welcome! If you'd like to contribute:
- Fork the repository.
- Create a new branch for your feature or bug fix.
- Submit a pull request with a detailed explanation of your changes.
