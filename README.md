# ESP32 LCD1602 Display with I2C and UART Input

This project demonstrates how to interface an **ESP32** with an **LCD1602 display** using the **PCF8574 I/O expander** over the I2C protocol. Additionally, the ESP32 receives input via UART, allowing the display of text input from a serial terminal.

## Features

- I2C communication with LCD1602 display using the PCF8574 I/O expander.
- UART input to display characters received from the serial terminal.
- Initialization of the LCD display in 4-bit mode with backlight control.
- Display functionality supporting two lines of text (16x2 characters).

## Hardware Requirements

- **ESP32** development board
- **1602 LCD** display with **I2C backpack** (PCF8574 I/O expander)
- Jumper wires for connections

## Software Requirements

- **ESP-IDF** v5.x or higher
- **UART** and **I2C** libraries (included with ESP-IDF)

## Connections

| ESP32 Pin        | LCD (PCF8574 I2C Backpack) |
|------------------|----------------------------|
| GPIO20           | SCL                        |
| GPIO21           | SDA                        |
| GND              | GND                        |
| 5V               | VCC                        |

**Note**: Make sure your LCD backpack is compatible with 3.3V if using that power level. Otherwise, use 5V if supported.

## I2C and UART Configuration

- **I2C Address**: `0x27` (default for most PCF8574 I2C backpacks)
- **I2C Frequency**: 100 kHz (standard mode for PCF8574)
- **UART Configuration**:
  - Baud Rate: 115200
  - Data Bits: 8
  - Parity: None
  - Stop Bits: 1

## Setup and Usage

1. **Clone this repository** and open it in your ESP-IDF environment.
2. Connect your ESP32 to the LCD display according to the connection table above.
3. Configure your serial terminal to send data over UART to the ESP32 at **115200 baud**.
4. **Flash** the code to your ESP32:
   ```bash
   idf.py build flash monitor
