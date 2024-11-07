#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

#define I2C_MASTER_SCL_IO GPIO_NUM_20 // Defines the GPIO pin for the I2C clock line (SCL)
#define I2C_MASTER_SDA_IO GPIO_NUM_21 // Defines the GPIO pin for the I2C data line (SDA)
#define I2C_MASTER_NUM I2C_NUM_0      // Specifies the I2C port number (0 in this case)
#define I2C_MASTER_FREQ_HZ 400000     // Sets the I2C clock speed to 400 kHz (for faster communication)
#define LCD_ADDR 0x27                 // I2C address of the PCF8574 I/O expander connected to the LCD
#define UART_NUM UART_NUM_0           // Defines the UART port number (0 for default serial communication)
#define LCD_BACKLIGHT 0x08            // Bit that controls the LCD backlight (when set, the backlight is on)

// LCD command definitions, used to control various functions and settings of the LCD
#define LCD_CMD_CLEAR_DISPLAY 0x01
#define LCD_CMD_RETURN_HOME 0x02
#define LCD_CMD_ENTRY_MODE_SET 0x04
#define LCD_CMD_DISPLAY_CONTROL 0x08
#define LCD_CMD_CURSOR_SHIFT 0x10
#define LCD_CMD_FUNCTION_SET 0x20
#define LCD_CMD_SET_CGRAM_ADDR 0x40
#define LCD_CMD_SET_DDRAM_ADDR 0x80

// Flags for controlling entry mode on the LCD
#define LCD_ENTRY_RIGHT 0x00
#define LCD_ENTRY_LEFT 0x02
#define LCD_ENTRY_SHIFT_INCREMENT 0x01
#define LCD_ENTRY_SHIFT_DECREMENT 0x00

// Flags for controlling the display, cursor, and blinking settings
#define LCD_DISPLAY_ON 0x04
#define LCD_DISPLAY_OFF 0x00
#define LCD_CURSOR_ON 0x02
#define LCD_CURSOR_OFF 0x00
#define LCD_BLINK_ON 0x01
#define LCD_BLINK_OFF 0x00

// Flags for controlling display or cursor shift settings
#define LCD_DISPLAY_MOVE 0x08
#define LCD_CURSOR_MOVE 0x00
#define LCD_MOVE_RIGHT 0x04
#define LCD_MOVE_LEFT 0x00

// Flags for LCD function set mode (bit mode, line number, and font size)
#define LCD_8BIT_MODE 0x10
#define LCD_4BIT_MODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10_DOTS 0x04
#define LCD_5x8_DOTS 0x00

static const char *TAG = "LCD_Project"; // Tag used in logging to identify messages from this program

// Initializes the I2C master interface for communication with the LCD
void i2c_master_init()
{
    ESP_LOGI(TAG, "Initializing I2C master...");
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,                // Sets the I2C mode to master
        .sda_io_num = I2C_MASTER_SDA_IO,        // Configures the SDA pin for I2C
        .scl_io_num = I2C_MASTER_SCL_IO,        // Configures the SCL pin for I2C
        .sda_pullup_en = GPIO_PULLUP_ENABLE,    // Enables pull-up resistor for SDA
        .scl_pullup_en = GPIO_PULLUP_ENABLE,    // Enables pull-up resistor for SCL
        .master.clk_speed = I2C_MASTER_FREQ_HZ, // Sets the clock frequency
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);                // Configures I2C with the above settings
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0); // Installs the I2C driver
    ESP_LOGI(TAG, "I2C master initialized.");
}

// Sends a command to the LCD to configure or control it
void lcd_send_command(uint8_t command)
{
    ESP_LOGI(TAG, "Sending LCD command: 0x%02X", command);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // Creates an I2C command link
    i2c_master_start(cmd);                        // Starts the I2C communication

    // Sends the I2C address with the write mode flag
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);

    // Sends the upper nibble (high 4 bits) with enable bit set
    i2c_master_write_byte(cmd, (command & 0xF0) | LCD_BACKLIGHT | 0x04, true); // RS=0, E=1, Backlight=1
    i2c_master_write_byte(cmd, (command & 0xF0) | LCD_BACKLIGHT, true);        // RS=0, E=0, Backlight=1 (latch)

    // Sends the lower nibble (low 4 bits) with enable bit set
    i2c_master_write_byte(cmd, ((command << 4) & 0xF0) | LCD_BACKLIGHT | 0x04, true); // RS=0, E=1, Backlight=1
    i2c_master_write_byte(cmd, ((command << 4) & 0xF0) | LCD_BACKLIGHT, true);        // RS=0, E=0, Backlight=1 (latch)

    i2c_master_stop(cmd); // Ends the I2C communication

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000)); // Executes the I2C command
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error sending command 0x%02X: %s", command, esp_err_to_name(ret));
    }

    i2c_cmd_link_delete(cmd); // Deletes the command link after execution
    ESP_LOGI(TAG, "LCD command 0x%02X sent", command);
}

// Sends a data nibble to the LCD to display characters or move the cursor
void lcd_send_data(uint8_t nibble)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    // Sends the I2C address in write mode
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);

    // Sends the data nibble with RS=1 (data mode), E=1 (enable high), and backlight on
    i2c_master_write_byte(cmd, (nibble << 4) | LCD_BACKLIGHT | 0x05, true); // RS=1, E=1, Backlight=1
    i2c_master_write_byte(cmd, (nibble << 4) | LCD_BACKLIGHT | 0x01, true); // RS=1, E=0, Backlight=1 (latch)

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000)); // Sends the queued I2C commands
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error sending data nibble: %s", esp_err_to_name(ret));
    }
    i2c_cmd_link_delete(cmd); // Deletes the I2C command link
}

// Initializes the LCD with required settings (4-bit mode, display on, cursor off)
void lcd_init()
{
    ESP_LOGI(TAG, "Initializing LCD...");

    // Sends a dummy command to enable the backlight initially
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, LCD_BACKLIGHT, true); // Enables backlight
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    vTaskDelay(50 / portTICK_PERIOD_MS); // Waits for the LCD to stabilize after powering on

    // Step 1: Sends 0x30 three times to initialize the LCD in 8-bit mode
    lcd_send_command(0x30);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    lcd_send_command(0x30);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    lcd_send_command(0x30);
    vTaskDelay(1 / portTICK_PERIOD_MS);

    // Step 2: Switches to 4-bit mode
    lcd_send_command(LCD_CMD_FUNCTION_SET | LCD_4BIT_MODE);

    // Step 3: Finalizes 4-bit mode setup
    lcd_send_command(LCD_CMD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE | LCD_5x8_DOTS);
    lcd_send_command(LCD_CMD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);
    lcd_send_command(LCD_CMD_ENTRY_MODE_SET | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DECREMENT);
    lcd_send_command(LCD_CMD_CLEAR_DISPLAY); // Clears the display
    vTaskDelay(2 / portTICK_PERIOD_MS);      // Allows time for display clearing

    ESP_LOGI(TAG, "LCD initialized.");
}

// Writes a single character to the LCD
void lcd_write_char(char ch)
{
    uint8_t ascii_value = (uint8_t)ch; // Converts character to its ASCII code

    // Logs the character to display its ASCII value in hexadecimal
    ESP_LOGI(TAG, "Writing character '%c' to LCD (ASCII hex: 0x%02X)", ch, ascii_value);

    // Sends upper nibble (4 most significant bits) of ASCII code
    lcd_send_data((ascii_value & 0xF0) >> 4); // Send upper nibble

    // Sends lower nibble (4 least significant bits) of ASCII code
    lcd_send_data(ascii_value & 0x0F); // Send lower nibble
}

// Displays a string of text on the LCD
void lcd_display_text(const char *text)
{
    ESP_LOGI(TAG, "Displaying text on LCD: \"%s\"", text);

    lcd_send_command(LCD_CMD_CLEAR_DISPLAY); // Clears display before showing new text

    lcd_send_command(LCD_CMD_SET_DDRAM_ADDR | 0x00); // Moves cursor to the start of the first line

    int i;
    for (i = 0; i < 16 && text[i] != '\0'; i++)
    {
        lcd_write_char(text[i]); // Writes each character to the LCD
    }

    // If there's more text, moves to the second line and continues displaying
    if (text[i] != '\0')
    {
        lcd_send_command(LCD_CMD_SET_DDRAM_ADDR | 0x40); // Moves cursor to the second line
        for (int j = 0; j < 16 && text[i] != '\0'; j++, i++)
        {
            lcd_write_char(text[i]);
        }
    }
}

// Main function that sets up I2C, initializes LCD, configures UART, and handles text input
void app_main()
{
    ESP_LOGI(TAG, "Starting application...");
    i2c_master_init(); // Initializes I2C master
    lcd_init();        // Initializes LCD

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT};
    uart_param_config(UART_NUM, &uart_config);              // Configures UART with defined settings
    uart_driver_install(UART_NUM, 1024 * 2, 0, 0, NULL, 0); // Installs UART driver
    ESP_LOGI(TAG, "UART initialized.");

    char buffer[256];
    while (1)
    {                                               
        int len = uart_read_bytes(UART_NUM, buffer, sizeof(buffer) - 1, 20 / portTICK_PERIOD_MS); // Reads UART input

        if (len > 0)
        {
            ESP_LOGI(TAG, "Read %d bytes from UART: '%s'", len, buffer);

            // Logs each character in hexadecimal format for verification
            for (int i = 0; i < len; i++)
            {
                ESP_LOGI(TAG, "Character: '%c' ASCII (hex): 0x%02X", buffer[i], buffer[i]);
            }

            buffer[len] = '\0';       // Null-terminate string at the end of input so we only send the input
            lcd_display_text(buffer); // Displays input on LCD
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
}
