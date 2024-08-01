#include "spiAVR.h"
#include "periph.h"
// #include "timerISR.h"

// Define MAX7219 registers
#define MAX7219_REG_NOOP 0x00
#define MAX7219_REG_DIGIT0 0x01
#define MAX7219_REG_DIGIT1 0x02
#define MAX7219_REG_DIGIT2 0x03
#define MAX7219_REG_DIGIT3 0x04
#define MAX7219_REG_DIGIT4 0x05
#define MAX7219_REG_DIGIT5 0x06
#define MAX7219_REG_DIGIT6 0x07
#define MAX7219_REG_DIGIT7 0x08
#define MAX7219_REG_DECODEMODE 0x09
#define MAX7219_REG_INTENSITY 0x0A
#define MAX7219_REG_SCANLIMIT 0x0B
#define MAX7219_REG_SHUTDOWN 0x0C
#define MAX7219_REG_DISPLAYTEST 0x0F

// Function to write data to a MAX7219 register
void Matrix_write(unsigned char reg, unsigned char data)
{
    PORTB &= ~(1 << PIN_SS); // Set SS to low to select the device
    SPI_SEND(reg);           // Send the register address
    SPI_SEND(data);          // Send the data
    PORTB |= (1 << PIN_SS);  // Set SS to high to deselect the device
}

// Function to initialize the MAX7219 module
void Matrix_init()
{
    Matrix_write(MAX7219_REG_SHUTDOWN, 0x01);    // Exit shutdown mode
    Matrix_write(MAX7219_REG_SCANLIMIT, 0x07);   // Set scan limit to 8 digits
    Matrix_write(MAX7219_REG_DECODEMODE, 0x00);  // Use no decode mode (raw data mode)
    Matrix_write(MAX7219_REG_INTENSITY, 0x08);   // Set medium intensity
    Matrix_write(MAX7219_REG_DISPLAYTEST, 0x00); // Disable display test mode
}
