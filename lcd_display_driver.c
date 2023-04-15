//////////////////////////////////////////////////////////////////////////////////////////
//
// File name: lcd_display_driver.c
// Author: Juhyung Lee
// Date: 9/28/22
// Description: This source file implements the functions defined in the header file
// lcd_display_driver.h.
// 
//////////////////////////////////////////////////////////////////////////////////////////

#include <xc.h>
#include <stdint.h>
#include "lcd_display_driver.h"

// set to input -> read -> 1
// set to output -> write -> 0
// registers used for the LCD display:
// E = D4
// RW = D5, RS = B15
// DB0 = E0, DB1 = E1, DB2= E2, DB3 = E3, DB4 = E4, DB5 = E5, DB6 = E6, DB7 = E7

// function prototypes for helper fucntions
void set_lcd_RS(int value);
void set_lcd_RW(int value);
void set_lcd_DB(uint8_t value);
void set_output_pins();


void lcd_display_driver_enable() // turn on pin 81 for a short time
{	
	LATDbits.LATD4 = 1; // set register D bit 4 to 1
    int i;
	for(i = 0; i < 2000; ++i)
	{ }
    LATDbits.LATD4 = 0; // set register D bit 4 back to 0
}

void lcd_display_driver_initialize() //
{
	uint8_t DB_value;

	// set the E, RS, RW, and first 8 bits of register E to output
	set_output_pins();

	// step 1
	set_lcd_RS(0); // set RS to 0
	set_lcd_RW(0); // set RW to 0
	DB_value = 0b00111000; // 5x7 dots, 2-line mode
	set_lcd_DB(DB_value); // set the bits on DB to value
	lcd_display_driver_enable(); // send the configuration
	
	// step 2
	set_lcd_RS(0);
	set_lcd_RW(0);
	DB_value = 0b00001101; // blink on, cursor off, display on
	set_lcd_DB(DB_value);
	lcd_display_driver_enable();

	// step 3
	set_lcd_RS(0);
	set_lcd_RW(0);
	DB_value = 0b00000001;
	set_lcd_DB(DB_value);
	lcd_display_driver_enable();

	// step 4
	set_lcd_RS(0);
	set_lcd_RW(0);
	DB_value = 0b00000110;
	set_lcd_DB(DB_value);
	lcd_display_driver_enable();
}

void lcd_display_driver_clear()
{
    uint8_t value;
    
	set_lcd_RS(0);
	set_lcd_RW(0);
	value = 0b00000001;
	set_lcd_DB(value);
	lcd_display_driver_enable();
}

void lcd_display_driver_write(char* data, int length)
{
    char each_char;
    
    int i;
    for(i = 0; i < length; ++i)
    {
        set_lcd_RS(1);
        set_lcd_RW(0);
        each_char = data[i]; // store each character in the array "data" to the char variable "each_char"
        set_lcd_DB(each_char); // write the ASCII value of the char to the DB bits
        lcd_display_driver_enable(); // send the write command
    }
}

void display_driver_use_first_line()
{
    uint8_t DB_value;

	set_lcd_RS(0);
	set_lcd_RW(0);
	DB_value = 0x80; // set DB bits to 0x80
    set_lcd_DB(DB_value);
    lcd_display_driver_enable();
}

void display_driver_use_second_line()
{
    uint8_t DB_value;

	set_lcd_RS(0);
	set_lcd_RW(0);
	DB_value = 0xC0; // set DB bits to 0xC0
    set_lcd_DB(DB_value);
    lcd_display_driver_enable();
}

void set_lcd_RS(int value)
{
	LATBbits.LATB15 = value; // set register B bit 15 to value
}

void set_lcd_RW(int value)
{
	LATDbits.LATD5 = value; // set register D bit 5 to value
}

void set_lcd_DB(uint8_t value)
{
	LATE = value; // set first 8 bits on register E as value
}

void set_output_pins()
{
	TRISDbits.TRISD4 = 0; // E bit (enable)
	TRISBbits.TRISB15 = 0; // RS bit
	TRISDbits.TRISD5 = 0; // RW bit
	TRISE = 0xFF00; // set first 8 bits on register E to output
}
