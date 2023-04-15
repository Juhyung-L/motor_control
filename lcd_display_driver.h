//////////////////////////////////////////////////////////////////////////////////////////
//
// File name: lcd_display_driver.h
// Author: Juhyung Lee
// Date: 9/28/22
// Description: This header file contains the function prototypes of the functions 
// implemented in lcd_display_driver.c. The usage of each function is described.
//
//////////////////////////////////////////////////////////////////////////////////////////

#ifndef LCD_DISPLAY_DRIVER_H
#define LCD_DISPLAY_DRIVER_H

void lcd_display_driver_enable(); // turns on the enable bit for a short time to send command to the LCD display
void lcd_display_driver_initialize(); // goes through the initialization steps to start up the LCD display
void lcd_display_driver_clear(); // clears all the characters on the LCD display
void lcd_display_driver_write(char* data, int length); // writes "length" number of letters store in the char array "data" to the LCD display
void display_driver_use_first_line(); // move the cursor to the first line of the LCD display
void display_driver_use_second_line(); // move the cursor to the second line of the LCD display

# endif