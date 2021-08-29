/*
 * name: i2c-lcd.h
 *
 * author: Mahmut Safa BULAT
 *
 * description: This is a driver for 16x02 LCD Display with i2c peripheral.
 * It is based on stm32g031k8 i2c1, if you want to use it with your board/setup;
 *
 * write your own i2c function (only i2c_write will be enough)
 * change the slave address (lcd_address) for your device
 */
#ifndef STM32_G0_I2C_LCD_H
#define STM32_G0_I2C_LCD_H

#include "stm32g0xx.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

void lcd_init(void);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(char *str);
void lcd_clear_all(void);
void lcd_clearLine(int line);
void lcd_PrintInt(int number);
void lcd_PrintFloat(float number, int decimalPoints);
void lcd_noCursor(void);
void lcd_cursor(void);
void lcd_setCursor(uint8_t row, uint8_t col);
void lcd_delay(volatile uint32_t s);

//externing i2c write function from main.c file.
extern void write_I2C(uint8_t devAddr,uint8_t* data, int numBytes);

//END
#endif
