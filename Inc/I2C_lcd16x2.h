/*
 * VSS=GND, VDD=5V, RW=GND
 */

#ifndef LCD16X2_H
#define LCD16X2_H

#include <stdio.h>

/*i2c address*/
#define I2C_ADDRESS				0x4e //0x4e

//For total comprehension please read the HD44780 datasheet table 6 about the instructions
/*Command list*/
#define LCD_CLEARDISPLAY 		0x01
#define LCD_RETURNHOME 			0x02
#define LCD_ENTRYMODESET 		0x04
#define LCD_DISPLAYCONTROL 		0x08
#define LCD_CURSORSHIFT 		0x10
#define LCD_FUNCTIONSET 		0x20
#define LCD_SETCGRAMADDR 		0x40
#define LCD_SETDDRAMADDR 		0x80

/*Bitmask for commands*/
#define LCD_ID					0x02
#define LCD_S					0x01

#define LCD_DISPLAY_B			0x01
#define LCD_DISPLAY_C			0x02
#define LCD_DISPLAY_D			0x04

#define LCD_SHIFT_RL			0x04
#define LCD_SHIFT_SC			0x08

#define LCD_FUNCTION_F			0x04
#define LCD_FUNCTION_N			0x08
#define LCD_FUNCTION_DL			0x10

//***** Functions prototype *****//

static void LCD1602_writeCommand(uint8_t command);
static void LCD1602_writeData(uint8_t data);


static uint8_t comparar16bits(uint16_t *string1, char *string2);


void LCD1602_i2c_Init(void);
void LCD1602_print(char string[]);
void LCD1602_setCursor(uint8_t row, uint8_t col);
void LCD1602_noCursor(void);
void LCD1602_cursor(void);
void LCD1602_clear(void);
void LCD1602_noBlink(void);
void LCD1602_blink(void);
void LCD1602_shiftToRight(uint8_t num);
void LCD1602_shiftToLeft(uint8_t num);
void LCD1602_PrintInt(int number);
void LCD1602_PrintFloat(float number, int decimalPoints);

void LCD1602_special_caracter_Init(char *a, char *b, char *c, char *d, char *e, char *f, char *g, char *h);
static void LCD1602_print_special_caracter(char *caractere);

#endif /* LCD16X2_H */
