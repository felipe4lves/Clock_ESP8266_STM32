/*
 *The purpose of this code is to use a 16x2 LCD screen with a controller
 *HD44780 from Hitachi which is connected to a PCF8574 controller (for I2C connection),
 *connected to an I2C bus with a stm32l432kc microcontroller
 *
 *Author: Felipe Alves
 *email: alves.f.souza@gmail.com
 *
 * Reference:
 * Mohamed Yaqoob
 * https://www.youtube.com/watch?v=zfn5YqFIqbc&t=44s
 * */

#include <I2C_lcd16x2.h>
#include "stm32l4xx_hal.h"
#include <string.h>

//public variable
uint8_t displaycontrol;

uint16_t copia_vetor_de_caracteres[8];
const uint8_t a_agudo_minusculo[]={0x02,0x04,0x0e,0x01,0x0f,0x11,0x0f,0x00},
			  e_agudo_minusculo[]={0x02,0x04,0x0e,0x11,0x1f,0x10,0x0e,0x00},
			  i_agudo_minusculo[]={0x02,0x04,0x00,0x0c,0x04,0x04,0x0e,0x00},
			  o_agudo_minusculo[]={0x02,0x04,0x00,0x0e,0x11,0x11,0x0e,0x00},
			  u_agudo_minusculo[]={0x02,0x04,0x00,0x11,0x11,0x13,0x0d,0x00},
			  cedilha_minusculo[]={0x00,0x00,0x0e,0x10,0x11,0x0e,0x04,0x0c},
			  a_til_minusculo[]={0x05,0x0a,0x0e,0x01,0x0f,0x11,0x0f,0x00},
			  o_circunflexo_minusculo[]={0x04,0x0a,0x00,0x0e,0x11,0x11,0x0e,0x00},
			  a_circunflexo_minusculo[]={0x04,0x0a,0x0e,0x01,0x0f,0x11,0x0f,0x00},
			  e_circunflexo_minusculo[]={0x04,0x0a,0x0e,0x11,0x1f,0x10,0x0e,0x00},
			  o_til_minusculo[]={0x05,0x0a,0x00,0x0e,0x11,0x11,0x0e,0x00},
			  a_grave_minusculo[]={0x08,0x04,0x0e,0x01,0x0f,0x11,0x0f,0x00},
			  vazio[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
			  a_agudo_maiusculo[]={0x2,0x4,0xa,0x11,0x1f,0x11,0x11,0x0},
			  e_agudo_maiusculo[]={0x2,0x4,0x1f,0x10,0x1e,0x10,0x1f,0x0},
			  i_agudo_maisculo[]={0x2,0x4,0xe,0x4,0x4,0x4,0xe,0x0},
			  o_agudo_maisculo[]={0x2,0x4,0xe,0x11,0x11,0x11,0xe,0x0},
			  u_agudo_maisculo[]={0x2,0x4,0x11,0x11,0x11,0x11,0xe,0x0},
			  a_circunflexo_maiusculo[]={0x4,0xa,0x0,0xe,0x11,0x1f,0x11,0x0},
			  e_circunflexo_maiusculo[]={0x4,0xa,0x1f,0x10,0x1e,0x10,0x1f,0x0},
			  i_circunflexo_maisculo[]={0x4,0xa,0x0,0xe,0x4,0x4,0xe,0x0},
			  o_circunflexo_maisculo[]={0x4,0xa,0xe,0x11,0x11,0x11,0xe,0x0},
			  u_circunflexo_maisculo[]={0x4,0xa,0x11,0x11,0x11,0x11,0xe,0x0},
			  a_til_maisculo[]={0xd,0x12,0x4,0xa,0x11,0x1f,0x11,0x0},
			  o_til_maiusculo[]={0xd,0x12,0xe,0x11,0x11,0x11,0xe,0x0},

			  lua_nova_esquerda[]={0x03,0x0C,0x08,0x10,0x10,0x08,0x0C,0x03},
			  lua_nova_direita[]={0x18,0x06,0x02,0x01,0x01,0x02,0x06,0x18},
			  lua_cheia_direita[]={0x18,0x1E,0x1E,0x1F,0x1F,0x1E,0x1E,0x18},
			  lua_cheia_esquerda[]={0x03,0x0F,0x0F,0x1F,0x1F,0x0F,0x0F,0x03},
			  lua_crescente[]={0x03,0x0C,0x0C,0x1C,0x1C,0x0C,0x0C,0x03},
			  lua_crescente_gibosa[]={0x18,0x1E,0x1A,0x19,0x19,0x1A,0x1E,0x18},
			  lua_minguante_gibosa[]={0x03,0x0F,0x0B,0x13,0x13,0x0B,0x0F,0x03},
			  lua_minguante[]={0x18,0x06,0x06,0x07,0x07,0x06,0x06,0x18};

//Private variable
I2C_HandleTypeDef hi2c1;

//Private function
//Write a command on the LCD controller
static void LCD1602_writeCommand(uint8_t command)
{
	int time=1;
	uint8_t caractere;
	uint8_t LSB_nibble = command & 0xF, MSB_nibble = (command>>4)&0xF;

	//character
	//first part
	caractere=(MSB_nibble<<4) | 0xc;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS, &caractere, 1, 100);
	HAL_Delay(time);
	caractere=(MSB_nibble<<4) | 0x8;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS, &caractere, 1, 100);

	HAL_Delay(time);

	//second part
	caractere=(LSB_nibble<<4) | 0xc;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS, &caractere, 1, 100);
	HAL_Delay(time);
	caractere=(LSB_nibble<<4) | 0x8;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS, &caractere, 1, 100);

	HAL_Delay(time);
}

//Write a character on the LCD screen
static void LCD1602_writeData(uint8_t data)
{
	int time=1;
	uint8_t caractere;
	uint8_t LSB_nibble = data&0xF, MSB_nibble = (data>>4)&0xF;

	//character
	//first part
	caractere=(MSB_nibble<<4) | 0xd;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS, &caractere, 1, HAL_MAX_DELAY);
	HAL_Delay(time);
	caractere=(MSB_nibble<<4) | 0x9;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS, &caractere, 1, HAL_MAX_DELAY);
	HAL_Delay(time);

	//second part
	caractere=(LSB_nibble<<4) | 0xd;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS, &caractere, 1, HAL_MAX_DELAY);
	HAL_Delay(time);
	caractere=(LSB_nibble<<4) | 0x9;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS, &caractere, 1, HAL_MAX_DELAY);
	HAL_Delay(time);
}

static uint8_t comparar16bits(uint16_t *string1, char *string2)
{
	char string1_para_char[2], string2_para_char[2];

	string1_para_char[0]=*string1 & 0x00ff;
	string1_para_char[1]=(*string1>>8) & 0x00ff;

	string2_para_char[0]=(string2_para_char[0] & 0x00) | *(string2+1);
	string2_para_char[1]=(string2_para_char[1] & 0x00) | *string2;

	if((strncmp(string1_para_char,string2_para_char,2)==0))
	{

	}
	else
		return 1;
	return 0;
}
//public functions
//Initializes the display (this function must be performed first to set the LCD screen)
void LCD1602_i2c_Init(void)
{
	HAL_Delay(150);

	//Inicialização
	LCD1602_writeCommand(0x33);
	LCD1602_writeCommand(0x32);

	 //Function Set
	LCD1602_writeCommand(LCD_FUNCTIONSET | LCD_FUNCTION_N);

	//Display on/off control
	displaycontrol=LCD_DISPLAYCONTROL | LCD_DISPLAY_B | LCD_DISPLAY_C | LCD_DISPLAY_D;
	LCD1602_writeCommand(displaycontrol);

	//Clear display
	LCD1602_writeCommand(LCD_CLEARDISPLAY);

	//Entry mode set
	LCD1602_writeCommand(LCD_ENTRYMODESET | LCD_ID);

}

//Write a string on the screen (if this function is called right after the "lcd1602_i2c_Init" function then the string will be written from the first
// row in the first column)
void LCD1602_print(char string[])
{
	uint8_t flag=0;
	uint16_t caractere=0;
	char transformacao[2];

	for(uint8_t i=0; string[i]!='\0'; i++)
	{
		for(uint8_t j=0;j<8;j++)
		{
			caractere=(caractere | string[i])<<8 | (string[i+1] & 0x00ff);
			if(caractere==copia_vetor_de_caracteres[j])
			{
				flag=1;
				break;
			}
			caractere=0;
		}
		transformacao[0]=(caractere>>8) & 0x00ff;
		transformacao[1]=caractere & 0x00ff;
		if(flag==0)
		{
			LCD1602_writeData(string[i]);
		}
		else
		{
			LCD1602_print_special_caracter(transformacao);
			flag=0;
			i++;
		}
		caractere=0;
	}
}

//Put the cursor on a particular row and column of the LCD screen
void LCD1602_setCursor(uint8_t row, uint8_t col)
{
	uint8_t maskData;
	maskData = (col-1)&0x0F;
	if(row==1)
	{
		maskData |= (0x80);
		LCD1602_writeCommand(maskData);
	}
	else
	{
		maskData |= (0xc0);
		LCD1602_writeCommand(maskData);
	}
}

//Disable cursor
void LCD1602_noCursor(void)
{
	LCD1602_writeCommand(displaycontrol & ~(LCD_DISPLAY_C));
}

//Enable cursor
void LCD1602_cursor(void)
{
	LCD1602_writeCommand(displaycontrol | LCD_DISPLAY_C);
}

//Clean the LCD screen
void LCD1602_clear(void)
{
	LCD1602_writeCommand(LCD_CLEARDISPLAY);
}

//no blink cursor
void LCD1602_noBlink(void)
{
	LCD1602_writeCommand(displaycontrol & ~(LCD_DISPLAY_B));
}

//blink cursor
void LCD1602_blink(void)
{
	LCD1602_writeCommand(displaycontrol | LCD_DISPLAY_B);
}

//shift the display to right
void LCD1602_shiftToRight(uint8_t num)
{
	for(uint8_t i=0; i<num;i++)
	{
		LCD1602_writeCommand(LCD_CURSORSHIFT | LCD_SHIFT_RL | LCD_SHIFT_SC);
	}
}

//shift the display to left
void LCD1602_shiftToLeft(uint8_t num)
{
	for(uint8_t i=0; i<num;i++)
	{
		LCD1602_writeCommand(LCD_CURSORSHIFT | LCD_SHIFT_SC);
	}
}

//Write an integer number
void LCD1602_PrintInt(int number)
{
	char numStr[16];
	sprintf(numStr,"%d", number);
	LCD1602_print(numStr);
}
//write a float number
void LCD1602_PrintFloat(float number, int decimalPoints)
{
	char numStr[16];
	sprintf(numStr,"%.*f",decimalPoints, number);
	LCD1602_print(numStr);
}

/*Write a special character on the screen. You must enter the character in double quotation marks.*/
static void LCD1602_print_special_caracter(char *caractere)
{
	for(uint8_t j=0; j<8; j++)
	{
		if(comparar16bits(&copia_vetor_de_caracteres[j], caractere)==0)
		{
			LCD1602_writeData(j);
		}
		else
		{
			continue;
		}
	}
}
/* Records 8 special characters on the LCD's CGRAM. You must type the characters in double quotation marks.
 * The characters available are: á, é, í, ó, ú, â, ê, ô, à, ã, õ, ç.*/
void LCD1602_special_caracter_Init(char *a, char *b, char *c, char *d, char *e, char *f, char *g, char *h)
{
	uint16_t vetor_de_caracteres[8];

	for(uint8_t z=0;z<8;z++)
	{
		vetor_de_caracteres[z]=0;
		if(z==0)
		{
			vetor_de_caracteres[z]=(vetor_de_caracteres[z] | *a);
			vetor_de_caracteres[z]=vetor_de_caracteres[z] <<8;
			vetor_de_caracteres[z]=vetor_de_caracteres[z] | (0x00ff & *(a+1));
		}
		else if(z==1)
		{
			vetor_de_caracteres[z]=(vetor_de_caracteres[z] | *b);
			vetor_de_caracteres[z]=vetor_de_caracteres[z] <<8;
			vetor_de_caracteres[z]=vetor_de_caracteres[z] | (0x00ff & *(b+1));
		}
		else if(z==2)
		{
			vetor_de_caracteres[z]=(vetor_de_caracteres[z] | *c);
			vetor_de_caracteres[z]=vetor_de_caracteres[z] <<8;
			vetor_de_caracteres[z]=vetor_de_caracteres[z] | (0x00ff & *(c+1));
		}
		else if(z==3)
		{
			vetor_de_caracteres[z]=(vetor_de_caracteres[z] | *d);
			vetor_de_caracteres[z]=vetor_de_caracteres[z] <<8;
			vetor_de_caracteres[z]=vetor_de_caracteres[z] | (0x00ff & *(d+1));
		}
		else if(z==4)
		{
			vetor_de_caracteres[z]=(vetor_de_caracteres[z] | *e);
			vetor_de_caracteres[z]=vetor_de_caracteres[z] <<8;
			vetor_de_caracteres[z]=vetor_de_caracteres[z] | (0x00ff & *(e+1));
		}
		else if(z==5)
		{
			vetor_de_caracteres[z]=(vetor_de_caracteres[z] | *f);
			vetor_de_caracteres[z]=vetor_de_caracteres[z] <<8;
			vetor_de_caracteres[z]=vetor_de_caracteres[z] | (0x00ff & *(f+1));
		}
		else if(z==6)
		{
			vetor_de_caracteres[z]=(vetor_de_caracteres[z] | *g);
			vetor_de_caracteres[z]=vetor_de_caracteres[z] <<8;
			vetor_de_caracteres[z]=vetor_de_caracteres[z] | (0x00ff & *(g+1));
		}
		else if(z==7)
		{
			vetor_de_caracteres[z]=(vetor_de_caracteres[z] | *h);
			vetor_de_caracteres[z]=vetor_de_caracteres[z] <<8;
			vetor_de_caracteres[z]=vetor_de_caracteres[z] | (0x00ff & *(h+1));
		}
		else
		{
			continue;
		}
	}
	//char vetor_de_caracteres[]={a, b, c, d, e, f, g, h};
	//uint16_t vetor_de_caracteres2[]={a+1, b+1, c+1, d+1, e+1, f+1, g+1, h+1};
	for(uint8_t z=0; z<8; z++)
	{
		copia_vetor_de_caracteres[z]=vetor_de_caracteres[z];
	}
	LCD1602_writeCommand(LCD_SETCGRAMADDR);

	for(uint8_t j=0; j<8; j++)
	{
		if(comparar16bits(&vetor_de_caracteres[j], "á")==0)
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(a_agudo_minusculo[i]);
				HAL_Delay(1);
			}
		}
		else if(comparar16bits(&vetor_de_caracteres[j],"é")==0)
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(e_agudo_minusculo[i]);
				HAL_Delay(1);
			}
		}
		else if(comparar16bits(&vetor_de_caracteres[j],"í")==0)
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(i_agudo_minusculo[i]);
				HAL_Delay(1);
			}
		}
		else if(comparar16bits(&vetor_de_caracteres[j],"ó")==0)
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(o_agudo_minusculo[i]);
				HAL_Delay(1);
			}
		}
		else if(comparar16bits(&vetor_de_caracteres[j],"ú")==0)
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(u_agudo_minusculo[i]);
				HAL_Delay(1);
			}
		}
		else if(comparar16bits(&vetor_de_caracteres[j],"ç")==0)
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(cedilha_minusculo[i]);
				HAL_Delay(1);
			}
		}
		else if(comparar16bits(&vetor_de_caracteres[j],"â")==0)
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(a_circunflexo_minusculo[i]);
				HAL_Delay(1);
			}
		}
		else if(comparar16bits(&vetor_de_caracteres[j],"ê")==0)
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(e_circunflexo_minusculo[i]);
				HAL_Delay(1);
			}
		}
		else if(comparar16bits(&vetor_de_caracteres[j],"ô")==0)
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(o_circunflexo_minusculo[i]);
				HAL_Delay(1);
			}
		}
		else if(comparar16bits(&vetor_de_caracteres[j],"õ")==0)
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(o_til_minusculo[i]);
				HAL_Delay(1);
			}
		}
		else if(comparar16bits(&vetor_de_caracteres[j],"à")==0)
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(a_grave_minusculo[i]);
				HAL_Delay(1);
			}
		}
		else if(comparar16bits(&vetor_de_caracteres[j],"ã")==0)
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(a_til_minusculo[i]);
				HAL_Delay(1);
			}
		}
		else if(comparar16bits(&vetor_de_caracteres[j],"L1")==0)//lua_nova_esquerda
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(lua_nova_esquerda[i]);
				HAL_Delay(1);
			}
		}
		else if(comparar16bits(&vetor_de_caracteres[j],"L2")==0)//lua_nova_direita
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(lua_nova_direita[i]);
				HAL_Delay(1);
			}
		}
		else if(comparar16bits(&vetor_de_caracteres[j],"L3")==0)//lua_cheia_direita
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(lua_cheia_direita[i]);
				HAL_Delay(1);
			}
		}
		else if(comparar16bits(&vetor_de_caracteres[j],"L4")==0)//lua_cheia_esquerda
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(lua_cheia_esquerda[i]);
				HAL_Delay(1);
			}
		}
		else if(comparar16bits(&vetor_de_caracteres[j],"L5")==0)//lua_crescente_gibosa_direita
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(lua_crescente_gibosa[i]);
				HAL_Delay(1);
			}
		}
		else if(comparar16bits(&vetor_de_caracteres[j],"L6")==0)//lua_minguante_gibosa_esquerda
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(lua_minguante_gibosa[i]);
				HAL_Delay(1);
			}
		}
		else if(comparar16bits(&vetor_de_caracteres[j],"L7")==0)//lua_minguante_direita
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(lua_minguante[i]);
				HAL_Delay(1);
			}
		}
		else if(comparar16bits(&vetor_de_caracteres[j],"L8")==0)//lua_crescente_esquerda
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(lua_crescente[i]);
				HAL_Delay(1);
			}
		}
		else
		{
			for(uint8_t i=0; i<8; i++)
			{
				LCD1602_writeData(vazio[i]);
				HAL_Delay(1);
			}
		}
	}
	LCD1602_writeCommand(LCD_CLEARDISPLAY);
};


