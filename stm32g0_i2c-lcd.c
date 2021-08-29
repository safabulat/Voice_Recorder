/*
 * name: i2c-lcd.c
 *
 * author: Mahmut Safa BULAT
 *
 * description: This is a driver for 16x02 LCD Display with i2c peripheral.
 * It is based on stm32g031k8 i2c1, if you want to use it with your board/setup;
 *
 * write your own i2c function (only i2c_write will be enough)
 * change the slave address (lcd_address) for your device
 */
#include "stm32g0_i2c-lcd.h"

#define lcd_address 0x27 								// change this according to ur setup

void lcd_delay(volatile uint32_t s){					//delay function
    for(; s>0; s--);
    /* @Name:	delay
	 *
	 * @Brief:	this function causes desired amount of delay.
	 */
}
void lcd_send_cmd (char cmd){							//send commands
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = (char)((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0

	write_I2C(lcd_address,(uint8_t *)data_t,4);

    /* @Name:	lcd_send_cmd
	 *
	 * @Brief:	this function send command(s) as bytes (4xbyte)
	 */
}
void lcd_send_data (char data){							//send data
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = (char)((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0

	write_I2C(lcd_address,(uint8_t *)data_t,4);

    /* @Name:	lcd_send_data
	 *
	 * @Brief:	this function send data(s) as bytes (4xbyte)
	 */
}
void lcd_clear_all(void){								//clear all
	lcd_send_cmd (0x80);
	for (int i=0; i<70; i++)
	{
		lcd_send_data (' ');
	}

    /* @Name:	lcd_clear_all
	 *
	 * @Brief:	this function clears all display by sending ' '.
	 */
}
void lcd_clearLine(int line){							//clear selected line
	if(line==1){
		lcd_send_cmd (0x80);
		for (int i=0; i<16; i++)
		{
			lcd_send_data (' ');
		}
	}
	else if(line==2){
		lcd_send_cmd (0xC0);
		for (int i=0; i<16; i++)
		{
			lcd_send_data (' ');
		}
	}
	else if(line==3){
		lcd_send_cmd (0x38);
		for (int i=0; i<16; i++)
		{
			lcd_send_data (' ');
		}
	}
	else{
		lcd_setCursor(1,1);
		lcd_send_string("LCD Clear ERROR!");
		lcd_setCursor(2,1);
		lcd_send_string("Clearing ALL..!");
		lcd_delay(5000000);
		lcd_clear_all();
	}
    /* @Name:	lcd_clearLine
	 *
	 * @Brief:	this function clears selecting line by sending ' '.
	 *
	 * arg 1: clear 1st line
	 * arg 2: clear 2nd line
	 * arg 3: clear all (dual mode)
	 * else : error , clear all
	 */
}
void lcd_init (void){									//initialize the lcd
	// 4 bit initialisation
	lcd_delay(500000);
	lcd_send_cmd (0x30);
	lcd_delay(50000);
	lcd_send_cmd (0x30);
	lcd_delay(10000);
	lcd_send_cmd (0x30);
	lcd_delay(100000);
	lcd_send_cmd (0x20);  // 4bit mode
	lcd_delay(100000);

	// dislay initialisation
	lcd_send_cmd (0x28); //Display on/off control --> D=0,C=0, B=0  ---> display off
	lcd_delay(10000);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	lcd_delay(10000);
	lcd_send_cmd (0x01);  // clear display
	lcd_delay(10000);
	lcd_delay(10000);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	lcd_delay(10000);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)

}
void lcd_send_string (char *str){						//printf
	while (*str) lcd_send_data (*str++);
}
void lcd_PrintInt(int number){							//printf int
	char numStr[16];
	sprintf(numStr,"%d", number);
	lcd_send_string(numStr);
}
void lcd_PrintFloat(float number, int decimalPoints){	//print float
	char numStr[16];
	sprintf(numStr,"%.*f",decimalPoints, number);
	lcd_send_string(numStr);
}
void lcd_noCursor(void){								//disable cursor
	lcd_send_cmd(0xC);
}
void lcd_cursor(void){									//enable cursor
	lcd_send_cmd(0xF);
}
void lcd_setCursor(uint8_t row, uint8_t col){			//select line
	uint8_t maskData;
	maskData = (col-1)&0x0F;
	if(row==1)								//1st row
	{
		maskData |= (0x80);
		lcd_send_cmd(maskData);
	}
	else if(row==2)							//2nd row
	{
		maskData |= (0xC0);
		lcd_send_cmd(maskData);
	}
	else if(row==3)							//dual mode
	{
		maskData |= (0x38);
		lcd_send_cmd(maskData);
	}
	else									//error: return home
	{
		maskData |= (0x02);
		lcd_send_cmd(maskData);
	}
}
//END
