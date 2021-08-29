/*
 * name: main.c
 *
 * author: Mahmut Safa BULAT
 *
 * description: This is a project3 main file for ELEC334 project3;
 * "Voice Recorder"
 * Every section is properly explained with comments.
 * You can find pin connections at the end of the code (down below the main)
 */

//********************************Includes*******************************//
#include "stm32g0xx.h"
#include "stm32g0_i2c-lcd.h"
#include "stdio.h"
//**************************Includes * END*******************************//
//*****************************Definitions*******************************//
#define eeprom_1_address	0x50
#define eeprom_2_address	0x54
#define lcd_address			0x27
#define write_page 			128
//********************************Definitions * END******************************//

//********************************Global Variables*******************************//
int state,full,idle =0;			//state flags
int track_select=0;				//track select
int recorded[4]={0,0,0,0};		//if recorded set to 1; exaple record[0] = 1 for track 1
int isEEPfull,pass;				//FULL state variables
int i2c_rec_play=0;				//main_rec_play_delete flags
int monitor;					//is monitor feedback open?

int pressedKey;					//get key
int clock_counter=0;			//tim1 counter
int second;						//second counter
int play, record=0;				//play, record flags

int data_ready=0;				//is data ready to write or read

uint8_t inputCaptureVal;		//adc capture value
uint8_t pwmvalue_save[32];		//variable to send eeprom
uint8_t pwmvalue_load;			//variable to load from eeprom
uint8_t track[2];				//variable to load from eeprom buffer

uint16_t regAddr1_tracker=0x0;		//address 1 tracker from 0 to 0xD000
uint16_t regAddr2_tracker=0x8000;	//address 2 tracker from 0x8000 to 0xFFFF
//******************************Global Variables * END*************************//

//**********************************Functions**********************************//
void sysclock_64M(){										//Change system clock to 64 Mhz

	RCC->CR |= (1U << 8);			//Enable HSI16
	while(!(RCC->CR & (1U << 10)));	//wait for cyrstal

	FLASH->ACR |= (1U << 8);		//CPU Prefetch enable
	FLASH->ACR &= ~(1U << 0);		//Flash memory access latency; reset
	FLASH->ACR &= ~(1U << 1);		//Flash memory access latency; reset
	FLASH->ACR &= ~(1U << 2);		//Flash memory access latency; reset
	FLASH->ACR |= (1U << 0);		//Flash memory access latency; set to 1 wait state

	RCC->PLLCFGR |= (2U << 0);		//Select HSI16 for pll source
	RCC->PLLCFGR |= (0U << 4);	//PLLM division /1
	RCC->PLLCFGR &= ~(0x7FU << 8);	//PLL frequency multiplication factor N; reset
	RCC->PLLCFGR |= (0x8U << 8);	//PLL frequency multiplication factor N; x8
	RCC->PLLCFGR &= ~(1U << 29);	//PLL VCO division factor R for PLLRCLK clock output; reset
	RCC->PLLCFGR &= ~(1U << 30);	//PLL VCO division factor R for PLLRCLK clock output; reset
	RCC->PLLCFGR &= ~(1U << 31);	//PLL VCO division factor R for PLLRCLK clock output; reset
	RCC->PLLCFGR |= (1U << 29);		//PLL VCO division factor R for PLLRCLK clock output; /2
	RCC->PLLCFGR |= (1U << 28);		//PLLRCLK clock output enable

	RCC->CFGR &= ~(1U << 11);		//AHB prescaler to /1
	RCC->CFGR &= ~(1U << 14);		//APB prescaler to /1

	RCC->CR |= (1U << 24);			//PLLON: PLL enable
	while(!(RCC->CR & (1U << 25)));//Wait for PLL to stable

	RCC->CFGR &= ~(1U << 0);		//System clock switch to; reset
	RCC->CFGR &= ~(1U << 1);		//System clock switch to; reset
	RCC->CFGR |= (2U << 0);			//System clock switch to; PLLRCLK

	while(!(RCC->CFGR & (2U << 3)));//Wait for PLL to stable

	SystemCoreClockUpdate();		//Update system clock

	SysTick_Config(SystemCoreClock);//Configure system timer

	/* @Name:	sysclock_64M
	 *
	 * @Brief:	this function changes system clock to 64 MegaHerz.
	 */
}
void delay(volatile uint32_t s) {							//delay function
    for(; s>0; s--);
    /* @Name:	delay
	 *
	 * @Brief:	this function causes desired amount of delay. (in micro seconds fo 16 Mhz)
	 */
}
void init_timer1(){											//initialization of the TIM1
	RCC->APBENR2 |= (1U << 11); //enable clock for timer1

	TIM1->CR1	 = 0;
	TIM1->CR1	|= (1<<7);		//enable auto reload
	TIM1->CR1	|= (0<<4);		//direction = up-counter

	TIM1->CNT	 = 0;			//Set default TIM1 values
	TIM1->ARR	 = 2000;		//for 0.2 second interval
	TIM1->PSC	 = 1600*4;		//With AutoReloadRegister and PreSCaler.

	TIM1->DIER	|= (1<<0);		//enable update interrupt
	TIM1->CR1	|= (1<<0);		//enable TIM1

	NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn,2);	//Set priority to 2
	NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);		//Enable NVIC for TIM1

	/* @Name:	init_timer1
	 *
	 * @Brief:	this function configures and enables timer1 with priority of 2.
	 */
}
void init_timer2(){											//initialization of the TIM2 for PWM
	RCC->APBENR1 |= (1U << 0); 		//enable clock for timer2
	RCC->IOPENR  |= (1U << 1);		//enable GPIOB for PWM out at PB3


	TIM2->CR1	 = 0;				//reset control register1
	TIM2->CCMR1	 = 0;				//reset capture/compare register

	TIM2->CR1	|= (1U<<4);			//direction = down-counter

	TIM2->CNT	 = 0;				//Set default TIM2 values
	TIM2->ARR	 = 200;				//for 8-bit setup = 160kHz
	TIM2->PSC	 = 1;				//With AutoReloadRegister and PreSCaler.

	//PWM configuration for TIM2_CH2
	TIM2->CCMR1 &= ~(0x7U << 12);			//reset oc2m
    TIM2->CCMR1 |= (0x6U << 12);			//oc2m; pwm mode 1 for channel2

    TIM2->CR1	|= (1U<<7);					//enable auto reload
    TIM2->EGR 	|= (1<<0);      			//update generation

    TIM2->CCMR1 |= (1U << 11);				//enable output compare2 preload
    TIM2->CCER 	|= (1U << 4);        		//enable capture/compare2 output

    GPIOB->MODER &= ~(3U << 2*3);      		//pb3
    GPIOB->MODER |= (2U << 2*3);        	//alternate mode
    GPIOB->OSPEEDR |= (3U << 2*3);        	//high precision on change
    GPIOB->AFR[0] &= ~(0xFU << 4*3);   		//af4 for = TIM2_CH2
    GPIOB->AFR[0] |= (2U << 4*3);        	// AFSEL5 -> AF2 [0010]

	TIM2->DIER	|= (1<<0);		//enable update interrupt
	TIM2->CR1	|= (1<<0);		//enable TIM2

	NVIC_SetPriority(TIM2_IRQn,2);	//Set priority to 2
	NVIC_EnableIRQ(TIM2_IRQn);		//Enable NVIC for TIM1
	/* @Name:	init_timer2
	 *
	 * @Brief:	this function configures and enables timer with PWM output
	 * 			at pin PB3.
	 */
}
void init_timer3(){											//initialization of the TIM3 for ADC
	RCC->APBENR1 |= (1U << 1); //enable clock for timer3

	TIM3->CR1	 = 0;
	TIM3->CR1	|= (1<<7);		//enable auto reload
	TIM3->CR1	|= (1<<4);		//direction = down-counter

	TIM3->CNT	 = 0;		//Set default TIM3 values
	TIM3->ARR	 = 50;		//for 6400 Hz
	TIM3->PSC	 = 200;		//With AutoReloadRegister and PreSCaler.

	TIM3->CR2 |= (2U << 4);		//MMS; Update

	TIM3->CR1	|= (1<<0);		//enable TIM3
	/* @Name:	init_timer3
	 *
	 * @Brief:	this function configures and enables timer3 for ADC with 6400 Hz.
	 */
}
void init_timer16(){											//initialization of the TIM3 for ADC
	RCC->APBENR2 |= (1U << 17); //enable clock for timer16

	TIM16->CR1	 = 0;
	TIM16->CR1	|= (1<<7);		//enable auto reload

	TIM16->CNT	 = 0;		//Set default TIM3 values
	TIM16->ARR	 = 1000;		//for 25600 Hz
	TIM16->PSC	 = 1;		//With AutoReloadRegister and PreSCaler.

	TIM16->DIER	|= (1<<0);		//enable update interrupt
	TIM16->CR1	|= (1<<0);		//enable TIM3

	NVIC_SetPriority(TIM16_IRQn,2);	//Set priority to 2
	NVIC_EnableIRQ(TIM16_IRQn);		//Enable NVIC for TIM1
	/* @Name:	init_timer3
	 *
	 * @Brief:	this function configures and enables timer.
	 */
}
void init_i2c(){											//initialization of the I2C1

	//Enable GPIOB
	RCC->IOPENR |= (1U << 1);

	//Setup PB6 as AF6
	GPIOB->MODER &= ~(3U << 2*6);
	GPIOB->MODER |= (2 << 2*6);
	GPIOB->OTYPER |= (1U << 6);
	GPIOB->PUPDR |= (1<< 2*6);  //Pull up for PB6
	//Chose AF6 from mux
	GPIOB->AFR[0] &= ~(0xFU << 4*6);
	GPIOB->AFR[0] |= (6 << 4*6);
	//Setup PB7 as AF6
	GPIOB->MODER &= ~(3U << 2*7);
	GPIOB->MODER |= (2U << 2*7);
	GPIOB->OTYPER |= (1U << 7);
	GPIOB->PUPDR |= (1<< 2*7);  //Pull up for PB7
	//Chose AF6 from mux
	GPIOB->AFR[0] &= ~(0xFU << 4*7);
	GPIOB->AFR[0] |= (6 << 4*7);
	//Enable i2c1
	RCC->APBENR1 |= (1U << 21);

	I2C1->CR1 =0;
	I2C1->CR1 |= (1U << 7); //ERRI

	I2C1->TIMINGR |= (3 << 28);		//PRESCALAR
	I2C1->TIMINGR |= (0x13 << 0);	//SCLL
	I2C1->TIMINGR |= (0xF << 8);	//SCLH
	I2C1->TIMINGR |= (0x2 << 16);	//SDADEL
	I2C1->TIMINGR |= (0x4) << 20;	//SCLDEL

	I2C1->CR1 |= (1U << 0);			//PE

	NVIC_SetPriority(I2C1_IRQn,2);	//Set priority to 2
	NVIC_EnableIRQ(I2C1_IRQn);		//Enable NVIC for TIM1

	/* @Name:	init_i2c
	 *
	 * @Brief:	this function configures and enables i2c bus for
	 * PB6 (for SCL) and PB7 (for SDA) pins with priority of 2.
	 */
}
void init_adc(){

	RCC->APBENR2 |= (1U << 20); 	//enable rcc for adc
	RCC->IOPENR |= (1U << 1);		//enable GPIOB
	//PB1 pin for adc in analog mode (by default)

	ADC1->CR	 = 0;				//reset adc cr
	ADC1->CFGR1  = 0;				//reset adc cfgr1

	ADC1->CR |= (1U << 28);			//Enable adc voltage regulator
	delay(500);						//delay >20 uS

	//enable calibration, wait until completion
	ADC1->CR |= (1U << 31);			//calibration enable
	while(!(ADC1->ISR & (1 << 11)));//Wait until EOCAL=1.

	//enable end of cal. or sequence interrupts
	ADC1->IER |= (1U << 2); //end of conversion sequence interrupt

	//select resolution [conf. bit sample (6,8,10,12)]
	ADC1->CFGR1 |= (2U << 3);// ; 8bit

	//conf. single/continuous;
	ADC1->CFGR1 &= ~(1U << 13);//cont=0;
	ADC1->CFGR1 &= ~(1U << 16);//discen =0; single

	//select sampling time from SMPR
	ADC1->SMPR |= (0 << 0);//SMP1
	ADC1->SMPR |= (1U << 4);//SMP2

	//select tim3 trgo
	ADC1->CFGR1 |= (3U << 6); //TGRO (extsel); 0xb011=3U for TIM3_TRGO
	ADC1->CFGR1 |= (1U << 10); //Choose detect at rising edge (exten); 01

	//enable channels (for the ANx pins)
	ADC1->CFGR1 |= (9U << 26);//analog input channel 9; PB1
	ADC1->CHSELR |= (1U << 9);//analog input channel 9; PB1

	//Clear the ADRDY bit in ADC_ISR register by programming this bit to 1.
	ADC1->ISR |= (1U << 0);

	//enable adc and wait until it is ready
	ADC1->CR |= (1U << 0);
	while(!(ADC1->ISR & (1U << 0)));

	NVIC_SetPriority(ADC1_IRQn,2);	//Set priority to 2
	NVIC_EnableIRQ(ADC1_IRQn);		//Enable NVIC for TIM1

	//Start conversion
	ADC1->CR |= (1U << 2);

	/* @Name:	init_adc
	 *
	 * @Brief:	this function configures and enables ADC1 peripheral with tim3
	 * trigger for and takes input from PB1 pin and works with interrupt with priority of 2.
	 */
}
void write_I2C(uint8_t devAddr,uint8_t* data, int Size){	//write function for i2c1 (no address)
	//Send address and register to read
	I2C1->CR2 =0;									//clear register
	I2C1->CR2 |= (uint32_t)(devAddr << 1);			//Send slave address
	I2C1->CR2 |= (uint32_t)(Size << 16); 			//Number of bytes
	I2C1->CR2 |= (1U << 25); 						//AUTOEND
	I2C1->CR2 |= (1U << 13); 						//Generate start

	while(Size){									//while size loop
		while( !(I2C1->ISR & (1 << 1)));			//is flag busy?
		I2C1->TXDR = (*data++);						//sen data, data++
		Size--;										//size = size-1;
	}
	/* @Name:	write_I2C
	 *
	 * @Brief:	This function writes data to i2c devices without address.
	 * it writes data faster than other functions. ('with address' functions)
	 * it writes data wherever address pointer points at.
	 */
}
void read_I2C(uint8_t devAddr,uint8_t* data,int Size){		//read function for i2c1 (no address)
	//read Data
	I2C1->CR2 =0;
	I2C1->CR2 |= (uint32_t)(devAddr << 1);
	I2C1->CR2 |= (1U << 10); 						//Read mode
	I2C1->CR2 |= (uint32_t)(Size << 16); 			//Number of bytes
	I2C1->CR2 |= (1U << 25); 						//AUTOEND
	I2C1->CR2 |= (1U << 13); 						//Generate start

	while(Size){
		while( !(I2C1->ISR & (1 << 2)));
		(*data++) = (uint8_t)I2C1->RXDR;
		Size--;
	}
	/* @Name:	read_I2C
	 *
	 * @Brief:	This function reads from i2c devices without address.
	 * it reads data faster than other functions. ('with address' functions)
	 * it reads data wherever address pointer points at.
	 */
}
void write_memory_I2C(uint8_t devAddr,uint16_t memAddr,uint8_t* data, int Size){
	//Send address and register to read
	I2C1->CR2 =0;
	I2C1->CR2 |= (uint32_t)(devAddr << 1);
	I2C1->CR2 |= (uint32_t)(( Size + 2)<< 16); 		//Number of bytes
	I2C1->CR2 |= (1U << 25); 						//AUTOEND
	I2C1->CR2 |= (1U << 13); 						//Generate start

	while(!(I2C1->ISR & (1 << 1)));					//high address
	I2C1->TXDR = (uint32_t)(memAddr >> 8);			//(uint32_t)

	while(!(I2C1->ISR & (1 << 1)));					//low address
	I2C1->TXDR = (uint32_t)(memAddr & 0xFF);		//(uint32_t)

	while(Size){									//while size loop
		while( !(I2C1->ISR & (1 << 1)));			//is flag busy?
		I2C1->TXDR = (*data++);						//sen data, data++
		Size--;										//size = size-1;
	}
	/* @Name:	write_memory_I2C
	 *
	 * @Brief:	This function writes data to i2c devices with 16bit address.
	 * it needs amount of delay for device spesipic ( for EEPROM; 5ms delay)
	 * it divides address into 2 pieces; HIGH ADDr, LOW ADDr
	 */
}
void random_read_I2C(uint8_t devAddr,uint16_t memAddr,uint8_t* data,int Size){
	//Send address and register to read
	I2C1->CR2 =0;
	I2C1->CR2 |= (uint32_t)(devAddr << 1);
	I2C1->CR2 |= (2U << 16); 						//Number of bytes
	I2C1->CR2 |= (1U << 13); 						//Generate start

	while(!(I2C1->ISR & (1 << 1)));
	I2C1->TXDR = (uint32_t)(memAddr >> 8);			//(uint32_t)

	while(!(I2C1->ISR & (1 << 1)));
	I2C1->TXDR = (uint32_t)(memAddr & 0xFF);		//(uint32_t)

	while(!(I2C1->ISR & (1 << 6)));					//transmission complete

	//read Data
	I2C1->CR2 =0;
	I2C1->CR2 |= (uint32_t)(devAddr << 1);
	I2C1->CR2 |= (1U << 10); 						//Read mode
	I2C1->CR2 |= (uint32_t)(Size << 16); 			//Number of bytes
	I2C1->CR2 |= (1U << 25); 						//AUTOEND
	I2C1->CR2 |= (1U << 13); 						//Generate start


	while(Size){
		while( !(I2C1->ISR & (1 << 2)));
		(*data++) = (uint8_t)I2C1->RXDR;
		Size--;
	}
	/* @Name:	random_read_I2C
	 *
	 * @Brief:	This function reads from i2c devices with 16bit address.
	 * it doesn't need delays.
	 * it divides address into 2 pieces; HIGH ADDr, LOW ADDr
	 */
}
void init_keypad(){					//initialization of the Keypad

	/*Keypad Configuration
	* inputs:	pins; PORTA_12/11/6/7	roles; COL_1/2/3/4
	* outputs:	pins; PORTA_5/4/1/0		roles; ROW_1/2/3/4
	*/

    //PORTA
    GPIOA->MODER &= ~(0x3C0FF0FU);	//allocate pins 0 to 12
    //outputs
    GPIOA->MODER |= (1U << 2*5);	//row1
    GPIOA->MODER |= (1U << 2*4);	//row2
    GPIOA->MODER |= (1U << 2*1);	//row3
    GPIOA->MODER |= (1U << 2*0);	//row4
    //inputs
/*	since we allocate pins above, that sends 00' to the
 *	related bit field. 00' also means input mode so we
 *	don't have to do GPIOA->MODER |= (0U << 2*12);	//col1
 *	for inputs. OR operation with 0 is useless anyway.
 *
 *	::: So that means our inputs are ready.
 */

	/* @Name:	init_keypad
	 *
	 * @Brief:	this function activates the pins and set them as outputs or inputs for the keypad module.
	 */

}
uint8_t keypad(){					//keypad button configurations

	//Set 1 row active(logic 0) and rest deactive(logic 1)
	//and check inputs; if there is button pressed (logic 0)
	//and return a value if pressed.

	//***********************************************ROW 1******************************//
	GPIOA->ODR &= (0U << 5);	//	row1
	GPIOA->ODR |= (1U << 4);	//	row2
	GPIOA->ODR |= (1U << 1);	//	row3
	GPIOA->ODR |= (1U << 0);	//	row4

	delay(10);
	if((~(GPIOA->IDR)) & (1U << 12)){
		return '1';
	}
	if((~(GPIOA->IDR)) & (1U << 11)){
		return '2';
	}
	if((~(GPIOA->IDR)) & (1U << 6)){
		return '3';
	}
	if((~(GPIOA->IDR)) & (1U << 7)){
		return 'A';
	}
	//***********************************************ROW 2******************************//
	GPIOA->ODR &= (0U << 4);	//	row2
	GPIOA->ODR |= (1U << 5);	//	row1
	GPIOA->ODR |= (1U << 1);	//	row3
	GPIOA->ODR |= (1U << 0);	//	row4

	delay(10);
	if((~(GPIOA->IDR)) & (1U << 12)){
		return '4';
	}
	if((~(GPIOA->IDR)) & (1U << 11)){
		return '5';
	}
	if((~(GPIOA->IDR)) & (1U << 6)){
		return '6';
	}
	if((~(GPIOA->IDR)) & (1U << 7)){
		return 'B';
	}
	//***********************************************ROW 3******************************//
	GPIOA->ODR &= (0U << 1);	//	row3
	GPIOA->ODR |= (1U << 5);	//	row1
	GPIOA->ODR |= (1U << 4);	//	row2
	GPIOA->ODR |= (1U << 0);	//	row4

	delay(10);
	if((~(GPIOA->IDR)) & (1U << 12)){
		return '7';
	}
	if((~(GPIOA->IDR)) & (1U << 11)){
		return '8';
	}
	if((~(GPIOA->IDR)) & (1U << 6)){
		return '9';
	}
	if((~(GPIOA->IDR)) & (1U << 7)){
		return 'C';
	}
	//***********************************************ROW 4******************************//
	GPIOA->ODR &= (0U << 0);	//	row4
	GPIOA->ODR |= (1U << 5);	//	row1
	GPIOA->ODR |= (1U << 4);	//	row2
	GPIOA->ODR |= (1U << 1);	//	row3


	delay(10);
	if((~(GPIOA->IDR)) & (1U << 12)){
		return 'E';
	}
	if((~(GPIOA->IDR)) & (1U << 11)){
		return '0';
	}
	if((~(GPIOA->IDR)) & (1U << 6)){
		return 'F';
	}
	if((~(GPIOA->IDR)) & (1U << 7)){
		return 'D';
	}
	//***********************************************ELSE******************************//
	else
		return 'Q'; //return if no button pressed.


		/* @Name:	keypad
		 *
		 * @Brief:	this function gets a value when we press the button on keypad,
		 * and returnes that value to itself. And we can catch the key press like;
		 * 		a = keypad()  => a' captures the key value...
		 */
}
void capture_key(int keycapture){	//capture (switched) key

	if(keycapture == 'Q'){								//default

	}
	else if(keycapture == '1'){							//track_select = 1
		track_select =1;
		second=0;
		clock_counter=0;
	}
	else if(keycapture == '2'){							//track_select = 2
		track_select =2;
		second=0;
		clock_counter=0;
	}
	else if(keycapture == '3'){							//track_select = 3
		track_select =3;
		second=0;
		clock_counter=0;
	}
	else if(keycapture == 'A'){							//track_select = 4
		track_select =4;
		second=0;
		clock_counter=0;
	}
	else if(keycapture == 'B'){							//monitor feedback on/off
		monitor ^= 1;
	}

	else if(keycapture == 'D'){							//Reset Key
		//Reset the timeout value if  key pressed
		second=0;
		clock_counter=0;
		play=0;
		record=0;
		regAddr1_tracker=0x0;
		regAddr2_tracker=0x8000;
		i2c_rec_play=0;
	}
	else if(keycapture == '9'){							// -menu key
		//Reset the timeout value if  key pressed
		second=0;
		clock_counter=0;
		state--;
		idle=0;
	}
	else if(keycapture == 'C'){							// +menu key
		//Reset the timeout value if  key pressed
		second=0;
		clock_counter=0;
		state++;
		idle=0;
	}
	else if(keycapture == 'E'){							//Record Key
		//Reset the timeout value if  key pressed
		second=0;
		clock_counter=0;
		record=1;
		play=0;
		i2c_rec_play=1;	//clear values go to the record section
	}
	else if(keycapture == '0'){							//Playback Key
		//Reset the timeout value if  key pressed
		second=0;
		clock_counter=0;
		play=1;
		record=0;
		i2c_rec_play=2;	//clear values go to the playback section
	}
	else if(keycapture == 'F'){							//delete Key
		//Reset the timeout value if  key pressed
		second=0;
		clock_counter=0;
		play=0;
		record=0;
		i2c_rec_play=3;	//clear values go to the delete section
	}
	/* @Name:	capture_key
	 * @Brief:	The mechanism is basicly tells what keys to use and what to do if pressed
	 */
}
void idle_state(){

	lcd_setCursor(1,1);
	lcd_send_string("   IDLE State   ");
	lcd_setCursor(2,1);
	lcd_send_string("Waiting Menu Key");
	//print IdLE and wait for menu select key

	 /* @Name:	idle_state
	 *
	 * @Brief:	this function is stand-by phase of the voice recorder.
	 * It prints idle state and waits for key.
	 */
}
void status_state(){
	lcd_setCursor(1,1);
	lcd_send_string("  STATUS State  ");
	lcd_setCursor(2,1);
	lcd_PrintInt(isEEPfull);
	lcd_setCursor(2,2);
	lcd_send_string(" ");
	lcd_setCursor(2,3);
	lcd_send_string("Rec Avaliable.");

	//show rcd and wait for track select or status

	 /* @Name:	status_state
	 *
	 * @Brief:	this function is Status state of the voice recorder.
	 * It prints status state and shows how many tracks is recorded.
	 */
}
void full_state(){
	lcd_setCursor(1,1);
	lcd_send_string("   FULL State   ");
	lcd_setCursor(2,1);
	lcd_send_string("EEPROMs are FULL");
	delay(6000000);
	 /* @Name:	full_state
	 *
	 * @Brief:	this function is FULL state of the voice recorder.
	 * It prints FULL and go back to idle after 2 seconds.
	 */
}
void record_state(){
	lcd_setCursor(1,1);
	lcd_send_string("  RECORD State  ");
	lcd_setCursor(2,1);
	lcd_send_string("Track?: ");
	lcd_setCursor(2,9);
	lcd_PrintInt(track_select);
	lcd_setCursor(2,10);
	lcd_send_string("       ");



	 /* @Name:	record_state
	 *
	 * @Brief:	this function is RECORD state of the voice recorder.
	 * It prints record and selected track
	 * and go to the recording if record button is pressed.
	 *
	 * after record is done, go back to idle state.
	 */
}
void playback_state(){
	lcd_setCursor(1,1);
	lcd_send_string(" PLAYBACK State ");
	lcd_setCursor(2,1);
	lcd_send_string("Track?: ");
	lcd_setCursor(2,9);
	lcd_PrintInt(track_select);
	lcd_setCursor(2,10);
	lcd_send_string("       ");


	 /* @Name:	playback_state
	 *
	 * @Brief:	this function is PLAYBACK state of the voice recorder.
	 * It prints playback and selected track
	 * and go to the playback if play button is pressed.
	 *
	 * after playback is done, go back to idle state.
	 */
}
void delete_state(){
	lcd_setCursor(1,1);
	lcd_send_string("  DELETE State  ");
	lcd_setCursor(2,1);
	lcd_send_string("Track?: ");
	lcd_setCursor(2,9);
	lcd_PrintInt(track_select);
	lcd_setCursor(2,10);
	lcd_send_string("       ");


	 /* @Name:	delete_state
	  *
	 * @Brief:	this function is DELETE state of the voice recorder.
	 * It prints delete and selected track
	 * and go to the delete if play button is pressed.
	 *
	 * after delete is done, go back to idle state.
	 */
}
void voice_recorder(){					//Voice recorder main function


	//*******************STATEs***************************//
		if(idle == 1){						//idle_state
			if((isEEPfull == 4) && (pass == 0)){
				full_state();
				pass=1;
			}
			else{
				idle_state();
			}
		}
		else if(state == 1){				//record_state
			record_state();
		}
		else if(state == 2){				//playback_state
			playback_state();
		}
		else if(state == 3){				//status_state
			status_state();
		}
		else if(state == 4){				//status_state
			delete_state();
		}
		else if(state > 4){					//reset/clear etc. state
			state=1;
		}
		else if(state < 1){					//reset/clear etc. state
			state=4;
		}

	 /* @Name:	voice_recorder
	 *
	 * @Brief:	this function is voice_recorder. It operates as state machine and
	 * runs whatever state is selected.
	 *
	 */
}
//*******************************Functions * END*******************************//

//*****************************Interrupts and main****************************//
void TIM1_BRK_UP_TRG_COM_IRQHandler(void){		//timer1 interrupt handler
	pressedKey= keypad();						//get pressed key
	capture_key(pressedKey);					//send pressed key
	clock_counter++;							//count 0.2 seconds to get 1 second
    isEEPfull=recorded[0]+recorded[1]+recorded[2]+recorded[3];	//update recored tracks

	if((record == 1) || (play == 1)){			//dont go to idle or etc
		clock_counter=0;						//when its recording or
		second=0;								//when its playing
	}
	if(clock_counter == 5){						//for 1sec timeout
		clock_counter=0;
		second++;
		GPIOC->ODR ^= (1U << 6);				//blink test led (PC6 onboard led)

		if(second==10){							//if 10sec has passed;
			second=0;							//reset second counter
			idle=1;								//go to the idle state
		}
	}
	TIM1->SR	&= ~(1U <<0); 					//clear update status
}
void TIM2_IRQHandler(void){						//timer2 interrupt handler
	if(play==1){										//if play has pressed
		if(data_ready == 1){							//and data is ready
			//send the values from eeprom to speaker
			TIM2->CCR2=(uint32_t)(pwmvalue_load-120)/2;	//send value buffer to pwm (to speaker)
		}
		data_ready =0;									//clear data ready flag
	}
	else if(monitor == 1){								//if monitor on
		TIM2->CCR2=(uint32_t)(inputCaptureVal-120)/2;	//feed ADC values directly to the speaker
	}
	TIM2->SR	&= ~(1U <<0); 							//clear update status
}
void TIM3_IRQHandler(void){						//timer3 interrupt handler
	TIM3->SR	&= ~(1U <<0); 			//clear update status
	//closed
}
void I2C1_IRQHandler(void){						//i2c handler
	//only come here if error occurs..
	I2C1->ISR |= (1U << 5);
}
void ADC_COMP_IRQHandler(void){					//adc1 interrupt handler
	if(record ==1){										//if record is pressed
		if(data_ready == 0){							//and data is ready

			for(int i=0;i<32;i++){
			inputCaptureVal=(uint8_t)(ADC1->DR);		//capture mic value
			pwmvalue_save[i]=(inputCaptureVal)%256;		//send captured value to save buffer
			inputCaptureVal=0;							//clear captured
			}
			data_ready=1;								//set data ready flag
		}
	}
	else if(monitor == 1){								//if monitor is 1
		inputCaptureVal=(uint8_t)(ADC1->DR);			//send ADC values directly to PWM out
	}
	ADC1->ISR &= (1U <<2); 								//clear interrupt
}
int main(void) {								//main
	//Enable clock for PORT A and PORT B
	RCC->IOPENR |= 0x3;
	RCC->IOPENR |= (1U << 2); //port c for test
	//**** Function inits ****//
	sysclock_64M();

	init_timer1();
	init_timer2();
	init_timer3();

	init_i2c();
	init_adc();

	lcd_init();
	init_keypad();
	//**** conf. PC6 as test led ****//
    GPIOC->MODER &= ~(3U << 2*6);
    GPIOC->MODER |= (1U << 2*6);

    delay(1000000*4);
    GPIOC->BRR |= (1U << 6);					//clear test led

    //**startup**//
    lcd_clear_all();							//clear lcd
    delay(100000*2);							//wait 2sec

	lcd_setCursor(1,1);							//start state
	lcd_send_string("   Safa BULAT   ");		//
	lcd_setCursor(2,1);							//show my id and
	lcd_send_string("    141024051   ");		//my number
    delay(2000000*4);							//delay 2sec
    											//dont occur again

    lcd_clear_all();							//clear lcd after start state
    idle=1;										//go to the idle state
    //**end startup**//
    while(1) {
//************************************************VOI_REC********************************************//
		if(i2c_rec_play == 0){					//activate voice recorder
			voice_recorder();					//print states; record etc.
		}
//************************************************RECORD********************************************//
		else if(i2c_rec_play==1){								//if record start
			if(track_select == 1){								//go to selected track
		    	if(record == 1){								//go to recording
		    		play=0;										//can't play when recording
		    		if(data_ready == 1){						//is data ready?
		    			//save datas to the eeprom with 32byte packages
						write_memory_I2C(eeprom_1_address, (uint16_t)regAddr1_tracker, (uint8_t *)&pwmvalue_save, 32);
						delay(20000);
						regAddr1_tracker+=32;					//address++
		    		}
		    		if((regAddr1_tracker >= 0x7D00)  || (record==0)){
		    			//if rec=0 button is pressed, or 5 second has passed
		    			//(if address is pointing half of the EEPROM (0x7D000)
		    			record=0;								//end recording
		    			regAddr1_tracker=0x0;					//reset address
		    			recorded[0]=1;							//record status++ for selected track

						lcd_setCursor(2, 11);					//put cursor
						lcd_send_string("rec.ed");				//print rec.ed for recorded
						delay(4000000*2); 						//delay for a second
						i2c_rec_play=0;							//go to the main function
						idle=1;									//go back to idle state
		    		}
		    		data_ready=0;								//clear data ready flag
		    	}
			}													//rest is same with other tracks
			else if(track_select == 2){
		    	if(record == 1){
		    		play=0;
		    		if(data_ready == 1){
						write_memory_I2C(eeprom_1_address, (uint16_t)regAddr2_tracker, (uint8_t *)&pwmvalue_save, 32);
						delay(20000);
						regAddr2_tracker+=32;
		    		}
		    		if((regAddr2_tracker >= 0xFCFF)  || (record==0)){
		    			record=0;
		    			regAddr2_tracker=0x8000;
		    			recorded[1]=1;

						lcd_setCursor(2, 11);					//put cursor
						lcd_send_string("rec.ed");				//print rec.ed for recorded
						delay(4000000*2); 						//delay for a second
						i2c_rec_play=0;							//go to the main function
						idle=1;									//go back to idle state
		    		}
		    		data_ready=0;
		    	}

			}
			else if(track_select == 3){
				if(record == 1){
					play=0;
					if(data_ready == 1){
						write_memory_I2C(eeprom_2_address, (uint16_t)regAddr1_tracker, (uint8_t *)&pwmvalue_save, 32);
						delay(20000);
						regAddr1_tracker+=32;
					}
				    if((regAddr1_tracker >= 0x7D00 ) || (record==0)){
				    	record=0;
				    	regAddr1_tracker=0x0;
				    	recorded[2]=1;

						lcd_setCursor(2, 11);					//put cursor
						lcd_send_string("rec.ed");				//print rec.ed for recorded
						delay(4000000*2); 						//delay for a second
						i2c_rec_play=0;							//go to the main function
						idle=1;									//go back to idle state
				    }
				    data_ready=0;
				}

			}
			else if(track_select == 4){
		    	if(record == 1){
		    		play=0;
		    		if(data_ready == 1){
						write_memory_I2C(eeprom_2_address, (uint16_t)regAddr2_tracker, (uint8_t *)&pwmvalue_save, 32);
						delay(20000);
						regAddr2_tracker+=32;
		    		}
		    		if((regAddr2_tracker >= 0xFCFF ) || (record==0)){
		    			record=0;
		    			regAddr2_tracker=0x8000;
		    			recorded[3]=1;

						lcd_setCursor(2, 11);					//put cursor
						lcd_send_string("rec.ed");				//print rec.ed for recorded
						delay(4000000*2); 						//delay for a second
						i2c_rec_play=0;							//go to the main function
						idle=1;	;								//go to the idle state
		    		}
		    		data_ready=0;
		    	}

			}
		}
//************************************************PLAYBACK*******************************************//
		else if(i2c_rec_play==2){										//if playback is start

			if(track_select == 1){										//go to the selected track
		    	if(play == 1){											//play?
		    		record=0;											//cant record when playing
		    		if(data_ready == 0){								//is data ready?
		    			//go to the address and read datas, and send them to the PWM.
						random_read_I2C(eeprom_1_address, (uint16_t)regAddr1_tracker, (uint8_t *)&track[0], 1);
						pwmvalue_load=track[0];					//buffer to pwmval_load
						regAddr1_tracker++;						//update address (increase)
		    		}
		    		if((regAddr1_tracker >= 0x7FFF ) || (play==0)){
		    													//if recording is end
		    			play=0;									//play=0
		    			regAddr1_tracker=0x0;					//reset address tracker

						lcd_setCursor(2, 11);					//put cursor
						lcd_send_string("played");				//print played
						delay(4000000*2); 						//delay for a second
						i2c_rec_play=0;							//go to the main function
						idle=1;									//go to the idle state
		    		}
		    		data_ready=1;								//set data flag
		    	}
			}													//the rest is same for other tracks
			else if(track_select == 2){
		    	if(play == 1){
		    		record=0;
		    		if(data_ready == 0){
						random_read_I2C(eeprom_1_address, (uint16_t)regAddr2_tracker, (uint8_t *)&track[0], 1);
						pwmvalue_load=track[0];
						regAddr2_tracker++;
		    		}
		    		if((regAddr2_tracker >= 0xFFFF ) || (play==0)){
		    			play=0;
		    			regAddr2_tracker=0x8000;

						lcd_setCursor(2, 11);					//put cursor
						lcd_send_string("played");				//print played
						delay(4000000*2); 						//delay for a second
						i2c_rec_play=0;							//go to the main function
						idle=1;									//go to the idle state
		    		}
		    		data_ready=1;
		    	}

			}
			else if(track_select == 3){
				if(play == 1){
					record=0;
					if(data_ready == 0){
						random_read_I2C(eeprom_2_address, (uint16_t)regAddr1_tracker, (uint8_t *)&track[0], 1);
						pwmvalue_load=track[0];
						regAddr1_tracker++;
					}
				    if((regAddr1_tracker >= 0x7FFF ) || (play==0)){
		    			play=0;
		    			regAddr1_tracker=0x0;

						lcd_setCursor(2, 11);					//put cursor
						lcd_send_string("played");				//print played
						delay(4000000*2); 						//delay for a second
						i2c_rec_play=0;							//go to the main function
						idle=1;									//go to the idle state
				    }
				    data_ready=1;
				}

			}
			else if(track_select == 4){
		    	if(play == 1){
		    		record=0;
		    		if(data_ready == 0){
						random_read_I2C(eeprom_2_address, (uint16_t)regAddr2_tracker, (uint8_t *)&track[0], 1);
						pwmvalue_load=track[0];
						regAddr2_tracker++;
		    		}
		    		if((regAddr2_tracker >= 0xFFFF ) || (play==0)){
		    			play=0;
		    			regAddr2_tracker=0x8000;

						lcd_setCursor(2, 11);					//put cursor
						lcd_send_string("played");				//print played
						delay(4000000*2); 						//delay for a second
						i2c_rec_play=0;							//go to the main function
						idle=1;									//go to the idle state
		    		}
		    		data_ready=1;
		    	}

			}
		}
//************************************************DELETE*******************************************//
		else if(i2c_rec_play==3){											//if delete is start
				if(track_select == 1){										//track select
					play=0;													//cant play
					record=0;												//or record
					clock_counter=0;										//or go to the idle
					second=0;												//when deleting
					//delete data with sending 0x0
					write_memory_I2C(eeprom_1_address, (uint16_t)regAddr1_tracker, (uint8_t *)0x0, 32);
					delay(30000);								//writing delay 5ms
					regAddr1_tracker+=32;						//memory address++

					if(regAddr1_tracker >= 0x7D00){
						regAddr1_tracker=0x0;					//reset memory address
						recorded[0]=0;							//record = 0; deleted
						lcd_setCursor(2, 1);					//put cursor
						lcd_send_string("track 1 deleted.");	//print deleted
						delay(4000000); 						//delay for a second
						i2c_rec_play=0;							//go to the main function
						idle=1;									//go back to idle state
					}
				}												//the rest is same with other tracks
				else if(track_select == 2){
					play=0;
					record=0;
					clock_counter=0;
					second=0;
					//delete data with sending 0x0
					write_memory_I2C(eeprom_1_address, (uint16_t)regAddr2_tracker, (uint8_t *)0x0, 32);
					delay(20000);								//writing delay 5ms
					regAddr2_tracker+=32;						//memory address++

					if(regAddr2_tracker >= 0xFCFF){
						regAddr2_tracker=0x8000;				//reset memory address
						recorded[1]=0;							//record = 0; deleted
						lcd_setCursor(2, 1);					//put cursor
						lcd_send_string("track 2 deleted.");	//print deleted
						delay(4000000); 						//delay for a second
						i2c_rec_play=0;							//go to the main function
						idle=1;									//go back to idle state
					}
				}
				else if(track_select == 3){
					play=0;
					record=0;
					clock_counter=0;
					second=0;
					//delete data with sending 0x0
					write_memory_I2C(eeprom_2_address, (uint16_t)regAddr1_tracker, (uint8_t *)0x0, 32);
					delay(20000);								//writing delay 5ms
					regAddr1_tracker+=32;						//memory address++

					if((regAddr1_tracker >= 0x7D00) ){
						regAddr1_tracker=0x0;					//reset memory address
						recorded[2]=0;							//record = 0; deleted
						lcd_setCursor(2, 1);					//put cursor
						lcd_send_string("track 2 deleted.");	//print deleted
						delay(4000000); 						//delay for a second
						i2c_rec_play=0;							//go to the main function
						idle=1;									//go back to idle state
					}
				}
				else if(track_select == 4){
					play=0;
					record=0;
					clock_counter=0;
					second=0;
					//delete data with sending 0x0
					write_memory_I2C(eeprom_2_address, (uint16_t)regAddr2_tracker, (uint8_t *)0x0, 32);
					delay(20000);								//writing delay 5ms
					regAddr2_tracker+=32;						//memory address++

					if(regAddr2_tracker >= 0xFCFF){
						regAddr2_tracker=0x8000;				//reset memory address
						recorded[3]=0;							//record = 0; deleted
						lcd_setCursor(2, 1);					//put cursor
						lcd_send_string("track 2 deleted.");	//print deleted
						delay(4000000); 						//delay for a second
						i2c_rec_play=0;							//go to the main function
						idle=1;									//go back to idle state
					}
				}
			}
    }
    return 0;
}
//*************************Interrupts and main * END*************************//


//**********************PIN CONNECTIONs of the BOARD************************//

/*
 * Keypad;.....................................................................
 *  					(Don't forget the de-bounce buttons.)
 *
 *  Col1				PA7
 *  Col2				PA6
 *  Col3				PA11
 *  Col4				PA12
 *
 *  Row1				PA5
 *  Row2				PA4
 *  Row3				PA1
 *  Row4				PA0
 * ............................................................................
 *
 * I2C;........................................................................
 *
 * SCL: PB6
 * SDA: PB7
 *
 * On I2C bus i have 3 devices; EEPROM1, EEPROM2, 1602 LCD (i2c)
 * ............................................................................
 *
 * ADC; Mic. is connected to ADC at PB1........................................
 * ............................................................................
 *
 * PWM; PWM out is PB3 (connected to speaker)..................................
 * ............................................................................
 *
 * Test LED(s);................................................................
 * Onboard LED;			PC6
 * ............................................................................
 */
