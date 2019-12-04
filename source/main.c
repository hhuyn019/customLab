/*
 * CustomLab.c
 *
 * Created: 11/23/2019 11:06:21 PM
 * Author : Chris
 */ 

#include <avr/io.h>
#ifdef _SIMULATE_
#include "simAVRHeader.h"
#endif
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>
#define input (~PINA & 0xFF)

#define SET_BIT(p,i) ((p) |= (1 << (i)))
#define CLR_BIT(p,i) ((p) &= ~(1 << (i)))
#define GET_BIT(p,i) ((p) & (1 << (i)))

/*-------------------------------------------------------------------------*/

#define DATA_BUS PORTC		// port connected to pins 7-14 of LCD display
#define CONTROL_BUS PORTD	// port connected to pins 4 and 6 of LCD disp.
#define RS 6			// pin number of uC connected to pin 4 of LCD disp.
#define E 7			// pin number of uC connected to pin 6 of LCD disp.

/*-------------------------------------------------------------------------*/

typedef enum Direction{NONE, LEFT, RIGHT, UP, DOWN} Direction;
	
#define JOYSTICK_LIMIT_LEFT 100
#define JOYSTICK_LIMIT_RIGHT 1000
#define JOYSTICK_LIMIT_UP 100
#define JOYSTICK_LIMIT_DOWN 1000

#define DDR_SPI DDRB
#define MOSI 5
#define SCK 7
#define CSN 2
#define SPI_PORT PORTB
#define CE 4

void spi_init(void)
{
	/* Set MOSI, SCK, CSN, and CE as output, all others as input */
	DDR_SPI = ( 1 << MOSI ) | ( 1 << SCK ) | ( 1 << CSN ) | ( 1 << CE );
	/* Enable SPI, set clock rate fck/16 */
	SPCR = ( 1 << SPE ) | ( 1 << MSTR ) | ( 1 << SPR0 );
}

uint8_t spi_transfer(uint8_t data)
{
	/* Start transmission */
	SPDR = data;
	/* Wait for transmission complete */
	while( !( SPSR & ( 1 << SPIF )));

	return SPDR;
}

#define LCD_PORT 	PORTB
#define LCD_DDR 	DDRB
#define LCD_DC 		PB1
#define LCD_RST 	PB0
#define LCD_CE 		PB4

// User-Defined SPI Settings (configure these macros to use your SPI library and functions)

#define SPI_INIT() 		spi_init()
#define SPI_WRITE(x) 	spi_transfer(x) // Expected to accept and return a byte

// =========================================================================================

#define LCD_DATA 	1
#define LCD_CMD 	0
#define LCD_HEIGHT 	48
#define LCD_WIDTH 	84
#define BLACK 		1
#define WHITE 		0

extern uint8_t buffer[504];

void lcd_init( void );

void lcd_send( uint8_t dataOrCmd, uint8_t byte );

void lcd_gotoXY( uint8_t x, uint8_t y);

void lcd_update( void );

void lcd_clear( void );

void lcd_clearBuffer( void );

void lcd_contrast(uint8_t contrast);

void lcd_putPixel(uint8_t x, uint8_t y, uint8_t bw);

int divideRoundUp(int num, int divisor);

void lcd_drawImage( uint8_t* image, uint8_t x, uint8_t y );

void lcd_drawLine( uint8_t xLeft, uint8_t yLow, uint8_t xRight, uint8_t yHigh, uint8_t bw );

void lcd_fillRect( uint8_t xLeft, uint8_t yLow, uint8_t xRight, uint8_t yHigh, uint8_t bw );

void lcd_drawRect( uint8_t xLeft, uint8_t yLow, uint8_t xRight, uint8_t yHigh, uint8_t bw );

void spi_init(void);
uint8_t spi_transfer(uint8_t data);

uint8_t buffer[504];

void lcd_init( void )
{
	// Set outputs
	LCD_DDR |= (1 << LCD_DC) | (1 << LCD_RST) | (1 << LCD_CE);

	// Required Reset
	LCD_PORT &= ~(1 << LCD_RST);
	LCD_PORT |= (1 << LCD_RST);

	// Configure LCD (Refer to datasheet to alter settings)
	lcd_send( LCD_CMD, 0x21 );
	lcd_send( LCD_CMD, 0xB0 );
	lcd_send( LCD_CMD, 0x04 );
	lcd_send( LCD_CMD, 0x14 );
	lcd_send( LCD_CMD, 0x20 );
	lcd_send( LCD_CMD, 0x0C );

	lcd_contrast(55);
}

void lcd_send( uint8_t dataOrCmd, uint8_t byte )
{
	if ( dataOrCmd ) LCD_PORT |= (1 << LCD_DC);
	else LCD_PORT &= ~(1 << LCD_DC);

	LCD_PORT &= ~(1 << LCD_CE);
	SPI_WRITE( byte );
	LCD_PORT |= (1 << LCD_CE);
}

void lcd_gotoXY( uint8_t x, uint8_t y)
{
	lcd_send(0, 0x80 | x);
	lcd_send(0, 0x40 | y);
}

void lcd_update( void )
{
	lcd_gotoXY(0, 0);
	int i = 0;
	for (i=0; i < (LCD_HEIGHT * LCD_WIDTH / 8); i++)
	{
		lcd_send(LCD_DATA, buffer[i]);
	}
}

void lcd_clear( void )
{
	lcd_gotoXY(0, 0);
	int i = 0;
	for (i=0; i < (LCD_HEIGHT * LCD_WIDTH / 8); i++)
	{
		lcd_send(LCD_DATA, 0x00);
	}
}

void lcd_clearBuffer( void )
{
	int i = 0;
	for (i=0; i < (LCD_HEIGHT * LCD_WIDTH / 8); i++)
	{
		buffer[i] = 0;
	}
}

void lcd_contrast(uint8_t contrast)
{
	lcd_send(LCD_CMD, 0x21);
	lcd_send(LCD_CMD, 0x80 | contrast);
	lcd_send(LCD_CMD, 0x20);
}

void lcd_putPixel(uint8_t x, uint8_t y, uint8_t bw)
{
	// Make sure coordinate is within bounds
	if ((x >= 0) && (x < LCD_WIDTH) && (y >= 0) && (y < LCD_HEIGHT))
	{
		uint8_t shift = y % 8;
		
		if (bw) // If black, set the bit.
		buffer[x + (y/8)*LCD_WIDTH] |= 1<<shift;
		else   // If white clear the bit.
		buffer[x + (y/8)*LCD_WIDTH] &= ~(1<<shift);
	}
}

int divideRoundUp(int num, int divisor)
{
	int i, quotient;
	for(i = num, quotient = 0; i > 0; i -= divisor, quotient++);
	return quotient;
}

void lcd_drawImage( uint8_t* image, uint8_t x, uint8_t y )
{
	int row, bit, byteColumn, lineSize, height, width;
	
	height = image[0];
	width = image[1];
	lineSize = divideRoundUp( width, 8 );
	
	for ( row = height - 1; row >= 0; row-- )
	{
		for ( byteColumn = 0; byteColumn < lineSize; byteColumn++ )
		{
			for ( bit = 7; bit >= 0; bit--)
			{
				if ( (image[row*lineSize + byteColumn + 2] & (1 << bit) ) )
				lcd_putPixel( x + (byteColumn * 8) + ( 7 - bit ), y + height - row, BLACK );
			}
		}
	}
}

void lcd_drawLine( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t bw )
{
	#define sign(x) ((x) > 0 ? 1: ((x) == 0 ? 0: (-1)))

	int dx, dy, dxabs, dyabs, i, px, py, sdx, sdy, x, y;

	dx = x2 - x1;
	dy = y2 - y1;
	sdx = sign( dx );
	sdy = sign( dy );
	dxabs = ( dx > 0 ) ? dx : -dx;
	dyabs = ( dy > 0 ) ? dy : -dy;
	x = 0;
	y = 0;
	px = x1;
	py = y1;

	if ( dxabs >= dyabs )
	{
		for ( i = 0; i <= dxabs; i++ )
		{
			y += dyabs;
			if ( y >= dxabs )
			{
				y -= dxabs;
				py += sdy;
			}
			lcd_putPixel( px, py, bw );
			px += sdx;
		}
	}
	else
	{
		for ( i = 0; i <= dyabs; i++ )
		{
			x += dxabs;
			if ( x >= dyabs )
			{
				x -= dyabs;
				px += sdx;
			}
			lcd_putPixel( px, py, bw );
			py += sdy;
		}
	}
}

void lcd_fillRect( uint8_t xLeft, uint8_t yLow, uint8_t xRight, uint8_t yHigh, uint8_t bw )
{
	int i = yLow;
	int k = xLeft;
	for (i = yLow; i < yHigh + 1; i++ )
	{
		for (k = xLeft; k < xRight + 1; k++ )
		lcd_putPixel( k, i, bw );
	}
}

void lcd_drawRect( uint8_t xLeft, uint8_t yLow, uint8_t xRight, uint8_t yHigh, uint8_t bw )
{
	// Lower line
	lcd_drawLine( xLeft, yLow, xRight, yLow, bw );
	
	// Upper line
	lcd_drawLine( xLeft, yHigh, xRight, yHigh, bw );

	// Sinelines
	int i = yLow;
	for (i = yLow + 1; i < yHigh; i++ )
	{
		lcd_putPixel( xLeft, i, bw );
		lcd_putPixel( xRight, i, bw );
	}
}

char LCD_msg[33];
unsigned short myADC = 0x0000;
unsigned char tmpA = 0x00;
unsigned char tmpB = 0x00;
unsigned char tmpC = 0x00;
unsigned char tmpD = 0x00;

void ADC_init() {
	ADCSRA |= (1 << ADEN) | (1 << ADSC);
}

void ADC_channel(unsigned char channel){
	if(channel < 8 && channel >= 0){
		//CLEAR ADMUX2:0
		ADMUX &= 0xF8;
		//Set ADMUX
		ADMUX |= (channel & 0x07);
	}
}

unsigned short ADC_read(unsigned char channel){
	unsigned short myADC = 0x0000;
	ADC_channel(channel);
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1 << ADSC));
	myADC = ADC;
	return myADC;
}

typedef struct Joystick_Frame { // Code for joystick functionality given by fellow UCR classmate, Padraic Reilly
	
	unsigned short raw_x;
	unsigned short raw_y;
	unsigned char click;
	
	//These will be filled by 2 calls to Joystick_Process_Raw();
	Direction X_direction;
	Direction Y_direction;
} Joystick_Frame;

Joystick_Frame* currentJoystickFramePtr;
Joystick_Frame* nextJoystickFramePtr;

void Joystick_Process_Raw(Joystick_Frame* frame);
void Joystick_Read(Joystick_Frame* frame);
void Joystick_Tick();

void Joystick_Process_Raw(Joystick_Frame* frame){ // Code for joystick functionality given by fellow UCR classmate, Padraic Reilly
	
	//Set X
	if(frame->raw_x < JOYSTICK_LIMIT_LEFT){frame->X_direction = LEFT;}
	else if(frame->raw_x > JOYSTICK_LIMIT_RIGHT){frame->X_direction = RIGHT;}
	else{frame->X_direction = NONE;}
	//Set Y
	if(frame->raw_y < JOYSTICK_LIMIT_UP){frame->Y_direction = UP;}
	else if(frame->raw_y > JOYSTICK_LIMIT_DOWN){frame->Y_direction = DOWN;}
	else{frame->Y_direction = NONE;}
	//Decide what to do on ambiguous inputs:
	//I have decided to ignore the vertical input as it isn't 
	//time critical to move the block downward
	if(frame->X_direction != NONE && frame->Y_direction != NONE){
		frame->Y_direction = NONE;
	} 
}

void Joystick_Read(Joystick_Frame* frame){ // Code for joystick functionality given by fellow UCR classmate, Padraic Reilly
	
	//We will read the Vx and Vy from the 2-potentiometer Joystick;
	
	frame->raw_x = ADC_read(0);		//read 10 bit ADC value on ADC0	
	frame->raw_y = ADC_read(1);		//read ADC1 as well
	frame->click = (~PINA & 0x08) ? 1 : 0; //read the click
}

void LCD_init();
void LCD_ClearScreen(void);
void LCD_WriteCommand (unsigned char Command);
void LCD_Cursor (unsigned char column);
void LCD_DisplayString(unsigned char column, const unsigned char* string);
void delay_ms(int miliSec);

void LCD_ClearScreen(void) {
	LCD_WriteCommand(0x01);
}

void LCD_init(void) {

	//wait for 100 ms.
	delay_ms(100);
	LCD_WriteCommand(0x38);
	LCD_WriteCommand(0x06);
	LCD_WriteCommand(0x0f);
	LCD_WriteCommand(0x01);
	delay_ms(10);
}

void LCD_WriteCommand (unsigned char Command) {
	CLR_BIT(CONTROL_BUS,RS);
	DATA_BUS = Command;
	SET_BIT(CONTROL_BUS,E);
	asm("nop");
	CLR_BIT(CONTROL_BUS,E);
	delay_ms(2); // ClearScreen requires 1.52ms to execute
}

void LCD_WriteData(unsigned char Data) {
	SET_BIT(CONTROL_BUS,RS);
	DATA_BUS = Data;
	SET_BIT(CONTROL_BUS,E);
	asm("nop");
	CLR_BIT(CONTROL_BUS,E);
	delay_ms(1);
}

void LCD_DisplayString( unsigned char column, const unsigned char* string) {
	LCD_ClearScreen();
	unsigned char c = column;
	while(*string) {
		LCD_Cursor(c++);
		LCD_WriteData(*string++);
	}
}

void LCD_Cursor(unsigned char column) {
	if ( column < 17 ) { // 16x1 LCD: column < 9
		// 16x2 LCD: column < 17
		LCD_WriteCommand(0x80 + column - 1);
		} else {
		LCD_WriteCommand(0xB8 + column - 9);	// 16x1 LCD: column - 1
		// 16x2 LCD: column - 9
	}
}

void delay_ms(int miliSec) //for 8 Mhz crystal

{
	int i,j;
	for(i=0;i<miliSec;i++)
	for(j=0;j<775;j++)
	{
		asm("nop");
	}
}

volatile unsigned char TimerFlag = 0; // TimerISR() sets this to 1. C programmer should clear to 0.

// Internal variables for mapping AVR's ISR to our cleaner TimerISR model.
unsigned long _avr_timer_M = 1; // Start count from here, down to 0. Default 1ms
unsigned long _avr_timer_cntcurr = 0; // Current internal count of 1ms ticks

// Set TimerISR() to tick every M ms
void TimerSet(unsigned long M) {
	_avr_timer_M = M;
	_avr_timer_cntcurr = _avr_timer_M;
}

void TimerOn() {
	// AVR timer/counter controller register TCCR1
	TCCR1B 	= 0x0B;	// bit3 = 1: CTC mode (clear timer on compare)
	// bit2bit1bit0=011: prescaler /64
	// 00001011: 0x0B
	// SO, 8 MHz clock or 8,000,000 /64 = 125,000 ticks/s
	// Thus, TCNT1 register will count at 125,000 ticks/s

	// AVR output compare register OCR1A.
	OCR1A 	= 125;	// Timer interrupt will be generated when TCNT1==OCR1A
	// We want a 1 ms tick. 0.001 s * 125,000 ticks/s = 125
	// So when TCNT1 register equals 125,
	// 1 ms has passed. Thus, we compare to 125.
	// AVR timer interrupt mask register

	TIMSK1 	= 0x02; // bit1: OCIE1A -- enables compare match interrupt

	//Initialize avr counter
	TCNT1 = 0;

	// TimerISR will be called every _avr_timer_cntcurr milliseconds
	_avr_timer_cntcurr = _avr_timer_M;

	//Enable global interrupts
	SREG |= 0x80;	// 0x80: 1000000
}

void TimerOff() {
	TCCR1B 	= 0x00; // bit3bit2bit1bit0=0000: timer off
}

void TimerISR() {
	TimerFlag = 1;
}

// In our approach, the C programmer does not touch this ISR, but rather TimerISR()
ISR(TIMER1_COMPA_vect)
{
	// CPU automatically calls when TCNT0 == OCR0 (every 1 ms per TimerOn settings)
	_avr_timer_cntcurr--; 			// Count down to 0 rather than up to TOP
	if (_avr_timer_cntcurr == 0) { 	// results in a more efficient compare
		TimerISR(); 				// Call the ISR that the user uses
		_avr_timer_cntcurr = _avr_timer_M;
	}
}

void Joystick_Tick(){
	//READ + POPULATE always into the next frame before swapping buffer
	Joystick_Read(nextJoystickFramePtr);
	Joystick_Process_Raw(nextJoystickFramePtr);
	//COPY POINTER BEFORE SWAPPING
	Joystick_Frame* temp = currentJoystickFramePtr;
	//SWAP BUFFER
	currentJoystickFramePtr = nextJoystickFramePtr;
	nextJoystickFramePtr = temp;
}

unsigned char timeLeft = 0x00;
unsigned char g = 0x00;
unsigned char h = 0x00;
unsigned char score = 0x00;
unsigned char scoreTime = 0x00;
unsigned char scoreTemp = 0x00;
unsigned char a2 = 0x00;
unsigned char yHigh = 1;
unsigned char yLow = 6;
unsigned char yHigha2 = 1, yHigha3 = 1, yHigha4 = 1, yHigha5 = 1, yHigha6 = 1, yHigha7 = 1, yHigha8 = 1, yHigha9 = 1, yHigha10 = 1, yHigha11 = 1, yHigha12 = 1, yHigha13 = 1, yHigha14 = 1, yHigha15 = 1, yHigha16  = 1, yHigha17 = 1;
unsigned char yLowa2 = 6, yLowa3 = 6, yLowa4 = 6, yLowa5 = 6, yLowa6 = 6, yLowa7 = 6, yLowa8 = 6, yLowa9 = 6, yLowa10 = 6, yLowa11 = 6, yLowa12 = 6, yLowa13 = 6, yLowa14 = 6, yLowa15 = 6, yLowa16 = 6, yLowa17 = 6;
char joyVals[10];
char scoremsg[3];
enum Menu_States {INIT, GAME, SCORE} Menu_State;

Menu() {
	switch(Menu_State) {
		case INIT:
			if (currentJoystickFramePtr->click == 1) {
				LCD_DisplayString(1, "GAME IN PROGRESSSCORE 10  to win ");
				Menu_State = GAME;
				break;
			} else {
				Menu_State = INIT;
				break;
			}
		case GAME:
		
			if(currentJoystickFramePtr->click == 1) {
				Menu_State = INIT;
				break;
			}
			++timeLeft;
			if(timeLeft == 89) { // 9 sec
				Menu_State = SCORE;
				break;
			} else {
				Menu_State = GAME;
				break;
			}
		case SCORE:
			if(currentJoystickFramePtr->click == 1) {
				Menu_State = INIT;
				break;
			}
			Menu_State = SCORE;
			break;
	}
	
	switch(Menu_State) {
		case INIT:
			timeLeft = 0x00;
			score = 0x00;
			scoreTime = 0x00;
			scoreTemp = 0x00;
			a2 = 0;
			g = 0;
			h = 0;
			yHigh = yHigha2 = yHigha3 = yHigha4 = yHigha5 = yHigha6 = yHigha7 = yHigha8 = yHigha9 = yHigha10 = yHigha11 = yHigha12 = yHigha13 = yHigha14 = yHigha15 = yHigha16 = yHigha17 = 1;
			yLow = yLowa2 = yLowa3 = yLowa4 = yLowa5 = yLowa6 = yLowa7 = yLowa8 = yLowa9 = yLowa10 = yLowa11 = yLowa12 = yLowa13 = yLowa14 = yLowa15 = yLowa16 = yLowa17 = 6;
			LCD_DisplayString(1, "Press button to START");
			break;
		case GAME:
				//LCD_DisplayString(1, "GAME IN PROGRESSSCORE 10  to win ");
				break;
		case SCORE:
			lcd_clearBuffer();
			lcd_drawLine( 20, 5, 35, 5, BLACK );
			lcd_drawLine( 20, 5, 20, 42, BLACK );
			lcd_drawLine( 20, 42, 40, 42, BLACK );
			lcd_drawLine( 40, 25, 40, 42, BLACK );
			lcd_drawLine( 30, 25, 40, 25, BLACK );
			
			lcd_drawLine( 43, 5, 58, 5, BLACK );
			lcd_drawLine( 43, 5, 43, 42, BLACK );
			lcd_drawLine( 43, 42, 63, 42, BLACK );
			lcd_drawLine( 63, 25, 63, 42, BLACK );
			lcd_drawLine( 53, 25, 63, 25, BLACK );
			lcd_update();
			if(scoreTime < 20) {
				LCD_DisplayString(1, "Final Score: ");
				++scoreTime;
				break;
			}
			if(scoreTime == 20) {
				scoreTemp = score;
				scoremsg[1] = (score % 10) + '0';
				score /= 10;
				scoremsg[0] = (score % 10) + '0';
				LCD_DisplayString(1, &scoremsg);
				++scoreTime;
				break;
			}
			if((scoreTime > 20) && (scoreTime < 40)) {
				++scoreTime;
			}
			if(scoreTime == 40) {
				if(scoreTemp > 9) {
					LCD_DisplayString(1, "CONGRATS YOU AREA WINNER!");
					++scoreTime;
					break;
				} else {
					LCD_DisplayString(1, "YOU LOSE. PRESS BUTTON TO RETRY.");
					++scoreTime;
					break;
				}
			}
			break;
	}
}

unsigned char tempCount = 0;
unsigned char tempi = 0;


Apples() {
	if(Menu_State == GAME) {
		unsigned short tmpADC = currentJoystickFramePtr->raw_x;
		LCD_msg[3] = (tmpADC % 10) + '0';
		tmpADC /= 10;
		LCD_msg[2] = (tmpADC % 10) + '0';
		tmpADC /= 10;
		LCD_msg[1] = (tmpADC % 10) + '0';
		tmpADC /= 10;
		LCD_msg[0] = (tmpADC % 10) + '0';
		LCD_msg[4] = ' ';
		
		if(LCD_msg[0] == '1') {
			lcd_clearBuffer();
			lcd_fillRect(1,41,28,47, BLACK);
			lcd_fillRect(57,41,83,47, WHITE);
			lcd_fillRect(29,41,56,47, WHITE);
			lcd_update();
			} else if (LCD_msg[1] == '0') {
			lcd_clearBuffer();
			lcd_fillRect(57,41,83,47, BLACK);
			lcd_fillRect(1,41,28,47, WHITE);
			lcd_fillRect(29,41,56,47, WHITE);
			lcd_update();
			} else {
			lcd_clearBuffer();
			lcd_fillRect(29,41,56,47, BLACK);
			lcd_fillRect(1,41,28,47, WHITE);
			lcd_fillRect(57,41,83,47, WHITE);
			lcd_update();
		}
	a2+=3;
	//lcd_clearBuffer();
	lcd_drawRect(1, yHigh, 6, yLow, BLACK);
	lcd_update();
	yHigh = yHigh + 3;
	yLow = yLow + 3;
	if((a2 > 20) && (a2 < 70)) {
		//lcd_clearBuffer();
		lcd_drawRect(19, yHigha2, 24, yLowa2, BLACK);
		lcd_update();
		yHigha2 = yHigha2 + 3;
		yLowa2 +=3;
	}
	if((a2 > 40) && (a2 < 90)) {
		lcd_drawRect(68, yHigha3, 73, yLowa3, BLACK);
		lcd_update();
		yHigha3+=3;
		yLowa3+=3;
	}
	if((a2 > 60) && (a2 < 110)) {
		lcd_drawRect(38, yHigha4, 43, yLowa4, BLACK);
		lcd_update();
		yHigha4+=3;
		yLowa4+=3;
	}
	if((a2 > 80) && (a2 < 130)) {
		lcd_drawRect(62, yHigha5, 67, yLowa5, BLACK);
		lcd_update();
		yHigha5+=3;
		yLowa5+=3;
	}
	if((a2 > 100) && (a2 < 150)) {
		lcd_drawRect(16, yHigha6, 21, yLowa6, BLACK);
		lcd_update();
		yHigha6+=3;
		yLowa6+=3;
	}
	if((a2 > 120) && (a2 < 170)){
		lcd_drawRect(77, yHigha7, 82, yLowa7, BLACK);
		lcd_update();
		yHigha7+=3;
		yLowa7+=3;
	}
	if((a2 > 140) && (a2 < 190)){
		lcd_drawRect(44, yHigha8, 49, yLowa8, BLACK);
		lcd_update();
		yHigha8+=3;
		yLowa8+=3;
	}
	if((a2 > 160) && (a2 < 210)) {
		lcd_drawRect(8, yHigha9, 13, yLowa9, BLACK);
		lcd_update();
		yHigha9+=3;
		yLowa9+=3;
	}
	if((a2 > 180) && (a2 < 230)){
		lcd_drawRect(71, yHigha10, 76, yLowa10, BLACK);
		lcd_update();
		yHigha10+=3;
		yLowa10+=3;
	}
	if((a2 > 200) && (a2 < 250)){
		lcd_drawRect(42, yHigha11, 47, yLowa11, BLACK);
		lcd_update();
		yHigha11+=3;
		yLowa11+=3;
	}
	if((a2 > 220) && (a2 < 270)){
		lcd_drawRect(17, yHigha12, 22, yLowa12, BLACK);
		lcd_update();
		yHigha12+=3;
		yLowa12+=3;
	}
	/*if((a2 > 240) && (a2 < 290)){
		lcd_drawRect(5, yHigha13, 10, yLowa13, BLACK);
		lcd_update();
		yHigha13+=3;
		yLowa13+=3;
	}*/
	if((a2 > 260) && (a2 < 310)){
		lcd_drawRect(33, yHigha14, 38, yLowa14, BLACK);
		lcd_update();
		yHigha14+=3;
		yLowa14+=3;
	}
	if((a2 > 280) && (a2 < 330)){
		lcd_drawRect(71, yHigha15, 76, yLowa15, BLACK);
		lcd_update();
		yHigha15+=3;
		yLowa15+=3;
	}
	if((a2 > 300) && (a2 < 350)){
		lcd_drawRect(26, yHigha16, 31, yLowa16, BLACK);
		lcd_update();
		yHigha16+=3;
		yLowa16+=3;
	}
	if((a2 > 320) && (a2 < 370)){
		lcd_drawRect(56, yHigha17, 61, yLowa17, BLACK);
		lcd_update();
		yHigha17+=3;
		yLowa17+=3;
	}
	}
}

countScore() {
	++g;
	if((g == 15) && LCD_msg[0] == '1') { //L
		score++;
	}
	if((g == 21) && LCD_msg[0] == '1') { //L
		score++;
	}
	if((g == 28) && LCD_msg[1] == '0' && LCD_msg[0] == '0') { //R
		score++;
	}
	if((g == 35) && LCD_msg[1] == '5') { //M
		score++;
	}
	if((g == 41) && LCD_msg[1] == '0' && LCD_msg[0] == '0') { //R
		score++;
	}
	if((g == 48) && LCD_msg[0] == '1') { //L
		score++;
	}
	if((g == 55) && LCD_msg[1] == '0' && LCD_msg[0] == '0') { //R
		score++;
	}
	if((g == 61) && LCD_msg[1] == '5') { //M
		score++;
	}
	if((g == 68) && LCD_msg[0] == '1') { //L
		score++;
	}
	if((g == 75) && LCD_msg[1] == '0' && LCD_msg[0] == '0') { //R
		score++;
	}
	if((g == 81) && LCD_msg[1] == '5') { //M
		score++;
	}
	if((g == 88) && LCD_msg[0] == '1') { // L
		score++;
	}
	
}

int main(void)
{
	DDRA = 0x00; PORTA = 0xFF;
	DDRB = 0xFF; PORTB = 0x00;
	DDRC = 0xFF; PORTC = 0x00;
	DDRD = 0xFF; PORTD = 0x00;
	ADC_init();
	LCD_init();
	SPI_INIT();
	lcd_init();
	currentJoystickFramePtr = (Joystick_Frame*) malloc(sizeof(Joystick_Frame));
	nextJoystickFramePtr = (Joystick_Frame*) malloc(sizeof(Joystick_Frame));
	TimerSet(100);
	TimerOn();

    while (1) {
		Menu();
		Joystick_Tick();
		Apples();
		countScore();
		while (!TimerFlag);
		TimerFlag = 0;
	}
	return 0;
}
