#define D4 eS_PORTD4
#define D5 eS_PORTD5
#define D6 eS_PORTD6
#define D7 eS_PORTD7
#define RS eS_PORTC6
#define EN eS_PORTC7

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include "util/delay.h"
#include "lcd.h"
#include "gps.h"
#include "path.h"
#include "print.h"



#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#ifndef BAUD
#define BAUD 9600
#endif
#include <util/setbaud.h>

#define LEFT_MOTOR PA0
#define RIGHT_MOTOR PA1

//push-button states
#define NOT_STARTED 0
#define STARTED 1
#define STOPPED 2

//LCD-display states
#define SHOW_CURR_LOC 0
#define SHOW_LOC_DIST 1
#define SHOW_DIRECTION 2
#define DONT_CHANGE 3

//str to primarily get gps string
char str[150] = "START";
int cursor;
char started;
char firstData;
char completed;
TrackPoint curr, prev, next;
CurrentLocation currLoc;
uint16_t totalDist;
int lastidx;
int wayptidx;
char direction;
char button_state = NOT_STARTED;
char lcd_state = DONT_CHANGE;
volatile uint8_t tot_overflow;
char timer1_count=0;
uint16_t hour ;
uint16_t min ;
float sec ;
char x;


void navigation();
void motorSwitch(char left, char right);
void useDirection(int direction);
void getData();
void init();
void showLatLonInLCD();
void showDistanceOnLCD(uint16_t dist);
void showCurrentLocation();
void showLocationDistance();
void showDirection();

void uart_putchar(char c, FILE *stream) {
	if (c == '\n') {
		uart_putchar('\r', stream);
	}
	loop_until_bit_is_set(UCSRA, UDRE);
	UDR = c;
}

char uart_getchar(FILE *stream) {
	loop_until_bit_is_set(UCSRA, RXC);
	return UDR;
}

//double transmission speed, TX,RX pin enabled, asynchronous mode, no parity, 1 stop bit
void usart_init()
{
	UCSRA = (1<<U2X);
	UCSRB = (1<<(RXEN)) | (1<<(TXEN));
	UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); // Use 8-bit character sizes
	
	UBRRH=0;
	UBRRL=0xC;	
}

//keep polling until get data from GPS
unsigned int usart_getch()
{
	while ((UCSRA & (1 << RXC)) == 0)
	{
		
	} // Do nothing until data have been recieved and is ready to be read from UDR
	return(UDR); // return the byte
}

//timer initialization
void timer1_init()
{
	// set up timer with prescaler = 64 and CTC mode
	TCCR1B |= (1 << WGM12)|(1 << CS11)|(1 << CS10);
	// initialize counter
	TCNT1 = 0;
	// initialize compare value
	OCR1A = 24999;
	// enable compare interrupt
	TIMSK |= (1 << OCIE1A);
	// enable global interrupts
	sei();
}

//interrupt1 initialization for rising edge
void int1_init(){
	GICR = (1<<INT1);
	MCUCR = MCUCR | 0b00001100;
	sei();
}

// this ISR is fired whenever a match occurs
// triggers getting data from GPS and navigation
ISR (TIMER1_COMPA_vect)
{
	//PORTB = ~PORTB;
	timer1_count++;
	if(timer1_count == 2){
		timer1_count = 0;
		if(button_state==STARTED && completed == 0)
		getData();
	}
}

//maintains push-button state
ISR(INT1_vect)
{
	printf("button pushed\n");
	Lcd4_Clear();
	Lcd4_Set_Cursor(1,0);
	
	if(button_state == NOT_STARTED){
		Lcd4_Write_String("Starting...");
		init();
		button_state = STARTED;
		
	}else if(button_state == STARTED){
		button_state =  STOPPED;
		started = 0;
		
		Lcd4_Write_String("Stopping...");
		_delay_ms(1000);
		Lcd4_Clear();
		
	}else if(button_state == STOPPED){
		
		Lcd4_Clear();
		Lcd4_Set_Cursor(1,0);
		
		Lcd4_Write_String("Welcome To!");
		Lcd4_Set_Cursor(2,0);
		Lcd4_Write_String("-GPS NAVIGATOR-");
		button_state = NOT_STARTED;
	}
	//_delay_ms(1000);
	printf("button current state %d\n",button_state);
}


void timer0_init()
{
	// set up timer with prescaler = 256
	TCCR0 |= (1 << CS02);
	// initialize counter
	TCNT0 = 0;
	// enable overflow interrupt
	TIMSK |= (1 << TOIE0);	
	// enable global interrupts
	sei();
	// initialize overflow counter variable
	tot_overflow = 0;
}

// TIMER0 overflow interrupt service routine
// called whenever TCNT0 overflows
ISR(TIMER0_OVF_vect)
{
	// keep a track of number of overflows
	tot_overflow++;
	if(tot_overflow == 46){
		sec += 3.0;
		if(sec >= 60){
			min++;
			sec -= 60;
			if(min >= 60){
				hour++;
				min -= 60;
			}
		}
		//PORTB = ~PORTB;
		printf("timer0\n");
		tot_overflow = 0;
		if(completed == 0 && button_state== STARTED ){
			printf("lcd state: %d\n",lcd_state);
			if(lcd_state == SHOW_CURR_LOC){
				showCurrentLocation();
				//lcd_state = SHOW_LOC_DIST;
			}else if(lcd_state == SHOW_DIRECTION){
				//showDirection();
				lcd_state = SHOW_CURR_LOC;
			}else if(lcd_state == SHOW_LOC_DIST){
				showLocationDistance();
				lcd_state = SHOW_CURR_LOC;
			}else if(lcd_state == DONT_CHANGE ){
				
				lcd_state = SHOW_CURR_LOC;
			}
		}
	}
}

/************************************************************************/
/* initializes flags and variables                                                                     */
/************************************************************************/
void init(){
	totalDist = 0;
	printf("initializing variables\n");
	
	//showLatLonInLCD();
	_delay_ms(500);
	
	
	printf("initializing variables\n");
	
	timer1_init();
	timer0_init();
	
	PORTB = 0;
	
	str[0] = 0;
	cursor=0;
	started=0;
	firstData=0;
	completed=0;
	initTrackPoint(&prev);
	initTrackPoint(&curr);
	initTrackPoint(&next);
	currLoc.tp = &curr;
	currLoc.distToWp = INFINITY;
	totalDist = 0;
	lastidx = sizeof(path)/sizeof(path[0]);
	wayptidx = 1;
	direction = STRAIGHT;
	lcd_state = DONT_CHANGE;
	hour = 0 ;
	min = 0;
	sec = 0;
	
	printf("completed initializing variables\n");
}

int main(void)
{
	//lcd init
	DDRD = 0xff;
	DDRC = 0xFF;
	
	DDRA = 0x03;//button and motor
	DDRB = 0xff;//lED 4 test

	//initializing LCD
	Lcd4_Init();
	//interrupt init
	int1_init();
	
	//USART initialization
	usart_init();
	FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
	FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);
	stdout = &uart_output;
	stdin  = &uart_input;
	printf("\n\n-----------------------MAIN-----------------------\n\n");
   
   Lcd4_Clear();
   Lcd4_Set_Cursor(1,0);
   
   Lcd4_Write_String("Welcome To!");
   Lcd4_Set_Cursor(2,0);
   Lcd4_Write_String("-GPS NAVIGATOR-");
   _delay_ms(1000);
	
	while(1){
		
	}
}

/************************************************************************/
/* keep polling until get a NMEA string
checks the string's validity.
if valid start navigation
else keep fetching data                                                                  */
/************************************************************************/

void getData(){
	printf("\n\n\nget Data\n");
	if(button_state == STARTED && completed == 0){
		while(1){
			char value = usart_getch();
			
			if(value == '$')
			{
				str[cursor]=0;
				printf("%s\n",str);
				
				int res = getCurrlocation(str, &curr, &prev);
				if(res==false)
				printf("invalid data\n");
				else{
					char buff1[10];
					char buff2[10];
					snprintf(buff1,sizeof(buff1),"%f", curr.lat);
					snprintf(buff2,sizeof(buff2),"%f", curr.lon);
					printf("---------valid lat: %s lon: %s--------\n",buff1,buff2);
					//printf("lat %d lon %d \n",curr.lat,curr.lon);
					
					if(firstData==1)
						navigation();
					firstData=1;
				}
				cursor=0;
			}
			else
			{
				str[cursor] = value;
				cursor++;
			}
			//printf("%c)",value);
		}
	}
}


/************************************************************************/
/* we calculate how far next waypoint is
if it is more than 25m then we calculate expected bearing( bearing of waypoint 
with respect to current location) and actual bearing (bearing between previous and current loaction)   
and calculate direction. 
if user is diverting from route we vibrate motor to let user know. 

if waypoint is within 25m we calculate direction and guide user through motor and LCD display
if waypoint is within 20m we stop vibrating the motor we were previously vibrating

if user should turn left , left motor is vibrated
if user should turn right , right motor is vibrated
                                                                  */
/************************************************************************/
void navigation(int idx){
	
	printf("\n\nstarting navigation\n");
	char b1[17]; char b2[17]; snprintf(b1,sizeof b1,"%lf",curr.lat); snprintf(b2,sizeof b2,"%lf",curr.lon);
	printf("\ncurr: %s %s \n",b1,b2);
	snprintf(b1,sizeof b1,"%lf",prev.lat); snprintf(b2,sizeof b2,"%lf",prev.lon);
	printf("prev: %s %s \n",b1,b2);
	printf("waypoint idx: %d \n",wayptidx);
	snprintf(b1,sizeof b1,"%lf",path[wayptidx][0]); snprintf(b2,sizeof b2,"%lf",path[wayptidx][1]);
	printf("waypoint : %s %s\n",b1,b2);
	if(wayptidx<lastidx) {
		
		uint16_t dist = distance(curr.lat,curr.lon,path[wayptidx][0],path[wayptidx][1],'m');
		currLoc.distToWp=dist;
		
		printf("distance to next wp: %d m\n",dist);
		uint16_t tempDist=0;
		
		tempDist = distance(prev.lat,prev.lon,curr.lat,curr.lon,'m');
		printf("tempdist: %d m\n",tempDist);
		if(dist<NEAR_DIST){
			printf("NEAR_DIST\n");
			wayptidx++;
			if(started == 0 ){
				double angle = getAngle(prev.lat,prev.lon,curr.lat,curr.lon,path[wayptidx][0],path[wayptidx][1]);
				direction = getTurn(angle);
				useDirection(direction);

			}else
			{
				useDirection(STRAIGHT);
				//showLatLonInLCD();
			}
			//turn off both Motor
			motorSwitch(0,0);
		}
		else if(dist<INIT_DIST){
			printf("INIT_DIST\n");

			double angle = getAngle(prev.lat,prev.lon,curr.lat,curr.lon,path[wayptidx][0],path[wayptidx][1]);
			char direction = getTurn(angle);
			useDirection(direction);
		}
		else{
			printf("More than 25 meter");
			useDirection(STRAIGHT);
		}
		totalDist+=tempDist;
		printf("distance covered till now : %d m\n",totalDist);

	}
	else{//if destination reached
		uint16_t tempDist = distance(prev.lat,prev.lon,curr.lat,curr.lon,'m');
		if(currLoc.distToWp - tempDist > 5)
			 currLoc.distToWp = distance(curr.lat,curr.lon,path[lastidx-1][0],path[lastidx-1][1],'m');
		else{
			printf("\n***RUN COMPLETE***\n\n");
			completed = 1;
			lcd_state = DONT_CHANGE;
		
			Lcd4_Clear();
			Lcd4_Set_Cursor(1,0);
			Lcd4_Write_String("RUN COMPLETE");
			char buffer[17];
			snprintf(buffer, sizeof buffer, "Total %dm", totalDist);
			Lcd4_Set_Cursor(2,0);
			Lcd4_Write_String(buffer);
		
			//turn off left motor turn off right motor
			motorSwitch(1,1);
			_delay_ms(1000);
			motorSwitch(0,0);
			printf("total distance %d\n",totalDist);
			_delay_ms(2000);
		}
		
	}
	started=1;
	snprintf(b1,sizeof b1,"%lf",path[wayptidx][0]); snprintf(b2,sizeof b2,"%lf",path[wayptidx][1]);
	printf("next waypoint    : %s %s\n",b1,b2);
	printf("\nending navigation\n");
}




/************************************************************************/
/* given direction proper instruction is show on LCD 
and proper motor's are vibrated                                                                     */
/************************************************************************/
void useDirection(int direction)
{
	printf("\nhandling direction\n");
	printf("\ndirection: %d\n",direction);
	if(direction==LEFT) {
		printf("****TURN LEFT\n");
		//showDirection();
		lcd_state = SHOW_DIRECTION;
		//show in lcd turn left
		Lcd4_Clear();
		Lcd4_Set_Cursor(1,0);
		Lcd4_Write_String("TURN LEFT");
		char buffer[17];
		snprintf(buffer, sizeof buffer, "covered %dm", totalDist);
		Lcd4_Set_Cursor(2,0);
		Lcd4_Write_String(buffer);
		/*snprintf(buffer, sizeof buffer, "%d", currLoc.distToWp);
		Lcd4_Set_Cursor(2,0);
		Lcd4_Write_String("Dist");
		Lcd4_Set_Cursor(2,5);
		Lcd4_Write_String(buffer);*/
		
		//turn on left motor turn off right motor
		motorSwitch(1,0);
	}
	else if(direction == RIGHT){
		printf("****TURN RIGHT\n");
		lcd_state = SHOW_DIRECTION;
		//showDirection();
		//show in lcd turn left
		Lcd4_Clear();
		Lcd4_Set_Cursor(1,0);
		Lcd4_Write_String("TURN RIGHT");
		char buffer[17];
		snprintf(buffer, sizeof buffer, "covered %dm", totalDist);
		Lcd4_Set_Cursor(2,0);
		Lcd4_Write_String(buffer);
		/*snprintf(buffer, sizeof buffer, "%d", currLoc.distToWp);
		Lcd4_Set_Cursor(2,0);
		Lcd4_Write_String("Dist");
		Lcd4_Set_Cursor(2,5);
		Lcd4_Write_String(buffer);*/
		
		//turn off left motor turn on right motor
		motorSwitch(0,1);
	}
	else if(direction == STRAIGHT || direction == NO_CHANGE){
		printf("****GO STRAIGHT\n");
		//show in lcd turn left
		
		Lcd4_Clear();
		Lcd4_Set_Cursor(1,0);
		Lcd4_Write_String("Go Straight");
		char buffer[17];
		snprintf(buffer, sizeof buffer, "covered %dm", totalDist);
		Lcd4_Set_Cursor(2,0);
		Lcd4_Write_String(buffer);
		/*snprintf(buffer, sizeof buffer, "%d", currLoc.distToWp);
		Lcd4_Set_Cursor(2,0);
		Lcd4_Write_String("Dist");
		Lcd4_Set_Cursor(2,5);
		Lcd4_Write_String(buffer);*/
		if(x==0){
			lcd_state = SHOW_CURR_LOC;
			x++;
		}else{
			lcd_state = SHOW_LOC_DIST;
			x=0;
		}
		
		//turn off left motor turn off right motor
		motorSwitch(0,0);
	}
	else if(direction == ERROR){
		printf("--------ERROR-------");
		motorSwitch(0,0);
	}
	printf("handled direction\n\n");
}

//LCD display shows current Lattitude and Longitude
void showLatLonInLCD(){
	printf("show lat lon on LCD");
	char buff1[10];
	char buff2[10];
	ftoa(curr.lat,buff1,4);
	ftoa(curr.lon,buff2,4);
	Lcd4_Clear();
	Lcd4_Set_Cursor(1,0);
	Lcd4_Write_String("Current Location");

	
	Lcd4_Set_Cursor(2,0);
	Lcd4_Write_String(buff1);
	Lcd4_Set_Cursor(2,9);
	Lcd4_Write_String(buff2);
	
}

/************************************************************************/
/* controls which motor to turn on, which one to turn off
0 = off , 1 = on                                                                     */
/************************************************************************/
void motorSwitch(char left, char right){
	if(left == 0)
	PORTA = PORTA & (~(1<<LEFT_MOTOR));
	else
	PORTA = PORTA | (1<<LEFT_MOTOR);
	if(right == 0)
	PORTA = PORTA & (~(1<<RIGHT_MOTOR));
	else
	PORTA = PORTA | (1<<RIGHT_MOTOR);
}

void showDistanceOnLCD(uint16_t dist){
	printf("show distance on LCD \n");
	char buffer[17];
	snprintf(buffer, sizeof buffer, "Dist %d m", dist);
	//Lcd4_Clear();
	//Lcd4_Set_Cursor(2,0);
	//Lcd4_Write_String("Dist");
	Lcd4_Set_Cursor(2,0);
	Lcd4_Write_String(buffer);
}

void showCurrentLocation(){
	printf("showing current location\n");
	char buff1[17];char buff2[17];
	snprintf(buff1,sizeof buff1,"curLat %f",curr.lat);
	snprintf(buff2,sizeof buff2,"curLon %f",curr.lon);
	Lcd4_Clear();
	Lcd4_Set_Cursor(1,0);
	Lcd4_Write_String(buff1);
	Lcd4_Set_Cursor(2,0);
	Lcd4_Write_String(buff2);
}

void showLocationDistance(){
	printf("showing currloc & distance left");
	Lcd4_Clear();
	char buff1[17];
	//char buff2[10];
	//ftoa(curr.lat,buff1,4);
	//ftoa(curr.lon,buff2,4);
	//Lcd4_Set_Cursor(1,0);
	//Lcd4_Write_String(buff1);
	//Lcd4_Set_Cursor(1,9);
	//Lcd4_Write_String(buff2);
	snprintf(buff1,sizeof buff1,"covered %d m",totalDist);
	printf(buff1);
	Lcd4_Set_Cursor(1,0);
	Lcd4_Write_String(buff1);
	
	//uint16_t d = distance(curr.lat,curr.lon,path[lastidx-1][0],path[lastidx-1][1],'m');
	//printf("distance left %d covered %d\n",d,totalDist);
	char buff3[17];
	//snprintf(buff3,sizeof buff3,"%dm to go",d);
	float dd = totalDist;
	uint16_t elapsedTime = hour*3600 + min*60 + sec;
	float velo =dd/elapsedTime; //precision lost
	snprintf(buff3,sizeof buff3,"Speed %.2f mps",velo);
	printf(buff3);
	printf("totalDist %d elapsed time %d s\n",totalDist,elapsedTime);
	Lcd4_Set_Cursor(2,0);
	Lcd4_Write_String(buff3);
}

void showDirection(){
	printf("showing Direction\n");
	Lcd4_Clear();
	Lcd4_Set_Cursor(1,0);
	if(direction == LEFT){
		printf("left\n");
		Lcd4_Write_String("TURN LEFT");
	}else if(direction == RIGHT){
		printf("right\n");
		Lcd4_Write_String("TURN RIGHT");
	}else{
		printf("straight\n");
		Lcd4_Write_String("GO STRAIGHT");
	}
	
	char buff[17];
	snprintf(buff,sizeof buff,"Covered %d m",totalDist);
	Lcd4_Set_Cursor(2,0);
	Lcd4_Write_String(buff);
}