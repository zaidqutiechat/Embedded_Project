/* -- Pin masks	-------------------------------------------------- */
#define BUZZER_MASK    	0x10	/* RC4, controls the buzzer */
#define NIGHTLED_MASK  	0x01	/* RB0, controls the blue LED, using the LDR */
#define TRIG_MASK      	0x04	/* RB2, sends trig pulse to the ultra-sonic sensor */
#define ECHO_MASK      	0x02	/* RB1, read the echo from the ultra-sonic sensor */
#define CLR_MOTOR      	0xC7    /* RB3, RB4 and RB5, mask that clears the H-Bridge lines */
#define RIGHT_MASK     	0x28	/* RB3 & RB5, sets the DC motor to run clockwise */
#define LEFT_MASK      	0x30	/* RB3 & RB5, sets the DC motor to run counter clockwise */
/* -- Timing constants ---------------------------------------------------- */
#define LOOP_MS        	20 //	the amount of time between each interation within main, controls how ofter the IR ultrasonic and lid motor are checked and updated
#define TIMEOUT_MS     	2000 // amount of time the lid can remain open before it closes automatically, in case a person is standing near the bin and not using it or moving away
#define PROX_HOLD_MS   	500 // how long the person needs to stand near the bin before it opens, prevents false positives from people or animals passing by
/* -- global variables ---------------------------------------------------- */
volatile unsigned int  angle  = 2000;	/* the ISR reads this value to know how long to drive HIGH, set to volatile so ISR always reads latest val */
volatile unsigned char HL 	= 1; // toggle flag, 1: set pin to HIGH, 0: set pin to LOW
const unsigned int servo_pw[3] = {2250, 3250, 1250}; //sets the servo levels for the status indicator

unsigned char ir50_val = 0, ir100_val = 0;
unsigned char fill_lvl = 0;    	
unsigned char prev_lvl = 0xFF; 	

unsigned int  ldrValue = 0;
const unsigned int LDR_THRESHOLD = 400;
volatile unsigned long Distance = 999;
unsigned char  open_flag  	= 0;
unsigned long  open_elapsed_ms = 0;
unsigned long  proximity_ms	= 0;
unsigned char  wasFar      	= 1;
unsigned long  sonar_hi=0, sonar_lo=0, sonar_cnt=0;
unsigned int   d_i = 0;
unsigned int i;

/* -- Prototypes 	(so we can use the functions without declaring them sequentially) ------------------------------------- */
void initCore(void); 
void initServo(void);
void updateFillLevel(void); 
void setServoLevel(void);
void ATD_init(void); 
unsigned int ATD_read(void); 
void updateRoomLight(void);
void readSonar(void);
void updateLidMotor(void);
void motorRight(void);
void motorLeft(void);
void motorStop(void);
void delay_us(unsigned int); 
void delay_ms(unsigned int);

/* -- CCP1 ISR (servo PWM) --------------------------- */
void interrupt(void)
{
	if (PIR1 & 0x04) { //check the second but of the PIR1 register, if ccp1 caused the interrupt, in other words, did timer-1 match the compare value:
    	if (HL) // if PWM is in the high phase
		{ 
			CCPR1H=angle>>8; 

			CCPR1L=angle; 
			
			HL=0; 

			CCP1CON=0x09; 
		}
    	else 
		{ 
			CCPR1H=(40000-angle)>>8;
			 
			CCPR1L=(40000-angle);
			  
			 HL=1; 

			CCP1CON=0x08;
		}

    	TMR1H=TMR1L=0; // reset high low timers
		
		PIR1&=0xFB; 
	}
}

void main(void)
{
	initCore(); 
	
	initServo();

	while (1)
	{
    	if (open_flag == 0) //if the lid is closed, check the fill level.
		{
		updateFillLevel();
		}

    	readSonar(); // check if someone is close
    	updateLidMotor(); // open then close the lid when someone approaches
    	delay_ms(LOOP_MS); // wait before the next cycle
	}
}

/* -- IR -> servo/buzzer ---------------------- */
void updateFillLevel(void)
{
	ir50_val  = !(PORTC & 0x01);	/* RC0 */

	ir100_val = !(PORTD & 0x01);	/* RD0 */

	if (ir100_val)

	 	fill_lvl = 2;
	
	else if (ir50_val)

	  	fill_lvl = 1;
	
	else 

		fill_lvl = 0;

	setServoLevel();

	if (fill_lvl == 2 && prev_lvl != 2) { //check if the lid just became full or not, ensure that the buzzer only sounds once

		for(i=0; i<3; i++){
    	PORTC |= BUZZER_MASK; //buzzer on

		delay_ms(300);

		PORTC &= ~BUZZER_MASK; //buzzer off

		delay_ms(300);
		}
	}
	prev_lvl = fill_lvl;
}

void setServoLevel(void)
{ 
	angle = servo_pw[fill_lvl]; 
}

/* -- LDR -- */
void updateRoomLight(void)
{
	ldrValue = ATD_read();
	if (ldrValue < LDR_THRESHOLD) 
		PORTB |= NIGHTLED_MASK;
	else       
		PORTB &= ~NIGHTLED_MASK;
}

void readSonar(void)
{
	PIE1 &= ~0x04; 
	T1CON=0x18; 
	TMR1H=TMR1L=0;
	PORTB |= TRIG_MASK; 
	delay_us(10); 
	PORTB &= ~TRIG_MASK;
	sonar_cnt=60000; 
	while(! (PORTB&ECHO_MASK) && sonar_cnt--) 
	asm NOP;
	T1CON=0x19; 
	sonar_cnt=60000;
	while( (PORTB&ECHO_MASK) && sonar_cnt--)
	asm NOP;
	T1CON=0x18;
	sonar_hi=TMR1H;
	sonar_lo=TMR1L;
	Distance=((unsigned long)sonar_hi<<8|sonar_lo) * 17UL / 1000UL;
	PIE1 |= 0x04; T1CON=0x01;
}
/* -- code that updates the lid motor --*/
void updateLidMotor(void)
{
	if (open_flag == 0){ //if the lid is closed
    	open_elapsed_ms = 0; 
    	if (Distance <= 15){
        	if (wasFar) //if the person was previously far away
			proximity_ms += LOOP_MS; //start counting how long the person has been standing near the bin
        	if (proximity_ms >= PROX_HOLD_MS){	//if they have been standing near the bin for long enough
            	open_flag = 1; 
            	updateRoomLight(); //check the light levels in the room
            	motorRight();  //move the motor right to open it
				delay_ms(500);
				motorStop(); //stop moving the motor
            	wasFar = 0; //set was far to 0
				proximity_ms = 0; //reset the prox timer, the idea is to ensure that the same person can not trigger the bin multiple times
        	}
    	} else { 
			wasFar = 1; 
			proximity_ms = 0;
		}
	} else {
    	if (Distance > 15){ //if the person moves away from the bin
        	open_flag = 0;
			motorLeft(); //close the bin
			delay_ms(500); 
			motorStop();
        	PORTB &= ~NIGHTLED_MASK; //turn off the lights
			wasFar = 1;
    	} else { //person is still standing near the bin
        	open_elapsed_ms += LOOP_MS; //count how long the bin has been open
        	if (open_elapsed_ms >= TIMEOUT_MS){ //if the bin has been open for too long, assume false positive and close the bin
            	motorLeft();
				delay_ms(500);
				motorStop();
            	PORTB &= ~NIGHTLED_MASK;
            	open_flag = 0;
        	}
    	}
	}
}

void motorRight(void){ PORTB = (PORTB & CLR_MOTOR) | RIGHT_MASK; }
void motorLeft (void){ PORTB = (PORTB & CLR_MOTOR) | LEFT_MASK;  }
void motorStop (void){ PORTB &= CLR_MOTOR; }

void ATD_init(void){ ADCON0=0x41; ADCON1=0xCE; TRISA=0x01; }
unsigned int ATD_read(void){ ADCON0|=0x04; while(ADCON0&0x04); return (ADRESH<<8)|ADRESL; }

void initCore(void)
{
	TRISC = 0x03;
	PORTC &= ~BUZZER_MASK;
	TRISB &= ~0x38;
	TRISB &= ~NIGHTLED_MASK;
	TRISB |=  ECHO_MASK;
	TRISB &= ~TRIG_MASK;
	PORTB &= ~(NIGHTLED_MASK|TRIG_MASK); 
	motorStop();
	ATD_init();
}
void initServo(void){ TMR1H=TMR1L=0; CCP1CON=0x08; PIE1|=0x04; T1CON=0x01; INTCON=0xC0; }
void delay_us(unsigned int us){ while(us--){ asm NOP; asm NOP; } }
void delay_ms(unsigned int ms){ while(ms--) for(d_i=0; d_i<155; ++d_i) asm NOP; }
