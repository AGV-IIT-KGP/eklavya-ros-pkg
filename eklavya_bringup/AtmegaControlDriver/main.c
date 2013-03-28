/*
 * PID_trial_3_ver.1.1.c
 *
 1.sampling rate in ms....affects counts
 2.
 
 
 * Created: 24-Mar-13 10:26:13 PM
 *  Author: Bismaya , Jit
 */ 
#define F_CPU 16000000UL
#define PI 3.141592653589793238462643383
/*Define Headers here*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

/*Define Definitions here*/
int Gcount=0;
float  Kp=220;
float  Ki=10;
//12
float  Kd=15;
int tuning_temp=0;
//15
float Ko=12000.0;
#define PID_PWM_SCALE 1
#define PID 1
/* Define the robot paramters */
int cpr = 400; //
float wheelDiameter = 0.146; // meters
float wheelTrack = 0.291; // meters
//float ticksPerMeter = cpr / (PI * wheelDiameter);


/* Run the PID loop at 30 times per second */
int PID_RATE=30;    // Hz
int PID_INTERVAL;
/* Convert the rate into an interval */


/* Stop the robot if it hasn't received a movement command
in this number of milliseconds */
#define AUTO_STOP_INTERVAL 8000
long lastMotorCommand = AUTO_STOP_INTERVAL;
unsigned long tempTest = 0; 
#define R1 (PINC&0b01000000)
#define L1 (PINC&0b10000000)
#define L2 (PIND&0b01000000)
#define R2 (PINB&0b00000001)

#define MOTOR_MAX_PWM 255
#define MOTOR_MIN_PWM 20
#define MOTOR_OFFSET_PWM 20
#define L_MAX_COUNT 375
#define R_MAX_COUNT 375 //counts per sampling rate.
#define SamplingRate 10//sampling rate in millisecs

/*Define Global Variables here*/
 uint16_t rCount=0,lCount=0,lCount2=0,rCount2=0;
 int8_t rDir=1,lDir=1,lDirOut=1,rDirOut=1;
 
 int tempL[200],tempR[200];
 int j=0;

 //*The PID functions supposed to be triggered at 5ms*/
int16_t lErrorPrevious=0,lError=0,rError=0,rErrorPrevious=0,lTarget=0,rTarget=0,lErrorSum=0,rErrorSum=0;
int16_t lCountOut=0,rCountOut=0,L_PWM=0,R_PWM=0;
int state=0,mildTarget_L=0,mildTarget_R=0,motion=0,pid_tuning_state=0;
int wflag=0;
int16_t LErrorI=0,RErrorI=0,LITerm=0,RITerm=0,LErrorD=0,RErrorD=0,eiThresh=5000;
int16_t lCountTarget,rCountTarget;
void sendEncoder(void);
void resetState(void);
void USARTWriteInt(int number);
void InitPWM(void);
void motorLeft(int i);
void motorRight(int i);
void setMotors(int l, int r);
void initUSART(uint16_t ubrr_value);
char USARTReadChar(void);
void USARTWriteChar(char data);
void USARTWriteInt16u(uint16_t number);
unsigned long millis(void);
void init_milli_timer(void);
void init_extint(void);


#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 


// the whole number of milliseconds per timer0 overflow
#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

/* Convert meters per second to ticks per time frame */
int16_t SpeedToTicks(float v) {
  //int result=(v * cpr / (PID_RATE * PI * wheelDiameter));
  int16_t result=v*cpr/PID_RATE;
  return result;
}

ISR(TIMER0_OVF_vect)
{
   // copy these to local variables so they can be stored in registers
   // (volatile variables must be read from memory on every access)
   unsigned long m = timer0_millis;
   unsigned char f = timer0_fract;

   m += MILLIS_INC;
   f += FRACT_INC;
   if (f >= FRACT_MAX) {
      f -= FRACT_MAX;
      m += 1;
   }

   timer0_fract = f;
   timer0_millis = m;
   timer0_overflow_count++;
}



int state_machine(char RByte)
{
	if (RByte==' ')
	{
		resetState();
		return 0;
	}
	if (RByte=='q')
	{
		resetState();
		return 0;
	}
	switch(state)
	{
		case 0:			
			switch(RByte)
			{
				case 'w':
					lDirOut=1;
					rDirOut=1;
					state=1;
					break;
				case 'a':
					lDirOut=-1;
					rDirOut=1;
					state=1;
					break;
				case 'd':
					lDirOut=1;
					rDirOut=-1;
					state=1;
					break;
				case 's':
					lDirOut=-1;
					rDirOut=-1;
					state=1;
					break;
					
				case 'k':
					lDirOut=-1;
					rDirOut=-1;
					state=5;
					break;
				default:
					state=0;
					lDirOut=1;
					rDirOut=1;
					break;
			}
			break;
		
		case 1:
			mildTarget_L=10*(RByte-48);
			state=2;
			break;
		
		case 2:
			mildTarget_L+=(RByte-48);
			state=3;
			break;
		
		case 3:
			mildTarget_R=10*(RByte-48);
			state=4;
			break;
		
		case 4:
			mildTarget_R+=(RByte-48);
			motion=1;
			
			if(PID==0) 
				setMotors((lDirOut*mildTarget_L*255/100),(rDirOut*mildTarget_R*255/100));
			
			//lCountTarget=(mildTarget_L * L_MAX_COUNT)/100;
			//rCountTarget=(mildTarget_R * R_MAX_COUNT)/100;
			 /* Convert speeds to encoder ticks per frame */
			lCountTarget = SpeedToTicks(lDirOut*mildTarget_L);
			rCountTarget = SpeedToTicks(rDirOut*mildTarget_R);
  
			LErrorI=0;RErrorI=0;LITerm=0;RITerm=0;LErrorD=0;RErrorD=0;
			lCount2=0;	rCount2=0;
			
			state=0;
			break;
		case 5:
			state=6;
			if(RByte=='p')
				pid_tuning_state=1;
			else if (RByte=='i')
				pid_tuning_state=2;
			else if (RByte=='d')
				pid_tuning_state=3;
			else if (RByte=='h')
				pid_tuning_state=4;
			else state=0;
			
			USARTWriteChar('\n');
			USARTWriteChar('\r');
			USARTWriteChar('P');
			USARTWriteInt(Kp);
			USARTWriteChar(' ');
			USARTWriteChar('I');
			USARTWriteInt(Ki);
			USARTWriteChar(' ');
			USARTWriteChar('D');
			USARTWriteInt(Kd);
			USARTWriteChar('H');
			USARTWriteChar('z');
			USARTWriteInt(PID_RATE);
			USARTWriteChar('#');	
			break;
		case 6:
			tuning_temp=0;
			tuning_temp+=100*(RByte-48);
			state=7;
			break;
		case 7:
			tuning_temp+=10*(RByte-48);
			state=8;
			break;
		case 8:
			tuning_temp+=(RByte-48);
			
			if(pid_tuning_state==1)
				Kp=tuning_temp;
			if(pid_tuning_state==2)
				Ki=tuning_temp;
			if(pid_tuning_state==3)
				Kd=tuning_temp;
			if(pid_tuning_state==4)
			{
				PID_RATE=tuning_temp;	
				PID_INTERVAL = 1000 / PID_RATE;
			}
			
			USARTWriteChar('\n');
			USARTWriteChar('\r');
			USARTWriteChar('P');
			USARTWriteInt(Kp);
			USARTWriteChar(' ');
			USARTWriteChar('I');
			USARTWriteInt(Ki);
			USARTWriteChar(' ');
			USARTWriteChar('D');
			USARTWriteInt(Kd);
			USARTWriteChar('H');
			USARTWriteChar('z');
			USARTWriteInt(PID_RATE);
			USARTWriteChar('#');
			
			state=0;
			break;
			
		default:
			state=0;
			break;
	}
	lastMotorCommand = 0;
	return 0;
}

/* PID routine to compute the next motor commands */
void PID_exec(void)
{
	//int LErrorI=0,RErrorI=0,LITerm=0,RITerm=0,LErrorD=0,RErrorD=0;
	lError =  lCountTarget  -(lCount);
	rError =  rCountTarget  -(rCount);
	

	lCountOut= ( (Ki * LErrorI) + (Kp * lError) + (Kd* (lError - lErrorPrevious)) )/Ko;
	rCountOut= ( (Ki * RErrorI) + (Kp * rError) + (Kd* (rError - rErrorPrevious))  )/Ko;
	
	//L_PWM=(lCountOut*255)/L_MAX_COUNT;
	//R_PWM=(rCountOut*255)/R_MAX_COUNT;
	L_PWM = lCountOut*PID_PWM_SCALE +L_PWM;
	R_PWM = rCountOut*PID_PWM_SCALE +R_PWM;
	
	//if(L_PWM < MOTOR_MIN_PWM)
	//	L_PWM = MOTOR_MIN_PWM;
	//else 
	if ( L_PWM > MOTOR_MAX_PWM)
		L_PWM = MOTOR_MAX_PWM;
		
	//if(R_PWM < MOTOR_MIN_PWM)
	//	R_PWM = MOTOR_MIN_PWM;
	//else
 if (R_PWM > MOTOR_MAX_PWM)
		R_PWM = MOTOR_MAX_PWM;
		
	if(motion==0)
	{
		L_PWM=0;
		R_PWM=0;
	}
	
	setMotors(L_PWM,R_PWM);
	
	USARTWriteChar('\n');
	USARTWriteChar('\r');
	
/*	
	
	USARTWriteChar('m');
	USARTWriteChar(':');
	USARTWriteChar('L');
	USARTWriteInt(lCount);
	USARTWriteChar('R');
	USARTWriteInt(rCount);
*/	
	USARTWriteChar(' ');
	USARTWriteChar('o');
	USARTWriteChar(':');
	USARTWriteChar('L');
	USARTWriteInt(lCountOut);
	USARTWriteChar('R');
	USARTWriteInt(rCountOut);
/*	
	USARTWriteChar(' ');
	USARTWriteChar('w');
	USARTWriteChar(':');
	USARTWriteChar('L');
	USARTWriteInt(L_PWM);
	USARTWriteChar('R');
	USARTWriteInt(R_PWM);
	
	USARTWriteChar(' ');
	USARTWriteChar('t');
	USARTWriteChar(':');
	USARTWriteChar('L');
	USARTWriteInt(lCountTarget);
	USARTWriteChar('R');
	USARTWriteInt(rCountTarget);
	
	USARTWriteChar(' ');
	USARTWriteChar('p');
	USARTWriteChar(':');
	USARTWriteChar('L');
	USARTWriteInt(lError);
	USARTWriteChar('R');
	USARTWriteInt(rError);
	
	USARTWriteChar(' ');
	USARTWriteChar('i');
	USARTWriteChar(':');
	USARTWriteChar('L');
	USARTWriteInt(LErrorI);
	USARTWriteChar('R');
	USARTWriteInt(RErrorI);
	
	USARTWriteChar(' ');
	USARTWriteChar('d');
	USARTWriteChar(':');
	USARTWriteChar('L');
	USARTWriteInt(lError - lErrorPrevious);
	USARTWriteChar('R');
	USARTWriteInt(rError - rErrorPrevious);
	
	USARTWriteChar(' ');
	USARTWriteChar('e');
	USARTWriteChar(' ');
	USARTWriteChar('L');
	USARTWriteInt(lCount2+lCount);
	USARTWriteChar('R');
	USARTWriteInt(rCount2+rCount);
*/
	
	
	
	lErrorPrevious=lError;
	rErrorPrevious=rError;
		
	LErrorI += lError;
	RErrorI += rError;
	
	if(LErrorI > eiThresh)LErrorI=eiThresh;
	else if(LErrorI <- eiThresh)LErrorI=-eiThresh;
	if(RErrorI > eiThresh)RErrorI=eiThresh;
	else if(RErrorI <- eiThresh)RErrorI=-eiThresh;
	
	lCount2=lCount2+lCount;
	rCount2=rCount2+rCount;
	lCount=0;rCount=0;
}

ISR(INT0_vect)
{
	//lCount++;
	if(L2)
		lCount--;//lDir=-1;
	else
		lCount++;//lDir=1;
}
ISR(INT1_vect)
{
	//rCount++;
	if(R2)
		rCount--;//rDir=-1;
	else
		rCount++;//rDir=1;
}
/*Usart Functions Definations*/

ISR(USART_RXC_vect) 
{ 
   char ReceivedByte; 
   ReceivedByte = UDR;// Fetch the received byte value into the variable "ByteReceived" 
   state_machine(ReceivedByte); 
   USARTWriteInt(state);
}

int main(void)
{
   /**PORT DDR Definations*/
   DDRA=0xFF;
   PORTA=0;
   DDRB=0;
   DDRC=0x0F;
   DDRD=0;
   
   /*main_local variable declarations*/
   
   /*Initialisation Modules*/
   
   
   
   USARTWriteChar('A');
   setMotors(0,0);
 
   //DeactivateAll();
   init_extint();
   initUSART(51);
   InitPWM();
   //init_timer();  
   PID_INTERVAL = 1000 / PID_RATE;
   /* Track the next time we make a PID calculation */
   unsigned long nextPID = PID_INTERVAL;
   init_milli_timer();
   
   while(1)
    {
	    if (tempTest == 0) 
        { 
            tempTest = 1000 + millis(); 
        }
		
		if( millis() > tempTest ) //lastMotorCommand == -1 && 
		{
			USARTWriteChar('.');
			USARTWriteInt(state);

			USARTWriteChar('c');
			USARTWriteInt(Gcount);
            Gcount=0;
			tempTest=0;
		}
		/*
        if (lastMotorCommand == 0) 
        { 
            lastMotorCommand = AUTO_STOP_INTERVAL + millis(); 
        } 
        
        if (millis() > lastMotorCommand && lastMotorCommand!=-1) { 
			sendEncoder();
			resetState();
			lastMotorCommand=-1;
        } 
		*/
		if (millis() > nextPID) {
			if(PID)
			{
				uint8_t oldSREG = SREG;
				cli();
				Gcount++;
				PID_exec();
				SREG=oldSREG;
			}
			nextPID += PID_INTERVAL;
		}
	}
}

void sendEncoder(void)
{
	uint8_t oldSREG = SREG;
	cli();
	
	USARTWriteChar('\n');
	USARTWriteChar('\r');
	USARTWriteChar('m');
	USARTWriteChar(' ');
	USARTWriteChar('L');
	USARTWriteInt16u(lCount2+lCount);
	USARTWriteChar('R');
	USARTWriteInt16u(rCount2+rCount);
	lCount2=0;
	rCount2=0;
	//lastMotorCommand = 0; 
	SREG = oldSREG;
}

/*The PWM init Function*/
void InitPWM(void)
{
	TCCR1A|=(1<<WGM10)|(1<<WGM12)|(1<<COM1A1)|(1<<COM1B1);
	TCCR1B|=(1<<CS11);//prescaler 8 instead of 64.this should increase the response rate.
	DDRD|=(1<<PD4)|(1<<PD5);
}
void motorLeft(int i)
{
	if(i<-MOTOR_MAX_PWM-1||i>MOTOR_MAX_PWM)
	{
		PORTC&=(~(1<<3));
		PORTC|=(1<<2);
		OCR1A=0;
	}
	else
	{
		if(i>0)
		{
			PORTC&=(~(1<<2));
			PORTC|=(1<<3);
			OCR1A=i;
		}
		else
		{
			PORTC&=(~(1<<3));
			PORTC|=(1<<2);
			OCR1A=-i;
		}
	}
}
void motorRight(int i)
{
	if (i < -MOTOR_MAX_PWM - 1 || i > MOTOR_MAX_PWM)
	{
		PORTC&=(~(1<<1));
		PORTC|=(1<<0);
		OCR1B=0;
	}
	else
	{
		if (i==0)
		{
			PORTC |= 3;//free run????
		}
		else
		{
			if (i>0)
			{
				PORTC&=(~(1<<0));
				PORTC|=(1<<1);
				OCR1B=i;
			}
			else
			{
				PORTC&=(~(1<<1));
				PORTC|=(1<<0);
				OCR1B=-i;
			}
		}
	}	
}
void setMotors(int l, int r) 
{
	if(motion==0)
	{
		l=0;
		r=0;
	}
	motorLeft(l);
	motorRight(r);
}
void initUSART(uint16_t ubrr_value)
{
  UBRRL = ubrr_value;
  UBRRH = (ubrr_value>>8);
  UCSRC=(1<<URSEL)|(3<<UCSZ0);
  UCSRB=(1<<RXEN)|(1<<TXEN);
  UCSRB |= (1 << RXCIE);
  sei();
}
char USARTReadChar(void)
{
  while(!(UCSRA & (1<<RXC)))
  {
  }
  return UDR;
}
void USARTWriteChar(char data)
{
  while(!(UCSRA & (1<<UDRE)))
  {
  }
  UDR=data;
}


void USARTWriteInt16u(uint16_t number)
{
	uint16_t i;
	uint16_t m=1;
	
	uint16_t digits=0;
	uint16_t zero=0;
	
	uint16_t number2=number;
	while(number2>zero)
	{
		number2/=10;
		digits++;
	}
	if(number==zero)
		digits=1;
	/*
	for(i=0;i<digits;i++)
	{
		m*=10;
	}
	while(m>1)
	{
		m/=10;
		USARTWriteChar((number/m)%10+48);
	}*/
	//USARTWriteChar(digits+48);
	//USARTWriteChar('\'');
	m=digits;
	while(m>0)
	{
		number2=number;
		for(i=1;i<m;i++)
			number2/=10;
		USARTWriteChar((number2)%10+48);
		m--;
	}
	for(;digits<=5;digits++)USARTWriteChar(' ');
}

void USARTWriteInt(int16_t number)
{
	int16_t i;
	int16_t m=1;
	int16_t digits=0;
	int16_t zero=0;
	if(number< zero)
	{
		USARTWriteChar('-');
		number *= -1;
	}else
	{
	    USARTWriteChar(' ');
	}
	int16_t number2=number;
	while(number2>zero)
	{
		number2/=10;
		digits++;
	}
	if(number==0)
		digits=1;
		
	//USARTWriteChar(digits+48);
	//USARTWriteChar('\'');
	
	//for(i=0;i<digits;i++)
	//{
	//	m*=10;
	//}
	m=digits;
	while(m>0)
	{
		number2=number;
		for(i=1;i<m;i++)
			number2/=10;
		USARTWriteChar((number2)%10+48);
		m--;
	}
	for(;digits<=5;digits++)USARTWriteChar(' ');
}

void resetState(void)
{
	setMotors(0,0);
	state=0;
	lDirOut=1;
	rDirOut=1;
	lCountTarget=0;
	rCountTarget=0;
	lastMotorCommand = 0;
	motion=0;
}

unsigned long millis(void)
{
   unsigned long m;
   uint8_t oldSREG = SREG;

   // disable interrupts while we read timer0_millis or we might get an
   // inconsistent value (e.g. in the middle of a write to timer0_millis)
   cli();
   m = timer0_millis;
   SREG = oldSREG;

   return m;
}


void init_milli_timer(void)
{
    // set timer 0 prescale factor to 64
#if defined(__AVR_ATmega128__)
	// CPU specific: different values for the ATmega128
	sbi(TCCR0, CS02);
#elif defined(TCCR0) && defined(CS01) && defined(CS00)
	// this combination is for the standard atmega8
	sbi(TCCR0, CS01);
	sbi(TCCR0, CS00);
#elif defined(TCCR0B) && defined(CS01) && defined(CS00)
	// this combination is for the standard 168/328/1280/2560
	sbi(TCCR0B, CS01);
	sbi(TCCR0B, CS00);
#elif defined(TCCR0A) && defined(CS01) && defined(CS00)
	// this combination is for the __AVR_ATmega645__ series
	sbi(TCCR0A, CS01);
	sbi(TCCR0A, CS00);
#else
	#error Timer 0 prescale factor 64 not set correctly
#endif

	// enable timer 0 overflow interrupt
#if defined(TIMSK) && defined(TOIE0)
	sbi(TIMSK, TOIE0);
#elif defined(TIMSK0) && defined(TOIE0)
	sbi(TIMSK0, TOIE0);
#else
	#error	Timer 0 overflow interrupt not set correctly
#endif
}
void DeactivateAll(void)
{
	TCCR0&=0;//timer disable
	GICR&=0;//External Interrupts Disable
	UCSRB=0;
	UCSRC=0;
}
/*External Interrupt and its ISRs*/
void init_extint(void)
{
	GICR|= (1<<INT0)|(1<<INT1);					// Enable INT0
	MCUCR|=(1<<ISC01)|(1<<ISC00)|(1<<ISC11)|(0<<ISC10);	// Trigger INT0 on rising edge
 	sei();				//Enable Global Interrupt
}


/*Timer and its ISR*/
/*
void init_timer(void)
{
  TCCR0=(1<<CS02)|(1<<WGM01);//prescaler 256;CTC mode;20 hz(50ms) sampling Freq
  OCR0=197;
  TIMSK|=(1<<OCIE0);
  sei();
}*/
/*
ISR(TIMER0_COMP_vect)
{
	uint8_t oldSREG = SREG;

   // disable interrupts while we read timer0_millis or we might get an
   // inconsistent value (e.g. in the middle of a write to timer0_millis)
    cli();
	if(PID)
	PID_exec();
	tempL[j]=lCount;
	tempR[j]=rCount;
	//USARTWriteChar('\n');
	//USARTWriteChar('\r');
	//USARTWriteInt((lCount*lDir));
	//USARTWriteChar(',');
	//USARTWriteInt((rCount*rDir));
	PORTA=!PORTA;
	lCount=0;rCount=0;j++;
	if(j>=150)
	{
		j=0;
		//USARTWriteChar('\n');
		//USARTWriteChar('\r');
			//USARTWriteInt(lDir);
			//USARTWriteChar(',');
			//USARTWriteInt(rDir);
		
	}
	SREG=oldSREG;
}
*/
