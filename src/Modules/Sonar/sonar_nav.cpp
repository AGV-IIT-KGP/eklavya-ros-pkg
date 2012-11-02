#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "serial_lnx.h"`
#define UART_COMM_PORT_SONAR 	"/dev/ttyUSB2"
#define UART_BAUD_RATE_SONAR  19200
#define UART_COMM_PORT_MOTOR 	"/dev/ttyUSB2"
#define UART_BAUD_RATE_MOTOR   19200
#define MIN_DIST   50
#define SIDE_DIST  35
#define DISPLAY 1
#define FWD 	0
#define LEFT 	1
#define RIGHT 	2
#define STOP 	3
#define TXD     0
struct SonarDist
{
    unsigned int frontA;
    unsigned int frontB;
    unsigned int frontC;
    unsigned int left;
    unsigned int right;
};


/* restore original terminal settings on exit */
int     pfda;
static struct termios pots;
void cleanup_termios(int signal)
{
    tcsetattr(pfda, TCSANOW, &pots);
    exit(0);
}


void DispSonarData(struct SonarDist *S)
{
    printf("Front Distance = %u cm\n",S->frontA);
    printf("Front Distance = %u cm\n",S->frontB);
    printf("Front Distance = %u cm\n",S->frontB);
    printf("Right Distance = %u cm\n",S->right);
    printf("Left Distance  = %u cm\n",S->left);
}

void getSonarData(struct SonarDist *S)
{
   
    Tserial* p;
    long count=0;
    p = new Tserial();
    int t = p->connect(UART_COMM_PORT_SONAR, UART_BAUD_RATE_SONAR, spNONE,true);
    if(t<0)
    {
    exit(0);
    //printf("debug= %d ",t);
    }
    p->sendChar('s');
    
   if(TXD)
   {
    while(p->bytesToRead()<5)//waiting for full data to be avaible with count to avoid infinite loop
    {usleep(100);count++; if(count>10000)return;}
    
    S->frontA 	= p->getChar();
    S->frontA 	= p->getChar();
    S->frontA 	= p->getChar();
    S->left 	= p->getChar();
    S->right 	= p->getChar();
    }
    else
    {
    
    printf ("inside while");
    
    while(p->bytesToRead()<34)//waiting for full data to be avaible with count to avoid infinite loop
    {
    usleep(100);
    count++;
    if(count>10000)
    //exit(0);
    return;
    }
    
    while(p->bytesToRead()>0)
    {
    printf("char =%c/n",p->getChar());
    }
    }
    


    p->disconnect();
}

int fsm(struct SonarDist *S)
{
	int state;
	struct SonarDist sonar = *S;
	if(sonar.frontA<MIN_DIST || sonar.frontB<MIN_DIST || sonar.frontC<MIN_DIST)
	{		if(sonar.frontB>10 && sonar.frontB>10 && sonar.frontC>10)
			{
				if(sonar.frontC < sonar.frontB)
				{
					if(sonar.left>SIDE_DIST) 
					state = LEFT;
					else if(sonar.right > SIDE_DIST)
							state = RIGHT;
							else
							state =STOP;
				}
				else 
					{
						if(sonar.right > SIDE_DIST)
							state =RIGHT;
						else if(sonar.left > SIDE_DIST)
								state = LEFT;
							else
								state = STOP;
					}
			}
			else
			state = STOP;
	}
	else
	state = FWD;
}
/*
int getState(struct SonarDist *S)
{
    int state;
    struct SonarDist sonar=*S;
    if	   (sonar.front>MIN_DIST && sonar.left>MIN_DIST && sonar.right>MIN_DIST)                 //All clear
        state=0;
    else if(sonar.front>MIN_DIST && sonar.left>MIN_DIST && sonar.right<MIN_DIST)            //F L !R
        state=1;
    else if(sonar.front>MIN_DIST && sonar.left<MIN_DIST && sonar.right>MIN_DIST)            //F !L R
        state=2;
    else if(sonar.front>MIN_DIST && sonar.left<MIN_DIST && sonar.right<MIN_DIST)            //F !L !R
        state=3;
    else if(sonar.front<MIN_DIST && sonar.left>MIN_DIST && sonar.right>MIN_DIST)            // !F L R
        state=4;
    else if(sonar.front<MIN_DIST && sonar.left>MIN_DIST && sonar.right<MIN_DIST)            // !F L !R
        state=5;
    else if(sonar.front<MIN_DIST && sonar.left<MIN_DIST && sonar.right>MIN_DIST)            // !F !L R
        state=6;
    else if(sonar.front<MIN_DIST && sonar.left<MIN_DIST && sonar.right<MIN_DIST)            // !F !L !R
			state=7;
    return state;
}
*/
void motorCmd(char dir,unsigned int l, unsigned int r)
{
    Tserial* p;
    p->connect(UART_COMM_PORT_MOTOR, UART_BAUD_RATE_MOTOR, spNONE,false);
    sleep(10);
    p->sendChar(dir);
    sleep(10);
    p->sendChar('0'+ l/10);
    sleep(10);
    p->sendChar('0'+ l%10);
    sleep(10);
    p->sendChar('0'+ r/10);
    sleep(10);
    p->sendChar('0'+ r%10);
    p->disconnect();
}
int output(int state)
{
	  switch(state)
            {
            case FWD:                        //All Clear
                //motorCmd('w',20,20);
                if(DISPLAY)
                    printf("Moving straight...\n");
                break;

            case LEFT:                         //Right Obstacle: Front & Left Clear
                //motorCmd('a',20,20);         //Turn Left
                if(DISPLAY)
                    printf("Turning Left...\n");
                break;

            case RIGHT:                         //Left Obstacle: Front & Right Clear
                //motorCmd('d',20,20);         //Turn Right
                if(DISPLAY)
                    printf("Turning Right...\n");
                break;

            case STOP:                         //Left and Right Obsatcle: Front Clear
                //motorCmd(' ',0,0);	    //Stop
                if(DISPLAY)
                    printf("Stopped...\n");
                break;
			default:
				//motorCmd(' ',0,0);	    //Stop
                if(DISPLAY)
                    printf("Stopped...\n");
                break;
            }
}
int main()
{
	
    struct SonarDist sonar;
    sonar.frontA=sonar.frontB=sonar.frontC=sonar.left=sonar.right=0;
    unsigned int state;
    if(DISPLAY)
        printf("SONAR NAVIGATION\n");
    while(1)
    {
         printf("debug ");
        getSonarData(&sonar);
        state = fsm(&sonar);
        if(DISPLAY)
        DispSonarData(&sonar);
		output(state);
        // sleep(1);
        //clrscr();
    }

}

