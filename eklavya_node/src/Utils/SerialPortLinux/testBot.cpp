#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "serial_lnx.h"
#include <time.h>
#define UART_COMM_PORT 	"/dev/ttyUSB0"
#define UART_BAUD_RATE  19200
#define Sleep(i) usleep(i)

#define SONAR 1
#define ESCAPE 27
//#define VERBOSE true
#define VERBOSE false
#define AUTOSTOP 0 // 0 for no autostop , 3 for autostop in 3 seconds.. or any integer.
int kbhit(void);
int main()
{
    Tserial* p;
    p = new Tserial();
    p->connect(UART_COMM_PORT, UART_BAUD_RATE, spNONE,VERBOSE);

    char array[2] = {' ', ' '};
    p->sendArray(array, 2);
    usleep(100);
    p->disconnect();
    
    srand(time(0));
    
    for(int i = 0; i < 100; i++) {      
      p->connect(UART_COMM_PORT, UART_BAUD_RATE, spNONE,VERBOSE);
      char array1[5] = {'w', '3', 
                             '0', 
                             '3', 
                             '0'};
    
      p->sendArray(array1, 5);
      usleep(100);
      p->disconnect();
    
      usleep(100 * 1000);
    }
    
    p->connect(UART_COMM_PORT, UART_BAUD_RATE, spNONE,VERBOSE);

    p->sendChar(' ');
    usleep(100);
    p->disconnect();
}    
