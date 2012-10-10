#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "serial_lnx.h"
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

    char data_in[10];
    int flag=1,i;
    char inp;
    int linefeed_count=0;
    //p->sendArray("w5050",5);
    while(flag)
    {
        if(p->bytesToRead()>0)
        {   data_in[0]=p->getChar();
            if(data_in[0] != '\0')
            {
                if((linefeed_count==0) && (data_in[0]=='\n'))
                {
                    linefeed_count=1;
                    printf("%c",data_in[0]);
                }
                else if(data_in[0]!='\n')
                {
                    linefeed_count=0;
                    printf("%c",data_in[0]);
                }
            }
            data_in[0] = '\0'; //reset and wait for next char
        }

        if(kbhit())
        {
            inp =getchar();
            printf("\nSending.. '%c' ascii %d !\n", inp ,(int)inp );

            if(inp ==ESCAPE)flag=0;
            p->sendChar(inp);
            if(SONAR>0 && inp=='s')//SONAR
            {
                char a[22];
                //while(p->bytesToRead()==0)printf(".");
                a[0]='\0';
                p->getArray(a,6);
                printf("\n%s",a);
            }
            if((AUTOSTOP >0) && ((inp =='w') ||( inp =='s') || (inp =='d') || (inp =='a') ))
            {
                i=0;
                while((i<4))
                {
                    while(!kbhit());//printf(".");
                    i++;
                    inp=getchar();
                    printf("Sending.. '%c' ascii %d !\n", inp,(int)inp);
                    p->sendChar(inp);
                }
                //struct timespec t = { 3/*seconds*/, 0/*nanoseconds*/};
                // nanosleep(&t,NULL);
                sleep(AUTOSTOP);
                inp=32;
                p->sendChar(inp);
            }
        }
        usleep(20000);
    }
    p->disconnect();
    return 0;
}


int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}


