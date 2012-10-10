#include <stdlib.h>
#include <stdio.h>
#include "../SerialPortLinux/serial_lnx.h"
#include <stdexcept>
#include "../../Modules/devices.h"

#define UART_COMM_PORT BOT_PATH
#define UART_BAUD_RATE 19200

Tserial *p;

void Init()
{
  p = new Tserial();
  p->connect(UART_COMM_PORT, UART_BAUD_RATE, spNONE);
  usleep(100);

  p->sendChar('w');
  usleep(100);
}

void Turn(int l, int r)
{
  p->sendChar('w');
  usleep(100);
  
  p->sendChar('0'+ l/10);
  usleep(100);
  p->sendChar('0'+ l%10);
  usleep(100);
  p->sendChar('0'+ r/10);
  usleep(100);
  p->sendChar('0'+ r%10);
  usleep(100);
}

void Terminate()
{
  p->sendChar(' ');
  usleep(100);
  
  p->disconnect();
  usleep(100);
}

int main()
{
  int i;
  Init();

  Turn(40, 20);

  p->disconnect();
  usleep(5000 * 1000);
  
  p->connect(UART_COMM_PORT, UART_BAUD_RATE, spNONE);
  usleep(100);

  Terminate();
  
  return 0;
}
