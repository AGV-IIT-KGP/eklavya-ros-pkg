SRC = sonar_nav.cpp ../Utils/SerialPortLinux/serial_lnx.cpp

TARG = sonar_nav

CC = g++

CPPFLAGS = -g -W `pkg-config --cflags opencv`   -I../Utils/SerialPortLinux  #-Wall -Wno-unused-function

LDFLAGS = `pkg-config --libs opencv` 

OBJ = $(SRC:.cpp=.o)

all: $(TARG)

$(TARG):
	$(CC) $(CPPFLAGS)  $(SRC) -o $(TARG) $(LDFLAGS) $<
clean:
	rm -f *~ *.o $(TARG)
