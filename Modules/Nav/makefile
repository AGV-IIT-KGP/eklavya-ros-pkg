SRC = ./seedGen2.cpp
TARG = seedGen

CC = g++

CPPFLAGS = -g -Wall -Wno-unused-function `pkg-config --cflags opencv`

LDFLAGS = `pkg-config --libs opencv -lm` `pkg-config --libs` 

OBJ = $(SRC:.cpp=.o)

seeds: $(TARG)

$(TARG):
	$(CC) $(CPPFLAGS)  $(SRC) -o $(TARG) $<
  
clean:
	rm -f *~ *.o $(TARG)
  
