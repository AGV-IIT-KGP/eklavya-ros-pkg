
#sSRC = AGV.cpp ./Modules/Nav/PathPlanner.cpp ./Utils/SerialPortLinux/serial_lnx.cpp  ./Modules/Lane/Lane.cpp
#SRC = AGV.cpp ./Modules/Nav/PathPlannerMod.cpp ./Utils/SerialPortLinux/serial_lnx.cpp  ./Modules/Lane/Lane.cpp ./Modules/GPS/GPS.cpp
#SRC = AGV.cpp ./Modules/Nav/PathPlannerMod.cpp ./Utils/SerialPortLinux/serial_lnx.cpp  ./Modules/Lane/Lane.cpp
SRC = AGV.cpp ./Modules/Lane/Lane.cpp ./Modules/IMU/IMU.cpp ./Modules/Nav/PathPlannerMod.cpp ./Utils/SerialPortLinux/serial_lnx.cpp ./Modules/Lidar/LidarData.cpp
#SRC = ./Utils/SerialPortLinux/serial_lnx.cpp ./Utils/VehicleTest/vehicle.cpp
#SRC = AGV.cpp ./Modules/IMU/IMU.cpp ./Modules/Nav/PathPlannerMod.cpp ./Utils/SerialPortLinux/serial_lnx.cpp
TARG = eklavya

CC = g++

#CPPFLAGS = -g -Wall -Wno-unused-function `pkg-config --cflags opencv` -I./Nav -I./Lane -I./GPS -I./IMU -I./Stereo  -I./Utils/SerialPortLinux
CPPFLAGS = -g -w `pkg-config --cflags opencv mrpt-slam mrpt-vision mrpt-hwdrivers` -I./Utils/SerialPortLinux -I./Modules/GPS -I./Modules/IMU -I./Modules/Nav -I./Modules/Lane -I./Modules/Lidar

# -Wall -Wno-unused-function
#LDFLAGS = `pkg-config --libs opencv` 
#CPPFLAGS = -g -Wall -Wno-unused-function `pkg-config --cflags opencv` -I./Nav -I./Lane -I./GPS -I./IMU -I./Stereo
LDFLAGS = `pkg-config --libs opencv` `pkg-config --libs mrpt-base mrpt-hwdrivers mrpt-slam` 
#LDFLAGS = `pkg-config --libs opencv` `pkg-config --libs` 

OBJ = $(SRC:.cpp=.o)

all: $(TARG)

$(TARG):
	$(CC) $(CPPFLAGS)  $(SRC) -o $(TARG) $(LDFLAGS) $<
clean:
	rm -f *~ *.o $(TARG)




#% : %.cpp
#	g++ $(CFLAGS) $(LIBS) -o $@ $<

#g++ `pkg-config --cflags opencv` main.cpp -o mouse_kalman `pkg-config --libs opencv`

#g++ -g -Wall -Wno-unused-function `pkg-config --cflags opencv` `pkg-config --libs opencv` main.cpp -o mouse_kalman

