#include <stdio.h>
#include <unistd.h>
#include "vectornav.h"
#include "math.h"
#include <signal.h>

/* Change the connection settings to your configuration. */
const char* const COM_PORT = "/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_FTUTUVO5-if00-port0";
const int BAUD_RATE = 115200;
Vn200 vn200;

void sigHandler(int sig) {
  signal(sig, SIG_IGN);

  vn200_disconnect(&vn200);
}

int main()
{
	double gpsTime;
	unsigned short gpsWeek, status;
	VnVector3 ypr, latitudeLognitudeAltitude, nedVelocity;
	float attitudeUncertainty, positionUncertainty, velocityUncertainty;
  double x,y,z,a,f,b,e2,Nphi,latitude,longitude,altitude;

	//signal(SIGINT, sigHandler);
  
	int i = 0;

	vn200_connect(&vn200, COM_PORT, BAUD_RATE);
  
  while(1) {
    printf("Iter: %d\n", i++);
    
		vn200_getInsSolution(
			&vn200,
			&gpsTime,
      &gpsWeek,
			&status,
			&ypr,
			&latitudeLognitudeAltitude,
			&nedVelocity,
			&attitudeUncertainty,
			&positionUncertainty,
			&velocityUncertainty);
                
    latitude = latitudeLognitudeAltitude.c0*PI/180;
    longitude =latitudeLognitudeAltitude.c1*PI/180;
    altitude  = latitudeLognitudeAltitude.c2;
    
    // WGS84 parameters.
    a = 6378137; f = 1/298.257223563; 
    b = a*(1 - f); 
    e2 = 1 - pow((b/a),2);

    // Conversion from:
    // en.wikipedia.org/wiki/Geodetic_system#Conversion_calculations
    Nphi = a/sqrt(1 - pow(e2*sin(latitude),2));
    x = (Nphi + altitude)*cos(latitude)*cos(longitude);
    y = (Nphi + altitude)*cos(latitude)*sin(longitude);
    z = (Nphi*(1 - e2) + altitude)*sin(latitude);

		printf("INS Solution:\n"
			"  GPS Time:               %f\n"
			"  GPS Week:               %u\n"
			"  INS Status:             %.4X\n"
			"  YPR.Yaw:                %+#7.2f\n"
			"  YPR.Pitch:              %+#7.2f\n"
			"  YPR.Roll:               %+#7.2f\n"
			"  LLA.Lattitude:          %+#7.8f\n"
			"  LLA.Longitude:          %+#7.8f\n"
			"  LLA.Altitude:           %+#7.8f\n"
			"  Velocity.North:         %+#7.2f\n"
			"  Velocity.East:          %+#7.2f\n"
			"  Velocity.Down:          %+#7.2f\n"
			"  Attitude Uncertainty:   %+#7.2f\n"
			"  Position Uncertainty:   %+#7.2f\n"
			"  Velocity Uncertainty:   %+#7.2f\n"
      "  Cartesian X:            %+#lf\n"
      "  Cartesian Y:            %+#lf\n"
      "  Cartesian Z:            %+#lf\n",
			gpsTime,
			gpsWeek,
			status,
			ypr.c0,
			ypr.c1,
			ypr.c2,
			latitudeLognitudeAltitude.c0,
			latitudeLognitudeAltitude.c1,
			latitudeLognitudeAltitude.c2,
			nedVelocity.c0,
			nedVelocity.c1,
			nedVelocity.c2,
			attitudeUncertainty,
			positionUncertainty,
			velocityUncertainty,
      x,
      y,
      z);

		printf("\n\n");

		sleep(1);
	}
	
	vn200_disconnect(&vn200);

	return 0;
}

