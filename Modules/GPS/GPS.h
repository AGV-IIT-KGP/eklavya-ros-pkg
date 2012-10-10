namespace GPSspace
{
	class GPS
	{
	public:
		/*static void initGPS(int, int, double, double);
		static void fetchTarget(int *, int *);
		static void closeGPS();*/
		static void GPS_Init();
		static void _GPS(double *north, double *east);
		static void Local_Map_Coordinate(double north_d,double east_d,double yaw_d, int* tx, int *ty);
	};
}