namespace IMUspace
{
	class IMU
	{
	public:
		static void initIMU();
		static void getYaw(double *);
		static void closeIMU();
	};
}