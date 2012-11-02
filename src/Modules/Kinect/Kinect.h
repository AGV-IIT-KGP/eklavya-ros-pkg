#include <mrpt/hwdrivers.h>
#include <mrpt/gui.h>
#include <mrpt/maps.h>
#include <mrpt/system/filesystem.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace std;

namespace Kinect
{
struct TThreadParam
{
	TThreadParam() : quit(false), pushed_key(0), tilt_ang_deg(0), Hz(0) { }

	volatile bool   quit;
	volatile int    pushed_key;
	volatile double tilt_ang_deg;
	volatile double Hz;

	mrpt::synch::CThreadSafeVariable<CObservation3DRangeScanPtr> new_obs;     // RGB+D (+3D points)
	mrpt::synch::CThreadSafeVariable<CObservationIMUPtr>         new_obs_imu; // Accelerometers
};

class KinectData // Defines object that has args required by kinect
{
public:
	opengl::COpenGLViewportPtr viewRange, viewInt; // Extra viewports for the RGB & D images.
	mrpt::opengl::CPointCloudColouredPtr gl_points; //OpenGL point cloud
	mrpt::system::TThreadHandle thHandle; // Handle to created thread in which kinect data is read
	TThreadParam thrPar; //Thread param
	CObservation3DRangeScanPtr  last_obs; // to check if new obstacle, or simply prev one
	CObservationIMUPtr          last_obs_imu;
	float zCut;
	CColouredPointsMap pntsMap; //This is the 3D Point map required for obstacle detection. No thread safety required.
	KinectData()
	{
		zCut = 0;
	}
};

void thread_grabbing(TThreadParam &p);
void initKinect(KinectData &kD, mrpt::gui::CDisplayWindow3D &win3D);
int runKinect(KinectData &kD, mrpt::gui::CDisplayWindow3D &win3D);
};
