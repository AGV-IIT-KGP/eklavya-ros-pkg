#ifndef FLAG_DATA_H
#define	FLAG_DATA_H

#include "../../eklavya2.h"
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <stdexcept>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core_c.h>
#include <sensor_msgs/Image.h>


using namespace std;
#define IMGDATA(image,i,j,k) *((uchar *)&image->imageData[(i)*(image->widthStep) + (j)*(image->nChannels) + (k)])
#define Hred1 0 
#define Hred2 180
#define Hblue 120
//#define Sclr 100
//#define Vclr 100
#define Htol 10
#define Stol 150
#define Vtol 150
#define mtz(a) ((a)>0?(a):0)//mtz==more than zero HSV values
#define ltt(a) ((a)<255?(a):255)//ltt== less than two fifty five
#define MAXSIZE_BLOB 100
#define MINSIZE_BLOB 9
#define MAX_HOR_DIST 200

namespace flag_space
{
	class FlagDetection {
	public:
    	void markFlag(const sensor_msgs::ImageConstPtr& image) ;
	
	};
}

#endif
