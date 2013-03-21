/*
 * File:   LidarData.cpp
 * Author: bhuvnesh
 *
 * Created on 18 September, 2012, 6:56 PM
 */

#include "LidarData.h"
#include <mrpt/hwdrivers/CHokuyoURG.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/gui.h>
#include <mrpt/maps.h>
#include <opencv2/core/types_c.h>
#include "../../eklavya2.h"
#include <cvblob.h>

#define CENTERX 125
#define CENTERY 500
#define MAP_X 1000
#define MAP_Y 1000
#define HOKUYO_SCALE 100
#define VIEW_OBSTACLES 0
#define RADIUS 8
#define intensity(img,i,j,n) *(uchar*)(img->imageData + img->widthStep*i + j*img->nChannels + n) 
#define IMGDATA(image,i,j,k) (((uchar *)image->imageData)[(i)*(image->widthStep) + (j)*(image->nChannels) + (k)])
#define IMGDATAG(image,i,j) (((uchar *)image->imageData)[(i)*(image->widthStep) + (j)])
using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::slam;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace std;
using namespace cvb;


LidarData::LidarData(string serial_name) {
 
    //laser.setSerialPort(serial_name);
    //if (!laser.turnOn()) {
        //printf("[TEST] Initialization failed!\n");
        //return;
    //}
}

int checksum(char **localmap, int x, int y) {
    int threshold = 3;
    int win_size = 4;
    int sum = 0;
    for (int i = x - win_size; i < x + win_size; i++) {
        for (int j = y - win_size; j < y + win_size; j++) {
            if (i < 950 - RADIUS && i > RADIUS + 50 && j > RADIUS && j < 875 - RADIUS) {
                if (j > CENTERX + 5) {
                    if (localmap[i][j] == 1)
                        sum++;
                }
            }
        }
    }
    if (sum >= threshold)
        return 1;
    else
        return 0;

}

void LidarData::createCircle(int x, int y, int R) {
    //TODO: optimize using circle drawing alogrithm    

    for (int i = -RADIUS; i < RADIUS; i++) {
        for (int j = -RADIUS; j < RADIUS; j++) {
            if (i * i + j * j <= RADIUS * RADIUS) {
                global_map[x + i][y + j] = 255;
            }
        }
    }
}

void plotMap() {
    cvNamedWindow("Global Map", 0);
    IplImage *mapImg = cvCreateImage(cvSize(MAP_X, MAP_Y), IPL_DEPTH_8U, 1);

    for (int i = 0; i < MAP_X; i++) {
        uchar* ptr = (uchar *) (mapImg->imageData + i * mapImg->widthStep);
        for (int j = 0; j < MAP_Y; j++) {
            if (global_map[j][MAP_X - i - 1] == 0) {
                ptr[j] = 0;
            } else {
                ptr[j] = 255;
            }
        }
    }

    cvShowImage("Global Map", mapImg);
    cvWaitKey(1);
    //cvSaveImage("map.png", mapImg);
    cvReleaseImage(&mapImg);
}

void LidarData::update_map(const sensor_msgs::LaserScan& scan) {
  
  //initilize variables

  IplImage *filtered_img, *nblobs, *nblobs1, *labelImg;
  IplConvKernel *ker1, *ker2;

  ///TODO: put kernel initialization in constructor
  ker1 = cvCreateStructuringElementEx(9,9,4,4,CV_SHAPE_ELLIPSE);
  ker2 = cvCreateStructuringElementEx(7,7,3,3,CV_SHAPE_ELLIPSE);
  filtered_img = cvCreateImage(cvSize(MAP_X,MAP_Y),8,1);
  labelImg = cvCreateImage(cvSize(MAP_X,MAP_Y),IPL_DEPTH_LABEL,1);
  nblobs = cvCreateImage(cvSize(MAP_X,MAP_Y),8,3);
  nblobs1 = cvCreateImage(cvSize(MAP_X,MAP_Y),8,3);
  CvBlobs blobs;
  uchar * ptr;

  pthread_mutex_lock(&map_mutex);
  
///TODO: it has to be removed from here
  for (int i = 0; i < MAP_X; i++) {
    for (int j = 0; j < MAP_Y; j++) {
      global_map[i][j] = 0;
    }
  }
  //initilize variables ended

  // taking data from hokuyo node
  size_t size = scan.ranges.size();
  float angle = scan.angle_min;
  float maxRangeForContainer = scan.range_max - 0.1f;
  cvSet(filtered_img, cvScalar(0));

  for (size_t i = 0; i < size; ++i)
  {
    float dist = scan.ranges[i];
    if ( (dist > scan.range_min) && (dist < maxRangeForContainer))
    {
      double x1 = cos(angle) * dist;
      double y1 = sin(angle) * dist;
      int x = (int)((y1 * 200) + CENTERY);
      int y = (int)((x1 * 200) + CENTERX);
      
      if(x>=0 && y>=0 && (int)x<MAP_X && (int)y<MAP_Y) {
        ptr = (uchar *)(filtered_img->imageData + (MAP_X - (int)x - 1)*filtered_img->widthStep );
        ptr[(int)y] = 255;
      }
    }
    angle += scan.angle_increment;
  }
  
  //Filtering
  cvDilate(filtered_img,filtered_img,ker1,2);
  //cvErode(filtered_img,filtered_img,ker2,1);
  
  unsigned int result = cvLabel(filtered_img,labelImg,blobs);
  cvRenderBlobs(labelImg,blobs,nblobs,nblobs,CV_BLOB_RENDER_COLOR);
  cvFilterByArea(blobs,500,filtered_img->height*filtered_img->width);
  cvRenderBlobs(labelImg,blobs,nblobs1,nblobs1,CV_BLOB_RENDER_COLOR);
  //converts nblobs1 to filtered_img(grayscale)
  cvCvtColor(nblobs1,filtered_img,CV_RGB2GRAY);
  //thresholds the filtered_img based on threshold value
  cvThreshold(filtered_img,filtered_img,5,255,CV_THRESH_BINARY);
  cvReleaseImage(&labelImg);
  cvReleaseImage(&nblobs);
  cvReleaseImage(&nblobs1);
  cvReleaseBlobs(blobs);
  
  cvNamedWindow("Filtered Image",0);
  cvShowImage("Filtered Image",filtered_img);
  
  for(int i = 0;i<MAP_X;i++) {
    for(int j = 0;j<MAP_Y;j++) {
      global_map[i][j] = IMGDATA(filtered_img,i,j,0);

    }
  }
  cvReleaseImage(&filtered_img);
  cvReleaseStructuringElement(&ker1);
  cvReleaseStructuringElement(&ker2);
  
  pthread_mutex_unlock(&map_mutex);
}

LidarData::~LidarData() {
}

