#include <cv.h>
#include <highgui.h>
#include <cxcore.h>
#include <cvaux.h>
#include "Lane.h"
#include <stdexcept>
#define N 7 // canny kernel 

#define SCALE_X 10

#define MAP_MAX 1000


IplImage *kernel_frame;
IplImage *edge_frame;
IplImage *hough_frame;
IplImage *gray_hough_frame;
IplImage *gray_frame;
IplImage *lane;
IplImage *warp_img;
CvPoint offset;
CvPoint2D32f srcQuad[4], dstQuad[4];
CvMat* warp_matrix = cvCreateMat(3,3,CV_32FC1);

int canny_kernel=7, high_threshold=500,low_threshold=300,vote=25,length=25,merge=5,maxvalue_R=255,maxvalue_G=255,maxvalue_B=255,minvalue_R=210,minvalue_G=210,minvalue_B=210;


namespace Lanespace
{
  IplImage* LaneDetection::colorBasedLaneDetection(IplImage *frame,int maxvalue_B,int maxvalue_G,int maxvalue_R,int minvalue_B,int minvalue_G,int minvalue_R,int vote,int length,int merge)
  {
    uchar* frame_data;
    uchar* frame_out_data;
    int i,j,index,index_out;
    IplImage *frame_out = cvCreateImage(cvGetSize(frame), frame->depth, 1);
    frame_out_data = (uchar*)frame_out->imageData;
    frame_data = (uchar*)frame->imageData;
      for(i=0;i<frame->height;i++)
      {
        for(j=0;j<frame->width;j++)
        {
          index = (i*frame->widthStep)+(j*frame->nChannels);
          index_out = (i*frame_out->widthStep) + j;
          if(     (frame_data[index+0]<=maxvalue_B) && (frame_data[index+0]>=minvalue_B) &&
              (frame_data[index+1]<=maxvalue_G) && (frame_data[index+1]>=minvalue_G) &&
              (frame_data[index+2]<=maxvalue_R) && (frame_data[index+2]>=minvalue_R)
            )
            {
              frame_out_data[index_out]=255;
            }
            else
            {
              frame_out_data[index_out]=0;
            }
        }
      }
      cvDilate(frame_out, frame_out, 0, 2);
      return frame_out;
  }
  
  void LaneDetection::initImage(IplImage* img)
  {
    int i,j;
    uchar* img_data = (uchar*)img->imageData;
    for(i=0;i<img->height;i++)
    {
      for(j=0; j<img->width; j++)
      {
        img_data[(i*img->widthStep)+(j*img->nChannels)]=0;
        img_data[(i*img->widthStep)+(j*img->nChannels)+1]=0;
        img_data[(i*img->widthStep)+(j*img->nChannels)+2]=0;
      }
    }
  }

  IplImage* LaneDetection::applyHoughTransform(IplImage* img, int vote, int length, int merge)
  {
    CvSeq* lines;
    CvMemStorage* storage = cvCreateMemStorage(0);
    IplImage* dst = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
    LaneDetection::initImage(dst);
    int i;
    lines = cvHoughLines2( img,storage,CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, vote, length, merge);
    int n = lines->total;
    for( i = 0; i < n; i++ )
    {
      CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
      cvLine( dst, line[0], line[1], CV_RGB(255,255,255), 15, 8 );
    }
    return dst;
  }

  CvSeq* GetHoughLanes(IplImage* img, int vote, int length, int merge)
  {
    CvSeq* lines;
    CvMemStorage* storage = cvCreateMemStorage(0);
    lines = cvHoughLines2( img,storage,CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, vote, length, merge);

    return lines;
  }

  void LaneDetection::initializeLaneVariables(IplImage *input_frame)
  {
    offset = cvPoint((N-1)/2,(N-1)/2);
    gray_frame = cvCreateImage(cvGetSize(input_frame), input_frame->depth, 1);
    kernel_frame = cvCreateImage( cvSize( gray_frame->width+N-1, gray_frame->height+N-1), gray_frame->depth, gray_frame->nChannels);
    edge_frame = cvCreateImage( cvGetSize(kernel_frame), IPL_DEPTH_8U, kernel_frame->nChannels );
    hough_frame = cvCreateImage( cvGetSize(kernel_frame), IPL_DEPTH_8U, kernel_frame->nChannels );
    gray_hough_frame = cvCreateImage( cvGetSize(hough_frame), IPL_DEPTH_8U, 1);

    cvNamedWindow("Control Box",1);
    cvCreateTrackbar("B Max", "Control Box", &maxvalue_B, 255, NULL);
    cvCreateTrackbar("G Max", "Control Box", &maxvalue_G, 255, NULL);
    cvCreateTrackbar("R Max", "Control Box", &maxvalue_R, 255, NULL);
    cvCreateTrackbar("B Min", "Control Box", &minvalue_B, 255, NULL);
    cvCreateTrackbar("G Min", "Control Box", &minvalue_G, 255, NULL);
    cvCreateTrackbar("R Min", "Control Box", &minvalue_R, 255, NULL);
    cvCreateTrackbar("Vote", "Control Box", &vote, 50, NULL);
    cvCreateTrackbar("Length", "Control Box", &length, 100, NULL);
    cvCreateTrackbar("Merge", "Control Box", &merge, 15, NULL);
    cvCreateTrackbar("Canny Kernel", "Control Box", &canny_kernel, 10, NULL);
    cvCreateTrackbar("High Canny Threshold", "Control Box", &high_threshold, 1000, NULL);
    cvCreateTrackbar("Low Canny Threshold", "Control Box", &low_threshold, 500, NULL);

      float Z=1;
    dstQuad[0].x = (float)200; //src Top left
    dstQuad[0].y = (float)37;
    dstQuad[1].x = (float)380; //src Top right
    dstQuad[1].y = (float)37;
    dstQuad[2].x = (float)40;  //src Bottom left
    dstQuad[2].y = (float)220;
    dstQuad[3].x = (float)520; //src Bot right
    dstQuad[3].y = (float)220;

    int lOff = 107, tOff = 265;
    srcQuad[0].x = (float)tOff; //dst Top left
    srcQuad[0].y = (float)lOff;
    srcQuad[1].x = (float)(640-tOff); //dst Top right
    srcQuad[1].y = (float)lOff;
    srcQuad[2].x = (float)tOff; //dst Bottom left
    srcQuad[2].y = (float)(480-lOff);
    srcQuad[3].x = (float)(640-tOff); //dst Bot right
    srcQuad[3].y = (float)(480-lOff);

    cvGetPerspectiveTransform(srcQuad, dstQuad, warp_matrix);
  
  }

  IplImage *getLaneLines(IplImage* src)
  {
    IplImage* dst = cvCreateImage( cvGetSize(src), IPL_DEPTH_8U, 1 );
    IplImage* color_dst = cvCreateImage( cvGetSize(src), IPL_DEPTH_8U, 3 );
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* lines = 0;
    int i;
    cvCanny( src, dst, 50, 200, 3 );
    
    color_dst = cvCreateImage(cvGetSize(dst),dst->depth,3);
    LaneDetection::initImage(color_dst);
    //cvCvtColor( dst, color_dst, CV_GRAY2BGR );
    
    lines = cvHoughLines2(dst, storage, CV_HOUGH_STANDARD, 1, CV_PI/180, 60, 0, 0 );

    for( i = 0; i < MIN(lines->total,100); i++ )
    {
      float* line = (float*)cvGetSeqElem(lines,i);
      float rho = line[0];
      float theta = line[1];
      CvPoint pt1, pt2;
      //if( !((10*theta>=24 && 10*theta<=30)||(100*theta>=50 && 100*theta<=70)) )
      //{
        //printf("%.2f \n",theta);
      
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        cvLine( color_dst, pt1, pt2, CV_RGB(255,255,255), 3, 8 );
      //}
    }

    cvCvtColor(color_dst, dst, CV_BGR2GRAY);
    return dst;
  }

  void markPixel(IplImage *img, int r, int c)
  {
    int i,j;
    uchar *data = (uchar*)img->imageData;
    for(i=-2;i<0;i++)
    {
      for(j=-2;j<2;j++)
      {
        data[(r+i)*img->widthStep + (c+j)*img->nChannels + 0] = 255;
        data[(r+i)*img->widthStep + (c+j)*img->nChannels + 1] = 255;
        data[(r+i)*img->widthStep + (c+j)*img->nChannels + 2] = 255;
      }
    }
  }

  void getVehiclePosition(IplImage *img)
  {
    float dl = -1.0, dr = -1.0;
    int vx,vy;
    int i,j;
    int flag;
    flag = 1;
    vy = img->height;
    vx = img->width/2;
    //markPixel(img, img->height, img->width/2);
    uchar *data = (uchar*)img->imageData;
    for(i=vy; i>0 && flag==1; i--)
    {
      for(j=vx-5; j>0 && flag==1; j--)
      {
        if(data[i*img->widthStep + j*img->nChannels + 0] > 0)
        {
          dl = (vx - j) * SCALE_X;
          //printf("dl :%.2f row : %d \n", dl, vy - i);
          flag = 0;
          break;  
        }
      }
    }
    flag =1;
    for(i=vy; i>0 && flag==1; i--)
    {
      for(j=vx+5; j<vx*2 && flag==1; j++)
      {
        if(data[i*img->widthStep + j*img->nChannels + 0] > 0)
        {
          dr = (j- vx) * float(SCALE_X);
          //printf("dr :%.2f row : %d \n", dr, vy - i);
          flag = 0;
          break;  
        }
      }
    }
    //printf("----------------\n");
  }

  int getDestinationPosition(IplImage *img)
  {
    float destl = -1.0, destr = -1.0;
    int vx,vy;
    int i,j;
    int flag;
    flag = 1;
    vy = img->height;
    vx = img->width/2;
    //markPixel(img, img->height, img->width/2);
    uchar *data = (uchar*)img->imageData;
    for(i=0; i<vy && flag==1; i++)
    {
      for(j=0; j<vx*2 && flag==1; j++)
      {
        if(data[i*img->widthStep + j*img->nChannels + 0] > 0)
        {
          destl = (j) * float(SCALE_X);
          flag = 0;
          break;  
        }
      }
    }
    flag =1;
    for(i=0; i<vy && flag==1; i++)
    {
      for(j=vx*2; j>(destl/SCALE_X) && flag==1; j--)
      {
        if(data[i*img->widthStep + j*img->nChannels + 0] > 0)
        {
          destr = (vx*2 - j) * float(SCALE_X);
          flag = 0;
          break;  
        }
      }
    }
    int dest = (int)((destl+destr)/2);
    for(i=0;i<2;i++)
    {
      for(j=-2;j<2;j++)
      {
        data[(i)*img->widthStep + (dest+j)*img->nChannels + 0] = 255;
        data[(i)*img->widthStep + (dest+j)*img->nChannels + 1] = 0;
        data[(i)*img->widthStep + (dest+j)*img->nChannels + 2] = 0;
      }
    }
    return dest;
  }

  int min(int value)
  {
    if(value > 999)
      return 999;
    if(value < 0)
      return 0;
  }

  void populateLanes(IplImage *img, char **map, int scale)
  {
    int i,j,index,k;      
    for(i=0;i<img->height;i++)
    {
      uchar *data = (uchar *)(img->imageData + i*img->widthStep);
      for(j=0;j<img->width;j++)
      {
          for(k=0;k<scale;k++)
        map[(int)(min((0.5*MAP_MAX) + (scale * (j-img->width/2))+ k))][(int)(img->height-(i)+(0.1*MAP_MAX))] = (data[j] == 255)? 255 : 0;
      }
    }
  }

  void LaneDetection::markLane(IplImage *input_frame, char** map, int choice, int scale)
  {
    lane =  LaneDetection::colorBasedLaneDetection(input_frame,maxvalue_B,maxvalue_G,maxvalue_R,minvalue_B,minvalue_G,minvalue_R,vote,length,merge);
    if(choice == 1)
    {
      lane =  LaneDetection::colorBasedLaneDetection(input_frame,maxvalue_B,maxvalue_G,maxvalue_R,minvalue_B,minvalue_G,minvalue_R,vote,length,merge);
    }
    if((choice == 2) || (choice == 0))
    {
      cvCvtColor(input_frame, gray_frame, CV_BGR2GRAY); // converting image to gray scale

      /* Todo : enhancing the image contrast
        good for better edge detection
        histogram equilization is one such naive(but good) technique
      */
      cvEqualizeHist( gray_frame, gray_frame);
      
      // canny edge detection
      cvCopyMakeBorder(gray_frame, kernel_frame, offset, IPL_BORDER_REPLICATE, cvScalarAll(0));
      cvCanny( kernel_frame, edge_frame, low_threshold*canny_kernel*canny_kernel, high_threshold*canny_kernel*canny_kernel, canny_kernel );
      hough_frame = LaneDetection::applyHoughTransform(edge_frame, vote, length, merge);
    
      cvCvtColor(hough_frame, gray_hough_frame, CV_BGR2GRAY); // converting image to gray scale
    }
    if(choice==0)
    {
      lane = gray_hough_frame;
    }
    if(choice==2)
    {
      lane = joinResult(LaneDetection::colorBasedLaneDetection(input_frame,maxvalue_B,maxvalue_G,maxvalue_R,minvalue_B,minvalue_G,minvalue_R,vote,length,merge),gray_hough_frame);
    }

    warp_img = cvCloneImage(lane);
    cvWarpPerspective( lane, warp_img, warp_matrix, CV_INTER_LINEAR|CV_WARP_INVERSE_MAP|CV_WARP_FILL_OUTLIERS);
    //May or may not be required depends upon the results.
    //warp_img = getLaneLines(warp_img);
    cvDilate(warp_img,warp_img,0,10);
    getDestinationPosition(warp_img);
    populateLanes(warp_img, map, scale);

    //Release warp_img and input frame and lane.
  }

  IplImage* LaneDetection::joinResult(IplImage* color_gray,IplImage* hough_gray)
  {
    int i,j;
    int index_color;
    int index_hough;
    IplImage* lane_gray = cvCreateImage(cvGetSize(color_gray), color_gray->depth, 1);
  
    uchar* color_data = (uchar*)color_gray->imageData;
    uchar* hough_data = (uchar*)hough_gray->imageData;
    uchar* lane_data = (uchar*)lane_gray->imageData;

    for(i=0;i<color_gray->height;i++)
    {
      for(j=0; j<color_gray->width; j++)
      {
        index_color = (i*color_gray->widthStep)+(j);
        index_hough = ((i+((canny_kernel-1)))*hough_gray->widthStep)+((j+((canny_kernel-1))));
      
        if((color_data[index_color]>0) && (hough_data[index_hough]>0))
          lane_data[index_color]= 255;
      }
    }
    return lane_gray;
  }

}
