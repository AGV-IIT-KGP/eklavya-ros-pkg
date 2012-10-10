// grid.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include <opencv\highgui.h>
#include <opencv\cv.h>
#include <opencv\cxcore.h>

void main()
{
	IplImage* img;
  	char *win = "image";
  	img = cvLoadImage("nature.jpg",1);
	int i=0,j_0=0,j_1=1000;
	int thickness=1,lineType=8,shift=0;
  	
	if(!img)
	{
		printf("Could not load the image!\n");
		exit(0);
	}
    else
    {
    	for(i=0;i<1000;i+=25)
		{
			CvPoint pt1= cvPoint(i,j_0);
            CvPoint pt2= cvPoint(i,j_1);
			cvLine(img,pt1,pt2,cvScalar(0,0,0),thickness,lineType,shift);
		}

		for(i=0;i<1000;i+=25)
		{
			CvPoint pt1= cvPoint(j_0,i);
            CvPoint pt2= cvPoint(j_1,i);
			cvLine(img,pt1,pt2,cvScalar(0,0,0),thickness,lineType,shift);
		}

		cvNamedWindow(win,CV_WINDOW_AUTOSIZE);
     	cvShowImage(win,img);
		cvWaitKey(0);
	}

	cvReleaseImage(&img);
  	cvDestroyWindow(win);
}
