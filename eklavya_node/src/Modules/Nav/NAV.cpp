#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include "tserial.h"
#include "id.h"

#define SAFEZONE_LL (-191)
#define SAFEZONE_RL 159
#define DL_MINUS_DR_UPPER_LIMIT 159			// Have to be chosen such that both
#define DL_MINUS_DR_LOWER_LIMIT (-191)		// the lanes are visible within these limits
#define SINGLE_LANE_WIDTH 20

typedef struct botData
{
	int dL, dR;
	double angle;
}botData;
typedef struct obstacle 
{
	CvPoint loc;
	int rad;
}obstacle;
typedef struct map 
{
	int nObstacles;
	int nLanes;
	int LLDst, RLDst;
	obstacle *obs;
	botData d;
}map;
typedef struct location
{
	double x, y;	// Coordinates
	double theta;	// Angle w.r.t horizontal (x-axis)
}location;
typedef struct elem
{
	location l, parent;		// Location of this Element
	double g, h;	// Costs
	int id;			// The identity of this point w.r.t its parent
}elem;
typedef struct list
{
	elem p;
	list *next;
}list;
typedef struct lineList
{
	CvPoint p;
	float m;
}lineList;

void findFit(IplImage* img,float *xp,float *yp)
{
	float a1=0,b1=0,c1=0,a2=0,b2=0,c2=0;
	int w=img->height;
	for( int y=10; y<img->height-10; y++ ) 
	{
		uchar* ptr = (uchar*) (img->imageData + y * img->widthStep);
		for( int x=10; x<img->width-10; x++ ) 
		{
			if(ptr[x]>100)
			{
				printf("(%d,%d)",x,y);
				a1+=x*x;
				b1+=x;
				c1+=x*y;
				a2+=x;
				b2+=1;
				c2+=y;
			}
		}
	}
	*xp=(c2*b1-c1*b2)/(a1*b2-a2*b1);
	*yp=(c1*a2-c2*a1)/(a1*b2-a2*b1);
}
IplImage* doCanny(IplImage* in,double lowThresh,double highThresh,double aperture) 
{
	if(in->nChannels != 1)
		return(0); //Canny only handles gray scale images

	IplImage* out = cvCreateImage(cvGetSize( in ),IPL_DEPTH_8U,1);
	cvCanny( in, out, lowThresh, highThresh, aperture );
	return( out );
}
IplImage* simplethreshold(IplImage* img, int threshold) 
{
	for( int y=0; y<img->height; y++ ) 
	{
		uchar* ptr = (uchar*) (img->imageData + y * img->widthStep);
		for( int x=0; x<img->width; x++ ) 
		{
			if(ptr[x+1]>=threshold)
			{
				ptr[x+1] = 255 ;
			}
			else
			{
				ptr[x+1] = 0 ;
			}
		}
	}
	return img;
}
void navCommand(int l, int r)
{
	Tserial* p = new Tserial();
    p->connect("COM2",9600,spNONE);
	
	p->sendChar('w');
	p->sendChar('0'+l);
	p->sendChar('0'+r);
	
	p->disconnect();
}
IplImage* cutImg(IplImage* img)
{
	for( int y=0; y < 200; y++ ) 
	{
		uchar* ptr = (uchar*) (img->imageData + y * img->widthStep);
		for( int x=0; x<img->width; x++ ) 
		{
			ptr[x] = 0;
		}
	}
	return img;
}
char* nameGen(int idx)
{
	char* name = (char*)malloc(8*sizeof(char));
	int t = idx;
	for(int i=2; i>=0; i--)
	{	
		name[i] = t%10+'0';
		t /= 10;
	}
	name[3] = '.';
	name[4] = 'j';
	name[5] = 'p';
	name[6] = 'g';
	name[7] = '\0';

	return name;
}
void plotLine(IplImage* img, CvPoint p, double m, CvScalar color)
{
	double x1, y1, x2, y2;
	y1 = 0;
	y2 = img->height;
	x1 = p.x - (y1-p.y)/m;
	x2 = p.x - (y2-p.y)/m; 
	cvLine(img, cvPoint(x1, y1), cvPoint(x2, y2), color, 1, CV_AA, 0);
}
void plotPoint(IplImage* img, CvPoint p, CvScalar color)
{
	cvLine(img, p, cvPoint(p.x-1, p.y-1), color, 2, CV_AA, 0);
}
void updateMap(botData d)
{
	IplImage* map = cvCreateImage(cvSize(500, 500), IPL_DEPTH_8U, 3);
	cvNamedWindow("map", CV_WINDOW_AUTOSIZE);

	for(int i = 0; i < 500; i++) 
	{
		uchar* p = (uchar*)(map->imageData + i*map->widthStep);
		for(int j = 0; j < 500; j++) 
		{
			p[3*j + 0] = 0;
			p[3*j + 1] = 0;
			p[3*j + 2] = 0;
		}
	}

	int lOff = 120, tOff = 400, semiSZW = 50;
	cvLine(map, cvPoint(lOff, 0), cvPoint(lOff, 500), CV_RGB(255, 255, 255), 5, CV_AA, 0);
	cvLine(map, cvPoint(500-lOff, 0), cvPoint(500-lOff, 500), CV_RGB(255, 255, 255), 5, CV_AA, 0);
	for(int i = 0; i < 500; i++)
	{
		uchar* p = (uchar*)(map->imageData + i*map->widthStep);
		for(int j = 250-semiSZW; j <= 250+semiSZW; j++)
		{
			p[3*j + 0] = 150;
			p[3*j + 1] = 150;
			p[3*j + 2] = 150;
		}
	}
	cvLine(map, cvPoint(250, 0), cvPoint(250, 500), CV_RGB(255, 255, 0), 1, CV_AA, 0);
	
	// The Path is assumed to be piecewise linear

	double r = 25;
	CvPoint u = cvPoint(lOff + d.dL*(500.0-2*lOff)/(d.dL+d.dR), tOff);
	CvPoint v = cvPoint(u.x + r*sin(d.angle*CV_PI/180.0), u.y - r*cos(d.angle*CV_PI/180.0));
	cvLine(map, u, v, CV_RGB(255, 0, 0), 2, CV_AA, 0);

	cvShowImage("map", map);
}
void genMap(map m)
{
	//if((m.d.dL - m.d.dR >= DL_MINUS_DR_LOWER_LIMIT) && (m.d.dL - m.d.dR <= DL_MINUS_DR_UPPER_LIMIT))
	int lOff = 120, semiSZW = 50;
	if((m.d.dL*(500.0-2*lOff)/(m.d.dL+m.d.dR) - 250 + lOff >= -semiSZW) && (m.d.dL*(500.0-2*lOff)/(m.d.dL+m.d.dR) - 250 + lOff <= semiSZW))
	{
		printf("Inside Unbiased Zone\n");
		if(m.d.dL >= m.d.dR)
		{
			if(m.d.angle < 0)
			{
				printf("Towards the Central Axis\n");
				// Make a circular turn towards the central axis
			}
			else
			{
				printf("Away from the Central Axis\n");
				// Follow unbiased 3-7 rule
			}
		}
		else
		{
			if(m.d.angle < 0)
			{
				printf("Away from the Central Axis\n");
				// Follow unbiased 3-7 rule
			}
			else
			{
				printf("Towards the Central Axis\n");
				// Make a circular turn towards the central axis
			}
		}
	}
	else
	{
		printf("Outside Unbiased Zone\n");
		// Follow 3-7 rule biased by dL / dR
	}
	// Add Obstacle Data
	// Plan a Path
}

elem searchforcoor(list *cl,double x,double y){
	list *t = cl;
	while(t)
	{
		//cvLine(img,cvPoint(t->p.parent.x*40,500-(t->p.parent.y*40)),cvPoint(t->p.parent.x*40+2,500-(t->p.parent.y*40)-2),CV_RGB(255,255,0),2,CV_AA,0);
		//printf("(%.3f, %.3f) ", t->p.l.x, t->p.l.y);
		if((t->p.l.x==x)&&(t->p.l.y==y)){
			return t->p;
		}
		t=t->next;
	}
}
list *append(list *l, elem c)
{
	list *node = (list *)malloc(sizeof(list));
	node->p = c;
	node->next = NULL;

	list *t = l;
	if(t == NULL)
		return node;
	while(t->next != NULL)
	{
		if((t->p.l.x == c.l.x) && (t->p.l.y == c.l.y))
		{
			t->p = c;
			return l;
		}
		t = t->next;
	}
	t->next = node;
	return l;
}
bool isNear(location a, location b)
{
	//if((pow(a.x - b.x, 2) + pow(a.y - b.y, 2) < 800) && (abs(a.theta - b.theta) < 5))
	if((abs(a.x - b.x) <= 10) && (a.y >= 5*24) && (abs(a.theta - b.theta) < 10))
		return true;
	else
		return false;
}
void printPos(elem p)
{
	printf("(%.3f, %.3f, %.3f):(%d)\n", p.l.x, p.l.y, p.l.theta, p.id);
}
elem *neighbours(elem p, elem *np1, int n)
{
	double r = 1;
	double R;

	elem *np = (elem *)malloc(n*sizeof(elem));
	for(int i=0; i<n; i++)
		np[i] = np1[i];
	
	//np[4].l.x = 0;	np[4].l.y = r;	np[4].l.theta = 90;	
	//np[4].g = 10;	np[4].h = INFINITY; np[4].id = _55;
	//for(int i = 0; i < 4; i++)	// Reference Points
	//{
	//	R = radiusOfCurvature(10-(i+1), i+1);
	//	np[i].l.x = genX(r, R);
	//	np[i].l.y = genY(r, R);
	//	np[i].l.theta = genAngle(r, R);
	//	np[i].g = 10;	np[i].h = INFINITY;

	//	np[8-i].l.x = -np[i].l.x;
	//	np[8-i].l.y = np[i].l.y;
	//	np[8-i].l.theta = 180 - np[i].l.theta;
	//	np[8-i].g = np[i].g;	np[8-i].h = INFINITY;
	//}
	/*np[0].id = _91;	np[1].id = _82;	np[2].id = _73;	np[3].id = _64;
	np[8].id = _19;	np[7].id = _28;	np[6].id = _37;	np[5].id = _46;*/

	/*printf("Reference\n");
	for(int i = 0; i < 9; i++)
		printPos(np[i]);*/

	for(int i = 0; i < n; i++)	// Rotation
	{
		double tx, ty;
		tx = np[i].l.x; 
		ty = np[i].l.y;
		np[i].l.x = tx*sin(p.l.theta*(3.14/180)) + ty*cos(p.l.theta*(3.14/180));
		np[i].l.y = -tx*cos(p.l.theta*(3.14/180)) + ty*sin(p.l.theta*(3.14/180));
		np[i].l.theta = np[i].l.theta - (90 - p.l.theta);
	}
	
	/*printf("Rotation\n");
	for(int i = 0; i < 9; i++)
		printPos(np[i]);
*/
	for(int i = 0; i < n; i++)
	{
		np[i].l.x += p.l.x;
		np[i].l.y += p.l.y;
		np[i].parent = p.l;
	}
	//printf("Translation\n");
	/*for(int i = 0; i < n; i++)
		printPos(np[i]);*/
	
	return np;
}
list *update(list *ol, elem e, location t, elem *np, int n)
{
	elem *nextElem = neighbours(e, np, n);
	for(int i = 0; i < n; i++)
	{
		//printPos(nextElem[i]);
		nextElem[i].h = abs(nextElem[i].l.x - t.x);	// Manhattan Distance
		//nextElem[i].h = sqrt(pow((nextElem[i].l.x - t.x),2)+pow(nextElem[i].l.y - t.y,2));  //euclidean distance
		if(abs(nextElem[i].l.theta-90)<25)
			ol = append(ol, nextElem[i]);
	}
	
	return ol;
}
elem findMin(list *l)
{
	double minF = 1000;
	elem min;
	list *t = l;
	while(t != NULL)
	{
		double f = t->p.h;
		printf("(%f, %d)\n", f, t->p.id);
		if(f < minF)
		{
			minF = f;
			min = t->p;
		}
		t = t->next;
	}
	return min;
}
list *detach(list *l, elem c)
{
	list *t = l;
	if((t == NULL) || (t->next == NULL))
		return NULL;
	if((t->p.l.x == c.l.x) && (t->p.l.y == c.l.y))
	{
		return l->next;
	}

	while(t->next != NULL)
	{
		if((t->next->p.l.x == c.l.x) && (t->next->p.l.y == c.l.y))
			break;
		t = t->next;
	}
	if(t->next != NULL)
		t->next = t->next->next;
	
	return l;
}
void printList(list *l)
{
	list *t = l;
	while(t != NULL)
	{
		printf("(%.3f, %.3f, %.3f) ", t->p.l.x, t->p.l.y, t->p.l.theta);
		t = t->next;
	}
	printf("\n");
}
elem *loadPosData(int n)
{
	float x, y, z;
	int id;
	elem* np = (elem *)malloc(n*sizeof(elem));
	FILE *fp = fopen("n9.txt", "r");
	
	for(int i=0; i<n; i++)
	{
		fscanf(fp, "%f", &x);
		fscanf(fp, "%f", &y);
		fscanf(fp, "%f", &z);
		fscanf(fp, "%d", &id);
		np[i].l.x = x;
		np[i].l.y = y;
		np[i].l.theta = z;
		np[i].g = 10;
		np[i].h = INFINITY;
		np[i].id = id;
		//printPos(np[i]);
	}
	fclose(fp);
	return np;
}
int PercentileThreshold(IplImage* in, float percentile)
{
	int bins[256];
	for(int i=0; i<256; i++)
		bins[i] = 0;
	
	for(int i=0; i<in->height; i++){	// Histogram construction
		uchar* ptr = (uchar*) (in->imageData + i*in->widthStep);
		for(int j=0; j<in->width; j++)
			bins[ptr[j]]++;							
	}
	int sum=0, i=255;
	while((i >= 0)&&(sum < in->imageSize*percentile/100))
		sum += bins[i--];
	printf("Percentile Threshold:%d\n", i);
	
	return i;
}
void insert(lineList *l, CvPoint p, float m, int idx)
{
	if(idx==0)
	{
		l[0].p = p;
		l[0].m = m;
		return;
	}

	int j = idx;
	while((l[j].p.x > p.x) && (j>=0))
	{
		l[j+1].p = l[j].p;
		l[j+1].m = l[j].m;
		j--;
	}
	l[j+1].p = p;
	l[j+1].m = m;
}

void main(int argc, char** argv)
{
	cvNamedWindow("Source", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Gray Image", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Histogram Equalized Image", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Smoothed Image", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow("canny", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Thresholded Image", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Warped Image", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow("canny", CV_WINDOW_AUTOSIZE);

	CvPoint2D32f srcQuad[4], dstQuad[4];
	CvMat* warp_matrix = cvCreateMat(3,3,CV_32FC1);
	float Z=1;

	dstQuad[0].x = 233; //src Top left
	dstQuad[0].y = 94;
	dstQuad[1].x = 420; //src Top right
	dstQuad[1].y = 103;
	dstQuad[2].x = 4; //src Bottom left
	dstQuad[2].y = 192;
	dstQuad[3].x = 632; //src Bot right
	dstQuad[3].y = 222;

	//srcQuad[0].x = 100; //dst Top left
	//srcQuad[0].y = 120;
	//srcQuad[1].x = 540; //dst Top right
	//srcQuad[1].y = 120;
	//srcQuad[2].x = 100; //dst Bottom left
	//srcQuad[2].y = 360;
	//srcQuad[3].x = 540; //dst Bot right
	//srcQuad[3].y = 360;

	int lOff = 50, tOff = 150;
	srcQuad[0].x = tOff; //dst Top left
	srcQuad[0].y = lOff;
	srcQuad[1].x = 640-tOff; //dst Top right
	srcQuad[1].y = lOff;
	srcQuad[2].x = tOff; //dst Bottom left
	srcQuad[2].y = 480-lOff;
	srcQuad[3].x = 640-tOff; //dst Bot right
	srcQuad[3].y = 480-lOff;

	cvGetPerspectiveTransform(srcQuad, dstQuad,	warp_matrix);
	
	int ik=0, ni = 0, niX = 22-1;
	char names[22][25] = {
							"../../Data/6 Dec/009.jpg", 
							"../../Data/6 Dec/011.jpg",
							"../../Data/6 Dec/012.jpg",
							"../../Data/6 Dec/016.jpg",
							"../../Data/6 Dec/018.jpg",
							"../../Data/6 Dec/019.jpg",
							"../../Data/6 Dec/020.jpg",
							"../../Data/6 Dec/022.jpg",
							"../../Data/6 Dec/024.jpg",
							"../../Data/6 Dec/064.jpg",
							"../../Data/6 Dec/065.jpg",
							"../../Data/6 Dec/066.jpg",
							"../../Data/6 Dec/067.jpg",
							"../../Data/6 Dec/068.jpg",
							"../../Data/6 Dec/069.jpg",
							"../../Data/6 Dec/070.jpg",
							"../../Data/6 Dec/071.jpg",
							"../../Data/6 Dec/072.jpg",
							"../../Data/6 Dec/073.jpg",
							"../../Data/6 Dec/074.jpg",
							"../../Data/6 Dec/075.jpg",
							"../../Data/6 Dec/076.jpg"
						};
	int lwSum = 0, nopf = 0;
	//CvCapture *capture = cvCaptureFromCAM(0);
	//CvCapture *capture = cvCaptureFromFile("../../Data/29 Jan/120129-171828_clip0.avi");
	/*double fps = cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
	IplImage* image = cvRetrieveFrame(capture);
	CvSize imgSize;
    imgSize.width = image->width;
    imgSize.height = image->height;
	CvVideoWriter *writer = cvCreateVideoWriter("out.avi", CV_FOURCC('M', 'J', 'P', 'G'), fps, imgSize);*/
	while(1)
	{
		//IplImage* img = cvQueryFrame(capture);
		IplImage* img = cvLoadImage( "../../Data/29 Jan/ref1.jpg", CV_LOAD_IMAGE_COLOR);
		cvShowImage( "Source", img );
		//cvWriteFrame(writer, img);
		//cvSaveImage(nameGen(ik++), img, 0);
		
		cvWaitKey(10);

		IplImage* grayimg = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
		cvCvtColor(img, grayimg, CV_RGB2GRAY);
		cvShowImage("Gray Image", grayimg);

		//cvEqualizeHist(grayimg, grayimg);
		cvShowImage("Histogram Equalized Image", grayimg);
		
		cvSmooth(grayimg, grayimg, CV_GAUSSIAN, 3, 3, 0.0, 0.0);
		cvShowImage("Smoothed Image", grayimg);
		
		cvThreshold(grayimg, grayimg, 245, 255, CV_THRESH_BINARY);
		cvShowImage("Thresholded Image", grayimg);
		
		IplImage* warp_img = cvCloneImage(grayimg);
		CV_MAT_ELEM(*warp_matrix, float, 2, 2) = Z;
		cvWarpPerspective(grayimg, warp_img, warp_matrix, CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS);
		cvShowImage("Warped Image", warp_img);

		IplImage* finalimg = cvCreateImage(cvGetSize(warp_img), IPL_DEPTH_8U, 3);
		CvMemStorage* line_storage=cvCreateMemStorage(0);

		CvSeq* results =  cvHoughLines2(warp_img, line_storage, CV_HOUGH_PROBABILISTIC, 10, CV_PI/180*5, 150, 75, 10);
		//printf("# hough lines: %d\n", results->total);

		cvWaitKey(10);
		//continue;

		double angle = 0.0, temp;
		double lengthSqd, wSum=0;
		CvPoint center = cvPoint(0, 0);
		for( int i = 0; i < results->total; i++ )
		{
			CvPoint* line = (CvPoint*)cvGetSeqElem(results,i);
			cvLine( finalimg, line[0], line[1], CV_RGB(255,0,0), 1, CV_AA, 0 );
			//lengthSqd = (line[0].x - line[1].x)*(line[0].x - line[1].x) + (line[0].y - line[1].y)*(line[0].y - line[1].y);
			wSum += 1;//lengthSqd;
			if(line[0].y > line[1].y)
				temp = atan((line[0].y - line[1].y + 0.0) / (line[0].x - line[1].x));
			else
				temp = atan((line[1].y - line[0].y + 0.0) / (line[1].x - line[0].x));
			if(temp < 0)
				angle += (90 + 180/3.14*temp)/* * lengthSqd*/;
			else
				angle += (180/3.14*temp - 90)/* * lengthSqd*/;
			center.x += (line[0].x + line[1].x)/2;
			center.y += (line[0].y + line[1].y)/2;
		}
		angle /= wSum;	// Angle Direction: Left == -ve and Right == +ve
							// Angle is calculated w.r.t Vertical
		//angle+=10;		// Angle Offset (Depends on camera's position)
		//printf("# Hough Lines: %d\nAngle: %f\n", results->total, angle);

		center.x /= results->total;
		center.y /= results->total;
		//printf("Center: (%d, %d)\n", center.x, center.y);
		plotPoint(finalimg, center, CV_RGB(0,255,0));

		double m = (angle != 0) ? tan(CV_PI*(0.5-angle/180)) : 100000;	// 100000 represents a very large slope (near vertical)
		//m=-m;		// Slope Correction
		//printf("Slope: %f\n", m);

		plotLine(finalimg, center, m, CV_RGB(0, 0, 255));	// The Center Line

		lineList *leftLines = (lineList*)malloc(results->total*sizeof(lineList));
		lineList *rightLines = (lineList*)malloc(results->total*sizeof(lineList));

		CvPoint leftCenter = cvPoint(0, 0), rightCenter = cvPoint(0, 0);
		double leftSlope = 0, rightSlope = 0, leftCount = 0, rightCount = 0;
		for( int i = 0; i < results->total; i++ )
		{
			CvPoint* line = (CvPoint*)cvGetSeqElem(results,i);
			CvPoint midPoint = cvPoint((line[0].x + line[1].x)/2, (line[0].y + line[1].y)/2);
			double L11 = (0-center.y + m*(0-center.x + 0.0))/m;
			double L22 = (midPoint.y-center.y + m*(midPoint.x-center.x + 0.0))/m;
			if(L11*L22 > 0)
			{
				leftCenter.x += midPoint.x;
				leftCenter.y += midPoint.y;
				leftSlope += -(line[1].y - line[0].y)/(line[1].x - line[0].x+0.0001);
				//insert(leftLines, leftCenter, leftSlope, leftCount);
				leftCount++;
			}
			else
			{
				rightCenter.x += midPoint.x;
				rightCenter.y += midPoint.y;
				rightSlope += -(line[1].y - line[0].y)/(line[1].x - line[0].x+0.0001);
				//insert(rightLines, rightCenter, rightSlope, rightCount);
				rightCount++;
			}
		}
		leftCenter.x /= leftCount;		leftCenter.y /= leftCount;		leftSlope /= leftCount;
		rightCenter.x /= rightCount;	rightCenter.y /= rightCount;	rightSlope /= rightCount;

		plotLine(finalimg, leftCenter, m, CV_RGB(255, 255, 0));	// The Left Line
		plotLine(finalimg, rightCenter, m, CV_RGB(0, 255, 255));	// The Right Line

		CvPoint botCenter = cvPoint(finalimg->width/2, finalimg->height);
		plotPoint(finalimg, botCenter, CV_RGB(255,0,255));	// The Bot Center

		int dL = abs(botCenter.y-leftCenter.y + m * (botCenter.x-leftCenter.x)) / sqrt(m*m + 1);
		int dR = abs(botCenter.y-rightCenter.y + m * (botCenter.x-rightCenter.x)) / sqrt(m*m + 1);
		//printf("dL: %d\n", dL);
		//printf("dR: %d\n", dR);

		int lw = abs((leftCenter.y - rightCenter.y) + m*(leftCenter.x - rightCenter.x)) / sqrt(m*m + 1);
		lwSum += lw;
		nopf++;
		//printf("lw: %d\nlwSum: %d\nnopf: %d\nlwm: %d\n", lw, lwSum, nopf, lwSum/nopf);

		if(lw <= SINGLE_LANE_WIDTH)
		{
			double L11 = (0-leftCenter.y + m*(0-leftCenter.x + 0.0))/m;
			double L22 = (botCenter.y-leftCenter.y + m*(botCenter.x-leftCenter.x + 0.0))/m;
			if(L11*L22 < 0)
				dR = lwSum/nopf - dL;	// Only Left Lane is visible
			else
				dL = lwSum/nopf - dR;	// Only Right Lane is visible
		}
		/*int lane;
		double L11 = (0-leftCenter.y + m*(0-leftCenter.x + 0.0))/m;
		double L22 = (botCenter.y-leftCenter.y + m*(botCenter.x-leftCenter.x + 0.0))/m;
		if(L11*L22 > 0)
			lane = RIGHT;
		else
			lane = LEFT;*/
		
		//printf("dL: %d\n", dL);
		//printf("dR: %d\n", dR);

		cvShowImage("final",finalimg);
		cvSaveImage("test.jpg", finalimg, 0);
		
		cvWaitKey(10);

		botData d;
		d.angle = angle;
		d.dL = dL;
		d.dR = dR;
		/*d.angle = 0;
		d.dL = 150;
		d.dR = 100;*/

		updateMap(d);

		map M;
		M.nObstacles = 0;
		M.d = d;
		//genMap(M);

		//if(dl-dr < SAFEZONE_LL)	// Assume that the bot lies just on the boundary of the safe zone
		//{
		//	if(angle < -10)
		//	{
		//		navCommand(7, angle);
		//	}
		//	else
		//	{
		//		navCommand(7, angle);
		//	}
		//}	
		//else if(dl-dr > SAFEZONE_RL)
		//{
		//	if(angle > 10)
		//	{
		//		navCommand(-7, angle);
		//	}
		//	else
		//	{
		//		navCommand(-7, angle);
		//	}
		//}
		//else
		//{
		//	if((angle < 10) && (angle > -10))
		//	{
		//		navCommand(angle, angle);
		//	}
		//	else
		//	{
		//		navCommand(0, angle);
		//	}
		//}

		printf("Bot:\t(%d, %d, %.3f)\n", dL, (finalimg->height)/10, 90.0+angle);
		printf("Target:\t(%d, %d, %.3f)\n", (dL+dR)/2, (finalimg->height)*9/10, 90.0);

		location bot, target;
		bot.x = dL+20;		bot.y = (finalimg->height)/10;		bot.theta = 90.0+angle-10;
		target.x = (dL+dR)/2;	target.y = (finalimg->height)*9/10;	target.theta = 90.0;

		list *ol = NULL, *cl = NULL;
		elem e,vare;
		e.l = bot;	e.g = 0;	e.h = 0;	e.id = UNDEFINED;

		int n = 9;
		elem* np = loadPosData(n);
	
		while(1)
		{
			cl = append(cl, e);
			//printList(cl);
			if(isNear(e.l, target))
				break;
			ol = update(ol, e, target, np, n);
			//printList(ol);
			e = findMin(ol);
			//printf("Min: (%.3f, %.3f, %.3f, %d)\n", e.l.x, e.l.y, e.l.theta, e.id);

			//cvWaitKey(0);
			ol = detach(ol, e);
			//printList(ol);
			//getchar();
		}
		//printf("path found!\n");
		//getchar();
		cvNamedWindow("plot",CV_WINDOW_AUTOSIZE);
		IplImage *plot = cvCreateImage(cvSize(500, 500), IPL_DEPTH_8U, 3);
	
		//list *t = cl;
		//while(t)
		//{
		//	cvLine(img,cvPoint(t->p.parent.x*40,500-(t->p.parent.y*40)),cvPoint(t->p.parent.x*40+2,500-(t->p.parent.y*40)-2),CV_RGB(255,255,0),2,CV_AA,0);
		//	//printf("(%.3f, %.3f) ", t->p.l.x, t->p.l.y);
		//	t=t->next;
		//}
		CvPoint a = cvPoint(target.x, 500 - (target.y));
		CvPoint b = cvPoint((target.x + 10*cos(target.theta*(CV_PI/180))), 500 - ((target.y+10*sin(target.theta*(CV_PI/180)))));
		cvLine(plot, a, b, CV_RGB(0,255,0), 2, CV_AA, 0);

		a = cvPoint(bot.x, 500 - (bot.y));
		b = cvPoint((bot.x + 10*cos(bot.theta*(CV_PI/180))), 500 - ((bot.y+10*sin(bot.theta*(CV_PI/180)))));
		cvLine(plot, a, b, CV_RGB(0,0,255), 2, CV_AA, 0);

		vare = e;
		a = cvPoint(vare.l.x, 500 - (vare.l.y));
		b = cvPoint((vare.l.x + 10*cos(vare.l.theta*(CV_PI/180))), 500 - ((vare.l.y+10*sin(vare.l.theta*(CV_PI/180)))));
		cvLine(plot, a, b, CV_RGB(255,0,0), 2, CV_AA, 0);
	
		printf("(%.3f, %.3f, %.3f) : %d\n", vare.l.x, vare.l.y, vare.l.theta, vare.id);
		while(!((abs(vare.l.x-bot.x) < 1.25) && (abs(vare.l.y-bot.y) < 1.25)))
		{
			vare=searchforcoor(cl,vare.parent.x,vare.parent.y);
			if(vare.id != -1)
			{
				printf("(%.3f, %.3f, %.3f) : %d\n", vare.l.x, vare.l.y, vare.l.theta, vare.id);
				a = cvPoint(vare.l.x, 500 - (vare.l.y));
				b = cvPoint((vare.l.x + 10*cos(vare.l.theta*(CV_PI/180))), 500 - ((vare.l.y+10*sin(vare.l.theta*(CV_PI/180)))));
				cvLine(plot, a, b, CV_RGB(255,0,0), 2, CV_AA, 0);
				e = vare;
			}
		}
		printf("\n(%.3f, %.3f, %.3f) : %d : ", e.l.x, e.l.y, e.l.theta, e.id);
		e.id > 50 ? printf("Left\n") : printf("Right\n");

		cvShowImage("plot",plot);
		//navCommand(10-e.id, e.id);
		int c = cvWaitKey(0);
		if(c == '4')
		{
			if(ni != 0)
				ni--;
		}
		else if(c == '6')
		{
			if(ni != niX)
				ni++;
		}
	}
}


