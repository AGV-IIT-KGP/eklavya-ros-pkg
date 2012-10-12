#include <stdio.h>
#include <stdlib.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"
//#include "seeds.h"

#define _USE_MATH_DEFINES
#include <math.h>

//IplImage *img;

void initSeed(seed *s)
{
	s->x = 0;
	s->y = 0;
	s->theta = 0;
	s->k = 0;
	s->g = 0;
	s->l = NULL;
	s->size = 0;
}
double radiusOfCurvature(double l, double r)	
{
	if(l == r)
		return INFINITE;
	else
		return (WHEEL_SEPARATION/2) * (l+r) / (l-r);
}
double genk(double R2, double theta)
{
	return (R2*sqrt(pow(tan(theta), 2) + 1) + WHEEL_SEPARATION)/(R2*sqrt(pow(tan(theta), 2) + 1) - WHEEL_SEPARATION);
}
seed genSeed(double alpha, double R2)
{
	seed s;
	initSeed(&s);
	
	double k = genk(R2, alpha);
	double R1 = radiusOfCurvature(k, 1);
	if(R1 == INFINITE)
	{
		s.x = 0;
		s.y = R2;
		s.theta = 90;
		s.g = R2;
		s.k = k;

		s.l = (location *)malloc(s.y * sizeof(location));
		s.size = s.y/10.0;
		for(int i=0; i<s.y/10.0; i++)
		{
			s.l[i].x = 0;
			s.l[i].y = (i+1)*10.0;
		}
	}
	else
	{
		s.x = R2*R2/(2*R1);
		s.y = sqrt(R2*R2 - s.x * s.x);
		s.theta = atan((R1-s.x)/s.y);
		/*if(s.theta < 0)
			s.theta += M_PI;*/
		s.g = R1 * (M_PI/2 - s.theta);
		s.theta *= 180/M_PI;
		printf("theta: %f\n", s.theta);
		
		s.k = k;

		location *loc = (location *)malloc(300 * sizeof(location));
		int i=0;
		for(double alpha=10*180.0/(M_PI*R1); alpha <= 90-s.theta; alpha += 10*180.0/(M_PI*R1), i++)
		{
			loc[i].x = R1 * (1-cos(alpha*M_PI/180.0));
			loc[i].y = R1 * sin(alpha*M_PI/180.0);
			//printf("(%f, %f) ", R2 * (1-cos(alpha*M_PI/180.0)), R2 * sin(alpha*M_PI/180.0));
		}
		s.l = loc;
		s.size = i;
		printf("size: %d\n", i);
	}
	//printf("%f\n",R1/R2);
	printf("R1: %f R2: %f\n", R1, R2);
	printf("\n");
	
	return s;
}
double degToRad(double deg)
{
	return deg*M_PI/180;
}
void printSeeds(FILE *fp, seed s[])
{
	for(int i=0; i<9; i++)
	{
		fprintf(fp, "%f %f %f %f %f\n", s[i].k, s[i].x, s[i].y, s[i].theta, s[i].g);
		fprintf(fp, "%d\n", s[i].size);
		for(int j=0; j<s[i].size; j++)
			fprintf(fp, "%f %f\n", s[i].l[j].x, s[i].l[j].y);
	}
}

int main()
{
	FILE *fp = fopen("seeds3.txt", "w");
	//img = cvCreateImage(cvSize(500, 500), IPL_DEPTH_8U, 3);
	//cvLine(img, cvPoint(250, 499), cvPoint(250+1, 498), CV_RGB(0, 0, 255), 3, CV_AA, 0);

	fprintf(fp, "36\n");
	seed s[4][9];
	for(int j=0; j<4; j++)
	{
		s[j][8] = s[j][0] = genSeed(degToRad(30+((j)*6)), (4.0-j)*WAVEFRONT_RADIUS/4);
		s[j][7] = s[j][1] = genSeed(degToRad(60+((j)*3)), (4.0-j)*WAVEFRONT_RADIUS/4);
		s[j][6] = s[j][2] = genSeed(degToRad(75+((j)*2)), (4.0-j)*WAVEFRONT_RADIUS/4);
		s[j][5] = s[j][3] = genSeed(degToRad(85+(j)), (4.0-j)*WAVEFRONT_RADIUS/4);
		s[j][4] = genSeed(degToRad(90), (4.0-j)*WAVEFRONT_RADIUS/4);
		for(int i=5; i<=8; i++)
		{
			s[j][i].x *= -1;
			s[j][i].theta = 180-s[j][i].theta;
			s[j][i].k = 1/s[j][i].k;

			s[j][i].l = (location *)malloc(300*sizeof(location));
			for(int p=0; p<s[j][i].size; p++)
			{
				s[j][i].l[p].x = -1*s[j][8-i].l[p].x;
				s[j][i].l[p].y = s[j][8-i].l[p].y;
			}
		}
		printSeeds(fp, s[j]);
		//for(int i=0; i<9; i++)
		//	cvLine(img, cvPoint(250+s[j][i].x, 500-s[j][i].y), cvPoint(250+s[j][i].x+1, 500-(s[j][i].y+1)), CV_RGB(255, 0, 0), 2, CV_AA, 0);
	}
	//cvNamedWindow("plot", CV_WINDOW_AUTOSIZE);
	//cvShowImage("plot", img);
	//cvWaitKey(0);
	fclose(fp);
	return 0;
}
