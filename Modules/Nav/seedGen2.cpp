#include <stdio.h>
#include <math.h>

// SCALE : 1 cm = 1 pixel
#define TRUNC 0.3    // Truncate upto TRUNC fraction of the path
#define RES 5 // Resolution of paths to detect obstacles = RES cm

double abslf(double num)
{
  if(num > 0)
    return num;
  else
    return -num;
}
  
int main()
{
  int nSeeds;

  FILE *fpIn = fopen("seeds1.txt", "r");
  FILE *fpOut = fopen("seeds.txt", "w");
  fscanf(fpIn, "%d\n", &nSeeds);
  fprintf(fpOut, "%d\n", nSeeds);

  double xOut;
  double yOut;
  double yawOut;
  double gCost;
      
  for(int i = 0; i < nSeeds; i++)
  {
    int vl, vr;
    double yawIn, xIn, yIn; 
    fscanf(fpIn, "%lf %lf %lf %d %d\n", &yawIn, &yIn, &xIn, &vl, &vr);
    
    if(vl == vr)
    {
      xOut = 0;
      yOut = yIn * TRUNC;
      yawOut = 90;
      gCost = yOut;
      int nPoints = yOut / RES + 1;
      
      fprintf(fpOut, "%d %d %lf %lf %lf %lf\n", vl, vr, xOut, yOut, yawOut, gCost);
      fprintf(fpOut, "%d\n", nPoints);
      for(int j = 0; j < nPoints; j++)
      {
        fprintf(fpOut, "%d %d\n", 0, j * RES);
      }
    }
    else
    {
      double d = sqrt(xIn * xIn + yIn * yIn);
      double theta = abslf(yawIn / 2);  // Assuming circular fit 
      double R = d / (2 * sin(theta)); // Theta is in rad
      xOut = (xIn < 0 ? -1 : 1) * (R - R * cos(2 * theta * TRUNC));
      yOut = R * sin(2 * theta * TRUNC);
      yawOut = (xIn < 0 ? -1 : 1) * (-2 * theta * TRUNC) + 3.14 / 2;
      double Lout = R * 2 * theta * TRUNC;
      gCost = Lout;
      //gCost = Lout * (1 + sin(abslf(yawOut)) * sin(abslf(yawOut)));
      
      fprintf(fpOut, "%d %d %lf %lf %lf %lf\n", vl, vr, xOut, yOut, yawOut * 180 / 3.14, gCost);
      //fprintf(stdout, "%d %d %lf %lf %lf %lf\n", vl, vr, xOut, yOut, yawOut * 180 / 3.14, gCost);
      
      int nPoints = Lout / RES + 1;
      fprintf(fpOut, "%d\n", nPoints);
      printf("npoints = %d\n", nPoints);
      
      for(double alpha = 0; alpha < 2 * theta * TRUNC; alpha += RES / R)
      {
          double px, py;
          px = (xIn < 0 ? -1 : 1) * (R - R * cos(alpha));
          py = R * sin(alpha);
          fprintf(fpOut, "%lf %lf\n", px, py);
      }
    }
  }

  fclose(fpIn);
  fclose(fpOut);
  
  return 0;
}
