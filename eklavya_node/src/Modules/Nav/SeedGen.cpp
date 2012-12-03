#include <stdio.h>
#include <math.h>

// SCALE : 1 cm = 1 pixel
#define TRUNC 0.5    // Truncate upto TRUNC fraction of the path
#define RES 5	// Resolution of paths to detect obstacles

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
  int nPoints;
      
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
      nPoints = yOut / RES;
      
      fprintf(fpOut, "%d %d %lf %lf %lf %lf\n", vl, vr, xOut, yOut, yawOut, gCost);
      fprintf(fpOut, "%d\n", nPoints);
      for(int j = 0; j < yOut; j += yOut / nPoints)
      {
        fprintf(fpOut, "%d %d\n", 0, j);
      }
    }
    else
    {  
      double Rout, Rin1, Rin2, Lin;
      
      Rin1 = yIn / tan(abslf(yawIn)) - abslf(xIn);
	  Rin2 = yIn / sin(abslf(yawIn));
      Rout = (Rin1 + Rin2) / 2;
      Lin = Rout * abslf(yawIn);

      double Lout = Lin * TRUNC;
      yawOut = yawIn * TRUNC;
      xOut = xIn * (1 - cos(abslf(yawOut))) / (1 - cos(abslf(yawIn)));
      yOut = yIn * sin(abslf(yawOut)) / sin(abslf(yawIn));
      gCost = Lout;
      //gCost = Lout * (1 + sin(abslf(yawOut)) * sin(abslf(yawOut)));
      
      fprintf(fpOut, "%d %d %lf %lf %lf %lf\n", vl, vr, xOut, yOut, 90 - yawOut, gCost);

      int nPoints = Lout / RES;
      fprintf(fpOut, "%d\n", nPoints);

      for(double theta = 0; theta < abslf(yawOut); theta += abslf(yawOut) / nPoints)
      {
          double px, py;
          px = xIn * (1 - cos(theta)) / (1 - cos(abslf(yawIn)));
          py = yIn * sin(theta) / sin(abslf(yawIn));
          fprintf(fpOut, "%lf %lf\n", px, py);
      }
    }
  }

  fclose(fpIn);
  fclose(fpOut);
  
  return 0;
}
