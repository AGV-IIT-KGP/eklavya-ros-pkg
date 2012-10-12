#include "cv.h"
#include "highgui.h"
#include <stdlib.h>
#include <stdio.h>
#include "PathPlanner.h"
#include "../../Utils/SerialPortLinux/serial_lnx.h"
#include <stdexcept>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include "../devices.h"

#define UART_COMM_PORT BOT_PATH
#define Sleep(i) usleep(i)
#define UART_BAUD_RATE 19200
#define Y 1
#define MAX_VEL 50
#define UNDEFINED -1
#define INFINITY -2
#define MAP_OUTPUT "map.txt"
#define CLOSED 0
#define OPEN 1
#define MAXMIN 999999
#define BOT_HEIGHT 10
#define BOT_WIDTH 8
#define X 0
#define MAP_MAX 1000
//#define VIS
//#define SIMCTL
//#define SIMOBS
//#define SIMSEEDS

#ifdef SIMSEEDS
#define BPS_SOURCE "Modules/Nav/seeds.txt"
#else
#define BPS_SOURCE "Modules/Nav/seeds3.txt"
#endif

using namespace std;

namespace Nav
{
  typedef struct list
  {
    int x, y;
    list *next, *prev;
  }list;
  typedef struct state
  {
    int x, y;
    double theta;
    bool wFlag;   // walkable or not
    double g, h;  // costs
    int listType; // open list / closed list / undefined
    list *l;
    int px, py;   // parent
    int vl, vr;
    double k;
    int id;
    int size;
  }state;
  typedef struct obstacle
  {
    int x, y, r;
  }obstacle;
  typedef struct seedPath
  {
    int x, y;
  }seedPath;

  bool inFlag = false;
  int cLW = 0, nSeeds;
  state map[MAP_MAX][MAP_MAX];
  list *cl, *ol, *path;
  seedPath **sPoints;
  IplImage *mapImg;
  int extendSeedsFlag;
  int flag=0;
  state s, b, t, *seeds, *N;
  int leftSpeedAt[9]={150,140,130,120,80,45,35,25,15};
  int rightSpeedAt[9]={15,25,35,45,80,120,130,140,150};
  double previousYaw=0;
  int count=0;
  Tserial *p;
  int iteration;

  void addObstacle(CvPoint ob, int rad)
  {
    for(int i=-rad; i<rad; i++)
    {
      for(int j=-rad; j<rad; j++)
      {
        if(i*i + j*j <= rad*rad)
          map[ob.x + i][ob.y + j].wFlag = false;
      }
    }
  }

  int transfer(int vl, int vr, double currentCurvature)
  {
    int mode = 7, Kp = 5;
    double velocityRatioAt[5] = {1.35, 1.16, 1, 0.85, 0.73};
    double targetVelocityRatio = vl / vr;
    double targetCurvature = 2 * (targetVelocityRatio - 1)/(targetVelocityRatio+1);
    for(int i=0;i<5;i++)
    {
      if(targetVelocityRatio>velocityRatioAt[4-i])
      {
        mode--;
      }
    }
    if(mode==5)
    {
      mode=4;
    }
    mode+=int(Kp*(targetCurvature-currentCurvature));
    return mode;
  }

  void navCommandProcess(int vl, int vr)
  {
    int l = vl;
    int r = vr;
//    printf("Input Command: (%d, %d)\n", l, r);

    if(l > r)
    {
      l = 40;
      r = 25;
    }
    else if(l < r)
    {
      l = 18;
      r = 40;
      //l = 0;
      //r = 50;
    }
    else if((l != 0) || (r != 0))
    {
      l = 22;
      r = 25;
    }
/*
    if(l == 40)
      r = 20;
    else if(r == 40)
      l = 20;
    else if(r == 34)
    {
      l = 23;
      r = 37;
    }
    else if(l == 34)
    {
      l = 37;
      r = 23;
    }
    else
    {
      l = 30;
      r = 30;
    }
*/
  //  printf("Command Sent: (%d, %d)\n", l, r);

#ifndef SIMCTL
    p->sendChar('w');
    Sleep(100);

    p->sendChar('0'+ l/10);
    Sleep(100);
    p->sendChar('0'+ l%10);
    Sleep(100);
    p->sendChar('0'+ r/10);
    Sleep(100);
    p->sendChar('0'+ r%10);
    Sleep(100);
/*
    //Sleep(100);

    p->connect(UART_COMM_PORT, UART_BAUD_RATE, spNONE);
    Sleep(100);

    p->sendChar(' ');
    Sleep(100);

    p->disconnect();
    Sleep(100);
*/
#endif
  }

  void navCommandProcess(int vl, int vr, FILE *logFile)
  {
    int l = vl;
    int r = vr;
//    printf("Input Command: (%d, %d)\n", l, r);

    if(l > r)
    {
      l = 40;
      r = 25;
    }
    else if(l < r)
    {
      l = 18;
      r = 40;
      //l = 0;
      //r = 50;
    }
    else if((l != 0) || (r != 0))
    {
      l = 18;
      r = 25;
    }

    fprintf(logFile, "Command: (%2d, %2d) | ", l, r);
/*
    if(l == 40)
      r = 20;
    else if(r == 40)
      l = 20;
    else if(r == 34)
    {
      l = 23;
      r = 37;
    }
    else if(l == 34)
    {
      l = 37;
      r = 23;
    }
    else
    {
      l = 30;
      r = 30;
    }
*/
  //  printf("Command Sent: (%d, %d)\n", l, r);

#ifndef SIMCTL
    p->sendChar('w');
    Sleep(100);

    p->sendChar('0'+ l/10);
    Sleep(100);
    p->sendChar('0'+ l%10);
    Sleep(100);
    p->sendChar('0'+ r/10);
    Sleep(100);
    p->sendChar('0'+ r%10);
    Sleep(100);
/*
    //Sleep(100);

    p->connect(UART_COMM_PORT, UART_BAUD_RATE, spNONE);
    Sleep(100);

    p->sendChar(' ');
    Sleep(100);

    p->disconnect();
    Sleep(100);
*/
#endif
  }

  void navCommand(int vl, int vr)
  {
    pid_t pid;
    int status;

    if(waitpid(WAIT_ANY, &status, WNOHANG) == 0)
    {
      printf("Executing Last Command...\n");
    }
    else
    {
      pid = fork();
      if(pid == 0)
      {
        // Child
        navCommandProcess(vl, vr);
        exit(0);
      }
    }
  }

  /**
   *   New overloaded function with radius of curvature incorporated.
   */
  void navCommand(int vl, int vr, double state)
  {
    int l, r, m = 4;
    double c = state/10;  // Should be replaced by angular velocity/speed.
    //char speedData[9];

    m = transfer(vl, vr, c);

    l = (int) leftSpeedAt[m] * 100 / 255;
    r = (int) rightSpeedAt[m] * 100 / 255;

    printf("navCommand: (%f, %d, %d)\n", l, r);

#ifndef SIMCTL
    Tserial* p;
    p = new Tserial();
    p->connect(UART_COMM_PORT, UART_BAUD_RATE, spNONE);
    Sleep(100);
    p->sendChar('w');
    Sleep(100);
    p->sendChar('0'+ l/10);
    Sleep(100);
    p->sendChar('0'+ l%10);
    Sleep(100);
    p->sendChar('0'+ r/10);
    Sleep(100);
    p->sendChar('0'+ r%10);

    p->disconnect();
#endif
  }

  void initState(state *s)
  {
    s->x = 0;
    s->y = 0;
    s->theta = 0;
    s->g = 0;
    s->h = INFINITY;
    s->listType = UNDEFINED;
    s->l = NULL;
    s->vl = UNDEFINED;
    s->vr = UNDEFINED;
    s->px = 0;
    s->py = 0;
    s->wFlag = true;
  }

  void initGMap()
  {
    for(int i=0; i<MAP_MAX; i++)
      for(int j=0; j<MAP_MAX; j++)
        initState(map[i]+j);
  }

  state* loadPosData()                // Loads the BPS (Basic Path Set) from the file
  {
    double x, y, theta, g;
    int vl, vr;
    FILE *fp = fopen(BPS_SOURCE, "r");
    fscanf(fp, "%d\n", &nSeeds);
    state* np = (state *)malloc(nSeeds * sizeof(state));
    sPoints = (seedPath **)malloc(nSeeds * sizeof(seedPath *));

    for(int i=0; i<nSeeds; i++)
    {
      initState(np+i);

      fscanf(fp, "%d %d %lf %lf %lf %lf\n", &vl, &vr, &x, &y, &theta, &g);
      double t=1;
      (x<0)?(t=-1.5):(t=0.5);
      np[i].x = (int)(x+t);
      np[i].y = (int)(y+t);
      np[i].theta = theta;
      np[i].g = g;
      np[i].vl = vl;
      np[i].vr = vr;
      np[i].id = i;

      int nSeedLocs;
      fscanf(fp, "%d\n", &nSeedLocs);
      np[i].size = nSeedLocs;
      sPoints[i] = (seedPath *)malloc(nSeedLocs * sizeof(seedPath));
      for(int j=0; j<nSeedLocs; j++)
      {
        fscanf(fp, "%lf %lf\n", &x, &y);
        sPoints[i][j].x = (int)(x+t);
        sPoints[i][j].y = (int)(y+t);
      }
    }
    fclose(fp);

    return np;
  }

  state* loadPosData1()                // Loads the BPS (Basic Path Set) from the file
  {
    double x, y, theta, g, k;
    int vl, vr;
    FILE *fp = fopen(BPS_SOURCE, "r");
    fscanf(fp, "%d\n", &nSeeds);
    state* np = (state *)malloc(nSeeds * sizeof(state));
    sPoints = (seedPath **)malloc(nSeeds * sizeof(seedPath *));

    for(int i=0; i<nSeeds; i++)
    {
      initState(np+i);

      fscanf(fp, "%lf %lf %lf %lf %lf\n", &k, &x, &y, &theta, &g);
      double t=1;
      (x<0)?(t=-1.5):(t=0.5);
      np[i].x = (int)(x+t);
      np[i].y = (int)(y+t);
      np[i].theta = theta;
      np[i].g = g;
      np[i].vl = vl;
      np[i].vr = vr;
      np[i].k = k;
      np[i].id = i;

      int nSeedLocs;
      fscanf(fp, "%d\n", &nSeedLocs);
      np[i].size = nSeedLocs;
      sPoints[i] = (seedPath *)malloc(nSeedLocs * sizeof(seedPath));
      for(int j=0; j<nSeedLocs; j++)
      {
        fscanf(fp, "%lf %lf\n", &x, &y);
        sPoints[i][j].x = (int)(x+t);
        sPoints[i][j].y = (int)(y+t);
      }
    }
    fclose(fp);

    return np;
  }

  state* neighbours(state s, state *np1)
  {
    state *np = (state *)malloc(nSeeds*sizeof(state));
    for(int i=0; i<nSeeds; i++)
      np[i] = np1[i];

    for(int i = 0; i < nSeeds; i++) // Rotation and Translation
    {
      int tx, ty;
      tx = np[i].x; ty = np[i].y;

      np[i].x = (int)(tx*sin(s.theta*(CV_PI/180)) + ty*cos(s.theta*(CV_PI/180)) + s.x);
      np[i].y = (int)(-tx*cos(s.theta*(CV_PI/180)) + ty*sin(s.theta*(CV_PI/180)) + s.y);
      np[i].theta = np[i].theta - (90 - s.theta);

      np[i].px = s.x; np[i].py = s.y;
    }

    return np;
  }

  list* append(list *l, state c)          // Appends an element to a list
  {
    list *node = (list *)malloc(sizeof(list));
    node->x = c.x;  node->y = c.y;
    node->next = NULL;  node->prev = NULL;

    node->next = l;
    if(l != NULL)
      l->prev = node;
    return node;
  }

  bool isNear(state a, state b)
  {
    if((pow(a.x - b.x+0.0, 2) + pow(a.y - b.y+0.0, 2) < 1000))
      return true;
    else
      return false;
  }

  state findMin(list *l)
  {
    double min=MAXMIN;
    list *minList = NULL;
    state minState;
    initState(&minState);

    while(l != NULL)
    {
      double f = map[l->x][l->y].g + map[l->x][l->y].h;
      if(f <= min)
      {
        min = f;
        minList = append(minList, map[l->x][l->y]);
        //minState = map[l->x][l->y];
      }
      l = l->next;
    }

    //return minState;

    double maxG = 0;
    while(minList)
    {
      double f = map[minList->x][minList->y].g + map[minList->x][minList->y].h;
      if(f == min)
      {
        double g = map[minList->x][minList->y].g;
        if(g > maxG)
        {
          maxG = g;
          minState = map[minList->x][minList->y];
        }
      }
      list *tempList = minList;
      minList = minList->next;
      free(tempList);
    }
    return minState;
  }

  list* detach(list *l, state s)
  {
    list *t = map[s.x][s.y].l;
    if(t != NULL)
    {
      if(t->prev != NULL)
      {
        if(t->next != NULL)
        {
          list *tempList = t;
          t->prev->next = t->next;
          t->next->prev = t->prev;
          free(tempList);
          return l;
        }
        else
        {
          list *tempList = t;
          t->prev->next = NULL;
          free(tempList);
          return l;
        }
      }
      else
      {
        if(t->next != NULL)
        {
          list *tempList = t;
          t->next->prev = NULL;
          t = t->next;
          free(tempList);
          return t;
        }
        else
          return NULL;
      }
    }
    else
      return NULL;
  }

  void plotGmap()
  {
    for(int i=0; i<MAP_MAX; i++)
    {
      uchar* ptr = (uchar *)(mapImg->imageData + i*mapImg->widthStep);
      for(int j=0; j<MAP_MAX; j++)
      {
        if(map[j][MAP_MAX-i-1].wFlag == true)
        {
          ptr[3*j] = 0;
          ptr[3*j+1] = 0;
          ptr[3*j+2] = 0;
        }
        else
        {
          ptr[3*j] = 200;
          ptr[3*j+1] = 200;
          ptr[3*j+2] = 200;
        }
      }
    }

    list *tp = path;
    while(tp)
    {
      cvLine(mapImg, cvPoint(tp->x, MAP_MAX - tp->y), cvPoint(tp->x+1, MAP_MAX - (tp->y+1)), CV_RGB(255, 0, 0), 3, CV_AA, 0);
      tp = tp->next;
    }

    cvLine(mapImg, cvPoint(b.x, MAP_MAX - b.y), cvPoint(b.x+5, MAP_MAX - (b.y+5)), CV_RGB(0, 0, 255), 5, CV_AA, 0);
    cvLine(mapImg, cvPoint(t.x, MAP_MAX - t.y), cvPoint(t.x+5, MAP_MAX - (t.y+5)), CV_RGB(255, 0, 255), 5, CV_AA, 0);
    cvShowImage("GMap", mapImg);
  }

  double minVal(double a, double b)
  {
    if(a < b)
      return a;
    else
      return b;
  }

  int isWalkable(state s)
  {
    int x, y;
    double alpha = map[s.px][s.py].theta;
    for(int i=0; i<s.size; i++) // Rotation
    {
      int tx, ty;
      tx = sPoints[s.id][i].x;  ty = sPoints[s.id][i].y;

      x = (int)(tx*sin(alpha*(CV_PI/180)) + ty*cos(alpha*(CV_PI/180)) + s.px);
      y = (int)(-tx*cos(alpha*(CV_PI/180)) + ty*sin(alpha*(CV_PI/180)) + s.py);

      if(((0 <= x) && (x < MAP_MAX)) && ((0 <= y) && (y < MAP_MAX)))
      {
        if(map[x][y].wFlag == false)
          return false;
      }
      else
        return false;
    }

    return true;
  }

  void loadMap(char **gMap)
  {
    for(int i=0; i<MAP_MAX; i++)
    {
      for(int j=0; j<MAP_MAX; j++)
      {
        if(gMap[i][j] == 0)
        {
          map[i][j].wFlag = true;
        }
        else
        {
          map[i][j].wFlag = false;
        }
      }
    }

    IplImage *mapImg1 = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), IPL_DEPTH_8U, 3);
    for(int i=0; i<MAP_MAX; i++)
    {
      uchar* ptr = (uchar *)(mapImg1->imageData + i*mapImg1->widthStep);
      for(int j=0; j<MAP_MAX; j++)
      {
        if(map[j][MAP_MAX-i-1].wFlag == true)
        {
          ptr[3*j] = 0;
          ptr[3*j+1] = 0;
          ptr[3*j+2] = 0;
        }
        else
        {
          ptr[3*j] = 200;
          ptr[3*j+1] = 200;
          ptr[3*j+2] = 200;
        }
      }
    }

    //cvShowImage("Global Map GOT", mapImg1);
    cvReleaseImage(&mapImg1);
    //plotGmap();
  }

  void Nav::NavCore::loadNavigator()
  {
#ifdef SIMSEEDS
  seeds = loadPosData1();
#else
    seeds = loadPosData();
#endif

    extendSeedsFlag = 0;

    // Called by the master while initializing all modules. loadGMap() will be called in every iteration
    initGMap();

    N = (state *)malloc(nSeeds*sizeof(state));

    initState(&b);
    initState(&t);

    b.x = (int)(0.5*MAP_MAX);
    b.y = (int)(0.1*MAP_MAX);
    b.theta = 90;

#ifndef SIMCTL
    p = new Tserial();
    p->connect(UART_COMM_PORT, UART_BAUD_RATE, spNONE);
    Sleep(100);

    p->sendChar('w');
    Sleep(100);
#endif

    cvNamedWindow("GMap", CV_WINDOW_NORMAL);
    //cvNamedWindow("Global Map GOT", 0);
    mapImg = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), IPL_DEPTH_8U, 3);

  iteration = 0;

    printf("Nav Initiated\n");
  }

  double max(double a, double b)
  {
  if(a > b)
    return a;
  else
    return b;
  }

  void Nav::NavCore::navigate(char **gMap, CvPoint target)
  {
    iteration++;
#ifdef VIS
    IplImage *vis = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), IPL_DEPTH_8U, 3);
    cvNamedWindow("Visualization", CV_WINDOW_NORMAL);
#endif

    for(int i=0; i<MAP_MAX; i++)
    {
      for(int j=0; j<MAP_MAX; j++)
      {
        initState(map[i]+j);
      }
    }

    // Load target data: obtained from Environment
    loadMap(gMap);

#ifdef SIMOBS
    srand(time(0));
    for(int i = 0; i < 15; i++)
    {
      addObstacle(cvPoint(200 + rand()%500, 200 + rand()%500), 10 + rand()%40);
    }
    //plotGmap();
#endif

    t.x = target.x;
    t.y = target.y;
    t.theta = 90;
    bool flag=true;
    int xPrev=t.x;
    while(map[t.x][t.y].wFlag==false)
    {
      if(flag)
      {
        t.x += 2;
        if(t.x>999)
        {
          t.x=xPrev;
          flag=false;
        }
      }
      else
      {
        t.x -= 2;
        if(t.x<1)
        {
          t.x=xPrev;
          t.y++;
          flag=true;
        }
      }
    }

    if(isNear(b, t))
    {
    printf("Target Reached\n");
      return;
     }

    cl = ol = NULL;
    s = b;
    map[s.x][s.y] = s;

#ifdef SIMSEEDS
    nSeeds = 9;
#endif

    int count = 0;
    while(1)
    {
      cl = append(cl, s);
      map[s.x][s.y].listType = CLOSED;
      map[s.x][s.y].l = cl;

#ifdef VIS
      cvLine(vis, cvPoint(s.x, s.y), cvPoint(s.x+1, s.y), CV_RGB(255, 0, 0), 1, CV_AA, 0);
      cvShowImage("Visualization", vis);
#endif

      if(isNear(s, t))
        break;

      N = neighbours(s, seeds);
      for(int i=0; i<nSeeds; i++)
      {
        if(((0 <= N[i].x) && (N[i].x <= MAP_MAX-1)) && ((0 <= N[i].y) && (N[i].y <= MAP_MAX-1)))
        {
          if(isWalkable(N[i]))
          {
            if(map[N[i].x][N[i].y].listType == UNDEFINED)
            {
              ol = append(ol, N[i]);
              map[N[i].x][N[i].y] = N[i];
              map[N[i].x][N[i].y].listType = OPEN;
              map[N[i].x][N[i].y].g = map[s.x][s.y].g + N[i].g;
              map[N[i].x][N[i].y].h = sqrt(pow(N[i].x-t.x+0.0, 2)+pow(N[i].y-t.y+0.0, 2));
              map[N[i].x][N[i].y].h = max(map[N[i].x][N[i].y].h, map[s.x][s.y].h - N[i].g); // Consistent Heuristic
              map[N[i].x][N[i].y].l = ol;
#ifdef VIS
              cvLine(vis, cvPoint(N[i].x, N[i].y), cvPoint(N[i].x+1, N[i].y), CV_RGB(0, 255, 0), 1, CV_AA, 0);
#endif
            }
          }
        }
      }

#ifdef VIS
      cvShowImage("Visualization", vis);
      static int visFlag = 0;
      cvWaitKey(0);
#endif

      s = findMin(ol);
      ol = detach(ol, s);
#ifdef SIMSEEDS
      if(ol == NULL)
      {
        if(nSeeds < 36)
        {
      printf("Extending seeds to %d and retrying\n", nSeeds += 9);
      cl = ol = NULL;
      s = b;
      map[s.x][s.y] = s;
      map[s.x][s.y].listType = UNDEFINED;
      map[s.x][s.y].l = NULL;
      continue;
    }
    else
    {
      printf("No path found\n");
      navCommandProcess(0, 0);
      return;
      }
      }
      else
      {
      nSeeds = 9;
    }
#else
    if(ol == NULL)
    {
    printf("No path found\n");
    navCommandProcess(0, 0);
    return;
    }
#endif

      map[s.x][s.y].listType = UNDEFINED;
      map[s.x][s.y].l = NULL;

      count++;
      if(count % 1000 == 0)
      {
        //printf("count: %d\n", count);
      }
    }
free(N);
#ifdef VIS
    cvShowImage("Visualization", vis);
    cvWaitKey(0);
#endif

    //printf("Count: %d\n", count);
    path = NULL;
    state pState;

    pState = s;
    while(!((pState.x == b.x) && (pState.y == b.y)))
    {
      path = append(path, pState);
      pState = map[pState.px][pState.py];
    }

    plotGmap();

    list *nextMove = path;
    if(nextMove)
    {
#ifdef SIMSEEDS
      double vAvg = 70;
      int lVel = 2 * map[nextMove->x][nextMove->y].k / (map[nextMove->x][nextMove->y].k + 1) * vAvg;
      int rVel = 2 / (map[nextMove->x][nextMove->y].k + 1) * vAvg;
      navCommandProcess(lVel, rVel);
#else
      navCommandProcess(map[nextMove->x][nextMove->y].vl, map[nextMove->x][nextMove->y].vr);
#endif
    }
  }

  void Nav::NavCore::navigate(char **gMap, CvPoint target, char *logFileId, FILE *logFile, int frame_count)
  {
    iteration++;
#ifdef VIS
    IplImage *vis = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), IPL_DEPTH_8U, 3);
    cvNamedWindow("Visualization", CV_WINDOW_NORMAL);
#endif

    for(int i=0; i<MAP_MAX; i++)
    {
      for(int j=0; j<MAP_MAX; j++)
      {
        initState(map[i]+j);
      }
    }

    // Load target data: obtained from Environment
    loadMap(gMap);

    char mapFile[30];
    sprintf(mapFile, "Logs/[%02d]Log/[%04d]Map.jpg", atoi(logFileId), frame_count);
    //sprintf(mapFile, "[%02d]Log/[%04d]Map.jpg", atoi(logFileId), frame_count);
    cvSaveImage(mapFile, mapImg);

#ifdef SIMOBS
    srand(time(0));
    for(int i = 0; i < 15; i++)
    {
      addObstacle(cvPoint(200 + rand()%500, 200 + rand()%500), 10 + rand()%40);
    }
    //plotGmap();
#endif

    t.x = target.x;
    t.y = target.y;
    t.theta = 90;
    bool flag=true;
    int xPrev=t.x;
    while(map[t.x][t.y].wFlag==false)
    {
      if(flag)
      {
        t.x += 2;
        if(t.x>999)
        {
          t.x=xPrev;
          flag=false;
        }
      }
      else
      {
        t.x -= 2;
        if(t.x<1)
        {
          t.x=xPrev;
          t.y++;
          flag=true;
        }
      }
    }

    if(isNear(b, t))
    {
    printf("Target Reached\n");
      return;
     }

    cl = ol = NULL;
    s = b;
    map[s.x][s.y] = s;

#ifdef SIMSEEDS
    nSeeds = 9;
#endif

    int count = 0;
    while(1)
    {
      cl = append(cl, s);
      map[s.x][s.y].listType = CLOSED;
      map[s.x][s.y].l = cl;

#ifdef VIS
      cvLine(vis, cvPoint(s.x, s.y), cvPoint(s.x+1, s.y), CV_RGB(255, 0, 0), 1, CV_AA, 0);
      cvShowImage("Visualization", vis);
#endif

      if(isNear(s, t))
        break;

      N = neighbours(s, seeds);
      for(int i=0; i<nSeeds; i++)
      {
        if(((0 <= N[i].x) && (N[i].x <= MAP_MAX-1)) && ((0 <= N[i].y) && (N[i].y <= MAP_MAX-1)))
        {
          if(isWalkable(N[i]))
          {
            if(map[N[i].x][N[i].y].listType == UNDEFINED)
            {
              ol = append(ol, N[i]);
              map[N[i].x][N[i].y] = N[i];
              map[N[i].x][N[i].y].listType = OPEN;
              map[N[i].x][N[i].y].g = map[s.x][s.y].g + N[i].g;
              map[N[i].x][N[i].y].h = sqrt(pow(N[i].x-t.x+0.0, 2)+pow(N[i].y-t.y+0.0, 2));
              map[N[i].x][N[i].y].h = max(map[N[i].x][N[i].y].h, map[s.x][s.y].h - N[i].g); // Consistent Heuristic
              map[N[i].x][N[i].y].l = ol;
#ifdef VIS
              cvLine(vis, cvPoint(N[i].x, N[i].y), cvPoint(N[i].x+1, N[i].y), CV_RGB(0, 255, 0), 1, CV_AA, 0);
#endif
            }
          }
        }
      }

#ifdef VIS
      cvShowImage("Visualization", vis);
      static int visFlag = 0;
      cvWaitKey(0);
#endif

      s = findMin(ol);
      ol = detach(ol, s);
#ifdef SIMSEEDS
      if(ol == NULL)
      {
        if(nSeeds < 36)
        {
      printf("Extending seeds to %d and retrying\n", nSeeds += 9);
      cl = ol = NULL;
      s = b;
      map[s.x][s.y] = s;
      map[s.x][s.y].listType = UNDEFINED;
      map[s.x][s.y].l = NULL;
      continue;
    }
    else
    {
      printf("No path found\n");
      navCommandProcess(0, 0);
      return;
      }
      }
      else
      {
      nSeeds = 9;
    }
#else
    if(ol == NULL)
    {
    printf("No path found\n");
    navCommandProcess(0, 0);
    return;
    }
#endif

      map[s.x][s.y].listType = UNDEFINED;
      map[s.x][s.y].l = NULL;

      count++;
      if(count % 1000 == 0)
      {
        //printf("count: %d\n", count);
      }
    }
free(N);
#ifdef VIS
    cvShowImage("Visualization", vis);
    cvWaitKey(0);
#endif

    //printf("Count: %d\n", count);
    path = NULL;
    state pState;

    pState = s;
    while(!((pState.x == b.x) && (pState.y == b.y)))
    {
      path = append(path, pState);
      pState = map[pState.px][pState.py];
    }

    plotGmap();

    char pathFile[30];
    sprintf(pathFile, "Logs/[%02d]Log/[%04d]Path.jpg", atoi(logFileId), frame_count);
    //sprintf(pathFile, "[%02d]Log/[%04d]Path.jpg", atoi(logFileId), frame_count);
    cvSaveImage(pathFile, mapImg);

    list *nextMove = path;
    if(nextMove)
    {
#ifdef SIMSEEDS
      double vAvg = 70;
      int lVel = 2 * map[nextMove->x][nextMove->y].k / (map[nextMove->x][nextMove->y].k + 1) * vAvg;
      int rVel = 2 / (map[nextMove->x][nextMove->y].k + 1) * vAvg;

      navCommandProcess(lVel, rVel);
#else
      navCommandProcess(map[nextMove->x][nextMove->y].vl, map[nextMove->x][nextMove->y].vr, logFile);
#endif
    }
  }

  void Nav::NavCore::navigate(char **gMap, CvPoint target, double yaw)
  {
    for(int i=0; i<MAP_MAX; i++)
    {
      for(int j=0; j<MAP_MAX; j++)
      {
        initState(map[i]+j);
      }
    }

    // Load target data: obtained from IP
    loadMap(gMap);
    addObstacle(cvPoint(500, 500), 50);

    t.x = target.x;
    t.y = target.y;
    t.theta = 90;

    cl = ol = NULL;
    s = b;
    map[s.x][s.y] = s;

#ifdef SIMSEEDS
    nSeeds = 9;
#endif

    int count = 0;
    while(1)
    {
      cl = append(cl, s);
      map[s.x][s.y].listType = CLOSED;
      map[s.x][s.y].l = cl;

      if(isNear(s, t))
        break;

      N = neighbours(s, seeds);
      for(int i=0; i<nSeeds; i++)
      {
        if(((0 <= N[i].x) && (N[i].x <= MAP_MAX-1)) && ((0 <= N[i].y) && (N[i].y <= MAP_MAX-1)))
        {
          if(isWalkable(N[i]))
          {
            if(map[N[i].x][N[i].y].listType == UNDEFINED)
            {
              ol = append(ol, N[i]);
              map[N[i].x][N[i].y] = N[i];
              map[N[i].x][N[i].y].listType = OPEN;
              map[N[i].x][N[i].y].g = map[s.x][s.y].g + N[i].g;
              map[N[i].x][N[i].y].h = sqrt(pow(N[i].x-t.x+0.0, 2)+pow(N[i].y-t.y+0.0, 2));
              map[N[i].x][N[i].y].l = ol;
            }
          }
        }
      }
      s = findMin(ol);
      ol = detach(ol, s);
      if(ol == NULL)
      {
#ifdef SIMSEEDS
        if(nSeeds < 36)
        {
          nSeeds += 9;
          printf("Extending Seeds to %d\n", nSeeds);
          extendSeedsFlag = 1;
        }
        else
        {
      printf("No path found\n");
      return;
    }
#else
        printf("No path found\n");
        return;
#endif
      }
#ifdef SIMSEEDS
      else if(extendSeedsFlag == 1)
      {
    printf("Resetting Seeds\n");
    extendSeedsFlag = 0;
        nSeeds = 9;
      }
#endif

      map[s.x][s.y].listType = UNDEFINED;
      map[s.x][s.y].l = NULL;

      count++;
      if(count > 1000)
      {
        printf("Counter Overflow\n");
        return;
      }
    }

    printf("Count: %d\n", count);
    path = NULL;
    state pState;

    pState = s;
    while(!((pState.x == b.x) && (pState.y == b.y)))
    {
      path = append(path, pState);
      pState = map[pState.px][pState.py];
    }

    plotGmap();

    list *nextMove = path;
    if(nextMove)
    {
      printf("nextMove: (%d, %d)\n", nextMove->x, nextMove->y);
      navCommand(map[nextMove->x][nextMove->y].vl, map[nextMove->x][nextMove->y].vr, yaw-previousYaw);
      previousYaw=yaw;
    }
  }
  void Nav::NavCore::closeNav()
  {
#ifndef SIMCTL
    p->sendChar(' ');
    Sleep(100);

    p->disconnect();
    Sleep(100);
#endif
  }
}
