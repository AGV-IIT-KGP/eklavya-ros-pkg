#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <stdexcept>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include "cv.h"
#include "highgui.h"
#include "PathPlanner.h"
#include "../devices.h"
#include "../../eklavya2.h"
#include "../../Utils/SerialPortLinux/serial_lnx.h"

#define UNDEFINED -1
#define INFINITY -2
#define CLOSED 0
#define OPEN 1
#define MAXMIN 999999
#define MAP_MAX 1000

//#define VIS
#define SIMCTL
//#define SIMOBS
//#define SIMSEEDS
//#define YAWPID

#ifdef SIMSEEDS
#define BPS_SOURCE "../src/Modules/Planner/seeds.txt"
#else
#define BPS_SOURCE "../src/Modules/Planner/seeds3.txt"
#endif

using namespace std;

namespace Nav {
  typedef struct list {
    int x, y;
    list *next, *prev;
  } list;

  typedef struct state {
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
  } state;

  typedef struct obstacle {
    int x, y, r;
  } obstacle;

  typedef struct seedPath {
    int x, y;
  } seedPath;

  int nSeeds;
  state map[MAP_MAX][MAP_MAX];
  list *cl, *ol, *path;
  seedPath **sPoints;
  IplImage *mapImg;

  state s, b, t, *seeds, *N;
  int count = 0;
  Tserial *p;

  void addObstacle(CvPoint ob, int rad) {
    for (int i = -rad; i < rad; i++) {
      for (int j = -rad; j < rad; j++) {
        if (i * i + j * j <= rad * rad) {
          map[ob.x + i][ob.y + j].wFlag = false;
        }
      }
    }
  }

  int transfer(int leftVelocity, int rightVelocity, double currentCurvature) {
    int mode = 8, Kp = 5;
    double velocityRatioAt[9] = {0.45, 0.54, 0.67, 0.84, 1, 1.19, 1.5, 1.84, 2.22};
    double targetVelocityRatio = (double)leftVelocity / rightVelocity;
    double targetCurvature = 2 * (targetVelocityRatio - 1) / (targetVelocityRatio + 1);

    //cout<<"Allow1 ("<<leftVelocity<<","<<rightVelocity<<") "<<targetCurvature<<","<<currentCurvature<<endl;

    for (int i = 0; i < 8; i++) {
      if (targetVelocityRatio < velocityRatioAt[8 - i]) {
        mode--;
      } else {
        break;
      }
    }

    //cout<<"Allow2 "<<mode<<" "<<targetVelocityRatio<<" "<<velocityRatioAt[8]<<endl;

    mode += (int) (Kp * (targetCurvature - currentCurvature));

    if(mode<0)
      mode=0;
    if(mode>8)
      mode=8;

    return mode;
  }

  void sendCommand(int leftVelocityIn, int rightVelocityIn, double omega) {
    int leftSpeedAt[9]  = {18, 19, 20, 21, 18, 26, 30, 34, 38};
    int rightSpeedAt[9] = {38, 34, 30, 26, 21, 22, 21, 20, 19};

    for (int i = 0; i < 9; i++) {
      leftSpeedAt[i] = (int) leftSpeedAt[i] * 1;
      rightSpeedAt[i] = (int) rightSpeedAt[i] * 1;
    }

    int leftVelocity, rightVelocity, mode = 4;
    double curvature = omega / 10;  // Should be replaced by angular velocity/speed.

    mode = transfer(leftVelocityIn, rightVelocityIn, curvature);

    leftVelocity = leftSpeedAt[mode];
    rightVelocity = rightSpeedAt[mode];

    //cout<<"Exact\t("<<leftVelocity<<","<<rightVelocity<<")\t---->\t"<<mode<<"\t"<<omega<<endl;

    #ifndef SIMCTL
      p->sendChar('w');
      usleep(100);

      p->sendChar('0' + leftVelocity / 10);
      usleep(100);
      p->sendChar('0' + leftVelocity % 10);
      usleep(100);
      p->sendChar('0' + rightVelocity / 10);
      usleep(100);
      p->sendChar('0' + rightVelocity % 10);
      usleep(100);
    #endif
  }

  void sendCommand(int leftVelocity, int rightVelocity) {
  int leftVel=0,rightVel=0;
    if (leftVelocity > rightVelocity) {
      leftVel = 34;
      rightVel = 22;
    } else if (leftVelocity < rightVelocity) {
      leftVel = 20;
      rightVel = 35;
    } else if ((leftVelocity != 0) || (rightVelocity != 0)) {
      leftVel = 22;
      rightVel = 25;
    }

	//printf("Velocity: (%d, %d)\n", leftVel, rightVel);

    #ifndef SIMCTL
      p->sendChar('w');
      usleep(100);

      p->sendChar('0' + leftVel / 10);
      usleep(100);
      p->sendChar('0' + leftVel % 10);
      usleep(100);
      p->sendChar('0' + rightVel / 10);
      usleep(100);
      p->sendChar('0' + rightVel % 10);
      usleep(100);
    #endif
  }

  void initState(state *s) {
    s->x = 0;
    s->y = 0;
    s->theta = 0;
    s->wFlag = true;
    s->g = 0;
    s->h = INFINITY;
    s->listType = UNDEFINED;
    s->l = NULL;
    s->px = 0;
    s->py = 0;
    s->vl = UNDEFINED;
    s->vr = UNDEFINED;
    s->k = UNDEFINED;
    s->id = UNDEFINED;
    s->size = 0;
  }

  void initGMap() {
    for (int i=0; i<MAP_MAX; i++) {
      for (int j=0; j<MAP_MAX; j++) {
        initState(map[i]+j);
      }
    }
  }

  #ifdef SIMSEEDS
    state* loadPosData() {
      double x, y, theta, g, k;
      FILE *fp = fopen(BPS_SOURCE, "r");
      fscanf(fp, "%d\n", &nSeeds);
      state* np = (state *)malloc(nSeeds * sizeof(state));
      sPoints = (seedPath **)malloc(nSeeds * sizeof(seedPath *));

      for (int i = 0; i < nSeeds; i++) {
        initState(np + i);

        fscanf(fp, "%lf %lf %lf %lf %lf\n", &k, &x, &y, &theta, &g);
        double t = 1;
        (x < 0) ? (t = -1.5) : (t = 0.5);
        np[i].x = (int)(x + t);
        np[i].y = (int)(y + t);
        np[i].theta = theta;
        np[i].g = g;
        np[i].k = k;
        np[i].id = i;

        int nSeedLocs;
        fscanf(fp, "%d\n", &nSeedLocs);
        np[i].size = nSeedLocs;
        sPoints[i] = (seedPath *)malloc(nSeedLocs * sizeof(seedPath));
        for (int j = 0; j < nSeedLocs; j++) {
          fscanf(fp, "%lf %lf\n", &x, &y);
          sPoints[i][j].x = (int)(x + t);
          sPoints[i][j].y = (int)(y + t);
        }
      }
      fclose(fp);

      return np;
    }
  #else
  state* loadPosData() {
      double x, y, theta, g;
      int vl, vr;
      FILE *fp = fopen(BPS_SOURCE, "r");
      fscanf(fp, "%d\n", &nSeeds);
      state* np = (state *)malloc(nSeeds * sizeof(state));
      sPoints = (seedPath **)malloc(nSeeds * sizeof(seedPath *));

      for (int i = 0; i < nSeeds; i++) {
        initState(np + i);

        fscanf(fp, "%d %d %lf %lf %lf %lf\n", &vl, &vr, &x, &y, &theta, &g);
        double t = 1;
        (x < 0) ? (t = -1.5) : (t = 0.5);
        np[i].x = (int)(x + t);
        np[i].y = (int)(y + t);
        np[i].theta = theta;
        np[i].g = g;
        np[i].vl = vl;
        np[i].vr = vr;
        np[i].id = i;

        int nSeedLocs;
        fscanf(fp, "%d\n", &nSeedLocs);
        np[i].size = nSeedLocs;
        sPoints[i] = (seedPath *)malloc(nSeedLocs * sizeof(seedPath));
        for (int j = 0; j < nSeedLocs; j++) {
          fscanf(fp, "%lf %lf\n", &x, &y);
          sPoints[i][j].x = (int)(x + t);
          sPoints[i][j].y = (int)(y + t);
        }
      }
      fclose(fp);

      return np;
    }
  #endif

  state* neighbours(state s, state *np1) {
    state *np = (state *)malloc(nSeeds * sizeof(state));
    for (int i = 0; i < nSeeds; i++) {
      np[i] = np1[i];
    }

    for (int i = 0; i < nSeeds; i++) {
      int tx, ty;
      tx = np[i].x; ty = np[i].y;

      np[i].x = (int)(tx * sin(s.theta * (CV_PI / 180)) + ty * cos(s.theta * (CV_PI / 180)) + s.x);
      np[i].y = (int)(-tx * cos(s.theta * (CV_PI / 180)) + ty * sin(s.theta * (CV_PI / 180)) + s.y);
      np[i].theta = np[i].theta - (90 - s.theta);

      np[i].px = s.x; np[i].py = s.y;
    }

    return np;
  }

  list* append(list *l, state c) {
    list *node = (list *)malloc(sizeof(list));
    node->x = c.x;  node->y = c.y;
    node->next = NULL;  node->prev = NULL;

    node->next = l;
    if (l != NULL) {
      l->prev = node;
    }

    return node;
  }

  bool isNear(state a, state b) {
    if ((pow(a.x - b.x + 0.0, 2) + pow(a.y - b.y + 0.0, 2) < 1000)) {
      return true;
    } else {
      return false;
    }
  }

  state findMin(list *l) {
    double min = MAXMIN;
    list *minList = NULL;
    state minState;
    initState(&minState);

    while (l != NULL) {
      double f = map[l->x][l->y].g + map[l->x][l->y].h;
      if (f <= min) {
        min = f;
        minList = append(minList, map[l->x][l->y]);
      }
      l = l->next;
    }

    double maxG = 0;
    while (minList) {
      double f = map[minList->x][minList->y].g + map[minList->x][minList->y].h;
      if (f == min) {
        double g = map[minList->x][minList->y].g;
        if (g > maxG) {
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

  list* detach(list *l, state s) {
    list *t = map[s.x][s.y].l;
    if (t != NULL) {
      if (t->prev != NULL) {
        if (t->next != NULL) {
          list *tempList = t;
          t->prev->next = t->next;
          t->next->prev = t->prev;
          free(tempList);
          return l;
        } else {
          list *tempList = t;
          t->prev->next = NULL;
          free(tempList);
          return l;
        }
      } else {
        if (t->next != NULL) {
          list *tempList = t;
          t->next->prev = NULL;
          t = t->next;
          free(tempList);
          return t;
        } else {
          return NULL;
        }
      }
    } else {
      return NULL;
    }
  }

  void plotGmap() {
    for (int i = 0; i < MAP_MAX; i++) {
      uchar* ptr = (uchar *)(mapImg->imageData + i * mapImg->widthStep);
      for (int j = 0; j < MAP_MAX; j++) {
        if (map[j][MAP_MAX - i - 1].wFlag == true) {
          ptr[3 * j] = 0;
          ptr[3 * j + 1] = 0;
          ptr[3 * j + 2] = 0;
        } else {
          ptr[3 * j] = 200;
          ptr[3 * j + 1] = 200;
          ptr[3 * j + 2] = 200;
        }
      }
    }

    list *tp = path;
    while(tp) {
      cvLine(mapImg, cvPoint(tp->x, MAP_MAX - tp->y), cvPoint(tp->x + 1, MAP_MAX - (tp->y + 1)), CV_RGB(255, 0, 0), 3, CV_AA, 0);
      tp = tp->next;
    }

    cvLine(mapImg, cvPoint(b.x, MAP_MAX - b.y), cvPoint(b.x, MAP_MAX - (b.y + 7)), CV_RGB(0, 0, 255), 5, CV_AA, 0);
    cvLine(mapImg, cvPoint(t.x, MAP_MAX - t.y), cvPoint(t.x, MAP_MAX - (t.y + 7)), CV_RGB(255, 0, 255), 5, CV_AA, 0);

    cout << "!!!!!!! Plotting" << endl;
    cvShowImage("GMap", mapImg);
    cvWaitKey(1);
  }

  void LogMap(char *fileName) {
    IplImage *image = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), IPL_DEPTH_8U, 3);

    for (int i = 0; i < MAP_MAX; i++) {
      uchar* ptr = (uchar *)(image->imageData + i * image->widthStep);
      for (int j = 0; j < MAP_MAX; j++) {
        if (map[j][MAP_MAX - i - 1].wFlag == true) {
          ptr[3 * j] = 0;
          ptr[3 * j + 1] = 0;
          ptr[3 * j + 2] = 0;
        } else {
          ptr[3 * j] = 200;
          ptr[3 * j + 1] = 200;
          ptr[3 * j + 2] = 200;
        }
      }
    }

    cvLine(image, cvPoint(b.x, MAP_MAX - b.y), cvPoint(b.x + 5, MAP_MAX - (b.y + 5)), CV_RGB(0, 0, 255), 5, CV_AA, 0);
    cvLine(image, cvPoint(t.x, MAP_MAX - t.y), cvPoint(t.x + 5, MAP_MAX - (t.y + 5)), CV_RGB(255, 0, 255), 5, CV_AA, 0);

    cvSaveImage(fileName, image);
  }

  void LogPath(char *fileName) {
    IplImage *image = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), IPL_DEPTH_8U, 3);

    for (int i = 0; i < MAP_MAX; i++) {
      uchar* ptr = (uchar *)(image->imageData + i * image->widthStep);
      for (int j = 0; j < MAP_MAX; j++) {
        if (map[j][MAP_MAX - i - 1].wFlag == true) {
          ptr[3 * j] = 0;
          ptr[3 * j + 1] = 0;
          ptr[3 * j + 2] = 0;
        } else {
          ptr[3 * j] = 200;
          ptr[3 * j + 1] = 200;
          ptr[3 * j + 2] = 200;
        }
      }
    }

    list *tp = path;
    while(tp) {
      cvLine(image, cvPoint(tp->x, MAP_MAX - tp->y), cvPoint(tp->x + 1, MAP_MAX - (tp->y + 1)), CV_RGB(255, 0, 0), 3, CV_AA, 0);
      tp = tp->next;
    }

    cvLine(image, cvPoint(b.x, MAP_MAX - b.y), cvPoint(b.x + 5, MAP_MAX - (b.y + 5)), CV_RGB(0, 0, 255), 5, CV_AA, 0);
    cvLine(image, cvPoint(t.x, MAP_MAX - t.y), cvPoint(t.x + 5, MAP_MAX - (t.y + 5)), CV_RGB(255, 0, 255), 5, CV_AA, 0);

    cvSaveImage(fileName, image);
  }

  void plot(char *name) {
    IplImage *img = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), IPL_DEPTH_8U, 3);
    for (int i = 0; i < MAP_MAX; i++) {
      uchar* ptr = (uchar *)(img->imageData + i * img->widthStep);
      for (int j = 0; j < MAP_MAX; j++) {
        if (map[j][MAP_MAX - i - 1].wFlag == true) {
          ptr[3 * j] = 0;
          ptr[3 * j + 1] = 0;
          ptr[3 * j + 2] = 0;
        } else {
          ptr[3 * j] = 200;
          ptr[3 * j + 1] = 200;
          ptr[3 * j + 2] = 200;
        }
      }
    }

    cvLine(img, cvPoint(b.x, MAP_MAX - b.y), cvPoint(b.x + 5, MAP_MAX - (b.y + 5)), CV_RGB(0, 0, 255), 5, CV_AA, 0);
    cvLine(img, cvPoint(t.x, MAP_MAX - t.y), cvPoint(t.x + 5, MAP_MAX - (t.y + 5)), CV_RGB(255, 0, 255), 5, CV_AA, 0);
    cvNamedWindow(name, CV_WINDOW_NORMAL);
    cvShowImage(name, img);
    cvWaitKey(1);
    cvReleaseImage(&img);
  }

  int isWalkable(state s) {
    int x, y;
    double alpha = map[s.px][s.py].theta;
    for (int i = 0; i < s.size; i++) {
      int tx, ty;
      tx = sPoints[s.id][i].x;
      ty = sPoints[s.id][i].y;

      x = (int)(tx * sin(alpha * (CV_PI / 180)) + ty * cos(alpha * (CV_PI / 180)) + s.px);
      y = (int)(-tx * cos(alpha * (CV_PI / 180)) + ty * sin(alpha * (CV_PI / 180)) + s.py);

      if (((0 <= x) && (x < MAP_MAX)) && ((0 <= y) && (y < MAP_MAX))) {
        if (map[x][y].wFlag == false) {
          return false;
        }
      } else {
        return false;
      }
    }

    return true;
  }

  void loadMap() {
    for (int i = 0; i < MAP_MAX; i++) {
      for (int j = 0; j < MAP_MAX; j++) {
        if (global_map[i][j] == 0) {
          map[i][j].wFlag = true;
        } else {
          map[i][j].wFlag = false;
        }
      }
    }
  }

  void Nav::NavCore::loadNavigator() {
    seeds = loadPosData();

    initGMap();

    N = (state *)malloc(nSeeds * sizeof(state));

    initState(&b);
    initState(&t);

    b.x = (int)(0.5 * MAP_MAX);
    b.y = (int)(0.1 * MAP_MAX);
    b.theta = 90;

    #ifndef SIMCTL
      p = new Tserial();
      p->connect(BOT_COM_PORT, BOT_BAUD_RATE, spNONE);
      usleep(100);

      p->sendChar('w');
      usleep(100);
    #endif

    cvNamedWindow("GMap", CV_WINDOW_NORMAL);
    mapImg = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), IPL_DEPTH_8U, 3);
  }

  double max(double a, double b) {
    if (a > b) {
      return a;
    } else {
      return b;
    }
  }

  int Nav::NavCore::navigate(Triplet target, int frame_count, double yaw) {
    #ifdef VIS
      IplImage *vis = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), IPL_DEPTH_8U, 3);
      cvNamedWindow("Visualization", CV_WINDOW_NORMAL);
    #endif

    for (int i = 0; i < MAP_MAX; i++) {
      for (int j = 0; j < MAP_MAX; j++) {
        initState(map[i] + j);
      }
    }

    loadMap();
    
    #ifdef SIMOBS
      srand(time(0));
      for (int i = 0; i < 15; i++) {
        addObstacle(cvPoint(200 + rand() % 700, 200 + rand() % 700), 10 + rand() % 90);
      }
    #endif

    t.x = target.x;
    t.y = target.y;
    t.theta = 90;
    bool flag = true;
    int xPrev = t.x;
    while (map[t.x][t.y].wFlag == false) {
      if (flag) {
        t.x += 2;
        if (t.x > 999) {
          t.x = xPrev;
          flag = false;
        }
      } else {
        t.x -= 2;
        if (t.x < 1) {
          t.x = xPrev;
          t.y++;
          flag = true;
        }
      }
    }

    if (isNear(b, t)) {
      printf("Target Reached\n");
      sendCommand(0, 0);
      return 0;
    }

    cl = ol = NULL;
    s = b;
    map[s.x][s.y] = s;

    #ifdef SIMSEEDS
      nSeeds = 36;
    #endif

    int count = 0;
    static double previousYaw = 0;
    while(1) {
      cl = append(cl, s);
      map[s.x][s.y].listType = CLOSED;
      map[s.x][s.y].l = cl;

      #ifdef VIS
        cvLine(vis, cvPoint(s.x, s.y), cvPoint(s.x + 3, s.y), CV_RGB(255, 0, 0), 3, CV_AA, 0);
        cvShowImage("Visualization", vis);
      #endif

      if (isNear(s, t)) {
        break;
      }

      N = neighbours(s, seeds);
      for (int i = 0; i < nSeeds; i++) {
        if (((0 <= N[i].x) && (N[i].x <= MAP_MAX - 1)) && ((0 <= N[i].y) && (N[i].y <= MAP_MAX - 1))) {
          if (isWalkable(N[i])) {
            if (map[N[i].x][N[i].y].listType == UNDEFINED) {
              ol = append(ol, N[i]);
              map[N[i].x][N[i].y] = N[i];
              map[N[i].x][N[i].y].listType = OPEN;
              map[N[i].x][N[i].y].g = map[s.x][s.y].g + N[i].g;
              map[N[i].x][N[i].y].h = sqrt(pow(N[i].x - t.x + 0.0, 2) + pow(N[i].y - t.y + 0.0, 2));
              map[N[i].x][N[i].y].h = max(map[N[i].x][N[i].y].h, map[s.x][s.y].h - N[i].g); // Consistent Heuristic
              map[N[i].x][N[i].y].l = ol;

              #ifdef VIS
                cvLine(vis, cvPoint(N[i].x, N[i].y), cvPoint(N[i].x + 3, N[i].y), CV_RGB(0, 255, 0), 3, CV_AA, 0);
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
        if (ol == NULL) {
          if (nSeeds < 36) {
            printf("Extending seeds to %d and retrying\n", nSeeds += 9);
            cl = ol = NULL;
            s = b;
            map[s.x][s.y] = s;
            map[s.x][s.y].listType = UNDEFINED;
            map[s.x][s.y].l = NULL;
            continue;
          } else {
            printf("No path found\n");
            sendCommand(0, 0);
            return 0;
          }
        } else {
          nSeeds = 9;
        }
      #else
        if (ol == NULL) {
          printf("No path found\n");
          sendCommand(0, 0);
          return 0;
        }
      #endif

      map[s.x][s.y].listType = UNDEFINED;
      map[s.x][s.y].l = NULL;

      count++;
      if (count % 5000 == 0) {
        if (count != 0) {
          printf("Counter Overflow\n");
          sendCommand(0, 0);
          return 1;
        }
      }
    }

    free(N);

    #ifdef VIS
      cvShowImage("Visualization", vis);
      cvWaitKey(0);
    #endif

    path = NULL;
    state pState;

    pState = s;
    while (!((pState.x == b.x) && (pState.y == b.y))) {
      path = append(path, pState);
      pState = map[pState.px][pState.py];
    }

    //plotGmap();

    list *nextMove = path;
    if (nextMove) {
      int lVel, rVel;

      #ifdef SIMSEEDS
        double vAvg = 70;
        lVel = 2 * map[nextMove->x][nextMove->y].k / (map[nextMove->x][nextMove->y].k + 1) * vAvg;
        rVel = 2 / (map[nextMove->x][nextMove->y].k + 1) * vAvg;
      #else
        lVel = map[nextMove->x][nextMove->y].vl;
        rVel = map[nextMove->x][nextMove->y].vr;
      #endif

      #ifdef YAWPID
    //cout<<"Yaw="<<yaw<<" Previous Yaw="<<previousYaw<<endl;
        sendCommand(lVel, rVel, yaw - previousYaw);
        previousYaw = yaw;
      #else
        sendCommand(lVel, rVel);
      #endif
    }

    while (cl) {
      list *temp = cl;
      cl = cl->next;
      free(temp);
    }

    return 0;
  }

  void Nav::NavCore::closeNav() {
    #ifndef SIMCTL
      p->sendChar(' ');
      usleep(100);

      p->disconnect();
      usleep(100);
    #endif
  }
}
