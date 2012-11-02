/*
1. Make the path work in any state configuration
2. Use G values to better it
3. Use only one data structure - state
4. Use integer values for state - correspondingly, we can choose a scaling factor like cm, meters etc.
5. Follow the A* algo line by line
*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "cv.h"
#include "highgui.h"
#include "Stereo.h"
#include "SDL.h"
#include "SDL_net.h"
#include "SDL_image.h"
#include <iostream>
#include <cmath>
#include <fstream>
#include <sstream>

#define MAP_MAX 1000
#define expansionSize 30

#define GridLength 40                       //aster
#define GridWidth  20
#define sq_size   0.25
#define left_lane 2.5          
#define right_lane 2.5
#define walkable 0
#define unwalkable 1
#define SHUTTER_TIME  12.61
#define GAIN  6.02
#define EXPOSURE  1.05
#define BRIGHTNESS 1.07
#define R 40
#define C 20
#define ground 0.9

CvPoint3D32f obstacle_centroids[100000];
float minx[100000];
float maxx[100000];
double x_sum,y_sum,z_sum;
long i_sum,j_sum;
int obstacle_num =0;

//didnt use these funcns
//IplImage* find_obstacles(IplImage* disp_img,float arr_x[240][320],float arr_y[240][320],float arr_z[240][320]);
//void obstacle(IplImage* dummy,float arr_x[240][320],float arr_y[240][320],float arr_z[240][320], int i,int j);

// maximum transmission unit
#define MTU 2048
// maximum image size
#define MAXIMGSIZE 20480
// end of image marker
#define EOI_BYTE1 -1
#define EOI_BYTE2 -39
// ip address and port number
#define IPADDR "192.168.0.15"
#define PORTLEFT 10001
#define PORTRIGHT 10002
// network commands timeout
#define TIMEOUT 2000

using namespace std;

int parserState;
SDL_Surface *screen1,*screen2;
TCPsocket tcpsock1,tcpsock2;

int weightofgrids[R][C];
#define A 15*3.14/180
#define clr .10

#define OBSTACLE_THRESHOLD 100
// stereo variables end
int flag = 0;

//stereo images
IplImage *leftStereoImage;
IplImage *rightStereoImage;

namespace Stereospace
{
  int Parser(char* buf, int size, char& imageSize)
  {
    for (int idx=0; idx<size; idx++) {
      bool failed = true;
      switch(parserState) {
        case 0:
        case 1:
          if (buf[idx] == '#') {
            parserState += 1;
            failed = false;
          }
          break;
        case 2:
          if (buf[idx] == 'I') {
            parserState += 1;
            failed = false;
          }
          break;
        case 3:
          if (buf[idx] == 'M') {
            parserState += 1;
            failed = false;
          }
          break;
        case 4:
          if (buf[idx] == 'J') {
            parserState += 1;
            failed = false;
          }
          break;
        case 5:
          imageSize = buf[idx]-48;
        case 6:
        case 7:
        case 8:
        case 9:
          failed = false;
          parserState += 1;
          break;
      }
      if (failed) {
        parserState = 0;
        return 0;
      } else if (parserState == 10) {
        parserState = 0;
        return (idx+1);
      }
    }
    return 0;
  }
  void ShowFrame(char* imageBuf, size_t size, SDL_Surface* screen) 
  {
    SDL_RWops *rwop;
    rwop = SDL_RWFromMem(imageBuf, (int)size);
    SDL_Surface *image  = IMG_LoadJPG_RW(rwop);
    SDL_BlitSurface(image, NULL, screen, NULL);
    SDL_FreeSurface(image);
    SDL_Flip(screen);
    SDL_FreeRW(rwop);
  }
  bool HandleEvents(string& commandsList, bool& waitForImage,bool& archive) 
  {
    commandsList.clear();
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      switch (event.type) {
      case SDL_KEYDOWN:
        switch (event.key.keysym.sym) {
        case SDLK_ESCAPE:
          return true;
          break;
            case SDLK_l:
          commandsList = "l";
          break;
            case SDLK_o:
          commandsList = "L";
          break;
            case SDLK_UP:
          commandsList += "M"+char(0x7F)+char(0x7F)+char(0x0);
          break;
            case SDLK_DOWN:
          commandsList += "M"+char(0x81)+char(0x81)+char(0x0);
          break;
            case SDLK_END:
          commandsList = "5";
          break;
            case SDLK_LEFT:
          commandsList += "M"+char(0x0)+char(0x7F)+char(0x0);
          break;
            case SDLK_RIGHT:
          commandsList += "M"+char(0x81)+char(0xFF)+char(0x0);
          break;
            case SDLK_a:
          archive = true;
          break;
            case SDLK_s:
          archive = false;
          break;
        default:
          break;
            }
        break;
        case SDL_QUIT:
        return true;
        break;
      }
    }
    waitForImage = false;
    if (commandsList.size() == 0) {
      commandsList = "I";
      waitForImage = true;
    }
    return false;
  }
  void convertStereo2Iplimage(IplImage *img,char *stereoimg)
  {
    int i,j;
    char *data = img->imageData;
    for(i=0;i<img->height;i++)
    {
      for(j=0;j<img->width;j++)
      {
        data[i*img->width+j] = stereoimg[i*img->width+j];
      }
    }
  }
  
  int ReceiveImage(TCPsocket tcpsock, SDL_Surface** screen,SDLNet_SocketSet socSet, char& imageSize,bool archive)
  {
    bool imageReady = false;
    bool imageStarted = false;
    char buf[MTU];
    char imageBuf[MAXIMGSIZE];
    int index = 0;

    for (; !imageReady;) {
      if (SDLNet_CheckSockets(socSet, TIMEOUT) == 0) {
        cout<<"SDLNet_CheckSockets: timeout"<<endl;
        return -1;
      }

      int result = SDLNet_TCP_Recv(tcpsock, buf, MTU);
      int tempIdx = 0;
      if (!imageStarted)
      {
        char tmpSize = 0;
        tempIdx = Parser(buf, result, tmpSize);
            
        if (tempIdx){
          imageStarted = true;
          if (tmpSize && (imageSize != tmpSize))
          {
            (*screen) = SDL_SetVideoMode(80*pow(2.0,(tmpSize/2)),64*pow(2.0,(tmpSize/2)), 16,SDL_DOUBLEBUF|SDL_HWSURFACE);
            imageSize = tmpSize;
          }

        }
      }
      if (imageStarted) 
      {
        memcpy(imageBuf+index, buf+tempIdx, result-tempIdx);
        index += (result-tempIdx);
        // check for EOI marker
        if ((buf[result-2] == EOI_BYTE1) && (buf[result-1] == EOI_BYTE2))
        {
          imageReady = true;
        }
      }
    }

    if (archive) 
    {
      ostringstream ssFileName;
      ssFileName<<"img_"<<SDL_GetTicks()<<".jpg";
      ofstream ofile(ssFileName.str().c_str(),ios_base::binary);
      ofile.write(imageBuf, index);
    }
    convertStereo2Iplimage(leftStereoImage,imageBuf);
    cvShowImage("LeftStereoImage",leftStereoImage);
    // decode the frame and show
    ShowFrame(imageBuf, index, (*screen));
    
    return 0;
  } 
  int getImages(IplImage **left,IplImage **right)
  {
    // frame request message
    string commandsList = "I";
    int result = SDLNet_TCP_Send(tcpsock1, commandsList.c_str(),(int)commandsList.size());
    if(result < 1) 
    {
      cout<<"SDLNet_TCP_Send: "<<SDLNet_GetError()<<endl;
      return 1;
    }

    SDLNet_SocketSet socSet = SDLNet_AllocSocketSet(1);
    SDLNet_TCP_AddSocket(socSet, tcpsock1);

    // wait until the frame is sent
    int ready = SDLNet_CheckSockets(socSet, TIMEOUT);
    if (ready == 0)
    {
      cout<<"SDLNet_CheckSockets: timeout"<<endl;
      return 2;
    }

    char imageSize = 0;
    char buf[MTU];
    bool waitForImage = true;
    bool archive = false;

    // main loop
    for (int i=0;i<1;i++) 
    {
      if (waitForImage)
      {
        ReceiveImage(tcpsock1, &screen1, socSet, imageSize,archive);
      } 
      else 
      {
        int result = SDLNet_TCP_Recv(tcpsock1, buf, MTU);
        if (result <= 0) 
        {
          cout<<"SDLNet_TCP_Recv: ERROR"<<endl;
          return 3;
        }
      }

      // check keyboard events
      if (HandleEvents(commandsList, waitForImage, archive))
        break;

      int sendResult = SDLNet_TCP_Send(tcpsock1,commandsList.c_str(),(int)commandsList.size());
      if(sendResult < 1)
        printf("SDLNet_TCP_Send: %s\n", SDLNet_GetError());

      // wait until the frame is sent
      int ready = SDLNet_CheckSockets(socSet, TIMEOUT);
      if (ready == 0)
      {
        cout<<"SDLNet_CheckSockets: timeout"<<endl;
        return 4;
      }
    }
    return 0;
  }
  int checkObstacle(int row, int col, int width, float *arr_x, float *arr_y, float *arr_z, IplImage* disp_img)
  {
    //return 1;
    int i,j;
    float angle;
    float x,y,z;
    int count = 0;
    for(i=-4;i<=4;i++)
    {
      for(j=-4; j<=4; j++)
      {
        if(arr_x[((row+i)*width)+col+j]!=-100&&arr_z[((row+i)*width)+col+j]!=-100&&arr_y[((row+i)*width)+col+j]!=-100)
        { y = abs(arr_y[(row*width)+col] - arr_y[((row+i)*width)+(col+j)]); 
          x = abs(arr_x[(row*width)+col] - arr_x[((row+i)*width)+(col+j)]); 
          z = abs(arr_z[(row*width)+col] - arr_z[((row+i)*width)+(col+j)]);
          /*if( y*100 > 10)
          {
            printf("hdiff : %.2f = %.2f - %.2f \n", 100*y, arr_y[(row*width)+col],arr_y[((row+i)*width)+(col+j)]);
            return 1;
          }*/
          angle = atan(y/sqrt(pow(x,2)+pow(z,2)));
          if(angle > A)
            count++;
        }
      }
    }
    if(count>50)
      return 1;
    ((uchar*)(disp_img->imageData + row*disp_img->widthStep))[col] = 0;
  
    return 0;
  }
  
  void loadStereo()
  {
    IplImage *lI,*rI;
    getImages(&lI,&rI);
    //cvShowImage("LeftStereoImage",lI);
    //cvShowImage("RightStereoImage",rI);
    cvWaitKey(100);
  }
  void populateStereoData(char **map)
  {
    for(int i=0; i<R; i++)
    {
      for(int j=0; j<C; j++)
      {
        if(weightofgrids[i][j] >= OBSTACLE_THRESHOLD)
        {
          for(int m=0; m<sq_size*100; m++)
          {
            for(int n=0; n<sq_size*100; n++)
            {
              for(int k =-expansionSize;k<=expansionSize;k++)
                for(int l =-expansionSize;l<=expansionSize;l++)
                  map[(int)(MAP_MAX/2-(C/2-j)*sq_size*100+m)+k][(int)(0.1*MAP_MAX + i*sq_size*100+n)+l] = 255;
              
            }
          }
        }
      }
    }
  }
  void printGrids()
  {
    for(int i=0; i<R; i++)
    {
      for(int j=0; j<C; j++)
      {
        printf("%d ", weightofgrids[i][j]);
      }
      printf("\n");
    }
  }
  //Public Function Definitions.....
  int Stereospace::Stereo::initializeStereo()
  {
    cvNamedWindow("LeftStereoImage", 1);
    cvNamedWindow("RightStereoImage", 1);
    //Creating Images for storing StereoImage
    leftStereoImage = cvCreateImage(cvSize(160,128),8,1);
    if((SDL_Init(SDL_INIT_VIDEO) == -1))
    { 
      cout<<"Could not initialize SDL: "<<SDL_GetError()<<endl;
      return 1;
    }
    if(SDLNet_Init() == -1)
    {
      cout<<"SDLNet_Init: "<<SDLNet_GetError()<<endl;
      return 2;
    }

    IPaddress ipaddress;

    if (SDLNet_ResolveHost(&ipaddress, IPADDR, PORTLEFT) == -1) 
    {
      cout<<"SDLNet_ResolveHost: "<<SDLNet_GetError()<<endl;
      return 2;
    }

    tcpsock1 = SDLNet_TCP_Open(&ipaddress);
    if(!tcpsock1) 
    {
      cout<<"SDLNet_TCP_Open: "<<SDLNet_GetError()<<endl;
      return 2;
    }

    screen1 = SDL_SetVideoMode(640, 480, 16, SDL_DOUBLEBUF|SDL_HWSURFACE);

    if (SDLNet_ResolveHost(&ipaddress, IPADDR, PORTRIGHT) == -1) 
    {
      cout<<"SDLNet_ResolveHost: "<<SDLNet_GetError()<<endl;
      return 2;
    }
    tcpsock2 = SDLNet_TCP_Open(&ipaddress);
    if(!tcpsock2) 
    {
      cout<<"SDLNet_TCP_Open: "<<SDLNet_GetError()<<endl;
      return 2;
    }
    
    screen2 = SDL_SetVideoMode(640, 480, 16, SDL_DOUBLEBUF|SDL_HWSURFACE);
    
    return 0;
  }
  void Stereospace::Stereo::runStereo(char **map)
  {
    loadStereo();
    //printGrids();
    //populateStereoData(map);
    printf("In stereo...\n");
  }
  int Stereospace::Stereo::closeStereo()
  {
    SDLNet_TCP_Close(tcpsock1);
    SDLNet_TCP_Close(tcpsock2);
    SDLNet_Quit();
    SDL_Quit();
    return 0;
  }
}
