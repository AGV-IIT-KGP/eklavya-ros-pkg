#include <iostream>
#include "cv.h"
#include "highgui.h"
#include "../../eklavya2.h"
#include "../../Utils/SerialPortLinux/serial_lnx.h"
#include "../devices.h"
#include "planner.h"

#define SIM_SEEDS
//#define DEBUG
#define SHOW_PATH

/**
 * SEEDS: seeds3.txt is valid but gives suboptimal results. Good Path. (6 - 8)
 *        seeds1.txt is for validation purposes ONLY. Grid A* - like path. Might be useful with DT. (101)
 *        seeds.txt contains the full set of paths. Almost always 

 stuck in inf loop.
 *        seeds4.txt contains 5 seeds with arc-length ~75. Works fine. (4)
 *        seeds5.txt contains 5 seeds with arc-lengths varying from 100 to 50. (2)
 *        seeds2.txt contains 5 seeds 75 - 100 - 50 (16-20)
 */

#ifdef SIM_SEEDS
  #define SEEDS_FILE "../src/Modules/Planner/seeds2.txt"
#else
  #define SEEDS_FILE "../src/Modules/Planner/seeds1.txt"
#endif
#define OPEN 1
#define CLOSED 2
#define UNASSIGNED 3
#define VMAX 100

using namespace std;

namespace planner_space {
  typedef struct state { // elemental data structure of openset
    Triplet pose;
    double g, h;  // costs
    int seed_id;
  } state;

  typedef struct seed_point {
    double x, y;
  } seed_point;

  typedef struct seed {
    Triplet dest;
    double cost;
    double k;             // velocity ratio
    double vl, vr;        // individual velocities
    vector<seed_point> seed_points;
  } seed;
  
  typedef struct open_map_element {
    char membership;
    double cost;
  } open_map_element;

  struct PoseCompare : public std::binary_function<Triplet, Triplet, bool> {
    bool operator() (Triplet const& triplet_1, Triplet const& triplet_2) const {
      double k11 = triplet_1.x; 
      double k12 = triplet_1.y; 
      double k13 = triplet_1.z;
      
      double cantor11 = 0.5 * (k11 + k12) * (k11 + k12 + 1) + k12;
      double cantor12 = 0.5 * (cantor11 + k13) * (cantor11 + k13 + 1) + k13;
      
      double k21 = triplet_2.x; 
      double k22 = triplet_2.y; 
      double k23 = triplet_2.z;
      
      double cantor21 = 0.5 * (k21 + k22) * (k21 + k22 + 1) + k22;
      double cantor22 = 0.5 * (cantor21 + k23) * (cantor21 + k23 + 1) + k23;
      
      return cantor12 < cantor22;
      //return true;
    }
  };
  
  struct StateCompare : public std::binary_function<state, state, bool> {
    bool operator() (state const& state_1, state const& state_2) const {
      double f1 = state_1.g + state_1.h;
      double f2 = state_2.g + state_2.h;
      
      double diff = sqrt((f1 - f2) * (f1 - f2));
      
      /*if(diff < 1) {
        return state_1.g < state_2.g;
      } else {
        return f1 > f2;
      }
      
      if(f1 > f2) {
        return true;
      } else if(f1 < f2) {
        return false;
      } else {
        return state_1.g > state_2.g;
      }*/
      
      return f1 > f2;
    }
  };
  
  Triplet bot, target;
  vector<seed> seeds;
  Tserial *p;
  
  /// ------------------------------------------------------------- ///
  
  void loadSeeds() {
    int n_seeds;
    double x, y, z, theta, g, k;
    FILE *fp = fopen(SEEDS_FILE, "r");
    fscanf(fp, "%d\n", &n_seeds);
    
    for (int i = 0; i < n_seeds; i++) {
      seed s;
      #ifdef SIM_SEEDS
        fscanf(fp, "%lf %lf %lf %lf %lf\n", &s.k, &x, &y, &z, &s.cost);
        s.vl = VMAX * s.k / (1 + s.k);
        s.vr = VMAX / (1 + s.k);
      #else
        fscanf(fp, "%lf %lf %lf %lf %lf %lf\n", &s.vl, &s.vr, &x, &y, &z, &s.cost);
        s.k = s.vl / s.vr;
      #endif
      
      s.dest.x = (int) x;
      s.dest.y = (int) y;
      s.dest.z = (int) z;
      
      int n_seed_points;
      fscanf(fp, "%d\n", &n_seed_points);
      
      for (int j = 0; j < n_seed_points; j++) {
        seed_point point;
        fscanf(fp, "%lf %lf\n", &point.x, &point.y);
        s.seed_points.insert(s.seed_points.begin(), point);
      }
      seeds.insert(seeds.begin(), s);
    }
  }
  
  double distance(Triplet a, Triplet b) {
    return sqrt ((a.x - b.x) * (a.x - b.x) + 
                 (a.y - b.y) * (a.y - b.y));
  }
  
  bool isEqual(state a, state b) {
    double error = sqrt((50 ^ 2) + (50 ^ 2));
    return (sqrt((a.pose.x - b.pose.x) * (a.pose.x - b.pose.x) + 
                 (a.pose.y - b.pose.y) * (a.pose.y - b.pose.y)) < error);
                // && ((a.pose.z - b.pose.z) * (a.pose.z - b.pose.z) < 25);
  }
  
  void plotPoint(IplImage *map_img, Triplet pose) {
    int x = pose.x;
    int y = MAP_MAX - pose.y - 1;
    int ax = x > MAP_MAX ? MAP_MAX - 1 : x;  
    ax = x < 0 ? 0 : x;
    int ay = y > MAP_MAX ? MAP_MAX - 1 : y;  
    ay = y < 0 ? 0 : y;
    
    int bx = ax;
    int by = y + 5 > MAP_MAX ? MAP_MAX - 1 : y + 5;  
    by = y + 5 < 0 ? 0 : y + 5;
    
    srand(time(0));
    cvLine(map_img, cvPoint(ax, ay), cvPoint(bx, by), CV_RGB(rand() % 255, rand() % 255, rand() % 255), 2, CV_AA, 0);
  }
  
  void initBot() {
    #ifndef SIMCTL
      p = new Tserial();
      p->connect(BOT_COM_PORT, BOT_BAUD_RATE, spNONE);
      usleep(100);
      
      p->sendChar('w');
      usleep(100);
    #endif
  }
  
  void sendCommand(seed s) {
    #ifndef SIMCTL
      int left_vel = 0;
      int right_vel = 0;
      int left_velocity = s.vl;
      int right_velocity = s.vr;
      
      if((left_velocity == 0) && (right_velocity == 0)) {
        p->sendChar(' ');
        usleep(100);
        return;
      }
      
      if (left_velocity > right_velocity) {
        left_vel = 28;
        right_vel = 20;
      } else if (left_velocity < right_velocity) {
        left_vel = 20;
        right_vel = 28;
      } else if ((left_velocity != 0) || (right_velocity != 0)) {
        left_vel = 20;
        right_vel = 20;
      }
      
      right_vel += 4;
      
      printf("Velocity: (%d, %d)\n", left_vel, right_vel);
    
      p->sendChar('w');
      usleep(100);

      p->sendChar('0' + left_vel / 10);
      usleep(100);
      p->sendChar('0' + left_vel % 10);
      usleep(100);
      p->sendChar('0' + right_vel / 10);
      usleep(100);
      p->sendChar('0' + right_vel % 10);
      usleep(100);
    #endif
  }
  
  void reconstructPath(map<Triplet, state, PoseCompare> came_from, IplImage *map_img, state current) {
    pthread_mutex_lock(&path_mutex);
    
    path.clear();
    
    state s = current;
    while (came_from.find(s.pose) != came_from.end()) {
      #ifdef SHOW_PATH
        plotPoint(map_img, s.pose);
      #endif
      
      path.insert(path.begin(), s.pose);
      s = came_from[s.pose];
    }
    
    sendCommand(seeds[s.seed_id]);
    
    #ifdef SHOW_PATH
      cvShowImage("Map", map_img);
      cvWaitKey(1);
    #endif
    
    pthread_mutex_unlock(&path_mutex);
  }
  
  vector<state> neighborNodes(state current) {
    vector<state> neighbours;
    for(int i = 0; i < seeds.size(); i++) {
      state neighbour;
      double sx = seeds[i].dest.x;
      double sy = seeds[i].dest.y;
      double sz = seeds[i].dest.z;
      
      neighbour.pose.x = (int) (current.pose.x + 
                                sx * sin(current.pose.z * (CV_PI / 180)) + 
                                sy * cos(current.pose.z * (CV_PI / 180)));
      neighbour.pose.y = (int) (current.pose.y + 
                                -sx * cos(current.pose.z * (CV_PI / 180)) + 
                                sy * sin(current.pose.z * (CV_PI / 180)));


      neighbour.pose.z = (int) (sz - (90 - current.pose.z));
      neighbour.g = seeds[i].cost;
      neighbour.h = 0;
      neighbour.seed_id = i;
      
      neighbours.push_back(neighbour);                
    }
    
    return neighbours;
  }
  
  void print(state s) {
    double f = s.g + s.h;
    cout << "{ " << s.pose.x << " , " 
                 << s.pose.y << " , " 
                 << s.pose.z << " , " 
                 << f 
         << " }" << endl;
  }
  
  bool isWalkable(state parent, state s) {
    for (int i = 0; i < seeds[s.seed_id].seed_points.size(); i++) {
      int x, y;
      double alpha = parent.pose.z;

      int tx, ty;
      tx = seeds[s.seed_id].seed_points[i].x;
      ty = seeds[s.seed_id].seed_points[i].y;

      x = (int)(tx * sin(alpha * (CV_PI / 180)) + ty * cos(alpha * (CV_PI / 180)) + parent.pose.x);
      y = (int)(-tx * cos(alpha * (CV_PI / 180)) + ty * sin(alpha * (CV_PI / 180)) + parent.pose.y);

      if (((0 <= x) && (x < MAP_MAX)) && ((0 <= y) && (y < MAP_MAX))) {
        return local_map[x][y] == 0;
      } else {
        return false;
      }
    }

    return true;
  }

  void closePlanner(IplImage *map_img) {
    #if defined(DEBUG) || defined(SHOW_PATH)
      cvReleaseImage(&map_img);
    #endif
  }
  
  void addObstacle(IplImage *map_img, int x, int y, int r) {
    for(int i = -r; i < r; i++) {
      for(int j = -r; j < r; j++) {
        local_map[x + i][y + j] = 255;  
      }
    }
    
    #if defined(DEBUG) || defined(SHOW_PATH)
      cvCircle(map_img, cvPoint(x, MAP_MAX - y - 1), r, CV_RGB(255, 255, 0), -1, CV_AA, 0);
    #endif
  }
  
  void loadMap() {
    
  }
  
  /// ------------------------------------------------------------- ///
  
  void Planner::loadPlanner() {
    loadSeeds();
    
    #if defined(DEBUG) || defined(SHOW_PATH)
      cvNamedWindow("Map", 0);
    #endif
	
    loadMap();
    
    initBot();
  }

  void Planner::findPath(Triplet bot, Triplet target) {
    state start, goal;
    start.pose = bot; start.seed_id = 0;
    start.g = 0; start.h = distance(bot, target); 
    goal.pose = target; goal.g = 0; goal.h = 0; goal.seed_id = 0;
    
    //cout << "TARGET: "; print(goal);
    
    IplImage *map_img;
    
    #if defined(DEBUG) || defined(SHOW_PATH)
      map_img = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), IPL_DEPTH_8U, 3);
    #endif
    
    //addObstacle(map_img, 500, 500, 100);  
    
    vector<state> open_list;
    open_list.insert(open_list.begin(), start);
    
    map<Triplet, open_map_element, PoseCompare> open_map;
    open_map[start.pose].membership = OPEN;
    open_map[start.pose].cost = start.g;
    
    map<Triplet, state, PoseCompare> came_from;
    
    while(!open_list.empty()) {
      
      #if defined(DEBUG) || defined(SHOW_PATH)
        cvShowImage("Map", map_img);
      
        #ifdef DEBUG
          cvWaitKey(0);
        #else
          cvWaitKey(1);
        #endif
        
      #endif
      
      state current = open_list.front();
      
      #ifdef DEBUG
        //cout << "==> CURRENT: ";  print(current);
        plotPoint(map_img, current.pose);
      #endif
      
      if((open_map.find(current.pose) != open_map.end()) && 
         (open_map[current.pose].membership == CLOSED)) {
        continue;
      }
      
      if (isEqual(current, goal)) {
        reconstructPath(came_from, map_img, current);
        
        #ifdef SHOW_PATH
          cout << "[SUCCESS] PATH FOUND" << endl;
          cvWaitKey(0);
        #endif
        
        closePlanner(map_img);
        return;
      }
      
      pop_heap(open_list.begin(), open_list.end(), StateCompare()); 
      open_list.pop_back();
      open_map[current.pose].membership = UNASSIGNED;
      open_map[current.pose].cost = -1;
      
      open_map[current.pose].membership = CLOSED;
      
      vector<state> neighbors = neighborNodes(current);
      
      //cout << "N OF NEIGHBORS: " << neighbors.size() << endl;
      for(int i = 0; i < neighbors.size(); i++) {
        state neighbor = neighbors[i];
        
        #ifdef DEBUG
          plotPoint(map_img, neighbor.pose);
        #endif

        if(!(((neighbor.pose.x >= 0) && (neighbor.pose.x < MAP_MAX)) &&
             ((neighbor.pose.y >= 0) && (neighbor.pose.y < MAP_MAX)))) {
          continue;
        }
        
        if(!isWalkable(current, neighbor)) {
          continue;
        }
        
        double tentative_g_score = neighbor.g + current.g;
        double admissible = distance(neighbor.pose, goal.pose);
        //double consistent = max(admissible, current.h - neighbor.g);
        double consistent = admissible;
        
        //cout << "[INFO] TENTATIVE: " << tentative_g_score << endl;
        //if((open_map.find(neighbor.pose) != open_map.end()) && 
               //(open_map[neighbor.pose].membership == OPEN)) {
          //cout << "[INFO] NEIGHBORG: " << open_map[neighbor.pose].cost << endl;
        //}
        
        if(!((open_map.find(neighbor.pose) != open_map.end()) && 
             (open_map[neighbor.pose].membership == OPEN))/* || (
             tentative_g_score < open_map[neighbor.pose].cost)*/) {
          came_from[neighbor.pose] = current;
          neighbor.g = tentative_g_score;
          neighbor.h = consistent;
          
          //cout << "====> NEIGHBOR: ";  print(neighbor);
      
          if(!((open_map.find(neighbor.pose) != open_map.end()) && 
               (open_map[neighbor.pose].membership == OPEN))) {
            open_list.push_back(neighbor);
            push_heap(open_list.begin(), open_list.end(), StateCompare());
            open_map[neighbor.pose].membership = OPEN;
            open_map[neighbor.pose].cost = neighbor.g;
          }
        }
      }
    }
    
    closePlanner(map_img);
    cout << "[ERROR] NO PATH FOUND" << endl;
  }

  void Planner::finBot() {
    #ifndef SIMCTL
      p = new Tserial();
      p->connect(BOT_COM_PORT, BOT_BAUD_RATE, spNONE);
      usleep(100);
      
      p->sendChar(' ');
      usleep(100);

      p->disconnect();
      usleep(100);
    #endif
  }
  
  
}
