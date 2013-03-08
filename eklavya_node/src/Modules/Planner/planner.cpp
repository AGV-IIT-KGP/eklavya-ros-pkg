#include <iostream>
#include "cv.h"
#include "highgui.h"
#include "../../eklavya2.h"
#include "planner.h"

#define SIM_SEEDS
//#define DEBUG
#define SHOW_PATH

/**
 * SEEDS: seeds3.txt is valid but gives suboptimal results
 *        seeds1.txt is for validation purposes ONLY
 *        seeds.txt contains the full set of paths
 */

#ifdef SIM_SEEDS
  #define SEEDS_FILE "../src/Modules/Planner/seeds4.txt"
#else
  #define SEEDS_FILE "../src/Modules/Planner/seeds1.txt"
#endif
#define OPEN 1
#define CLOSED 2
#define UNASSIGNED 3

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
      return state_1.g + state_1.h > state_2.g + state_2.h;
    }
  };
  
  Triplet bot, target;
  vector<seed> seeds;
  
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
      #else
        fscanf(fp, "%lf %lf %lf %lf %lf %lf\n", &s.vl, &s.vr, &x, &y, &z, &s.cost);
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
                (a.pose.y - b.pose.y) * (a.pose.y - b.pose.y)) < error) && 
           ((a.pose.z - b.pose.z) * (a.pose.z - b.pose.z) < 25);
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
    
    cvShowImage("Map", map_img);
    cvWaitKey(1);
  }
  
  void reconstructPath(map<Triplet, Triplet, PoseCompare> came_from, IplImage *map_img, state current) {
    pthread_mutex_lock(&path_mutex);
    
    path.clear();
    
    Triplet current_pose = current.pose;
    while (came_from.find(current_pose) != came_from.end()) {
      #ifdef SHOW_PATH
        plotPoint(map_img, current_pose);
      #endif
      
      path.insert(path.begin(), current_pose);
      current_pose = came_from[current_pose];
    }
    
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
  
  bool isWalkable(Triplet pose) {
    return local_map[pose.x][pose.y] == 0;
  }

  void closePlanner(IplImage *map_img) {
    cvReleaseImage(&map_img);
  }
  
  /// ------------------------------------------------------------- ///
  
  void Planner::loadPlanner() {
    loadSeeds();
    
    #ifdef SHOW_PATH
      cvNamedWindow("Map", 0);
    #endif
  }

  void Planner::findPath(Triplet bot, Triplet target) {
    state start, goal;
    start.pose = bot; start.seed_id = 0;
    start.g = 0; start.h = distance(bot, target); 
    goal.pose = target; goal.g = 0; goal.h = 0; goal.seed_id = 0;
    
    //cout << "TARGET: " << print(goal);
    
    IplImage *map_img;
    
    #if defined(DEBUG) || defined(SHOW_PATH)
      map_img = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), IPL_DEPTH_8U, 3);
    #endif
    
    vector<state> open_list;
    open_list.insert(open_list.begin(), start);
    
    map<Triplet, open_map_element, PoseCompare> open_map;
    open_map[start.pose].membership = OPEN;
    open_map[start.pose].cost = start.g;
    
    map<Triplet, Triplet, PoseCompare> came_from;
    
    while(!open_list.empty()) {
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
        
        if(!isWalkable(neighbor.pose)) {
          continue;
        }
        
        double tentative_g_score = neighbor.g + current.g;
        double admissible = distance(neighbor.pose, goal.pose);
        double consistent = max(admissible, current.h - neighbor.g);
        
        //cout << "[INFO] TENTATIVE: " << tentative_g_score << endl;
        //if((open_map.find(neighbor.pose) != open_map.end()) && 
               //(open_map[neighbor.pose].membership == OPEN)) {
          //cout << "[INFO] NEIGHBORG: " << open_map[neighbor.pose].cost << endl;
        //}
        
        if(!((open_map.find(neighbor.pose) != open_map.end()) && 
             (open_map[neighbor.pose].membership == OPEN)) || 
             (tentative_g_score - open_map[neighbor.pose].cost < 100)) {
          came_from[neighbor.pose] = current.pose;
          neighbor.g = tentative_g_score;
          neighbor.h = consistent;
          
          //cout << "====> NEIGHBOR: ";  print(neighbor); getchar();
      
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
}
