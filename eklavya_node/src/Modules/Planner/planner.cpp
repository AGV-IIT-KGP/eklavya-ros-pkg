#include <iostream>
#include "cv.h"
#include "highgui.h"
#include "../../eklavya2.h"
#include "planner.h"

/**
 * SEEDS: seeds3.txt is valid but gives suboptimal results
 *        seeds1.txt is for validation purposes ONLY
 */
#define SEEDS_FILE "../src/Modules/Planner/seeds1.txt"
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

  struct PoseCompare : public std::binary_function<Triplet, Triplet, bool> {
    bool operator() (Triplet const& triplet_1, Triplet const& triplet_2) const {
      double k11 = triplet_1.x; double k12 = triplet_1.y; double k13 = triplet_1.z;
      double cantor11 = 0.5 * (k11 + k12) * (k11 + k12 + 1) + k12;
      double cantor12 = 0.5 * (cantor11 + k13) * (cantor11 + k13 + 1) + k13;
      
      double k21 = triplet_2.x; double k22 = triplet_2.y; double k23 = triplet_2.z;
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
  vector<state> open_list;
  vector<seed> seeds;
  map<Triplet, char, PoseCompare> membership;
  map<Triplet, Triplet, PoseCompare> came_from;
  IplImage *map_img;
  
  /// ------------------------------------------------------------- ///
  
  void loadSeeds() {
    int n_seeds;
    double x, y, theta, g, k;
    FILE *fp = fopen(SEEDS_FILE, "r");
    fscanf(fp, "%d\n", &n_seeds);
    
    for (int i = 0; i < n_seeds; i++) {
      seed s;
      fscanf(fp, "%lf %lf %lf %lf %lf %lf\n", &s.vl, &s.vr, &s.dest.x, &s.dest.y, &s.dest.z, &s.cost);
      
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
    double error = sqrt(50^2 + 50^2);
    return sqrt((a.pose.x - b.pose.x) * (a.pose.x - b.pose.x) + 
                (a.pose.y - b.pose.y) * (a.pose.y - b.pose.y)) < error;
  }
  
  vector<Triplet> reconstructPath(state current) {
    Triplet current_pose = current.pose;
    vector<Triplet> path;
    
    while (came_from.find(current_pose) != came_from.end()) {
      path.insert(path.begin(), current_pose);
      current_pose = came_from[current_pose];
    }
    
    return path;
  }
  
  vector<state> neighborNodes(state current) {
    vector<state> neighbours;
    for(int i = 0; i < seeds.size(); i++) {
      state neighbour;
      double sx = seeds[i].dest.x;
      double sy = seeds[i].dest.y;
      double sz = seeds[i].dest.z;
      
      neighbour.pose.x = current.pose.x + 
                          sx * sin(current.pose.z * (CV_PI / 180)) + 
                          sy * cos(current.pose.z * (CV_PI / 180));
      neighbour.pose.y = current.pose.y + 
                          -sx * cos(current.pose.z * (CV_PI / 180)) + 
                          sy * sin(current.pose.z * (CV_PI / 180));
      neighbour.pose.z = sz - (90 - current.pose.z);
      neighbour.g = seeds[i].cost;
      neighbour.h = 0;
      neighbour.seed_id = i;
      
      neighbours.push_back(neighbour);                
    }
    
    return neighbours;
  }
  
  void print(state s) {
    double f = s.g + s.h;
    cout << "{ " << s.pose.x << " , " << s.pose.y << " , " << s.pose.z << " , " << f << " }" << endl;
  }
  
  bool isWalkable(Triplet pose) {
    return local_map[(int) pose.x][(int) pose.y] == 0;
  }
  
  /// ------------------------------------------------------------- ///
  
  void Planner::loadPlanner() {
    loadSeeds();
    for(int i = 0; i < seeds.size(); i++) {
      cout << "SEEDS: {" << seeds[i].dest.x << " , " << seeds[i].dest.y << " , " << seeds[i].dest.z << " , " << 
      seeds[i].cost << " }" << endl;
    }
    cvNamedWindow("Map", CV_WINDOW_NORMAL);
  }

  vector<Triplet> Planner::findPath(Triplet bot, Triplet target) {
    state start, goal;
    start.pose = bot; start.g = 0; start.h = distance(bot, target); start.seed_id = 0;
    goal.pose = target; goal.g = 0; goal.h = 0; goal.seed_id = 0;
    
    cout << "START: ";  print(start);
    
    open_list.insert(open_list.begin(), start);
    membership[start.pose] = OPEN;
    if (membership.find(start.pose) == membership.end()) {
      cout << "NOO!" << endl;
    }
    
    while(!open_list.empty()) {
      state current = open_list.front();
      if((membership.find(current.pose) != membership.end()) && (membership[current.pose] == CLOSED)) {
        cout << ">DUPLICATES FOUND. PEACE MAARO" << endl;
        continue;
      }
      
      cout << ">CURRENT: ";  print(current);
      cout << ">GOAL: ";  print(goal);
      
      //getchar();
      
      if (isEqual(current, goal)) {
        return reconstructPath(goal);
      }
      
      pop_heap(open_list.begin(), open_list.end(), StateCompare()); 
      open_list.pop_back();
      membership[current.pose] = UNASSIGNED;
      
      membership[current.pose] = CLOSED;
      
      vector<state> neighbors = neighborNodes(current);
      
      for(int i = 0; i < neighbors.size(); i++) {
        state neighbor = neighbors[i];
        
        if(!isWalkable(neighbor.pose)) {
          continue;
        }
        
        cout << ">>NEIGHBOR: "; print(neighbor);
        
        double tentative_g_score = neighbor.g + current.g;
        cout << ">>TGS: " << tentative_g_score << endl;
        
        if(!((membership.find(neighbor.pose) != membership.end()) && (membership[neighbor.pose] == OPEN)) || 
        tentative_g_score <= neighbor.g) {
          came_from[neighbor.pose] = current.pose;
          neighbor.g = tentative_g_score;
          neighbor.h = distance(neighbor.pose, goal.pose);
          if(!((membership.find(neighbor.pose) != membership.end()) && (membership[neighbor.pose] == OPEN))) {
            open_list.push_back(neighbor);
            push_heap(open_list.begin(), open_list.end(), StateCompare());
            membership[neighbor.pose] = OPEN;
          }
        }
      }
    }

    // FAILURE
    vector<Triplet> path;
    return path;
  }

  void Planner::closePlanner() {
    
  }
}
