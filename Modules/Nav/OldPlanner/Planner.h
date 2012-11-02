#ifndef EKLAVYA_MODULES_PLANNER_H_
#define EKLAVYA_MODULES_PLANNER_H_

#include <vector>
using namespace std;

namespace eklavya_modules_planner {
  enum {
    OPEN,
    CLOSED,
    UNASSIGNED
  };

  const int kMapSizeX = 1000;
  const int kMapSizeY = 1000;
  const int kPoseValues = 360;
  
  typedef struct Pose {
    double x, y, theta;
  } Pose;

  typedef struct VehicleState {
    int seed_index;
    int list_identity, list_index;
    double g_cost, f_cost, h_cost;
    Pose current_pose, previous_pose;
  } VehicleState;

  typedef struct Command {
    int left_velocity, right_velocity;
  } Command;

  typedef struct Seed {
    int index;
    Pose destination;
    vector <Pose> points;
  } Seed;
  
  class Planner {
    public:
      void Load();
      void Navigate();
    private:
      VehicleState state_space[kMapSizeX][kMapSizeY][kPoseValues];
      vector <Seed> seeds;
      vector <VehicleState> open_list, closed_list;
  };
}
#endif
