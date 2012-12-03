#include <stdio.h>
#include <math.h>
#include <limits>
#include <vector>
#include <queue>
#include <map>

#define MAP_SIZE_X 1000
#define MAP_SIZE_Y 1000
#define STATE_X MAP_SIZE_X
#define STATE_Y MAP_SIZE_Y
#define STATE_T 360
#define BOT_X 500
#define BOT_Y 100

#define TARGET_RADIUS 30
#define CV_PI 3.14

#define OPEN_LIST 1
#define CLOSED_LIST 2
#define UNASSIGNED 3

#define SEEDS_PATH "seeds.txt"

typedef struct Pose {
  double x, y, theta;
} Pose;

typedef struct State {
  int seed_index;
  double f_cost, g_cost, h_cost;
  Pose curr_pose, prev_pose;
} State;

typedef struct Seed {
  int index;
  int left_velocity, right_velocity;
  double g_cost;
  double velocity_ratio;
  Pose dest_pose;
  std::vector<Pose> seed_poses;
} Seed;

class ComparePoses {
  public:
    bool operator()(Pose pose_1, Pose pose_2) const {
      if ((pose_1.x < pose_2.x) && (pose_1.y < pose_2.y) && (pose_1.theta < pose_2.theta)) {
        return true;
      } else {
        return false;
      }
    }
};

class CompareStates {
  public:
    bool operator()(State state_1, State state_2) const {
      if ((state_1.curr_pose.x < state_2.curr_pose.x) && 
              (state_1.curr_pose.y < state_2.curr_pose.y) && 
              (state_1.curr_pose.theta < state_2.curr_pose.theta)) {
        return true;
      } else {
        return false;
      }
    }
};

char map[MAP_SIZE_X][MAP_SIZE_Y];
State target, start;
std::vector<State> path;
std::vector<Seed> seeds;
std::map<Pose, int, ComparePoses> membership;
std::map<Pose, State, ComparePoses> parentship;

void loadSeeds() {
  int number_of_seeds;
  FILE *fp = fopen(SEEDS_PATH, "r");
  fscanf(fp, "%d\n", &number_of_seeds);

  for (int i = 0; i < number_of_seeds; i++) {
    double temp_x, temp_y;
    double offset;
    Seed temp_seed;

    temp_seed.index = i;
    fscanf(fp, "%d %d %lf %lf %lf %lf\n", &temp_seed.left_velocity, &temp_seed.right_velocity, 
                                          &temp_x, &temp_y, &temp_seed.dest_pose.theta, &temp_seed.g_cost);

    temp_x < 0 ? offset = -1.5 : offset = 0.5;
    temp_seed.dest_pose.x = (int) (temp_x + offset);
    temp_seed.dest_pose.y = (int) (temp_y + offset);

    int number_of_seed_points;
    fscanf(fp, "%d\n", &number_of_seed_points);

    for (int j = 0; j < number_of_seed_points; j++) {
      double temp_x, temp_y;
      Pose temp_pose;

      fscanf(fp, "%lf %lf\n", &temp_x, &temp_y);
      temp_pose.x = (int) (temp_x + offset);
      temp_pose.y = (int) (temp_y + offset);
      temp_pose.theta = 0;
      
      temp_seed.seed_poses.insert(temp_seed.seed_poses.begin(), temp_pose);
    }

    seeds.insert(seeds.begin(), temp_seed);
  }
  fclose(fp);
}

void initPlanner(int argc, char **argv) {
  for (int i = 0; i < STATE_X; i++) {
    for (int j = 0; j < STATE_Y; j++) {
      map[i][j] = 0;
    }
  }

  target.curr_pose.x = BOT_X;
  target.curr_pose.y = BOT_Y + 0.5 * MAP_SIZE_Y;
  target.curr_pose.theta = 90;
  target.g_cost = 0;
  target.h_cost = 0; 
  target.f_cost = target.g_cost + target.h_cost;

  start.curr_pose.x = BOT_X;
  start.curr_pose.y = BOT_Y;
  start.curr_pose.theta = 90;
  start.g_cost = 0;
  start.h_cost = 0; 
  start.f_cost = start.g_cost + start.h_cost;

  
  
  loadSeeds();
}

bool isTargetReached(State current) {
  if ((current.curr_pose.x - target.curr_pose.x) * (current.curr_pose.x - target.curr_pose.x) +
      (current.curr_pose.y - target.curr_pose.y) * (current.curr_pose.y - target.curr_pose.y) < 
          TARGET_RADIUS * TARGET_RADIUS) {
    return true;
  } else {
    return false;
  }
}

double heuristicCostEstimate(State state_1, State state_2) {
  return sqrt((state_1.curr_pose.x - state_2.curr_pose.x) * (state_1.curr_pose.x - state_2.curr_pose.x) +
              (state_1.curr_pose.y - state_2.curr_pose.y) * (state_1.curr_pose.y - state_2.curr_pose.y));
}

bool hasValidParent(State current) {
  return parentship.count(current.curr_pose) == 1;
}

State getParent(State current) {
  return parentship[current.curr_pose];
}

void reconstructPath(State current) {
  while (hasValidParent(current)) {
    path.insert(path.begin(), current);
    current = getParent(current);
  }
}

void getCommand(int *left_velocity, int *right_velocity) {
  Seed command = seeds[path[1].seed_index];
  
  if (command.left_velocity > command.right_velocity) {
    *left_velocity = 30;
    *right_velocity = 22;
  } else if (command.left_velocity < command.right_velocity) {
    *left_velocity = 18;
    *right_velocity = 30;
  } else if ((command.left_velocity != 0) || (command.right_velocity != 0)) {
    *left_velocity = 22;
    *right_velocity = 25;
  }
}

bool isWalkable(State current) {
  double alpha = getParent(current).curr_pose.theta;
  std::vector<Pose> seed_poses = seeds[current.seed_index].seed_poses;

  for (int i = 0; i < seed_poses.size(); i++) {
    int original_x, original_y;
    int transformed_x, transformed_y;

    original_x = seed_poses[i].x;
    original_y = seed_poses[i].y;

    transformed_x = (int) (original_x * sin(alpha * (CV_PI / 180)) +
                           original_y * cos(alpha * (CV_PI / 180)) + getParent(current).curr_pose.x);
    transformed_y = (int) (-original_x * cos(alpha * (CV_PI / 180)) +
                           original_y * sin(alpha * (CV_PI / 180)) + getParent(current).curr_pose.y);

    if (((0 <= transformed_x) && (transformed_x < MAP_SIZE_X)) && ((0 <= transformed_y) && (transformed_y < MAP_SIZE_Y))) {
      if (map[transformed_x][transformed_y] != 0) {
        return false;
      }
    } else {
      return false;
    }
  }

  return true;
}

std::vector<State> getNeighbours(State current) {
  std::vector<State> neighbours;

  for (int i = 0; i < seeds.size(); i++) {
    State temp;

    temp.seed_index = seeds[i].index;
    temp.curr_pose.x = (int)(seeds[i].dest_pose.x * sin(current.curr_pose.theta * (CV_PI / 180)) +
                    seeds[i].dest_pose.y * cos(current.curr_pose.theta * (CV_PI / 180)) + current.curr_pose.x);
    temp.curr_pose.y = (int)(-seeds[i].dest_pose.x * cos(current.curr_pose.theta * (CV_PI / 180)) +
                    seeds[i].dest_pose.y * sin(current.curr_pose.theta * (CV_PI / 180)) + current.curr_pose.y);
    temp.curr_pose.theta = seeds[i].dest_pose.theta - (90 - current.curr_pose.theta);

    if ((0 <= temp.curr_pose.x) && (temp.curr_pose.x <= MAP_SIZE_X - 1) && (0 <= temp.curr_pose.y) && 
            (temp.curr_pose.y <= MAP_SIZE_Y - 1)) {
      if (isWalkable(temp)) {
        neighbours.insert(neighbours.begin(), temp);
      }
    }
  }

  return neighbours;
}

int getMembership(State current) {
  return membership.count(current.curr_pose) == 1 ? membership[current.curr_pose] : UNASSIGNED;
}

void setMembership(State current, int membership_status) {
  membership[current.curr_pose] = membership_status;
}

double distanceBetween(State state_1, State state_2) {
  return sqrt((state_1.g_cost - state_2.g_cost) * (state_1.g_cost - state_2.g_cost));
}

void setParent(State current, State parent) {
  parentship[current.curr_pose] = parent;
}

int main(int argc, char **argv) {
  int iteration_count;

  initPlanner(argc, argv);

  iteration_count = 0;

  while (1) {
    if (isTargetReached(start) == true) {
      printf("Target Reached\n");
      return 0;
    }

    std::priority_queue<State, std::vector<State>, CompareStates> open_list;

    start.g_cost = 0;
    start.f_cost = start.g_cost + heuristicCostEstimate(start, target);

    open_list.push(start);
    setMembership(start, OPEN_LIST);
    printf("> Pushing (%lf, %lf, %lf)\n", start.curr_pose.x, start.curr_pose.y, start.curr_pose.theta);

    while (open_list.size() != 0) {
      State current = open_list.top();
      printf(">> Current (%lf, %lf, %lf)\n", current.curr_pose.x, current.curr_pose.y, current.curr_pose.theta);

      if (isTargetReached(current)) {
        reconstructPath(current);

        int left_velocity, right_velocity;
        getCommand(&left_velocity, &right_velocity);
      }

      open_list.pop();
      setMembership(current, UNASSIGNED);
      
      if (getMembership(current) != CLOSED_LIST) {
        setMembership(current, CLOSED_LIST);
      } else {
        continue;
      }
      
      std::vector<State> neighbours = getNeighbours(current);
      for (int i = 0; i < neighbours.size(); i++) {
        if (getMembership(neighbours[i]) == CLOSED_LIST) {
          continue;
        }

        double tentative_g_cost = current.g_cost + distanceBetween(current, neighbours[i]);
        if ((getMembership(neighbours[i]) != OPEN_LIST) || (tentative_g_cost < neighbours[i].g_cost)) {
          setParent(neighbours[i], current);
          neighbours[i].g_cost = tentative_g_cost;
          neighbours[i].f_cost = neighbours[i].g_cost + heuristicCostEstimate(neighbours[i], target);

          open_list.push(neighbours[i]);
          setMembership(neighbours[i], OPEN_LIST);
        }
      }
    }

    iteration_count++;
  }
}
