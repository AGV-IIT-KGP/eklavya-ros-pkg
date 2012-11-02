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

typedef struct pose {
  double x, y, theta;
}

typedef struct state {
  int seed_index;
  double f_cost, g_cost, h_cost;
  pose current, parent;
} state;

typedef struct seed {
  int left_velocity, right_velocity;
  double g_cost;
  double velocity_ratio;
  pose current;
  std::vector<pose> seed_points;
} seed;

char map[MAP_SIZE_X][MAP_SIZE_Y];
state target, start;
std::vector<state> path;
std::vector<seed> seeds;
std::map<pose, int> membership;
std::map<pose, state> parentship;

void loadSeeds() {
  int number_of_seeds;
  FILE *fp = fopen(SEEDS_PATH, "r");
  fscanf(fp, "%d\n", &number_of_seeds);

  for (int i = 0; i < number_of_seeds; i++) {
    double temp_x, temp_y;
    double offset;
    seed temp_seed;

    temp_seed.index = i;
    fscanf(fp, "%d %d %lf %lf %lf %lf\n", &temp_seed.left_velocity, &temp_seed.right_velocity, 
                                          &temp_x, &temp_y, &temp_seed.theta, &temp_seed.g_cost);

    temp_seed.x < 0 ? offset = -1.5 : offset = 0.5;
    temp_seed.x = (int) (temp_x + offset);
    temp_seed.y = (int) (temp_y + offset);

    int number_of_seed_points;
    fscanf(fp, "%d\n", &number_of_seed_points);

    for (int j = 0; j < number_of_seed_points; j++) {
      double temp_x, temp_y;
      state temp_state;

      fscanf(fp, "%lf %lf\n", &temp_x, &temp_y);
      temp_state.x = (int) (temp_x + offset);
      temp_state.y = (int) (temp_y + offset);

      temp_seed.seed_points.insert(temp_seed.seed_points.begin(), temp_state);
    }

    seeds.insert(seeds.begin(), temp_seed);
  }
  fclose(fp);
}

void initPlanner(int argc, char **argv) {
  for (int i = 0; i < STATE_X; i++) {
    for (int j = 0; j < STATE_Y; j++) {
      map[i][j] = 0;
      for (int k = 0; k < STATE_T; k++) {
        membership[i][j][k] = UNASSIGNED;
        parents[i][j][k] = std::numeric_limits<int>::max();
      }
    }
  }

  target.x = BOT_X;
  target.y = BOT_Y + 0.5 * MAP_SIZE;
  target.theta = 90;
  target.g_cost = 0;
  target.h_cost = 0; 
  target.f_cost = target.g_cost + target.h_cost;

  start.x = BOT_X;
  start.y = BOT_Y;
  start.theta = 90;
  start.g_cost = 0;
  start.h_cost = 0; 
  start.f_cost = start.g_cost + start.h_cost;

  loadSeeds();
}

bool isTargetReached(state current) {
  if (((current.x - target.x) * (current.x - target.x) +
      (current.y - target.y) * (current.y - target.y) < TARGET_RADIUS * TARGET_RADIUS) && ()) {
    return true;
  } else {
    return false;
  }
}

class CompareStates {
  public:
    bool operator()(state& state_1, state& state_2) {
      if (state_1.f_cost < state_2.f_cost) {
        return true;
      } else if (state_1.f_cost == state_2.f_cost) {
        return state_1.g_cost < state_2.g_cost;
      } else {
        return false;
      }
    }
};

double heuristicCostEstimate(state state_1, state state_2) {
  return sqrt((state_1.x - state_2.x) * (state_1.x - state_2.x) +
              (state_1.y - state_2.y) * (state_1.y - state_2.y));
}

bool hasValidParent(state current) {
  return parents[current.x][current.y] != std::numeric_limits<int>::max();
}

state getParent(state current) {
  return parents[current.x][current.y];
}

void reconstructPath(state current) {
  while (hasValidParent(current)) {
    path.insert(path.begin(), current);
    current = getParent(current);
  }
}

void getCommand(int *left_velocity, int *right_velocity) {
  if (path[1].left_velocity > path[1].right_velocity) {
    *left_velocity = 30;
    *right_velocity = 22;
  } else if (path[1].left_velocity < path[1].right_velocity) {
    *left_velocity = 18;
    *right_velocity = 30;
  } else if ((path[1].left_velocity != 0) || (path[1].right_velocity != 0)) {
    *left_velocity = 22;
    *right_velocity = 25;
  }
}

bool isWalkable(state current) {
  double alpha = getParent(current).theta;
  std::vector<state> seed_points = seeds[current.seed_index].seed_points;

  for (int i = 0; i < seed_points.size(); i++) {
    int original_x, original_y;
    int transformed_x, transformed_y;

    original_x = seed_points[i].x;
    original_y = seed_points[i].y;

    transformed_x = (int) (original_x * sin(alpha * (CV_PI / 180)) +
                           original_y * cos(alpha * (CV_PI / 180)) + getParent(current).x);
    transformed_y = (int) (-original_x * cos(alpha * (CV_PI / 180)) +
                           original_y * sin(alpha * (CV_PI / 180)) + getParent(current).y);

    if (((0 <= transformed_x) && (transformed_x < MAP_SIZE)) && ((0 <= transformed_y) && (transformed_y < MAP_SIZE))) {
      if (map[transformed_x][transformed_y] != 0) {
        return false;
      }
    } else {
      return false;
    }
  }

  return true;
}

std::vector<state> getNeighbours(state current) {
  std::vector<state> neighbours;

  for (int i = 0; i < seeds.size(); i++) {
    state temp;

    temp.seed_index = seeds[i].index;
    temp.x = (int)(seeds[i].x * sin(current.theta * (CV_PI / 180)) +
                    seeds[i].y * cos(current.theta * (CV_PI / 180)) + current.x);
    temp.y = (int)(-seeds[i].x * cos(current.theta * (CV_PI / 180)) +
                    seeds[i].y * sin(current.theta * (CV_PI / 180)) + current.y);
    temp.theta = seeds[i].theta - (90 - current.theta);

    if ((0 <= temp.x) && (temp.x <= MAP_SIZE - 1) && (0 <= temp.y) && (temp.y <= MAP_SIZE - 1)) {
      if (isWalkable(temp)) {
        neighbours.insert(neighbours.begin(), temp);
      }
    }
  }

  return neighbours;
}

int getMembership(state current) {
  return membership[current.x][current.y];
}

void setMembership(state current, int membership_status) {
  membership[current.x][current.y] = membership_status;
}

double distanceBetween(state state_1, state state_2) {
  return sqrt((state_1.g_cost - state_2.g_cost) * (state_1.g_cost - state_2.g_cost));
}

void setParent(state current, state parent) {
  parents[current.x][current.y] = parent;
}

int main(int argc, char **argv) {
  int iteration_count;

  initPlanner(argc, argv);

  iteration_count = 0;

  while (1) {
    if (isTargetReached(start) == true) {
      printf("Target Reached\n");
      return;
    }

    std::priority_queue<state, std::vector<state>, CompareStates> open_list;

    start.g_cost = 0;
    start.f_cost = start.g_cost + heuristicCostEstimate(start, target);

    open_list.push(start);
    setMembership(start, OPEN_LIST);
    printf("> Pushing (%d, %d, %lf)\n", start.x, start.y, start.theta);

    while (open_list.size() != 0) {
      state current = open_list.top();
      printf(">> Current (%d, %d, %lf)\n", current.x, current.y, current.theta);

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
      
      std::vector<state> neighbours = getNeighbours(current);
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
