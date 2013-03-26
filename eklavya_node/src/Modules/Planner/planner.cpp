#include "planner.h"

/**
 * Control Modes:
 * 0: No PID
 * 1: Yaw PID with NO Thread
 * 2: Yaw PID with Controller Thread
 */

#define PID_MODE 0

#define SIMCTL
#define SIM_SEEDS

/**
 * Seed Files: 
 * seeds3.txt is valid but gives suboptimal results. Good Path. (6 - 8
 * seeds1.txt is for validation purposes ONLY. Grid A* - like path. Might be useful with DT. (101)
 * seeds.txt contains the full set of paths. Almost always stuck in inf loop.
 * seeds4.txt contains 5 seeds with arc-length ~75. Works fine. (4)
 * seeds5.txt contains 5 seeds with arc-lengths varying from 100 to 50. (2)
 * seeds2.txt contains 5 seeds 75 - 100 - 50 (16-20)
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
        double g, h; // costs
        int seed_id;
    } state;

    typedef struct seed_point {
        double x, y;
    } seed_point;

    typedef struct seed {
        Triplet dest;
        double cost;
        double k; // velocity ratio
        double vl, vr; // individual velocities
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
        }
    };

    struct StateCompare : public std::binary_function<state, state, bool> {

        bool operator() (state const& state_1, state const& state_2) const {
            double f1 = state_1.g + state_1.h;
            double f2 = state_2.g + state_2.h;

            return f1 > f2;
        }
    };

    Triplet bot, target;
    vector<seed> seeds;
    Tserial *p;

    pthread_mutex_t controllerMutex;
    volatile double targetCurvature = 1;

    /// ------------------------------------------------------------- ///

    void *controllerThread(void *arg) {
        double myTargetCurvature;
        double myYaw = 0.5, previousYaw = 1, Kp = 5;
        int left_vel = 0, right_vel = 0;

        while (ros::ok()) {
            pthread_mutex_lock(&controllerMutex);
            myTargetCurvature = targetCurvature;
            pthread_mutex_unlock(&controllerMutex);

            previousYaw = myYaw;
            pthread_mutex_lock(&pose_mutex);
            myYaw = pose.orientation.z;
            pthread_mutex_unlock(&pose_mutex);

            left_vel = 40 + Kp * (myTargetCurvature - (myYaw - previousYaw) / 0.5);
            right_vel = 40 - Kp * (myTargetCurvature - (myYaw - previousYaw) / 0.5);

            printf("[INFO] [Controller] %lf , %lf , %d , %d\n", myTargetCurvature, (myYaw - previousYaw)*2, left_vel, right_vel);

#ifndef SIMCTL
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

            usleep(10000);
        }

        return NULL;
    }

    void loadSeeds() {
        int n_seeds;
        int return_status;
        double x, y, z;
        FILE *fp = fopen(SEEDS_FILE, "r");
        return_status = fscanf(fp, "%d\n", &n_seeds);
        if (return_status == 0) {
            cout << "Error in reading seeds" << endl;
            Planner::finBot();
            exit(1);
        }

        for (int i = 0; i < n_seeds; i++) {
            seed s;

#ifdef SIM_SEEDS
            return_status = fscanf(fp, "%lf %lf %lf %lf %lf\n", &s.k, &x, &y, &z, &s.cost);
            if (return_status == 0) {
                cout << "Error in reading seeds" << endl;
                Planner::finBot();
                exit(1);
            }

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
            return_status = fscanf(fp, "%d\n", &n_seed_points);
            if (return_status == 0) {
                cout << "Error in reading seeds" << endl;
                Planner::finBot();
                exit(1);
            }

            for (int j = 0; j < n_seed_points; j++) {
                seed_point point;
                return_status = fscanf(fp, "%lf %lf\n", &point.x, &point.y);
                if (return_status == 0) {
                    cout << "Error in reading seeds" << endl;
                    Planner::finBot();
                    exit(1);
                }

                s.seed_points.insert(s.seed_points.begin(), point);
            }
            seeds.insert(seeds.begin(), s);
        }
    }

    double distance(Triplet a, Triplet b) {
        return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }

    bool isEqual(state a, state b) {
        double error = sqrt((50 ^ 2) + (50 ^ 2));
        return (sqrt((a.pose.x - b.pose.x) * (a.pose.x - b.pose.x) +
                (a.pose.y - b.pose.y) * (a.pose.y - b.pose.y)) < error);
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
        cvLine(
                map_img,
                cvPoint(ax, ay),
                cvPoint(bx, by),
                CV_RGB(rand() % 255, rand() % 255, rand() % 255),
                2,
                CV_AA,
                0);
    }

    void startThread(pthread_t *thread_id, pthread_attr_t *thread_attr, void *(*thread_name) (void *)) {
        if (pthread_create(thread_id, thread_attr, thread_name, NULL)) {
            cout << "[PLANNER] Unable to create thread" << endl;
            pthread_attr_destroy(thread_attr);
            exit(1);
        }
        sleep(1);
    }

    void initBot() {
        if (PID_MODE == 2) {
            pthread_attr_t attr;
            pthread_t controller_id;

            pthread_mutex_init(&controllerMutex, NULL);
            pthread_mutex_trylock(&controllerMutex);
            pthread_mutex_unlock(&controllerMutex);

            pthread_attr_init(&attr);
            pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

            startThread(&controller_id, &attr, &controllerThread);

            pthread_attr_destroy(&attr);
        }

#ifndef SIMCTL
        p = new Tserial();
        p->connect(BOT_COM_PORT, BOT_BAUD_RATE, spNONE);
        usleep(100);

        char arr[2] = {' ', ' '};
        p->sendArray(arr, 2);
        usleep(100);

        p->disconnect();
        usleep(100);
#endif
    }

    void sendCommand(seed s) {
        int left_vel = 0;
        int right_vel = 0;
        float left_velocity = s.vl;
        float right_velocity = s.vr;

        if ((left_velocity == 0) && (right_velocity == 0)) {

#ifndef SIMCTL
            p->connect(BOT_COM_PORT, BOT_BAUD_RATE, spNONE);
            usleep(100);

            p->sendChar(' ');
            usleep(100);

            p->disconnect();
            usleep(100);
#endif

            return;
        }

        switch (PID_MODE) {
            case 0:
            {
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

                break;
            }
            case 1:
            {
                double myTargetCurvature = 5.0 * ((double) (left_velocity - right_velocity)) / (left_velocity + right_velocity);
                static double myYaw = 0.5;
                static double previousYaw = 1;
                static double errorSum = 0;
                static double previousError;
                double Kp = 6.4, Kd = 0.01, Ki = 0.0001;
                left_vel = 0;
                right_vel = 0;
                int mode = 4;

                double error = (myTargetCurvature - (myYaw - previousYaw) / 0.37);
                errorSum += error;

                if (s.k > 1.35) {
                    mode = 2;
                } else if (s.k > 1.25) {
                    mode = 3;
                } else if (s.k > 0.99) {
                    mode = 4;
                } else if (s.k > 0.79) {
                    mode = 5;
                } else if (s.k > 0.73) {
                    mode = 6;
                } else {
                    mode = 4;
                }

                //int hashSpeedLeft[9] = {30, 33, 36, 38, 40, 42, 44, 47, 50};
                //int hashSpeedRight[9] = {50, 47, 44, 42, 40, 38, 36, 33, 30};

                int hashSpeedLeft[9] = {23, 30, 35, 38, 40, 42, 45, 50, 57};
                int hashSpeedRight[9] = {57, 50, 45, 42, 40, 38, 35, 30, 23};

                previousYaw = myYaw;

                pthread_mutex_lock(&pose_mutex);
                myYaw = pose.orientation.z;
                pthread_mutex_unlock(&pose_mutex);

                mode += (int) (Kp * error + Ki * errorSum + Kd * (error - previousError));

                if (mode < 0) {
                    mode = 0;
                }
                if (mode > 8) {
                    mode = 8;
                }

                left_vel = hashSpeedLeft[mode];
                right_vel = hashSpeedRight[mode];

                printf("[PID] %lf, %lf,  %lf, %lf, %d, %d\n",
                        myTargetCurvature,
                        (myYaw - previousYaw) / 0.37,
                        left_velocity,
                        right_velocity,
                        left_vel,
                        right_vel);

                break;
            }
            case 2:
            {
                pthread_mutex_lock(&controllerMutex);
                targetCurvature = 5.0 * ((double) (s.k - 1.0)) / (s.k + 1.0);
                printf("Updated : %lf, k = %lf, left = %lf, right = %lf\n", targetCurvature, s.k, s.vl, s.vr);
                pthread_mutex_unlock(&controllerMutex);

                break;
            }
        }

#ifndef SIMCTL
        p->connect(BOT_COM_PORT, BOT_BAUD_RATE, spNONE);
        usleep(100);

        char arr[5] = {'w',
            '0' + left_vel / 10,
            '0' + left_vel % 10,
            '0' + right_vel / 10,
            '0' + right_vel % 10};

        p->sendArray(arr, 5);
        usleep(100);

        p->disconnect();
        usleep(100);
#endif  

        printf("[Planner] [COMMAND] : (%d, %d)\n", left_vel, right_vel);
    }

    void reconstructPath(map<Triplet, state, PoseCompare> came_from, state current) {
        pthread_mutex_lock(&path_mutex);

        path.clear();

        int seed_id = -1;
        state s = current;
        while (came_from.find(s.pose) != came_from.end()) {
            path.insert(path.begin(), s.pose);
            seed_id = s.seed_id;
            s = came_from[s.pose];
        }

        pthread_mutex_unlock(&path_mutex);

        if (seed_id != -1) {
            sendCommand(seeds[seed_id]);
        } else {
            cout << "Invalid Command Ordered" << endl;
            Planner::finBot();
        }
    }

    void reconstructPath(map<Triplet, state, PoseCompare> came_from, IplImage *map_img, state current) {
        pthread_mutex_lock(&path_mutex);

        path.clear();

        int seed_id = -1;
        state s = current;
        while (came_from.find(s.pose) != came_from.end()) {
            plotPoint(map_img, s.pose);
            path.insert(path.begin(), s.pose);
            seed_id = s.seed_id;
            s = came_from[s.pose];
        }

        cvShowImage("Map", map_img);
        cvWaitKey(1);
        
        pthread_mutex_unlock(&path_mutex);

        if (seed_id != -1) {
            sendCommand(seeds[seed_id]);
        } else {
            cout << "Invalid Command Requested" << endl;
            Planner::finBot();
        }
    }
    
    vector<state> neighborNodes(state current) {
        vector<state> neighbours;
        for (unsigned int i = 0; i < seeds.size(); i++) {
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
        cout << "{ " <<
                s.pose.x << " , " <<
                s.pose.y << " , " <<
                s.pose.z << " , " <<
                f << " }" << endl;
    }

    bool isWalkable(state parent, state s) {
        for (unsigned int i = 0; i < seeds[s.seed_id].seed_points.size(); i++) {
            int x, y;
            double alpha = parent.pose.z;

            int tx, ty;
            tx = seeds[s.seed_id].seed_points[i].x;
            ty = seeds[s.seed_id].seed_points[i].y;

            x = (int) (tx * sin(alpha * (CV_PI / 180)) + ty * cos(alpha * (CV_PI / 180)) + parent.pose.x);
            y = (int) (-tx * cos(alpha * (CV_PI / 180)) + ty * sin(alpha * (CV_PI / 180)) + parent.pose.y);

            if (((0 <= x) && (x < MAP_MAX)) && ((0 <= y) && (y < MAP_MAX))) {
                return local_map[x][y] == 0;
            } else {
                return false;
            }
        }

        return true;
    }

    void closePlanner() {
    }

    void addObstacle(IplImage *map_img, int x, int y, int r) {
        for (int i = -r; i < r; i++) {
            for (int j = -r; j < r; j++) {
                local_map[x + i][y + j] = 255;
            }
        }
    }

    /// ------------------------------------------------------------- ///

    void Planner::loadPlanner() {
        loadSeeds();
        cout << "Seeds Loaded" << endl;

        initBot();
        cout << "Vehicle Initiated" << endl;
    }

    void Planner::findPath(Triplet bot, Triplet target) {
        state start, goal;
        start.pose = bot;
        start.seed_id = -1;
        start.g = 0;
        start.h = distance(bot, target);
        goal.pose = target;
        goal.g = 0;
        goal.h = 0;
        goal.seed_id = 0;

        vector<state> open_list;
        open_list.insert(open_list.begin(), start);
        map<Triplet, open_map_element, PoseCompare> open_map;
        open_map[start.pose].membership = OPEN;
        open_map[start.pose].cost = start.g;

        map<Triplet, state, PoseCompare> came_from;

        while (!open_list.empty()) {
            //TODO: This condition needs to be handled in the strategy module.
            if (local_map[start.pose.x][start.pose.y] > 0) {
                cout << "Robot is in Obstacles \n";
                seed *s = new seed;
                s->vl = s->vr = 0;
                sendCommand(*s);
            }

            state current = open_list.front();

            if ((open_map.find(current.pose) != open_map.end()) &&
                    (open_map[current.pose].membership == CLOSED)) {
                continue;
            }

            if (isEqual(current, goal)) {
                reconstructPath(came_from, current);

                closePlanner();
                return;
            }

            pop_heap(open_list.begin(), open_list.end(), StateCompare());
            open_list.pop_back();
            open_map[current.pose].membership = UNASSIGNED;
            open_map[current.pose].cost = -1;

            open_map[current.pose].membership = CLOSED;

            vector<state> neighbors = neighborNodes(current);

            for (unsigned int i = 0; i < neighbors.size(); i++) {
                state neighbor = neighbors[i];

                if (!(((neighbor.pose.x >= 0) && (neighbor.pose.x < MAP_MAX)) &&
                        ((neighbor.pose.y >= 0) && (neighbor.pose.y < MAP_MAX)))) {
                    continue;
                }

                if (!isWalkable(current, neighbor)) {
                    continue;
                }

                double tentative_g_score = neighbor.g + current.g;
                double admissible = distance(neighbor.pose, goal.pose);
                //double consistent = max(admissible, current.h - neighbor.g);
                double consistent = admissible;

                if (!((open_map.find(neighbor.pose) != open_map.end()) &&
                        (open_map[neighbor.pose].membership == OPEN))) {
                    came_from[neighbor.pose] = current;
                    neighbor.g = tentative_g_score;
                    neighbor.h = consistent;

                    if (!((open_map.find(neighbor.pose) != open_map.end()) &&
                            (open_map[neighbor.pose].membership == OPEN))) {
                        open_list.push_back(neighbor);
                        push_heap(open_list.begin(), open_list.end(), StateCompare());
                        open_map[neighbor.pose].membership = OPEN;
                        open_map[neighbor.pose].cost = neighbor.g;
                    }
                }
            }
        }

        closePlanner();
        cout << "[PLANNER] No Path Found" << endl;

        seed *s1 = new seed;
        s1->vl = 0.0;
        s1->vr = 0.0;
        sendCommand(*s1);
    }

    void Planner::finBot() {
#ifndef SIMCTL
        //p = new Tserial();
        p->connect(BOT_COM_PORT, BOT_BAUD_RATE, spNONE);
        usleep(100);

        p->sendChar(' ');
        usleep(100);

        p->disconnect();
        usleep(100);
#endif
    }


}
