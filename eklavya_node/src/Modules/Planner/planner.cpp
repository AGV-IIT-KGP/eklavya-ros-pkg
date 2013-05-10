#include "planner.h"
#include "plannerMethods.h"

using namespace cv;
namespace planner_space {

    void Planner::loadPlanner() {
        loadSeeds();
        ROS_INFO("[PLANNER] Seeds Loaded");

        initBot();
        ROS_INFO("[PLANNER] Vehicle Initiated");
    }

    geometry_msgs::Twist Planner::findPath(Triplet bot, Triplet target, Mat data_img) {
        state start, goal;

        start.pose = bot;
        start.seed_id = -1;
        start.g_dist = 0;
        start.h_dist = distance(bot, target);
        start.g_obs = 0;
        start.h_obs = 0;
        start.depth = 0;

        goal.pose = target;
        goal.g_dist = 0;
        goal.h_dist = 0;
        goal.seed_id = 0;
        goal.g_obs = 0;
        goal.h_obs = 0;

        vector<state> open_list;
        open_list.insert(open_list.begin(), start);
        map<Triplet, open_map_element, PoseCompare> open_map;
        open_map[start.pose].membership = OPEN;
        open_map[start.pose].cost = start.g_dist;
        geometry_msgs::Twist cmdvel;
        map<Triplet, state, PoseCompare> came_from;

        brake.vl = brake.vr = 0;

        if (isEqual(start, goal)) {
            ROS_INFO("[PLANNER] Target Reached");
            Planner::finBot();
            return cmdvel;
        }

        int iterations = 0;
        while (!open_list.empty()) {
            //TODO: This condition needs to be handled in the strategy module.
            if (local_map[start.pose.x][start.pose.y] > 0) {
                ROS_WARN("[PLANNER] Robot is in Obstacles");
                Planner::finBot();
                return cmdvel;
            }

            state current = open_list.front();

#ifdef DEBUG
            cout << "==> CURRENT: ";
            print(current);

            plotPoint(data_img, current.pose);
            cv::imshow("[PLANNER] Map", data_img);

            cvWaitKey(0);
#endif

            if ((open_map.find(current.pose) != open_map.end()) &&
                    (open_map[current.pose].membership == CLOSED)) {
                pop_heap(open_list.begin(), open_list.end(), StateCompare());
                open_list.pop_back();
                continue;
            }

            if (isEqual(current, goal)) {
                cmdvel = reconstructPath(came_from, current, data_img);

#ifdef DEBUG
                ROS_INFO("[PLANNER] Path Found");

                cv::imshow("[PLANNER] Map", data_img);

                cvWaitKey(0);
#endif
                //                Mat hoho;
                //                resize(data_img, hoho, cvSize(400, 400));

                cv::imshow("[PLANNER] Map", data_img);
                cvWaitKey(WAIT_TIME);
                closePlanner();

                return cmdvel;
            }

            if (onTarget(current, goal)) {
                cmdvel = reconstructPath(came_from, current, data_img);

#ifdef DEBUG
                ROS_INFO("[PLANNER] Path Found");
                cv::imshow("[PLANNER] Map", data_img);
                cvWaitKey(0);
#endif
                closePlanner();

                return cmdvel;
            }

            pop_heap(open_list.begin(), open_list.end(), StateCompare());
            open_list.pop_back();
            open_map[current.pose].membership = UNASSIGNED;
            open_map[current.pose].cost = -1;

            open_map[current.pose].membership = CLOSED;

            vector<state> neighbors = neighborNodes(current);

            for (unsigned int i = 0; i < neighbors.size(); i++) {
                state neighbor = neighbors[i];

#ifdef DEBUG
                //plotPoint(grayImg,neighbor.pose);
#endif

                if (!(((neighbor.pose.x >= 0) && (neighbor.pose.x < MAP_MAX)) &&
                        ((neighbor.pose.y >= 0) && (neighbor.pose.y < MAP_MAX)))) {
                    continue;
                }

                if (!isWalkable(current, neighbor)) {
                    continue;
                }

                double tentative_g_score = neighbor.g_dist + current.g_dist;
                double admissible = distance(neighbor.pose, goal.pose);
                double consistent = admissible;

                // If already in open_list update cost and came_from if cost is less
                // This update is implicit since we allow duplicates in open list
                if (!((open_map.find(neighbor.pose) != open_map.end()) &&
                        (open_map[neighbor.pose].membership == OPEN))) {
                    came_from[neighbor. pose] = current;
                    neighbor.g_dist = tentative_g_score;
                    neighbor.h_dist = consistent;

                    //next condition is always true
                    if (!((open_map.find(neighbor.pose) != open_map.end()) &&
                            (open_map[neighbor.pose].membership == OPEN))) {
                        open_list.push_back(neighbor);
                        push_heap(open_list.begin(), open_list.end(), StateCompare());
                        open_map[neighbor.pose].membership = OPEN;
                        open_map[neighbor.pose].cost = neighbor.g_dist;
                    }
                }
            }

            iterations++;
            if (iterations > MAX_ITER) {
                ROS_WARN("[PLANNER] Open List Overflow");
                Planner::finBot();
                return cmdvel;
            }
        }

        ROS_ERROR("[PLANNER] No Path Found");
        closePlanner();
        Planner::finBot();
        return cmdvel;
    }

    geometry_msgs::Twist Planner::findPathDT(Triplet bot, Triplet target, Mat data_img) {
        state start, goal;

        start.pose = bot;
        start.seed_id = -1;
        start.g_dist = 0;
        start.h_dist = distance(bot, target);
        start.g_obs = 0;
        start.h_obs = 0;
        start.depth = 0;

        goal.pose = target;
        goal.g_dist = 0;
        goal.h_dist = 0;
        goal.seed_id = 0;
        goal.g_obs = 0;
        goal.h_obs = 0;

        vector<state> open_list;
        open_list.insert(open_list.begin(), start);
        map<Triplet, open_map_element, PoseCompare> open_map;
        open_map[start.pose].membership = OPEN;
        open_map[start.pose].cost = start.g_dist;
        geometry_msgs::Twist cmdvel;
        map<Triplet, state, PoseCompare> came_from;

        brake.vl = brake.vr = 0;

//        addObstacleP(data_img, 500, 500, 2);
//                addObstacleP(data_img, 100 + rand() % 600, 100 + rand() % 600, 10);
//                addObstacleP(data_img, 100 + rand() % 600, 100 + rand() % 600, 10);
//                addObstacleP(data_img, 100 + rand() % 600, 100 + rand() % 600, 10);
//                addObstacleP(data_img, 100 + rand() % 600, 100 + rand() % 600, 10);
//                addObstacleP(data_img, 100 + rand() % 600, 100 + rand() % 600, 10);

        Mat img;

        Mat dist;
        img = 255 - data_img;

        // Marking the boundaries for Voronoi Cost Field
        int voronoi_border = 10;
        cv::line(img, cvPoint(voronoi_border, voronoi_border),
                cvPoint(voronoi_border, MAP_MAX - voronoi_border),
                CV_RGB(0, 0, 0), 1, CV_AA, 0);
        cv::line(img, cvPoint(MAP_MAX - voronoi_border, voronoi_border),
                cvPoint(MAP_MAX - voronoi_border, MAP_MAX - voronoi_border),
                CV_RGB(0, 0, 0), 1, CV_AA, 0);

        threshold(img, img, 128, 255, THRESH_BINARY);

        dilate(img, img, 5);
        distanceTransform(img, dist, CV_DIST_L2, 5);
        normalize(dist, dist, 0.0, 1.0, NORM_MINMAX);

        GaussianBlur(dist, dist, cvSize(3, 3), 3);
        Laplacian(dist, dist, CV_32F, 1, 1, 0, BORDER_DEFAULT);

        double minVal, maxVal;
        minMaxLoc(dist, &minVal, &maxVal);
        Mat normImage;
        dist.convertTo(normImage, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));

        Mat huhu;
        threshold(normImage, huhu, 90, 255, THRESH_BINARY);

        // TODO: Need to implement this efficiently
        for (int i = 0; i < dist.rows; i++) {
            for (int j = 0; j < dist.cols; j++) {
                if (i < 20 || i > 980 || j < 20 || j > 980) {
                    huhu.at<uchar>(i, j) = 255;
                }
            }
        }

        Mat final;
        distanceTransform(huhu, final, CV_DIST_L2, 3);
        Mat samuel;
        normalize(final, samuel, 0.0, 1.0, NORM_MINMAX);
        minMaxLoc(samuel, &minVal, &maxVal);

        Mat dtLaplacian;
        final.convertTo(dtLaplacian, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));

        float vdt_max = 0;
        for (int i = 0; i < dist.rows; i++) {
            for (int j = 0; j < dist.cols; j++) {
                if (dist.at<float>(i, j) > vdt_max) {
                    vdt_max = dist.at<float>(i, j);
                }
            }
        }

        if (isEqual(start, goal)) {
            ROS_INFO("[PLANNER] Target Reached");
            Planner::finBot();
            return cmdvel;
        }

        int iterations = 0;
        while (!open_list.empty()) {
            //TODO: This condition needs to be handled in the strategy module.
            if (local_map[start.pose.x][start.pose.y] > 0) {
                ROS_WARN("[PLANNER] Robot is in Obstacles");
                Planner::finBot();
                return cmdvel;
            }

            state current = open_list.front();

#ifdef DEBUG
            cout << "current :g=" << current.g_dist << ":h= " << current.h_dist << ":g_= " << current.g_obs << "iterations : " << current.depth << ":h_= " << current.h_obs << endl;

            plotPoint(data_img, current.pose);
            cv::imshow("[PLANNER] Map", data_img);

            cvWaitKey(0);
#endif

            if ((open_map.find(current.pose) != open_map.end()) &&
                    (open_map[current.pose].membership == CLOSED)) {
                pop_heap(open_list.begin(), open_list.end(), StateCompareDT());
                open_list.pop_back();
                continue;
            }

            if (isEqual(current, goal)) {
                cmdvel = reconstructPath(came_from, current, data_img);

#ifdef DEBUG
                ROS_INFO("[PLANNER] Path Found");

                cv::imshow("[PLANNER] Map", data_img);

                cvWaitKey(0);
#endif

                cv::imshow("[PLANNER] Map", data_img);
                cvWaitKey(WAIT_TIME);
                closePlanner();

                return cmdvel;
            }

            if (onTarget(current, goal)) {
                cmdvel = reconstructPath(came_from, current, data_img);

#ifdef DEBUG
                ROS_INFO("[PLANNER] Path Found");
                cv::imshow("[PLANNER] Map", data_img);
                cvWaitKey(0);
#endif

                cv::imshow("[PLANNER] Map", data_img);
                cvWaitKey(WAIT_TIME);
                closePlanner();

                return cmdvel;
            }

            //TODO : why pop_heap 
            pop_heap(open_list.begin(), open_list.end(), StateCompareDT());
            open_list.pop_back();
            open_map[current.pose].membership = UNASSIGNED;
            open_map[current.pose].cost = -1;

            open_map[current.pose].membership = CLOSED;

            vector<state> neighbors = neighborNodes(current);

            for (unsigned int i = 0; i < neighbors.size(); i++) {
                state neighbor = neighbors[i];

#ifdef DEBUG
                plotPoint(grayImg, neighbor.pose);
#endif

                if (!(((neighbor.pose.x >= 0) && (neighbor.pose.x < MAP_MAX)) &&
                        ((neighbor.pose.y >= 0) && (neighbor.pose.y < MAP_MAX)))) {
                    continue;
                }

                if (!isWalkable(current, neighbor)) {
                    continue;
                }

                double tentative_g_dist_score = neighbor.g_dist + current.g_dist;
                double dist_admissible = distance(neighbor.pose, goal.pose);
                double dist_consistent = dist_admissible;

                double vdt_neighbor = final.at<float>(MAP_MAX - 1 - neighbor.pose.y, neighbor.pose.x);
                double vdt_goal = final.at<float>(MAP_MAX - 1 - goal.pose.y, goal.pose.x);
                double obs_admissible = (vdt_neighbor + vdt_goal) / 2;
                double obs_consistent = obs_admissible;
                neighbor.g_obs = vdt_neighbor;
                double tentative_g_obs_score = (neighbor.g_obs + current.g_obs * current.depth) / (current.depth + 1);
                //                double tentative_g_obs_score = (neighbor.g_obs + current.g_obs) / 2;
                
                if (!((open_map.find(neighbor.pose) != open_map.end()) &&
                        (open_map[neighbor.pose].membership == OPEN))) {
                    came_from[neighbor. pose] = current;
                    neighbor.g_dist = tentative_g_dist_score;
                    neighbor.h_dist = dist_consistent;

//                    neighbor.g_obs = tentative_g_obs_score;
                    neighbor.h_obs = obs_consistent;
                    double scaling_factor = 1 * neighbor.h_dist / (2 * vdt_max + vdt_goal);
                    //double scaling_factor = 1;
                    neighbor.g_obs *= scaling_factor;
                    neighbor.h_obs *= scaling_factor;
                    neighbor.depth = current.depth + 1;

                    //next condition is always true
                    if (!((open_map.find(neighbor.pose) != open_map.end()) &&
                            (open_map[neighbor.pose].membership == OPEN))) {
                        open_list.push_back(neighbor);
                        push_heap(open_list.begin(), open_list.end(), StateCompareDT());
                        open_map[neighbor.pose].membership = OPEN;
                        open_map[neighbor.pose].cost = neighbor.g_dist;
                    }
                }
            }

            iterations++;
            if (iterations > MAX_ITER) {
                ROS_WARN("[PLANNER] DT Open List Overflow - RETRYING w/o DT");
                ol_overflow = 1;
                Planner::finBot();
                return cmdvel;
            }
        }

        ROS_ERROR("[PLANNER] No Path Found");
        closePlanner();
        Planner::finBot();
        return cmdvel;
    }

    void Planner::finBot() {
        sendCommand(brake);
    }
}
