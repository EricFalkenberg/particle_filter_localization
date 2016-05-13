#include "PathPlanner.h"
#include "math.h"
#include "stdio.h"
#include <cmath>
#define ALLOWED_ERROR 0.25

PathPlanner::PathPlanner(int8_t* MAP_DATA, int MAP_WIDTH, int MAP_HEIGHT, double MAP_RESOLUTION) {
    // CONSTRUCTOR
    this->MAP_DATA = MAP_DATA;
    this->MAP_WIDTH = MAP_WIDTH;
    this->MAP_HEIGHT = MAP_HEIGHT;
    this->MAP_RESOLUTION= MAP_RESOLUTION;
}

int8_t PathPlanner::get_pixel_val(double x, double y) {
    return MAP_DATA[(int)(MAP_HEIGHT / 2 + y/MAP_RESOLUTION) * MAP_WIDTH + (int)(x/MAP_RESOLUTION + MAP_WIDTH/2)];
}

geometry_msgs::PoseStamped PathPlanner::get_local_optimal(std::vector< geometry_msgs::PoseStamped >* poses, double start_x, double start_y, double goal_x, double goal_y) {
    geometry_msgs::PoseStamped min_p;
    double min_h = -1;
    int best_idx = -1;
    for (int idx = 0; idx < poses->size(); idx++) {
        double g = sqrt(pow(start_x-(*poses)[idx].pose.position.x, 2)+pow(start_y-(*poses)[idx].pose.position.y, 2));
        double h = sqrt(pow(goal_x-(*poses)[idx].pose.position.x, 2)+pow(goal_y-(*poses)[idx].pose.position.y, 2));
        if (min_h == -1 || g+h < min_h) {
            min_p = (*poses)[idx];
            min_h = g+h;
            best_idx = idx;
        }
    }
    poses->erase(poses->begin()+best_idx);
    return min_p;
}

std::vector< geometry_msgs::PoseStamped > PathPlanner::get_successors(geometry_msgs::PoseStamped node_current) {
    std::vector< geometry_msgs::PoseStamped > successors;
    geometry_msgs::PoseStamped n_node  = geometry_msgs::PoseStamped();
    geometry_msgs::PoseStamped e_node  = geometry_msgs::PoseStamped();
    geometry_msgs::PoseStamped w_node  = geometry_msgs::PoseStamped();
    geometry_msgs::PoseStamped s_node  = geometry_msgs::PoseStamped();
    geometry_msgs::PoseStamped ul_node = geometry_msgs::PoseStamped();
    geometry_msgs::PoseStamped ur_node = geometry_msgs::PoseStamped();
    geometry_msgs::PoseStamped ll_node = geometry_msgs::PoseStamped();
    geometry_msgs::PoseStamped lr_node = geometry_msgs::PoseStamped();
    if (get_pixel_val(node_current.pose.position.x, node_current.pose.position.y + ALLOWED_ERROR*2) == 0) {
        n_node.pose.position.x = node_current.pose.position.x; n_node.pose.position.y = node_current.pose.position.y+ALLOWED_ERROR;
        successors.push_back(n_node);
    }
    if (get_pixel_val(node_current.pose.position.x + ALLOWED_ERROR*2, node_current.pose.position.y) == 0) {
        e_node.pose.position.x = node_current.pose.position.x+ALLOWED_ERROR; e_node.pose.position.y = node_current.pose.position.y;
        successors.push_back(e_node);
    }
    if (get_pixel_val(node_current.pose.position.x, node_current.pose.position.y - ALLOWED_ERROR*2) == 0) {
        s_node.pose.position.x = node_current.pose.position.x; s_node.pose.position.y = node_current.pose.position.y-ALLOWED_ERROR;
        successors.push_back(s_node);
    }
    if (get_pixel_val(node_current.pose.position.x - ALLOWED_ERROR*2, node_current.pose.position.y) == 0) {
        w_node.pose.position.x = node_current.pose.position.x-ALLOWED_ERROR; w_node.pose.position.y = node_current.pose.position.y;
        successors.push_back(w_node);
    }
    if (get_pixel_val(node_current.pose.position.x+ALLOWED_ERROR*2, node_current.pose.position.y+ALLOWED_ERROR*2) == 0) {
        ur_node.pose.position.x = node_current.pose.position.x+ALLOWED_ERROR; ur_node.pose.position.y = node_current.pose.position.y+ALLOWED_ERROR;
        successors.push_back(ur_node);
    }
    if (get_pixel_val(node_current.pose.position.x+ALLOWED_ERROR*2, node_current.pose.position.y-ALLOWED_ERROR*2) == 0) {
        lr_node.pose.position.x = node_current.pose.position.x+ALLOWED_ERROR; lr_node.pose.position.y = node_current.pose.position.y-ALLOWED_ERROR;
        successors.push_back(lr_node);
    }
    if (get_pixel_val(node_current.pose.position.x-ALLOWED_ERROR*2, node_current.pose.position.y+ALLOWED_ERROR*2) == 0) {
        ul_node.pose.position.x = node_current.pose.position.x-ALLOWED_ERROR; ul_node.pose.position.y = node_current.pose.position.y+ALLOWED_ERROR;
        successors.push_back(ul_node);
    }
    if (get_pixel_val(node_current.pose.position.x-ALLOWED_ERROR*2, node_current.pose.position.y-ALLOWED_ERROR*2) == 0) {
        ll_node.pose.position.x = node_current.pose.position.x-ALLOWED_ERROR; ll_node.pose.position.y = node_current.pose.position.y-ALLOWED_ERROR;
        successors.push_back(ll_node);
    }
    return successors;
}

bool PathPlanner::target_in_list(std::vector< geometry_msgs::PoseStamped > list, geometry_msgs::PoseStamped target) {
    for (int idx = 0; idx < list.size(); idx++) {
        if (list[idx].pose.position.x == target.pose.position.x && list[idx].pose.position.y == target.pose.position.y) {
            return true;
        }
    }
    return false;
}

std::vector< geometry_msgs::PoseStamped > PathPlanner::remove_target_from_list(std::vector< geometry_msgs::PoseStamped > list, geometry_msgs::PoseStamped target) {
    int toRemove = -1;
    for (int idx = 0; idx < list.size(); idx++) {
        if (list[idx].pose.position.x == target.pose.position.x && list[idx].pose.position.y == target.pose.position.y) {
            toRemove = idx;
        }
    }
    list.erase(list.begin()+toRemove);
    return list;
}

geometry_msgs::PoseStamped PathPlanner::find_parent(std::vector< std::vector< geometry_msgs::PoseStamped > > history, geometry_msgs::PoseStamped target) {
    for (int idx = 0; idx < history[0].size(); idx++) {
        if (history[0][idx].pose.position.x == target.pose.position.x && history[0][idx].pose.position.y == target.pose.position.y) {
            return history[1][idx];
        }
    }
    geometry_msgs::PoseStamped p = geometry_msgs::PoseStamped();
    p.pose.position.x = 10000;
    return p;
}

nav_msgs::Path PathPlanner::plan(double x0, double y0, double x1, double y1) {
    std::vector< geometry_msgs::PoseStamped > open;
    std::vector< geometry_msgs::PoseStamped > close;
    std::vector< std::vector< geometry_msgs::PoseStamped > > history(2);
    geometry_msgs::PoseStamped start = geometry_msgs::PoseStamped(); start.pose.position.x = x0; start.pose.position.y = y0;
    open.push_back(start);
    geometry_msgs::PoseStamped node_current;
    while (!open.empty()) {
        node_current = get_local_optimal(&open, x0, y0, x1, y1);
        if (fabs(node_current.pose.position.x-x1) < ALLOWED_ERROR 
            && fabs(node_current.pose.position.y-y1) < ALLOWED_ERROR) {
            break;
        }
        close.push_back(node_current);
        std::vector< geometry_msgs::PoseStamped > successors = this->get_successors(node_current);
        double g_curr = sqrt(pow(x0-node_current.pose.position.x, 2)+pow(y0-node_current.pose.position.y, 2));
        for (int sIdx = 0; sIdx < successors.size(); sIdx++) {
            double g_succ = sqrt(pow(x0-successors[sIdx].pose.position.x, 2)+pow(y0-successors[sIdx].pose.position.y, 2));
            if (target_in_list(open, successors[sIdx])) {
                if (g_succ <= g_curr) {
                    open.push_back(successors[sIdx]);
                    history[0].push_back(successors[sIdx]);
                    history[1].push_back(node_current);
                }
            }
            else if (target_in_list(close, successors[sIdx])) {
                continue;
            }
            else {
                open.push_back(successors[sIdx]);
                history[0].push_back(successors[sIdx]);
                history[1].push_back(node_current); 
            }
        }
    }
    // CHECK THAT GOAL HAS ACTUALLY BEEN MET
    if (fabs(node_current.pose.position.x-x1) > 0.6 || fabs(node_current.pose.position.y-y1) > 0.6) {
        //printf("DEST: %.2f, %.2f\n", node_current.pose.position.x, node_current.pose.position.y);
        printf("COULDNT FIND GOAL\n");
    }
    nav_msgs::Path path = nav_msgs::Path();
    path.header.frame_id = "/map";
    geometry_msgs::PoseStamped curr_pose = node_current;
    while (curr_pose.pose.position.x != 10000) {
        path.poses.push_back(curr_pose);
        curr_pose = find_parent(history, curr_pose);
    } 
    return path;
}
