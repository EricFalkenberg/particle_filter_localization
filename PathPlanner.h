#include <stdint.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

class PathPlanner {
    private:
        // PRIVATE MEMBERS
        int8_t* MAP_DATA;
        int MAP_WIDTH;
        int MAP_HEIGHT;
        double MAP_RESOLUTION;
        int8_t get_pixel_val(double x, double y);
    public:
        // PUBLIC MEMBERS
        PathPlanner(int8_t* MAP_DATA, int MAP_WIDTH, int MAP_HEIGHT, double MAP_RESOLUTION);
        nav_msgs::Path plan(double x0, double y0, double x1, double y1);
        std::vector< geometry_msgs::PoseStamped > get_successors(geometry_msgs::PoseStamped node_current);
        geometry_msgs::PoseStamped get_local_optimal(std::vector< geometry_msgs::PoseStamped >* poses, double start_x, double start_y, double goal_x, double goal_y);
        bool target_in_list(std::vector< geometry_msgs::PoseStamped > list, geometry_msgs::PoseStamped target);
        std::vector< geometry_msgs::PoseStamped > remove_target_from_list(std::vector< geometry_msgs::PoseStamped > list, geometry_msgs::PoseStamped target);
        geometry_msgs::PoseStamped find_parent(std::vector< std::vector< geometry_msgs::PoseStamped > > history, geometry_msgs::PoseStamped target);
        nav_msgs::Path compression(nav_msgs::Path path);
};
