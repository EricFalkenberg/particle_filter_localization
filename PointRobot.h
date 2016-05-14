#include <stdint.h>
#include <string>
#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <math.h>
#include <fstream>
#include <stack>
#include <boost/thread.hpp>
#include <boost/timer.hpp>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "sensor_msgs/LaserScan.h"
#include "p2os_msgs/MotorState.h"
#include "p2os_msgs/SonarArray.h"
#include <nav_msgs/Path.h>
#include "Localizer.h"
#include "PathPlanner.h"
#define PI 3.14159265

struct dest {
    double x;
    double y;
    bool operator<( const dest &n) const {
        return this->x < n.x;
    }
};

class PointRobot {
private:
    int8_t *MAP_DATA;
    int32_t MAP_WIDTH;
    int32_t MAP_HEIGHT;
    double MAP_RESOLUTION;

    Localizer *localizer;
    PathPlanner *path_planner;
    // The amout of error we will allow.
    double VARIANCE;
    // The angular velocity of the robot.
    double ANGULAR_VELOCITY;
    // The forward velocity of the robot.
    double DEFAULT_SPEED;
    // A queue of destination points.
    std::queue<dest> destinations;
    // A flag that indicates whether we're currently avoiding an obstacle
    bool avoiding;
    //The intermediary destination while avoiding
    dest avoidance_dest;
    // The pose information returned from /r1/odom
    geometry_msgs::Pose    pose;
    sensor_msgs::LaserScan kinect_data;
    p2os_msgs::SonarArray  sonar_data;

    bool *sonar_change;

    Particle* suspectedLocation;

    nav_msgs::Path gotoPoints;

    bool pathCalculated;

    // The twist information returned from /r1/odom
    //geometry_msgs::Twist twist;

    ros::Publisher point_cloud_pub;
    ros::Publisher path_planning_pub;
public:
    PointRobot(char* fname, double SPEED, double VARIANCE);
    void whereAmI();
    void updateMap();
    void plotSonar(double x0, double y0, double x1, double y1);
    void plotKinect(double x0, double y0, double x1, double y1);
    void sonarCallback(const p2os_msgs::SonarArray msgs);
    void kinectCallback(const sensor_msgs::LaserScan msgs);
    void odomCallback(const nav_msgs::Odometry msgs);
    double getAngularVelocity();
    double getForwardVelocity();
    void boundary_following();
    int run(int argc, char** argv, bool run_kinect, bool run_sonar);
    std::queue<dest> read_file(char *file);
    void read_image(const char *file);
};


