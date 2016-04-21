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
#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "p2os_msgs/MotorState.h"
#include "p2os_msgs/SonarArray.h"
#define PI 3.14159265

/**
    Represents a destination point on the (x,y) plane.
*/
struct dest {
    double x;
    double y;
    bool operator<( const dest &n) const {
        return this->x < n.x;
    }
};


class PointRobot {
private:
    std::vector< std::vector<double> > vec;
    // The amout of error we will allow.
    double VARIANCE;
    // The angular velocity of the robot.
    double ANGULAR_VELOCITY;
    // The forward velocity of the robot.
    double DEFAULT_SPEED;
    // A queue of destination points.
    std::queue<dest> destinations;
    // The pose information returned from /r1/odom
    geometry_msgs::Pose pose;
    // The twist information returned from /r1/odom
    geometry_msgs::Twist twist;
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
    int run(int argc, char** argv, bool run_kinect, bool run_sonar);
    std::queue<dest> read_file(char *file);
};


