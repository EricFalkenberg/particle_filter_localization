#include <vector>
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "p2os_msgs/SonarArray.h"




/**
    a single particle. holds an x, y and theta location/orientation, and a weight value based on how well we think it describes the position of the robot
    based on sensor readings.
*/
class Particle {
    private:
        // private members
    public:
        // public members
        double x;
        double y;
        double theta;
        double weight;
        Particle(double x, double y, double theta);

        void update_location(double delta_x, double delta_y, double delta_theta);
        void update_weight(sensor_msgs::LaserScan kinect_data, p2os_msgs::SonarArray sonar_data,
                             int8_t *MAP_DATA, int32_t MAP_WIDTH, int32_t MAP_HEIGHT, double MAP_RESOLUTION, bool *sonar_change);
};

class Localizer {
    private:
        std::vector<Particle*> particles;
        geometry_msgs::Pose* last_odom;

        std::vector< std::vector<double> > starting_locations;

        void create_particles_around(double x, double y, double theta);

        int8_t *MAP_DATA;
        int32_t MAP_WIDTH;
        int32_t MAP_HEIGHT;
        double MAP_RESOLUTION;

        void resample();

        Particle* determine_location(sensor_msgs::LaserScan kinect_data, p2os_msgs::SonarArray sonar_data, bool *sonar_change);

    public:
        // public members
        Localizer(int8_t *MAP_DATA, int32_t MAP_WIDTH, int32_t MAP_HEIGHT, double MAP_RESOLUTION);
        Particle* update_location(geometry_msgs::Pose pose_msg, sensor_msgs::LaserScan kinect_data, p2os_msgs::SonarArray sonar_data, bool *sonar_change);
        geometry_msgs::PoseArray get_particle_poses();
};
