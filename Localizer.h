#include <vector>
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"




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

    public:
        // public members
        Localizer(int8_t *MAP_DATA, int32_t MAP_WIDTH, int32_t MAP_HEIGHT, double MAP_RESOLUTION);

        void update_location(geometry_msgs::Pose pose_msg);

        geometry_msgs::PoseArray get_particle_poses();
};
