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
        Particle(double x, double y, double theta);

        void update_location(double delta_x, double delta_y, double delta_theta);
};

class Localizer {
    private:
        std::vector<Particle*> particles;
        nav_msgs::Odometry* last_odom;
    public:
        // public members
        Localizer();

        void update_location(nav_msgs::Odometry* odom_msg);

        geometry_msgs::PoseArray get_particle_poses();
};
