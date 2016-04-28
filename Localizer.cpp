#include "Localizer.h"
#include <tf/transform_broadcaster.h>

#define PI 3.14159265

Particle::Particle(double x, double y, double theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

void Particle::update_location(double delta_x, double delta_y, double delta_theta) {
    this->x += delta_x;
    this->y += delta_y;
    this->theta += delta_theta;
}

Localizer::Localizer() {
    //generate points
    particles.insert(particles.end(), new Particle(8.0, -1.0, PI/2));

}

geometry_msgs::PoseArray Localizer::get_particle_poses(){
    geometry_msgs::PoseArray particle_poses;
       particle_poses.header.frame_id = "/map";

    for(int i = 0; i < particles.size(); i++){
        geometry_msgs::Pose tmp;
        tmp.position.x = particles[i]->x;
        tmp.position.y = particles[i]->y;
        tmp.orientation = tf::createQuaternionMsgFromYaw(particles[i]->theta);
        particle_poses.poses.push_back(tmp);
    }

    return particle_poses;
}

void Localizer::update_location(nav_msgs::Odometry* odom_msg) {
    //calculate deltas
    //call update on every particle

    if(this->last_odom == NULL){
        this->last_odom = odom_msg;
        return;
    }

    double delta_x = odom_msg->pose.pose.position.x - this->last_odom->pose.pose.position.x;
    double delta_y = odom_msg->pose.pose.position.y - this->last_odom->pose.pose.position.y;



    double cur_angle = atan2(2 * (odom_msg->pose.pose.orientation.w * odom_msg->pose.pose.orientation.z), odom_msg->pose.pose.orientation.w*odom_msg->pose.pose.orientation.w - odom_msg->pose.pose.orientation.z*odom_msg->pose.pose.orientation.z);
    double last_angle = atan2(2 * (last_odom->pose.pose.orientation.w * last_odom->pose.pose.orientation.z), last_odom->pose.pose.orientation.w*last_odom->pose.pose.orientation.w - last_odom->pose.pose.orientation.z*last_odom->pose.pose.orientation.z);
    double delta_theta = last_angle - cur_angle;

    for(int i = 0; i < particles.size(); i++){
        particles[i]->update_location(delta_x, delta_y, delta_theta);
    }

    this->last_odom = odom_msg;
}
