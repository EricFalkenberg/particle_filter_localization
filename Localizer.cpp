#include "Localizer.h"
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>

#define PI 3.14159265

#define NUM_POINTS 900.

Particle::Particle(double x, double y, double theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
    weight = .5;
}

void Particle::update_location(double delta_x, double delta_y, double delta_theta) {
    double distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
    this->theta += delta_theta;
    this->x += distance * cos(theta);
    this->y += distance * sin(theta);
}

void Particle::update_weight(sensor_msgs::LaserScan kinect_data, p2os_msgs::SonarArray sonar_data,
                             int8_t *MAP_DATA, int32_t MAP_WIDTH, int32_t MAP_HEIGHT, double MAP_RESOLUTION) {
    // PROCESS KINECT DATA
    double cumulative_error = 0.0;
    double inc  = 0.01;
    double curr_angle = this->theta + kinect_data.angle_min;
    int angleIdx = 0;
    while (curr_angle < this->theta + kinect_data.angle_max) {
        double curr_x = this->x;
        double curr_y = this->y;
        double curr_r = 0.0;
        // Starting pixel from slice
        int index = (int)
            (MAP_HEIGHT / 2 + curr_y/MAP_RESOLUTION) * MAP_WIDTH
            + (int)(curr_x/MAP_RESOLUTION + MAP_WIDTH/2); 
        while (MAP_DATA[index] != 100) {
            curr_r = curr_r + inc;
            curr_x = curr_r * cos(curr_angle);
            curr_y = curr_y * sin(curr_angle);
            index = (int)
                (MAP_HEIGHT / 2 + curr_y/MAP_RESOLUTION) * MAP_WIDTH
                + (int)(curr_x/MAP_RESOLUTION + MAP_WIDTH/2); 
        }
        if (curr_r < 8) {
            cumulative_error = cumulative_error + fabs(kinect_data.ranges[angleIdx]-curr_r);
        }
        curr_angle = curr_angle + kinect_data.angle_increment*20;
        angleIdx = angleIdx + 20;
    }
    if (cumulative_error == 0.0) {
        this->weight = 1.0;
    }
    else {
        this->weight = 1.0/cumulative_error;
    }
}

Localizer::Localizer(int8_t *MAP_DATA, int32_t MAP_WIDTH, int32_t MAP_HEIGHT, double MAP_RESOLUTION) {
    this->MAP_DATA = MAP_DATA;
    this->MAP_WIDTH = MAP_WIDTH;
    this->MAP_HEIGHT = MAP_HEIGHT;
    this->MAP_RESOLUTION = MAP_RESOLUTION;

    //generate points
    starting_locations.push_back(std::vector<double>());
    starting_locations.push_back(std::vector<double>());
    starting_locations.push_back(std::vector<double>());
    starting_locations.push_back(std::vector<double>());
    starting_locations.push_back(std::vector<double>());
    starting_locations.push_back(std::vector<double>());

    starting_locations[0].push_back(8);
    starting_locations[0].push_back(-.5);
    starting_locations[0].push_back(PI/2);

    starting_locations[1].push_back(-12);
    starting_locations[1].push_back(12);
    starting_locations[1].push_back(PI);

    starting_locations[2].push_back(-18.4);
    starting_locations[2].push_back(-8.9);
    starting_locations[2].push_back(0);

    starting_locations[3].push_back(10.8);
    starting_locations[3].push_back(12.7);
    starting_locations[3].push_back(PI);

    starting_locations[4].push_back(-54.5);
    starting_locations[4].push_back(7.6);
    starting_locations[4].push_back(PI);

    starting_locations[5].push_back(8);
    starting_locations[5].push_back(-1.5);
    starting_locations[5].push_back(-PI/2);


    create_particles_around(starting_locations[0][0], starting_locations[0][1], starting_locations[0][2]);
    create_particles_around(starting_locations[1][0], starting_locations[1][1], starting_locations[1][2]);
    create_particles_around(starting_locations[2][0], starting_locations[2][1], starting_locations[2][2]);
    create_particles_around(starting_locations[3][0], starting_locations[3][1], starting_locations[3][2]);
    create_particles_around(starting_locations[4][0], starting_locations[4][1], starting_locations[4][2]);
    create_particles_around(starting_locations[5][0], starting_locations[5][1], starting_locations[5][2]);

}

void Localizer::create_particles_around(double x, double y, double theta){
    srand(time(NULL));

    for(int i = 0; i < NUM_POINTS/6; i++){
        particles.push_back(new Particle(
            x + (rand() % (2 * (int)NUM_POINTS) - 1 * NUM_POINTS) / NUM_POINTS,
            y + (rand() % (2 * (int)NUM_POINTS) - 1 * NUM_POINTS) / NUM_POINTS,
            theta + PI/4 * ((rand() % (2 * (int)NUM_POINTS) - 1 * NUM_POINTS) / NUM_POINTS) 
        ));
    }
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

void Localizer::update_location(geometry_msgs::Pose pose_msg, sensor_msgs::LaserScan kinect_data, p2os_msgs::SonarArray sonar_data) {
    // Record current odom in the case that this is the first iteration.
    if(
        this->last_odom == NULL
    ) {
        last_odom = new geometry_msgs::Pose();
        this->last_odom->position.x = pose_msg.position.x;
        this->last_odom->position.y = pose_msg.position.y;
        this->last_odom->orientation.w = pose_msg.orientation.w;
        this->last_odom->orientation.z = pose_msg.orientation.z;
        return;
    }

    // Calculate deltas in x, y, theta
    double delta_x = pose_msg.position.x - this->last_odom->position.x;
    double delta_y = pose_msg.position.y - this->last_odom->position.y;
    double cur_angle = atan2(2 * (pose_msg.orientation.w * pose_msg.orientation.z), pose_msg.orientation.w*pose_msg.orientation.w - pose_msg.orientation.z*pose_msg.orientation.z);
    double last_angle = atan2(2 * (last_odom->orientation.w * last_odom->orientation.z), last_odom->orientation.w*last_odom->orientation.w - last_odom->orientation.z*last_odom->orientation.z);
    double delta_theta = cur_angle - last_angle;

    // Update all particle locations
    for(int i = 0; i < particles.size(); i++){
        particles[i]->update_location(delta_x, delta_y, delta_theta);
        particles[i]->update_weight(kinect_data, sonar_data, MAP_DATA, MAP_WIDTH, MAP_HEIGHT, MAP_RESOLUTION);
    }

    // Resample particles
    resample();

    // Set last odom for use in next iteration
    this->last_odom->position.x = pose_msg.position.x;
    this->last_odom->position.y = pose_msg.position.y;
    this->last_odom->orientation.w = pose_msg.orientation.w;
    this->last_odom->orientation.z = pose_msg.orientation.z;
}

void Localizer::resample() {
    std::vector<double> alphas;
    std::vector<Particle*> new_particles;

    srand(time(NULL));

    double weight_sum = 0;
    for(int i = 0; i < particles.size(); i++){
        int index = (int)
            (MAP_HEIGHT / 2 + particles[i]->y/MAP_RESOLUTION) * MAP_WIDTH
            + (int)(particles[i]->x/MAP_RESOLUTION + MAP_WIDTH/2);
        if(
            index < 0
            || index > MAP_WIDTH * MAP_HEIGHT
        ) {
            particles[i]->weight = 0;
            continue;
        }
        int pixel_val = MAP_DATA[index];
        
        if(
            pixel_val == 100
        ) {
            particles[i]->weight = 0;
        }
        alphas.push_back(particles[i]->weight);
        weight_sum += particles[i]->weight;
    }


    for(int i = 0; i < alphas.size(); i++){
        //TODO: REGENERATE ON THE WHOLE MAP IF NEED BE
        if(weight_sum != 0){
            alphas[i] = alphas[i] / weight_sum;
        }
        else{
            alphas[i] = 0;
        }
    }

    for(int i = 0; i < NUM_POINTS; i++){
        double sampled_alpha = (rand() % (1 * (int)NUM_POINTS)) / NUM_POINTS;
        double alpha_sum = 0;
        for(int j = 0; j < alphas.size(); j++){
            alpha_sum += alphas[j];
            if(
                alpha_sum >= sampled_alpha
            ) {
                Particle* p = new Particle(
                    particles[j]->x + (rand() % (2 * (int)NUM_POINTS) - 1 * NUM_POINTS) / (NUM_POINTS*100),
                    particles[j]->y + (rand() % (2 * (int)NUM_POINTS) - 1 * NUM_POINTS) / (NUM_POINTS*100),
                    particles[j]->theta + PI/4 * ((rand() % (2 * (int)NUM_POINTS) - 1 * NUM_POINTS) / (NUM_POINTS*100)) 
                );
                new_particles.push_back(p);
                break;
            }
        }
    }
    particles = new_particles;
}
