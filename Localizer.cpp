#include "Localizer.h"
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>

#define PI 3.14159265

#define NUM_POINTS 1200.


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
                             int8_t *MAP_DATA, int32_t MAP_WIDTH, int32_t MAP_HEIGHT, double MAP_RESOLUTION, bool *sonar_change) {
    
    // printf("starting update\n");

    // PROCESS KINECT DATA
    double cumulative_error = 0.0;
    double inc  = 0.1;
    double curr_angle = this->theta + kinect_data.angle_min;
    // if(curr_angle > 3.14159265){
    //     curr_angle -= 2*3.14159265;
    // }
    // else if(curr_angle < -3.14159265){
    //     curr_angle += 2*3.14150265;
    // }
    int angleIdx = 0;
    if(curr_angle >= this->theta+kinect_data.angle_max){cumulative_error=100;}
    while (curr_angle < this->theta + kinect_data.angle_max) {
        // printf("currangle: %f\n", curr_angle);
        // double curr_x = this->x;
        // double curr_y = this->y;
        if(isnan(kinect_data.ranges[angleIdx])){
            curr_angle = curr_angle + kinect_data.angle_increment*20;
            angleIdx = angleIdx + 20;
            continue;
        }
        double curr_r = 0.0;
        // Starting pixel from slice
        int index = (int)
            (MAP_HEIGHT / 2 + this->y/MAP_RESOLUTION) * MAP_WIDTH
            + (int)(this->x/MAP_RESOLUTION + MAP_WIDTH/2); 
        while (MAP_DATA[index] != 100) {
            curr_r += inc;
            double new_x = this->x + curr_r * cos(curr_angle);
            double new_y = this->y + curr_r * sin(curr_angle);
            // printf("old x: %f, new x: %f\n", this->x, new_x);
            // printf("old y: %f, new y: %f\n", this->y, new_y);
            // printf("curr_r: %f, curr_angle: %f, cos(curr_angle): %f, sin(curr_angle): %f\n", curr_r, curr_angle, cos(curr_angle), sin(curr_angle));
            // curr_x += curr_r * cos(curr_angle);
            // curr_y += curr_r * sin(curr_angle);
            index = (int)
                (MAP_HEIGHT / 2 + new_y/MAP_RESOLUTION) * MAP_WIDTH
                + (int)(new_x/MAP_RESOLUTION + MAP_WIDTH/2); 
            // printf("curr_r: %f\n", curr_r);
            if(curr_r >= 6){
                break;
            }
        }
        cumulative_error = cumulative_error + fabs(kinect_data.ranges[angleIdx]-curr_r);
        curr_angle = curr_angle + kinect_data.angle_increment*20;
        angleIdx = angleIdx + 20;
    }
    if(*sonar_change == true){
        *sonar_change = false;
        double angles[] = {-PI/4, -PI/7.2, -PI/12, -PI/36, PI/36, PI/12, PI/7.2, PI/4};
        for (int i = 0; i < 8; i++) {
            if(isnan(sonar_data.ranges[i])){
                continue;
            }
            if (sonar_data.ranges[i] > 0.0) {
                double curr_r = 0.0;
                int index = (int)
                    (MAP_HEIGHT / 2 + this->y/MAP_RESOLUTION) * MAP_WIDTH
                    + (int)(this->x/MAP_RESOLUTION + MAP_WIDTH/2); 
                while (MAP_DATA[index] != 100) {
                    curr_r += inc;
                    double new_x = this->x + curr_r * cos(this->theta+angles[i]);
                    double new_y = this->y + curr_r * sin(this->theta+angles[i]);
                    index = (int)
                        (MAP_HEIGHT / 2 + new_y/MAP_RESOLUTION) * MAP_WIDTH
                        + (int)(new_x/MAP_RESOLUTION + MAP_WIDTH/2); 
                    if(curr_r >= 2.5){
                        break;
                    }
                }
                cumulative_error = cumulative_error + fabs(sonar_data.ranges[i]-curr_r);
            }
        }
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
    starting_locations[4].push_back(PI/2);

    starting_locations[5].push_back(8);
    starting_locations[5].push_back(-1.5);
    starting_locations[5].push_back(-PI/2);


    create_particles_around(starting_locations[0][0], starting_locations[0][1], starting_locations[0][2]);
    create_particles_around(starting_locations[1][0], starting_locations[1][1], starting_locations[1][2]);
    create_particles_around(starting_locations[2][0], starting_locations[2][1], starting_locations[2][2]);
    create_particles_around(starting_locations[3][0], starting_locations[3][1], starting_locations[3][2]);
    create_particles_around(starting_locations[4][0], starting_locations[4][1], starting_locations[4][2]);
    create_particles_around(starting_locations[5][0], starting_locations[5][1], starting_locations[5][2]);

    last_odom = NULL;
}

void Localizer::create_particles_around(double x, double y, double theta){
    srand(time(NULL));

    for(int i = 0; i < NUM_POINTS/6; i++){
        particles.push_back(new Particle(
            x + (rand() % (2 * (int)NUM_POINTS) - (int)(1 * NUM_POINTS)) / NUM_POINTS,
            y + (rand() % (2 * (int)NUM_POINTS) - (int)(1 * NUM_POINTS)) / NUM_POINTS,
            theta + PI/4 * ((rand() % (2 * (int)NUM_POINTS) - (int)(1 * NUM_POINTS)) / NUM_POINTS) 
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

/**
    updates the location of all the particles based on the change in odom from the last step to this one.

    then calls the weight update on the particles to update our confidence based on the most recent sensor readings, and resamples based on these new weights

    after that it determines a single "location" for the robot with a confidence based on the width of the distribution of all the points.

    if the weight is below a certain threshold, the point robot will decide that it is localized and try to plan and execute a path.
*/
Particle* Localizer::update_location(geometry_msgs::Pose pose_msg, sensor_msgs::LaserScan kinect_data, p2os_msgs::SonarArray sonar_data, bool *sonar_change) {
    // Record current odom in the case that this is the first iteration.
    if(
        this->last_odom == NULL
    ) {
        printf("are we getting here?\n");
        last_odom = new geometry_msgs::Pose();
        this->last_odom->position.x = pose_msg.position.x;
        this->last_odom->position.y = pose_msg.position.y;
        this->last_odom->orientation.w = pose_msg.orientation.w;
        this->last_odom->orientation.z = pose_msg.orientation.z;
        Particle *test = new Particle(0, 0, 0);
        test->weight = 0.01;
        return test;
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
        particles[i]->update_weight(kinect_data, sonar_data, MAP_DATA, MAP_WIDTH, MAP_HEIGHT, MAP_RESOLUTION, sonar_change);
    }

    resample();

    Particle* pTest = determine_location(kinect_data, sonar_data, sonar_change);

    // Set last odom for use in next iteration
    this->last_odom->position.x = pose_msg.position.x;
    this->last_odom->position.y = pose_msg.position.y;
    this->last_odom->orientation.w = pose_msg.orientation.w;
    this->last_odom->orientation.z = pose_msg.orientation.z;

    return pTest;
}

/**
    averages the current particles and creates a single "location" particle for path planning and execution. The particle weight is
    based on the probability that the particle is actually there based on the variance in the distribution.
*/
Particle* Localizer::determine_location(sensor_msgs::LaserScan kinect_data, p2os_msgs::SonarArray sonar_data, bool *sonar_change){
    double average_x = 0;
    double average_y = 0;
    double average_theta = 0;

    for(int i = 0; i < (int)particles.size(); i++){
        average_x += particles[i]->x;
        average_y += particles[i]->y;
        average_theta += particles[i]->theta;
    }

    if(particles.size() > 0){
        average_x /= particles.size();
        average_y /= particles.size();
        average_theta /= particles.size();
    }

    Particle* result = new Particle(average_x, average_y, average_theta);
    result->update_weight(kinect_data, sonar_data, MAP_DATA, MAP_WIDTH, MAP_HEIGHT, MAP_RESOLUTION, sonar_change);

    return result;
}

/**
    randomly redistributes new particles based on the weights of the old ones
*/
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
        //TODO: REGENERATE ON THE WHOLE MAP IF NEED BE (we didn't have time, but this is where we would be resampling on the whole map if localization is lost)
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
                    particles[j]->x + (rand() % (2 * (int)NUM_POINTS) - (int)(1 * NUM_POINTS)) / (NUM_POINTS*10),
                    particles[j]->y + (rand() % (2 * (int)NUM_POINTS) - (int)(1 * NUM_POINTS)) / (NUM_POINTS*10),
                    particles[j]->theta + PI/4 * ((rand() % (2 * (int)NUM_POINTS) - (int)(1 * NUM_POINTS)) / (NUM_POINTS*10)) 
                );
                new_particles.push_back(p);
                break;
            }
        }
    }
    particles = new_particles;
}
