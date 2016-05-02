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
    // this->x += delta_x;
    // this->y += delta_y;


    this->theta += delta_theta;
    this->x += distance * cos(theta);
    this->y += distance * sin(theta);
    // printf("updating: %f\n", this->x);
}

Localizer::Localizer(int8_t *MAP_DATA, int32_t MAP_WIDTH, int32_t MAP_HEIGHT, double MAP_RESOLUTION) {
    this->MAP_DATA = MAP_DATA;
    this->MAP_WIDTH = MAP_WIDTH;
    this->MAP_HEIGHT = MAP_HEIGHT;
    this->MAP_RESOLUTION = MAP_RESOLUTION;

    // //generate points
    // particles.push_back(new Particle(8.0, -1.0, PI/2));

    // starting_locations = new float([6][3];
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

    printf("particle: %f\n", starting_locations[0][0]);

    create_particles_around(starting_locations[0][0], starting_locations[0][1], starting_locations[0][2]);
    create_particles_around(starting_locations[1][0], starting_locations[1][1], starting_locations[1][2]);
    create_particles_around(starting_locations[2][0], starting_locations[2][1], starting_locations[2][2]);
    create_particles_around(starting_locations[3][0], starting_locations[3][1], starting_locations[3][2]);
    create_particles_around(starting_locations[4][0], starting_locations[4][1], starting_locations[4][2]);
    create_particles_around(starting_locations[5][0], starting_locations[5][1], starting_locations[5][2]);

    // particles.push_back(new Particle(starting_locations[0][0], starting_locations[0][1], starting_locations[0][2]));
    // particles.push_back(new Particle(starting_locations[1][0], starting_locations[1][1], starting_locations[1][2]));
    // particles.push_back(new Particle(starting_locations[2][0], starting_locations[2][1], starting_locations[2][2]));
    // particles.push_back(new Particle(starting_locations[3][0], starting_locations[3][1], starting_locations[3][2]));
    // particles.push_back(new Particle(starting_locations[4][0], starting_locations[4][1], starting_locations[4][2]));
    // particles.push_back(new Particle(starting_locations[5][0], starting_locations[5][1], starting_locations[5][2]));

    // printf("%f", last_odom->pose.pose.position.x);
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
    printf("getting poses\n");
    for(int i = 0; i < particles.size(); i++){
        geometry_msgs::Pose tmp;
        tmp.position.x = particles[i]->x;
        tmp.position.y = particles[i]->y;
        tmp.orientation = tf::createQuaternionMsgFromYaw(particles[i]->theta);
        particle_poses.poses.push_back(tmp);
    }
    printf("done getting poses\n");
    return particle_poses;
}

void Localizer::update_location(nav_msgs::Odometry odom_msg) {
    //calculate deltas
    //call update on every particle

    if(
        this->last_odom == NULL
    ) {
        last_odom = new nav_msgs::Odometry();
        // this->last_odom = &odom_msg;
        this->last_odom->pose.pose.position.x = odom_msg.pose.pose.position.x;
        this->last_odom->pose.pose.position.y = odom_msg.pose.pose.position.y;
        

        this->last_odom->pose.pose.orientation.w = odom_msg.pose.pose.orientation.w;
        this->last_odom->pose.pose.orientation.z = odom_msg.pose.pose.orientation.z;
        return;
    }

    double delta_x = odom_msg.pose.pose.position.x - this->last_odom->pose.pose.position.x;
    double delta_y = odom_msg.pose.pose.position.y - this->last_odom->pose.pose.position.y;



    double cur_angle = atan2(2 * (odom_msg.pose.pose.orientation.w * odom_msg.pose.pose.orientation.z), odom_msg.pose.pose.orientation.w*odom_msg.pose.pose.orientation.w - odom_msg.pose.pose.orientation.z*odom_msg.pose.pose.orientation.z);
    double last_angle = atan2(2 * (last_odom->pose.pose.orientation.w * last_odom->pose.pose.orientation.z), last_odom->pose.pose.orientation.w*last_odom->pose.pose.orientation.w - last_odom->pose.pose.orientation.z*last_odom->pose.pose.orientation.z);
    double delta_theta = cur_angle - last_angle;

    for(int i = 0; i < particles.size(); i++){
        // printf("updating");
        particles[i]->update_location(delta_x, delta_y, delta_theta);
    }

    resample();
    printf("new particles size after: %d\n", (int)particles.size());

    this->last_odom->pose.pose.position.x = odom_msg.pose.pose.position.x;
    this->last_odom->pose.pose.position.y = odom_msg.pose.pose.position.y;
    

    this->last_odom->pose.pose.orientation.w = odom_msg.pose.pose.orientation.w;
    this->last_odom->pose.pose.orientation.z = odom_msg.pose.pose.orientation.z;
    printf("done updating\n");
}

void Localizer::resample(){
    std::vector<double> alphas;
    std::vector<Particle*> new_particles;

    srand(time(NULL));

    printf("1\n");
    double weight_sum = 0;
    for(int i = 0; i < particles.size(); i++){
        // printf("1.1\n");
        int index = (int)
            (MAP_HEIGHT / 2 + particles[i]->y/MAP_RESOLUTION) * MAP_WIDTH
            + (int)(particles[i]->x/MAP_RESOLUTION + MAP_WIDTH/2);
        // printf("%d\n", index);
        if(
            index < 0
            || index > MAP_WIDTH * MAP_HEIGHT
        ) {
            particles[i]->weight = 0;
            continue;
        }
        int pixel_val = MAP_DATA[index];
        
        // printf("x: %f, y: %f, 'x': %d, 'y': %d, index: %d, val: %d, size:%d, weightsum:%f\n", particles[i]->x, particles[i]->y, (int)(particles[i]->x/MAP_RESOLUTION + MAP_WIDTH/2), (int)((MAP_HEIGHT / 2 - particles[i]->y/MAP_RESOLUTION)), index, pixel_val, (int)particles.size(), weight_sum);
        // printf("1.2\n");
        if(
            pixel_val == 100
        ) {
            particles[i]->weight = 0;
        }
        // printf("1.3\n");
        alphas.push_back(particles[i]->weight);
        weight_sum += particles[i]->weight;
    }

    printf("2\n");

    for(int i = 0; i < alphas.size(); i++){
        //TODO: REGENERATE ON THE WHOLE MAP
        if(weight_sum != 0){
            alphas[i] = alphas[i] / weight_sum;
        }
        else{
            alphas[i] = 0;
        }
    }
    printf("q\n");

    for(int i = 0; i < NUM_POINTS; i++){
        // double sampled_alpha =  (rand() % (2 * (int)NUM_POINTS) - 1 * NUM_POINTS) / NUM_POINTS;
        double sampled_alpha = (rand() % (1 * (int)NUM_POINTS)) / NUM_POINTS;
        // printf("sampled: %f\n", sampled_alpha);
        // int count = 0;
        // for(double j = 0; j < 1; j+= alphas[count]){
        //     // printf("%f")
        //     if(
        //         j >= sampled_alpha
        //     ) {
        //         new_particles.push_back(new Particle(particles[count]->x, particles[count]->y, particles[count]->theta));
        //         break;
        //     }

        //     count++;
        // }
        double alpha_sum = 0;
        for(int j = 0; j < alphas.size(); j++){
            alpha_sum += alphas[j];
            if(
                alpha_sum >= sampled_alpha
            ) {
                if(j > 0){
                    // printf("weight: %f, alpha: %f, alpha_sum: %f, sampled_alpha: %f, last weight: %f, last alpha: %f\n", particles[j]->weight, alphas[j], alpha_sum, sampled_alpha, particles[j-1]->weight, alphas[j-1]);
                }
                // new_particles.push_back(new Particle(particles[j]->x, particles[j]->y, particles[j]->theta));

                new_particles.push_back(new Particle(
                    particles[j]->x + (rand() % (2 * (int)NUM_POINTS) - 1 * NUM_POINTS) / (NUM_POINTS*100),
                    particles[j]->y + (rand() % (2 * (int)NUM_POINTS) - 1 * NUM_POINTS) / (NUM_POINTS*100),
                    particles[j]->theta + PI/4 * ((rand() % (2 * (int)NUM_POINTS) - 1 * NUM_POINTS) / (NUM_POINTS*100)) 
                ));
                break;
            }
        }
    }

    printf("current particles size: %d, new particles size: %d\n", (int)particles.size(), (int)new_particles.size());

    printf("3\n");

    particles = new_particles;
    printf("4?\n");
}
