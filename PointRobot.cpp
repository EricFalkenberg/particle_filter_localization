#include "PointRobot.h"

/**
    Constructor
    @param fname The filename containing all destination points
    @param VARIANCE The amount of error we are willing to allow
*/
PointRobot::PointRobot(char* fname, double SPEED, double VARIANCE) {
    std::string map_name = "src/project/src/map.info";
    this->MAP_WIDTH = 2000;
    this->MAP_HEIGHT = 700;
    this->MAP_DATA = new int8_t[this->MAP_WIDTH*this->MAP_HEIGHT];
    this->read_image(map_name.c_str());
    this->destinations = this->read_file(fname);
    this->VARIANCE = VARIANCE;
    this->ANGULAR_VELOCITY = 0.0;
    this->DEFAULT_SPEED = SPEED;
    *(this->localizer) = Localizer();
}

void PointRobot::whereAmI() {
    // DO NOTHING FOR NOW
}

void PointRobot::updateMap() {
    // DO NOTHING FOR NOW
}

void PointRobot::sonarCallback(const p2os_msgs::SonarArray msgs) {
    boost::timer t = boost::timer();
    double x_pos = pose.position.x;
    double y_pos = pose.position.y;
    double position_theta = 2*atan2(pose.orientation.z, pose.orientation.w);
    double angles[] = {-PI/4, -PI/7.2, -PI/12, -PI/36, PI/36, PI/12, PI/7.2, PI/4};
    for (int i = 0; i < 9; i++) {
        if (
            msgs.ranges[i] > 0.0
        ) {
            double x = x_pos + (msgs.ranges[i])*cos(position_theta+angles[i]);
            double y = y_pos + (msgs.ranges[i])*sin(position_theta+angles[i]);
        }
    }
}   

void PointRobot::kinectCallback(const sensor_msgs::LaserScan msgs) {
    boost::timer t = boost::timer();
    double x_pos = pose.position.x;
    double y_pos = pose.position.y;
    double position_theta = 2*atan2(pose.orientation.z, pose.orientation.w);
    double angle_increment = msgs.angle_increment;
    double angle = position_theta + msgs.angle_min;
    double max_angle = position_theta + msgs.angle_max;
    int midpoint = (fabs(msgs.angle_min - msgs.angle_max) / angle_increment) / 2;
    int i = 45;
    angle = angle + angle_increment*45;
    while (angle < max_angle && i < 638-45) {
        if (
            ! isnan(msgs.ranges[i])
        ) {
            double x = x_pos + (msgs.ranges[i])*cos(angle);
            double y = y_pos + (msgs.ranges[i])*sin(angle);
        }
        angle = angle + 1*angle_increment;
        i += 1;
    }
}

/**
    The callback function responsible for handling any information
    retrieved from /r1/odom
    @param msgs The message received from /r1/odom
*/
void PointRobot::odomCallback(const nav_msgs::Odometry msgs) {
    this->pose  = msgs.pose.pose;
    this->twist = msgs.twist.twist;
    float position_theta = 2*atan2(pose.orientation.z, pose.orientation.w);
    float x_pos = pose.position.x;
    float y_pos = pose.position.y;
}
    
/**
    When called, this function will determine the angular velocity that
    the PointRobot instance should take on considering the destination location
    as well as the current position and heading of the instance itself.
    @return The new angular velocity value
*/
double PointRobot::getAngularVelocity() {
    double destination_theta;
    double delta_theta;
    double x_pos            = this->pose.position.x;
    double y_pos            = this->pose.position.y;
    double dest_x           = this->destinations.front().x;
    double dest_y           = this->destinations.front().y;
    double position_theta   = 2*atan2(pose.orientation.z, pose.orientation.w);
    double delta_x          = dest_x-x_pos;
    double delta_y          = dest_y-y_pos;

    if (
        delta_x == 0.0 
        || delta_y == 0.0
    ) {
        // If the robot is inline with the destination,
        // the destination theta is 0.
        destination_theta = 0.0;
    }
    else if (
        delta_x > 0.0 
        && delta_y > 0.0
    ) {
        // First Quadrant.
        destination_theta = atan(delta_y/delta_x);
    }
    else if (
        delta_x < 0.0
        && delta_y > 0.0
    ) {
        // Second Quadrant.
        destination_theta = PI + atan(delta_y/delta_x);
    }
    else if (
        delta_x < 0.0
        && delta_y < 0.0
    ) {
        // Third Quadrant.
        destination_theta = PI + atan(delta_y/delta_x);
    }
    else {
        // Fourth Quadrant.
        destination_theta = 2*PI + atan(delta_y/delta_x);
    }

    // Detect whether or not position_theta is negative and correct it.
    if (
        position_theta < 0.0
    ) { 
        position_theta  = 2*PI + position_theta; 
    }

    // Calculate the angle we still need to turn.
    delta_theta = destination_theta - position_theta;
    if (
        fabs(delta_theta) > PI
    ) {
        delta_theta *= -1;
    }
    // Alter the robots angular velocity based on the above calculated info.
    if (
        delta_theta >= -2*VARIANCE
        && delta_theta <= 2*VARIANCE
    ) {
        ANGULAR_VELOCITY = 0.0;
    }
    else if (
        ANGULAR_VELOCITY == 0.0
    ) {
        if (
            delta_theta <= 0
        ) {
            ANGULAR_VELOCITY = -1*DEFAULT_SPEED;
        }
        else if (
            delta_theta > 0
        ) {
            ANGULAR_VELOCITY = 1*DEFAULT_SPEED;
        }
    }

    // Return
    return ANGULAR_VELOCITY;
}

/**
    When called, this function is responsible for computing the forward velocity
    of the PointRobot based on its proximity to the active destination point.
    @return The forward velocity of the PointRobot
*/
double PointRobot::getForwardVelocity() {
    double x_pos        = pose.position.x;
    double y_pos        = pose.position.y;
    double dest_x       = this->destinations.front().x;
    double dest_y       = this->destinations.front().y;
    double EXT_VARIANCE = VARIANCE*5;    

    if (
        (
            x_pos >= dest_x-VARIANCE
            && x_pos <= dest_x+VARIANCE
        )
        && (
            y_pos >= dest_y-VARIANCE 
            && y_pos <= dest_y+VARIANCE
        )
    ) {
        // If the x_pos and y_pos values are in range of the destination point
        // allowing for the specified error, we can stop the PointRobot and pop
        // the active destination off of the destinations queue.           
        printf("DESTINATION REACHED: (%.2f, %.2f)\n", dest_x, dest_y);
        destinations.pop();
        return 0.0;
    }
    else if (
        (
            x_pos >= dest_x-EXT_VARIANCE
            && x_pos <= dest_x+EXT_VARIANCE
        )
        && (
            y_pos >= dest_y-EXT_VARIANCE
            && y_pos <= dest_y+EXT_VARIANCE
        )
    ) {
        // If the x_pos and y_pos values are not in range of the destination point
        // allowing for specified error, but are at close proximity to that location,
        // slow the speed of the PointRobot in order to prepare to stop.
        return DEFAULT_SPEED/2;
    }
    else {
        // Otherwise, full steam ahead.
        return DEFAULT_SPEED;
    }
}
    
/**
    The main ROS loop responsible for combining all the above functions
    in order to visit all the destination points specified by the input file.
    @param argc The program argc input
    @param argv The program argv input
*/
int PointRobot::run(int argc, char** argv, bool run_kinect, bool run_sonar) {
    // Initialization
    ros::init(argc, argv, "motion");
    ros::NodeHandle n;

    // Publish a latched value of 1 to the Motor State
    ros::Publisher motor_state = n.advertise<p2os_msgs::MotorState>("cmd_motor_state", 1000, true);
    p2os_msgs::MotorState state;
    state.state = 1;
    motor_state.publish(state);

    // Publish a latched occupancy grid / map to RVIZ
    ros::Publisher static_map = n.advertise<nav_msgs::OccupancyGrid>("static_map", 1000, true);
    nav_msgs::OccupancyGrid grid;
    std::vector<signed char> data(this->MAP_DATA, this->MAP_DATA+(this->MAP_WIDTH*this->MAP_HEIGHT));
    grid.data            = data;
    grid.info.resolution = 0.0633;
    grid.info.width      = this->MAP_WIDTH;
    grid.info.height     = this->MAP_HEIGHT;
    grid.info.origin.position.x = 0.0-grid.info.resolution*((double)this->MAP_WIDTH)/2;
    grid.info.origin.position.y = 0.0-grid.info.resolution*((double)this->MAP_HEIGHT)/2;
    static_map.publish(grid);

    // Test the pose estimation pusblisher with RVIZ, the generation of points will eventually
    // be offloaded to Loaclizer.
    ros::Publisher pointCloudTest = n.advertise<geometry_msgs::PoseArray>("point_cloud", 1000, true);
    geometry_msgs::PoseArray arr;
    arr.header.frame_id = "/map";
    geometry_msgs::Pose tmp;
    tmp.position.x = 8.0;
    tmp.position.y = -0.5;
    tmp.orientation = tf::createQuaternionMsgFromYaw(PI/2);
    arr.poses.push_back(tmp);
    pointCloudTest.publish(arr);

    // Create a odom subscriber so that the robot can tell where it is
    ros::Subscriber vel = n.subscribe<nav_msgs::Odometry>("pose", 1000, &PointRobot::odomCallback, this);
    ros::Subscriber kinect;
    ros::Subscriber sonar;
    if (
        run_kinect
    ) {
        kinect = n.subscribe<sensor_msgs::LaserScan>("scan", 50, &PointRobot::kinectCallback, this);
    }
    if (
        run_sonar
    ) {
        sonar  = n.subscribe<p2os_msgs::SonarArray>("sonar", 50, &PointRobot::sonarCallback, this);        
    }
    // Create the motion publisher and set the loop rate
    ros::Publisher motion = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Rate loop_rate(10);

    // Main event loop

    while (ros::ok()) 
    {
        geometry_msgs::Twist msg;
        // Allow the subscriber callbacks to fire
        ros::spinOnce();

        // If our destiinations queue is empty, try to find another location to visit
        // If we have visited all destinations, we can exit
        if (
            destinations.empty()
        ) {
            motion.publish(msg);
            break;
        }

        // Get the angular velocity of the PointRobot
        msg.angular.z = this->getAngularVelocity();        
        // If we are not currently turning, calculate the forward velocity
        // of the PointRobot
        if (
            msg.angular.z == 0.0
        ) {
            msg.linear.x = this->getForwardVelocity();
        }
        // Publish our velocities to /r1/cmd_vel
        motion.publish(msg);

        // Sleep until next required action
        loop_rate.sleep();
    }
    return 0;
}

/**
    The function responsible for reading the input file into memory
    and translating it into a queue of destination points.
    @param file The name of the file to be read
*/
std::queue<dest> PointRobot::read_file(char *file) {
    std::queue<dest> destinations; 
    std::ifstream read(file);
    std::string s;
    do {
        getline(read, s);
        if (
            read.eof()
        ) {

            break;
        }
        dest d;
        std::sscanf(s.c_str(), "%lf %lf", &d.x, &d.y);
        destinations.push(d);
    } while (!read.eof());
    return destinations;
}

void PointRobot::read_image(const char *file) {
    std::ifstream read(file);
    std::string s;
    int p_val;
    int mapIdx = this->MAP_WIDTH*this->MAP_HEIGHT - this->MAP_WIDTH;
    do {
        getline(read, s);
        if (
            read.eof()
        ) {

            break;
        }
        p_val = atoi(s.c_str());
        if (p_val != 0) {
            this->MAP_DATA[mapIdx] = 0;
        }
        else {
            this->MAP_DATA[mapIdx] = 100;
        }
        mapIdx++;
        if (mapIdx % this->MAP_WIDTH == 0) {
            mapIdx -= 2*this->MAP_WIDTH;
        }
    } while (!read.eof());
}

/**
    usage error message
*/
void usage() {
    printf("rosrun hw4 mapper <input_file> [sonar] [kinect]\n");
}

/**
    main
*/
int main(int argc, char **argv) {
    bool run_kinect = false;
    bool run_sonar = false;
    if (
        argc < 3
    ) {
        usage();
        return 0;
    }
    for (int i = 2; i < argc; i++) {
        std::string arg(argv[i]);
        if (
            arg.compare("sonar") == 0
        ) {
            run_sonar = true;
        }
        else if (
            arg.compare("kinect") == 0
        ) {
            run_kinect = true;
        }
        else {
            printf("INVALID ARGUMENT: %s\n", argv[i]);
            usage();
        }
    }
    PointRobot robot (argv[1], 0.3, 0.1);
    robot.run(argc, argv, run_kinect, run_sonar);
    ros::shutdown();
}
