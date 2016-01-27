#include "traj_generator.h"
#include <chrono>
#include <turtlebot_trajectory_controller/trajectory_point.h>
#include <turtlebot_trajectory_controller/trajectory_points.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>


class TrajectoryGeneratorBridge
{

    traj_generator trajectory_gen;
    
    double robot_radius_;
    

public:

TrajectoryGeneratorBridge();

void generate_trajectory(const nav_msgs::Odometry& curr_odom, traj_func* trajpntr);

void initFromOdom(const nav_msgs::Odometry& curr_odom, state_type& x0);





std::vector<turtlebot_trajectory_controller::trajectory_point> TrajectoryFromStates(std::vector<state_type> x_vec, std::vector<double> times);



const nav_msgs::Odometry OdomFromState(state_type& state, double t, std_msgs::Header header);

};

