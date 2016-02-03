#include "traj_generator.h"
#include <chrono>
#include <trajectory_generator/trajectory_point.h>
#include <trajectory_generator/trajectory_points.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#ifndef TRAJECTORY_GENERATOR_ROS_INTERFACE_H
#define TRAJECTORY_GENERATOR_ROS_INTERFACE_H

struct ni_trajectory
{
    std::vector<state_type>& x_vec;
    std::vector<double>& times;

    ni_trajectory( std::vector< state_type > &states , std::vector< double > &t ) : x_vec( states ) , times( t ) { }

    std::vector<trajectory_generator::trajectory_point> toTrajectoryMsg ();
};

class TrajectoryGeneratorBridge
{

    traj_generator trajectory_gen;
    
    double robot_radius_;
    

public:

TrajectoryGeneratorBridge();

std::vector<trajectory_generator::trajectory_point> generate_trajectory(const nav_msgs::OdometryPtr curr_odom, traj_func* trajpntr);

void initFromOdom(const nav_msgs::OdometryPtr curr_odom, state_type& x0);




const nav_msgs::OdometryPtr OdomFromState(state_type& state, double t, std_msgs::Header header);

};

#endif

