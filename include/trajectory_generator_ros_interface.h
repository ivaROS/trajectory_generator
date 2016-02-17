#include "traj_generator.h"
#include <chrono>
#include <trajectory_generator/trajectory_point.h>
#include <trajectory_generator/trajectory_points.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#ifndef TRAJECTORY_GENERATOR_ROS_INTERFACE_H
#define TRAJECTORY_GENERATOR_ROS_INTERFACE_H



struct ni_trajectory
{

    std::vector<state_type> x_vec;
    std::vector<double> times;
    std::string frame_id;  //Note sure whether to include Frame at this level


    ni_trajectory( std::vector< state_type > states , std::vector< double > t ) : x_vec( states ) , times( t ) { }

    trajectory_generator::trajectory_points toTrajectoryMsg ();
    std::vector<trajectory_generator::trajectory_point> toTrajectoryPointMsgs();
    nav_msgs::Path toPathMsg();
    void printTrajectory();
    size_t num_states();
    geometry_msgs::Vector3 getPoint(int i);

};

class TrajectoryGeneratorBridge
{

    traj_generator trajectory_gen;
    
    double robot_radius_;
    

public:

TrajectoryGeneratorBridge();

void updateParams();

ni_trajectory generate_trajectory(const nav_msgs::OdometryPtr curr_odom, traj_func* trajpntr);
//ni_trajectory generate_trajectory(const geometry_msgs::TransformStampedPtr curr_tf, traj_func* trajpntr);
ni_trajectory generate_trajectory(geometry_msgs::TransformStamped& curr_tf, traj_func* trajpntr);

ni_trajectory run(traj_func* trajpntr, state_type& x0);

void initFromOdom(const nav_msgs::OdometryPtr curr_odom, state_type& x0);
void initFromTF( geometry_msgs::TransformStamped& curr_tf, state_type& x0);

geometry_msgs::Quaternion yawToQuaternion(double yaw);
double quaternionToYaw(geometry_msgs::Quaternion& quaternion);

const nav_msgs::OdometryPtr OdomFromState(state_type& state, double t, std_msgs::Header header);

};

#endif

