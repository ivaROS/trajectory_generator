#include "traj_generator.h"
#include <chrono>
#include <trajectory_generator/trajectory_point.h>
#include <trajectory_generator/trajectory_points.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
//#include "tf2_trajectory.h"

#ifndef TRAJECTORY_GENERATOR_ROS_INTERFACE_H
#define TRAJECTORY_GENERATOR_ROS_INTERFACE_H



struct ni_trajectory
{

    std::vector<state_type> x_vec;
    std::vector<double> times;
    state_type x0_;
    std::string frame_id = "";  //Note sure whether to include Frame at this level
    std_msgs::Header header;
    traj_func* trajpntr;
    traj_params* params;
    
    ni_trajectory()
    {
    
    }

    ni_trajectory( std::vector< state_type > states , std::vector< double > t ) : x_vec( states ) , times( t ) { }

    trajectory_generator::trajectory_points toTrajectoryMsg ();
    trajectory_generator::trajectory_pointsPtr toTrajectoryMsgPtr ();
    std::vector<trajectory_generator::trajectory_point> toTrajectoryPointMsgs();
    nav_msgs::PathPtr toPathMsg();
    void print();
    virtual size_t num_states();
    geometry_msgs::Point getPoint(int i);
    geometry_msgs::PointStamped getPointStamped(int i);

};

class TrajectoryGeneratorBridge
{

    traj_generator trajectory_gen;
    
    double robot_radius_;
    

public:

TrajectoryGeneratorBridge();

void updateParams();
traj_params getDefaultParams();
void setDefaultParams(traj_params &new_params);

ni_trajectory* generate_trajectory(traj_func* trajpntr);

ni_trajectory* generate_trajectory(traj_func* trajpntr, const nav_msgs::OdometryPtr& curr_odom);
//ni_trajectory generate_trajectory(const geometry_msgs::TransformStampedPtr curr_tf, traj_func* trajpntr);
ni_trajectory* generate_trajectory(traj_func* trajpntr, geometry_msgs::TransformStamped& curr_tf);

ni_trajectory* run(traj_func* trajpntr, state_type& x0);

void generate_trajectory(ni_trajectory* trajectory);

inline
state_type initState();

inline
void initState(state_type& x0, const nav_msgs::OdometryPtr& curr_odom);

inline
void initState(state_type& x0);

inline
void initState(ni_trajectory* traj, const nav_msgs::OdometryPtr& curr_odom);


inline
static geometry_msgs::Quaternion yawToQuaternion(double yaw);

inline
static double quaternionToYaw(geometry_msgs::Quaternion& quaternion);

const nav_msgs::OdometryPtr OdomFromState(state_type& state, double t, std_msgs::Header header);

void publishPaths(ros::Publisher& pub, std::vector<ni_trajectory>& trajs, size_t num_total_paths);


};

#endif

