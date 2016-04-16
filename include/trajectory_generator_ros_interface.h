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


#ifndef TRAJECTORY_GENERATOR_ROS_INTERFACE_H
#define TRAJECTORY_GENERATOR_ROS_INTERFACE_H

typedef std::shared_ptr<traj_params> traj_params_ptr;
typedef std::shared_ptr<traj_func> traj_func_ptr;


struct ni_trajectory
{

    std::vector<state_type> x_vec;
    std::vector<double> times;
    state_type x0_;   //This will be the same for a number of trajectories; may want to replace with shared_ptr
    std_msgs::Header header;
    traj_func_ptr trajpntr;
    traj_params_ptr params;
    
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

typedef std::shared_ptr<ni_trajectory> ni_trajectory_ptr;

class TrajectoryGeneratorBridge
{

    traj_generator trajectory_gen;
    
    double robot_radius_;
    std::string name_="TrajectoryGeneratorBridge";

public:

TrajectoryGeneratorBridge();

void updateParams();
traj_params getDefaultParams();
void setDefaultParams(traj_params_ptr& new_params);

void generate_trajectory(ni_trajectory_ptr trajectory);

inline
state_type initState()
{
    state_type x0(8);
    TrajectoryGeneratorBridge::initState(x0);
    return x0;
}



inline
void initState(state_type& x0)
{
    x0[near_identity::X_IND] = 0;      //x
    x0[near_identity::Y_IND] = 0;      //y
    x0[near_identity::THETA_IND] = 0;  //theta
    x0[near_identity::V_IND] = 0;      //v
    x0[near_identity::W_IND] = 0;      //w
    x0[near_identity::LAMBDA_IND] = robot_radius_;    //lambda: must be > 0!
    x0[near_identity::XD_IND] = 0;    //x_d
    x0[near_identity::YD_IND] = 0;    //y_d
}

inline
void initState(state_type& x0, const nav_msgs::OdometryPtr& curr_odom)
{
    double vx = curr_odom->twist.twist.linear.x;
    double vy = curr_odom->twist.twist.linear.y;
    double v = std::sqrt((vx*vx) + (vy*vy)); 
    double w = curr_odom->twist.twist.angular.z;

    x0[near_identity::V_IND] = v;      //v
    x0[near_identity::W_IND] = w;      //w
}

inline
void initState(ni_trajectory_ptr& traj, const nav_msgs::OdometryPtr& curr_odom)
{
    initState(traj->x0_, curr_odom);
    
}


template<typename T>
state_type initState(T& source)
{
    state_type x0 = TrajectoryGeneratorBridge::initState();
    initState(x0, source);
    return x0;
}

template<const nav_msgs::OdometryPtr&> state_type initState(const nav_msgs::OdometryPtr& curr_odom);

inline
static geometry_msgs::Quaternion yawToQuaternion(double yaw);

inline
static double quaternionToYaw(geometry_msgs::Quaternion& quaternion);

inline
static const nav_msgs::OdometryPtr OdomFromState(state_type& state, double t, std_msgs::Header& header);

static ni_trajectory_ptr getLongestTrajectory(std::vector<ni_trajectory_ptr>& valid_trajs);

void publishPaths(ros::Publisher& pub, std::vector<ni_trajectory_ptr>& trajs, size_t num_total_paths);


};

#endif

