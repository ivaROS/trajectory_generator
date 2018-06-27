#include "traj_generator.h"
#include <chrono>
#include <pips_trajectory_msgs/trajectory_point.h>
#include <pips_trajectory_msgs/trajectory_points.h>
#include <pips_msgs/PathArray.h>
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

template <typename state_type>
struct trajectory_states
{
    using msg_state_type = typename state_type::msg_state_type;

    std::vector<state_type> x_vec;
    std::vector<double> times;
    state_type x0_;   //This will be the same for a number of trajectories; may want to replace with shared_ptr. On the other hand, it is a small structure, and could be passed by registers, which would be faster than  resolving the reference...
    std_msgs::Header header;
    traj_func_ptr trajpntr;
    traj_params_ptr params;
    
    trajectory_states()
    {
    
    }

    trajectory_states( std::vector< state_type > states , std::vector< double > t ) : x_vec( states ) , times( t ) { }

    
    std::vector<msg_state_type> toTrajectoryPointMsgs()
    {
      std::vector<msg_state_type> trajectory;
      for(size_t i = 0; i < this->num_states(); i++)
      {
        pips_trajectory_msgs::trajectory_point point;
        point.time = ros::Duration(times[i]);
        point.x = x_vec[i][near_identity::X_IND];
        point.y = x_vec[i][near_identity::Y_IND];
        point.theta = x_vec[i][near_identity::THETA_IND];
        point.v = x_vec[i][near_identity::V_IND];
        point.w = x_vec[i][near_identity::W_IND];
        trajectory.push_back(point);
      }
      return trajectory;
    }
    
    
    pips_trajectory_msgs::trajectory_points toTrajectoryMsg ()
    {
      //ni_trajectory::printTrajectory();
      std::vector<pips_trajectory_msgs::trajectory_point> points = ni_trajectory::toTrajectoryPointMsgs();
      
      pips_trajectory_msgs::trajectory_points trajectory_msg;
      trajectory_msg.points = points;
      trajectory_msg.header = header;
      
      return trajectory_msg;
    }
    
    //Not sure if this works; if it does, then the transforming function must not be
    pips_trajectory_msgs::trajectory_pointsPtr toTrajectoryMsgPtr ()
    {
      pips_trajectory_msgs::trajectory_pointsPtr msgPtr(new pips_trajectory_msgs::trajectory_points);
      msgPtr->points = ni_trajectory::toTrajectoryPointMsgs();
      msgPtr->header = header;
      
      return msgPtr;
    }
    
//     //TODO: change this to a toString() method for more flexibility
//     void trajectory_states::print()
//     {
//       std::cout << "Time" << '\t' << "Error" << '\t' << 'x' << '\t' << 'y' << '\t' << "theta" << '\t' << 'v' << '\t' << 'w' << '\t' << "lambda" << '\t' << "xd" << '\t' << "yd" << std::endl;
//       
//       for( size_t i=0; i < this->num_states(); i++ )
//       {
//         double error_x = x_vec[i][near_identity::X_IND] - x_vec[i][near_identity::XD_IND];
//         double error_y = x_vec[i][near_identity::Y_IND] - x_vec[i][near_identity::YD_IND];
//         
//         double error = sqrt(error_x*error_x + error_y*error_y);
//         printf("%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n", times[i], error, x_vec[i][0], x_vec[i][1], x_vec[i][2], x_vec[i][3], x_vec[i][4], x_vec[i][5], x_vec[i][6], x_vec[i][7]);
//       }
//     }
    
    
    nav_msgs::PathPtr toPathMsg()
    {
      nav_msgs::PathPtr path_msg(new nav_msgs::Path);
      path_msg->header = header;
      
      for(size_t i=0; i < this->num_states(); i++)
      {
        geometry_msgs::PoseStamped pose = getPoseStamped(i);
        
        path_msg->poses.push_back(pose);
      }
      
      return path_msg;
    }
    
    nav_msgs::PathPtr getDesiredPathMsg()
    {
      nav_msgs::PathPtr path_msg(new nav_msgs::Path);
      path_msg->header = header;
      
      for(size_t i=0; i < this->num_states(); i++)  // The desired path shouldn't be cropped by collision... yet graphically that looks terrible
      {
        geometry_msgs::PoseStamped pose = getPoseStamped(i);
        path_msg->poses.push_back(pose);
      }
      
      return path_msg;
    }
    
    size_t trajectory_states::num_states()
    {
      return x_vec.size();
    }
    void trajectory_states::setCollisionInd ( int i )
    {
      x_vec.resize(i);
    }
    
    geometry_msgs::Point getPoint(int i)
    {
      state_type state = x_vec[i];
      geometry_msgs::Point point;
      point.x = state[near_identity::X_IND];
      point.y = state[near_identity::Y_IND];
      return point;
    }
    
    geometry_msgs::PointStamped getPointStamped(int i)
    {
      geometry_msgs::PointStamped point;
      point.point = getPoint(i);
      point.header = getHeader(i);
      
      return point;
    }
    
    geometry_msgs::Quaternion getQuaternion(int i)
    {
      double theta = x_vec[i][near_identity::THETA_IND];
      geometry_msgs::Quaternion quat = TrajectoryGeneratorBridge::yawToQuaternion(theta);//what I had here before seemed to work, but I would prefer to encapsulate all my conversions
      return quat;
    }
    
    geometry_msgs::Pose getPose(int i)
    {
      geometry_msgs::Pose pose;
      pose.position = getPoint(i);
      pose.orientation = getQuaternion(i);
      return pose;
    }
    
    geometry_msgs::PoseStamped getPoseStamped(int i)
    {
      geometry_msgs::PoseStamped pose;
      pose.pose = getPose(i);
      pose.header = getHeader(i);
      return pose;
    }
    
    std_msgs::Header getHeader(int i)
    {
      std_msgs::Header t_header = header;
      t_header.stamp += ros::Duration(times[i]);
      return t_header;
    }
    
    ros::Duration getDuration()
    {
      return ros::Duration(times[num_states()-1]);
    }
};

typedef std::shared_ptr<trajectory_states> trajectory_ptr;

class TrajectoryGeneratorBridge
{

    traj_generator trajectory_gen_;
    std::string odom_frame_id_ = "odom";
    
    double robot_radius_;
    std::string name_="TrajectoryGeneratorBridge";

public:

TrajectoryGeneratorBridge();

void updateParams();
traj_params getDefaultParams();
void setDefaultParams(traj_params_ptr& new_params);

void generate_trajectory(trajectory_ptr trajectory);

inline
state_type initState()
{
    state_type x0(8);
    TrajectoryGeneratorBridge::initState(x0);
    return x0;
}

inline
void initState(state_type& x0, double v0)
{
    x0[near_identity::V_IND] = v0;
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
void initState(state_type& x0, const nav_msgs::Odometry::ConstPtr& curr_odom)
{
    double vx = curr_odom->twist.twist.linear.x;
    double vy = curr_odom->twist.twist.linear.y;
    double v = std::sqrt((vx*vx) + (vy*vy)); 
    double w = curr_odom->twist.twist.angular.z;

    x0[near_identity::V_IND] = v;      //v
    x0[near_identity::W_IND] = w;      //w
}

inline
void initState(trajectory_ptr& traj, const nav_msgs::Odometry::ConstPtr& curr_odom)
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

template<const nav_msgs::Odometry::ConstPtr&> state_type initState(const nav_msgs::Odometry::ConstPtr& curr_odom);

//inline
static geometry_msgs::Quaternion yawToQuaternion(const double yaw);

//inline
static double quaternionToYaw(const geometry_msgs::Quaternion& quaternion);

//inline
static const nav_msgs::OdometryPtr OdomFromState(const state_type& state, double t, const std_msgs::Header& header);

static trajectory_ptr getLongestTrajectory(const std::vector<ni_trajectory_ptr>& valid_trajs);
static trajectory_ptr getCenterLongestTrajectory(const std::vector<ni_trajectory_ptr>& valid_trajs);

static void publishPaths(const ros::Publisher& pub, const std::vector<ni_trajectory_ptr>& trajs);
static void publishDesiredPaths(const ros::Publisher& pub, const std::vector<ni_trajectory_ptr>& trajs);



};

#endif

