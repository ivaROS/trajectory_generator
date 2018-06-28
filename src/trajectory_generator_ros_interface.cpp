#include <chrono>
#include <ros/ros.h>

#include <tf/transform_datatypes.h>

#include <iostream>
#include "trajectory_generator_ros_interface.h"
#include <pips_msgs/PathArray.h>

    
    
    


TrajectoryGeneratorBridge::TrajectoryGeneratorBridge() :
trajectory_gen()
{
    robot_radius_ = .15;  //only used to initialize lambda; maybe there is a better way (to initialize it)? Definitely should pass in robot radius if it is needed
}


void TrajectoryGeneratorBridge::generate_trajectory(trajectory_ptr trajectory)
{   //How long does the integration take? Get current time
    auto t1 = std::chrono::high_resolution_clock::now();
    
    if(trajectory->params == NULL)
    {
      trajectory_gen_.run(*trajectory->trajpntr, trajectory->x0_, trajectory->x_vec, trajectory->times);
    }
    else
    {
      trajectory_gen_.run(*trajectory->trajpntr, trajectory->x0_, trajectory->x_vec, trajectory->times, *trajectory->params);
    }
    
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;
    
    ROS_DEBUG_STREAM_NAMED(name_, "Integration took " << fp_ms.count() << " ms\n");
}

/* My custom rviz display removes much of the original purpose of this. Another issue is having pointers to pips_trajectories */
void TrajectoryGeneratorBridge::publishPaths(const ros::Publisher& pub, const std::vector<trajectory_ptr>& trajs)
{
  if(pub.getNumSubscribers() > 0)
   {
    if(trajs.size() > 0)
    {
      pips_msgs::PathArray::Ptr pathArray(new pips_msgs::PathArray);
      for(size_t i = 0; i < trajs.size(); i++)
      {
        pathArray->paths.push_back(*trajs[i]->toPathMsg());
      }
      pathArray->header = pathArray->paths[0].header;
      pub.publish(pathArray);
    }
  }

}

void TrajectoryGeneratorBridge::publishDesiredPaths(const ros::Publisher& pub, const std::vector<trajectory_ptr>& trajs)
{
  if(pub.getNumSubscribers() > 0)
   {
    if(trajs.size() > 0)
    {
      pips_msgs::PathArray::Ptr pathArray(new pips_msgs::PathArray);
      for(size_t i = 0; i < trajs.size(); i++)
      {
        pathArray->paths.push_back(*trajs[i]->getDesiredPathMsg());
      }
      pathArray->header = pathArray->paths[0].header;
      pub.publish(pathArray);
    }
  }

}

void TrajectoryGeneratorBridge::updateParams()
{


}

traj_params TrajectoryGeneratorBridge::getDefaultParams()
{
  return trajectory_gen_.getDefaultParams();
}

void TrajectoryGeneratorBridge::setDefaultParams(traj_params_ptr& new_params)
{
  trajectory_gen_.setDefaultParams(*new_params);
}


geometry_msgs::Quaternion TrajectoryGeneratorBridge::yawToQuaternion(const double yaw)
{
    double roll = 0;
    double pitch = 0;
    return tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
}

double TrajectoryGeneratorBridge::quaternionToYaw(const geometry_msgs::Quaternion& quaternion)
{
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(quaternion, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    return yaw;
}




trajectory_ptr TrajectoryGeneratorBridge::getLongestTrajectory(const std::vector<trajectory_ptr>& valid_trajs)
{
    ros::Duration longest_length;
    trajectory_ptr longest_traj;
    for(size_t i=0; i < valid_trajs.size(); i++)
    {
      ros::Duration length = valid_trajs[i]->getDuration();
      if(length > longest_length)
      {
        longest_traj = valid_trajs[i];
        longest_length = length;
      }
    }
    
    return longest_traj;
}

trajectory_ptr TrajectoryGeneratorBridge::getCenterLongestTrajectory(const std::vector<trajectory_ptr>& valid_trajs)
{
    std::vector<trajectory_ptr> longest_trajs;
    
    ros::Duration longest_length;
    for(size_t i=0; i < valid_trajs.size(); i++)
    {
      ros::Duration length = valid_trajs[i]->getDuration();
      if(length > longest_length)
      {
        longest_trajs.clear();
        longest_trajs.push_back(valid_trajs[i]);
        longest_length = length;
      }
      else if(length == longest_length)
      {
        longest_trajs.push_back(valid_trajs[i]);
      }
    }
    
    trajectory_ptr longest_traj = longest_trajs[longest_trajs.size()/2];
    
    return longest_traj;
}




const nav_msgs::OdometryPtr TrajectoryGeneratorBridge::OdomFromState(const state_type& state, double t, const std_msgs::Header& header)
{
    nav_msgs::OdometryPtr odom;
    
    double x = state[near_identity::X_IND];      //x
    double y = state[near_identity::Y_IND];      //y
    double theta = state[near_identity::THETA_IND];  //theta
    double v = state[near_identity::V_IND];      //v
    double w = state[near_identity::W_IND];      //w
    
    double vx = v*std::cos(theta);
    double vy = v*std::sin(theta);
    
    odom->header.stamp = ros::Time(t);
    odom->header.frame_id = header.frame_id;
    
    odom->pose.pose.position.x = x;
    odom->pose.pose.position.y = y;
    odom->pose.pose.orientation = TrajectoryGeneratorBridge::yawToQuaternion(theta);
    odom->twist.twist.linear.x = vx;
    odom->twist.twist.linear.y = vy; 
    odom->twist.twist.angular.z = w;

    return odom;
}





    
