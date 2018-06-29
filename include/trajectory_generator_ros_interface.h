#ifndef TRAJECTORY_GENERATOR_ROS_INTERFACE_H
#define TRAJECTORY_GENERATOR_ROS_INTERFACE_H

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


#include <tf/transform_datatypes.h>

#include <iostream>





typedef std::shared_ptr<traj_params> traj_params_ptr;

template <typename state_type, typename traj_func_type>
struct trajectory_states
{
    //using msg_state_type = typename state_type::msg_state_type;

    typedef std::shared_ptr<traj_func_type> traj_func_ptr;
    
    typedef std::shared_ptr<trajectory_states<state_type, traj_func_type>  > trajectory_ptr;
    
    //TODO: update with my current naming scheme here
    
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

    
    typename state_type::trajectory_msg_t toMsg() //-> decltype(typename state_type::trajectory_msg_t)
    {
      typename state_type::trajectory_msg_t trajectory_msg;
      //trajectory_msg.points = points;
      trajectory_msg.header = header;
      
      auto& points = trajectory_msg.points;
      
      //using state_msg_t = decltype(state_type::toMsg);
      
      //ni_trajectory::printTrajectory();
      //std::vector<auto> points;
      for(int i = 0; i < num_states(); ++i)
      {
        auto msg = x_vec[i].toMsg();
        msg.time = ros::Duration(times[i]);
        points.push_back(msg);
      }
      
      
      return trajectory_msg;
    }
    
//     //Not sure if this works; if it does, then the transforming function must not be
//     pips_trajectory_msgs::trajectory_pointsPtr toTrajectoryMsgPtr ()
//     {
//       pips_trajectory_msgs::trajectory_pointsPtr msgPtr(new pips_trajectory_msgs::trajectory_points);
//       msgPtr->points = ni_trajectory::toTrajectoryPointMsgs();
//       msgPtr->header = header;
//       
//       return msgPtr;
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
    
    size_t num_states()
    {
      return x_vec.size();
    }
    void setCollisionInd ( int i )
    {
      x_vec.resize(i);
    }
    
    ros::Duration getDuration()
    {
      return ros::Duration(times[num_states()-1]);
    }
    
    std_msgs::Header getHeader(int i)
    {
      std_msgs::Header t_header = header;
      t_header.stamp += ros::Duration(times[i]);
      return t_header;
    }
    
    geometry_msgs::Point getPoint(int i)
    {
      state_type state = x_vec[i];
      geometry_msgs::Point point;
      state.to(point);
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
      state_type state = x_vec[i];
      geometry_msgs::Quaternion quat;
      state.to(quat);
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
    
};


template<typename state_type, typename F>
class TrajectoryGeneratorBridge
{

  traj_generator<state_type,F> trajectory_gen_;
  std::string odom_frame_id_ = "odom";
  
  double robot_radius_;
  std::string name_="TrajectoryGeneratorBridge";

public:
  
  typedef std::shared_ptr<trajectory_states<state_type, F> > trajectory_ptr;

  TrajectoryGeneratorBridge()
  {
  }
  
  void generate_trajectory(trajectory_ptr trajectory) //Can this be passed by reference safely?
  {   
    //How long does the integration take? Get current time
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

  //Parameter related functions, possibly unnecessary
  //Don't remember if this ever had any function
  void updateParams()
  {
    
    
  }
  
  traj_params getDefaultParams()
  {
    return trajectory_gen_.getDefaultParams();
  }
  
  void setDefaultParams(traj_params_ptr& new_params)
  {
    trajectory_gen_.setDefaultParams(*new_params);
  }
  
  
  
  
  
  //Static convenience functions
  static geometry_msgs::Quaternion yawToQuaternion(const double yaw)
  {
    double roll = 0;
    double pitch = 0;
    return tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  }
  
  static double quaternionToYaw(const geometry_msgs::Quaternion& quaternion)
  {
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(quaternion, quat);
    
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    return yaw;
  }
  
  static trajectory_ptr getLongestTrajectory(const std::vector<trajectory_ptr>& valid_trajs)
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

  static trajectory_ptr getCenterLongestTrajectory(const std::vector<trajectory_ptr>& valid_trajs)
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

  
  /* My custom rviz display removes much of the original purpose of this. Another issue is having pointers to pips_trajectories */
  static void publishPaths(const ros::Publisher& pub, const std::vector<trajectory_ptr>& trajs)
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
  
  static void publishDesiredPaths(const ros::Publisher& pub, const std::vector<trajectory_ptr>& trajs)
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


};

#endif

