#include <chrono>
#include <ros/ros.h>

#include <tf/transform_datatypes.h>

#include <iostream>
#include "trajectory_generator_ros_interface.h"
#include <pips_msgs/PathArray.h>

    std::vector<pips_trajectory_msgs::trajectory_point> trajectory_states::toTrajectoryPointMsgs()
    {
        std::vector<pips_trajectory_msgs::trajectory_point> trajectory;
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
    
    
    pips_trajectory_msgs::trajectory_points trajectory_states::toTrajectoryMsg ()
    {
        //ni_trajectory::printTrajectory();
        std::vector<pips_trajectory_msgs::trajectory_point> points = ni_trajectory::toTrajectoryPointMsgs();
    
        pips_trajectory_msgs::trajectory_points trajectory_msg;
        trajectory_msg.points = points;
        trajectory_msg.header = header;
        
        return trajectory_msg;
    }
    
    //Not sure if this works; if it does, then the transforming function must not be
    pips_trajectory_msgs::trajectory_pointsPtr trajectory_states::toTrajectoryMsgPtr ()
    {
      pips_trajectory_msgs::trajectory_pointsPtr msgPtr(new pips_trajectory_msgs::trajectory_points);
      msgPtr->points = ni_trajectory::toTrajectoryPointMsgs();
      msgPtr->header = header;
      
      return msgPtr;
    }
    
    //TODO: change this to a toString() method for more flexibility
    void trajectory_states::print()
    {
        std::cout << "Time" << '\t' << "Error" << '\t' << 'x' << '\t' << 'y' << '\t' << "theta" << '\t' << 'v' << '\t' << 'w' << '\t' << "lambda" << '\t' << "xd" << '\t' << "yd" << std::endl;
    
        for( size_t i=0; i < this->num_states(); i++ )
        {
            double error_x = x_vec[i][near_identity::X_IND] - x_vec[i][near_identity::XD_IND];
            double error_y = x_vec[i][near_identity::Y_IND] - x_vec[i][near_identity::YD_IND];
            
            double error = sqrt(error_x*error_x + error_y*error_y);
            printf("%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n", times[i], error, x_vec[i][0], x_vec[i][1], x_vec[i][2], x_vec[i][3], x_vec[i][4], x_vec[i][5], x_vec[i][6], x_vec[i][7]);
        }
    }
    
    
    nav_msgs::PathPtr trajectory_states::toPathMsg()
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
    
    nav_msgs::PathPtr trajectory_states::getDesiredPathMsg()
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

    geometry_msgs::Point trajectory_states::getPoint(int i)
    {
        state_type state = x_vec[i];
        geometry_msgs::Point point;
        point.x = state[near_identity::X_IND];
        point.y = state[near_identity::Y_IND];
        return point;
    }
    
    geometry_msgs::PointStamped trajectory_states::getPointStamped(int i)
    {
        geometry_msgs::PointStamped point;
        point.point = getPoint(i);
        point.header = getHeader(i);
        
        return point;
    }
    
    geometry_msgs::Quaternion trajectory_states::getQuaternion(int i)
    {
        double theta = x_vec[i][near_identity::THETA_IND];
        geometry_msgs::Quaternion quat = TrajectoryGeneratorBridge::yawToQuaternion(theta);//what I had here before seemed to work, but I would prefer to encapsulate all my conversions
        return quat;
    }
    
    geometry_msgs::Pose trajectory_states::getPose(int i)
    {
        geometry_msgs::Pose pose;
        pose.position = getPoint(i);
        pose.orientation = getQuaternion(i);
        return pose;
    }
    
    geometry_msgs::PoseStamped trajectory_states::getPoseStamped(int i)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose = getPose(i);
        pose.header = getHeader(i);
        return pose;
    }
    
    std_msgs::Header trajectory_states::getHeader(int i)
    {
        std_msgs::Header t_header = header;
        t_header.stamp += ros::Duration(times[i]);
        return t_header;
    }
    
    ros::Duration trajectory_states::getDuration()
    {
        return ros::Duration(times[num_states()-1]);
    }
    
    


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





    
