#include <chrono>
#include <ros/ros.h>

#include <tf/transform_datatypes.h>

#include <iostream>
#include "trajectory_generator_ros_interface.h"
#include <pips_msgs/PathArray.h>

    std::vector<trajectory_generator::trajectory_point> ni_trajectory::toTrajectoryPointMsgs()
    {
        std::vector<trajectory_generator::trajectory_point> trajectory;
        for(size_t i = 0; i < this->num_states(); i++)
        {
            trajectory_generator::trajectory_point point;
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
    
    
    trajectory_generator::trajectory_points ni_trajectory::toTrajectoryMsg ()
    {
        //ni_trajectory::printTrajectory();
        std::vector<trajectory_generator::trajectory_point> points = ni_trajectory::toTrajectoryPointMsgs();
    
        trajectory_generator::trajectory_points trajectory_msg;
        trajectory_msg.points = points;
        trajectory_msg.header = header;
        
        return trajectory_msg;
    }
    
    //Not sure if this works; if it does, then the transforming function must not be
    trajectory_generator::trajectory_pointsPtr ni_trajectory::toTrajectoryMsgPtr ()
    {
      trajectory_generator::trajectory_pointsPtr msgPtr(new trajectory_generator::trajectory_points);
      msgPtr->points = ni_trajectory::toTrajectoryPointMsgs();
      msgPtr->header = header;
      
      return msgPtr;
    }
    
    void ni_trajectory::print()
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
    
    
    nav_msgs::PathPtr ni_trajectory::toPathMsg()
    {
        nav_msgs::PathPtr path_msg(new nav_msgs::Path);
        path_msg->header = header;
        
        for(size_t i=0; i < this->num_states(); i++)
        {
            state_type state = x_vec[i];
            geometry_msgs::PoseStamped pose;
            pose.header = header;
            
            pose.pose.position.x = state[near_identity::X_IND];
            pose.pose.position.y = state[near_identity::Y_IND];

            double theta = state[near_identity::THETA_IND];
            pose.pose.orientation = TrajectoryGeneratorBridge::yawToQuaternion(theta);//what I had here before seemed to work, but I would prefer to encapsulate all my conversions

            path_msg->poses.push_back(pose);
        }
        
        return path_msg;
    }
    
    nav_msgs::PathPtr ni_trajectory::getDesiredPathMsg()
    {
        nav_msgs::PathPtr path_msg(new nav_msgs::Path);
        path_msg->header = header;
        
        for(size_t i=0; i < this->num_states(); i++)
        {
            state_type state = x_vec[i];
            geometry_msgs::PoseStamped pose;
            pose.header = header;
            
            pose.pose.position.x = state[near_identity::XD_IND];
            pose.pose.position.y = state[near_identity::YD_IND];

            path_msg->poses.push_back(pose);
        }
        
        return path_msg;
    }
    
    size_t ni_trajectory::num_states()
    {
        return x_vec.size();
    }
    
    geometry_msgs::Point ni_trajectory::getPoint(int i)
    {
        state_type state = x_vec[i];
        geometry_msgs::Point point;
        point.x = state[near_identity::X_IND];
        point.y = state[near_identity::Y_IND];
        return point;
    }
    
    geometry_msgs::PointStamped ni_trajectory::getPointStamped(int i)
    {
        geometry_msgs::PointStamped point;
        point.point = getPoint(i);
        point.header = header;
        point.header.stamp += ros::Duration(times[i]);
        
        return point;
    }
    
    ros::Duration ni_trajectory::getDuration()
    {
        return ros::Duration(times[num_states()-1]);
    }
    
    


TrajectoryGeneratorBridge::TrajectoryGeneratorBridge() :
trajectory_gen()  //Another option is to use a smart pointer
{
    robot_radius_ = .15;  //only used to initialize lambda; maybe there is a better way (to initialize it)? Definitely should pass in robot radius if it is needed
}


void TrajectoryGeneratorBridge::generate_trajectory(ni_trajectory_ptr trajectory)
{   //How long does the integration take? Get current time
    auto t1 = std::chrono::high_resolution_clock::now();
    
    if(trajectory->params == NULL)
    {
      trajectory_gen.run(*trajectory->trajpntr, trajectory->x0_, trajectory->x_vec, trajectory->times);
    }
    else
    {
      trajectory_gen.run(*trajectory->trajpntr, trajectory->x0_, trajectory->x_vec, trajectory->times, *trajectory->params);
    }
    
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;
    
    ROS_DEBUG_STREAM_NAMED(name_, "Integration took " << fp_ms.count() << " ms\n");
}

/* My custom rviz display removes much of the original purpose of this. Another issue is having pointers to pips_trajectories */
void TrajectoryGeneratorBridge::publishPaths(const ros::Publisher& pub, const std::vector<ni_trajectory_ptr>& trajs)
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

void TrajectoryGeneratorBridge::publishDesiredPaths(const ros::Publisher& pub, const std::vector<ni_trajectory_ptr>& trajs)
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
  return trajectory_gen.getDefaultParams();
}

void TrajectoryGeneratorBridge::setDefaultParams(traj_params_ptr& new_params)
{
  trajectory_gen.setDefaultParams(*new_params);
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




ni_trajectory_ptr TrajectoryGeneratorBridge::getLongestTrajectory(const std::vector<ni_trajectory_ptr>& valid_trajs)
{
    ros::Duration longest_length;
    ni_trajectory_ptr longest_traj;
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

ni_trajectory_ptr TrajectoryGeneratorBridge::getCenterLongestTrajectory(const std::vector<ni_trajectory_ptr>& valid_trajs)
{
    std::vector<ni_trajectory_ptr> longest_trajs;
    
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
    
    ni_trajectory_ptr longest_traj = longest_trajs[longest_trajs.size()/2];
    
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





    
