#include <chrono>
#include <ros/ros.h>

#include <tf/transform_datatypes.h>

#include <iostream>
#include "trajectory_generator_ros_interface.h"

#define DEBUG false

    std::vector<trajectory_generator::trajectory_point> ni_trajectory::toTrajectoryPointMsgs()
    {
        std::vector<trajectory_generator::trajectory_point> trajectory;
        for(size_t i = 0; i < x_vec.size(); i++)
        {
            trajectory_generator::trajectory_point point;
            point.time = ros::Duration(times[i]);
            point.x = x_vec[i][X_IND];
            point.y = x_vec[i][Y_IND];
            point.theta = x_vec[i][THETA_IND];
            point.v = x_vec[i][V_IND];
            point.w = x_vec[i][W_IND];
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
        
        return trajectory_msg;
    }
    
    void ni_trajectory::print()
    {
        std::cout << "Time" << '\t' << "Error" << '\t' << 'x' << '\t' << 'y' << '\t' << "theta" << '\t' << 'v' << '\t' << 'w' << '\t' << "lambda" << '\t' << "xd" << '\t' << "yd" << std::endl;
    
        for( size_t i=0; i<x_vec.size(); i++ )
        {
            double error_x = x_vec[i][X_IND] - x_vec[i][XD_IND];
            double error_y = x_vec[i][Y_IND] - x_vec[i][YD_IND];
            
            double error = sqrt(error_x*error_x + error_y*error_y);
            printf("%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n", times[i], error, x_vec[i][0], x_vec[i][1], x_vec[i][2], x_vec[i][3], x_vec[i][4], x_vec[i][5], x_vec[i][6], x_vec[i][7]);
        }
    }
    
    
    nav_msgs::PathPtr ni_trajectory::toPathMsg()
    {
        nav_msgs::PathPtr path_msg(new nav_msgs::Path);
        path_msg->header.frame_id = frame_id;
        
        for(size_t i=0; i<x_vec.size(); i++)
        {
            state_type state = x_vec[i];
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = frame_id;
            
            pose.pose.position.x = state[X_IND];
            pose.pose.position.y = state[Y_IND];

            double theta = state[THETA_IND];
            pose.pose.orientation.w = cos(theta/2);
            pose.pose.orientation.z = sin(theta/2);

            path_msg->poses.push_back(pose);
        }
        
        return path_msg;
    }
    
    size_t ni_trajectory::num_states()
    {
        return x_vec.size();
    }
    
    geometry_msgs::Vector3 ni_trajectory::getPoint(int i)
    {
        state_type state = x_vec[i];
        geometry_msgs::Vector3 vec;
        vec.x = state[X_IND];
        vec.y = state[Y_IND];
        return vec;
    }
    



TrajectoryGeneratorBridge::TrajectoryGeneratorBridge()
{

    traj_generator trajg;
    trajectory_gen = trajg;
    robot_radius_ = .15;
}

ni_trajectory* TrajectoryGeneratorBridge::generate_trajectory(traj_func* trajpntr)
{
    state_type x0(8);
    TrajectoryGeneratorBridge::initState(x0);

    
    ni_trajectory* traj = TrajectoryGeneratorBridge::run(trajpntr, x0);
    return traj;
}

ni_trajectory* TrajectoryGeneratorBridge::generate_trajectory(traj_func* trajpntr, const nav_msgs::OdometryPtr curr_odom)
{
    state_type x0(8);
    TrajectoryGeneratorBridge::initFromOdom(curr_odom, x0);


    
    ni_trajectory* traj = TrajectoryGeneratorBridge::run(trajpntr, x0);
    traj->frame_id = curr_odom->header.frame_id;
    return traj;
}
    
    
ni_trajectory* TrajectoryGeneratorBridge::generate_trajectory(traj_func* trajpntr, geometry_msgs::TransformStamped& curr_tf)
{
    state_type x0(8);
    TrajectoryGeneratorBridge::initFromTF(curr_tf, x0);


    
    ni_trajectory* traj = TrajectoryGeneratorBridge::run(trajpntr, x0);
    traj->frame_id = curr_tf.header.frame_id;
    return traj;
}
/*
ni_trajectory TrajectoryGeneratorBridge::generate_trajectory(geometry_msgs::TransformStamped curr_tf, traj_func* trajpntr)
{
    const geometry_msgs::TransformStampedPtr curr_tfPtr = geometry_msgs::TransformStampedPtr(new geometry_msgs::TransformStamped(curr_tf));
    return TrajectoryGeneratorBridge::generate_trajectory(curr_tfPtr, trajpntr);
}
*/


ni_trajectory* TrajectoryGeneratorBridge::run(traj_func* trajpntr, state_type& x0)
{
    if(DEBUG)ROS_INFO_STREAM("Initial State: " << x0[X_IND] << " " <<
    x0[Y_IND] << " " <<
    x0[THETA_IND]<< " " <<
    x0[V_IND]<< " " <<
    x0[W_IND]<< " " <<
    x0[LAMBDA_IND] << " " <<
    x0[XD_IND] << " " <<
    x0[YD_IND] << std::endl);
    
    TrajectoryGeneratorBridge::updateParams();
    trajectory_gen.setFunc(trajpntr);
    //[ Observer samples
    std::vector<state_type> x_vec;
    std::vector<double> times;
    
    std::size_t steps;


    //How long does the integration take? Get current time
    auto t1 = std::chrono::high_resolution_clock::now();

    steps = trajectory_gen.run(x0, x_vec, times);


    
    //How long did it take? 
    //Stop the clock before all of the printouts
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;
    if(DEBUG)ROS_DEBUG_STREAM("Integration took " << fp_ms.count() << " ms\n");



    if(DEBUG)ROS_DEBUG_STREAM("const observer: "  << steps << " steps. final: " << '\t' << x0[0] << '\t' << x0[1]<< std::endl);

    ni_trajectory*  traj = new ni_trajectory(x_vec, times);
    //traj.print();
    
    //std::cout << traj_msgs[0] << std::endl;
    
    return traj;
}

void TrajectoryGeneratorBridge::publishPaths(ros::Publisher& pub, std::vector<ni_trajectory>& trajs, size_t num_total_paths)
{
    for(size_t i = 0; i < num_total_paths; i++)
    {
        nav_msgs::PathPtr path;
        if(i < trajs.size())
            path = trajs[i].toPathMsg();
        else
        {
            path = nav_msgs::PathPtr(new nav_msgs::Path);
            path->header.frame_id = "odom";
        }
        pub.publish(path);
    }

}

void TrajectoryGeneratorBridge::updateParams()
{
/*
    
    std::string key;
    
    if(ros::param::search("tf", key))
    {
        ros::param::get(key, tf); 
    }
    
*/

}

void TrajectoryGeneratorBridge::initFromTF( geometry_msgs::TransformStamped& curr_tf, state_type& x0)
{
    double x = curr_tf.transform.translation.x;
    double y = curr_tf.transform.translation.y;
    double theta = TrajectoryGeneratorBridge::quaternionToYaw(curr_tf.transform.rotation);
    double vx = 0;
    double vy = 0;
    double v = sqrt(vx*vx + vy*vy); 
    double w = 0;


//The following is an option
/*// the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    */

    x0[X_IND] = x;      //x
    x0[Y_IND] = y;      //y
    x0[THETA_IND] = theta;  //theta
    x0[V_IND] = v;      //v
    x0[W_IND] = w;      //w
    x0[LAMBDA_IND] = robot_radius_;    //lambda: must be > 0!
    x0[XD_IND] = x;    //x_d
    x0[YD_IND] = y;    //y_d
}


geometry_msgs::Quaternion TrajectoryGeneratorBridge::yawToQuaternion(double yaw)
{
    double roll = 0;
    double pitch = 0;
    return tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
}

double TrajectoryGeneratorBridge::quaternionToYaw(geometry_msgs::Quaternion& quaternion)
{
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(quaternion, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    return yaw;
}

//In a local frame, this is not necessary. A variation that only uses current velocities would be helpful, but will add later
void TrajectoryGeneratorBridge::initFromOdom(const nav_msgs::OdometryPtr curr_odom, state_type& x0)
{
    double x = curr_odom->pose.pose.position.x;
    double y = curr_odom->pose.pose.position.y;
    double theta = TrajectoryGeneratorBridge::quaternionToYaw(curr_odom->pose.pose.orientation);
    double vx = curr_odom->twist.twist.linear.x;
    double vy = curr_odom->twist.twist.linear.y;
    double v = std::sqrt((vx*vx) + (vy*vy)); 
    double w = curr_odom->twist.twist.angular.z;

    geometry_msgs::Quaternion newquat = TrajectoryGeneratorBridge::yawToQuaternion(theta);

    if(DEBUG)std::cout << "Odom quaternion: w=" << curr_odom->pose.pose.orientation.w << ", z=" << curr_odom->pose.pose.orientation.z << "; Theta from quat: " << theta << "; quat from theta: w=" << newquat.w << ", z=" << newquat.z << std::endl;


    x0[X_IND] = x;      //x
    x0[Y_IND] = y;      //y
    x0[THETA_IND] = theta;  //theta
    x0[V_IND] = v;      //v
    x0[W_IND] = w;      //w
    x0[LAMBDA_IND] = robot_radius_;    //lambda: must be > 0!
    x0[XD_IND] = x;    //x_d
    x0[YD_IND] = y;    //y_d
}



void TrajectoryGeneratorBridge::initState(state_type& x0)
{
    x0[X_IND] = 0;      //x
    x0[Y_IND] = 0;      //y
    x0[THETA_IND] = 0;  //theta
    x0[V_IND] = 0;      //v
    x0[W_IND] = 0;      //w
    x0[LAMBDA_IND] = robot_radius_;    //lambda: must be > 0!
    x0[XD_IND] = 0;    //x_d
    x0[YD_IND] = 0;    //y_d
}








const nav_msgs::OdometryPtr TrajectoryGeneratorBridge::OdomFromState(state_type& state, double t, std_msgs::Header header)
{
    nav_msgs::OdometryPtr odom;
    
    double x = state[X_IND];      //x
    double y = state[Y_IND];      //y
    double theta = state[THETA_IND];  //theta
    double v = state[V_IND];      //v
    double w = state[W_IND];      //w
    
    double vx = v*std::cos(theta);
    double vy = v*std::sin(theta);
    
    odom->header.stamp = ros::Time(t);
    odom->header.frame_id = header.frame_id;
    
    odom->pose.pose.position.x = x;
    odom->pose.pose.position.y = y;
    odom->pose.pose.orientation.w = std::cos(theta/2);
    odom->pose.pose.orientation.z = std::sin(theta/2);
    odom->twist.twist.linear.x = vx;
    odom->twist.twist.linear.y = vy; 
    odom->twist.twist.angular.z = w;

    return odom;
}





    
