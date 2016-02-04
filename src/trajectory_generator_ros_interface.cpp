#include "traj_generator.h"
#include <chrono>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include "trajectory_generator_ros_interface.h"


    std::vector<trajectory_generator::trajectory_point> ni_trajectory::toTrajectoryPointMsgs()
    {
        std::vector<trajectory_generator::trajectory_point> trajectory;
        for(size_t i = 0; i < x_vec.size(); i++)
        {
            trajectory_generator::trajectory_point point;
            point.time = ros::Duration(times[i]);
            point.x = x_vec[i][0];
            point.y = x_vec[i][1];
            point.theta = x_vec[i][2];
            point.v = x_vec[i][3];
            point.w = x_vec[i][4];
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
    
    void ni_trajectory::printTrajectory()
    {
        std::cout << "Time" << '\t' << "Error" << '\t' << 'x' << '\t' << 'y' << '\t' << "theta" << '\t' << 'v' << '\t' << 'w' << '\t' << "lambda" << '\t' << 'x' << '\t' << 'y' << std::endl;
    
        for( size_t i=0; i<x_vec.size(); i++ )
        {
            double error_x = x_vec[i][0] - x_vec[i][6];
            double error_y = x_vec[i][1] - x_vec[i][7];
            
            double error = sqrt(error_x*error_x + error_y*error_y);
            printf("%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n", times[i], error, x_vec[i][0], x_vec[i][1], x_vec[i][2], x_vec[i][3], x_vec[i][4], x_vec[i][5], x_vec[i][6], x_vec[i][7]);
        }
    }
    
    
/*    static nav_msgs::Path getPath(trajectory_generator::trajectory_points traj_msg)
    {
        nav_msgs::Path
    
    geometry_msgs::Quaternion quat;
    quat.w = cos(theta/2);
    quat.z = sin(theta/2);
    
    }
    */



TrajectoryGeneratorBridge::TrajectoryGeneratorBridge()
{

    traj_generator trajg;
    trajectory_gen = trajg;
    robot_radius_ = .15;
}

ni_trajectory TrajectoryGeneratorBridge::generate_trajectory(const nav_msgs::OdometryPtr curr_odom, traj_func* trajpntr)
{
    state_type x0(8);
    TrajectoryGeneratorBridge::initFromOdom(curr_odom, x0);


    trajectory_gen.setFunc(trajpntr);
    
    return TrajectoryGeneratorBridge::run(trajpntr, x0);
}
    
    
ni_trajectory TrajectoryGeneratorBridge::generate_trajectory(const geometry_msgs::TransformStampedPtr curr_tf, traj_func* trajpntr)
{
    state_type x0(8);
    TrajectoryGeneratorBridge::initFromTF(curr_tf, x0);


    trajectory_gen.setFunc(trajpntr);
    
    return TrajectoryGeneratorBridge::run(trajpntr, x0);
}

    
    

ni_trajectory TrajectoryGeneratorBridge::run(traj_func* trajpntr, state_type& x0)
{

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
    std::cout << "Integration took " << fp_ms.count() << " ms\n";



    std::cout<< "const observer: "  << steps << " steps. final: " << '\t' << x0[0] << '\t' << x0[1]<< std::endl;

    ni_trajectory traj(x_vec, times);
    
    //std::cout << traj_msgs[0] << std::endl;
    
    return traj;
}

void TrajectoryGeneratorBridge::initFromTF(const geometry_msgs::TransformStampedPtr curr_tf, state_type& x0)
{
    double x = curr_tf->transform.translation.x;
    double y = curr_tf->transform.translation.y;
    double theta = 2* std::acos(curr_tf->transform.rotation.w);
    double vx = 0;
    double vy = 0;
    double v = 0; 
    double w = 0;


//The following is an option
/*// the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    */

    x0[0] = x;      //x
    x0[1] = y;      //y
    x0[2] = theta;  //theta
    x0[3] = v;      //v
    x0[4] = w;      //w
    x0[5] = robot_radius_;    //lambda: must be > 0!
    x0[6] = x;    //x_d
    x0[7] = y;    //y_d
}


void TrajectoryGeneratorBridge::initFromOdom(const nav_msgs::OdometryPtr curr_odom, state_type& x0)
{
    double x = curr_odom->pose.pose.position.x;
    double y = curr_odom->pose.pose.position.y;
    double theta = 2* std::acos(curr_odom->pose.pose.orientation.w);
    double vx = curr_odom->twist.twist.linear.x;
    double vy = curr_odom->twist.twist.linear.y;
    double v = std::sqrt((vx*vx) + (vy*vy)); 
    double w = curr_odom->twist.twist.angular.z;


//The following is an option
/*// the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    */

    x0[0] = x;      //x
    x0[1] = y;      //y
    x0[2] = theta;  //theta
    x0[3] = v;      //v
    x0[4] = w;      //w
    x0[5] = robot_radius_;    //lambda: must be > 0!
    x0[6] = x;    //x_d
    x0[7] = y;    //y_d
}










const nav_msgs::OdometryPtr TrajectoryGeneratorBridge::OdomFromState(state_type& state, double t, std_msgs::Header header)
{
    nav_msgs::OdometryPtr odom;
    
    double x = state[0];      //x
    double y = state[1];      //y
    double theta = state[2];  //theta
    double v = state[3];      //v
    double w = state[4];      //w
    
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





    
