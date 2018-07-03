
#ifndef NI_BOX_H
#define NI_BOX_H

#include <traj_generator.h>
#include <Eigen/Eigen>

//Since 'near identity' is no longer at the bottom but rather added from the top, it's probably ok for these dependencies to be included.
//It may be cleaner to do the conversion elsewhere, but this should be more straightforward
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <pips_trajectory_msgs/bm_traj_point.h>
#include <pips_trajectory_msgs/bm_traj_points.h>>
#include <tf/transform_datatypes.h>


class ni_state : public TrajectoryState<ni_state,5>
{
public:
  enum STATE_INDICIES { X_IND=0, Y_IND=1, THETA_IND=2, S_IND=3, PHI_IND=4};
  
  typedef pips_trajectory_msgs::bm_traj_points trajectory_msg_t;
  
  ElementReference<double> x,y,theta,s,phi;
  
  ni_state() :
  
    x(data[ni_state::X_IND]),
    y(data[ni_state::Y_IND]),
    theta(data[ni_state::THETA_IND]),
    s(data[ni_state::V_IND]),
    phi(data[ni_state::PHI_IND]),

    {
    }
  
//    TODO: this is needed for the turtlebot. Any replacment for RC car?
//   bool isValid()
//   {
//     return !(lambda ==0);
//   }
  
  //Implement any and all conversion methods
  void to(geometry_msgs::Point& point)
  {
    point.x = x;
    point.y = y;
    point.z = 0;
  }
  
  void to(geometry_msgs::Quaternion& quat)
  {
    double yaw = theta;
    double roll = 0;
    double pitch = 0;
    quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    
  }
  
  pips_trajectory_msgs::bm_traj_point toMsg()
  {
    pips_trajectory_msgs::bm_traj_point point;
    //point.time = ros::Duration(times[i]);
    point.x = x;
    point.y = y;
    point.theta = theta;
    point.s = s;
    point.phi = phi;
    
    return point;
  }
  
//   TODO:  void toString()
  
  
};


//   TODO: a prechecking class that makes sure that the paths are followable by the controler


class desired_traj_func {
  
public:
  //TODO: was there ever a reason to have this?
  virtual void init ( const ni_state &x0 )
  {
    //std::cout << "This should only print if an init function is not defined" << std::endl;
  }
  
  virtual void dState ( const ni_state &x , ni_state &dxdt , const double  t)=0;
  
  
  typedef std::shared_ptr< ::desired_traj_func> Ptr;
  
};


class ni_controller : public traj_func<ni_controller, ni_state>
{
  
//   near_identity ni_; this is for prechecking the trajectories
  desired_traj_func::Ptr traj_;   //Will look into using reference to function 
  
  
public:
  ni_controller( near_identity ni) :  ni_(ni) { }
  
  void setTrajFunc(desired_traj_func::Ptr& traj)
  {
    traj_ = traj;
  }
  
  void operator_impl ( const ni_state &x , ni_state &dxdt , const double  t  )
  {
    traj_->dState(x,dxdt,t);
//     ni_(x,dxdt,t); this is for prechecking the trajectories
  }
  
  typedef std::shared_ptr< ::ni_controller> Ptr;
  
};


#endif  /* !near_identity.h sen */

