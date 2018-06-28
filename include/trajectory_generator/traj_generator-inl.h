#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <traj_generator.h>


template<>
void TrajectoryState::to<geometry_msgs::Point>()
{
  return static_cast<T*>(this)->to<S>(obj) 
  
}
