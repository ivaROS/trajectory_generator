#ifndef TF2_TRAJECTORY_H
#define TF2_TRAJECTORY_H

#include <trajectory_generator/trajectory_point.h>
#include <trajectory_generator/trajectory_points.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Eigen>
#include <tf2/convert.h>  //needed to give access to templates
#include <tf2_eigen/tf2_eigen.h>  //needed to transform to eigen

namespace tf2
{

inline
  void doTransform(const trajectory_generator::trajectory_point& p_in, trajectory_generator::trajectory_point& p_out, const Eigen::Affine3d& transform, double rotation)
  {
      Eigen::Vector3d pos(p_in.x,p_in.y,0);
      pos = transform * pos;
      
      double theta = p_in.theta + rotation;

      p_out.time = p_in.time;

      p_out.x = pos(0);
      p_out.y = pos(1);
      p_out.theta = theta;
      p_out.v = p_in.v;
      p_out.w = p_in.w;
      
  }
  
  
//If my message was a 'Stamped' datatype, wouldn't need these top 2
// method to extract timestamp from object
template <>
inline
  const ros::Time& getTimestamp(const trajectory_generator::trajectory_points& t) {return t.header.stamp;}

// method to extract frame id from object
template <>
inline
  const std::string& getFrameId(const trajectory_generator::trajectory_points& t) {return t.header.frame_id;}

// this method needs to be implemented by client library developers
template <>
  void doTransform(const trajectory_generator::trajectory_points& t_in, trajectory_generator::trajectory_points& t_out, const geometry_msgs::TransformStamped& transform_stamped)
  {
    
    Eigen::Affine3d transform = tf2::transformToEigen(transform_stamped);     
        
    double rotation = atan2 (transform (1, 0), transform (0, 0));
    
    for(size_t i = 0; i < t_in.points.size(); i++)
    {
      trajectory_generator::trajectory_point point;
      doTransform((trajectory_generator::trajectory_point)t_in.points[i], point, transform, rotation);
      t_out.points.push_back(point);
    
    }
    
    
    t_out.header.stamp = t_in.header.stamp; //tf2 examples use the transform's stamp...
    t_out.header.frame_id = transform_stamped.header.frame_id;
  }





// method to extract timestamp from object
template <>
inline
  const ros::Time& getTimestamp(const trajectory_generator::trajectory_pointsPtr& t) {return t->header.stamp;}

// method to extract frame id from object
template <>
inline
  const std::string& getFrameId(const trajectory_generator::trajectory_pointsPtr& t) {return t->header.frame_id;}

// this method needs to be implemented by client library developers
template <>
  void doTransform(const trajectory_generator::trajectory_pointsPtr& t_in, trajectory_generator::trajectory_pointsPtr& t_out, const geometry_msgs::TransformStamped& transform_stamped)
  {
    doTransform(*t_in,*t_out,transform_stamped);
  
  }
  
  
  //This function based on pcl eigen.hpp
  /*inline
  void getEulerAngles (const Eigen::Affine3d &t, double &roll, double &pitch, double &yaw)
    {
      roll = atan2 (t (2, 1), t (2, 2));
      pitch = asin (-t (2, 0));
      yaw = atan2 (t (1, 0), t (0, 0));
    }
    */
  
}

#endif /* TF2_TRAJECTORY_H */
