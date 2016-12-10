// Generated by gencpp from file trajectory_generator/trajectory_point.msg
// DO NOT EDIT!


#ifndef TRAJECTORY_GENERATOR_MESSAGE_TRAJECTORY_POINT_H
#define TRAJECTORY_GENERATOR_MESSAGE_TRAJECTORY_POINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace trajectory_generator
{
template <class ContainerAllocator>
struct trajectory_point_
{
  typedef trajectory_point_<ContainerAllocator> Type;

  trajectory_point_()
    : time()
    , x(0.0)
    , y(0.0)
    , theta(0.0)
    , v(0.0)
    , w(0.0)  {
    }
  trajectory_point_(const ContainerAllocator& _alloc)
    : time()
    , x(0.0)
    , y(0.0)
    , theta(0.0)
    , v(0.0)
    , w(0.0)  {
  (void)_alloc;
    }



   typedef ros::Duration _time_type;
  _time_type time;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _theta_type;
  _theta_type theta;

   typedef float _v_type;
  _v_type v;

   typedef float _w_type;
  _w_type w;




  typedef boost::shared_ptr< ::trajectory_generator::trajectory_point_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::trajectory_generator::trajectory_point_<ContainerAllocator> const> ConstPtr;

}; // struct trajectory_point_

typedef ::trajectory_generator::trajectory_point_<std::allocator<void> > trajectory_point;

typedef boost::shared_ptr< ::trajectory_generator::trajectory_point > trajectory_pointPtr;
typedef boost::shared_ptr< ::trajectory_generator::trajectory_point const> trajectory_pointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::trajectory_generator::trajectory_point_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::trajectory_generator::trajectory_point_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace trajectory_generator

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'trajectory_generator': ['/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::trajectory_generator::trajectory_point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::trajectory_generator::trajectory_point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::trajectory_generator::trajectory_point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::trajectory_generator::trajectory_point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::trajectory_generator::trajectory_point_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::trajectory_generator::trajectory_point_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::trajectory_generator::trajectory_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "849cc973a1065edc686b7f5c4489396b";
  }

  static const char* value(const ::trajectory_generator::trajectory_point_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x849cc973a1065edcULL;
  static const uint64_t static_value2 = 0x686b7f5c4489396bULL;
};

template<class ContainerAllocator>
struct DataType< ::trajectory_generator::trajectory_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "trajectory_generator/trajectory_point";
  }

  static const char* value(const ::trajectory_generator::trajectory_point_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::trajectory_generator::trajectory_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "duration time\n\
float32 x\n\
float32 y\n\
float32 theta\n\
float32 v\n\
float32 w\n\
";
  }

  static const char* value(const ::trajectory_generator::trajectory_point_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::trajectory_generator::trajectory_point_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.time);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.theta);
      stream.next(m.v);
      stream.next(m.w);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct trajectory_point_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::trajectory_generator::trajectory_point_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::trajectory_generator::trajectory_point_<ContainerAllocator>& v)
  {
    s << indent << "time: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.time);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "theta: ";
    Printer<float>::stream(s, indent + "  ", v.theta);
    s << indent << "v: ";
    Printer<float>::stream(s, indent + "  ", v.v);
    s << indent << "w: ";
    Printer<float>::stream(s, indent + "  ", v.w);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TRAJECTORY_GENERATOR_MESSAGE_TRAJECTORY_POINT_H
