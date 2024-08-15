// Generated by gencpp from file rm_msgs/MoveC.msg
// DO NOT EDIT!


#ifndef RM_MSGS_MESSAGE_MOVEC_H
#define RM_MSGS_MESSAGE_MOVEC_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose.h>

namespace rm_msgs
{
template <class ContainerAllocator>
struct MoveC_
{
  typedef MoveC_<ContainerAllocator> Type;

  MoveC_()
    : Mid_Pose()
    , End_Pose()
    , speed(0.0)  {
    }
  MoveC_(const ContainerAllocator& _alloc)
    : Mid_Pose(_alloc)
    , End_Pose(_alloc)
    , speed(0.0)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _Mid_Pose_type;
  _Mid_Pose_type Mid_Pose;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _End_Pose_type;
  _End_Pose_type End_Pose;

   typedef float _speed_type;
  _speed_type speed;





  typedef boost::shared_ptr< ::rm_msgs::MoveC_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rm_msgs::MoveC_<ContainerAllocator> const> ConstPtr;

}; // struct MoveC_

typedef ::rm_msgs::MoveC_<std::allocator<void> > MoveC;

typedef boost::shared_ptr< ::rm_msgs::MoveC > MoveCPtr;
typedef boost::shared_ptr< ::rm_msgs::MoveC const> MoveCConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rm_msgs::MoveC_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rm_msgs::MoveC_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rm_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'rm_msgs': ['/home/nvidia/catkin_ws/src/RM_Robot/rm_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rm_msgs::MoveC_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rm_msgs::MoveC_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rm_msgs::MoveC_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rm_msgs::MoveC_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rm_msgs::MoveC_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rm_msgs::MoveC_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rm_msgs::MoveC_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6450bf023cd7a5f30a68c51718bc4f21";
  }

  static const char* value(const ::rm_msgs::MoveC_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6450bf023cd7a5f3ULL;
  static const uint64_t static_value2 = 0x0a68c51718bc4f21ULL;
};

template<class ContainerAllocator>
struct DataType< ::rm_msgs::MoveC_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rm_msgs/MoveC";
  }

  static const char* value(const ::rm_msgs::MoveC_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rm_msgs::MoveC_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Pose Mid_Pose\n\
geometry_msgs/Pose End_Pose\n\
float32 speed\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::rm_msgs::MoveC_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rm_msgs::MoveC_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.Mid_Pose);
      stream.next(m.End_Pose);
      stream.next(m.speed);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoveC_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rm_msgs::MoveC_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rm_msgs::MoveC_<ContainerAllocator>& v)
  {
    s << indent << "Mid_Pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.Mid_Pose);
    s << indent << "End_Pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.End_Pose);
    s << indent << "speed: ";
    Printer<float>::stream(s, indent + "  ", v.speed);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RM_MSGS_MESSAGE_MOVEC_H