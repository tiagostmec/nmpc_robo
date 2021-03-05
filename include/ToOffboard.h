// Generated by gencpp from file vant/ToOffboard.msg
// DO NOT EDIT!


#ifndef VANT_MESSAGE_TOOFFBOARD_H
#define VANT_MESSAGE_TOOFFBOARD_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

namespace vant
{
template <class ContainerAllocator>
struct ToOffboard_
{
  typedef ToOffboard_<ContainerAllocator> Type;

  ToOffboard_()
    : index(0)
    , TwistStamped()
    , PoseStamped()  {
    }
  ToOffboard_(const ContainerAllocator& _alloc)
    : index(0)
    , TwistStamped(_alloc)
    , PoseStamped(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _index_type;
  _index_type index;

   typedef  ::geometry_msgs::TwistStamped_<ContainerAllocator>  _TwistStamped_type;
  _TwistStamped_type TwistStamped;

   typedef  ::geometry_msgs::PoseStamped_<ContainerAllocator>  _PoseStamped_type;
  _PoseStamped_type PoseStamped;





  typedef boost::shared_ptr< ::vant::ToOffboard_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vant::ToOffboard_<ContainerAllocator> const> ConstPtr;

}; // struct ToOffboard_

typedef ::vant::ToOffboard_<std::allocator<void> > ToOffboard;

typedef boost::shared_ptr< ::vant::ToOffboard > ToOffboardPtr;
typedef boost::shared_ptr< ::vant::ToOffboard const> ToOffboardConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vant::ToOffboard_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vant::ToOffboard_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace vant

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'vant': ['/home/laic/catkin_ws/src/vant/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::vant::ToOffboard_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vant::ToOffboard_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vant::ToOffboard_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vant::ToOffboard_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vant::ToOffboard_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vant::ToOffboard_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vant::ToOffboard_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2f3d65253d0f0079900ba15c55fab658";
  }

  static const char* value(const ::vant::ToOffboard_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2f3d65253d0f0079ULL;
  static const uint64_t static_value2 = 0x900ba15c55fab658ULL;
};

template<class ContainerAllocator>
struct DataType< ::vant::ToOffboard_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vant/ToOffboard";
  }

  static const char* value(const ::vant::ToOffboard_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vant::ToOffboard_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
int32 index\n\
geometry_msgs/TwistStamped TwistStamped\n\
geometry_msgs/PoseStamped PoseStamped\n\
\n\
================================================================================\n\
MSG: geometry_msgs/TwistStamped\n\
# A twist with reference coordinate frame and timestamp\n\
Header header\n\
Twist twist\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into its linear and angular parts.\n\
Vector3  linear\n\
Vector3  angular\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: geometry_msgs/PoseStamped\n\
# A Pose with reference coordinate frame and timestamp\n\
Header header\n\
Pose pose\n\
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

  static const char* value(const ::vant::ToOffboard_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vant::ToOffboard_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.index);
      stream.next(m.TwistStamped);
      stream.next(m.PoseStamped);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ToOffboard_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vant::ToOffboard_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vant::ToOffboard_<ContainerAllocator>& v)
  {
    s << indent << "index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.index);
    s << indent << "TwistStamped: ";
    s << std::endl;
    Printer< ::geometry_msgs::TwistStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.TwistStamped);
    s << indent << "PoseStamped: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.PoseStamped);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VANT_MESSAGE_TOOFFBOARD_H