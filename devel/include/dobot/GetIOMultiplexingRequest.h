// Generated by gencpp from file dobot/GetIOMultiplexingRequest.msg
// DO NOT EDIT!


#ifndef DOBOT_MESSAGE_GETIOMULTIPLEXINGREQUEST_H
#define DOBOT_MESSAGE_GETIOMULTIPLEXINGREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dobot
{
template <class ContainerAllocator>
struct GetIOMultiplexingRequest_
{
  typedef GetIOMultiplexingRequest_<ContainerAllocator> Type;

  GetIOMultiplexingRequest_()
    : address(0)  {
    }
  GetIOMultiplexingRequest_(const ContainerAllocator& _alloc)
    : address(0)  {
  (void)_alloc;
    }



   typedef uint8_t _address_type;
  _address_type address;





  typedef boost::shared_ptr< ::dobot::GetIOMultiplexingRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot::GetIOMultiplexingRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetIOMultiplexingRequest_

typedef ::dobot::GetIOMultiplexingRequest_<std::allocator<void> > GetIOMultiplexingRequest;

typedef boost::shared_ptr< ::dobot::GetIOMultiplexingRequest > GetIOMultiplexingRequestPtr;
typedef boost::shared_ptr< ::dobot::GetIOMultiplexingRequest const> GetIOMultiplexingRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot::GetIOMultiplexingRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot::GetIOMultiplexingRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dobot

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'dobot': ['/home/student/myros/src/dobot/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dobot::GetIOMultiplexingRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot::GetIOMultiplexingRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot::GetIOMultiplexingRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot::GetIOMultiplexingRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot::GetIOMultiplexingRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot::GetIOMultiplexingRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot::GetIOMultiplexingRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "972132462544b1029bf37f19a88e11c4";
  }

  static const char* value(const ::dobot::GetIOMultiplexingRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x972132462544b102ULL;
  static const uint64_t static_value2 = 0x9bf37f19a88e11c4ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot::GetIOMultiplexingRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot/GetIOMultiplexingRequest";
  }

  static const char* value(const ::dobot::GetIOMultiplexingRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot::GetIOMultiplexingRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 address\n"
;
  }

  static const char* value(const ::dobot::GetIOMultiplexingRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot::GetIOMultiplexingRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.address);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetIOMultiplexingRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot::GetIOMultiplexingRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot::GetIOMultiplexingRequest_<ContainerAllocator>& v)
  {
    s << indent << "address: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.address);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_MESSAGE_GETIOMULTIPLEXINGREQUEST_H
