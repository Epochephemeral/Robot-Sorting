// Generated by gencpp from file axif_tf/getPoint.msg
// DO NOT EDIT!


#ifndef AXIF_TF_MESSAGE_GETPOINT_H
#define AXIF_TF_MESSAGE_GETPOINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace axif_tf
{
template <class ContainerAllocator>
struct getPoint_
{
  typedef getPoint_<ContainerAllocator> Type;

  getPoint_()
    : x1()
    , x2()
    , x3()  {
    }
  getPoint_(const ContainerAllocator& _alloc)
    : x1(_alloc)
    , x2(_alloc)
    , x3(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _x1_type;
  _x1_type x1;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _x2_type;
  _x2_type x2;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _x3_type;
  _x3_type x3;





  typedef boost::shared_ptr< ::axif_tf::getPoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::axif_tf::getPoint_<ContainerAllocator> const> ConstPtr;

}; // struct getPoint_

typedef ::axif_tf::getPoint_<std::allocator<void> > getPoint;

typedef boost::shared_ptr< ::axif_tf::getPoint > getPointPtr;
typedef boost::shared_ptr< ::axif_tf::getPoint const> getPointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::axif_tf::getPoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::axif_tf::getPoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace axif_tf

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'axif_tf': ['/home/student/myros/src/axif_tf/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::axif_tf::getPoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::axif_tf::getPoint_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::axif_tf::getPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::axif_tf::getPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::axif_tf::getPoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::axif_tf::getPoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::axif_tf::getPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4416512988bf8bb5f533b3277759fe81";
  }

  static const char* value(const ::axif_tf::getPoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4416512988bf8bb5ULL;
  static const uint64_t static_value2 = 0xf533b3277759fe81ULL;
};

template<class ContainerAllocator>
struct DataType< ::axif_tf::getPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "axif_tf/getPoint";
  }

  static const char* value(const ::axif_tf::getPoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::axif_tf::getPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32[] x1\n"
"float32[] x2\n"
"float32[] x3\n"
;
  }

  static const char* value(const ::axif_tf::getPoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::axif_tf::getPoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x1);
      stream.next(m.x2);
      stream.next(m.x3);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct getPoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::axif_tf::getPoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::axif_tf::getPoint_<ContainerAllocator>& v)
  {
    s << indent << "x1[]" << std::endl;
    for (size_t i = 0; i < v.x1.size(); ++i)
    {
      s << indent << "  x1[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.x1[i]);
    }
    s << indent << "x2[]" << std::endl;
    for (size_t i = 0; i < v.x2.size(); ++i)
    {
      s << indent << "  x2[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.x2[i]);
    }
    s << indent << "x3[]" << std::endl;
    for (size_t i = 0; i < v.x3.size(); ++i)
    {
      s << indent << "  x3[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.x3[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // AXIF_TF_MESSAGE_GETPOINT_H
