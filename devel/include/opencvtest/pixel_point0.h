// Generated by gencpp from file opencvtest/pixel_point0.msg
// DO NOT EDIT!


#ifndef OPENCVTEST_MESSAGE_PIXEL_POINT0_H
#define OPENCVTEST_MESSAGE_PIXEL_POINT0_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace opencvtest
{
template <class ContainerAllocator>
struct pixel_point0_
{
  typedef pixel_point0_<ContainerAllocator> Type;

  pixel_point0_()
    : name()
    , red_u()
    , red_v()
    , yellow_u()
    , yellow_v()
    , green_u()
    , green_v()
    , purple_u()
    , purple_v()
    , orange_u()
    , orange_v()
    , blue_u()
    , blue_v()  {
    }
  pixel_point0_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , red_u(_alloc)
    , red_v(_alloc)
    , yellow_u(_alloc)
    , yellow_v(_alloc)
    , green_u(_alloc)
    , green_v(_alloc)
    , purple_u(_alloc)
    , purple_v(_alloc)
    , orange_u(_alloc)
    , orange_v(_alloc)
    , blue_u(_alloc)
    , blue_v(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _red_u_type;
  _red_u_type red_u;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _red_v_type;
  _red_v_type red_v;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _yellow_u_type;
  _yellow_u_type yellow_u;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _yellow_v_type;
  _yellow_v_type yellow_v;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _green_u_type;
  _green_u_type green_u;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _green_v_type;
  _green_v_type green_v;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _purple_u_type;
  _purple_u_type purple_u;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _purple_v_type;
  _purple_v_type purple_v;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _orange_u_type;
  _orange_u_type orange_u;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _orange_v_type;
  _orange_v_type orange_v;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _blue_u_type;
  _blue_u_type blue_u;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _blue_v_type;
  _blue_v_type blue_v;





  typedef boost::shared_ptr< ::opencvtest::pixel_point0_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::opencvtest::pixel_point0_<ContainerAllocator> const> ConstPtr;

}; // struct pixel_point0_

typedef ::opencvtest::pixel_point0_<std::allocator<void> > pixel_point0;

typedef boost::shared_ptr< ::opencvtest::pixel_point0 > pixel_point0Ptr;
typedef boost::shared_ptr< ::opencvtest::pixel_point0 const> pixel_point0ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::opencvtest::pixel_point0_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::opencvtest::pixel_point0_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace opencvtest

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'opencvtest': ['/home/student/myros/src/opencvtest/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::opencvtest::pixel_point0_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::opencvtest::pixel_point0_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::opencvtest::pixel_point0_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::opencvtest::pixel_point0_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::opencvtest::pixel_point0_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::opencvtest::pixel_point0_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::opencvtest::pixel_point0_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e04ac0d2a8d1d4cb528470b2ef466922";
  }

  static const char* value(const ::opencvtest::pixel_point0_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe04ac0d2a8d1d4cbULL;
  static const uint64_t static_value2 = 0x528470b2ef466922ULL;
};

template<class ContainerAllocator>
struct DataType< ::opencvtest::pixel_point0_<ContainerAllocator> >
{
  static const char* value()
  {
    return "opencvtest/pixel_point0";
  }

  static const char* value(const ::opencvtest::pixel_point0_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::opencvtest::pixel_point0_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string name\n"
"float64[] red_u\n"
"float64[] red_v\n"
"float64[] yellow_u\n"
"float64[] yellow_v\n"
"float64[] green_u\n"
"float64[] green_v\n"
"float64[] purple_u\n"
"float64[] purple_v\n"
"float64[] orange_u\n"
"float64[] orange_v\n"
"\n"
"float64[] blue_u\n"
"float64[] blue_v\n"
;
  }

  static const char* value(const ::opencvtest::pixel_point0_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::opencvtest::pixel_point0_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
      stream.next(m.red_u);
      stream.next(m.red_v);
      stream.next(m.yellow_u);
      stream.next(m.yellow_v);
      stream.next(m.green_u);
      stream.next(m.green_v);
      stream.next(m.purple_u);
      stream.next(m.purple_v);
      stream.next(m.orange_u);
      stream.next(m.orange_v);
      stream.next(m.blue_u);
      stream.next(m.blue_v);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct pixel_point0_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::opencvtest::pixel_point0_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::opencvtest::pixel_point0_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "red_u[]" << std::endl;
    for (size_t i = 0; i < v.red_u.size(); ++i)
    {
      s << indent << "  red_u[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.red_u[i]);
    }
    s << indent << "red_v[]" << std::endl;
    for (size_t i = 0; i < v.red_v.size(); ++i)
    {
      s << indent << "  red_v[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.red_v[i]);
    }
    s << indent << "yellow_u[]" << std::endl;
    for (size_t i = 0; i < v.yellow_u.size(); ++i)
    {
      s << indent << "  yellow_u[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.yellow_u[i]);
    }
    s << indent << "yellow_v[]" << std::endl;
    for (size_t i = 0; i < v.yellow_v.size(); ++i)
    {
      s << indent << "  yellow_v[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.yellow_v[i]);
    }
    s << indent << "green_u[]" << std::endl;
    for (size_t i = 0; i < v.green_u.size(); ++i)
    {
      s << indent << "  green_u[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.green_u[i]);
    }
    s << indent << "green_v[]" << std::endl;
    for (size_t i = 0; i < v.green_v.size(); ++i)
    {
      s << indent << "  green_v[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.green_v[i]);
    }
    s << indent << "purple_u[]" << std::endl;
    for (size_t i = 0; i < v.purple_u.size(); ++i)
    {
      s << indent << "  purple_u[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.purple_u[i]);
    }
    s << indent << "purple_v[]" << std::endl;
    for (size_t i = 0; i < v.purple_v.size(); ++i)
    {
      s << indent << "  purple_v[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.purple_v[i]);
    }
    s << indent << "orange_u[]" << std::endl;
    for (size_t i = 0; i < v.orange_u.size(); ++i)
    {
      s << indent << "  orange_u[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.orange_u[i]);
    }
    s << indent << "orange_v[]" << std::endl;
    for (size_t i = 0; i < v.orange_v.size(); ++i)
    {
      s << indent << "  orange_v[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.orange_v[i]);
    }
    s << indent << "blue_u[]" << std::endl;
    for (size_t i = 0; i < v.blue_u.size(); ++i)
    {
      s << indent << "  blue_u[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.blue_u[i]);
    }
    s << indent << "blue_v[]" << std::endl;
    for (size_t i = 0; i < v.blue_v.size(); ++i)
    {
      s << indent << "  blue_v[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.blue_v[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPENCVTEST_MESSAGE_PIXEL_POINT0_H
