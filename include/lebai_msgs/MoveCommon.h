// Generated by gencpp from file lebai_msgs/MoveCommon.msg
// DO NOT EDIT!


#ifndef LEBAI_MSGS_MESSAGE_MOVECOMMON_H
#define LEBAI_MSGS_MESSAGE_MOVECOMMON_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <lebai_msgs/UntilInfo.h>

namespace lebai_msgs
{
template <class ContainerAllocator>
struct MoveCommon_
{
  typedef MoveCommon_<ContainerAllocator> Type;

  MoveCommon_()
    : vel(0.0)
    , acc(0.0)
    , time(0.0)
    , is_continous_mode(false)
    , continuation_type(0)
    , until(false)
    , until_info()  {
    }
  MoveCommon_(const ContainerAllocator& _alloc)
    : vel(0.0)
    , acc(0.0)
    , time(0.0)
    , is_continous_mode(false)
    , continuation_type(0)
    , until(false)
    , until_info(_alloc)  {
  (void)_alloc;
    }



   typedef double _vel_type;
  _vel_type vel;

   typedef double _acc_type;
  _acc_type acc;

   typedef double _time_type;
  _time_type time;

   typedef uint8_t _is_continous_mode_type;
  _is_continous_mode_type is_continous_mode;

   typedef uint8_t _continuation_type_type;
  _continuation_type_type continuation_type;

   typedef uint8_t _until_type;
  _until_type until;

   typedef  ::lebai_msgs::UntilInfo_<ContainerAllocator>  _until_info_type;
  _until_info_type until_info;





  typedef boost::shared_ptr< ::lebai_msgs::MoveCommon_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lebai_msgs::MoveCommon_<ContainerAllocator> const> ConstPtr;

}; // struct MoveCommon_

typedef ::lebai_msgs::MoveCommon_<std::allocator<void> > MoveCommon;

typedef boost::shared_ptr< ::lebai_msgs::MoveCommon > MoveCommonPtr;
typedef boost::shared_ptr< ::lebai_msgs::MoveCommon const> MoveCommonConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lebai_msgs::MoveCommon_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lebai_msgs::MoveCommon_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::lebai_msgs::MoveCommon_<ContainerAllocator1> & lhs, const ::lebai_msgs::MoveCommon_<ContainerAllocator2> & rhs)
{
  return lhs.vel == rhs.vel &&
    lhs.acc == rhs.acc &&
    lhs.time == rhs.time &&
    lhs.is_continous_mode == rhs.is_continous_mode &&
    lhs.continuation_type == rhs.continuation_type &&
    lhs.until == rhs.until &&
    lhs.until_info == rhs.until_info;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::lebai_msgs::MoveCommon_<ContainerAllocator1> & lhs, const ::lebai_msgs::MoveCommon_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace lebai_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::lebai_msgs::MoveCommon_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lebai_msgs::MoveCommon_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lebai_msgs::MoveCommon_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lebai_msgs::MoveCommon_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lebai_msgs::MoveCommon_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lebai_msgs::MoveCommon_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lebai_msgs::MoveCommon_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4ddffee09af26b88275ddf204fecc8d2";
  }

  static const char* value(const ::lebai_msgs::MoveCommon_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4ddffee09af26b88ULL;
  static const uint64_t static_value2 = 0x275ddf204fecc8d2ULL;
};

template<class ContainerAllocator>
struct DataType< ::lebai_msgs::MoveCommon_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lebai_msgs/MoveCommon";
  }

  static const char* value(const ::lebai_msgs::MoveCommon_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lebai_msgs::MoveCommon_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 vel\n"
"float64 acc\n"
"float64 time\n"
"bool is_continous_mode\n"
"# 0: normal move, 1: smooth move\n"
"uint8 continuation_type\n"
"# until data\n"
"bool until\n"
"lebai_msgs/UntilInfo until_info\n"
"================================================================================\n"
"MSG: lebai_msgs/UntilInfo\n"
"uint8 io_express_logic\n"
"lebai_msgs/IOConditionalExpress[] io_express\n"
"\n"
"uint8 LOGIC_AND=0\n"
"uint8 LOGIC_OR=1\n"
"\n"
"\n"
"================================================================================\n"
"MSG: lebai_msgs/IOConditionalExpress\n"
"uint32 group\n"
"uint32 pin\n"
"uint32 type\n"
"float64 float_value\n"
"uint8 uint_value\n"
"uint8 logic_operation\n"
"\n"
"uint8 GROUP_ROBOT=0\n"
"uint8 GROUP_FLANGE=1\n"
"\n"
"uint8 TYPE_ANALOG=0\n"
"uint8 TYPE_DIGITAL=1\n"
"\n"
"# great >\n"
"uint8 LOGIC_OP_GT=0\n"
"# great and equal >=\n"
"uint8 LOGIC_OP_GE=1\n"
"# equal\n"
"uint8 LOGIC_OP_EQ=2\n"
"# not equal\n"
"uint8 LOGIC_OP_NE=3\n"
"# less than\n"
"uint8 LOGIC_OP_LT=4\n"
"# less than and equal\n"
"uint8 LOGIC_OP_LE=5\n"
"\n"
"\n"
"\n"
;
  }

  static const char* value(const ::lebai_msgs::MoveCommon_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lebai_msgs::MoveCommon_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.vel);
      stream.next(m.acc);
      stream.next(m.time);
      stream.next(m.is_continous_mode);
      stream.next(m.continuation_type);
      stream.next(m.until);
      stream.next(m.until_info);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoveCommon_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lebai_msgs::MoveCommon_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lebai_msgs::MoveCommon_<ContainerAllocator>& v)
  {
    s << indent << "vel: ";
    Printer<double>::stream(s, indent + "  ", v.vel);
    s << indent << "acc: ";
    Printer<double>::stream(s, indent + "  ", v.acc);
    s << indent << "time: ";
    Printer<double>::stream(s, indent + "  ", v.time);
    s << indent << "is_continous_mode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_continous_mode);
    s << indent << "continuation_type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.continuation_type);
    s << indent << "until: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.until);
    s << indent << "until_info: ";
    s << std::endl;
    Printer< ::lebai_msgs::UntilInfo_<ContainerAllocator> >::stream(s, indent + "  ", v.until_info);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LEBAI_MSGS_MESSAGE_MOVECOMMON_H
