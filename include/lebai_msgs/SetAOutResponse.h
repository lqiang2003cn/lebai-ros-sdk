// Generated by gencpp from file lebai_msgs/SetAOutResponse.msg
// DO NOT EDIT!


#ifndef LEBAI_MSGS_MESSAGE_SETAOUTRESPONSE_H
#define LEBAI_MSGS_MESSAGE_SETAOUTRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <industrial_msgs/ServiceReturnCode.h>

namespace lebai_msgs
{
template <class ContainerAllocator>
struct SetAOutResponse_
{
  typedef SetAOutResponse_<ContainerAllocator> Type;

  SetAOutResponse_()
    : code()  {
    }
  SetAOutResponse_(const ContainerAllocator& _alloc)
    : code(_alloc)  {
  (void)_alloc;
    }



   typedef  ::industrial_msgs::ServiceReturnCode_<ContainerAllocator>  _code_type;
  _code_type code;





  typedef boost::shared_ptr< ::lebai_msgs::SetAOutResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lebai_msgs::SetAOutResponse_<ContainerAllocator> const> ConstPtr;

}; // struct SetAOutResponse_

typedef ::lebai_msgs::SetAOutResponse_<std::allocator<void> > SetAOutResponse;

typedef boost::shared_ptr< ::lebai_msgs::SetAOutResponse > SetAOutResponsePtr;
typedef boost::shared_ptr< ::lebai_msgs::SetAOutResponse const> SetAOutResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lebai_msgs::SetAOutResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lebai_msgs::SetAOutResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::lebai_msgs::SetAOutResponse_<ContainerAllocator1> & lhs, const ::lebai_msgs::SetAOutResponse_<ContainerAllocator2> & rhs)
{
  return lhs.code == rhs.code;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::lebai_msgs::SetAOutResponse_<ContainerAllocator1> & lhs, const ::lebai_msgs::SetAOutResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace lebai_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::lebai_msgs::SetAOutResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lebai_msgs::SetAOutResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lebai_msgs::SetAOutResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lebai_msgs::SetAOutResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lebai_msgs::SetAOutResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lebai_msgs::SetAOutResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lebai_msgs::SetAOutResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "50b1f38f75f5677e5692f3b3e7e1ea48";
  }

  static const char* value(const ::lebai_msgs::SetAOutResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x50b1f38f75f5677eULL;
  static const uint64_t static_value2 = 0x5692f3b3e7e1ea48ULL;
};

template<class ContainerAllocator>
struct DataType< ::lebai_msgs::SetAOutResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lebai_msgs/SetAOutResponse";
  }

  static const char* value(const ::lebai_msgs::SetAOutResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lebai_msgs::SetAOutResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "industrial_msgs/ServiceReturnCode code\n"
"\n"
"================================================================================\n"
"MSG: industrial_msgs/ServiceReturnCode\n"
"# Service return codes for simple requests.  All ROS-Industrial service\n"
"# replies are required to have a return code indicating success or failure\n"
"# Specific return codes for different failure should be negative.\n"
"int8 val\n"
"\n"
"int8 SUCCESS = 1\n"
"int8 FAILURE = -1\n"
"\n"
;
  }

  static const char* value(const ::lebai_msgs::SetAOutResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lebai_msgs::SetAOutResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.code);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetAOutResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lebai_msgs::SetAOutResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lebai_msgs::SetAOutResponse_<ContainerAllocator>& v)
  {
    s << indent << "code: ";
    s << std::endl;
    Printer< ::industrial_msgs::ServiceReturnCode_<ContainerAllocator> >::stream(s, indent + "  ", v.code);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LEBAI_MSGS_MESSAGE_SETAOUTRESPONSE_H
