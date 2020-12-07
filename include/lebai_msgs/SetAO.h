// Generated by gencpp from file lebai_msgs/SetAO.msg
// DO NOT EDIT!


#ifndef LEBAI_MSGS_MESSAGE_SETAO_H
#define LEBAI_MSGS_MESSAGE_SETAO_H

#include <ros/service_traits.h>


#include <lebai_msgs/SetAORequest.h>
#include <lebai_msgs/SetAOResponse.h>


namespace lebai_msgs
{

struct SetAO
{

typedef SetAORequest Request;
typedef SetAOResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetAO
} // namespace lebai_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::lebai_msgs::SetAO > {
  static const char* value()
  {
    return "62f232c76f8ee745408ad371412c03b5";
  }

  static const char* value(const ::lebai_msgs::SetAO&) { return value(); }
};

template<>
struct DataType< ::lebai_msgs::SetAO > {
  static const char* value()
  {
    return "lebai_msgs/SetAO";
  }

  static const char* value(const ::lebai_msgs::SetAO&) { return value(); }
};


// service_traits::MD5Sum< ::lebai_msgs::SetAORequest> should match
// service_traits::MD5Sum< ::lebai_msgs::SetAO >
template<>
struct MD5Sum< ::lebai_msgs::SetAORequest>
{
  static const char* value()
  {
    return MD5Sum< ::lebai_msgs::SetAO >::value();
  }
  static const char* value(const ::lebai_msgs::SetAORequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::lebai_msgs::SetAORequest> should match
// service_traits::DataType< ::lebai_msgs::SetAO >
template<>
struct DataType< ::lebai_msgs::SetAORequest>
{
  static const char* value()
  {
    return DataType< ::lebai_msgs::SetAO >::value();
  }
  static const char* value(const ::lebai_msgs::SetAORequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::lebai_msgs::SetAOResponse> should match
// service_traits::MD5Sum< ::lebai_msgs::SetAO >
template<>
struct MD5Sum< ::lebai_msgs::SetAOResponse>
{
  static const char* value()
  {
    return MD5Sum< ::lebai_msgs::SetAO >::value();
  }
  static const char* value(const ::lebai_msgs::SetAOResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::lebai_msgs::SetAOResponse> should match
// service_traits::DataType< ::lebai_msgs::SetAO >
template<>
struct DataType< ::lebai_msgs::SetAOResponse>
{
  static const char* value()
  {
    return DataType< ::lebai_msgs::SetAO >::value();
  }
  static const char* value(const ::lebai_msgs::SetAOResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // LEBAI_MSGS_MESSAGE_SETAO_H
