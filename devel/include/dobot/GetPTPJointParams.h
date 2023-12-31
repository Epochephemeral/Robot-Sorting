// Generated by gencpp from file dobot/GetPTPJointParams.msg
// DO NOT EDIT!


#ifndef DOBOT_MESSAGE_GETPTPJOINTPARAMS_H
#define DOBOT_MESSAGE_GETPTPJOINTPARAMS_H

#include <ros/service_traits.h>


#include <dobot/GetPTPJointParamsRequest.h>
#include <dobot/GetPTPJointParamsResponse.h>


namespace dobot
{

struct GetPTPJointParams
{

typedef GetPTPJointParamsRequest Request;
typedef GetPTPJointParamsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetPTPJointParams
} // namespace dobot


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot::GetPTPJointParams > {
  static const char* value()
  {
    return "46bf5bf78db0edaa99dd346e0307937e";
  }

  static const char* value(const ::dobot::GetPTPJointParams&) { return value(); }
};

template<>
struct DataType< ::dobot::GetPTPJointParams > {
  static const char* value()
  {
    return "dobot/GetPTPJointParams";
  }

  static const char* value(const ::dobot::GetPTPJointParams&) { return value(); }
};


// service_traits::MD5Sum< ::dobot::GetPTPJointParamsRequest> should match 
// service_traits::MD5Sum< ::dobot::GetPTPJointParams > 
template<>
struct MD5Sum< ::dobot::GetPTPJointParamsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot::GetPTPJointParams >::value();
  }
  static const char* value(const ::dobot::GetPTPJointParamsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot::GetPTPJointParamsRequest> should match 
// service_traits::DataType< ::dobot::GetPTPJointParams > 
template<>
struct DataType< ::dobot::GetPTPJointParamsRequest>
{
  static const char* value()
  {
    return DataType< ::dobot::GetPTPJointParams >::value();
  }
  static const char* value(const ::dobot::GetPTPJointParamsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot::GetPTPJointParamsResponse> should match 
// service_traits::MD5Sum< ::dobot::GetPTPJointParams > 
template<>
struct MD5Sum< ::dobot::GetPTPJointParamsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot::GetPTPJointParams >::value();
  }
  static const char* value(const ::dobot::GetPTPJointParamsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot::GetPTPJointParamsResponse> should match 
// service_traits::DataType< ::dobot::GetPTPJointParams > 
template<>
struct DataType< ::dobot::GetPTPJointParamsResponse>
{
  static const char* value()
  {
    return DataType< ::dobot::GetPTPJointParams >::value();
  }
  static const char* value(const ::dobot::GetPTPJointParamsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_MESSAGE_GETPTPJOINTPARAMS_H
