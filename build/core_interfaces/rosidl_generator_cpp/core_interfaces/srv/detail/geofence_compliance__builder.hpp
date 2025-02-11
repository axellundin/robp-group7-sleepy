// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from core_interfaces:srv/GeofenceCompliance.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "core_interfaces/srv/geofence_compliance.hpp"


#ifndef CORE_INTERFACES__SRV__DETAIL__GEOFENCE_COMPLIANCE__BUILDER_HPP_
#define CORE_INTERFACES__SRV__DETAIL__GEOFENCE_COMPLIANCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "core_interfaces/srv/detail/geofence_compliance__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace core_interfaces
{

namespace srv
{

namespace builder
{

class Init_GeofenceCompliance_Request_pose
{
public:
  Init_GeofenceCompliance_Request_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::core_interfaces::srv::GeofenceCompliance_Request pose(::core_interfaces::srv::GeofenceCompliance_Request::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::srv::GeofenceCompliance_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::srv::GeofenceCompliance_Request>()
{
  return core_interfaces::srv::builder::Init_GeofenceCompliance_Request_pose();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace srv
{

namespace builder
{

class Init_GeofenceCompliance_Response_success
{
public:
  Init_GeofenceCompliance_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::core_interfaces::srv::GeofenceCompliance_Response success(::core_interfaces::srv::GeofenceCompliance_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::srv::GeofenceCompliance_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::srv::GeofenceCompliance_Response>()
{
  return core_interfaces::srv::builder::Init_GeofenceCompliance_Response_success();
}

}  // namespace core_interfaces


namespace core_interfaces
{

namespace srv
{

namespace builder
{

class Init_GeofenceCompliance_Event_response
{
public:
  explicit Init_GeofenceCompliance_Event_response(::core_interfaces::srv::GeofenceCompliance_Event & msg)
  : msg_(msg)
  {}
  ::core_interfaces::srv::GeofenceCompliance_Event response(::core_interfaces::srv::GeofenceCompliance_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::srv::GeofenceCompliance_Event msg_;
};

class Init_GeofenceCompliance_Event_request
{
public:
  explicit Init_GeofenceCompliance_Event_request(::core_interfaces::srv::GeofenceCompliance_Event & msg)
  : msg_(msg)
  {}
  Init_GeofenceCompliance_Event_response request(::core_interfaces::srv::GeofenceCompliance_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GeofenceCompliance_Event_response(msg_);
  }

private:
  ::core_interfaces::srv::GeofenceCompliance_Event msg_;
};

class Init_GeofenceCompliance_Event_info
{
public:
  Init_GeofenceCompliance_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GeofenceCompliance_Event_request info(::core_interfaces::srv::GeofenceCompliance_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GeofenceCompliance_Event_request(msg_);
  }

private:
  ::core_interfaces::srv::GeofenceCompliance_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::srv::GeofenceCompliance_Event>()
{
  return core_interfaces::srv::builder::Init_GeofenceCompliance_Event_info();
}

}  // namespace core_interfaces

#endif  // CORE_INTERFACES__SRV__DETAIL__GEOFENCE_COMPLIANCE__BUILDER_HPP_
