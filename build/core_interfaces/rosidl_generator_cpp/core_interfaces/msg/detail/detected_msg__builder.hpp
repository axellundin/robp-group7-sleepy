// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from core_interfaces:msg/DetectedMsg.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "core_interfaces/msg/detected_msg.hpp"


#ifndef CORE_INTERFACES__MSG__DETAIL__DETECTED_MSG__BUILDER_HPP_
#define CORE_INTERFACES__MSG__DETAIL__DETECTED_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "core_interfaces/msg/detail/detected_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace core_interfaces
{

namespace msg
{

namespace builder
{

class Init_DetectedMsg_boxes
{
public:
  explicit Init_DetectedMsg_boxes(::core_interfaces::msg::DetectedMsg & msg)
  : msg_(msg)
  {}
  ::core_interfaces::msg::DetectedMsg boxes(::core_interfaces::msg::DetectedMsg::_boxes_type arg)
  {
    msg_.boxes = std::move(arg);
    return std::move(msg_);
  }

private:
  ::core_interfaces::msg::DetectedMsg msg_;
};

class Init_DetectedMsg_objects
{
public:
  Init_DetectedMsg_objects()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectedMsg_boxes objects(::core_interfaces::msg::DetectedMsg::_objects_type arg)
  {
    msg_.objects = std::move(arg);
    return Init_DetectedMsg_boxes(msg_);
  }

private:
  ::core_interfaces::msg::DetectedMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::core_interfaces::msg::DetectedMsg>()
{
  return core_interfaces::msg::builder::Init_DetectedMsg_objects();
}

}  // namespace core_interfaces

#endif  // CORE_INTERFACES__MSG__DETAIL__DETECTED_MSG__BUILDER_HPP_
