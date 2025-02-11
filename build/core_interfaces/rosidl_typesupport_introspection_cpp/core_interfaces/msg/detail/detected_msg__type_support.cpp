// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from core_interfaces:msg/DetectedMsg.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "core_interfaces/msg/detail/detected_msg__functions.h"
#include "core_interfaces/msg/detail/detected_msg__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace core_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void DetectedMsg_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) core_interfaces::msg::DetectedMsg(_init);
}

void DetectedMsg_fini_function(void * message_memory)
{
  auto typed_message = static_cast<core_interfaces::msg::DetectedMsg *>(message_memory);
  typed_message->~DetectedMsg();
}

size_t size_function__DetectedMsg__objects(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<geometry_msgs::msg::PoseStamped> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectedMsg__objects(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<geometry_msgs::msg::PoseStamped> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectedMsg__objects(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<geometry_msgs::msg::PoseStamped> *>(untyped_member);
  return &member[index];
}

void fetch_function__DetectedMsg__objects(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::PoseStamped *>(
    get_const_function__DetectedMsg__objects(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::PoseStamped *>(untyped_value);
  value = item;
}

void assign_function__DetectedMsg__objects(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::PoseStamped *>(
    get_function__DetectedMsg__objects(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::PoseStamped *>(untyped_value);
  item = value;
}

void resize_function__DetectedMsg__objects(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<geometry_msgs::msg::PoseStamped> *>(untyped_member);
  member->resize(size);
}

size_t size_function__DetectedMsg__boxes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<geometry_msgs::msg::PoseStamped> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectedMsg__boxes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<geometry_msgs::msg::PoseStamped> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectedMsg__boxes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<geometry_msgs::msg::PoseStamped> *>(untyped_member);
  return &member[index];
}

void fetch_function__DetectedMsg__boxes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::PoseStamped *>(
    get_const_function__DetectedMsg__boxes(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::PoseStamped *>(untyped_value);
  value = item;
}

void assign_function__DetectedMsg__boxes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::PoseStamped *>(
    get_function__DetectedMsg__boxes(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::PoseStamped *>(untyped_value);
  item = value;
}

void resize_function__DetectedMsg__boxes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<geometry_msgs::msg::PoseStamped> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DetectedMsg_message_member_array[2] = {
  {
    "objects",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::PoseStamped>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(core_interfaces::msg::DetectedMsg, objects),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectedMsg__objects,  // size() function pointer
    get_const_function__DetectedMsg__objects,  // get_const(index) function pointer
    get_function__DetectedMsg__objects,  // get(index) function pointer
    fetch_function__DetectedMsg__objects,  // fetch(index, &value) function pointer
    assign_function__DetectedMsg__objects,  // assign(index, value) function pointer
    resize_function__DetectedMsg__objects  // resize(index) function pointer
  },
  {
    "boxes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::PoseStamped>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(core_interfaces::msg::DetectedMsg, boxes),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectedMsg__boxes,  // size() function pointer
    get_const_function__DetectedMsg__boxes,  // get_const(index) function pointer
    get_function__DetectedMsg__boxes,  // get(index) function pointer
    fetch_function__DetectedMsg__boxes,  // fetch(index, &value) function pointer
    assign_function__DetectedMsg__boxes,  // assign(index, value) function pointer
    resize_function__DetectedMsg__boxes  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DetectedMsg_message_members = {
  "core_interfaces::msg",  // message namespace
  "DetectedMsg",  // message name
  2,  // number of fields
  sizeof(core_interfaces::msg::DetectedMsg),
  false,  // has_any_key_member_
  DetectedMsg_message_member_array,  // message members
  DetectedMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  DetectedMsg_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DetectedMsg_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DetectedMsg_message_members,
  get_message_typesupport_handle_function,
  &core_interfaces__msg__DetectedMsg__get_type_hash,
  &core_interfaces__msg__DetectedMsg__get_type_description,
  &core_interfaces__msg__DetectedMsg__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace core_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<core_interfaces::msg::DetectedMsg>()
{
  return &::core_interfaces::msg::rosidl_typesupport_introspection_cpp::DetectedMsg_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, core_interfaces, msg, DetectedMsg)() {
  return &::core_interfaces::msg::rosidl_typesupport_introspection_cpp::DetectedMsg_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
