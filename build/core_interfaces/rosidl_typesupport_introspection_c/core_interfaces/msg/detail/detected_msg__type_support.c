// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from core_interfaces:msg/DetectedMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "core_interfaces/msg/detail/detected_msg__rosidl_typesupport_introspection_c.h"
#include "core_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "core_interfaces/msg/detail/detected_msg__functions.h"
#include "core_interfaces/msg/detail/detected_msg__struct.h"


// Include directives for member types
// Member `objects`
// Member `boxes`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `objects`
// Member `boxes`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__DetectedMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  core_interfaces__msg__DetectedMsg__init(message_memory);
}

void core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__DetectedMsg_fini_function(void * message_memory)
{
  core_interfaces__msg__DetectedMsg__fini(message_memory);
}

size_t core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__size_function__DetectedMsg__objects(
  const void * untyped_member)
{
  const geometry_msgs__msg__PoseStamped__Sequence * member =
    (const geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return member->size;
}

const void * core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__get_const_function__DetectedMsg__objects(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__PoseStamped__Sequence * member =
    (const geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return &member->data[index];
}

void * core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__get_function__DetectedMsg__objects(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__PoseStamped__Sequence * member =
    (geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return &member->data[index];
}

void core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__fetch_function__DetectedMsg__objects(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__PoseStamped * item =
    ((const geometry_msgs__msg__PoseStamped *)
    core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__get_const_function__DetectedMsg__objects(untyped_member, index));
  geometry_msgs__msg__PoseStamped * value =
    (geometry_msgs__msg__PoseStamped *)(untyped_value);
  *value = *item;
}

void core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__assign_function__DetectedMsg__objects(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__PoseStamped * item =
    ((geometry_msgs__msg__PoseStamped *)
    core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__get_function__DetectedMsg__objects(untyped_member, index));
  const geometry_msgs__msg__PoseStamped * value =
    (const geometry_msgs__msg__PoseStamped *)(untyped_value);
  *item = *value;
}

bool core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__resize_function__DetectedMsg__objects(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__PoseStamped__Sequence * member =
    (geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  geometry_msgs__msg__PoseStamped__Sequence__fini(member);
  return geometry_msgs__msg__PoseStamped__Sequence__init(member, size);
}

size_t core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__size_function__DetectedMsg__boxes(
  const void * untyped_member)
{
  const geometry_msgs__msg__PoseStamped__Sequence * member =
    (const geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return member->size;
}

const void * core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__get_const_function__DetectedMsg__boxes(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__PoseStamped__Sequence * member =
    (const geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return &member->data[index];
}

void * core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__get_function__DetectedMsg__boxes(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__PoseStamped__Sequence * member =
    (geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return &member->data[index];
}

void core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__fetch_function__DetectedMsg__boxes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__PoseStamped * item =
    ((const geometry_msgs__msg__PoseStamped *)
    core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__get_const_function__DetectedMsg__boxes(untyped_member, index));
  geometry_msgs__msg__PoseStamped * value =
    (geometry_msgs__msg__PoseStamped *)(untyped_value);
  *value = *item;
}

void core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__assign_function__DetectedMsg__boxes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__PoseStamped * item =
    ((geometry_msgs__msg__PoseStamped *)
    core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__get_function__DetectedMsg__boxes(untyped_member, index));
  const geometry_msgs__msg__PoseStamped * value =
    (const geometry_msgs__msg__PoseStamped *)(untyped_value);
  *item = *value;
}

bool core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__resize_function__DetectedMsg__boxes(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__PoseStamped__Sequence * member =
    (geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  geometry_msgs__msg__PoseStamped__Sequence__fini(member);
  return geometry_msgs__msg__PoseStamped__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__DetectedMsg_message_member_array[2] = {
  {
    "objects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(core_interfaces__msg__DetectedMsg, objects),  // bytes offset in struct
    NULL,  // default value
    core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__size_function__DetectedMsg__objects,  // size() function pointer
    core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__get_const_function__DetectedMsg__objects,  // get_const(index) function pointer
    core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__get_function__DetectedMsg__objects,  // get(index) function pointer
    core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__fetch_function__DetectedMsg__objects,  // fetch(index, &value) function pointer
    core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__assign_function__DetectedMsg__objects,  // assign(index, value) function pointer
    core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__resize_function__DetectedMsg__objects  // resize(index) function pointer
  },
  {
    "boxes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(core_interfaces__msg__DetectedMsg, boxes),  // bytes offset in struct
    NULL,  // default value
    core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__size_function__DetectedMsg__boxes,  // size() function pointer
    core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__get_const_function__DetectedMsg__boxes,  // get_const(index) function pointer
    core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__get_function__DetectedMsg__boxes,  // get(index) function pointer
    core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__fetch_function__DetectedMsg__boxes,  // fetch(index, &value) function pointer
    core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__assign_function__DetectedMsg__boxes,  // assign(index, value) function pointer
    core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__resize_function__DetectedMsg__boxes  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__DetectedMsg_message_members = {
  "core_interfaces__msg",  // message namespace
  "DetectedMsg",  // message name
  2,  // number of fields
  sizeof(core_interfaces__msg__DetectedMsg),
  false,  // has_any_key_member_
  core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__DetectedMsg_message_member_array,  // message members
  core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__DetectedMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__DetectedMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__DetectedMsg_message_type_support_handle = {
  0,
  &core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__DetectedMsg_message_members,
  get_message_typesupport_handle_function,
  &core_interfaces__msg__DetectedMsg__get_type_hash,
  &core_interfaces__msg__DetectedMsg__get_type_description,
  &core_interfaces__msg__DetectedMsg__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_core_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, core_interfaces, msg, DetectedMsg)() {
  core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__DetectedMsg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__DetectedMsg_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  if (!core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__DetectedMsg_message_type_support_handle.typesupport_identifier) {
    core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__DetectedMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &core_interfaces__msg__DetectedMsg__rosidl_typesupport_introspection_c__DetectedMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
