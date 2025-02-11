// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from core_interfaces:msg/DetectedMsg.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "core_interfaces/msg/detected_msg.hpp"


#ifndef CORE_INTERFACES__MSG__DETAIL__DETECTED_MSG__STRUCT_HPP_
#define CORE_INTERFACES__MSG__DETAIL__DETECTED_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'objects'
// Member 'boxes'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__core_interfaces__msg__DetectedMsg __attribute__((deprecated))
#else
# define DEPRECATED__core_interfaces__msg__DetectedMsg __declspec(deprecated)
#endif

namespace core_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DetectedMsg_
{
  using Type = DetectedMsg_<ContainerAllocator>;

  explicit DetectedMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit DetectedMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _objects_type =
    std::vector<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>>;
  _objects_type objects;
  using _boxes_type =
    std::vector<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>>;
  _boxes_type boxes;

  // setters for named parameter idiom
  Type & set__objects(
    const std::vector<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>> & _arg)
  {
    this->objects = _arg;
    return *this;
  }
  Type & set__boxes(
    const std::vector<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>> & _arg)
  {
    this->boxes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    core_interfaces::msg::DetectedMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const core_interfaces::msg::DetectedMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<core_interfaces::msg::DetectedMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<core_interfaces::msg::DetectedMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      core_interfaces::msg::DetectedMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::msg::DetectedMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      core_interfaces::msg::DetectedMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::msg::DetectedMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<core_interfaces::msg::DetectedMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<core_interfaces::msg::DetectedMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__core_interfaces__msg__DetectedMsg
    std::shared_ptr<core_interfaces::msg::DetectedMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__core_interfaces__msg__DetectedMsg
    std::shared_ptr<core_interfaces::msg::DetectedMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectedMsg_ & other) const
  {
    if (this->objects != other.objects) {
      return false;
    }
    if (this->boxes != other.boxes) {
      return false;
    }
    return true;
  }
  bool operator!=(const DetectedMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectedMsg_

// alias to use template instance with default allocator
using DetectedMsg =
  core_interfaces::msg::DetectedMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace core_interfaces

#endif  // CORE_INTERFACES__MSG__DETAIL__DETECTED_MSG__STRUCT_HPP_
