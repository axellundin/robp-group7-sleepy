// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from core_interfaces:action/MoveTo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "core_interfaces/action/move_to.hpp"


#ifndef CORE_INTERFACES__ACTION__DETAIL__MOVE_TO__STRUCT_HPP_
#define CORE_INTERFACES__ACTION__DETAIL__MOVE_TO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'way_point'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__core_interfaces__action__MoveTo_Goal __attribute__((deprecated))
#else
# define DEPRECATED__core_interfaces__action__MoveTo_Goal __declspec(deprecated)
#endif

namespace core_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveTo_Goal_
{
  using Type = MoveTo_Goal_<ContainerAllocator>;

  explicit MoveTo_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : way_point(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enforce_orientation = false;
      this->stop_at_goal = false;
    }
  }

  explicit MoveTo_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : way_point(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enforce_orientation = false;
      this->stop_at_goal = false;
    }
  }

  // field types and members
  using _way_point_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _way_point_type way_point;
  using _enforce_orientation_type =
    bool;
  _enforce_orientation_type enforce_orientation;
  using _stop_at_goal_type =
    bool;
  _stop_at_goal_type stop_at_goal;

  // setters for named parameter idiom
  Type & set__way_point(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->way_point = _arg;
    return *this;
  }
  Type & set__enforce_orientation(
    const bool & _arg)
  {
    this->enforce_orientation = _arg;
    return *this;
  }
  Type & set__stop_at_goal(
    const bool & _arg)
  {
    this->stop_at_goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    core_interfaces::action::MoveTo_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const core_interfaces::action::MoveTo_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__core_interfaces__action__MoveTo_Goal
    std::shared_ptr<core_interfaces::action::MoveTo_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__core_interfaces__action__MoveTo_Goal
    std::shared_ptr<core_interfaces::action::MoveTo_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveTo_Goal_ & other) const
  {
    if (this->way_point != other.way_point) {
      return false;
    }
    if (this->enforce_orientation != other.enforce_orientation) {
      return false;
    }
    if (this->stop_at_goal != other.stop_at_goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveTo_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveTo_Goal_

// alias to use template instance with default allocator
using MoveTo_Goal =
  core_interfaces::action::MoveTo_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace core_interfaces


#ifndef _WIN32
# define DEPRECATED__core_interfaces__action__MoveTo_Result __attribute__((deprecated))
#else
# define DEPRECATED__core_interfaces__action__MoveTo_Result __declspec(deprecated)
#endif

namespace core_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveTo_Result_
{
  using Type = MoveTo_Result_<ContainerAllocator>;

  explicit MoveTo_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit MoveTo_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    core_interfaces::action::MoveTo_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const core_interfaces::action::MoveTo_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__core_interfaces__action__MoveTo_Result
    std::shared_ptr<core_interfaces::action::MoveTo_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__core_interfaces__action__MoveTo_Result
    std::shared_ptr<core_interfaces::action::MoveTo_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveTo_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveTo_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveTo_Result_

// alias to use template instance with default allocator
using MoveTo_Result =
  core_interfaces::action::MoveTo_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace core_interfaces


#ifndef _WIN32
# define DEPRECATED__core_interfaces__action__MoveTo_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__core_interfaces__action__MoveTo_Feedback __declspec(deprecated)
#endif

namespace core_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveTo_Feedback_
{
  using Type = MoveTo_Feedback_<ContainerAllocator>;

  explicit MoveTo_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->distance_to_goal = 0.0f;
    }
  }

  explicit MoveTo_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->distance_to_goal = 0.0f;
    }
  }

  // field types and members
  using _distance_to_goal_type =
    float;
  _distance_to_goal_type distance_to_goal;

  // setters for named parameter idiom
  Type & set__distance_to_goal(
    const float & _arg)
  {
    this->distance_to_goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    core_interfaces::action::MoveTo_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const core_interfaces::action::MoveTo_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__core_interfaces__action__MoveTo_Feedback
    std::shared_ptr<core_interfaces::action::MoveTo_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__core_interfaces__action__MoveTo_Feedback
    std::shared_ptr<core_interfaces::action::MoveTo_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveTo_Feedback_ & other) const
  {
    if (this->distance_to_goal != other.distance_to_goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveTo_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveTo_Feedback_

// alias to use template instance with default allocator
using MoveTo_Feedback =
  core_interfaces::action::MoveTo_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace core_interfaces


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "core_interfaces/action/detail/move_to__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__core_interfaces__action__MoveTo_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__core_interfaces__action__MoveTo_SendGoal_Request __declspec(deprecated)
#endif

namespace core_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveTo_SendGoal_Request_
{
  using Type = MoveTo_SendGoal_Request_<ContainerAllocator>;

  explicit MoveTo_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit MoveTo_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    core_interfaces::action::MoveTo_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const core_interfaces::action::MoveTo_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    core_interfaces::action::MoveTo_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const core_interfaces::action::MoveTo_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__core_interfaces__action__MoveTo_SendGoal_Request
    std::shared_ptr<core_interfaces::action::MoveTo_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__core_interfaces__action__MoveTo_SendGoal_Request
    std::shared_ptr<core_interfaces::action::MoveTo_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveTo_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveTo_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveTo_SendGoal_Request_

// alias to use template instance with default allocator
using MoveTo_SendGoal_Request =
  core_interfaces::action::MoveTo_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace core_interfaces


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__core_interfaces__action__MoveTo_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__core_interfaces__action__MoveTo_SendGoal_Response __declspec(deprecated)
#endif

namespace core_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveTo_SendGoal_Response_
{
  using Type = MoveTo_SendGoal_Response_<ContainerAllocator>;

  explicit MoveTo_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit MoveTo_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    core_interfaces::action::MoveTo_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const core_interfaces::action::MoveTo_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__core_interfaces__action__MoveTo_SendGoal_Response
    std::shared_ptr<core_interfaces::action::MoveTo_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__core_interfaces__action__MoveTo_SendGoal_Response
    std::shared_ptr<core_interfaces::action::MoveTo_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveTo_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveTo_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveTo_SendGoal_Response_

// alias to use template instance with default allocator
using MoveTo_SendGoal_Response =
  core_interfaces::action::MoveTo_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace core_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__core_interfaces__action__MoveTo_SendGoal_Event __attribute__((deprecated))
#else
# define DEPRECATED__core_interfaces__action__MoveTo_SendGoal_Event __declspec(deprecated)
#endif

namespace core_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveTo_SendGoal_Event_
{
  using Type = MoveTo_SendGoal_Event_<ContainerAllocator>;

  explicit MoveTo_SendGoal_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit MoveTo_SendGoal_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<core_interfaces::action::MoveTo_SendGoal_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<core_interfaces::action::MoveTo_SendGoal_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<core_interfaces::action::MoveTo_SendGoal_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<core_interfaces::action::MoveTo_SendGoal_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<core_interfaces::action::MoveTo_SendGoal_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<core_interfaces::action::MoveTo_SendGoal_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<core_interfaces::action::MoveTo_SendGoal_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<core_interfaces::action::MoveTo_SendGoal_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    core_interfaces::action::MoveTo_SendGoal_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const core_interfaces::action::MoveTo_SendGoal_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_SendGoal_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_SendGoal_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_SendGoal_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_SendGoal_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_SendGoal_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_SendGoal_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_SendGoal_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_SendGoal_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__core_interfaces__action__MoveTo_SendGoal_Event
    std::shared_ptr<core_interfaces::action::MoveTo_SendGoal_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__core_interfaces__action__MoveTo_SendGoal_Event
    std::shared_ptr<core_interfaces::action::MoveTo_SendGoal_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveTo_SendGoal_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveTo_SendGoal_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveTo_SendGoal_Event_

// alias to use template instance with default allocator
using MoveTo_SendGoal_Event =
  core_interfaces::action::MoveTo_SendGoal_Event_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace core_interfaces

namespace core_interfaces
{

namespace action
{

struct MoveTo_SendGoal
{
  using Request = core_interfaces::action::MoveTo_SendGoal_Request;
  using Response = core_interfaces::action::MoveTo_SendGoal_Response;
  using Event = core_interfaces::action::MoveTo_SendGoal_Event;
};

}  // namespace action

}  // namespace core_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__core_interfaces__action__MoveTo_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__core_interfaces__action__MoveTo_GetResult_Request __declspec(deprecated)
#endif

namespace core_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveTo_GetResult_Request_
{
  using Type = MoveTo_GetResult_Request_<ContainerAllocator>;

  explicit MoveTo_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit MoveTo_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    core_interfaces::action::MoveTo_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const core_interfaces::action::MoveTo_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__core_interfaces__action__MoveTo_GetResult_Request
    std::shared_ptr<core_interfaces::action::MoveTo_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__core_interfaces__action__MoveTo_GetResult_Request
    std::shared_ptr<core_interfaces::action::MoveTo_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveTo_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveTo_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveTo_GetResult_Request_

// alias to use template instance with default allocator
using MoveTo_GetResult_Request =
  core_interfaces::action::MoveTo_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace core_interfaces


// Include directives for member types
// Member 'result'
// already included above
// #include "core_interfaces/action/detail/move_to__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__core_interfaces__action__MoveTo_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__core_interfaces__action__MoveTo_GetResult_Response __declspec(deprecated)
#endif

namespace core_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveTo_GetResult_Response_
{
  using Type = MoveTo_GetResult_Response_<ContainerAllocator>;

  explicit MoveTo_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit MoveTo_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    core_interfaces::action::MoveTo_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const core_interfaces::action::MoveTo_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    core_interfaces::action::MoveTo_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const core_interfaces::action::MoveTo_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__core_interfaces__action__MoveTo_GetResult_Response
    std::shared_ptr<core_interfaces::action::MoveTo_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__core_interfaces__action__MoveTo_GetResult_Response
    std::shared_ptr<core_interfaces::action::MoveTo_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveTo_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveTo_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveTo_GetResult_Response_

// alias to use template instance with default allocator
using MoveTo_GetResult_Response =
  core_interfaces::action::MoveTo_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace core_interfaces


// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__core_interfaces__action__MoveTo_GetResult_Event __attribute__((deprecated))
#else
# define DEPRECATED__core_interfaces__action__MoveTo_GetResult_Event __declspec(deprecated)
#endif

namespace core_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveTo_GetResult_Event_
{
  using Type = MoveTo_GetResult_Event_<ContainerAllocator>;

  explicit MoveTo_GetResult_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit MoveTo_GetResult_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<core_interfaces::action::MoveTo_GetResult_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<core_interfaces::action::MoveTo_GetResult_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<core_interfaces::action::MoveTo_GetResult_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<core_interfaces::action::MoveTo_GetResult_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<core_interfaces::action::MoveTo_GetResult_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<core_interfaces::action::MoveTo_GetResult_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<core_interfaces::action::MoveTo_GetResult_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<core_interfaces::action::MoveTo_GetResult_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    core_interfaces::action::MoveTo_GetResult_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const core_interfaces::action::MoveTo_GetResult_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_GetResult_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_GetResult_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_GetResult_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_GetResult_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_GetResult_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_GetResult_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_GetResult_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_GetResult_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__core_interfaces__action__MoveTo_GetResult_Event
    std::shared_ptr<core_interfaces::action::MoveTo_GetResult_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__core_interfaces__action__MoveTo_GetResult_Event
    std::shared_ptr<core_interfaces::action::MoveTo_GetResult_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveTo_GetResult_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveTo_GetResult_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveTo_GetResult_Event_

// alias to use template instance with default allocator
using MoveTo_GetResult_Event =
  core_interfaces::action::MoveTo_GetResult_Event_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace core_interfaces

namespace core_interfaces
{

namespace action
{

struct MoveTo_GetResult
{
  using Request = core_interfaces::action::MoveTo_GetResult_Request;
  using Response = core_interfaces::action::MoveTo_GetResult_Response;
  using Event = core_interfaces::action::MoveTo_GetResult_Event;
};

}  // namespace action

}  // namespace core_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "core_interfaces/action/detail/move_to__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__core_interfaces__action__MoveTo_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__core_interfaces__action__MoveTo_FeedbackMessage __declspec(deprecated)
#endif

namespace core_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveTo_FeedbackMessage_
{
  using Type = MoveTo_FeedbackMessage_<ContainerAllocator>;

  explicit MoveTo_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit MoveTo_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    core_interfaces::action::MoveTo_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const core_interfaces::action::MoveTo_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    core_interfaces::action::MoveTo_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const core_interfaces::action::MoveTo_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<core_interfaces::action::MoveTo_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      core_interfaces::action::MoveTo_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<core_interfaces::action::MoveTo_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<core_interfaces::action::MoveTo_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__core_interfaces__action__MoveTo_FeedbackMessage
    std::shared_ptr<core_interfaces::action::MoveTo_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__core_interfaces__action__MoveTo_FeedbackMessage
    std::shared_ptr<core_interfaces::action::MoveTo_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveTo_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveTo_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveTo_FeedbackMessage_

// alias to use template instance with default allocator
using MoveTo_FeedbackMessage =
  core_interfaces::action::MoveTo_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace core_interfaces

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace core_interfaces
{

namespace action
{

struct MoveTo
{
  /// The goal message defined in the action definition.
  using Goal = core_interfaces::action::MoveTo_Goal;
  /// The result message defined in the action definition.
  using Result = core_interfaces::action::MoveTo_Result;
  /// The feedback message defined in the action definition.
  using Feedback = core_interfaces::action::MoveTo_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = core_interfaces::action::MoveTo_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = core_interfaces::action::MoveTo_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = core_interfaces::action::MoveTo_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct MoveTo MoveTo;

}  // namespace action

}  // namespace core_interfaces

#endif  // CORE_INTERFACES__ACTION__DETAIL__MOVE_TO__STRUCT_HPP_
