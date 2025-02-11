// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from core_interfaces:msg/DetectedMsg.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "core_interfaces/msg/detected_msg.hpp"


#ifndef CORE_INTERFACES__MSG__DETAIL__DETECTED_MSG__TRAITS_HPP_
#define CORE_INTERFACES__MSG__DETAIL__DETECTED_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "core_interfaces/msg/detail/detected_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'objects'
// Member 'boxes'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace core_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const DetectedMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: objects
  {
    if (msg.objects.size() == 0) {
      out << "objects: []";
    } else {
      out << "objects: [";
      size_t pending_items = msg.objects.size();
      for (auto item : msg.objects) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: boxes
  {
    if (msg.boxes.size() == 0) {
      out << "boxes: []";
    } else {
      out << "boxes: [";
      size_t pending_items = msg.boxes.size();
      for (auto item : msg.boxes) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DetectedMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: objects
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.objects.size() == 0) {
      out << "objects: []\n";
    } else {
      out << "objects:\n";
      for (auto item : msg.objects) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: boxes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.boxes.size() == 0) {
      out << "boxes: []\n";
    } else {
      out << "boxes:\n";
      for (auto item : msg.boxes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DetectedMsg & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace core_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use core_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const core_interfaces::msg::DetectedMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  core_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use core_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const core_interfaces::msg::DetectedMsg & msg)
{
  return core_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<core_interfaces::msg::DetectedMsg>()
{
  return "core_interfaces::msg::DetectedMsg";
}

template<>
inline const char * name<core_interfaces::msg::DetectedMsg>()
{
  return "core_interfaces/msg/DetectedMsg";
}

template<>
struct has_fixed_size<core_interfaces::msg::DetectedMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<core_interfaces::msg::DetectedMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<core_interfaces::msg::DetectedMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CORE_INTERFACES__MSG__DETAIL__DETECTED_MSG__TRAITS_HPP_
