// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rio_ui_msgs:msg/RobotCall.idl
// generated code does not contain a copyright notice

#ifndef RIO_UI_MSGS__MSG__DETAIL__ROBOT_CALL__TRAITS_HPP_
#define RIO_UI_MSGS__MSG__DETAIL__ROBOT_CALL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rio_ui_msgs/msg/detail/robot_call__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rio_ui_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotCall & msg,
  std::ostream & out)
{
  out << "{";
  // member: office_number
  {
    out << "office_number: ";
    rosidl_generator_traits::value_to_yaml(msg.office_number, out);
    out << ", ";
  }

  // member: date
  {
    out << "date: ";
    rosidl_generator_traits::value_to_yaml(msg.date, out);
    out << ", ";
  }

  // member: robot_type
  {
    out << "robot_type: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_type, out);
    out << ", ";
  }

  // member: destination
  {
    out << "destination: ";
    rosidl_generator_traits::value_to_yaml(msg.destination, out);
    out << ", ";
  }

  // member: receiver
  {
    out << "receiver: ";
    rosidl_generator_traits::value_to_yaml(msg.receiver, out);
    out << ", ";
  }

  // member: items
  {
    out << "items: ";
    rosidl_generator_traits::value_to_yaml(msg.items, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotCall & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: office_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "office_number: ";
    rosidl_generator_traits::value_to_yaml(msg.office_number, out);
    out << "\n";
  }

  // member: date
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "date: ";
    rosidl_generator_traits::value_to_yaml(msg.date, out);
    out << "\n";
  }

  // member: robot_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_type: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_type, out);
    out << "\n";
  }

  // member: destination
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "destination: ";
    rosidl_generator_traits::value_to_yaml(msg.destination, out);
    out << "\n";
  }

  // member: receiver
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "receiver: ";
    rosidl_generator_traits::value_to_yaml(msg.receiver, out);
    out << "\n";
  }

  // member: items
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "items: ";
    rosidl_generator_traits::value_to_yaml(msg.items, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotCall & msg, bool use_flow_style = false)
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

}  // namespace rio_ui_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rio_ui_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rio_ui_msgs::msg::RobotCall & msg,
  std::ostream & out, size_t indentation = 0)
{
  rio_ui_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rio_ui_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const rio_ui_msgs::msg::RobotCall & msg)
{
  return rio_ui_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rio_ui_msgs::msg::RobotCall>()
{
  return "rio_ui_msgs::msg::RobotCall";
}

template<>
inline const char * name<rio_ui_msgs::msg::RobotCall>()
{
  return "rio_ui_msgs/msg/RobotCall";
}

template<>
struct has_fixed_size<rio_ui_msgs::msg::RobotCall>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rio_ui_msgs::msg::RobotCall>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rio_ui_msgs::msg::RobotCall>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RIO_UI_MSGS__MSG__DETAIL__ROBOT_CALL__TRAITS_HPP_
