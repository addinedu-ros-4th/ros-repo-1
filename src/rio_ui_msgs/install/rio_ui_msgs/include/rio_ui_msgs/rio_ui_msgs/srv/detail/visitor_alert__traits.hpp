// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rio_ui_msgs:srv/VisitorAlert.idl
// generated code does not contain a copyright notice

#ifndef RIO_UI_MSGS__SRV__DETAIL__VISITOR_ALERT__TRAITS_HPP_
#define RIO_UI_MSGS__SRV__DETAIL__VISITOR_ALERT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rio_ui_msgs/srv/detail/visitor_alert__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rio_ui_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const VisitorAlert_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: affiliation
  {
    out << "affiliation: ";
    rosidl_generator_traits::value_to_yaml(msg.affiliation, out);
    out << ", ";
  }

  // member: visit_place
  {
    out << "visit_place: ";
    rosidl_generator_traits::value_to_yaml(msg.visit_place, out);
    out << ", ";
  }

  // member: robot_guidance
  {
    out << "robot_guidance: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_guidance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VisitorAlert_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: affiliation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "affiliation: ";
    rosidl_generator_traits::value_to_yaml(msg.affiliation, out);
    out << "\n";
  }

  // member: visit_place
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "visit_place: ";
    rosidl_generator_traits::value_to_yaml(msg.visit_place, out);
    out << "\n";
  }

  // member: robot_guidance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_guidance: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_guidance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VisitorAlert_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rio_ui_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rio_ui_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rio_ui_msgs::srv::VisitorAlert_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rio_ui_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rio_ui_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rio_ui_msgs::srv::VisitorAlert_Request & msg)
{
  return rio_ui_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rio_ui_msgs::srv::VisitorAlert_Request>()
{
  return "rio_ui_msgs::srv::VisitorAlert_Request";
}

template<>
inline const char * name<rio_ui_msgs::srv::VisitorAlert_Request>()
{
  return "rio_ui_msgs/srv/VisitorAlert_Request";
}

template<>
struct has_fixed_size<rio_ui_msgs::srv::VisitorAlert_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rio_ui_msgs::srv::VisitorAlert_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rio_ui_msgs::srv::VisitorAlert_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rio_ui_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const VisitorAlert_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VisitorAlert_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VisitorAlert_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rio_ui_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rio_ui_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rio_ui_msgs::srv::VisitorAlert_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rio_ui_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rio_ui_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rio_ui_msgs::srv::VisitorAlert_Response & msg)
{
  return rio_ui_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rio_ui_msgs::srv::VisitorAlert_Response>()
{
  return "rio_ui_msgs::srv::VisitorAlert_Response";
}

template<>
inline const char * name<rio_ui_msgs::srv::VisitorAlert_Response>()
{
  return "rio_ui_msgs/srv/VisitorAlert_Response";
}

template<>
struct has_fixed_size<rio_ui_msgs::srv::VisitorAlert_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rio_ui_msgs::srv::VisitorAlert_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rio_ui_msgs::srv::VisitorAlert_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rio_ui_msgs::srv::VisitorAlert>()
{
  return "rio_ui_msgs::srv::VisitorAlert";
}

template<>
inline const char * name<rio_ui_msgs::srv::VisitorAlert>()
{
  return "rio_ui_msgs/srv/VisitorAlert";
}

template<>
struct has_fixed_size<rio_ui_msgs::srv::VisitorAlert>
  : std::integral_constant<
    bool,
    has_fixed_size<rio_ui_msgs::srv::VisitorAlert_Request>::value &&
    has_fixed_size<rio_ui_msgs::srv::VisitorAlert_Response>::value
  >
{
};

template<>
struct has_bounded_size<rio_ui_msgs::srv::VisitorAlert>
  : std::integral_constant<
    bool,
    has_bounded_size<rio_ui_msgs::srv::VisitorAlert_Request>::value &&
    has_bounded_size<rio_ui_msgs::srv::VisitorAlert_Response>::value
  >
{
};

template<>
struct is_service<rio_ui_msgs::srv::VisitorAlert>
  : std::true_type
{
};

template<>
struct is_service_request<rio_ui_msgs::srv::VisitorAlert_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rio_ui_msgs::srv::VisitorAlert_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // RIO_UI_MSGS__SRV__DETAIL__VISITOR_ALERT__TRAITS_HPP_
