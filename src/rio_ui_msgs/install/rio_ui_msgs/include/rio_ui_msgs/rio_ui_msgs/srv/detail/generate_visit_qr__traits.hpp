// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rio_ui_msgs:srv/GenerateVisitQR.idl
// generated code does not contain a copyright notice

#ifndef RIO_UI_MSGS__SRV__DETAIL__GENERATE_VISIT_QR__TRAITS_HPP_
#define RIO_UI_MSGS__SRV__DETAIL__GENERATE_VISIT_QR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rio_ui_msgs/srv/detail/generate_visit_qr__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rio_ui_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GenerateVisitQR_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: visitor_info
  {
    out << "visitor_info: ";
    rosidl_generator_traits::value_to_yaml(msg.visitor_info, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GenerateVisitQR_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: visitor_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "visitor_info: ";
    rosidl_generator_traits::value_to_yaml(msg.visitor_info, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GenerateVisitQR_Request & msg, bool use_flow_style = false)
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
  const rio_ui_msgs::srv::GenerateVisitQR_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rio_ui_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rio_ui_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rio_ui_msgs::srv::GenerateVisitQR_Request & msg)
{
  return rio_ui_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rio_ui_msgs::srv::GenerateVisitQR_Request>()
{
  return "rio_ui_msgs::srv::GenerateVisitQR_Request";
}

template<>
inline const char * name<rio_ui_msgs::srv::GenerateVisitQR_Request>()
{
  return "rio_ui_msgs/srv/GenerateVisitQR_Request";
}

template<>
struct has_fixed_size<rio_ui_msgs::srv::GenerateVisitQR_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rio_ui_msgs::srv::GenerateVisitQR_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rio_ui_msgs::srv::GenerateVisitQR_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rio_ui_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GenerateVisitQR_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: qr_code_path
  {
    out << "qr_code_path: ";
    rosidl_generator_traits::value_to_yaml(msg.qr_code_path, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GenerateVisitQR_Response & msg,
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

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: qr_code_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "qr_code_path: ";
    rosidl_generator_traits::value_to_yaml(msg.qr_code_path, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GenerateVisitQR_Response & msg, bool use_flow_style = false)
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
  const rio_ui_msgs::srv::GenerateVisitQR_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rio_ui_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rio_ui_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rio_ui_msgs::srv::GenerateVisitQR_Response & msg)
{
  return rio_ui_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rio_ui_msgs::srv::GenerateVisitQR_Response>()
{
  return "rio_ui_msgs::srv::GenerateVisitQR_Response";
}

template<>
inline const char * name<rio_ui_msgs::srv::GenerateVisitQR_Response>()
{
  return "rio_ui_msgs/srv/GenerateVisitQR_Response";
}

template<>
struct has_fixed_size<rio_ui_msgs::srv::GenerateVisitQR_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rio_ui_msgs::srv::GenerateVisitQR_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rio_ui_msgs::srv::GenerateVisitQR_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rio_ui_msgs::srv::GenerateVisitQR>()
{
  return "rio_ui_msgs::srv::GenerateVisitQR";
}

template<>
inline const char * name<rio_ui_msgs::srv::GenerateVisitQR>()
{
  return "rio_ui_msgs/srv/GenerateVisitQR";
}

template<>
struct has_fixed_size<rio_ui_msgs::srv::GenerateVisitQR>
  : std::integral_constant<
    bool,
    has_fixed_size<rio_ui_msgs::srv::GenerateVisitQR_Request>::value &&
    has_fixed_size<rio_ui_msgs::srv::GenerateVisitQR_Response>::value
  >
{
};

template<>
struct has_bounded_size<rio_ui_msgs::srv::GenerateVisitQR>
  : std::integral_constant<
    bool,
    has_bounded_size<rio_ui_msgs::srv::GenerateVisitQR_Request>::value &&
    has_bounded_size<rio_ui_msgs::srv::GenerateVisitQR_Response>::value
  >
{
};

template<>
struct is_service<rio_ui_msgs::srv::GenerateVisitQR>
  : std::true_type
{
};

template<>
struct is_service_request<rio_ui_msgs::srv::GenerateVisitQR_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rio_ui_msgs::srv::GenerateVisitQR_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // RIO_UI_MSGS__SRV__DETAIL__GENERATE_VISIT_QR__TRAITS_HPP_
