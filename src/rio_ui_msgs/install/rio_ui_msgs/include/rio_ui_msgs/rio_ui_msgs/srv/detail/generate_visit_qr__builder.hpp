// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rio_ui_msgs:srv/GenerateVisitQR.idl
// generated code does not contain a copyright notice

#ifndef RIO_UI_MSGS__SRV__DETAIL__GENERATE_VISIT_QR__BUILDER_HPP_
#define RIO_UI_MSGS__SRV__DETAIL__GENERATE_VISIT_QR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rio_ui_msgs/srv/detail/generate_visit_qr__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rio_ui_msgs
{

namespace srv
{

namespace builder
{

class Init_GenerateVisitQR_Request_visitor_info
{
public:
  Init_GenerateVisitQR_Request_visitor_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rio_ui_msgs::srv::GenerateVisitQR_Request visitor_info(::rio_ui_msgs::srv::GenerateVisitQR_Request::_visitor_info_type arg)
  {
    msg_.visitor_info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rio_ui_msgs::srv::GenerateVisitQR_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rio_ui_msgs::srv::GenerateVisitQR_Request>()
{
  return rio_ui_msgs::srv::builder::Init_GenerateVisitQR_Request_visitor_info();
}

}  // namespace rio_ui_msgs


namespace rio_ui_msgs
{

namespace srv
{

namespace builder
{

class Init_GenerateVisitQR_Response_qr_code_path
{
public:
  explicit Init_GenerateVisitQR_Response_qr_code_path(::rio_ui_msgs::srv::GenerateVisitQR_Response & msg)
  : msg_(msg)
  {}
  ::rio_ui_msgs::srv::GenerateVisitQR_Response qr_code_path(::rio_ui_msgs::srv::GenerateVisitQR_Response::_qr_code_path_type arg)
  {
    msg_.qr_code_path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rio_ui_msgs::srv::GenerateVisitQR_Response msg_;
};

class Init_GenerateVisitQR_Response_message
{
public:
  explicit Init_GenerateVisitQR_Response_message(::rio_ui_msgs::srv::GenerateVisitQR_Response & msg)
  : msg_(msg)
  {}
  Init_GenerateVisitQR_Response_qr_code_path message(::rio_ui_msgs::srv::GenerateVisitQR_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_GenerateVisitQR_Response_qr_code_path(msg_);
  }

private:
  ::rio_ui_msgs::srv::GenerateVisitQR_Response msg_;
};

class Init_GenerateVisitQR_Response_success
{
public:
  Init_GenerateVisitQR_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GenerateVisitQR_Response_message success(::rio_ui_msgs::srv::GenerateVisitQR_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GenerateVisitQR_Response_message(msg_);
  }

private:
  ::rio_ui_msgs::srv::GenerateVisitQR_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rio_ui_msgs::srv::GenerateVisitQR_Response>()
{
  return rio_ui_msgs::srv::builder::Init_GenerateVisitQR_Response_success();
}

}  // namespace rio_ui_msgs

#endif  // RIO_UI_MSGS__SRV__DETAIL__GENERATE_VISIT_QR__BUILDER_HPP_
