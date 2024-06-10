// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rio_ui_msgs:srv/QRCheck.idl
// generated code does not contain a copyright notice

#ifndef RIO_UI_MSGS__SRV__DETAIL__QR_CHECK__BUILDER_HPP_
#define RIO_UI_MSGS__SRV__DETAIL__QR_CHECK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rio_ui_msgs/srv/detail/qr_check__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rio_ui_msgs
{

namespace srv
{

namespace builder
{

class Init_QRCheck_Request_hashed_data
{
public:
  Init_QRCheck_Request_hashed_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rio_ui_msgs::srv::QRCheck_Request hashed_data(::rio_ui_msgs::srv::QRCheck_Request::_hashed_data_type arg)
  {
    msg_.hashed_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rio_ui_msgs::srv::QRCheck_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rio_ui_msgs::srv::QRCheck_Request>()
{
  return rio_ui_msgs::srv::builder::Init_QRCheck_Request_hashed_data();
}

}  // namespace rio_ui_msgs


namespace rio_ui_msgs
{

namespace srv
{

namespace builder
{

class Init_QRCheck_Response_message
{
public:
  explicit Init_QRCheck_Response_message(::rio_ui_msgs::srv::QRCheck_Response & msg)
  : msg_(msg)
  {}
  ::rio_ui_msgs::srv::QRCheck_Response message(::rio_ui_msgs::srv::QRCheck_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rio_ui_msgs::srv::QRCheck_Response msg_;
};

class Init_QRCheck_Response_success
{
public:
  Init_QRCheck_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_QRCheck_Response_message success(::rio_ui_msgs::srv::QRCheck_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_QRCheck_Response_message(msg_);
  }

private:
  ::rio_ui_msgs::srv::QRCheck_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rio_ui_msgs::srv::QRCheck_Response>()
{
  return rio_ui_msgs::srv::builder::Init_QRCheck_Response_success();
}

}  // namespace rio_ui_msgs

#endif  // RIO_UI_MSGS__SRV__DETAIL__QR_CHECK__BUILDER_HPP_
