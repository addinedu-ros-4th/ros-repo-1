// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rio_ui_msgs:srv/VisitorAlert.idl
// generated code does not contain a copyright notice

#ifndef RIO_UI_MSGS__SRV__DETAIL__VISITOR_ALERT__BUILDER_HPP_
#define RIO_UI_MSGS__SRV__DETAIL__VISITOR_ALERT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rio_ui_msgs/srv/detail/visitor_alert__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rio_ui_msgs
{

namespace srv
{

namespace builder
{

class Init_VisitorAlert_Request_robot_guidance
{
public:
  explicit Init_VisitorAlert_Request_robot_guidance(::rio_ui_msgs::srv::VisitorAlert_Request & msg)
  : msg_(msg)
  {}
  ::rio_ui_msgs::srv::VisitorAlert_Request robot_guidance(::rio_ui_msgs::srv::VisitorAlert_Request::_robot_guidance_type arg)
  {
    msg_.robot_guidance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rio_ui_msgs::srv::VisitorAlert_Request msg_;
};

class Init_VisitorAlert_Request_visit_place
{
public:
  explicit Init_VisitorAlert_Request_visit_place(::rio_ui_msgs::srv::VisitorAlert_Request & msg)
  : msg_(msg)
  {}
  Init_VisitorAlert_Request_robot_guidance visit_place(::rio_ui_msgs::srv::VisitorAlert_Request::_visit_place_type arg)
  {
    msg_.visit_place = std::move(arg);
    return Init_VisitorAlert_Request_robot_guidance(msg_);
  }

private:
  ::rio_ui_msgs::srv::VisitorAlert_Request msg_;
};

class Init_VisitorAlert_Request_affiliation
{
public:
  explicit Init_VisitorAlert_Request_affiliation(::rio_ui_msgs::srv::VisitorAlert_Request & msg)
  : msg_(msg)
  {}
  Init_VisitorAlert_Request_visit_place affiliation(::rio_ui_msgs::srv::VisitorAlert_Request::_affiliation_type arg)
  {
    msg_.affiliation = std::move(arg);
    return Init_VisitorAlert_Request_visit_place(msg_);
  }

private:
  ::rio_ui_msgs::srv::VisitorAlert_Request msg_;
};

class Init_VisitorAlert_Request_name
{
public:
  Init_VisitorAlert_Request_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VisitorAlert_Request_affiliation name(::rio_ui_msgs::srv::VisitorAlert_Request::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_VisitorAlert_Request_affiliation(msg_);
  }

private:
  ::rio_ui_msgs::srv::VisitorAlert_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rio_ui_msgs::srv::VisitorAlert_Request>()
{
  return rio_ui_msgs::srv::builder::Init_VisitorAlert_Request_name();
}

}  // namespace rio_ui_msgs


namespace rio_ui_msgs
{

namespace srv
{

namespace builder
{

class Init_VisitorAlert_Response_success
{
public:
  Init_VisitorAlert_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rio_ui_msgs::srv::VisitorAlert_Response success(::rio_ui_msgs::srv::VisitorAlert_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rio_ui_msgs::srv::VisitorAlert_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rio_ui_msgs::srv::VisitorAlert_Response>()
{
  return rio_ui_msgs::srv::builder::Init_VisitorAlert_Response_success();
}

}  // namespace rio_ui_msgs

#endif  // RIO_UI_MSGS__SRV__DETAIL__VISITOR_ALERT__BUILDER_HPP_
