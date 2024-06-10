// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rio_ui_msgs:msg/RobotCall.idl
// generated code does not contain a copyright notice

#ifndef RIO_UI_MSGS__MSG__DETAIL__ROBOT_CALL__BUILDER_HPP_
#define RIO_UI_MSGS__MSG__DETAIL__ROBOT_CALL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rio_ui_msgs/msg/detail/robot_call__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rio_ui_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotCall_items
{
public:
  explicit Init_RobotCall_items(::rio_ui_msgs::msg::RobotCall & msg)
  : msg_(msg)
  {}
  ::rio_ui_msgs::msg::RobotCall items(::rio_ui_msgs::msg::RobotCall::_items_type arg)
  {
    msg_.items = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rio_ui_msgs::msg::RobotCall msg_;
};

class Init_RobotCall_receiver
{
public:
  explicit Init_RobotCall_receiver(::rio_ui_msgs::msg::RobotCall & msg)
  : msg_(msg)
  {}
  Init_RobotCall_items receiver(::rio_ui_msgs::msg::RobotCall::_receiver_type arg)
  {
    msg_.receiver = std::move(arg);
    return Init_RobotCall_items(msg_);
  }

private:
  ::rio_ui_msgs::msg::RobotCall msg_;
};

class Init_RobotCall_destination
{
public:
  explicit Init_RobotCall_destination(::rio_ui_msgs::msg::RobotCall & msg)
  : msg_(msg)
  {}
  Init_RobotCall_receiver destination(::rio_ui_msgs::msg::RobotCall::_destination_type arg)
  {
    msg_.destination = std::move(arg);
    return Init_RobotCall_receiver(msg_);
  }

private:
  ::rio_ui_msgs::msg::RobotCall msg_;
};

class Init_RobotCall_robot_type
{
public:
  explicit Init_RobotCall_robot_type(::rio_ui_msgs::msg::RobotCall & msg)
  : msg_(msg)
  {}
  Init_RobotCall_destination robot_type(::rio_ui_msgs::msg::RobotCall::_robot_type_type arg)
  {
    msg_.robot_type = std::move(arg);
    return Init_RobotCall_destination(msg_);
  }

private:
  ::rio_ui_msgs::msg::RobotCall msg_;
};

class Init_RobotCall_date
{
public:
  explicit Init_RobotCall_date(::rio_ui_msgs::msg::RobotCall & msg)
  : msg_(msg)
  {}
  Init_RobotCall_robot_type date(::rio_ui_msgs::msg::RobotCall::_date_type arg)
  {
    msg_.date = std::move(arg);
    return Init_RobotCall_robot_type(msg_);
  }

private:
  ::rio_ui_msgs::msg::RobotCall msg_;
};

class Init_RobotCall_office_number
{
public:
  Init_RobotCall_office_number()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotCall_date office_number(::rio_ui_msgs::msg::RobotCall::_office_number_type arg)
  {
    msg_.office_number = std::move(arg);
    return Init_RobotCall_date(msg_);
  }

private:
  ::rio_ui_msgs::msg::RobotCall msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rio_ui_msgs::msg::RobotCall>()
{
  return rio_ui_msgs::msg::builder::Init_RobotCall_office_number();
}

}  // namespace rio_ui_msgs

#endif  // RIO_UI_MSGS__MSG__DETAIL__ROBOT_CALL__BUILDER_HPP_
