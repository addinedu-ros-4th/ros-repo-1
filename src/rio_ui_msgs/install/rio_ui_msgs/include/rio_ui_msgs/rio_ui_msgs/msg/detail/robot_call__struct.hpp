// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rio_ui_msgs:msg/RobotCall.idl
// generated code does not contain a copyright notice

#ifndef RIO_UI_MSGS__MSG__DETAIL__ROBOT_CALL__STRUCT_HPP_
#define RIO_UI_MSGS__MSG__DETAIL__ROBOT_CALL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rio_ui_msgs__msg__RobotCall __attribute__((deprecated))
#else
# define DEPRECATED__rio_ui_msgs__msg__RobotCall __declspec(deprecated)
#endif

namespace rio_ui_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotCall_
{
  using Type = RobotCall_<ContainerAllocator>;

  explicit RobotCall_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->office_number = 0l;
      this->date = 0l;
      this->robot_type = "";
      this->destination = "";
      this->receiver = "";
      this->items = "";
    }
  }

  explicit RobotCall_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : robot_type(_alloc),
    destination(_alloc),
    receiver(_alloc),
    items(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->office_number = 0l;
      this->date = 0l;
      this->robot_type = "";
      this->destination = "";
      this->receiver = "";
      this->items = "";
    }
  }

  // field types and members
  using _office_number_type =
    int32_t;
  _office_number_type office_number;
  using _date_type =
    int32_t;
  _date_type date;
  using _robot_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_type_type robot_type;
  using _destination_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _destination_type destination;
  using _receiver_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _receiver_type receiver;
  using _items_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _items_type items;

  // setters for named parameter idiom
  Type & set__office_number(
    const int32_t & _arg)
  {
    this->office_number = _arg;
    return *this;
  }
  Type & set__date(
    const int32_t & _arg)
  {
    this->date = _arg;
    return *this;
  }
  Type & set__robot_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_type = _arg;
    return *this;
  }
  Type & set__destination(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->destination = _arg;
    return *this;
  }
  Type & set__receiver(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->receiver = _arg;
    return *this;
  }
  Type & set__items(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->items = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rio_ui_msgs::msg::RobotCall_<ContainerAllocator> *;
  using ConstRawPtr =
    const rio_ui_msgs::msg::RobotCall_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rio_ui_msgs::msg::RobotCall_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rio_ui_msgs::msg::RobotCall_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rio_ui_msgs::msg::RobotCall_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rio_ui_msgs::msg::RobotCall_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rio_ui_msgs::msg::RobotCall_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rio_ui_msgs::msg::RobotCall_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rio_ui_msgs::msg::RobotCall_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rio_ui_msgs::msg::RobotCall_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rio_ui_msgs__msg__RobotCall
    std::shared_ptr<rio_ui_msgs::msg::RobotCall_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rio_ui_msgs__msg__RobotCall
    std::shared_ptr<rio_ui_msgs::msg::RobotCall_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotCall_ & other) const
  {
    if (this->office_number != other.office_number) {
      return false;
    }
    if (this->date != other.date) {
      return false;
    }
    if (this->robot_type != other.robot_type) {
      return false;
    }
    if (this->destination != other.destination) {
      return false;
    }
    if (this->receiver != other.receiver) {
      return false;
    }
    if (this->items != other.items) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotCall_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotCall_

// alias to use template instance with default allocator
using RobotCall =
  rio_ui_msgs::msg::RobotCall_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rio_ui_msgs

#endif  // RIO_UI_MSGS__MSG__DETAIL__ROBOT_CALL__STRUCT_HPP_
