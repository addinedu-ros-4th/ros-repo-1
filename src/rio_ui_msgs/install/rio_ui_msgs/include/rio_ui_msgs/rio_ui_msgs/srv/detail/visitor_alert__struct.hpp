// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rio_ui_msgs:srv/VisitorAlert.idl
// generated code does not contain a copyright notice

#ifndef RIO_UI_MSGS__SRV__DETAIL__VISITOR_ALERT__STRUCT_HPP_
#define RIO_UI_MSGS__SRV__DETAIL__VISITOR_ALERT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rio_ui_msgs__srv__VisitorAlert_Request __attribute__((deprecated))
#else
# define DEPRECATED__rio_ui_msgs__srv__VisitorAlert_Request __declspec(deprecated)
#endif

namespace rio_ui_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct VisitorAlert_Request_
{
  using Type = VisitorAlert_Request_<ContainerAllocator>;

  explicit VisitorAlert_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->affiliation = "";
      this->visit_place = "";
      this->robot_guidance = false;
    }
  }

  explicit VisitorAlert_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc),
    affiliation(_alloc),
    visit_place(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->affiliation = "";
      this->visit_place = "";
      this->robot_guidance = false;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _affiliation_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _affiliation_type affiliation;
  using _visit_place_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _visit_place_type visit_place;
  using _robot_guidance_type =
    bool;
  _robot_guidance_type robot_guidance;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__affiliation(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->affiliation = _arg;
    return *this;
  }
  Type & set__visit_place(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->visit_place = _arg;
    return *this;
  }
  Type & set__robot_guidance(
    const bool & _arg)
  {
    this->robot_guidance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rio_ui_msgs::srv::VisitorAlert_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rio_ui_msgs::srv::VisitorAlert_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rio_ui_msgs::srv::VisitorAlert_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rio_ui_msgs::srv::VisitorAlert_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rio_ui_msgs::srv::VisitorAlert_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rio_ui_msgs::srv::VisitorAlert_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rio_ui_msgs::srv::VisitorAlert_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rio_ui_msgs::srv::VisitorAlert_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rio_ui_msgs::srv::VisitorAlert_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rio_ui_msgs::srv::VisitorAlert_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rio_ui_msgs__srv__VisitorAlert_Request
    std::shared_ptr<rio_ui_msgs::srv::VisitorAlert_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rio_ui_msgs__srv__VisitorAlert_Request
    std::shared_ptr<rio_ui_msgs::srv::VisitorAlert_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VisitorAlert_Request_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->affiliation != other.affiliation) {
      return false;
    }
    if (this->visit_place != other.visit_place) {
      return false;
    }
    if (this->robot_guidance != other.robot_guidance) {
      return false;
    }
    return true;
  }
  bool operator!=(const VisitorAlert_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VisitorAlert_Request_

// alias to use template instance with default allocator
using VisitorAlert_Request =
  rio_ui_msgs::srv::VisitorAlert_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rio_ui_msgs


#ifndef _WIN32
# define DEPRECATED__rio_ui_msgs__srv__VisitorAlert_Response __attribute__((deprecated))
#else
# define DEPRECATED__rio_ui_msgs__srv__VisitorAlert_Response __declspec(deprecated)
#endif

namespace rio_ui_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct VisitorAlert_Response_
{
  using Type = VisitorAlert_Response_<ContainerAllocator>;

  explicit VisitorAlert_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit VisitorAlert_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    rio_ui_msgs::srv::VisitorAlert_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rio_ui_msgs::srv::VisitorAlert_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rio_ui_msgs::srv::VisitorAlert_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rio_ui_msgs::srv::VisitorAlert_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rio_ui_msgs::srv::VisitorAlert_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rio_ui_msgs::srv::VisitorAlert_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rio_ui_msgs::srv::VisitorAlert_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rio_ui_msgs::srv::VisitorAlert_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rio_ui_msgs::srv::VisitorAlert_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rio_ui_msgs::srv::VisitorAlert_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rio_ui_msgs__srv__VisitorAlert_Response
    std::shared_ptr<rio_ui_msgs::srv::VisitorAlert_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rio_ui_msgs__srv__VisitorAlert_Response
    std::shared_ptr<rio_ui_msgs::srv::VisitorAlert_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VisitorAlert_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const VisitorAlert_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VisitorAlert_Response_

// alias to use template instance with default allocator
using VisitorAlert_Response =
  rio_ui_msgs::srv::VisitorAlert_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rio_ui_msgs

namespace rio_ui_msgs
{

namespace srv
{

struct VisitorAlert
{
  using Request = rio_ui_msgs::srv::VisitorAlert_Request;
  using Response = rio_ui_msgs::srv::VisitorAlert_Response;
};

}  // namespace srv

}  // namespace rio_ui_msgs

#endif  // RIO_UI_MSGS__SRV__DETAIL__VISITOR_ALERT__STRUCT_HPP_
