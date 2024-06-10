// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rio_ui_msgs:srv/GenerateVisitQR.idl
// generated code does not contain a copyright notice

#ifndef RIO_UI_MSGS__SRV__DETAIL__GENERATE_VISIT_QR__STRUCT_HPP_
#define RIO_UI_MSGS__SRV__DETAIL__GENERATE_VISIT_QR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rio_ui_msgs__srv__GenerateVisitQR_Request __attribute__((deprecated))
#else
# define DEPRECATED__rio_ui_msgs__srv__GenerateVisitQR_Request __declspec(deprecated)
#endif

namespace rio_ui_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GenerateVisitQR_Request_
{
  using Type = GenerateVisitQR_Request_<ContainerAllocator>;

  explicit GenerateVisitQR_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->visitor_info = "";
    }
  }

  explicit GenerateVisitQR_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : visitor_info(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->visitor_info = "";
    }
  }

  // field types and members
  using _visitor_info_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _visitor_info_type visitor_info;

  // setters for named parameter idiom
  Type & set__visitor_info(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->visitor_info = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rio_ui_msgs::srv::GenerateVisitQR_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rio_ui_msgs::srv::GenerateVisitQR_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rio_ui_msgs::srv::GenerateVisitQR_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rio_ui_msgs::srv::GenerateVisitQR_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rio_ui_msgs::srv::GenerateVisitQR_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rio_ui_msgs::srv::GenerateVisitQR_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rio_ui_msgs::srv::GenerateVisitQR_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rio_ui_msgs::srv::GenerateVisitQR_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rio_ui_msgs::srv::GenerateVisitQR_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rio_ui_msgs::srv::GenerateVisitQR_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rio_ui_msgs__srv__GenerateVisitQR_Request
    std::shared_ptr<rio_ui_msgs::srv::GenerateVisitQR_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rio_ui_msgs__srv__GenerateVisitQR_Request
    std::shared_ptr<rio_ui_msgs::srv::GenerateVisitQR_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GenerateVisitQR_Request_ & other) const
  {
    if (this->visitor_info != other.visitor_info) {
      return false;
    }
    return true;
  }
  bool operator!=(const GenerateVisitQR_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GenerateVisitQR_Request_

// alias to use template instance with default allocator
using GenerateVisitQR_Request =
  rio_ui_msgs::srv::GenerateVisitQR_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rio_ui_msgs


#ifndef _WIN32
# define DEPRECATED__rio_ui_msgs__srv__GenerateVisitQR_Response __attribute__((deprecated))
#else
# define DEPRECATED__rio_ui_msgs__srv__GenerateVisitQR_Response __declspec(deprecated)
#endif

namespace rio_ui_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GenerateVisitQR_Response_
{
  using Type = GenerateVisitQR_Response_<ContainerAllocator>;

  explicit GenerateVisitQR_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->qr_code_path = "";
    }
  }

  explicit GenerateVisitQR_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    qr_code_path(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->qr_code_path = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _qr_code_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _qr_code_path_type qr_code_path;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__qr_code_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->qr_code_path = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rio_ui_msgs::srv::GenerateVisitQR_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rio_ui_msgs::srv::GenerateVisitQR_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rio_ui_msgs::srv::GenerateVisitQR_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rio_ui_msgs::srv::GenerateVisitQR_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rio_ui_msgs::srv::GenerateVisitQR_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rio_ui_msgs::srv::GenerateVisitQR_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rio_ui_msgs::srv::GenerateVisitQR_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rio_ui_msgs::srv::GenerateVisitQR_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rio_ui_msgs::srv::GenerateVisitQR_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rio_ui_msgs::srv::GenerateVisitQR_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rio_ui_msgs__srv__GenerateVisitQR_Response
    std::shared_ptr<rio_ui_msgs::srv::GenerateVisitQR_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rio_ui_msgs__srv__GenerateVisitQR_Response
    std::shared_ptr<rio_ui_msgs::srv::GenerateVisitQR_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GenerateVisitQR_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->qr_code_path != other.qr_code_path) {
      return false;
    }
    return true;
  }
  bool operator!=(const GenerateVisitQR_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GenerateVisitQR_Response_

// alias to use template instance with default allocator
using GenerateVisitQR_Response =
  rio_ui_msgs::srv::GenerateVisitQR_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rio_ui_msgs

namespace rio_ui_msgs
{

namespace srv
{

struct GenerateVisitQR
{
  using Request = rio_ui_msgs::srv::GenerateVisitQR_Request;
  using Response = rio_ui_msgs::srv::GenerateVisitQR_Response;
};

}  // namespace srv

}  // namespace rio_ui_msgs

#endif  // RIO_UI_MSGS__SRV__DETAIL__GENERATE_VISIT_QR__STRUCT_HPP_
