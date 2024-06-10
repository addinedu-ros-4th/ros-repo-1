// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rio_ui_msgs:msg/RobotCall.idl
// generated code does not contain a copyright notice

#ifndef RIO_UI_MSGS__MSG__DETAIL__ROBOT_CALL__STRUCT_H_
#define RIO_UI_MSGS__MSG__DETAIL__ROBOT_CALL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot_type'
// Member 'destination'
// Member 'receiver'
// Member 'items'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/RobotCall in the package rio_ui_msgs.
typedef struct rio_ui_msgs__msg__RobotCall
{
  int32_t office_number;
  int32_t date;
  rosidl_runtime_c__String robot_type;
  rosidl_runtime_c__String destination;
  rosidl_runtime_c__String receiver;
  rosidl_runtime_c__String items;
} rio_ui_msgs__msg__RobotCall;

// Struct for a sequence of rio_ui_msgs__msg__RobotCall.
typedef struct rio_ui_msgs__msg__RobotCall__Sequence
{
  rio_ui_msgs__msg__RobotCall * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rio_ui_msgs__msg__RobotCall__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RIO_UI_MSGS__MSG__DETAIL__ROBOT_CALL__STRUCT_H_
