// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rio_ui_msgs:srv/VisitorAlert.idl
// generated code does not contain a copyright notice

#ifndef RIO_UI_MSGS__SRV__DETAIL__VISITOR_ALERT__STRUCT_H_
#define RIO_UI_MSGS__SRV__DETAIL__VISITOR_ALERT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
// Member 'affiliation'
// Member 'visit_place'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/VisitorAlert in the package rio_ui_msgs.
typedef struct rio_ui_msgs__srv__VisitorAlert_Request
{
  rosidl_runtime_c__String name;
  rosidl_runtime_c__String affiliation;
  rosidl_runtime_c__String visit_place;
  bool robot_guidance;
} rio_ui_msgs__srv__VisitorAlert_Request;

// Struct for a sequence of rio_ui_msgs__srv__VisitorAlert_Request.
typedef struct rio_ui_msgs__srv__VisitorAlert_Request__Sequence
{
  rio_ui_msgs__srv__VisitorAlert_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rio_ui_msgs__srv__VisitorAlert_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/VisitorAlert in the package rio_ui_msgs.
typedef struct rio_ui_msgs__srv__VisitorAlert_Response
{
  bool success;
} rio_ui_msgs__srv__VisitorAlert_Response;

// Struct for a sequence of rio_ui_msgs__srv__VisitorAlert_Response.
typedef struct rio_ui_msgs__srv__VisitorAlert_Response__Sequence
{
  rio_ui_msgs__srv__VisitorAlert_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rio_ui_msgs__srv__VisitorAlert_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RIO_UI_MSGS__SRV__DETAIL__VISITOR_ALERT__STRUCT_H_
