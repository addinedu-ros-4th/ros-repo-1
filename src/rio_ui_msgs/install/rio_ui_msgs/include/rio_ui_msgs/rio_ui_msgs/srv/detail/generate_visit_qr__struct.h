// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rio_ui_msgs:srv/GenerateVisitQR.idl
// generated code does not contain a copyright notice

#ifndef RIO_UI_MSGS__SRV__DETAIL__GENERATE_VISIT_QR__STRUCT_H_
#define RIO_UI_MSGS__SRV__DETAIL__GENERATE_VISIT_QR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'visitor_info'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GenerateVisitQR in the package rio_ui_msgs.
typedef struct rio_ui_msgs__srv__GenerateVisitQR_Request
{
  rosidl_runtime_c__String visitor_info;
} rio_ui_msgs__srv__GenerateVisitQR_Request;

// Struct for a sequence of rio_ui_msgs__srv__GenerateVisitQR_Request.
typedef struct rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence
{
  rio_ui_msgs__srv__GenerateVisitQR_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// Member 'qr_code_path'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GenerateVisitQR in the package rio_ui_msgs.
typedef struct rio_ui_msgs__srv__GenerateVisitQR_Response
{
  bool success;
  rosidl_runtime_c__String message;
  rosidl_runtime_c__String qr_code_path;
} rio_ui_msgs__srv__GenerateVisitQR_Response;

// Struct for a sequence of rio_ui_msgs__srv__GenerateVisitQR_Response.
typedef struct rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence
{
  rio_ui_msgs__srv__GenerateVisitQR_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RIO_UI_MSGS__SRV__DETAIL__GENERATE_VISIT_QR__STRUCT_H_
