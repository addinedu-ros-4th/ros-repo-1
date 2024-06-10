// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rio_ui_msgs:srv/GenerateVisitQR.idl
// generated code does not contain a copyright notice
#include "rio_ui_msgs/srv/detail/generate_visit_qr__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `visitor_info`
#include "rosidl_runtime_c/string_functions.h"

bool
rio_ui_msgs__srv__GenerateVisitQR_Request__init(rio_ui_msgs__srv__GenerateVisitQR_Request * msg)
{
  if (!msg) {
    return false;
  }
  // visitor_info
  if (!rosidl_runtime_c__String__init(&msg->visitor_info)) {
    rio_ui_msgs__srv__GenerateVisitQR_Request__fini(msg);
    return false;
  }
  return true;
}

void
rio_ui_msgs__srv__GenerateVisitQR_Request__fini(rio_ui_msgs__srv__GenerateVisitQR_Request * msg)
{
  if (!msg) {
    return;
  }
  // visitor_info
  rosidl_runtime_c__String__fini(&msg->visitor_info);
}

bool
rio_ui_msgs__srv__GenerateVisitQR_Request__are_equal(const rio_ui_msgs__srv__GenerateVisitQR_Request * lhs, const rio_ui_msgs__srv__GenerateVisitQR_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // visitor_info
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->visitor_info), &(rhs->visitor_info)))
  {
    return false;
  }
  return true;
}

bool
rio_ui_msgs__srv__GenerateVisitQR_Request__copy(
  const rio_ui_msgs__srv__GenerateVisitQR_Request * input,
  rio_ui_msgs__srv__GenerateVisitQR_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // visitor_info
  if (!rosidl_runtime_c__String__copy(
      &(input->visitor_info), &(output->visitor_info)))
  {
    return false;
  }
  return true;
}

rio_ui_msgs__srv__GenerateVisitQR_Request *
rio_ui_msgs__srv__GenerateVisitQR_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__srv__GenerateVisitQR_Request * msg = (rio_ui_msgs__srv__GenerateVisitQR_Request *)allocator.allocate(sizeof(rio_ui_msgs__srv__GenerateVisitQR_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rio_ui_msgs__srv__GenerateVisitQR_Request));
  bool success = rio_ui_msgs__srv__GenerateVisitQR_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rio_ui_msgs__srv__GenerateVisitQR_Request__destroy(rio_ui_msgs__srv__GenerateVisitQR_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rio_ui_msgs__srv__GenerateVisitQR_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence__init(rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__srv__GenerateVisitQR_Request * data = NULL;

  if (size) {
    data = (rio_ui_msgs__srv__GenerateVisitQR_Request *)allocator.zero_allocate(size, sizeof(rio_ui_msgs__srv__GenerateVisitQR_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rio_ui_msgs__srv__GenerateVisitQR_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rio_ui_msgs__srv__GenerateVisitQR_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence__fini(rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rio_ui_msgs__srv__GenerateVisitQR_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence *
rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence * array = (rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence *)allocator.allocate(sizeof(rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence__destroy(rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence__are_equal(const rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence * lhs, const rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rio_ui_msgs__srv__GenerateVisitQR_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence__copy(
  const rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence * input,
  rio_ui_msgs__srv__GenerateVisitQR_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rio_ui_msgs__srv__GenerateVisitQR_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rio_ui_msgs__srv__GenerateVisitQR_Request * data =
      (rio_ui_msgs__srv__GenerateVisitQR_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rio_ui_msgs__srv__GenerateVisitQR_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rio_ui_msgs__srv__GenerateVisitQR_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rio_ui_msgs__srv__GenerateVisitQR_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// Member `qr_code_path`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
rio_ui_msgs__srv__GenerateVisitQR_Response__init(rio_ui_msgs__srv__GenerateVisitQR_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    rio_ui_msgs__srv__GenerateVisitQR_Response__fini(msg);
    return false;
  }
  // qr_code_path
  if (!rosidl_runtime_c__String__init(&msg->qr_code_path)) {
    rio_ui_msgs__srv__GenerateVisitQR_Response__fini(msg);
    return false;
  }
  return true;
}

void
rio_ui_msgs__srv__GenerateVisitQR_Response__fini(rio_ui_msgs__srv__GenerateVisitQR_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // qr_code_path
  rosidl_runtime_c__String__fini(&msg->qr_code_path);
}

bool
rio_ui_msgs__srv__GenerateVisitQR_Response__are_equal(const rio_ui_msgs__srv__GenerateVisitQR_Response * lhs, const rio_ui_msgs__srv__GenerateVisitQR_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  // qr_code_path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->qr_code_path), &(rhs->qr_code_path)))
  {
    return false;
  }
  return true;
}

bool
rio_ui_msgs__srv__GenerateVisitQR_Response__copy(
  const rio_ui_msgs__srv__GenerateVisitQR_Response * input,
  rio_ui_msgs__srv__GenerateVisitQR_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  // qr_code_path
  if (!rosidl_runtime_c__String__copy(
      &(input->qr_code_path), &(output->qr_code_path)))
  {
    return false;
  }
  return true;
}

rio_ui_msgs__srv__GenerateVisitQR_Response *
rio_ui_msgs__srv__GenerateVisitQR_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__srv__GenerateVisitQR_Response * msg = (rio_ui_msgs__srv__GenerateVisitQR_Response *)allocator.allocate(sizeof(rio_ui_msgs__srv__GenerateVisitQR_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rio_ui_msgs__srv__GenerateVisitQR_Response));
  bool success = rio_ui_msgs__srv__GenerateVisitQR_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rio_ui_msgs__srv__GenerateVisitQR_Response__destroy(rio_ui_msgs__srv__GenerateVisitQR_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rio_ui_msgs__srv__GenerateVisitQR_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence__init(rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__srv__GenerateVisitQR_Response * data = NULL;

  if (size) {
    data = (rio_ui_msgs__srv__GenerateVisitQR_Response *)allocator.zero_allocate(size, sizeof(rio_ui_msgs__srv__GenerateVisitQR_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rio_ui_msgs__srv__GenerateVisitQR_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rio_ui_msgs__srv__GenerateVisitQR_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence__fini(rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rio_ui_msgs__srv__GenerateVisitQR_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence *
rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence * array = (rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence *)allocator.allocate(sizeof(rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence__destroy(rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence__are_equal(const rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence * lhs, const rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rio_ui_msgs__srv__GenerateVisitQR_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence__copy(
  const rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence * input,
  rio_ui_msgs__srv__GenerateVisitQR_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rio_ui_msgs__srv__GenerateVisitQR_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rio_ui_msgs__srv__GenerateVisitQR_Response * data =
      (rio_ui_msgs__srv__GenerateVisitQR_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rio_ui_msgs__srv__GenerateVisitQR_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rio_ui_msgs__srv__GenerateVisitQR_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rio_ui_msgs__srv__GenerateVisitQR_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
