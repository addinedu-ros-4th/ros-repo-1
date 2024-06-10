// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rio_ui_msgs:srv/QRCheck.idl
// generated code does not contain a copyright notice
#include "rio_ui_msgs/srv/detail/qr_check__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `hashed_data`
#include "rosidl_runtime_c/string_functions.h"

bool
rio_ui_msgs__srv__QRCheck_Request__init(rio_ui_msgs__srv__QRCheck_Request * msg)
{
  if (!msg) {
    return false;
  }
  // hashed_data
  if (!rosidl_runtime_c__String__init(&msg->hashed_data)) {
    rio_ui_msgs__srv__QRCheck_Request__fini(msg);
    return false;
  }
  return true;
}

void
rio_ui_msgs__srv__QRCheck_Request__fini(rio_ui_msgs__srv__QRCheck_Request * msg)
{
  if (!msg) {
    return;
  }
  // hashed_data
  rosidl_runtime_c__String__fini(&msg->hashed_data);
}

bool
rio_ui_msgs__srv__QRCheck_Request__are_equal(const rio_ui_msgs__srv__QRCheck_Request * lhs, const rio_ui_msgs__srv__QRCheck_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // hashed_data
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->hashed_data), &(rhs->hashed_data)))
  {
    return false;
  }
  return true;
}

bool
rio_ui_msgs__srv__QRCheck_Request__copy(
  const rio_ui_msgs__srv__QRCheck_Request * input,
  rio_ui_msgs__srv__QRCheck_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // hashed_data
  if (!rosidl_runtime_c__String__copy(
      &(input->hashed_data), &(output->hashed_data)))
  {
    return false;
  }
  return true;
}

rio_ui_msgs__srv__QRCheck_Request *
rio_ui_msgs__srv__QRCheck_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__srv__QRCheck_Request * msg = (rio_ui_msgs__srv__QRCheck_Request *)allocator.allocate(sizeof(rio_ui_msgs__srv__QRCheck_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rio_ui_msgs__srv__QRCheck_Request));
  bool success = rio_ui_msgs__srv__QRCheck_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rio_ui_msgs__srv__QRCheck_Request__destroy(rio_ui_msgs__srv__QRCheck_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rio_ui_msgs__srv__QRCheck_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rio_ui_msgs__srv__QRCheck_Request__Sequence__init(rio_ui_msgs__srv__QRCheck_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__srv__QRCheck_Request * data = NULL;

  if (size) {
    data = (rio_ui_msgs__srv__QRCheck_Request *)allocator.zero_allocate(size, sizeof(rio_ui_msgs__srv__QRCheck_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rio_ui_msgs__srv__QRCheck_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rio_ui_msgs__srv__QRCheck_Request__fini(&data[i - 1]);
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
rio_ui_msgs__srv__QRCheck_Request__Sequence__fini(rio_ui_msgs__srv__QRCheck_Request__Sequence * array)
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
      rio_ui_msgs__srv__QRCheck_Request__fini(&array->data[i]);
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

rio_ui_msgs__srv__QRCheck_Request__Sequence *
rio_ui_msgs__srv__QRCheck_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__srv__QRCheck_Request__Sequence * array = (rio_ui_msgs__srv__QRCheck_Request__Sequence *)allocator.allocate(sizeof(rio_ui_msgs__srv__QRCheck_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rio_ui_msgs__srv__QRCheck_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rio_ui_msgs__srv__QRCheck_Request__Sequence__destroy(rio_ui_msgs__srv__QRCheck_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rio_ui_msgs__srv__QRCheck_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rio_ui_msgs__srv__QRCheck_Request__Sequence__are_equal(const rio_ui_msgs__srv__QRCheck_Request__Sequence * lhs, const rio_ui_msgs__srv__QRCheck_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rio_ui_msgs__srv__QRCheck_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rio_ui_msgs__srv__QRCheck_Request__Sequence__copy(
  const rio_ui_msgs__srv__QRCheck_Request__Sequence * input,
  rio_ui_msgs__srv__QRCheck_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rio_ui_msgs__srv__QRCheck_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rio_ui_msgs__srv__QRCheck_Request * data =
      (rio_ui_msgs__srv__QRCheck_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rio_ui_msgs__srv__QRCheck_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rio_ui_msgs__srv__QRCheck_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rio_ui_msgs__srv__QRCheck_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
rio_ui_msgs__srv__QRCheck_Response__init(rio_ui_msgs__srv__QRCheck_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    rio_ui_msgs__srv__QRCheck_Response__fini(msg);
    return false;
  }
  return true;
}

void
rio_ui_msgs__srv__QRCheck_Response__fini(rio_ui_msgs__srv__QRCheck_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
rio_ui_msgs__srv__QRCheck_Response__are_equal(const rio_ui_msgs__srv__QRCheck_Response * lhs, const rio_ui_msgs__srv__QRCheck_Response * rhs)
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
  return true;
}

bool
rio_ui_msgs__srv__QRCheck_Response__copy(
  const rio_ui_msgs__srv__QRCheck_Response * input,
  rio_ui_msgs__srv__QRCheck_Response * output)
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
  return true;
}

rio_ui_msgs__srv__QRCheck_Response *
rio_ui_msgs__srv__QRCheck_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__srv__QRCheck_Response * msg = (rio_ui_msgs__srv__QRCheck_Response *)allocator.allocate(sizeof(rio_ui_msgs__srv__QRCheck_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rio_ui_msgs__srv__QRCheck_Response));
  bool success = rio_ui_msgs__srv__QRCheck_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rio_ui_msgs__srv__QRCheck_Response__destroy(rio_ui_msgs__srv__QRCheck_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rio_ui_msgs__srv__QRCheck_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rio_ui_msgs__srv__QRCheck_Response__Sequence__init(rio_ui_msgs__srv__QRCheck_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__srv__QRCheck_Response * data = NULL;

  if (size) {
    data = (rio_ui_msgs__srv__QRCheck_Response *)allocator.zero_allocate(size, sizeof(rio_ui_msgs__srv__QRCheck_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rio_ui_msgs__srv__QRCheck_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rio_ui_msgs__srv__QRCheck_Response__fini(&data[i - 1]);
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
rio_ui_msgs__srv__QRCheck_Response__Sequence__fini(rio_ui_msgs__srv__QRCheck_Response__Sequence * array)
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
      rio_ui_msgs__srv__QRCheck_Response__fini(&array->data[i]);
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

rio_ui_msgs__srv__QRCheck_Response__Sequence *
rio_ui_msgs__srv__QRCheck_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__srv__QRCheck_Response__Sequence * array = (rio_ui_msgs__srv__QRCheck_Response__Sequence *)allocator.allocate(sizeof(rio_ui_msgs__srv__QRCheck_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rio_ui_msgs__srv__QRCheck_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rio_ui_msgs__srv__QRCheck_Response__Sequence__destroy(rio_ui_msgs__srv__QRCheck_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rio_ui_msgs__srv__QRCheck_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rio_ui_msgs__srv__QRCheck_Response__Sequence__are_equal(const rio_ui_msgs__srv__QRCheck_Response__Sequence * lhs, const rio_ui_msgs__srv__QRCheck_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rio_ui_msgs__srv__QRCheck_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rio_ui_msgs__srv__QRCheck_Response__Sequence__copy(
  const rio_ui_msgs__srv__QRCheck_Response__Sequence * input,
  rio_ui_msgs__srv__QRCheck_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rio_ui_msgs__srv__QRCheck_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rio_ui_msgs__srv__QRCheck_Response * data =
      (rio_ui_msgs__srv__QRCheck_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rio_ui_msgs__srv__QRCheck_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rio_ui_msgs__srv__QRCheck_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rio_ui_msgs__srv__QRCheck_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
