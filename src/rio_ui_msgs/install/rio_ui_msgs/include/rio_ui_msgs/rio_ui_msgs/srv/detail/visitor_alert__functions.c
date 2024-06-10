// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rio_ui_msgs:srv/VisitorAlert.idl
// generated code does not contain a copyright notice
#include "rio_ui_msgs/srv/detail/visitor_alert__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `name`
// Member `affiliation`
// Member `visit_place`
#include "rosidl_runtime_c/string_functions.h"

bool
rio_ui_msgs__srv__VisitorAlert_Request__init(rio_ui_msgs__srv__VisitorAlert_Request * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    rio_ui_msgs__srv__VisitorAlert_Request__fini(msg);
    return false;
  }
  // affiliation
  if (!rosidl_runtime_c__String__init(&msg->affiliation)) {
    rio_ui_msgs__srv__VisitorAlert_Request__fini(msg);
    return false;
  }
  // visit_place
  if (!rosidl_runtime_c__String__init(&msg->visit_place)) {
    rio_ui_msgs__srv__VisitorAlert_Request__fini(msg);
    return false;
  }
  // robot_guidance
  return true;
}

void
rio_ui_msgs__srv__VisitorAlert_Request__fini(rio_ui_msgs__srv__VisitorAlert_Request * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // affiliation
  rosidl_runtime_c__String__fini(&msg->affiliation);
  // visit_place
  rosidl_runtime_c__String__fini(&msg->visit_place);
  // robot_guidance
}

bool
rio_ui_msgs__srv__VisitorAlert_Request__are_equal(const rio_ui_msgs__srv__VisitorAlert_Request * lhs, const rio_ui_msgs__srv__VisitorAlert_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // affiliation
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->affiliation), &(rhs->affiliation)))
  {
    return false;
  }
  // visit_place
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->visit_place), &(rhs->visit_place)))
  {
    return false;
  }
  // robot_guidance
  if (lhs->robot_guidance != rhs->robot_guidance) {
    return false;
  }
  return true;
}

bool
rio_ui_msgs__srv__VisitorAlert_Request__copy(
  const rio_ui_msgs__srv__VisitorAlert_Request * input,
  rio_ui_msgs__srv__VisitorAlert_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // affiliation
  if (!rosidl_runtime_c__String__copy(
      &(input->affiliation), &(output->affiliation)))
  {
    return false;
  }
  // visit_place
  if (!rosidl_runtime_c__String__copy(
      &(input->visit_place), &(output->visit_place)))
  {
    return false;
  }
  // robot_guidance
  output->robot_guidance = input->robot_guidance;
  return true;
}

rio_ui_msgs__srv__VisitorAlert_Request *
rio_ui_msgs__srv__VisitorAlert_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__srv__VisitorAlert_Request * msg = (rio_ui_msgs__srv__VisitorAlert_Request *)allocator.allocate(sizeof(rio_ui_msgs__srv__VisitorAlert_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rio_ui_msgs__srv__VisitorAlert_Request));
  bool success = rio_ui_msgs__srv__VisitorAlert_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rio_ui_msgs__srv__VisitorAlert_Request__destroy(rio_ui_msgs__srv__VisitorAlert_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rio_ui_msgs__srv__VisitorAlert_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rio_ui_msgs__srv__VisitorAlert_Request__Sequence__init(rio_ui_msgs__srv__VisitorAlert_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__srv__VisitorAlert_Request * data = NULL;

  if (size) {
    data = (rio_ui_msgs__srv__VisitorAlert_Request *)allocator.zero_allocate(size, sizeof(rio_ui_msgs__srv__VisitorAlert_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rio_ui_msgs__srv__VisitorAlert_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rio_ui_msgs__srv__VisitorAlert_Request__fini(&data[i - 1]);
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
rio_ui_msgs__srv__VisitorAlert_Request__Sequence__fini(rio_ui_msgs__srv__VisitorAlert_Request__Sequence * array)
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
      rio_ui_msgs__srv__VisitorAlert_Request__fini(&array->data[i]);
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

rio_ui_msgs__srv__VisitorAlert_Request__Sequence *
rio_ui_msgs__srv__VisitorAlert_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__srv__VisitorAlert_Request__Sequence * array = (rio_ui_msgs__srv__VisitorAlert_Request__Sequence *)allocator.allocate(sizeof(rio_ui_msgs__srv__VisitorAlert_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rio_ui_msgs__srv__VisitorAlert_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rio_ui_msgs__srv__VisitorAlert_Request__Sequence__destroy(rio_ui_msgs__srv__VisitorAlert_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rio_ui_msgs__srv__VisitorAlert_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rio_ui_msgs__srv__VisitorAlert_Request__Sequence__are_equal(const rio_ui_msgs__srv__VisitorAlert_Request__Sequence * lhs, const rio_ui_msgs__srv__VisitorAlert_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rio_ui_msgs__srv__VisitorAlert_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rio_ui_msgs__srv__VisitorAlert_Request__Sequence__copy(
  const rio_ui_msgs__srv__VisitorAlert_Request__Sequence * input,
  rio_ui_msgs__srv__VisitorAlert_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rio_ui_msgs__srv__VisitorAlert_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rio_ui_msgs__srv__VisitorAlert_Request * data =
      (rio_ui_msgs__srv__VisitorAlert_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rio_ui_msgs__srv__VisitorAlert_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rio_ui_msgs__srv__VisitorAlert_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rio_ui_msgs__srv__VisitorAlert_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
rio_ui_msgs__srv__VisitorAlert_Response__init(rio_ui_msgs__srv__VisitorAlert_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
rio_ui_msgs__srv__VisitorAlert_Response__fini(rio_ui_msgs__srv__VisitorAlert_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
rio_ui_msgs__srv__VisitorAlert_Response__are_equal(const rio_ui_msgs__srv__VisitorAlert_Response * lhs, const rio_ui_msgs__srv__VisitorAlert_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
rio_ui_msgs__srv__VisitorAlert_Response__copy(
  const rio_ui_msgs__srv__VisitorAlert_Response * input,
  rio_ui_msgs__srv__VisitorAlert_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

rio_ui_msgs__srv__VisitorAlert_Response *
rio_ui_msgs__srv__VisitorAlert_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__srv__VisitorAlert_Response * msg = (rio_ui_msgs__srv__VisitorAlert_Response *)allocator.allocate(sizeof(rio_ui_msgs__srv__VisitorAlert_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rio_ui_msgs__srv__VisitorAlert_Response));
  bool success = rio_ui_msgs__srv__VisitorAlert_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rio_ui_msgs__srv__VisitorAlert_Response__destroy(rio_ui_msgs__srv__VisitorAlert_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rio_ui_msgs__srv__VisitorAlert_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rio_ui_msgs__srv__VisitorAlert_Response__Sequence__init(rio_ui_msgs__srv__VisitorAlert_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__srv__VisitorAlert_Response * data = NULL;

  if (size) {
    data = (rio_ui_msgs__srv__VisitorAlert_Response *)allocator.zero_allocate(size, sizeof(rio_ui_msgs__srv__VisitorAlert_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rio_ui_msgs__srv__VisitorAlert_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rio_ui_msgs__srv__VisitorAlert_Response__fini(&data[i - 1]);
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
rio_ui_msgs__srv__VisitorAlert_Response__Sequence__fini(rio_ui_msgs__srv__VisitorAlert_Response__Sequence * array)
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
      rio_ui_msgs__srv__VisitorAlert_Response__fini(&array->data[i]);
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

rio_ui_msgs__srv__VisitorAlert_Response__Sequence *
rio_ui_msgs__srv__VisitorAlert_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__srv__VisitorAlert_Response__Sequence * array = (rio_ui_msgs__srv__VisitorAlert_Response__Sequence *)allocator.allocate(sizeof(rio_ui_msgs__srv__VisitorAlert_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rio_ui_msgs__srv__VisitorAlert_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rio_ui_msgs__srv__VisitorAlert_Response__Sequence__destroy(rio_ui_msgs__srv__VisitorAlert_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rio_ui_msgs__srv__VisitorAlert_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rio_ui_msgs__srv__VisitorAlert_Response__Sequence__are_equal(const rio_ui_msgs__srv__VisitorAlert_Response__Sequence * lhs, const rio_ui_msgs__srv__VisitorAlert_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rio_ui_msgs__srv__VisitorAlert_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rio_ui_msgs__srv__VisitorAlert_Response__Sequence__copy(
  const rio_ui_msgs__srv__VisitorAlert_Response__Sequence * input,
  rio_ui_msgs__srv__VisitorAlert_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rio_ui_msgs__srv__VisitorAlert_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rio_ui_msgs__srv__VisitorAlert_Response * data =
      (rio_ui_msgs__srv__VisitorAlert_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rio_ui_msgs__srv__VisitorAlert_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rio_ui_msgs__srv__VisitorAlert_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rio_ui_msgs__srv__VisitorAlert_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
