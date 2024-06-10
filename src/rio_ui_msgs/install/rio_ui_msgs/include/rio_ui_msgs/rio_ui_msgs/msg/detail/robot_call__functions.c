// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rio_ui_msgs:msg/RobotCall.idl
// generated code does not contain a copyright notice
#include "rio_ui_msgs/msg/detail/robot_call__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `robot_type`
// Member `destination`
// Member `receiver`
// Member `items`
#include "rosidl_runtime_c/string_functions.h"

bool
rio_ui_msgs__msg__RobotCall__init(rio_ui_msgs__msg__RobotCall * msg)
{
  if (!msg) {
    return false;
  }
  // office_number
  // date
  // robot_type
  if (!rosidl_runtime_c__String__init(&msg->robot_type)) {
    rio_ui_msgs__msg__RobotCall__fini(msg);
    return false;
  }
  // destination
  if (!rosidl_runtime_c__String__init(&msg->destination)) {
    rio_ui_msgs__msg__RobotCall__fini(msg);
    return false;
  }
  // receiver
  if (!rosidl_runtime_c__String__init(&msg->receiver)) {
    rio_ui_msgs__msg__RobotCall__fini(msg);
    return false;
  }
  // items
  if (!rosidl_runtime_c__String__init(&msg->items)) {
    rio_ui_msgs__msg__RobotCall__fini(msg);
    return false;
  }
  return true;
}

void
rio_ui_msgs__msg__RobotCall__fini(rio_ui_msgs__msg__RobotCall * msg)
{
  if (!msg) {
    return;
  }
  // office_number
  // date
  // robot_type
  rosidl_runtime_c__String__fini(&msg->robot_type);
  // destination
  rosidl_runtime_c__String__fini(&msg->destination);
  // receiver
  rosidl_runtime_c__String__fini(&msg->receiver);
  // items
  rosidl_runtime_c__String__fini(&msg->items);
}

bool
rio_ui_msgs__msg__RobotCall__are_equal(const rio_ui_msgs__msg__RobotCall * lhs, const rio_ui_msgs__msg__RobotCall * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // office_number
  if (lhs->office_number != rhs->office_number) {
    return false;
  }
  // date
  if (lhs->date != rhs->date) {
    return false;
  }
  // robot_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot_type), &(rhs->robot_type)))
  {
    return false;
  }
  // destination
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->destination), &(rhs->destination)))
  {
    return false;
  }
  // receiver
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->receiver), &(rhs->receiver)))
  {
    return false;
  }
  // items
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->items), &(rhs->items)))
  {
    return false;
  }
  return true;
}

bool
rio_ui_msgs__msg__RobotCall__copy(
  const rio_ui_msgs__msg__RobotCall * input,
  rio_ui_msgs__msg__RobotCall * output)
{
  if (!input || !output) {
    return false;
  }
  // office_number
  output->office_number = input->office_number;
  // date
  output->date = input->date;
  // robot_type
  if (!rosidl_runtime_c__String__copy(
      &(input->robot_type), &(output->robot_type)))
  {
    return false;
  }
  // destination
  if (!rosidl_runtime_c__String__copy(
      &(input->destination), &(output->destination)))
  {
    return false;
  }
  // receiver
  if (!rosidl_runtime_c__String__copy(
      &(input->receiver), &(output->receiver)))
  {
    return false;
  }
  // items
  if (!rosidl_runtime_c__String__copy(
      &(input->items), &(output->items)))
  {
    return false;
  }
  return true;
}

rio_ui_msgs__msg__RobotCall *
rio_ui_msgs__msg__RobotCall__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__msg__RobotCall * msg = (rio_ui_msgs__msg__RobotCall *)allocator.allocate(sizeof(rio_ui_msgs__msg__RobotCall), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rio_ui_msgs__msg__RobotCall));
  bool success = rio_ui_msgs__msg__RobotCall__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rio_ui_msgs__msg__RobotCall__destroy(rio_ui_msgs__msg__RobotCall * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rio_ui_msgs__msg__RobotCall__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rio_ui_msgs__msg__RobotCall__Sequence__init(rio_ui_msgs__msg__RobotCall__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__msg__RobotCall * data = NULL;

  if (size) {
    data = (rio_ui_msgs__msg__RobotCall *)allocator.zero_allocate(size, sizeof(rio_ui_msgs__msg__RobotCall), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rio_ui_msgs__msg__RobotCall__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rio_ui_msgs__msg__RobotCall__fini(&data[i - 1]);
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
rio_ui_msgs__msg__RobotCall__Sequence__fini(rio_ui_msgs__msg__RobotCall__Sequence * array)
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
      rio_ui_msgs__msg__RobotCall__fini(&array->data[i]);
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

rio_ui_msgs__msg__RobotCall__Sequence *
rio_ui_msgs__msg__RobotCall__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rio_ui_msgs__msg__RobotCall__Sequence * array = (rio_ui_msgs__msg__RobotCall__Sequence *)allocator.allocate(sizeof(rio_ui_msgs__msg__RobotCall__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rio_ui_msgs__msg__RobotCall__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rio_ui_msgs__msg__RobotCall__Sequence__destroy(rio_ui_msgs__msg__RobotCall__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rio_ui_msgs__msg__RobotCall__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rio_ui_msgs__msg__RobotCall__Sequence__are_equal(const rio_ui_msgs__msg__RobotCall__Sequence * lhs, const rio_ui_msgs__msg__RobotCall__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rio_ui_msgs__msg__RobotCall__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rio_ui_msgs__msg__RobotCall__Sequence__copy(
  const rio_ui_msgs__msg__RobotCall__Sequence * input,
  rio_ui_msgs__msg__RobotCall__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rio_ui_msgs__msg__RobotCall);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rio_ui_msgs__msg__RobotCall * data =
      (rio_ui_msgs__msg__RobotCall *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rio_ui_msgs__msg__RobotCall__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rio_ui_msgs__msg__RobotCall__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rio_ui_msgs__msg__RobotCall__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
