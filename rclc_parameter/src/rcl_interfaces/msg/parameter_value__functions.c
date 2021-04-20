// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rcl_interfaces:msg/ParameterValue.idl
// generated code does not contain a copyright notice
#include "parameter_value__functions.h"
#include "../string/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "rcutils/allocator.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

bool
parameter__ParameterValue__init(parameter__ParameterValue * msg, size_t size)
{
  if (!msg) {
    return false;
  }
  if (!parameter__String__init(&msg->string_value)) {
    parameter__ParameterValue__fini(msg);
    return false;
  }
  // byte_array_value
  if (!rosidl_runtime_c__octet__Sequence__init(&msg->byte_array_value, size)) {
    parameter__ParameterValue__fini(msg);
    return false;
  }
  // bool_array_value
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->bool_array_value, size)) {
    parameter__ParameterValue__fini(msg);
    return false;
  }
  // integer_array_value
  if (!rosidl_runtime_c__int64__Sequence__init(&msg->integer_array_value, size)) {
    parameter__ParameterValue__fini(msg);
    return false;
  }
  // double_array_value
  if (!rosidl_runtime_c__double__Sequence__init(&msg->double_array_value, size)) {
    parameter__ParameterValue__fini(msg);
    return false;
  }
  // string_array_value
  if (!parameter__String__Sequence__init(&msg->string_array_value, size)) {
    parameter__ParameterValue__fini(msg);
    return false;
  }
  return true;
}

void
parameter__ParameterValue__fini(parameter__ParameterValue * msg)
{
  if (!msg) {
    return;
  }
  parameter__String__fini(&msg->string_value);
  // byte_array_value
  rosidl_runtime_c__octet__Sequence__fini(&msg->byte_array_value);
  // bool_array_value
  rosidl_runtime_c__boolean__Sequence__fini(&msg->bool_array_value);
  // integer_array_value
  rosidl_runtime_c__int64__Sequence__fini(&msg->integer_array_value);
  // double_array_value
  rosidl_runtime_c__double__Sequence__fini(&msg->double_array_value);
  // string_array_value
  parameter__String__Sequence__fini(&msg->string_array_value);
}

parameter__ParameterValue *
parameter__ParameterValue__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parameter__ParameterValue * msg = allocator.allocate(sizeof(parameter__ParameterValue), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parameter__ParameterValue));
  bool success = parameter__ParameterValue__init(msg, size);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parameter__ParameterValue__destroy(parameter__ParameterValue * msg)
{
  if (msg) {
    parameter__ParameterValue__fini(msg);
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  allocator.deallocate(msg, allocator.state);
}

// TODO: array size
bool
parameter__ParameterValue__Sequence__init(parameter__ParameterValue__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parameter__ParameterValue * data = NULL;
  if (size) {
    
    data = allocator.zero_allocate(size, sizeof(parameter__ParameterValue), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parameter__ParameterValue__init(&data[i], 0);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parameter__ParameterValue__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = 0;
  array->capacity = size;
  return true;
}

void
parameter__ParameterValue__Sequence__fini(parameter__ParameterValue__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      parameter__ParameterValue__fini(&array->data[i]);
    }
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
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

parameter__ParameterValue__Sequence *
parameter__ParameterValue__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parameter__ParameterValue__Sequence * array = allocator.allocate(sizeof(parameter__ParameterValue__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parameter__ParameterValue__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parameter__ParameterValue__Sequence__destroy(parameter__ParameterValue__Sequence * array)
{
  if (array) {
    parameter__ParameterValue__Sequence__fini(array);
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  allocator.deallocate(array, allocator.state);
}
