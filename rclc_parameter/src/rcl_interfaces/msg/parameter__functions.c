// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rcl_interfaces:msg/Parameter.idl
// generated code does not contain a copyright notice
#include "parameter_value__functions.h"
#include "parameter__functions.h"
#include "../string/string_functions.h"
#include "rcutils/allocator.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

bool
parameter__Parameter__init(parameter__Parameter * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!parameter__String__init(&msg->name)) {
    parameter__Parameter__fini(msg);
    return false;
  }
  // value
  if (!parameter__ParameterValue__init(&msg->value, 0)) {
    parameter__Parameter__fini(msg);
    return false;
  }
  return true;
}

void
parameter__Parameter__fini(parameter__Parameter * msg)
{
  if (!msg) {
    return;
  }
  // name
  parameter__String__fini(&msg->name);
  // value
  parameter__ParameterValue__fini(&msg->value);
}

parameter__Parameter *
parameter__Parameter__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parameter__Parameter * msg = allocator.allocate(sizeof(parameter__Parameter), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parameter__Parameter));
  bool success = parameter__Parameter__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parameter__Parameter__destroy(parameter__Parameter * msg)
{
  if (msg) {
    parameter__Parameter__fini(msg);
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  allocator.deallocate(msg, allocator.state);
}

bool
parameter__Parameter__Sequence__init(parameter__Parameter__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parameter__Parameter * data = NULL;
  if (size) {
    
    data = allocator.zero_allocate(size, sizeof(parameter__Parameter), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = parameter__Parameter__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        parameter__Parameter__fini(&data[i - 1]);
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
parameter__Parameter__Sequence__fini(parameter__Parameter__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      parameter__Parameter__fini(&array->data[i]);
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

parameter__Parameter__Sequence *
parameter__Parameter__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parameter__Parameter__Sequence * array = allocator.allocate(sizeof(parameter__Parameter__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = parameter__Parameter__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
parameter__Parameter__Sequence__destroy(parameter__Parameter__Sequence * array)
{
  if (array) {
    parameter__Parameter__Sequence__fini(array);
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  allocator.deallocate(array, allocator.state);
}
