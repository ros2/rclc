// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rcl_interfaces:srv/GetParameterTypes.idl
// generated code does not contain a copyright notice
#include "get_parameter_types__functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "../string/string_functions.h"
#include "rcutils/allocator.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// Request message
bool
parameter__GetParameterTypes_Request__init(parameter__GetParameterTypes_Request * msg, size_t size)
{
  if (!msg) {
    return false;
  }
  // names
  if (!parameter__String__Sequence__init(&msg->names, size)) {
    parameter__GetParameterTypes_Request__fini(msg);
    return false;
  }
  return true;
}

void
parameter__GetParameterTypes_Request__fini(parameter__GetParameterTypes_Request * msg)
{
  if (!msg) {
    return;
  }
  // names
  parameter__String__Sequence__fini(&msg->names);
}

parameter__GetParameterTypes_Request *
parameter__GetParameterTypes_Request__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parameter__GetParameterTypes_Request * msg = allocator.allocate(sizeof(parameter__GetParameterTypes_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parameter__GetParameterTypes_Request));
  bool success = parameter__GetParameterTypes_Request__init(msg, size);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parameter__GetParameterTypes_Request__destroy(parameter__GetParameterTypes_Request * msg)
{
  if (msg) {
    parameter__GetParameterTypes_Request__fini(msg);
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  allocator.deallocate(msg, allocator.state);
}

// Response message
bool
parameter__GetParameterTypes_Response__init(parameter__GetParameterTypes_Response * msg, size_t size)
{
  if (!msg) {
    return false;
  }
  // types
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->types, size)) {
    parameter__GetParameterTypes_Response__fini(msg);
    return false;
  }
  return true;
}

void
parameter__GetParameterTypes_Response__fini(parameter__GetParameterTypes_Response * msg)
{
  if (!msg) {
    return;
  }
  // types
  rosidl_runtime_c__uint8__Sequence__fini(&msg->types);
}

parameter__GetParameterTypes_Response *
parameter__GetParameterTypes_Response__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parameter__GetParameterTypes_Response * msg = allocator.allocate(sizeof(parameter__GetParameterTypes_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parameter__GetParameterTypes_Response));
  bool success = parameter__GetParameterTypes_Response__init(msg, size);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parameter__GetParameterTypes_Response__destroy(parameter__GetParameterTypes_Response * msg)
{
  if (msg) {
    parameter__GetParameterTypes_Response__fini(msg);
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  allocator.deallocate(msg, allocator.state);
}
