// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rcl_interfaces:srv/GetParameters.idl
// generated code does not contain a copyright notice
#include "../msg/parameter_value__functions.h"
#include "../string/string_functions.h"
#include "get_parameters__functions.h"
#include "rcutils/allocator.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// Request message
bool
parameter__GetParameters_Request__init(parameter__GetParameters_Request * msg, size_t size)
{
  if (!msg) {
    return false;
  }
  // names
  if (!parameter__String__Sequence__init(&msg->names, size)) {
    parameter__GetParameters_Request__fini(msg);
    return false;
  }
  return true;
}

void
parameter__GetParameters_Request__fini(parameter__GetParameters_Request * msg)
{
  if (!msg) {
    return;
  }
  // names
  parameter__String__Sequence__fini(&msg->names);
}

parameter__GetParameters_Request *
parameter__GetParameters_Request__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parameter__GetParameters_Request * msg = allocator.allocate(sizeof(parameter__GetParameters_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parameter__GetParameters_Request));
  bool success = parameter__GetParameters_Request__init(msg, size);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parameter__GetParameters_Request__destroy(parameter__GetParameters_Request * msg)
{
  if (msg) {
    parameter__GetParameters_Request__fini(msg);
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  allocator.deallocate(msg, allocator.state);
}

// Response message
bool
parameter__GetParameters_Response__init(parameter__GetParameters_Response * msg, size_t size)
{
  if (!msg) {
    return false;
  }
  // values
  if (!parameter__ParameterValue__Sequence__init(&msg->values, size)) {
    parameter__GetParameters_Response__fini(msg);
    return false;
  }
  return true;
}

void
parameter__GetParameters_Response__fini(parameter__GetParameters_Response * msg)
{
  if (!msg) {
    return;
  }
  // values
  parameter__ParameterValue__Sequence__fini(&msg->values);
}

parameter__GetParameters_Response *
parameter__GetParameters_Response__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parameter__GetParameters_Response * msg = allocator.allocate(sizeof(parameter__GetParameters_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parameter__GetParameters_Response));
  bool success = parameter__GetParameters_Response__init(msg, size);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parameter__GetParameters_Response__destroy(parameter__GetParameters_Response * msg)
{
  if (msg) {
    parameter__GetParameters_Response__fini(msg);
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  allocator.deallocate(msg, allocator.state);
}
