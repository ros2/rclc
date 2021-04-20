// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rcl_interfaces:srv/SetParameters.idl
// generated code does not contain a copyright notice
#include "../msg/set_parameters_result__functions.h"
#include "../msg/parameter__functions.h"
#include "set_parameters__functions.h"
#include "rcutils/allocator.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// Request message
bool
parameter__SetParameters_Request__init(parameter__SetParameters_Request * msg, size_t size)
{
  if (!msg) {
    return false;
  }
  // parameters
  if (!parameter__Parameter__Sequence__init(&msg->parameters, size)) {
    parameter__SetParameters_Request__fini(msg);
    return false;
  }
  return true;
}

void
parameter__SetParameters_Request__fini(parameter__SetParameters_Request * msg)
{
  if (!msg) {
    return;
  }
  // parameters
  parameter__Parameter__Sequence__fini(&msg->parameters);
}

parameter__SetParameters_Request *
parameter__SetParameters_Request__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parameter__SetParameters_Request * msg = allocator.allocate(sizeof(parameter__SetParameters_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parameter__SetParameters_Request));
  bool success = parameter__SetParameters_Request__init(msg, size);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parameter__SetParameters_Request__destroy(parameter__SetParameters_Request * msg)
{
  if (msg) {
    parameter__SetParameters_Request__fini(msg);
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  allocator.deallocate(msg, allocator.state);
}

// Response message
bool
parameter__SetParameters_Response__init(parameter__SetParameters_Response * msg, size_t size)
{
  if (!msg) {
    return false;
  }
  // results
  if (!parameter__SetParametersResult__Sequence__init(&msg->results, size)) {
    parameter__SetParameters_Response__fini(msg);
    return false;
  }
  return true;
}

void
parameter__SetParameters_Response__fini(parameter__SetParameters_Response * msg)
{
  if (!msg) {
    return;
  }
  // results
  parameter__SetParametersResult__Sequence__fini(&msg->results);
}

parameter__SetParameters_Response *
parameter__SetParameters_Response__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parameter__SetParameters_Response * msg = allocator.allocate(sizeof(parameter__SetParameters_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parameter__SetParameters_Response));
  bool success = parameter__SetParameters_Response__init(msg, size);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parameter__SetParameters_Response__destroy(parameter__SetParameters_Response * msg)
{
  if (msg) {
    parameter__SetParameters_Response__fini(msg);
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  allocator.deallocate(msg, allocator.state);
}
