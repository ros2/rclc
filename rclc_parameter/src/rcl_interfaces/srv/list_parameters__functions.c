// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rcl_interfaces:srv/ListParameters.idl
// generated code does not contain a copyright notice
#include "../msg/list_parameters_result__functions.h"
#include "list_parameters__functions.h"
#include "../string/string_functions.h"
#include "rcutils/allocator.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// Request message
bool
parameter__ListParameters_Request__init(parameter__ListParameters_Request * msg, size_t size)
{
  if (!msg) {
    return false;
  }
  // prefixes
  if (!parameter__String__Sequence__init(&msg->prefixes, size)) {
    parameter__ListParameters_Request__fini(msg);
    return false;
  }
  // depth
  return true;
}

void
parameter__ListParameters_Request__fini(parameter__ListParameters_Request * msg)
{
  if (!msg) {
    return;
  }
  // prefixes
  parameter__String__Sequence__fini(&msg->prefixes);
  // depth
}

parameter__ListParameters_Request *
parameter__ListParameters_Request__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parameter__ListParameters_Request * msg = allocator.allocate(sizeof(parameter__ListParameters_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parameter__ListParameters_Request));
  bool success = parameter__ListParameters_Request__init(msg, size);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parameter__ListParameters_Request__destroy(parameter__ListParameters_Request * msg)
{
  if (msg) {
    parameter__ListParameters_Request__fini(msg);
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  allocator.deallocate(msg, allocator.state);
}

// Response message
bool
parameter__ListParameters_Response__init(parameter__ListParameters_Response * msg, size_t size)
{
  if (!msg) {
    return false;
  }
  // result
  if (!parameter__ListParametersResult__init(&msg->result, size)) {
    parameter__ListParameters_Response__fini(msg);
    return false;
  }
  return true;
}

void
parameter__ListParameters_Response__fini(parameter__ListParameters_Response * msg)
{
  if (!msg) {
    return;
  }
  // result
  parameter__ListParametersResult__fini(&msg->result);
}

parameter__ListParameters_Response *
parameter__ListParameters_Response__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parameter__ListParameters_Response * msg = allocator.allocate(sizeof(parameter__ListParameters_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parameter__ListParameters_Response));
  bool success = parameter__ListParameters_Response__init(msg, size);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parameter__ListParameters_Response__destroy(parameter__ListParameters_Response * msg)
{
  if (msg) {
    parameter__ListParameters_Response__fini(msg);
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  allocator.deallocate(msg, allocator.state);
}

