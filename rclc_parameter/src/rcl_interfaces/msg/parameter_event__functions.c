// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rcl_interfaces:msg/ParameterEvent.idl
// generated code does not contain a copyright notice
#include "parameter_event__functions.h"
#include "../string/string_functions.h"
#include "parameter__functions.h"
#include "rcutils/allocator.h"

#include "builtin_interfaces/msg/detail/time__struct.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// TODO: delete parameters?
bool
parameter__ParameterEvent__init(parameter__ParameterEvent * msg, size_t size)
{
  if (!msg) {
    return false;
  }
  // node
  if (!parameter__String__init(&msg->node)) {
    parameter__ParameterEvent__fini(msg);
    return false;
  }
  // new_parameters
  if (!parameter__Parameter__Sequence__init(&msg->new_parameters, size)) {
    parameter__ParameterEvent__fini(msg);
    return false;
  }
  // changed_parameters
  if (!parameter__Parameter__Sequence__init(&msg->changed_parameters, size)) {
    parameter__ParameterEvent__fini(msg);
    return false;
  }
  
  // deleted_parameters
  if (!parameter__Parameter__Sequence__init(&msg->deleted_parameters, size)) {
    parameter__ParameterEvent__fini(msg);
    return false;
  }
  
  return true;
}

void
parameter__ParameterEvent__fini(parameter__ParameterEvent * msg)
{
  if (!msg) {
    return;
  }
  // node
  parameter__String__fini(&msg->node);
  // new_parameters
  parameter__Parameter__Sequence__fini(&msg->new_parameters);
  // changed_parameters
  parameter__Parameter__Sequence__fini(&msg->changed_parameters);
  // deleted_parameters
  parameter__Parameter__Sequence__fini(&msg->deleted_parameters);
}

parameter__ParameterEvent *
parameter__ParameterEvent__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parameter__ParameterEvent * msg = allocator.allocate(sizeof(parameter__ParameterEvent), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parameter__ParameterEvent));
  bool success = parameter__ParameterEvent__init(msg, size);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parameter__ParameterEvent__destroy(parameter__ParameterEvent * msg)
{
  if (msg) {
    parameter__ParameterEvent__fini(msg);
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  allocator.deallocate(msg, allocator.state);
}
