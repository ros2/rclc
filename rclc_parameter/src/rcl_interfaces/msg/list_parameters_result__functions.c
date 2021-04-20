// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rcl_interfaces:msg/ListParametersResult.idl
// generated code does not contain a copyright notice
#include "list_parameters_result__functions.h"
#include "../string/string_functions.h"
#include "rcutils/allocator.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

bool
parameter__ListParametersResult__init(parameter__ListParametersResult * msg, size_t size)
{
  if (!msg) {
    return false;
  }
  // names
  if (!parameter__String__Sequence__init(&msg->names, size)) {
    parameter__ListParametersResult__fini(msg);
    return false;
  }
  // prefixes
  // TODO: Check prefixes expected lenght and values
  if (!parameter__String__Sequence__init(&msg->prefixes, size)) {
    parameter__ListParametersResult__fini(msg);
    return false;
  }
  return true;
}

void
parameter__ListParametersResult__fini(parameter__ListParametersResult * msg)
{
  if (!msg) {
    return;
  }
  // names
  parameter__String__Sequence__fini(&msg->names);
  // prefixes
  parameter__String__Sequence__fini(&msg->prefixes);
}

parameter__ListParametersResult *
parameter__ListParametersResult__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  parameter__ListParametersResult * msg = allocator.allocate(sizeof(parameter__ListParametersResult), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(parameter__ListParametersResult));
  bool success = parameter__ListParametersResult__init(msg, size);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
parameter__ListParametersResult__destroy(parameter__ListParametersResult * msg)
{
  if (msg) {
    parameter__ListParametersResult__fini(msg);
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  allocator.deallocate(msg, allocator.state);
}
