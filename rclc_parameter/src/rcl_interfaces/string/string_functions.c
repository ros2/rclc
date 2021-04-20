// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rosidl_runtime_c/string_functions.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "rcutils/allocator.h"
#include "rcutils/macros.h"

#define STRING_MAX_SIZE 20

bool
parameter__String__init(rosidl_runtime_c__String * str)
{
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(false);

  if (!str) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  str->data = allocator.allocate(STRING_MAX_SIZE + 1, allocator.state);
  if (!str->data) {
    return false;
  }
  str->data[0] = '\0';
  str->size = 0;
  str->capacity = STRING_MAX_SIZE + 1;
  return true;
}

void
parameter__String__fini(rosidl_runtime_c__String * str)
{
  if (!str) {
    return;
  }
  if (str->data) {
    /* ensure that data and capacity values are consistent */
    if (str->capacity <= 0) {
      fprintf(
        stderr, "Unexpected condition: string capacity was zero for allocated data! "
        "Exiting.\n");
      exit(-1);
    }
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    allocator.deallocate(str->data, allocator.state);
    str->data = NULL;
    str->size = 0;
    str->capacity = 0;
  } else {
    /* ensure that data, size, and capacity values are consistent */
    if (0 != str->size) {
      fprintf(
        stderr, "Unexpected condition: string size was non-zero for deallocated data! "
        "Exiting.\n");
      exit(-1);
    }
    if (0 != str->capacity) {
      fprintf(
        stderr, "Unexpected behavior: string capacity was non-zero for deallocated data! "
        "Exiting.\n");
      exit(-1);
    }
  }
}

bool
parameter__String__assignn(
  rosidl_runtime_c__String * str, const char * value, size_t lenght)
{
  if (!str) {
    return false;
  }
  // a NULL value is not valid
  if (!value) {
    return false;
  }
  // since STRING_MAX_SIZE + 1 bytes are allocated, lenght can't be superior to STRING_MAX_SIZE
  if (lenght > STRING_MAX_SIZE) {
    return false;
  }
  /*
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  char * data = allocator.reallocate(str->data, n + 1, allocator.state);
  if (!data) {
    return false;
  }
  */
  memcpy(str->data, value, lenght);
  str->data[lenght] = '\0';
  str->size = lenght;
  return true;
}

bool
parameter__String__assign(
  rosidl_runtime_c__String * str, const char * value)
{
  if (!value) {
    // strlen is not defined for nullptr, let assignn take care of other bad values
    return false;
  }
  return parameter__String__assignn(
    str, value, strlen(value));
}

bool
parameter__String__Sequence__init(
  rosidl_runtime_c__String__Sequence * sequence, size_t size)
{
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(false);

  if (!sequence) {
    return false;
  }
  rosidl_runtime_c__String * data = NULL;
  if (size) {
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    data = allocator.zero_allocate(size, sizeof(rosidl_runtime_c__String), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all sequence elements
    for (size_t i = 0; i < size; ++i) {
      if (!parameter__String__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > 0; ) {
          parameter__String__fini(&data[i]);
        }
        allocator.deallocate(data, allocator.state);
        return false;
      }
    }
  }
  sequence->data = data;
  sequence->size = 0;
  sequence->capacity = size;
  return true;
}

void
parameter__String__Sequence__fini(
  rosidl_runtime_c__String__Sequence * sequence)
{
  if (!sequence) {
    return;
  }
  if (sequence->data) {
    // ensure that data and capacity values are consistent
    assert(sequence->capacity > 0);
    // finalize all sequence elements
    for (size_t i = 0; i < sequence->capacity; ++i) {
      parameter__String__fini(&sequence->data[i]);
    }
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    allocator.deallocate(sequence->data, allocator.state);
    sequence->data = NULL;
    sequence->size = 0;
    sequence->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == sequence->size);
    assert(0 == sequence->capacity);
  }
}

rosidl_runtime_c__String__Sequence *
parameter__String__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_runtime_c__String__Sequence * sequence = allocator.allocate(sizeof(rosidl_runtime_c__String__Sequence), allocator.state);
  if (!sequence) {
    return NULL;
  }
  bool success = parameter__String__Sequence__init(sequence, size);
  if (!success) {
    allocator.deallocate(sequence, allocator.state);
    return NULL;
  }
  return sequence;
}

void
parameter__String__Sequence__destroy(
  rosidl_runtime_c__String__Sequence * sequence)
{
  if (sequence) {
    parameter__String__Sequence__fini(sequence);
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  allocator.deallocate(sequence, allocator.state);
}
