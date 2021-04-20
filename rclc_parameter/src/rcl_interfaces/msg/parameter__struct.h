// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcl_interfaces:msg/Parameter.idl
// generated code does not contain a copyright notice

#ifndef RCL_PARAMETERS__MSG__DETAIL__PARAMETER__STRUCT_H_
#define RCL_PARAMETERS__MSG__DETAIL__PARAMETER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "../string/string.h"
// Member 'value'
#include "parameter_value__struct.h"

// Struct defined in msg/Parameter in the package rcl_interfaces.
typedef struct parameter__Parameter
{
  rosidl_runtime_c__String name;
  parameter__ParameterValue value;
} parameter__Parameter;

// Struct for a sequence of parameter__Parameter.
typedef struct parameter__Parameter__Sequence
{
  parameter__Parameter * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parameter__Parameter__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCL_PARAMETERS__MSG__DETAIL__PARAMETER__STRUCT_H_
