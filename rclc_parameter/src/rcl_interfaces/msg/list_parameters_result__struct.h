// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcl_interfaces:msg/ListParametersResult.idl
// generated code does not contain a copyright notice

#ifndef RCL_PARAMETERS__MSG__DETAIL__LIST_PARAMETERS_RESULT__STRUCT_H_
#define RCL_PARAMETERS__MSG__DETAIL__LIST_PARAMETERS_RESULT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'names'
// Member 'prefixes'
#include "../string/string.h"

// Struct defined in msg/ListParametersResult in the package rcl_interfaces.
typedef struct parameter__ListParametersResult
{
  rosidl_runtime_c__String__Sequence names;
  rosidl_runtime_c__String__Sequence prefixes;
} parameter__ListParametersResult;

// Struct for a sequence of parameter__ListParametersResult.
typedef struct parameter__ListParametersResult__Sequence
{
  parameter__ListParametersResult * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parameter__ListParametersResult__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCL_PARAMETERS__MSG__DETAIL__LIST_PARAMETERS_RESULT__STRUCT_H_
