// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcl_interfaces:msg/ParameterValue.idl
// generated code does not contain a copyright notice

#ifndef RCL_PARAMETERS__MSG__DETAIL__PARAMETER_VALUE__STRUCT_H_
#define RCL_PARAMETERS__MSG__DETAIL__PARAMETER_VALUE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'string_value'
// Member 'string_array_value'
#include "../string/string.h"
// Member 'byte_array_value'
// Member 'bool_array_value'
// Member 'integer_array_value'
// Member 'double_array_value'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/ParameterValue in the package rcl_interfaces.
typedef struct parameter__ParameterValue
{
  uint8_t type;
  bool bool_value;
  int64_t integer_value;
  double double_value;
  rosidl_runtime_c__String string_value;
  rosidl_runtime_c__octet__Sequence byte_array_value;
  rosidl_runtime_c__boolean__Sequence bool_array_value;
  rosidl_runtime_c__int64__Sequence integer_array_value;
  rosidl_runtime_c__double__Sequence double_array_value;
  rosidl_runtime_c__String__Sequence string_array_value;
} parameter__ParameterValue;

// Struct for a sequence of parameter__ParameterValue.
typedef struct parameter__ParameterValue__Sequence
{
  parameter__ParameterValue * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parameter__ParameterValue__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCL_PARAMETERS__MSG__DETAIL__PARAMETER_VALUE__STRUCT_H_
