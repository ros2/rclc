// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcl_interfaces:msg/ParameterEvent.idl
// generated code does not contain a copyright notice

#ifndef RCL_PARAMETERS__MSG__DETAIL__PARAMETER_EVENT__STRUCT_H_
#define RCL_PARAMETERS__MSG__DETAIL__PARAMETER_EVENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'node'
#include "../string/string.h"
// Member 'new_parameters'
// Member 'changed_parameters'
// Member 'deleted_parameters'
#include "parameter__struct.h"

// Struct defined in msg/ParameterEvent in the package rcl_interfaces.
typedef struct parameter__ParameterEvent
{
  builtin_interfaces__msg__Time stamp;
  rosidl_runtime_c__String node;
  parameter__Parameter__Sequence new_parameters;
  parameter__Parameter__Sequence changed_parameters;
  parameter__Parameter__Sequence deleted_parameters;
} parameter__ParameterEvent;

// Struct for a sequence of parameter__ParameterEvent.
typedef struct parameter__ParameterEvent__Sequence
{
  parameter__ParameterEvent * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parameter__ParameterEvent__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCL_PARAMETERS__MSG__DETAIL__PARAMETER_EVENT__STRUCT_H_
