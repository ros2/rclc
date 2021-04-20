// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcl_interfaces:srv/GetParameterTypes.idl
// generated code does not contain a copyright notice

#ifndef RCL_PARAMETERS__SRV__DETAIL__GET_PARAMETER_TYPES__STRUCT_H_
#define RCL_PARAMETERS__SRV__DETAIL__GET_PARAMETER_TYPES__STRUCT_H_

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
#include "../string/string_functions.h"

// Struct defined in srv/GetParameterTypes in the package rcl_interfaces.
typedef struct parameter__GetParameterTypes_Request
{
  rosidl_runtime_c__String__Sequence names;
} parameter__GetParameterTypes_Request;

// Struct for a sequence of parameter__GetParameterTypes_Request.
typedef struct parameter__GetParameterTypes_Request__Sequence
{
  parameter__GetParameterTypes_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parameter__GetParameterTypes_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'types'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in srv/GetParameterTypes in the package rcl_interfaces.
typedef struct parameter__GetParameterTypes_Response
{
  rosidl_runtime_c__uint8__Sequence types;
} parameter__GetParameterTypes_Response;

// Struct for a sequence of parameter__GetParameterTypes_Response.
typedef struct parameter__GetParameterTypes_Response__Sequence
{
  parameter__GetParameterTypes_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parameter__GetParameterTypes_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCL_PARAMETERS__SRV__DETAIL__GET_PARAMETER_TYPES__STRUCT_H_
