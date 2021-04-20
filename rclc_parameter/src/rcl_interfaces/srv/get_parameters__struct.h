// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcl_interfaces:srv/GetParameters.idl
// generated code does not contain a copyright notice

#ifndef RCL_PARAMETERS__SRV__DETAIL__GET_PARAMETERS__STRUCT_H_
#define RCL_PARAMETERS__SRV__DETAIL__GET_PARAMETERS__STRUCT_H_

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
#include "../string/string.h"

// Struct defined in srv/GetParameters in the package rcl_interfaces.
typedef struct parameter__GetParameters_Request
{
  rosidl_runtime_c__String__Sequence names;
} parameter__GetParameters_Request;

// Struct for a sequence of parameter__GetParameters_Request.
typedef struct parameter__GetParameters_Request__Sequence
{
  parameter__GetParameters_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parameter__GetParameters_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'values'
#include "../msg/parameter_value__struct.h"

// Struct defined in srv/GetParameters in the package rcl_interfaces.
typedef struct parameter__GetParameters_Response
{
  parameter__ParameterValue__Sequence values;
} parameter__GetParameters_Response;

// Struct for a sequence of parameter__GetParameters_Response.
typedef struct parameter__GetParameters_Response__Sequence
{
  parameter__GetParameters_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parameter__GetParameters_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCL_PARAMETERS__SRV__DETAIL__GET_PARAMETERS__STRUCT_H_
