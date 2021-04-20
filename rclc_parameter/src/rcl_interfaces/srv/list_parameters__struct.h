// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcl_interfaces:srv/ListParameters.idl
// generated code does not contain a copyright notice

#ifndef RCL_PARAMETERS__SRV__DETAIL__LIST_PARAMETERS__STRUCT_H_
#define RCL_PARAMETERS__SRV__DETAIL__LIST_PARAMETERS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'DEPTH_RECURSIVE'.
enum
{
  parameter__ListParameters_Request__DEPTH_RECURSIVE = 0ull
};

// Include directives for member types
// Member 'prefixes'
#include "../string/string.h"

// Struct defined in srv/ListParameters in the package rcl_interfaces.
typedef struct parameter__ListParameters_Request
{
  rosidl_runtime_c__String__Sequence prefixes;
  uint64_t depth;
} parameter__ListParameters_Request;

// Struct for a sequence of parameter__ListParameters_Request.
typedef struct parameter__ListParameters_Request__Sequence
{
  parameter__ListParameters_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parameter__ListParameters_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
#include "../msg/list_parameters_result__struct.h"

// Struct defined in srv/ListParameters in the package rcl_interfaces.
typedef struct parameter__ListParameters_Response
{
  parameter__ListParametersResult result;
} parameter__ListParameters_Response;

// Struct for a sequence of parameter__ListParameters_Response.
typedef struct parameter__ListParameters_Response__Sequence
{
  parameter__ListParameters_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} parameter__ListParameters_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCL_PARAMETERS__SRV__DETAIL__LIST_PARAMETERS__STRUCT_H_
