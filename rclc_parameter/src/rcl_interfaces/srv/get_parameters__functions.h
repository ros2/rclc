// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rcl_interfaces:srv/GetParameters.idl
// generated code does not contain a copyright notice

#ifndef RCL_PARAMETERS__SRV__DETAIL__GET_PARAMETERS__FUNCTIONS_H_
#define RCL_PARAMETERS__SRV__DETAIL__GET_PARAMETERS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include <rcl/visibility_control.h>

#include "get_parameters__struct.h"

/// Initialize srv/GetParameters message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * parameter__GetParameters_Request
 * )) before or use
 * parameter__GetParameters_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */

RCL_PUBLIC
bool
parameter__GetParameters_Request__init(parameter__GetParameters_Request * msg, size_t size);

/// Finalize srv/GetParameters message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
RCL_PUBLIC
void
parameter__GetParameters_Request__fini(parameter__GetParameters_Request * msg);

/// Create srv/GetParameters message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * parameter__GetParameters_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
RCL_PUBLIC
parameter__GetParameters_Request *
parameter__GetParameters_Request__create(size_t size);

/// Destroy srv/GetParameters message.
/**
 * It calls
 * parameter__GetParameters_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
RCL_PUBLIC
void
parameter__GetParameters_Request__destroy(parameter__GetParameters_Request * msg);

/// Initialize srv/GetParameters message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * parameter__GetParameters_Response
 * )) before or use
 * parameter__GetParameters_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
RCL_PUBLIC
bool
parameter__GetParameters_Response__init(parameter__GetParameters_Response * msg, size_t size);

/// Finalize srv/GetParameters message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
RCL_PUBLIC
void
parameter__GetParameters_Response__fini(parameter__GetParameters_Response * msg);

/// Create srv/GetParameters message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * parameter__GetParameters_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
RCL_PUBLIC
parameter__GetParameters_Response *
parameter__GetParameters_Response__create(size_t size);

/// Destroy srv/GetParameters message.
/**
 * It calls
 * parameter__GetParameters_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
RCL_PUBLIC
void
parameter__GetParameters_Response__destroy(parameter__GetParameters_Response * msg);

#ifdef __cplusplus
}
#endif

#endif  // RCL_PARAMETERS__SRV__DETAIL__GET_PARAMETERS__FUNCTIONS_H_
