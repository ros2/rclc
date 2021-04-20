// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rcl_interfaces:srv/SetParameters.idl
// generated code does not contain a copyright notice

#ifndef RCL_PARAMETERS__SRV__DETAIL__SET_PARAMETERS__FUNCTIONS_H_
#define RCL_PARAMETERS__SRV__DETAIL__SET_PARAMETERS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include <rcl/visibility_control.h>
#include "set_parameters__struct.h"

/// Initialize srv/SetParameters message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * parameter__SetParameters_Request
 * )) before or use
 * parameter__SetParameters_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
RCL_PUBLIC
bool
parameter__SetParameters_Request__init(parameter__SetParameters_Request * msg, size_t size);

/// Finalize srv/SetParameters message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
RCL_PUBLIC
void
parameter__SetParameters_Request__fini(parameter__SetParameters_Request * msg);

/// Create srv/SetParameters message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * parameter__SetParameters_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
RCL_PUBLIC
parameter__SetParameters_Request *
parameter__SetParameters_Request__create(size_t size);

/// Destroy srv/SetParameters message.
/**
 * It calls
 * parameter__SetParameters_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
RCL_PUBLIC
void
parameter__SetParameters_Request__destroy(parameter__SetParameters_Request * msg);


/// Initialize array of srv/SetParameters messages.
/**
 * It allocates the memory for the number of elements and calls
 * parameter__SetParameters_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
RCL_PUBLIC
bool
parameter__SetParameters_Request__Sequence__init(parameter__SetParameters_Request__Sequence * array, size_t size);

/// Finalize array of srv/SetParameters messages.
/**
 * It calls
 * parameter__SetParameters_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
RCL_PUBLIC
void
parameter__SetParameters_Request__Sequence__fini(parameter__SetParameters_Request__Sequence * array);

/// Initialize srv/SetParameters message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * parameter__SetParameters_Response
 * )) before or use
 * parameter__SetParameters_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
RCL_PUBLIC
bool
parameter__SetParameters_Response__init(parameter__SetParameters_Response * msg, size_t size);

/// Finalize srv/SetParameters message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
RCL_PUBLIC
void
parameter__SetParameters_Response__fini(parameter__SetParameters_Response * msg);

/// Create srv/SetParameters message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * parameter__SetParameters_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
RCL_PUBLIC
parameter__SetParameters_Response *
parameter__SetParameters_Response__create(size_t size);

/// Destroy srv/SetParameters message.
/**
 * It calls
 * parameter__SetParameters_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
RCL_PUBLIC
void
parameter__SetParameters_Response__destroy(parameter__SetParameters_Response * msg);

#ifdef __cplusplus
}
#endif

#endif  // RCL_PARAMETERS__SRV__DETAIL__SET_PARAMETERS__FUNCTIONS_H_
