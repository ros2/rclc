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

#ifndef ROSIDL_RUNTIME_C__STRING_FUNCTIONS_2_H_
#define ROSIDL_RUNTIME_C__STRING_FUNCTIONS_2_H_


#ifdef __cplusplus
extern "C"
{
#endif

#include <stddef.h>

#include "string.h"
#include "visibility_control.h"
#include <stdbool.h>

/// Initialize a rosidl_runtime_c__String structure.
/*
 * The contents of rosidl_runtime_c__String are initialized to a single null character ('\0').
 * The string initially has size 0 and capacity 1.
 * Size represents the size of the contents of the string, while capacity represents the overall
 * storage of the string (counting the null terminator).
 * All strings must be null-terminated.
 * The rosidl_runtime_c__String structure should be deallocated using the given function
 * parameter__String__fini() when it is no longer needed.
 * Calling this function with an already initialized U16 string sequence structure will leak
 * memory.
 *
 * \param[inout] str a pointer to a rosidl_runtime_c__String structure
 * \return true if successful, false if the passed string pointer is null
 *   or the memory allocation failed
 */
ROSIDL_GENERATOR_C_EXPORT
bool
parameter__String__init(rosidl_runtime_c__String * str);

/// Deallocate the memory of the rosidl_runtime_c__String structure.
/*
* Calling the function with an already deallocated sequence is a no-op.
*
* \param[inout] str a pointer to a rosidl_runtime_c__String structure to be finalized
*/
ROSIDL_GENERATOR_C_EXPORT
void
parameter__String__fini(rosidl_runtime_c__String * str);

/// Assign the c string pointer of n characters to the rosidl_runtime_c__String structure.
/*
 * \param[inout] str a pointer to a string structure
 * \param[in] value c string pointer to be assigned
 * \param[in] n size of the value string
 * \return true if successful, false if the passed string pointer is null
 *   or if the passed value pointer is null or if the size is higher than SIZE_MAX
 *   or if the memory reallocation failed.
 */
ROSIDL_GENERATOR_C_EXPORT
bool
parameter__String__assignn(
  rosidl_runtime_c__String * str, const char * value, size_t n);

/// Assign the c string pointer to the rosidl_runtime_c__String structure.
/*
 *
 * This function is identical to rosidl_runtime_c__String__assignn() except the length of the
 * c string does not have to be given and therefore the c string must be null terminated.
 *
 * \see rosidl_runtime_c__String__assignn()
 *
 * \param[inout] str a pointer to a rosidl_runtime_c__String structure
 * \param[in] value c string pointer to be assigned
 * \return true if successful, false if the passed string pointer is null
 *   or if the passed value pointer is null or if the size is higher than SIZE_MAX
 *   or if the memory reallocation failed.
 */
ROSIDL_GENERATOR_C_EXPORT
bool
parameter__String__assign(
  rosidl_runtime_c__String * str, const char * value);

/// Initialize a parameter__String__Sequence__init structure.
/*
 * The rosidl_runtime_c__String__Sequence is initialized with the size passed to the function.
 * The rosidl_runtime_c__String__Sequence structure should be deallocated using the given function
 * parameter__String__Sequence__fini() when it is no longer needed.
 * Calling this function with an already initialized U16 string sequence structure will leak
 * memory.
 *
 * \param[inout] sequence a pointer to a string sequence
 * \param[in] size represents the size of the string sequence
 * \return true if successful, false if the passed string pointer is null
 *   or the memory allocation failed
 */
ROSIDL_GENERATOR_C_EXPORT
bool
parameter__String__Sequence__init(
  rosidl_runtime_c__String__Sequence * sequence, size_t size);

/// Deallocate the memory of the string sequence structure.
/*
 * Calling the function with an already deallocated sequence is a no-op.
 *
 * \param[inout] sequence a pointer to a string sequence to be finalized
 */
ROSIDL_GENERATOR_C_EXPORT
void
parameter__String__Sequence__fini(
  rosidl_runtime_c__String__Sequence * sequence);

/// Create a rosidl_runtime_c__String__Sequence structure with a specific size.
/*
 * The string sequence initially has size and capacity equal to the size argument passed to the
 * function.
 * The rosidl_runtime_c__String__Sequence structure should be deallocated using the given function
 * rosidl_runtime_c__String__Sequence__destroy() when it is no longer needed.
 *
 * \param[in] size of the desired string sequence
 * \return a string sequence if initialization was successfully, otherwise NULL.
 */
ROSIDL_GENERATOR_C_EXPORT
rosidl_runtime_c__String__Sequence *
parameter__String__Sequence__create(size_t size);

/// Destroy a rosidl_runtime_c__String__Sequence structure.
/*
 * Calling the function with an already deallocated sequence is a no-op.
 *
 * \param[inout] sequence a pointer to a string sequence to be finalized
 */
ROSIDL_GENERATOR_C_EXPORT
void
parameter__String__Sequence__destroy(
  rosidl_runtime_c__String__Sequence * sequence);

#ifdef __cplusplus
}
#endif

#endif  // ROSIDL_RUNTIME_C__STRING_FUNCTIONS_H_
