// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef RCL__PARAMETER_SERVER_H_
#define RCL__PARAMETER_SERVER_H_

#if __cplusplus
extern "C"
{
#endif // if __cplusplus

#include "rclc_parameter/rclc_parameter.h"

rcl_ret_t rclc_parameter_server_init_service(
        rcl_service_t* service,
        rcl_node_t* node,
        char* service_name,
        const rosidl_service_type_support_t* srv_type);

rcl_ret_t rclc_parameter_service_publish_event(
        rclc_parameter_server_t* parameter_server);

#if __cplusplus
}
#endif // if __cplusplus

#endif  // RCL__PARAMETER_SERVER_H_
