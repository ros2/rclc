#include <stdio.h>
#include <unistd.h>

// Copyright (c) 2021 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/ros2/rclc.
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

#include <stdio.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rclc_parameter/rclc_parameter.h>

rclc_parameter_server_t param_server;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) timer;
  (void) last_call_time;

  int value;
  rclc_parameter_get_int(&param_server, "param2", &value);
  value++;
  rclc_parameter_set_int(&param_server, "param2", (int64_t) value);
}

void on_parameter_changed(Parameter * param)
{
  printf("Parameter %s modified.", param->name.data);
  switch (param->value.type) {
    case RCLC_PARAMETER_BOOL:
      printf(" New value: %d (bool)", param->value.bool_value);
      break;
    case RCLC_PARAMETER_INT:
      printf(" New value: %ld (int)", param->value.integer_value);
      break;
    case RCLC_PARAMETER_DOUBLE:
      printf(" New value: %f (double)", param->value.double_value);
      break;
    default:
      break;
  }
  printf("\n");
}

int main()
{
  rcl_ret_t rc;
  rcl_allocator_t allocator = rcl_get_default_allocator();

  // Create init_options
  rclc_support_t support;
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rcl_node_t node;
  rclc_node_init_default(&node, "demo_param_node", "", &support);

  // Create parameter service
  rclc_parameter_server_init_default(&param_server, &node);

  // create timer,
  rcl_timer_t timer;
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(1000),
    timer_callback);

  // Create executor
  // Note:
  // If you need more than the default number of publisher/subscribers, etc., you
  // need to configure the micro-ROS middleware also!
  // See documentation in the executor.h at the function rclc_executor_init()
  // for more details.
  rclc_executor_t executor;
  rclc_executor_init(
    &executor, &support.context, RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER + 1,
    &allocator);
  rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed);
  rclc_executor_add_timer(&executor, &timer);

  // Add parameters
  rclc_add_parameter(&param_server, "param1", RCLC_PARAMETER_BOOL);
  rclc_add_parameter(&param_server, "param2", RCLC_PARAMETER_INT);
  rclc_add_parameter(&param_server, "param3", RCLC_PARAMETER_DOUBLE);

  rclc_parameter_set_bool(&param_server, "param1", false);
  rclc_parameter_set_int(&param_server, "param2", 10);
  rclc_parameter_set_double(&param_server, "param3", 0.01);

  bool param1;
  int param2;
  double param3;

  rclc_parameter_get_bool(&param_server, "param1", &param1);
  rclc_parameter_get_int(&param_server, "param2", &param2);
  rclc_parameter_get_double(&param_server, "param3", &param3);

  // Start Executor
  rclc_executor_spin(&executor);

  // clean up
  rc = rclc_executor_fini(&executor);
  rc += rclc_parameter_server_fini(&param_server, &node);
  rc += rcl_node_fini(&node);

  if (rc != RCL_RET_OK) {
    printf("Error while cleaning up!\n");
    return -1;
  }
  return 0;
}
