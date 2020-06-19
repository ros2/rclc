#include <stdio.h>
#include <unistd.h>

#include <rcl/error_handling.h>
#include <rcutils/logging_macros.h>

#include <lifecycle_msgs/msg/transition_description.h>
#include <lifecycle_msgs/msg/transition_event.h>
#include <lifecycle_msgs/srv/change_state.h>
#include <lifecycle_msgs/srv/get_state.h>
#include <lifecycle_msgs/srv/get_available_states.h>
#include <lifecycle_msgs/srv/get_available_transitions.h>

#include "rclc_lifecycle/lifecycle.h"

rcl_ret_t on_configure() {
  printf("  >>> lifecycle_node: on_configure() callback called.\n");
  return RCL_RET_OK;
}

rcl_ret_t on_activate() {
  printf("  >>> lifecycle_node: on_activate() callback called.\n");
  return RCL_RET_OK;
}

rcl_ret_t foobar() {
  printf("  >>> lifecycle_node: on_deactivate() callback called.\n");
  return RCL_RET_OK;
}

rcl_ret_t on_cleanup() {
  printf("  >>> lifecycle_node: on_cleanup() callback called.\n");
  return RCL_RET_OK;
}

int main(int argc, char **argv)
{
  rcl_context_t context = rcl_get_zero_initialized_context();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t rc;

  // create init_options
  rc = rcl_init_options_init(&init_options, allocator);
  if (rc != RCL_RET_OK) {
    printf("Error rcl_init_options_init.\n");
    return -1;
  }

  // create context
  rc = rcl_init(argc, argv, &init_options, &context);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_init.\n");
    return -1;
  }

  // create rcl_node
  printf("creating rcl node...\n");
  rcl_node_t my_node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rc = rcl_node_init(&my_node, "lifecycle_node", "rclc", &context, &node_ops);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_node_init\n");
    return -1;
  }
  
  // make it a lifecycle node
  printf("make it a lifecycle node...\n");
  rcl_lifecycle_state_machine_t state_machine_ = rcl_lifecycle_get_zero_initialized_state_machine();
  rclc_lifecycle_node_t lifecycle_node = rclc_make_node_a_lifecycle_node(
    &my_node,
    &state_machine_,
    &node_ops);
    
  // register callbacks
  rclc_lifecycle_register_on_configure(&lifecycle_node, &on_configure);
  rclc_lifecycle_register_on_deactivate(&lifecycle_node, &foobar);
 
  printf(" >configuring lifecycle node...\n");
  rc = rclc_lifecycle_change_state(
    &lifecycle_node,
    lifecycle_msgs__msg__Transition__TRANSITION_CONFIGURE,
    true);
  if (rc != RCL_RET_OK) {
    printf("Error in TRANSITION_CONFIGURE\n");
    return -1;
  }

  printf(" >activating lifecycle node...\n");
  rc = rclc_lifecycle_change_state(
    &lifecycle_node,
    lifecycle_msgs__msg__Transition__TRANSITION_ACTIVATE,
    true);
  if (rc != RCL_RET_OK) {
    printf("Error in TRANSITION_ACTIVATE\n");
    return -1;
  }
 
  printf(" >deactivating lifecycle node...\n");
  rc = rclc_lifecycle_change_state(
    &lifecycle_node,
    lifecycle_msgs__msg__Transition__TRANSITION_DEACTIVATE,
    true);
  if (rc != RCL_RET_OK) {
    printf("Error in TRANSITION_DEACTIVATE\n");
    return -1;
  }

  printf(" >cleaning rcl node up...\n");
  rc = rclc_lifecycle_change_state(
    &lifecycle_node,
    lifecycle_msgs__msg__Transition__TRANSITION_CLEANUP,
    true);
  if (rc != RCL_RET_OK) {
    printf("Error in TRANSITION_CLEANUP\n");
    return -1;
  }

  printf(" >destroying lifecycle node...\n");
  rc = rclc_lifecycle_change_state(
    &lifecycle_node,
    lifecycle_msgs__msg__Transition__TRANSITION_UNCONFIGURED_SHUTDOWN,
    true);
  if (rc != RCL_RET_OK) {
    printf("Error in TRANSITION_UNCONFIGURED_SHUTDOWN\n");
    return -1;
  }

  printf("cleaning up...\n");
  rc = rcl_lifecycle_node_fini(&lifecycle_node, &node_ops);
  rc += rcl_init_options_fini(&init_options);

  if (rc != RCL_RET_OK) {
    printf("Error while cleaning up!\n");
    return -1;
  }
  return 0;
}
