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

#include "rclc_lifecycle/rclc_lifecycle.h"

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

int main(int argc, const char * argv[])
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_ret_t rc;

  // create init_options
  rc = rclc_support_init(&support, argc, argv, &allocator);
  if (rc != RCL_RET_OK) {
    printf("Error rclc_support_init.\n");
    return -1;
  }

  // create rcl_node
  rcl_node_t my_node = rcl_get_zero_initialized_node();
  rc = rclc_node_init_default(&my_node, "lifecycle_node", "rclc", &support);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_node_init_default\n");
    return -1;
  }
  
  // make it a lifecycle node
  printf("make it a lifecycle node...\n");
  rcl_lifecycle_state_machine_t state_machine_ = rcl_lifecycle_get_zero_initialized_state_machine();
  rclc_lifecycle_node_t lifecycle_node;
  rc = rclc_make_node_a_lifecycle_node(
    &lifecycle_node,
    &my_node,
    &state_machine_,
    &allocator);
  if (rc != RCL_RET_OK) {
    printf("Error in creating lifecycle node.\n");
    return -1;
  }
    
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
  rc = rcl_lifecycle_node_fini(&lifecycle_node, &allocator);
  rc += rclc_support_fini(&support);

  if (rc != RCL_RET_OK) {
    printf("Error while cleaning up!\n");
    return -1;
  }
  return 0;
}
