# The rclc_lifecycle package

## Overview

The rclc_lifecycle package is a [ROS 2](http://www.ros2.org/) package and provides convenience functions to bundle a ROS Client Library (RCL) node with the ROS 2 Node Lifecycle state machine in the C programming language, similar to the [rclcpp Lifecycle Node](https://github.com/ros2/rclcpp/blob/master/rclcpp_lifecycle/include/rclcpp_lifecycle/lifecycle_node.hpp) for C++.

The quality declaration is available in [QUALITY_DECLARATION.md](QUALITY_DECLARATION.md).
## API

The API of the RCLC Lifecycle Node can be divided in several phases: Initialization, Running and Clean-Up.

### Initialization

Creation of a lifecycle node as a bundle of an rcl node and the rcl Node Lifecycle state machine.

```C
#include "rclc_lifecycle/rclc_lifecycle.h"

rcl_allocator_t allocator = rcl_get_default_allocator();
rclc_support_t support;
rcl_ret_t rc;

// create rcl node
rc = rclc_support_init(&support, argc, argv, &allocator);
rcl_node_t my_node = rcl_get_zero_initialized_node();
rc = rclc_node_init_default(&my_node, "lifecycle_node", "rclc", &support);

// rcl state machine
rcl_lifecycle_state_machine_t state_machine_ =   
  rcl_lifecycle_get_zero_initialized_state_machine();
...

// create the lifecycle node
rclc_lifecycle_node_t lifecycle_node;
rcl_ret_t rc = rclc_make_node_a_lifecycle_node(
  &lifecycle_node,
  &my_node,
  &state_machine_,
  &allocator);
```

Optionally create hooks for lifecycle state changes.

```C
// declare callback
rcl_ret_t my_on_configure() {
  printf("  >>> lifecycle_node: on_configure() callback called.\n");
  return RCL_RET_OK;
}
...

// register callbacks
rclc_lifecycle_register_on_configure(&lifecycle_node, &my_on_configure);
```

### Running

Change states of the lifecycle node, e.g.

```C
bool publish_transition = true;
rc += rclc_lifecycle_change_state(
  &lifecycle_node,
  lifecycle_msgs__msg__Transition__TRANSITION_CONFIGURE,
  publish_transition);
rc += rclc_lifecycle_change_state(
  &lifecycle_node,
  lifecycle_msgs__msg__Transition__TRANSITION_ACTIVATE,
  publish_transition);
...
```

Except for error processing transitions, transitions are usually triggered from outside, e.g., by ROS 2 services.

### Cleaning Up

To clean everything up, simply do

```C
rc += rcl_lifecycle_node_fini(&lifecycle_node, &allocator);
```

## Example

An example, how to use the RCLC Lifecycle Node is given in the file `lifecycle_node.c` in the [rclc_examples](../rclc_examples) package.

## Limitations

The state machine publishes state changes, however, lifecycle services are not yet exposed via ROS 2 services (tbd).
