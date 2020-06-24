# The rclc_lifecycle package

## Overview

The rclc_lifecycle package is a [ROS 2](http://www.ros2.org/) package, which provides convenience functions to create ROS Client Library(RCL) data types and an RCLC Lifecycle Node in the C programming language.
The convenience functions are a thin API layer on top of RCL-layer to bunde an rcl node with the ROS 2 default state machine similar to the rclcpp Lifecycle Node.#

The rclc Lifecycle Node is a ROS 2 Lifecycle Node implemented based on and for the rcl API, for applications written in the C language.

## API

The API of the RCLC Lifecycle Node can be divided in several phases: Initialization, Running and Clean-Up.

### Initialization

Creation of a lifecycle node as a bundle of an rcl node and the rcl state machine.

```C
#include "rclc_lifecycle/rclc_lifecycle.h"

// rcl node and node options
rcl_node_t my_node = rcl_get_zero_initialized_node();
rcl_node_options_t node_ops = rcl_node_get_default_options();
...

// rcl state machine
rcl_lifecycle_state_machine_t state_machine_ =   
  rcl_lifecycle_get_zero_initialized_state_machine();
...

// create the lifecycle node
rclc_lifecycle_node_t lifecycle_node;
rcl_ret_t rc = rclc_make_node_a_lifecycle_node(
  &my_node,
  &state_machine_,
  &node_ops);
```

Optionally create hooks for lifecycle state changes.

```C
// declare callback
rcl_ret_t on_configure() {
  printf("  >>> lifecycle_node: on_configure() callback called.\n");
  return RCL_RET_OK;
}
...

// register callbacks
rclc_lifecycle_register_on_configure(&lifecycle_node, &on_configure);
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

### Cleaning Up

To clean everything up, simply do

```C
rc += rcl_lifecycle_node_fini(&lifecycle_node, &node_ops);
```

## Example

An example, how to use the RCLC Lifecycle Node is given in the file `lifecycle_node.c` in the [rclc_examples](../rclc_examples) package.

## Limitations

The state machine publishes state changes, however, lifecycle services are not yet exposed via ROS 2 services (tbd).
