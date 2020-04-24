General information about this repository, including legal information, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.

# The rclc_examples package

The rclc_examples package provides examples for using the RCLC-Exector and convenience functions.
- [example_executor.c](src/example_executor.c) provides the example for the RCLC-Executor. It creates one publisher and one subscriber and configures the RCLC-Executor accordingly. Then the spin_some() function is demonstrated.
- [example_executor_convenience.c](src/example_executor_convenience.c) provides the example for the RCLC-Executor with the convenience functions from rclc. It creates one publisher and one subscriber and configures the RCLC-Executor accordingly. Then the spin_some() function is demonstrated.
- [example_executor_trigger.c](src/example_executor_trigger.c) demonstrates the trigger condition of the RCLC-Executor.

The reduction of code lines for configuring the necessary RCL objects for RCLC-Executor directly with RCL objects compared to using the convenience functions is about 24%:
- example_executor.c: 92 LoC (lines 56-148)
- example_executor_convenience.c: 70 LoC (line 17 + lines 57-126)

 counting only the lines of code in which the RCL objects are defined).

## Example RCLC-Executor using RCL objects directly
**Step 1** Setup ROS 2 Workspace

Open a terminal with ROS 2 workspace. Assuming that the ROS 2 installation resides in `/opt/ros/eloquent`, setup the ROS2 environment by:
```C
~$ source /opt/ros/eloquent/setup.bash
```

**Step 2** Build the package
Download and build the the packages `rclc` and `rclc_examples` in a workspace (for example `ros2_ws`). Then source the workspace:
```C
~/ros2_ws/$ colcon build --packages-up-to rclc_examples
~/ros2_ws/$ source ./install/local_setup.bash
```
It should build these packages:
- rcl_yaml_param_parser
- rcl
- rclc
- rclc_examples


**Step 3** Run the example executor.

The binary of the example is `example_executor`.

```C
~/ros2_ws/$ ros2 run rclc_examples  example_executor
```
The publisher publishes the message `Hello World!`in `topic_0` at a rate of 1Hz and the subscriber prints out in the callback `Callback: I heard: Hello World!`.

You should see the following output:

```C
Created timer with timeout 1000 ms.
Created subscriber topic_0:
Debug: number of DDS handles: 2
Published message Hello World!
Callback: I heard: Hello World!
Published message Hello World!
Callback: I heard: Hello World!
Published message Hello World!
Callback: I heard: Hello World!
Published message Hello World!
Callback: I heard: Hello World!
Published message Hello World!
Callback: I heard: Hello World!
```



## Example RCLC-Executor with convenience functions
**Step 1** Setup ROS 2 Workspace

Open a terminal with ROS 2 workspace. Assuming that the ROS 2 installation resides in `/opt/ros/eloquent`, setup
the ROS2 environment by:
```C
~$ source /opt/ros/eloquent/setup.bash
```

**Step 2** Build the package
Download and build the the packages `rclc` and `rclc_examples` in a workspace (for example `ros2_ws`). Then source the workspace:
```C
~/ros2_ws/$ colcon build --packages-up-to rclc_examples
~/ros2_ws/$ source ./install/local_setup.bash
```
It should build these packages:
- rcl_yaml_param_parser
- rcl
- rclc
- rclc_examples

**Step 3** Run the example executor with the convenience functions from the package rclc.

The binary of the example is `example_executor_convenience`.

```C
~/ros2_ws/$ ros2 run rclc_examples  example_executor_convenience
```
The same setup as in the example_executor, just using the RCLC convenience functions. You should see the exact same output:

```C
Created timer with timeout 1000 ms.
Created subscriber topic_0:
Debug: number of DDS handles: 2
Published message Hello World!
Callback: I heard: Hello World!
Published message Hello World!
Callback: I heard: Hello World!
Published message Hello World!
Callback: I heard: Hello World!
Published message Hello World!
Callback: I heard: Hello World!
Published message Hello World!
Callback: I heard: Hello World!
```

## Example RCLC-Executor with trigger function

**Step 1, Step 2**
To setup ROS2 workspace and build the package refer to Step 1 and Step 2 in the [Example RCLC-Executor](#example-rclc-executor).

**Step 3**
This example implements two RCLC Executors, one for publishing `executor_pub`and one for subscribing messages `executor_sub`.

The Executor `executor_pub` publishes string `topic_0` every 100ms (using a timer with 100ms) and an integer `topic_1` every 1000ms (using a timer with 1000ms).

With the trigger condition `rclc_executor_trigger_any` this executor publishes whenenver any timer is ready.

Executor `executor_sub` has two subscriptions, `my_string_sub` and `my_int_sub` subscribing to `topic_0` and `topic_1`, respectivly.

With the trigger condition `rclc_executor_trigger_all` this executor starts evaluating the callbacks only when both messages have arrived. To make this clearly visible, we set the quality of service parameter of the length of the DDS-queue to 0 for subscription `my_string_sub`, which subscribes to `topic_0` with the higher rate.

```C
my_subscription_options.qos.depth = 0
rc = rcl_subscription_init(
  &my_string_sub,
  &my_node,
  my_type_support,
  topic_name,
  &my_subscription_options);
```

Consequently, the messages "in between" are lost.

The binary of the example is `example_executor_trigger`. You run the example with:
```C
~/ros2_ws/$ ros2 run rclc_examples  example_executor_trigger
```
 Then you should see the following output:

```C
Created timer 'my_string_timer' with timeout 100 ms.
Created 'my_int_timer' with timeout 1000 ms.
Created subscriber topic_0:
Created subscriber topic_1:
Executor_pub: number of DDS handles: 2
Executor_sub: number of DDS handles: 2
Published: Hello World! 0
Published: Hello World! 1
Published: Hello World! 2
Published: Hello World! 3
Published: Hello World! 4
Published: Hello World! 5
Published: Hello World! 6
Published: Hello World! 7
Published: Hello World! 8
Published: Hello World! 9
Published: 0
Callback 1: Hello World! 9  <---
Callback 2: 0               <---
Published: Hello World! 10
Published: Hello World! 11
Published: Hello World! 12
Published: Hello World! 13
Published: Hello World! 14
Published: Hello World! 15
Published: Hello World! 16
Published: Hello World! 17
Published: Hello World! 18
Published: Hello World! 19
Published: 1
Callback 1: Hello World! 19 <---
Callback 2: 1               <---
```
The results show, that the callbacks are triggered together, only when the integer message `topic_1` was published and received. At that moment the current string message of the `topic_0` is processed as well.
