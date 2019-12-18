General information about this repository, including legal information, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.

# The rclc_examples package

The rclc_examples package provides an example for the LET-Exector.
- [example_executor.c](example_executor.c) provides the example for the LET-Executor. It creates one publisher and one subscriber and configures the let-executor accordingly. Then the spin_some() function is demonstrated. 

## Example LET-Executor
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

**Step 3** Run the example executor.

The binary of the example is `example_executor`.

```C
~/ros2_ws/$ ros2 run rclc_examples  example_executor
```
 Then you should see the following output:

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
