# The rclc package

The package rclc is a [ROS 2](http://www.ros2.org/) package, which provides convenience functions to create rcl data types and an Executor. As a thin API layer on top of RCL you can create publishers, subscribers, timers and nodes with a one-liner like in rclcpp. The Executor provides an API to add subscriptions and timers as well as some spin() functions similar to the ones in rclcpp.

API:
- rclc_init()
- rclc_node_init_default()
- rclc_publisher_init_default()
- rclc_subscription_init_default()
- rclc_timer_init_default()


A complete code example with the `rclc` convenience functions and the `let-executor` is provided in [test](./test)


# The LET Executor

This real-time executor is targeted as C API  based on the RCL layer for ROS2 applications running
on a micro-controller[micro-ROS]. The rcl_executor is a small [ROS 2](http://www.ros2.org/) package
for providing real-time scheduling. Currently the package supports a static order scheduler with
logical-execution-time (LET) semantics.

Static order scheduling refers to the fact, that during configuration the order of DDS handles,
e.g. subscriptions and timers, is specfied. If at runtime multiple handles are ready, e.g.
subscriptions received new messages or timers are ready, then these handles are always processed
in the order as specified during configuration.

LET refers to a semantic, where input data is first read for all handles, then all handels are
processed. The benefit for static order scheduling, is that provides a deterministic execution in
the case when multiple data is available. The benefit of LET is that there is no interference of
input data for all handles processed. For example, if a timer callback evaluates the data processed
by a subscription callback and the order of these is not given, then the timer callback could
sometimes process the old data and sometimes the new. By reading all data first (and locally
copying it), this dependence is removed.
The benefit is very low synchronization effort because interference between input data is avoided
[EK2018] [BP2017]. For more details about this Real Time Executor see
[micro-ROS website](https://micro-ros.github.io/docs/concepts/client_library/real-time_executor/).

## LET semantics 

The LET-semantic is implemented by, 
  1) waiting for new data from DDS (rcl_wait()), then 
  2) requesting all new data for all ready handles from DDS (rcl_take()) and then
  3) executing all ready handles in the order specified by the user. 

 Waiting until all callbacks are processed and then publishing them, has been omitted in this
 implementation of the LET semantics, because this would require potentially unbounded buffers
 for publishing messages. However, this sequence already guarantees no interference between input
data for the handles.

## Memory allocation

Special care has been taken about memory allocation. Memory is only allocated in the configuration
phase, i.e. by specifying how many handles shall be executed, while at running phase, no memory is
allocated by the RCL Executor.

## User Interface

The API of the RCL-Executor provides functions, like `add_subscription` and `add_timer`, to add
rcl-subscriptions and rcl-timers to the Executor. The order, in which the user adds these handles
to the Executor, determines later on the execution order when new data is available for these
handles. The executor is activated by calling the `spin_some`, `spin_period()`, `spin_one_period` or `spin()` method.

The API of the static LET scheduler provide the following functions for configuration, defining the
execution order, running the scheduler and cleaning-up:

### API Function Overview
**Initialization**
- rclc_let_executor_init()
- rclc_let_executor_set_timeout()

**Configuration**
- rclc_let_executor_add_subscription()
- rclc_let_executor_add_timer()

**Running the Executor**
- rclc_let_executor_spin_some()
- rclc_let_executor_spin_one_period()
- rclc_let_executor_spin_period()
- rclc_let_executor_spin()

**Clean-up memory**
- rclc_let_executor_fini()

### API Description

In the function `rclc_let_executor_init`, the user must specify among other things how many handles
shall be scheduled.

TODO (jan): timeout for spinning / not for rcl_wait()

The function `rclc_let_executor_set_timeout` is an optional configuration function, which defines
the timeout for calling rcl_wait(), i.e. the time to wait for new data from the DDS queue. The
default timeout is 100ms.

The functions `rclc_let_executor_add_subscription` and `rclc_let_executor_add_timer` add the
corresponding handle to the executor. The maximum number of handles is defined in
`rclc_let_executor_init`. The sequential order of these function calls defines the static
execution order in the spin-functions.

The function `rclc_let_executor_spin_some` checks for new data from the DDS queue once. It first
copies all data into local data structures and then executes all handles according the specified
order. This implements the LET semantics.

The function `rclc_let_executor_spin_period` calls `rclc_let_executor_spin_some` periodically
(as defined with the argument period) as long as the ROS system is alive.

The function `rclc_let_executor_spin` calls `rclc_let_executor_spin_some` indefinitely as long
as the ROS system is alive. This might create a high performance load on your processor.

The function `rlce_executor_fini` frees the dynamically allocated memory of the executor.

## Example

An example, how to use the LET Executor with RCL objects given in the package:
[rclc_examples](https://github.com/micro-ROS/rclc/rclc_examples). (Branch feature/new_api_and_LET_executor)

## Limitations: 

- support for subscriptions and timers (services, clients, guard conditions are not supported yet)

## References

[micro-ROS] [micro-ROS project](https://micro-ros.github.io/) 

[EK2018] R. Ernst, S. Kuntz, S. Quinton, M. Simons: The Logical Execution Time Paradigm: New Perspectives for Multicore Systems, February 25-28 2018 (Dagstuhl Seminar 18092). [Paper](http://drops.dagstuhl.de/opus/volltexte/2018/9293/pdf/dagrep_v008_i002_p122_18092.pdf)

[BP2017] A. Biondi, P. Pazzaglia, A. Balsini, M. D. Natale: Logical Execution Time Implementation and Memory Optimization Issues in AUTOSAR Applications for Multicores, International Worshop on Analysis Tools and Methodologies for Embedded and Real-Time Systems (WATERS2017), Dubrovnik, Croatia.[Paper](https://pdfs.semanticscholar.org/4a9e/b9a616c25fd0b4a4f7810924e73eee0e7515.pdf)



## Guide to setup the let-executor with RCL API

You find the complete source code in the package `rclc_examples`, file example_executor.c.

**Step 1:** <a name="Step1"> </a> Include the `let_executor.h` from the rclc package and other headers in your C code.
As well as some global data structured used by the publisher and subscriber.

```C
#include <stdio.h>
#include <std_msgs/msg/string.h>
#include "rclc/let_executor.h"
// these data structures for the publisher and subscriber are global, so that
// they can be configured in main() and can be used in the corresponding callback.
rcl_publisher_t my_pub;
std_msgs__msg__String pub_msg;
std_msgs__msg__String sub_msg;
```

**Step 2:** <a name="Step2"> </a> Define a subscription callback `my_subscriber_callback`.

```C
void my_subscriber_callback(const void * msgin)
{
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  if (msg == NULL) {
    printf("Callback: msg NULL\n");
  } else {
    printf("Callback: I heard: %s\n", msg->data.data);
  }
}
```

**Step 3:** <a name="Step3"> </a> Define a timer callback `my_timer_callback`.

```C
#define UNUSED(x) (void)x;
void my_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  rcl_ret_t rc;
  UNUSED(last_call_time);
  if (timer != NULL) {
    //printf("Timer: time since last call %d\n", (int) last_call_time);
    rc = rcl_publish(&my_pub, &pub_msg, NULL);
    if (rc == RCL_RET_OK) {
      printf("Published message %s\n", pub_msg.data.data);
    } else {
      printf("timer_callback: Error publishing message %s\n", pub_msg.data.data);
    }
  } else {
    printf("timer_callback Error: timer parameter is NULL\n");
  }
}
```

**Step 4:** <a name="Step4"> </a> Create a rcl_node in main function.

```C
int main(int argc, const char * argv[])
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
  rcl_node_t my_node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rc = rcl_node_init(&my_node, "node_0", "let_executor_examples", &context, &node_ops);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_node_init\n");
    return -1;
  }
```
**Step 5:** <a name="Step5"> </a> Create an rcl_publisher `my_pub` which publishes messages using the rcl_timer `my_timer`.
```C
  // create a publisher to publish topic 'topic_0' with type std_msg::msg::String
  // my_pub is global, so the timer_callback access this publisher.
  const char * topic_name = "topic_0";
  rcl_publisher_options_t pub_options = rcl_publisher_get_default_options();
  rc = rcl_publisher_init(
    &my_pub,
    &my_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    topic_name,
    &pub_options);
  if (RCL_RET_OK != rc) {
    printf("Error in rcl_publisher_init %s.\n", topic_name);
    return -1;
  }


  // create a timer, which will call the publisher every 'timer_timeout' ms in the 'my_timer_callback'
  rcl_clock_t clock;
  rc = rcl_clock_init(RCL_STEADY_TIME, &clock, &allocator);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_clock_init.\n");
    return -1;
  }
  rcl_timer_t my_timer = rcl_get_zero_initialized_timer();
  const unsigned int timer_timeout = 1000;
  rc = rcl_timer_init(
    &my_timer,
    &clock,
    &context,
    RCL_MS_TO_NS(timer_timeout),
    my_timer_callback,
    allocator);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_timer_init.\n");
    return -1;
  } else {
    printf("Created timer with timeout %d ms.\n", timer_timeout);
  }

  // assign message to publisher
  std_msgs__msg__String__init(&pub_msg);
  const unsigned int PUB_MSG_SIZE = 20;
  char pub_string[PUB_MSG_SIZE];
  snprintf(pub_string, 13, "%s", "Hello World!");
  rosidl_generator_c__String__assignn(&pub_msg, pub_string, PUB_MSG_SIZE);
```
**Step 6:** <a name="Step6"> </a> Create an rcl_subscription `my_sub` with the topic `topic_name`.

```C
  // create subscription
  rcl_subscription_t my_sub = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t my_subscription_options = rcl_subscription_get_default_options();
  const rosidl_message_type_support_t * my_type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);

  rc = rcl_subscription_init(
    &my_sub,
    &my_node,
    my_type_support,
    topic_name,
    &my_subscription_options);

  if (rc != RCL_RET_OK) {
    printf("Failed to create subscriber %s.\n", topic_name);
    return -1;
  } else {
    printf("Created subscriber %s:\n", topic_name);
  }

  // one string message for subscriber
  std_msgs__msg__String__init(&sub_msg);
```

**Step 7:** <a name="Step7"> </a> Create an LET-executor and initialize it with the ROS context
(`context`), number of handles (`2`) and use the `allocator` for memory allocation.

The user can configure, when the callback shall be invoked: Options are `ALWAYS` and `ON_NEW_DATA`. If `ALWAYS` is selected, the callback is always called, even if no new data is available. In this case, the callback is given a `NULL`pointer for the argument `msgin` and the callback needs to handle this correctly. If `ON_NEW_DATA` is selected, then the callback is called only if new data from the DDS queue is available. In this case the parameter `msgin` of the callback always points to memory-allocated message.

```C
  rclc_let_executor_t executor;
  // compute total number of subsribers and timers
  unsigned int num_handles = 1 + 1;
  executor = rclc_let_executor_get_zero_initialized_executor();
  rclc_let_executor_init(&executor, &context, num_handles, &allocator);
```
**Step 8:** <a name="Step8"> </a>(Optionally) Define the blocking time when requesting new data from DDS (timeout for rcl_wait()). Here the timeout is `1000ms`.
The default timeout is 100ms.

```C
  // set timeout for rcl_wait()
  unsigned int rcl_wait_timeout = 1000;   // in ms
  rc = rclc_let_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout));
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_let_executor_set_timeout.");
  }
```

**Step 9:** <a name="Step9"> </a> Add the subscription `my_sub` (See [Step 6](#Step6)) to the `executor` with the message variable `my_msg`, in which the new data is stored and the callback function `my_subscriber_callback`(See [Step 2](#Step2)). The callback is invoked if new data is available (`ON_NEW_DATA`). 

```C
  // add subscription to executor
  rc = rclc_let_executor_add_subscription(&executor, &my_sub, &sub_msg, &my_subscriber_callback,
      ON_NEW_DATA);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_let_executor_add_subscription. \n");
  }
```

**Step 10:** <a name="Step10"> </a> Add timer `my_timer`, as defined in [Step 5](#Step5) to the `executor`. The period of the timer and the callback to call are already configured in the timer object itself.

```C
  rclc_let_executor_add_timer(&executor, &my_timer);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_let_executor_add_timer.\n");
  }
```

**Step 11:** <a name="Step11"> </a> Run the executor. As an example, we demonstrate also spin ten times with a rcl_wait-timeout of 1s. 

```C
  for (unsigned int i = 0; i < 10; i++) {
    // timeout specified in ns (here 1s)
    rclc_let_executor_spin_some(&executor, 1000 * (1000 * 1000));
  }
```

**Step 12:** <a name="Step12"> </a> Clean up memory for the LET-Executor and other other RCL objects

```C
  rc = rclc_let_executor_fini(&executor);
  rc += rcl_publisher_fini(&my_pub, &my_node);
  rc += rcl_timer_fini(&my_timer);
  rc += rcl_subscription_fini(&my_sub, &my_node);
  rc += rcl_node_fini(&my_node);
  rc += rcl_init_options_fini(&init_options);
  if (rc != RCL_RET_OK) {
    printf("Error while cleaning up!\n");
    return -1;
  }
  return 0;
}  // main
```

**Output** If you run the example executor, you should see the following output:
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