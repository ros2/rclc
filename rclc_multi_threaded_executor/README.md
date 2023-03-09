# The rclc multi threaded executor package

Multi-threaded Executor allows customization of callback processing in multiple threads and prioritization.

TODO [`rclc/rclc_examples/src/example_multi_threaded_executor.c`](../rclc_examples/src/example_multi_threaded_executor.c).

## Table of contents
*   [Initialization](#initialization)


## Initialization

- Default initialization:
    ```c
    // define executor
    rclc_executor_t executor;
    // configure multi-threaded executor
    rcl_ret_t rc = rclc_mult_threaded_executor_configure(&executor);

    if (RCL_RET_OK != rc) {
      ... // Handle error
      return -1;
    }
    ```
