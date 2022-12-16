# The rclc dispatcher executor

Supports multi-threaded executor and prioritization of callbacks.

TODO
- Functionality
  - rclc_dispatcher_executor_spin_some() is necessary only spin once for testing
- Unit tests:
  - initialization
  - functional test
  - how to check that executor has been initialized by rclc_executor_init?

- Example:
  - check priority of the thread
  - design meaningful functional test (two prios one gets more work done)
  - priority is not set correctly for worker thread

- Clean-up
  - reduce dependencies in CmakeLists.txt

- Write documentation here in README
  - call only one _init function?
  - introspection, display which is the actual priority of the thread

## Table of contents
*   [Initialization](#initialization)
*   [Configuration](#configuration)
*   [Runtime](#runtime)
*   [Cleaning up](#cleaning-up)

## Initialization

- Default initialization:
    ```c
TODO
    ```

## Configuration


TODO

## Runtime

TODO

```c
// xxx
TODO
```


## Cleaning up

To destroy an initialized executor:

```c
// Delete dispatching executor
TODO
```
