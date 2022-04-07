# The rclc parameter package

ROS 2 parameters allow the user to create variables on a node and manipulate/read them with different ROS 2 commands. Further information about ROS 2 parameters can be found [here](https://docs.ros.org/en/galactic/Tutorials/Parameters/Understanding-ROS2-Parameters.html)

This package provides the rclc with parameter server instances with full ROS 2 parameters client compatibility. A parameters client for rclc has not been implemented (yet).
Ready to use code related to this tutorial can be found in [`rclc/rclc_examples/src/example_parameter_server.c`](../rclc_examples/src/example_parameter_server.c).

## Table of contents
*   [Initialization](#initialization)
*   [Memory requirements](#memory-requirements)
*   [Callback](#callback)
*   [Add a parameter](#add-a-parameter)
*   [Delete a parameter](#delete-a-parameter)
*   [Parameters description](#parameters-description)
*   [Cleaning up](#cleaning-up)

## Initialization

- Default initialization:
    ```c
    // Parameter server object
    rclc_parameter_server_t param_server;
    // Initialize parameter server with default configuration
    rcl_ret_t rc = rclc_parameter_server_init_default(&param_server, &node);

    if (RCL_RET_OK != rc) {
      ... // Handle error
      return -1;
    }
    ```

- Custom options:

  The following options can be configured:
  - notify_changed_over_dds: Publish parameters events to the rest of nodes.
  - max_params: Maximum number of parameters allowed on the `rclc_parameter_server_t` object.
  - allow_undeclared_parameters: Allows creation of parameters from external parameters clients. A new parameter will be created if a `set` operation is requested on a non-existing parameter.
  - low_mem_mode: Reduces the memory used by the parameter server, functionality constrains are applied.  

    ```c
    // Parameter server object
    rclc_parameter_server_t param_server;

    // Initialize parameter server options
    const rclc_parameter_options_t options = {
        .notify_changed_over_dds = true,
        .max_params = 4,
        .allow_undeclared_parameters = true,
        .low_mem_mode = false; };

    // Initialize parameter server with configured options
    rcl_ret_t rc = rclc_parameter_server_init_with_option(&param_server, &node, &options);
    if (RCL_RET_OK != rc) {
      ... // Handle error
      return -1;
    }
    ```

- Low memory mode:

    This mode ports the parameters functionality to memory constrained devices. The following constrains are applied:
    - Request size limited to 1 parameter on Set, Get, Get types and Describe services.
    - List parameters request has no prefixes enabled nor depth.
    - Parameter description strings not allowed, `rclc_add_parameter_description` is disabled.

    Memory benchmark on `STM32F4` for 7 parameters with `RCLC_PARAMETER_MAX_STRING_LENGTH = 50` and `notify_changed_over_dds = true`:
    - Full mode: 11736 B
    - Low memory mode: 4160 B

## Memory requirements

The parameter server uses five services and an optional publisher. These need to be taken into account on the `rmw-microxredds` package memory configuration:

```yaml
# colcon.meta example with memory requirements to use a parameter server
{
    "names": {
        "rmw_microxrcedds": {
            "cmake-args": [
                "-DRMW_UXRCE_MAX_NODES=1",
                "-DRMW_UXRCE_MAX_PUBLISHERS=1",
                "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=0",
                "-DRMW_UXRCE_MAX_SERVICES=5",
                "-DRMW_UXRCE_MAX_CLIENTS=0"
            ]
        }
    }
}
```

At runtime, the variable `RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER` defines the rclc Executor handles required for a parameter server:

```c
// Executor init example with the minimum RCLC executor handles required
rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
rc = rclc_executor_init(
    &executor, &support.context,
    RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER, &allocator);
```

## Callback

When adding the parameter server to the executor, a callback can be configured. This callback will be executed on the following events:  
- Parameter value change: Internal and external parameter set on existing parameters.
- Parameter creation: External parameter set on unexisting parameter if `allow_undeclared_parameters` is set.
- Parameter delete: External parameter delete on existing parameter.
- The user can allow or reject this operations using the `bool` return value.

Callback parameters:
- `old_param`: Parameter actual value, `NULL` for new parameter request.
- `new_param`: Parameter new value, `NULL` for parameter removal request.
- `context`: User context, configured on `rclc_executor_add_parameter_server_with_context`.

```c
bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context)
{
    (void) context;

    if (old_param == NULL)
    {
        printf("Creating new parameter %s\n", new_param->name.data);
    }
    else if (new_param == NULL)
    {
        printf("Deleting parameter %s\n", old_param->name.data);
    }
    else
    {
        printf("Parameter %s modified.", old_param->name.data);
        switch (old_param->value.type)
        {
            case RCLC_PARAMETER_BOOL:
                printf(
                " Old value: %d, New value: %d (bool)", old_param->value.bool_value,
                new_param->value.bool_value);
                break;
            case RCLC_PARAMETER_INT:
                printf(
                " Old value: %ld, New value: %ld (int)", old_param->value.integer_value,
                new_param->value.integer_value);
                break;
            case RCLC_PARAMETER_DOUBLE:
                printf(
                " Old value: %f, New value: %f (double)", old_param->value.double_value,
                new_param->value.double_value);
                break;
            default:
                break;
        }

        printf("\n");
    }

    return true;
}
```

Parameters modifications are disabled while this callback is executed, causing the following methods to return `RCLC_PARAMETER_DISABLED_ON_CALLBACK` if they are invoked:
- rclc_add_parameter
- rclc_delete_parameter
- rclc_parameter_set_bool
- rclc_parameter_set_int
- rclc_parameter_set_double
- rclc_set_parameter_read_only
- rclc_add_parameter_constraints_double
- rclc_add_parameter_constraints_integer

Once the parameter server and the executor are initialized, the parameter server must be added to the executor in order to accept parameters commands from ROS 2:
```c
// Add parameter server to executor including defined callback
rc = rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed);
```

Note that this callback is optional as its just an event information for the user. To use the parameter server without a callback:
```c
// Add parameter server to executor without callback
rc = rclc_executor_add_parameter_server(&executor, &param_server, NULL);
```

To configure the callback context:
```c
// Add parameter server to executor including defined callback and a context
rc = rclc_executor_add_parameter_server_with_context(&executor, &param_server, on_parameter_changed, &context);
```

## Add a parameter

micro-ROS parameter server supports the following parameter types:

- Boolean:
    ```c
    const char * parameter_name = "parameter_bool";
    bool param_value = true;

    // Add parameter to the server
    rcl_ret_t rc = rclc_add_parameter(&param_server, parameter_name, RCLC_PARAMETER_BOOL);

    // Set parameter value (Triggers parameter change callback)
    rc = rclc_parameter_set_bool(&param_server, parameter_name, param_value);

    // Get parameter value on param_value
    rc = rclc_parameter_get_bool(&param_server, "param1", &param_value);

    if (RCL_RET_OK != rc) {
        ... // Handle error
        return -1;
    }
    ```

- Integer:
    ```c
    const char * parameter_name = "parameter_int";
    int param_value = 100;

    // Add parameter to the server
    rcl_ret_t rc = rclc_add_parameter(&param_server, parameter_name, RCLC_PARAMETER_INT);

    // Set parameter value
    rc = rclc_parameter_set_int(&param_server, parameter_name, param_value);

    // Get parameter value on param_value
    rc = rclc_parameter_get_int(&param_server, parameter_name, &param_value);
    ```

- Double:
    ```c
    const char * parameter_name = "parameter_double";
    double param_value = 0.15;

    // Add parameter to the server
    rcl_ret_t rc = rclc_add_parameter(&param_server, parameter_name, RCLC_PARAMETER_DOUBLE);

    // Set parameter value
    rc = rclc_parameter_set_double(&param_server, parameter_name, param_value);

    // Get parameter value on param_value
    rc = rclc_parameter_get_double(&param_server, parameter_name, &param_value);
    ```

Parameters can also be created by external clients if `allow_undeclared_parameters` flag is set.
They client just needs to set a value on a unexisting parameter, the parameter will be created if the server is not full and the callback allows the operation.

*Max name size is controlled by the compile-time option `RCLC_PARAMETER_MAX_STRING_LENGTH`, default value is 50.*

## Delete a parameter
Parameters can be deleted by both, the parameter server and external clients:
```c
rclc_delete_parameter(&param_server, "param2");
```

Note that for external delete requests, the server callback will be executed, allowing the node to reject the operation.

## Parameters description

- Parameter description  
    Adds a description of a parameter and its constrains, which will be returned on a describe parameters requests:
    ```c
    rclc_add_parameter_description(&param_server, "param2", "Second parameter", "Only even numbers");
    ```

    *The maximum string size is controlled by the compilation time option `RCLC_PARAMETER_MAX_STRING_LENGTH`, default value is 50.*

- Parameter constraints  
    Informative numeric constraints can be added to int and double parameters, returning this values on describe parameters requests:
    - from_value: Start value for valid values, inclusive.
    - to_value: End value for valid values, inclusive.
    - step: Size of valid steps between the from and to bound.

        ```c
        int64_t int_from = 0;
        int64_t int_to = 20;
        uint64_t int_step = 2;
        rclc_add_parameter_constraints_integer(&param_server, "param2", int_from, int_to, int_step);

        double double_from = -0.5;
        double double_to = 0.5;
        double double_step = 0.01;
        rclc_add_parameter_constraints_double(&param_server, "param3", double_from, double_to, double_step);
        ```

    *This constrains will not be applied by the parameter server, leaving values filtering to the user callback.*

- Read-only parameters:  
    The new API offers a read-only flag. This flag blocks parameter changes from external clients, but allows changes on the server side:
    ```c
    bool read_only = true;
    rclc_set_parameter_read_only(&param_server, "param3", read_only);
    ```

## Cleaning up

To destroy an initialized parameter server:

```c
// Delete parameter server
rclc_parameter_server_fini(&param_server, &node);
```

This will delete any automatically created infrastructure on the agent (if possible) and deallocate used memory on the client side.