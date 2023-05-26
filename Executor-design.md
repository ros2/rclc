# Notice
In this feature branch (feature/restructured-executor), a new modular rclc-Executor is developed. It will provide the following functionality:

- single-threaded Executor
- multi-threaded Executor
- AUTOSAR-like Executor (for periodic operating systems)


# Design of modular rclc Executor

- one base rclc package with single-threaded Executor
- several packages for additional configurations (multi-threaded, AUTOSAR-like OS)
- one configuration function, ...
- each method in rclc library has a function pointer to specialization 
- this fn-pointer is set in the configuration function



# Comparison of Executor Implementations

This table gives an overview of the differences between the Executor implementations. 
Each function in the rclc-library might have a function pointer to a specialization function, e.g. work that is specific to the particular Executor. 

| Function name                  | Single-threaded | Multi-threaded | AUTOSAR-like |
|:--                             |  :--            |  :--           |  :--         |
| rclc_executor_init             | none            | none           | none         |
| rclc_executor_add_subscription | none            | none           | none         |
| rclc_executor_add_timer        | none            | none           | none         |
| _rclc_check_for_new_data       | none            | none           | none         |


