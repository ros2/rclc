#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <stdio.h>
#include <unistd.h>

#include <rclc_parameter/rclc_parameter.h>

void parameter_changed(void * param_server, const char **names, size_t size)
{
    (void) param_server;

    printf("%ld Parameters modified: ", size);
    for (size_t i = 0; i < size; i++)
    {
        printf("%s\n", names[i]);
    }
}

int main()
{
    rcl_ret_t rc;
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // Create init_options
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    // Create node
    rcl_node_t node;
    rclc_node_init_default(&node, "param_test", "", &support);

    // Create parameter service
    rclc_parameter_server_t param_server;
    rclc_parameter_server_init_default(&param_server, 4, &node);

    // Add parameters
    rclc_add_parameter(&param_server, "test_param1", 1);
    rclc_add_parameter(&param_server, "test_param2", 2);
    rclc_add_parameter(&param_server, "test_param3", 3);

    // Create executor
    rclc_executor_t executor;
    size_t handle_number = 6;
    rclc_executor_init(&executor, &support.context, handle_number, &allocator);
    rclc_executor_add_parameter_server(&executor, &param_server, parameter_changed);

    // Modify parameters
    bool param1;
    int64_t param2;
    double param3;

    rclc_parameter_get_bool(&param_server, "test_param1", &param1);
    rclc_parameter_get_int(&param_server, "test_param2", &param2);
    rclc_parameter_get_double(&param_server, "test_param3", &param3);

    rclc_parameter_set(&param_server, "test_param1", (bool) false);
    rclc_parameter_set(&param_server, "test_param2", (int) -50);
    rclc_parameter_set(&param_server, "test_param3", (double) 0.01);

    rclc_parameter_get_bool(&param_server, "test_param1", &param1);
    rclc_parameter_get_int(&param_server, "test_param2", &param2);
    rclc_parameter_get_double(&param_server, "test_param3", &param3);

    rclc_executor_spin(&executor);

    // clean up
    rc = rclc_executor_fini(&executor);
    rc += rclc_parameter_server_fini(&param_server, &node);
    rc += rcl_node_fini(&node);

    if (rc != RCL_RET_OK) {
        printf("Error while cleaning up!\n");
        return -1;
    }
    return 0;
}
