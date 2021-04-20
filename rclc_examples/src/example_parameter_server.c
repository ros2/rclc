#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <stdio.h>
#include <unistd.h>

#include <rclc_parameter/rclc_parameter.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)){printf( \
                                                                               "Failed status on line %d: %d. Aborting.\n", __LINE__, \
                                                                               (int)temp_rc);} \
}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)){printf( \
                                                                                   "Failed status on line %d: %d. Continuing.\n", __LINE__, \
                                                                                   (int)temp_rc);} \
}

void parameter_changed(void * param_server, const char **names, size_t size)
{
    (void) param_server;

    printf("%ld Parameters modified: ", size);
    for (size_t i = 0; i < size; i++)
    {
        printf("%s\n", names[i]);
    }
}

void parameter_2_changed(void *param_server, const char *name)
{
    int64_t param_value = 0;
    if (!rclc_parameter_get_int(param_server, name, &param_value))
    {
        printf("Parameter %s value modified to: %ld", name, param_value);
    }
}

int main()
{
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // Create init_options
    rclc_support_t support;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "param_test", "", &support));

    // Create parameter service
    rcl_parameter_server_t param_server;
    RCCHECK(rclc_parameter_server_init_default(&param_server, 4, &node));

    // Add parameter change callbacks
    RCCHECK(rclc_parameter_server_add_callback(&param_server, "test_param2", parameter_2_changed));
    RCCHECK(rclc_parameter_server_add_callback_all(&param_server, parameter_changed));

    // Add parameters
    RCCHECK(rclc_add_parameter(&param_server, "test_param1", 1));
    RCCHECK(rclc_add_parameter(&param_server, "test_param2", 2));
    RCCHECK(rclc_add_parameter(&param_server, "test_param3", 3));

    bool param1;
    int64_t param2;
    double param3;

    RCCHECK(rclc_parameter_get_bool(&param_server, "test_param1", &param1));
    RCCHECK(rclc_parameter_get_int(&param_server, "test_param2", &param2));
    RCCHECK(rclc_parameter_get_double(&param_server, "test_param3", &param3));

    RCCHECK(rclc_parameter_set_bool(&param_server, "test_param1", 0));
    RCCHECK(rclc_parameter_set_int(&param_server, "test_param2", -50));
    RCCHECK(rclc_parameter_set_double(&param_server, "test_param3", 0.01));

    RCCHECK(rclc_parameter_get_bool(&param_server, "test_param1", &param1));
    RCCHECK(rclc_parameter_get_int(&param_server, "test_param2", &param2));
    RCCHECK(rclc_parameter_get_double(&param_server, "test_param3", &param3));

    // Create executor
    rclc_executor_t executor;
    size_t handle_number = 6;
    RCCHECK(rclc_executor_init(&executor, &support.context, handle_number, &allocator));
    RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server));

    rclc_executor_spin(&executor);

    RCCHECK(rclc_parameter_server_fini(&param_server, &node));
    RCCHECK(rcl_node_fini(&node));

    return 0;
}
