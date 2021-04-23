#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <stdio.h>
#include <unistd.h>

#include <rclc_parameter/rclc_parameter.h>

void parameter_changed(Parameter * param)
{
    printf("Parameter %s modified.", param->name.data);
    switch (param->value.type)
    {
    case RCLC_PARAMETER_BOOL:
        printf(" New value (bool) %d", param->value.bool_value);
        break;
    case RCLC_PARAMETER_INT:
        printf(" New value (int) %ld", param->value.integer_value);
        break;
    case RCLC_PARAMETER_DOUBLE:
        printf(" New value (double) %f", param->value.double_value);
        break;
    default:
        break;
    }
    printf("\n");
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
    rclc_parameter_server_init_default(&param_server, &node);

    // Create executor
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER, &allocator);
    rclc_executor_add_parameter_server(&executor, &param_server, parameter_changed);

    // Add parameters
    rclc_add_parameter(&param_server, "test_param1", RCLC_PARAMETER_BOOL);
    rclc_add_parameter(&param_server, "test_param2", RCLC_PARAMETER_INT);
    rclc_add_parameter(&param_server, "test_param3", RCLC_PARAMETER_DOUBLE);

    bool param1;
    int64_t param2;
    double param3;

    rclc_parameter_get_bool(&param_server, "test_param1", &param1);
    rclc_parameter_get_int(&param_server, "test_param2", &param2);
    rclc_parameter_get_double(&param_server, "test_param3", &param3);

    rclc_parameter_set(&param_server, "test_param1", (bool) false);
    rclc_parameter_set(&param_server, "test_param2", (int) 10);
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
