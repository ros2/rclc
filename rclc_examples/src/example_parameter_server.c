#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <stdio.h>
#include <unistd.h>

#include <rclc_parameter/parameter_server.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)){printf( \
                                                                               "Failed status on line %d: %d. Aborting.\n", __LINE__, \
                                                                               (int)temp_rc);} \
}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)){printf( \
                                                                                   "Failed status on line %d: %d. Continuing.\n", __LINE__, \
                                                                                   (int)temp_rc);} \
}

int main(
        void)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // create init_options
    rclc_support_t support;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "param_test", "", &support));

    // create service
    rcl_parameter_server_t param_server;
    RCCHECK(rclc_parameter_server_init_default(&param_server, &node));

    // Add parameters
    RCCHECK(rclc_add_parameter_bool(&param_server, "test_param1", true));
    RCCHECK(rclc_add_parameter_int(&param_server, "test_param2", 198));
    RCCHECK(rclc_add_parameter_double(&param_server, "test_param3", 10.28));
    RCCHECK(rclc_add_parameter_string(&param_server, "test_param4", "value"));

    size_t param4_lenght;
    RCCHECK(rclc_parameter_get_string_lenght(&param_server.parameter_list, "test_param4", &param4_lenght));

    bool param1;
    int64_t param2;
    double param3;
    char param4[param4_lenght];

    RCCHECK(rclc_parameter_get_bool(&param_server.parameter_list, "test_param1", &param1));
    RCCHECK(rclc_parameter_get_int(&param_server.parameter_list, "test_param2", &param2));
    RCCHECK(rclc_parameter_get_double(&param_server.parameter_list, "test_param3", &param3));
    RCCHECK(rclc_parameter_get_string(&param_server.parameter_list, "test_param4", param4));

    printf("Test parameters Add\n");
    printf("Bool parameter: %d\n", param1);
    printf("Int parameter: %ld\n", param2);
    printf("Double parameter: %f\n", param3);
    printf("String parameter: %s\n\n", param4);

    RCCHECK(rclc_parameter_set_bool(&param_server.parameter_list, "test_param1", 0));
    RCCHECK(rclc_parameter_set_int(&param_server.parameter_list, "test_param2", -50));
    RCCHECK(rclc_parameter_set_double(&param_server.parameter_list, "test_param3", 0.01));
    RCCHECK(rclc_parameter_set_string(&param_server.parameter_list, "test_param4", "new_test"));

    RCCHECK(rclc_parameter_get_bool(&param_server.parameter_list, "test_param1", &param1));
    RCCHECK(rclc_parameter_get_int(&param_server.parameter_list, "test_param2", &param2));
    RCCHECK(rclc_parameter_get_double(&param_server.parameter_list, "test_param3", &param3));
    RCCHECK(rclc_parameter_get_string(&param_server.parameter_list, "test_param4", param4));

    printf("Test parameters Set Get\n");
    printf("Bool parameter: %d\n", param1);
    printf("Int parameter: %ld\n", param2);
    printf("Double parameter: %f\n", param3);
    printf("String parameter: %s\n", param4);

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
