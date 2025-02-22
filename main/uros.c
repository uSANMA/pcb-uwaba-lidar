#include "math.h"

#include <time.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <rmw_microros/rmw_microros.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <uros_network_interfaces.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <sensor_msgs/msg/laser_scan.h>

#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#define PING_AGENT  1

#define CHECK(fn)                                                                                           \
	{                                                                                                       \
		rcl_ret_t temp_rc = fn;                                                                             \
		if ((temp_rc != RCL_RET_OK))                                                                        \
		{                                                                                                   \
			ESP_LOGE("SYSTEM-uROS", "Failed status on line: %d > returned: %d > Aborting", __LINE__, (int)temp_rc);  \
            while(1);                                                                                       \
		}                                                                                                   \
	}

static const char *TAG = "uROS";

extern void timestamp_update(void*arg);

void uros_task(void *argument);
void lidar_pub_callback();
void init_msgs_laserscan();
void ping_agent();

SemaphoreHandle_t uros_boot_lidar;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;

rclc_executor_t executor_pub_msgs;

rcl_publisher_t pub_msgs_laserscan;
rmw_publisher_allocation_t allocation;

rosidl_runtime_c__String__Sequence msgs_encoders_name_sequence;

sensor_msgs__msg__LaserScan msgs_laserscan;
/*
    std_msgs__msg__Header header;
        builtin_interfaces__msg__Time stamp;
            int32_t sec;
            uint32_t nanosec;
        rosidl_runtime_c__String frame_id;
            char * data;
            size_t size;
            size_t capacity;

    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;

    rosidl_runtime_c__float__Sequence ranges; TYPE_NAME = float
        TYPE_NAME * data;
        size_t size;
        size_t capacity;

    rosidl_runtime_c__float__Sequence intensities; TYPE_NAME = float
        TYPE_NAME * data;
        size_t size;
        size_t capacity;
*/

static const int n_handles_pub = 1; //number of handles that will be added in executor (executor_add_...)

void lidar_pub_callback() {
    ESP_LOGI(TAG,"Sending to ROS2");
    CHECK(rcl_publish(&pub_msgs_laserscan, &msgs_laserscan, &allocation));
}

void init_msgs_laserscan(){
    //https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
    //sensor_msgs__msg__LaserScan__init(&msgs_laserscan);
    //msgs_laserscan = sensor_msgs__msg__LaserScan__create();

    msgs_laserscan.header.frame_id.capacity = 12;
    msgs_laserscan.header.frame_id.size = 11;
    msgs_laserscan.header.frame_id.data = (char*) malloc(msgs_laserscan.header.frame_id.capacity * sizeof(char));

    //msgs_laserscan.header.frame_id.data = "laser_frame";
    //rosidl_runtime_c__String__init(&msgs_laserscan->header.frame_id);
    msgs_laserscan.header.frame_id = micro_ros_string_utilities_init("laser_frame");
    //msgs_laserscan->header.frame_id = micro_ros_string_utilities_set(msgs_laserscan->header.frame_id, "laser_frame");

    msgs_laserscan.angle_min = 0.0;
    msgs_laserscan.angle_max = (float)(2*M_PI) - msgs_laserscan.angle_increment;
    msgs_laserscan.angle_increment = (float)(M_PI/180);
    msgs_laserscan.time_increment = (float)msgs_laserscan.scan_time/360; // period/360 scans
    msgs_laserscan.scan_time = (float)1/7; //period scans
    msgs_laserscan.range_min = 0.12;
    msgs_laserscan.range_max = 16.0;

    msgs_laserscan.ranges.capacity = 360;
    msgs_laserscan.ranges.size = 360;
    msgs_laserscan.ranges.data = (float*) malloc(msgs_laserscan.ranges.capacity * sizeof(float));
    //msgs_laserscan.ranges.data = (float*) malloc(msgs_laserscan.ranges.capacity * sizeof(float));
    //(float*) calloc(msgs_laserscan.ranges.capacity, sizeof(float));

    //msgs_laserscan->intensities.capacity = 360;
    //msgs_laserscan->intensities.size = 360;
    //msgs_laserscan->intensities.data = (float*) malloc(msgs_laserscan->intensities.capacity * sizeof(float));
}

rcl_ret_t init_ping_struct(){
    rclc_support_t ping_support;
    //CHECK(rcl_init_options_fini(&ping_init_options));
    rcl_ret_t rc0 = rclc_support_init_with_options(&ping_support, 0, NULL, &init_options, &allocator);
    if (rc0 == RMW_RET_OK){
        CHECK(rclc_support_fini(&ping_support));
    }
    return rc0;
}

void ping_agent(){
    ESP_LOGW(TAG,"Searching for agent...");
    rcl_ret_t rc = init_ping_struct();

    if (RMW_RET_OK == rc) { //timeout_ms, attempts
        ESP_LOGI(TAG,"Agent found!");
    } else {
        int uros_agent_attempts = 0;
        ESP_LOGE(TAG,"Error on searching for agent");
        while (RMW_RET_OK != rc) {
            ESP_LOGW(TAG,"Trying again: %d", uros_agent_attempts);
            rc = init_ping_struct();
            uros_agent_attempts++;
            if (uros_agent_attempts >= 300){esp_restart();}
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        rc = init_ping_struct();
        if (RMW_RET_OK == rc) { //timeout_ms, attempts
            ESP_LOGI(TAG,"Connection with agent established!");
            ESP_LOGI(TAG,"Resuming...");
        } else {
            ESP_LOGE(TAG,"Impossible to find agent");
            ESP_LOGE(TAG,"Unstable connection! > Aborting");
            esp_restart();
        }
    }
}

void uros_task(void *arg) {

    uros_boot_lidar = xSemaphoreCreateBinary();

    allocator = rcl_get_default_allocator();

    pub_msgs_laserscan = rcl_get_zero_initialized_publisher();

    init_options = rcl_get_zero_initialized_init_options();
    CHECK(rcl_init_options_init(&init_options, allocator));

    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    rmw_options->localhost_only = RMW_LOCALHOST_ONLY_ENABLED;
    rmw_options->security_options.enforce_security = RMW_SECURITY_ENFORCEMENT_PERMISSIVE;

    CHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

#if (PING_AGENT == 1)
    ping_agent();
#endif

    CHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    init_msgs_laserscan();

    node = rcl_get_zero_initialized_node();
    CHECK(rclc_node_init_default(
        &node, 
        "uWABA_node_lidar", 
        "", 
        &support));
    ESP_LOGI(TAG,"Node created");

    const rosidl_message_type_support_t *type_support_msgs_laserscan = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan);
    CHECK(rclc_publisher_init_best_effort(
        &pub_msgs_laserscan, 
        &node, 
        type_support_msgs_laserscan,
        "micro_laserscan"));
    ESP_LOGI(TAG,"Laserscan publisher created");

	CHECK(rclc_executor_init(&executor_pub_msgs, &support.context, n_handles_pub, &allocator));
    ESP_LOGI(TAG,"Executor initialized");

    //CHECK(rclc_executor_set_timeout(&executor_publishers, RCL_MS_TO_NS(33)));

    xSemaphoreGive(uros_boot_lidar);

    //ESP_LOGI(TAG,"Executor spin");
    //rclc_executor_spin(&executor_pub_msgs);

    while(1){
        //lidar_pub_callback();
        CHECK(rclc_executor_spin_some(&executor_pub_msgs, RCL_MS_TO_NS(200)));
        //ESP_LOGI(TAG,"Executor spin");
        taskYIELD();
    }

    ESP_LOGI(TAG,"Clear memory");
    CHECK(rclc_executor_fini(&executor_pub_msgs));
    CHECK(rcl_publisher_fini(&pub_msgs_laserscan, &node));
	CHECK(rcl_node_fini(&node));
    CHECK(rclc_support_fini(&support));

    // rc = rclc_executor_fini(&executor);
    // rc += rcl_publisher_fini(&my_pub, &my_node);
    // rc += rcl_timer_fini(&my_timer);
    // rc += rcl_subscription_fini(&my_sub, &my_node);
    // rc += rcl_node_fini(&my_node);
    // rc += rclc_support_fini(&support);
    // std_msgs__msg__String__fini(&pub_msg);
    // std_msgs__msg__String__fini(&sub_msg);
    ESP_LOGE(TAG, "Task Delete");
    vTaskDelete(NULL);
}