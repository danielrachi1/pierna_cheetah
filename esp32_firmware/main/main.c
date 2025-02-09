#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "message_parser.h"
#include "cJSON.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_spiffs.h"
#include "motion_profile.h"
#include "mdns.h"
#include "can_bus.h"

#define LOG_TAG "ESP32_FIRMWARE"

#define CAN_TASK_STACK_SIZE (4096)
#define CAN_CMD_LENGTH 8 // CAN message length

// Wi-Fi Configuration
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASS CONFIG_WIFI_PASSWORD
#define WIFI_MAXIMUM_RETRY CONFIG_WIFI_MAXIMUM_RETRY

/* Event group bits */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define FILEPATH_MAX 520

/* Define ranges for position, velocity, etc. */
#define P_MIN -95.5f
#define P_MAX 95.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define I_MIN -18.0f
#define I_MAX 18.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

#define KP1 10
#define KD1 0
#define KP2 0
#define KD2 0
#define KP3 0
#define KD3 0

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

// Global flag array for each motor (index 1-3 are used; index 0 unused)
static bool motor_engaged[4] = {false, false, false, false};

const uint8_t ENTER_MOTOR_MODE_CMD[DATA_LENGTH] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
const uint8_t EXIT_MOTOR_MODE_CMD[DATA_LENGTH] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
const uint8_t ZERO_POS_SENSOR_CMD[DATA_LENGTH] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

static motion_profile_point_t *current_trajectory = NULL;
static int current_trajectory_points = 0;
static int current_trajectory_index = 0;
static bool trajectory_active = false;
static int current_trajectory_motor_id = 1;
static float current_motor_position = 0.0f;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < WIFI_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(LOG_TAG, "Retrying to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(LOG_TAG, "Failed to connect to the AP");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(LOG_TAG, "Got IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void connect_wifi(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(LOG_TAG, "Wi-Fi initialization completed.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(LOG_TAG, "Connected to AP SSID: %s", WIFI_SSID);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(LOG_TAG, "Failed to connect to SSID: %s", WIFI_SSID);
    }
    else
    {
        ESP_LOGE(LOG_TAG, "Unexpected event");
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

static void can_receive_task(void *arg)
{
    while (1)
    {
        twai_message_t received_msg;
        esp_err_t err = can_bus_receive(&received_msg);
        if (err == ESP_OK)
        {
            if (received_msg.data_length_code >= 5)
            {
                motor_reply_t reply = {0};
                unpack_reply(received_msg.data, &reply);
                current_motor_position = reply.position;
                ESP_LOGI(LOG_TAG, "Unpacked reply:");
                ESP_LOGI(LOG_TAG, "  Motor ID: %d", reply.motor_id);
                ESP_LOGI(LOG_TAG, "  Position: %.4f radians", reply.position);
                ESP_LOGI(LOG_TAG, "  Velocity: %.4f rad/s", reply.velocity);
                ESP_LOGI(LOG_TAG, "  Current: %.2f A", reply.current);
            }
            else
            {
                ESP_LOGE(LOG_TAG, "Received CAN message does not contain enough data to unpack");
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

static void log_trajectory_csv(motion_profile_point_t *trajectory, int num_points)
{
    char *csv_buffer = malloc(num_points * 80);
    if (!csv_buffer)
    {
        ESP_LOGE(LOG_TAG, "Failed to allocate memory for CSV buffer");
        return;
    }

    csv_buffer[0] = '\0';
    for (int i = 0; i < num_points; i++)
    {
        char line[80];
        snprintf(line, sizeof(line), "%d,%.6f,%.6f,%.6f%s",
                 i, trajectory[i].position, trajectory[i].velocity, trajectory[i].acceleration,
                 (i == num_points - 1) ? "" : ";");
        strcat(csv_buffer, line);
    }

    ESP_LOGI(LOG_TAG, "Trajectory CSV: index,position,velocity,acceleration; %s", csv_buffer);
    free(csv_buffer);
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    FILE *f = fopen("/spiffs/index.html", "r");
    if (f == NULL)
    {
        ESP_LOGE(LOG_TAG, "Failed to open index.html");
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/html");

    char buffer[512];
    size_t read_bytes;
    do
    {
        read_bytes = fread(buffer, 1, sizeof(buffer), f);
        if (read_bytes > 0)
        {
            if (httpd_resp_send_chunk(req, buffer, read_bytes) != ESP_OK)
            {
                fclose(f);
                ESP_LOGE(LOG_TAG, "File sending failed!");
                httpd_resp_sendstr_chunk(req, NULL);
                httpd_resp_send_500(req);
                return ESP_FAIL;
            }
        }
    } while (read_bytes > 0);

    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t file_get_handler(httpd_req_t *req)
{
    char filepath[FILEPATH_MAX];
    snprintf(filepath, sizeof(filepath), "/spiffs%s", req->uri);

    FILE *f = fopen(filepath, "r");
    if (f == NULL)
    {
        ESP_LOGE(LOG_TAG, "Failed to open file : %s", filepath);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    const char *type = "text/plain";
    if (strstr(req->uri, ".css"))
    {
        type = "text/css";
    }
    else if (strstr(req->uri, ".js"))
    {
        type = "application/javascript";
    }
    else if (strstr(req->uri, ".html"))
    {
        type = "text/html";
    }

    httpd_resp_set_type(req, type);

    char buffer[512];
    size_t read_bytes;
    do
    {
        read_bytes = fread(buffer, 1, sizeof(buffer), f);
        if (read_bytes > 0)
        {
            if (httpd_resp_send_chunk(req, buffer, read_bytes) != ESP_OK)
            {
                fclose(f);
                ESP_LOGE(LOG_TAG, "File sending failed!");
                httpd_resp_sendstr_chunk(req, NULL);
                httpd_resp_send_500(req);
                return ESP_FAIL;
            }
        }
    } while (read_bytes > 0);

    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/**
 * @brief Synchronizes the motor's target by forcing a fresh sensor read,
 *        then commands the motor to hold that position before entering motor mode.
 *
 * The function performs the following steps:
 * 1) Sends a dummy command to force the motor to reply with its sensor reading.
 * 2) Waits for a sensor reply from the motor.
 * 3) Unpacks the sensor reading and sends a hold command to update the motor's target.
 * 4) Finally, sends the enter motor mode command.
 *
 * @param motor_id The CAN identifier of the motor.
 */
static void sync_and_engage_motor_control(int motor_id)
{
    // Step 1: Send a dummy command to force a sensor update.
    uint8_t dummy_cmd[CAN_CMD_LENGTH] = {0};
    pack_cmd(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, dummy_cmd);
    if (can_bus_transmit(dummy_cmd, CAN_CMD_LENGTH, motor_id) != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Motor ID %d: Failed to send dummy command for synchronization", motor_id);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow time for the motor to process and respond

    // Step 2: Wait for the sensor reply from the motor.
    twai_message_t sensor_msg;
    bool sensor_received = false;
    const int max_attempts = 5;
    for (int i = 0; i < max_attempts; i++)
    {
        if (can_bus_receive(&sensor_msg) == ESP_OK &&
            sensor_msg.data_length_code == 6 && // Expect a 6-byte sensor reply
            sensor_msg.identifier == motor_id)  // Verify the response comes from the correct motor
        {
            sensor_received = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!sensor_received)
    {
        ESP_LOGE(LOG_TAG, "Motor ID %d: No sensor reply received during synchronization", motor_id);
        return;
    }

    // Step 3: Unpack the sensor reading and send a hold command using that value.
    motor_reply_t reply = {0};
    unpack_reply(sensor_msg.data, &reply);
    ESP_LOGI(LOG_TAG, "Motor ID %d: Sensor reading: position=%.4f, velocity=%.4f, current=%.2f",
             motor_id, reply.position, reply.velocity, reply.current);

    uint8_t hold_cmd[CAN_CMD_LENGTH] = {0};
    // Build the hold command using the sensor reading as the target.
    pack_cmd(reply.position, 0.0f, 10, 1, 0.0f, hold_cmd);
    if (can_bus_transmit(hold_cmd, CAN_CMD_LENGTH, motor_id) != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Motor ID %d: Failed to send hold command", motor_id);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // Optional: wait a short moment for the hold command to take effect

    // Step 4: Finally, send the enter motor mode command.
    if (can_bus_send_enter_mode(motor_id) == ESP_OK)
    {
        ESP_LOGI(LOG_TAG, "Motor ID %d: Enter motor mode command sent successfully", motor_id);
    }
    else
    {
        ESP_LOGE(LOG_TAG, "Motor ID %d: Failed to send enter motor mode command", motor_id);
    }
}

static esp_err_t send_command_post_handler(httpd_req_t *req)
{
    int total_len = req->content_len;
    int cur_len = 0;
    int received = 0;
    char *buf = malloc(total_len + 1);
    if (!buf)
    {
        ESP_LOGE(LOG_TAG, "Failed to allocate memory for POST buffer");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to allocate memory");
        return ESP_FAIL;
    }

    while (cur_len < total_len)
    {
        received = httpd_req_recv(req, buf + cur_len, total_len - cur_len);
        if (received <= 0)
        {
            if (received == HTTPD_SOCK_ERR_TIMEOUT)
            {
                continue;
            }
            free(buf);
            return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[total_len] = '\0';

    ESP_LOGI(LOG_TAG, "Received POST data: %s", buf);

    motor_command_t command = {0};
    char special_command[BUF_SIZE] = {0};

    if (!parse_json_command(buf, &command, special_command))
    {
        ESP_LOGE(LOG_TAG, "Failed to parse JSON command");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON command");
        free(buf);
        return ESP_FAIL;
    }

    free(buf);

    if (strlen(special_command) > 0)
    {
        if (strcmp(special_command, "ENTER_MODE") == 0)
        {
            // If not engaged, engage the motor and set flag to true.
            if (!motor_engaged[command.motor_id])
            {
                sync_and_engage_motor_control(command.motor_id);
                motor_engaged[command.motor_id] = true;
            }
            else
            {
                ESP_LOGI(LOG_TAG, "Motor ID %d already engaged.", command.motor_id);
            }
        }
        else if (strcmp(special_command, "EXIT_MODE") == 0)
        {
            if (motor_engaged[command.motor_id])
            {
                if (can_bus_send_exit_mode(command.motor_id) == ESP_OK)
                {
                    ESP_LOGI(LOG_TAG, "Exit Motor Mode command sent successfully");
                    motor_engaged[command.motor_id] = false;
                }
                else
                {
                    ESP_LOGE(LOG_TAG, "Failed to send Exit Motor Mode command");
                }
            }
            else
            {
                ESP_LOGI(LOG_TAG, "Motor ID %d already disengaged.", command.motor_id);
            }
        }
        else if (strcmp(special_command, "ZERO_POS") == 0)
        {
            // Allow ZERO_POS only when motor is unlocked.
            if (!motor_engaged[command.motor_id])
            {
                if (can_bus_send_zero_pos_sensor(command.motor_id) == ESP_OK)
                {
                    ESP_LOGI(LOG_TAG, "Zero Position Sensor command sent successfully");
                }
                else
                {
                    ESP_LOGE(LOG_TAG, "Failed to send Zero Position Sensor command");
                }
            }
            else
            {
                ESP_LOGE(LOG_TAG, "Cannot set sensor zero while motor is engaged! Command rejected.");
            }
        }
    }
    else
    {
        // For move (position) commands.
        if (!motor_engaged[command.motor_id])
        {
            ESP_LOGE(LOG_TAG, "Motor ID %d is not engaged. Please enter motor mode first.", command.motor_id);
            httpd_resp_send(req, "Motor not engaged", HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        }

        bool position_only = (command.velocity == 0.0f &&
                              command.kp == 0.0f &&
                              command.kd == 0.0f &&
                              command.feed_forward_torque == 0.0f);

        if (position_only)
        {
            if (current_trajectory)
            {
                motion_profile_free_trajectory(current_trajectory);
                current_trajectory = NULL;
                current_trajectory_points = 0;
                current_trajectory_index = 0;
                trajectory_active = false;
            }

            current_trajectory_motor_id = command.motor_id;

            if (motion_profile_generate_s_curve(
                    current_motor_position, 0.0f, command.position, 0.0f,
                    MP_DEFAULT_MAX_VEL, MP_DEFAULT_MAX_ACC, MP_DEFAULT_MAX_JERK, MP_TIME_STEP,
                    &current_trajectory, &current_trajectory_points))
            {
                ESP_LOGI(LOG_TAG, "S-curve trajectory generated with %d points", current_trajectory_points);
                log_trajectory_csv(current_trajectory, current_trajectory_points);
                trajectory_active = true;
                current_trajectory_index = 0;
            }
            else
            {
                ESP_LOGE(LOG_TAG, "Failed to generate s-curve trajectory");
            }
        }
        else
        {
            ESP_LOGW(LOG_TAG, "WARNING: Advanced command used. This can be dangerous!");
            ESP_LOGI(LOG_TAG, "Parsed Command Data:");
            ESP_LOGI(LOG_TAG, "  Position: %.4f radians", command.position);
            ESP_LOGI(LOG_TAG, "  Velocity: %.4f rad/s", command.velocity);
            ESP_LOGI(LOG_TAG, "  kp: %.4f", command.kp);
            ESP_LOGI(LOG_TAG, "  kd: %.4f", command.kd);
            ESP_LOGI(LOG_TAG, "  Feed-Forward Torque: %.4f N-m", command.feed_forward_torque);

            uint8_t can_msg_data[CAN_CMD_LENGTH] = {0};
            pack_cmd(command.position, command.velocity, command.kp, command.kd, command.feed_forward_torque, can_msg_data);

            if (can_bus_transmit(can_msg_data, CAN_CMD_LENGTH, command.motor_id) == ESP_OK)
            {
                ESP_LOGI(LOG_TAG, "Direct CAN message sent successfully to Motor ID %d", command.motor_id);
            }
            else
            {
                ESP_LOGE(LOG_TAG, "Failed to send direct CAN message");
            }
        }
    }

    return ESP_OK;
}

httpd_uri_t uri_root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler,
    .user_ctx = NULL};

httpd_uri_t uri_send_command = {
    .uri = "/send_command",
    .method = HTTP_POST,
    .handler = send_command_post_handler,
    .user_ctx = NULL};

httpd_uri_t file_uri = {
    .uri = "/*",
    .method = HTTP_GET,
    .handler = file_get_handler,
    .user_ctx = NULL};

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_send_command);
        httpd_register_uri_handler(server, &file_uri);
        return server;
    }

    ESP_LOGI(LOG_TAG, "Error starting server!");
    return NULL;
}

static void motion_control_task(void *arg)
{
    const TickType_t delay_ticks = pdMS_TO_TICKS((int)(MP_TIME_STEP * 1000));
    for (;;)
    {
        if (trajectory_active && current_trajectory_index < current_trajectory_points)
        {
            motion_profile_point_t *pt = &current_trajectory[current_trajectory_index];

            float kp_current = 0.0f;
            float kd_current = 0.0f;

            // Decide which KP/KD to use
            switch (current_trajectory_motor_id)
            {
            case 1:
                kp_current = KP1;
                kd_current = KD1;
                break;
            case 2:
                kp_current = KP2;
                kd_current = KD2;
                break;
            case 3:
                kp_current = KP3;
                kd_current = KD3;
                break;
            default:
                // fallback
                kp_current = 0.0f;
                kd_current = 0.0f;
                break;
            }

            uint8_t can_msg_data[CAN_CMD_LENGTH] = {0};
            pack_cmd(pt->position, pt->velocity, kp_current, kd_current, 0.0f, can_msg_data);

            if (can_bus_transmit(can_msg_data, CAN_CMD_LENGTH, current_trajectory_motor_id) == ESP_OK)
            {
                ESP_LOGI(LOG_TAG, "Trajectory point %d/%d -> Motor %d | pos=%.3f, vel=%.3f, kp=%.3f, kd=%.3f",
                         current_trajectory_index, current_trajectory_points, current_trajectory_motor_id,
                         pt->position, pt->velocity, kp_current, kd_current);
            }

            current_trajectory_index++;
            if (current_trajectory_index >= current_trajectory_points)
            {
                trajectory_active = false;
                ESP_LOGI(LOG_TAG, "Trajectory completed.");
            }
        }

        vTaskDelay(delay_ticks);
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    connect_wifi();

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};
    ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(LOG_TAG, "SPIFFS mounted successfully");

    ESP_ERROR_CHECK(can_bus_init());
    start_webserver();

    // Initialize mDNS
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("cheetah"));
    ESP_ERROR_CHECK(mdns_instance_name_set("Cheetah Leg Controller"));
    ESP_ERROR_CHECK(mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0));
    ESP_LOGI(LOG_TAG, "mDNS started. You can now access http://cheetah.local/");

    // On boot, force all motors to exit motor mode (flag remains false)
    can_bus_send_exit_mode(0x1);
    can_bus_send_exit_mode(0x2);
    can_bus_send_exit_mode(0x3);

    xTaskCreate(can_receive_task, "can_receive_task", CAN_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(motion_control_task, "motion_control_task", 4096, NULL, 10, NULL);
}
