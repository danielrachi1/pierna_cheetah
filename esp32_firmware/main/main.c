#include <stdio.h>
#include <string.h> // For string operations
#include <stdlib.h> // For malloc and free
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h" // For system functions
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/twai.h" // For CAN (TWAI) driver
#include "message_parser.h"
#include "cJSON.h" // Include cJSON library
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h" // For HTTP server functions
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_spiffs.h"
#include "motion_profile.h"

#define LOG_TAG "ESP32_FIRMWARE"

#define CAN_TASK_STACK_SIZE (4096)
#define CAN_CMD_LENGTH 8 // CAN message length
#define DATA_LENGTH 8

// TWAI (CAN) Configuration
#define TWAI_TX_GPIO_NUM (21)
#define TWAI_RX_GPIO_NUM (22)
#define TWAI_MODE TWAI_MODE_NORMAL // Set to normal mode for actual CAN communication; no ack for testing
#define TWAI_BIT_RATE TWAI_TIMING_CONFIG_1MBITS()
#define TWAI_FILTER_CONFIG TWAI_FILTER_CONFIG_ACCEPT_ALL()

// Wi-Fi Configuration
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASS CONFIG_WIFI_PASSWORD
#define WIFI_MAXIMUM_RETRY CONFIG_WIFI_MAXIMUM_RETRY

/* Event group bits */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define FILEPATH_MAX 520

#define KP 0
#define KD 0

/* FreeRTOS event group to signal when we are connected */
static EventGroupHandle_t s_wifi_event_group;

static int s_retry_num = 0;

/* Special Command Byte Arrays */
const uint8_t ENTER_MOTOR_MODE_CMD[DATA_LENGTH] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
const uint8_t EXIT_MOTOR_MODE_CMD[DATA_LENGTH] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
const uint8_t ZERO_POS_SENSOR_CMD[DATA_LENGTH] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

static motion_profile_point_t *current_trajectory = NULL;
static int current_trajectory_points = 0;
static int current_trajectory_index = 0;
static bool trajectory_active = false;

static float current_motor_position = 0.0f; // Track this from the received replies

/**
 * @brief Event handler for Wi-Fi events.
 */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    /* Handle Wi-Fi events */
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

/**
 * @brief Initializes and connects to the Wi-Fi network.
 */
static void connect_wifi(void)
{
    s_wifi_event_group = xEventGroupCreate();

    /* Initialize TCP/IP network interface */
    ESP_ERROR_CHECK(esp_netif_init());

    /* Create default event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Create default Wi-Fi station */
    esp_netif_create_default_wifi_sta();

    /* Initialize Wi-Fi with default configurations */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Register event handlers */
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

    /* Configure Wi-Fi connection settings */
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA)); // Set Wi-Fi to station mode
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(LOG_TAG, "Wi-Fi initialization completed.");

    /* Wait until either the connection is established or the maximum number of retries is reached */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* Check which event occurred */
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

    /* Cleanup event handlers */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

/**
 * @brief Initializes the TWAI (CAN) driver.
 */
static void twai_init()
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO_NUM, TWAI_RX_GPIO_NUM, TWAI_MODE);
    twai_timing_config_t t_config = TWAI_BIT_RATE;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG;

    /* Install TWAI driver */
    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to install TWAI driver: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(LOG_TAG, "TWAI driver installed");

    /* Start TWAI driver */
    err = twai_start();
    if (err != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to start TWAI driver: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(LOG_TAG, "TWAI driver started");
}

/**
 * @brief Sends a CAN message via TWAI.
 *
 * @param msg_data Pointer to the CAN message data array.
 * @param length   Length of the CAN message data (up to 8 bytes).
 * @param motor_id The motor ID to which the message is sent.
 * @return esp_err_t ESP_OK on success, otherwise an error code.
 */
static esp_err_t send_can_message(uint8_t *msg_data, size_t length, int motor_id)
{
    twai_message_t message;
    memset(&message, 0, sizeof(twai_message_t));

    /* Set message fields */
    message.identifier = motor_id;
    message.data_length_code = length;
    memcpy(message.data, msg_data, length);

    /* Log the raw TWAI message before sending */
    ESP_LOGI(LOG_TAG, "Sending TWAI Message to Motor ID %d:", motor_id);
    ESP_LOGI(LOG_TAG, "  ID: 0x%lx", message.identifier);
    ESP_LOGI(LOG_TAG, "  Data Length: %d", message.data_length_code);
    ESP_LOG_BUFFER_HEX(LOG_TAG, message.data, message.data_length_code);

    /* Transmit the message */
    esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(1000)); // Timeout 1 second
    if (err != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "TWAI transmit failed: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(LOG_TAG, "TWAI transmit successful");
    }
    return err;
}

/**
 * @brief Receives a CAN message via TWAI.
 *
 * @param message Pointer to the twai_message_t structure to store the received message.
 * @return esp_err_t ESP_OK on success, otherwise an error code.
 */
static esp_err_t receive_can_message(twai_message_t *message)
{
    /* Attempt to receive a CAN message with a 1-second timeout */
    esp_err_t err = twai_receive(message, pdMS_TO_TICKS(1000)); // Timeout 1 second

    if (err == ESP_OK)
    {
        /* Log the raw TWAI message after receiving */
        ESP_LOGI(LOG_TAG, "Received TWAI Message:");
        ESP_LOGI(LOG_TAG, "  ID: 0x%lx", message->identifier);
        ESP_LOGI(LOG_TAG, "  Data Length: %d", message->data_length_code);
        ESP_LOG_BUFFER_HEX(LOG_TAG, message->data, message->data_length_code);
    }
    else if (err != ESP_ERR_TIMEOUT)
    {
        /* Log the error if reception failed, excluding timeout which is normal */
        ESP_LOGE(LOG_TAG, "TWAI receive failed: %s", esp_err_to_name(err));
    }

    return err;
}

/**
 * @brief Sends the "Enter Motor Mode" special command.
 *
 * @return esp_err_t ESP_OK on success, otherwise an error code.
 */
static esp_err_t send_enter_motor_mode(int motor_id)
{
    ESP_LOGI(LOG_TAG, "Sending Enter Motor Mode command to Motor ID %d", motor_id);
    return send_can_message((uint8_t *)ENTER_MOTOR_MODE_CMD, DATA_LENGTH, motor_id);
}

/**
 * @brief Sends the "Exit Motor Mode" special command.
 *
 * @return esp_err_t ESP_OK on success, otherwise an error code.
 */
static esp_err_t send_exit_motor_mode(int motor_id)
{
    ESP_LOGI(LOG_TAG, "Sending Exit Motor Mode command to Motor ID %d", motor_id);
    return send_can_message((uint8_t *)EXIT_MOTOR_MODE_CMD, DATA_LENGTH, motor_id);
}

/**
 * @brief Sends the "Zero Position Sensor" special command.
 *
 * @return esp_err_t ESP_OK on success, otherwise an error code.
 */
static esp_err_t send_zero_position_sensor(int motor_id)
{
    ESP_LOGI(LOG_TAG, "Sending Zero Position Sensor command to Motor ID %d", motor_id);
    return send_can_message((uint8_t *)ZERO_POS_SENSOR_CMD, DATA_LENGTH, motor_id);
}

/**
 * @brief Task to continuously receive CAN messages.
 *
 * This task runs indefinitely, listening for incoming CAN messages
 * and processing them as they arrive.
 *
 * @param arg Pointer to parameters (unused).
 */
static void can_receive_task(void *arg)
{
    while (1)
    {
        twai_message_t received_msg;
        esp_err_t err = receive_can_message(&received_msg);
        if (err == ESP_OK)
        {
            /* Process the received CAN message */
            if (received_msg.data_length_code >= 5) // Adjust based on expected data
            {
                motor_reply_t reply = {0}; // Initialize the reply structure

                /* Call the unpack_reply function */
                unpack_reply(received_msg.data, &reply);

                current_motor_position = reply.position;

                /* Log the unpacked reply data */
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
        else if (err != ESP_ERR_TIMEOUT)
        {
            /* Log the error if reception failed, excluding timeout which is normal */
            ESP_LOGE(LOG_TAG, "CAN receive error: %s", esp_err_to_name(err));
        }

        /* Short delay to prevent task hogging the CPU */
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    /* Should never reach here, but good practice to delete the task if it does */
    vTaskDelete(NULL);
}

/**
 * @brief HTTP GET handler to serve the main page.
 */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    // Open the index.html file from SPIFFS
    FILE *f = fopen("/spiffs/index.html", "r");
    if (f == NULL)
    {
        ESP_LOGE(LOG_TAG, "Failed to open index.html");
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Set the Content-Type header
    httpd_resp_set_type(req, "text/html");

    // Read and send the file in chunks
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
                httpd_resp_sendstr_chunk(req, NULL); // Terminate chunked response
                httpd_resp_send_500(req);
                return ESP_FAIL;
            }
        }
    } while (read_bytes > 0);

    // Close the file
    fclose(f);

    // Indicate that the entire chunked response is complete
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t file_get_handler(httpd_req_t *req)
{
    char filepath[FILEPATH_MAX];
    snprintf(filepath, sizeof(filepath), "/spiffs%s", req->uri);

    // Open the requested file
    FILE *f = fopen(filepath, "r");
    if (f == NULL)
    {
        ESP_LOGE(LOG_TAG, "Failed to open file : %s", filepath);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Determine content type
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

    // Set the Content-Type header
    httpd_resp_set_type(req, type);

    // Read and send the file in chunks
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
                httpd_resp_sendstr_chunk(req, NULL); // Terminate chunked response
                httpd_resp_send_500(req);
                return ESP_FAIL;
            }
        }
    } while (read_bytes > 0);

    // Close the file
    fclose(f);

    // Indicate that the entire chunked response is complete
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/**
 * @brief HTTP POST handler to receive and process motor commands.
 */
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

    /* Read the data for the POST request */
    while (cur_len < total_len)
    {
        received = httpd_req_recv(req, buf + cur_len, total_len - cur_len);
        if (received <= 0)
        {
            if (received == HTTPD_SOCK_ERR_TIMEOUT)
            {
                continue; // Retry receiving if timeout occurred
            }
            free(buf);
            return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[total_len] = '\0'; // Null-terminate the buffer

    /* Log the received data */
    ESP_LOGI(LOG_TAG, "Received POST data: %s", buf);

    motor_command_t command = {0};
    char special_command[BUF_SIZE] = {0};

    /* Parse the JSON command */
    if (!parse_json_command(buf, &command, special_command))
    {
        ESP_LOGE(LOG_TAG, "Failed to parse JSON command");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON command");
        free(buf);
        return ESP_FAIL;
    }
    else if (strlen(special_command) > 0)
    {
        /* Process special command */
        if (strcmp(special_command, "ENTER_MODE") == 0)
        {
            if (send_enter_motor_mode(command.motor_id) == ESP_OK)
            {
                ESP_LOGI(LOG_TAG, "Enter Motor Mode command sent successfully");
            }
            else
            {
                ESP_LOGE(LOG_TAG, "Failed to send Enter Motor Mode command");
            }
        }
        else if (strcmp(special_command, "EXIT_MODE") == 0)
        {
            if (send_exit_motor_mode(command.motor_id) == ESP_OK)
            {
                ESP_LOGI(LOG_TAG, "Exit Motor Mode command sent successfully");
            }
            else
            {
                ESP_LOGE(LOG_TAG, "Failed to send Exit Motor Mode command");
            }
        }
        else if (strcmp(special_command, "ZERO_POS") == 0)
        {
            if (send_zero_position_sensor(command.motor_id) == ESP_OK)
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
            ESP_LOGW(LOG_TAG, "Unknown special command: %s", special_command);
        }
    }
    else
    {
        // Process regular motor command
        ESP_LOGI(LOG_TAG, "Parsed Command Data:");
        ESP_LOGI(LOG_TAG, "  Position: %.4f radians", command.position);
        ESP_LOGI(LOG_TAG, "  Velocity: %.4f rad/s", command.velocity);
        ESP_LOGI(LOG_TAG, "  Proportional Gain (kp): %.4f N-m/rad", command.kp);
        ESP_LOGI(LOG_TAG, "  Derivative Gain (kd): %.4f N-m*s/rad", command.kd);
        ESP_LOGI(LOG_TAG, "  Feed-Forward Torque: %.4f N-m", command.feed_forward_torque);

        // For simplicity, let's assume we want to move from current_motor_position with start_vel=0 to
        // command.position with end_vel=0 using an s-curve profile.
        // We'll ignore kp, kd, and feed_forward_torque for the trajectory generation itself,
        // but we'll still send them with each setpoint if needed.
        if (current_trajectory)
        {
            motion_profile_free_trajectory(current_trajectory);
            current_trajectory = NULL;
            current_trajectory_points = 0;
            current_trajectory_index = 0;
            trajectory_active = false;
        }

        // Generate s-curve trajectory
        if (motion_profile_generate_s_curve(
                current_motor_position, 0.0f, command.position, 0.0f,
                MP_DEFAULT_MAX_VEL, MP_DEFAULT_MAX_ACC, MP_DEFAULT_MAX_JERK, MP_TIME_STEP,
                &current_trajectory, &current_trajectory_points))
        {
            ESP_LOGI(LOG_TAG, "S-curve trajectory generated with %d points", current_trajectory_points);
            trajectory_active = true;
            current_trajectory_index = 0;
        }
        else
        {
            ESP_LOGE(LOG_TAG, "Failed to generate s-curve trajectory");
        }
    }

    free(buf);

    /* Send a response back to the client */
    httpd_resp_send(req, "Command received", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* URI handlers */
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
    .uri = "/*", // Match all URIs
    .method = HTTP_GET,
    .handler = file_get_handler,
    .user_ctx = NULL};

/**
 * @brief Starts the web server.
 *
 * @return httpd_handle_t Handle to the HTTP server.
 */
static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    /* Enable wildcard URI matching */
    config.uri_match_fn = httpd_uri_match_wildcard;

    /* Start the HTTP server */
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK)
    {
        /* Register URI handlers */
        httpd_register_uri_handler(server, &uri_root);         // Handler for "/"
        httpd_register_uri_handler(server, &uri_send_command); // Handler for "/send_command"
        httpd_register_uri_handler(server, &file_uri);         // Handler for all other URIs (static files)
        return server;
    }

    ESP_LOGI(LOG_TAG, "Error starting server!");
    return NULL;
}

// This is just an example. You could create a separate task for this.
static void motion_control_task(void *arg)
{
    // Suppose we run at 100 Hz (time step = 0.01 s)
    const TickType_t delay_ticks = pdMS_TO_TICKS((int)(MP_TIME_STEP * 1000));
    for (;;)
    {
        if (trajectory_active && current_trajectory_index < current_trajectory_points)
        {
            motion_profile_point_t *pt = &current_trajectory[current_trajectory_index];

            // Use pack_cmd to convert position/velocity to CAN frame.
            // You might set kp, kd, and torque here as well. For now, let's just re-use the existing command gains.
            // If you want different kp/kd/torque for each point, store them or define them as needed.
            uint8_t can_msg_data[CAN_CMD_LENGTH] = {0};

            // For a smooth profile, you might just send position and let velocity feed-forward = pt->velocity,
            // and no torque feed-forward. Or if your motor requires torque feed-forward, adjust accordingly.
            pack_cmd(pt->position, pt->velocity, KP, KD, 0.0f, can_msg_data);

            // Assume motor_id was previously set or use a fixed ID for now. For example:
            int motor_id = 1; // Adjust as needed

            if (send_can_message(can_msg_data, CAN_CMD_LENGTH, motor_id) == ESP_OK)
            {
                ESP_LOGI(LOG_TAG, "Trajectory point %d/%d sent: pos=%.3f, vel=%.3f",
                         current_trajectory_index, current_trajectory_points, pt->position, pt->velocity);
            }

            current_trajectory_index++;
            if (current_trajectory_index >= current_trajectory_points)
            {
                // Done with trajectory
                trajectory_active = false;
                ESP_LOGI(LOG_TAG, "Trajectory completed.");
            }
        }

        vTaskDelay(delay_ticks);
    }

    vTaskDelete(NULL);
}

/**
 * @brief Application entry point.
 */
void app_main(void)
{
    /* Initialize NVS (required for Wi-Fi) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        /* NVS partition was truncated and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize Wi-Fi */
    connect_wifi();

    /* Initialize SPIFFS */
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

    /* Initialize TWAI (CAN) driver */
    twai_init();

    /* Start web server */
    start_webserver();

    /* Create CAN receive task */
    xTaskCreate(can_receive_task, "can_receive_task", CAN_TASK_STACK_SIZE, NULL, 10, NULL);

    xTaskCreate(motion_control_task, "motion_control_task", 4096, NULL, 10, NULL);
}
