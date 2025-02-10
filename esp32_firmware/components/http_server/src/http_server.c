#include "http_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "message_parser.h"
#include "motor_control.h"
#include <math.h> // for M_PI

#define TAG "HTTP_SERVER"
#define FILEPATH_MAX 520
#define BUF_SIZE 256

/* Handler to serve the index.html file from SPIFFS */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    FILE *f = fopen("/spiffs/index.html", "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open index.html");
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
                ESP_LOGE(TAG, "File sending failed!");
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

/* Handler to serve any requested file from SPIFFS */
static esp_err_t file_get_handler(httpd_req_t *req)
{
    char filepath[FILEPATH_MAX];
    snprintf(filepath, sizeof(filepath), "/spiffs%s", req->uri);

    FILE *f = fopen(filepath, "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file : %s", filepath);
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
                ESP_LOGE(TAG, "File sending failed!");
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

/* HTTP POST handler for receiving motor commands as JSON */
static esp_err_t send_command_post_handler(httpd_req_t *req)
{
    int total_len = req->content_len;
    int cur_len = 0;
    int received = 0;
    char *buf = malloc(total_len + 1);
    if (!buf)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for POST buffer");
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

    ESP_LOGI(TAG, "Received POST data: %s", buf);

    motor_command_t command = {0};
    char special_command[BUF_SIZE] = {0};

    if (!parse_json_command(buf, &command, special_command))
    {
        ESP_LOGE(TAG, "Failed to parse JSON command");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON command");
        free(buf);
        return ESP_FAIL;
    }
    free(buf);

    /* Forward the parsed command to the motor control module */
    esp_err_t err = motor_control_handle_command(&command, special_command);
    if (err != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Motor command failed");
        return ESP_FAIL;
    }

    httpd_resp_send(req, "Command sent", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* New API endpoint handlers */

/* GET /api/enter-motor-mode/[motor id] */
static esp_err_t api_enter_motor_mode_handler(httpd_req_t *req)
{
    const char *uri = req->uri;
    const char *id_str = uri + strlen("/api/enter-motor-mode/");
    int motor_id = atoi(id_str);
    if (motor_id < 1 || motor_id > NUM_MOTORS)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid motor ID");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "API: Enter Motor Mode for Motor ID %d", motor_id);
    motor_command_t command = {0};
    command.motor_id = motor_id;
    esp_err_t err = motor_control_handle_command(&command, "ENTER_MODE");
    if (err != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to enter motor mode");
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, "Enter Motor Mode command sent");
    return ESP_OK;
}

/* GET /api/exit-motor-mode/[motor id] */
static esp_err_t api_exit_motor_mode_handler(httpd_req_t *req)
{
    const char *uri = req->uri;
    const char *id_str = uri + strlen("/api/exit-motor-mode/");
    int motor_id = atoi(id_str);
    if (motor_id < 1 || motor_id > NUM_MOTORS)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid motor ID");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "API: Exit Motor Mode for Motor ID %d", motor_id);
    motor_command_t command = {0};
    command.motor_id = motor_id;
    esp_err_t err = motor_control_handle_command(&command, "EXIT_MODE");
    if (err != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to exit motor mode");
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, "Exit Motor Mode command sent");
    return ESP_OK;
}

/* GET /api/set-sensor-zero-pos/[motor id] */
static esp_err_t api_set_sensor_zero_pos_handler(httpd_req_t *req)
{
    const char *uri = req->uri;
    const char *id_str = uri + strlen("/api/set-sensor-zero-pos/");
    int motor_id = atoi(id_str);
    if (motor_id < 1 || motor_id > NUM_MOTORS)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid motor ID");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "API: Set Sensor Zero Position for Motor ID %d", motor_id);
    motor_command_t command = {0};
    command.motor_id = motor_id;
    esp_err_t err = motor_control_handle_command(&command, "ZERO_POS");
    if (err != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to set sensor zero position");
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, "Set Sensor Zero Position command sent");
    return ESP_OK;
}

/* GET /api/move-to-pos/[motor id]/[angle] */
static esp_err_t api_move_to_pos_handler(httpd_req_t *req)
{
    const char *uri = req->uri;
    int motor_id;
    float angle_deg;
    if (sscanf(uri, "/api/move-to-pos/%d/%f", &motor_id, &angle_deg) != 2)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid URI format");
        return ESP_FAIL;
    }
    if (motor_id < 1 || motor_id > NUM_MOTORS)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid motor ID");
        return ESP_FAIL;
    }
    if (angle_deg < 0 || angle_deg > 360)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Angle must be between 0 and 360 degrees");
        return ESP_FAIL;
    }
    // Convert degrees to radians
    float angle_rad = angle_deg * (M_PI / 180.0f);
    ESP_LOGI(TAG, "API: Move to Position for Motor ID %d, angle %.2f° (%.4f rad)", motor_id, angle_deg, angle_rad);
    motor_command_t command = {0};
    command.motor_id = motor_id;
    command.position = angle_rad;
    // Note: Do not invert the angle here—the motor_control module will handle inversion for motor 1.
    esp_err_t err = motor_control_handle_command(&command, "");
    if (err != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to generate move command");
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, "Move to Position command sent");
    return ESP_OK;
}

/* Define URI handlers */
static httpd_uri_t uri_root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler,
    .user_ctx = NULL};

static httpd_uri_t uri_send_command = {
    .uri = "/send_command",
    .method = HTTP_POST,
    .handler = send_command_post_handler,
    .user_ctx = NULL};

static httpd_uri_t uri_api_enter_motor_mode = {
    .uri = "/api/enter-motor-mode/*",
    .method = HTTP_GET,
    .handler = api_enter_motor_mode_handler,
    .user_ctx = NULL};

static httpd_uri_t uri_api_exit_motor_mode = {
    .uri = "/api/exit-motor-mode/*",
    .method = HTTP_GET,
    .handler = api_exit_motor_mode_handler,
    .user_ctx = NULL};

static httpd_uri_t uri_api_set_sensor_zero_pos = {
    .uri = "/api/set-sensor-zero-pos/*",
    .method = HTTP_GET,
    .handler = api_set_sensor_zero_pos_handler,
    .user_ctx = NULL};

static httpd_uri_t uri_api_move_to_pos = {
    .uri = "/api/move-to-pos/*",
    .method = HTTP_GET,
    .handler = api_move_to_pos_handler,
    .user_ctx = NULL};

/* Wildcard file handler (registered last so that API endpoints take precedence) */
static httpd_uri_t file_uri = {
    .uri = "/*",
    .method = HTTP_GET,
    .handler = file_get_handler,
    .user_ctx = NULL};

httpd_handle_t http_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    esp_err_t ret = httpd_start(&server, &config);
    if (ret == ESP_OK)
    {
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_send_command);
        httpd_register_uri_handler(server, &uri_api_enter_motor_mode);
        httpd_register_uri_handler(server, &uri_api_exit_motor_mode);
        httpd_register_uri_handler(server, &uri_api_set_sensor_zero_pos);
        httpd_register_uri_handler(server, &uri_api_move_to_pos);
        httpd_register_uri_handler(server, &file_uri);
        ESP_LOGI(TAG, "HTTP server started successfully");
        return server;
    }
    else
    {
        ESP_LOGE(TAG, "Error starting HTTP server!");
        return NULL;
    }
}
