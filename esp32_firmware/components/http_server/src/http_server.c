#include "http_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "motor_control.h"
#include "robot_controller.h"

#define TAG "HTTP_SERVER"
#define FILEPATH_MAX 520

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

static esp_err_t file_get_handler(httpd_req_t *req)
{
    char filepath[FILEPATH_MAX];
    snprintf(filepath, sizeof(filepath), "/spiffs%s", req->uri);

    FILE *f = fopen(filepath, "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file: %s", filepath);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    const char *type = "text/plain";
    if (strstr(req->uri, ".css"))
        type = "text/css";
    if (strstr(req->uri, ".js"))
        type = "application/javascript";
    if (strstr(req->uri, ".html"))
        type = "text/html";
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

/**
 * @brief Utility: If recovery is needed, respond with a descriptive error.
 */
static esp_err_t check_recovery_needed(httpd_req_t *req)
{
    if (robot_controller_is_recovery_needed())
    {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req,
                           "{\"status\":\"error\",\"message\":\"Recovery needed. Please physically power off each motor using its switch or emergency stop, manually move all motors to the home position, then power them on again. Once done, call /api/recovery/clear to proceed.\"}");
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief POST /api/robot/on
 *
 * Turns the robot on unless recovery is needed.
 */
static esp_err_t api_robot_on_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    if (check_recovery_needed(req) != ESP_OK)
        return ESP_FAIL;

    esp_err_t err = robot_controller_turn_on();
    if (err == ESP_OK)
    {
        httpd_resp_sendstr(req, "{\"status\":\"ok\",\"message\":\"Robot turned ON successfully\"}");
    }
    else
    {
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Failed to turn on\"}");
    }
    return ESP_OK;
}

/**
 * @brief POST /api/robot/off
 *
 * Turns the robot off. This endpoint is now also gated: if recovery is needed, it is rejected.
 */
static esp_err_t api_robot_off_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    if (check_recovery_needed(req) != ESP_OK)
        return ESP_FAIL;

    esp_err_t err = robot_controller_turn_off();
    if (err == ESP_OK)
    {
        httpd_resp_sendstr(req, "{\"status\":\"ok\",\"message\":\"Robot turned OFF successfully\"}");
    }
    else
    {
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Failed to turn off\"}");
    }
    return ESP_OK;
}

/**
 * @brief POST /api/recovery/clear
 *
 * Clears the "recovery needed" flag. This endpoint should be called only after the user has physically:
 *   1. Powered off each motor (via its switch or emergency stop),
 *   2. Manually moved all motors to the home position,
 *   3. Powered them on again.
 */
static esp_err_t api_recovery_clear_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    if (!robot_controller_is_recovery_needed())
    {
        httpd_resp_sendstr(req,
                           "{\"status\":\"ok\",\"message\":\"No recovery is needed. System is normal.\"}");
        return ESP_OK;
    }
    esp_err_t err = robot_controller_clear_recovery();
    if (err == ESP_OK)
    {
        httpd_resp_sendstr(req,
                           "{\"status\":\"ok\",\"message\":\"Recovery cleared. Motors are now considered safe. You may now use other commands.\"}");
    }
    else
    {
        httpd_resp_sendstr(req,
                           "{\"status\":\"error\",\"message\":\"Failed to clear recovery state\"}");
    }
    return ESP_OK;
}

/*
 * POST /api/command
 * JSON body: {
 *   "motor_id": 1|2|3,
 *   "speed": 1..100,
 *   "command": "go_to_position",
 *   "position": <degrees>
 * }
 */
static esp_err_t api_command_post_handler(httpd_req_t *req)
{
    int total_len = req->content_len;
    int cur_len = 0;
    int received = 0;
    char *buf = malloc(total_len + 1);
    if (!buf)
    {
        ESP_LOGE(TAG, "Failed to allocate POST buffer");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        return ESP_FAIL;
    }
    while (cur_len < total_len)
    {
        received = httpd_req_recv(req, buf + cur_len, total_len - cur_len);
        if (received <= 0)
        {
            if (received == HTTPD_SOCK_ERR_TIMEOUT)
                continue;
            free(buf);
            return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[total_len] = '\0';
    httpd_resp_set_type(req, "application/json");

    if (check_recovery_needed(req) != ESP_OK)
    {
        free(buf);
        return ESP_FAIL;
    }

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root)
    {
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
        return ESP_FAIL;
    }
    cJSON *motor_id_item = cJSON_GetObjectItem(root, "motor_id");
    cJSON *command_item = cJSON_GetObjectItem(root, "command");
    if (!cJSON_IsNumber(motor_id_item) || !cJSON_IsString(command_item))
    {
        cJSON_Delete(root);
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Missing 'motor_id' or 'command'\"}");
        return ESP_FAIL;
    }
    int motor_id = motor_id_item->valueint;
    const char *cmd_str = command_item->valuestring;
    if (strcmp(cmd_str, "go_to_position") != 0)
    {
        cJSON_Delete(root);
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Unknown or disallowed command\"}");
        return ESP_FAIL;
    }
    if (!robot_controller_is_engaged())
    {
        cJSON_Delete(root);
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Robot is off. Please turn on the robot first.\"}");
        return ESP_FAIL;
    }
    cJSON *pos_item = cJSON_GetObjectItem(root, "position");
    if (!cJSON_IsNumber(pos_item))
    {
        cJSON_Delete(root);
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Missing or invalid 'position'\"}");
        return ESP_FAIL;
    }
    float deg = (float)pos_item->valuedouble;
    cJSON *speed_item = cJSON_GetObjectItem(root, "speed");
    if (!cJSON_IsNumber(speed_item))
    {
        cJSON_Delete(root);
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Missing or invalid 'speed'\"}");
        return ESP_FAIL;
    }
    float speed_percentage = (float)speed_item->valuedouble;
    if (speed_percentage < 1.0f)
        speed_percentage = 1.0f;
    if (speed_percentage > 100.0f)
        speed_percentage = 100.0f;
    cJSON_Delete(root);

    motor_command_t cmd = {
        .motor_id = motor_id,
        .cmd_type = MOTOR_CMD_MOVE,
        .position = deg * (M_PI / 180.0f),
        .speed_percentage = speed_percentage};

    esp_err_t err = motor_control_handle_command(&cmd);
    if (err == ESP_OK)
    {
        httpd_resp_sendstr(req, "{\"status\":\"ok\",\"message\":\"Move command accepted\"}");
    }
    else if (err == ESP_ERR_INVALID_ARG)
    {
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Invalid motor_id or position out of range\"}");
    }
    else if (err == ESP_ERR_INVALID_STATE)
    {
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Motor is busy, not engaged, or recovery needed\"}");
    }
    else
    {
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Command failed\"}");
    }
    return ESP_OK;
}

static httpd_uri_t uri_root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler,
    .user_ctx = NULL};

static httpd_uri_t uri_api_command = {
    .uri = "/api/command",
    .method = HTTP_POST,
    .handler = api_command_post_handler,
    .user_ctx = NULL};

static httpd_uri_t uri_robot_on = {
    .uri = "/api/robot/on",
    .method = HTTP_POST,
    .handler = api_robot_on_handler,
    .user_ctx = NULL};

static httpd_uri_t uri_robot_off = {
    .uri = "/api/robot/off",
    .method = HTTP_POST,
    .handler = api_robot_off_handler,
    .user_ctx = NULL};

static httpd_uri_t uri_recovery_clear = {
    .uri = "/api/recovery/clear",
    .method = HTTP_POST,
    .handler = api_recovery_clear_handler,
    .user_ctx = NULL};

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
        httpd_register_uri_handler(server, &uri_api_command);
        httpd_register_uri_handler(server, &uri_robot_on);
        httpd_register_uri_handler(server, &uri_robot_off);
        httpd_register_uri_handler(server, &uri_recovery_clear);
        httpd_register_uri_handler(server, &file_uri);
        ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
        return server;
    }
    else
    {
        ESP_LOGE(TAG, "Error starting HTTP server!");
        return NULL;
    }
}
