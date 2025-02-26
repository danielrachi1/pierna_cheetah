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
        ESP_LOGE(TAG, "Failed to open file : %s", filepath);
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
 * @brief POST /api/robot/on
 *
 * Turns the robot on (relay -> ON, zero sensor, enter motor mode).
 */
static esp_err_t api_robot_on_handler(httpd_req_t *req)
{
    // We ignore request body (if any). Just call turn_on.
    esp_err_t err = robot_controller_turn_on();
    httpd_resp_set_type(req, "application/json");
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
 * Turns the robot off (drive to home, exit mode, relay -> OFF).
 */
static esp_err_t api_robot_off_handler(httpd_req_t *req)
{
    esp_err_t err = robot_controller_turn_off();
    httpd_resp_set_type(req, "application/json");
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

    cJSON *root = cJSON_Parse(buf);
    free(buf);

    httpd_resp_set_type(req, "application/json");
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

    // We only allow "go_to_position" right now
    if (strcmp(cmd_str, "go_to_position") != 0)
    {
        cJSON_Delete(root);
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Unknown or disallowed command\"}");
        return ESP_FAIL;
    }

    // Make sure robot state is ENGAGED_READY or OPERATING
    robot_state_t rstate = robot_controller_get_state();
    if (rstate != ROBOT_STATE_ENGAGED_READY && rstate != ROBOT_STATE_OPERATING)
    {
        cJSON_Delete(root);
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Robot is not turned on\"}");
        return ESP_FAIL;
    }

    // Retrieve "position"
    cJSON *pos_item = cJSON_GetObjectItem(root, "position");
    if (!cJSON_IsNumber(pos_item))
    {
        cJSON_Delete(root);
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Missing or invalid 'position'\"}");
        return ESP_FAIL;
    }
    float deg = (float)pos_item->valuedouble;

    // Retrieve "speed" (1..100)
    cJSON *speed_item = cJSON_GetObjectItem(root, "speed");
    if (!cJSON_IsNumber(speed_item))
    {
        cJSON_Delete(root);
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Missing or invalid 'speed'\"}");
        return ESP_FAIL;
    }
    float speed_percentage = (float)speed_item->valuedouble;

    // Clamp to [1..100] so 0% is not allowed anymore
    if (speed_percentage < 1.0f)
        speed_percentage = 1.0f;
    if (speed_percentage > 100.0f)
        speed_percentage = 100.0f;

    cJSON_Delete(root);

    // Build command
    motor_command_t cmd = {
        .motor_id = motor_id,
        .cmd_type = MOTOR_CMD_MOVE,
        .position = deg * (M_PI / 180.0f), // convert to radians
        .speed_percentage = speed_percentage};

    esp_err_t err = motor_control_handle_command(&cmd);

    if (err == ESP_OK)
    {
        // If a move command is accepted, we can set state to OPERATING
        robot_controller_set_state(ROBOT_STATE_OPERATING);
        httpd_resp_sendstr(req, "{\"status\":\"ok\",\"message\":\"Move command accepted\"}");
    }
    else if (err == ESP_ERR_INVALID_ARG)
    {
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Invalid motor_id or position out of range\"}");
    }
    else if (err == ESP_ERR_INVALID_STATE)
    {
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Motor is busy or not engaged\"}");
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

/* New URIs for robot on/off */
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
