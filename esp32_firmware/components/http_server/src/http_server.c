#include "http_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "motor_control.h" // for motor_command_t, motor_command_type_t, etc.

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

/* Serve static files from SPIFFS */
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

/*
 * POST /api/command
 * JSON body: {
 *   "motor_id": 1|2|3,
 *   "command": "enter_motor_mode"|"exit_motor_mode"|"zero_position_sensor"|"go_to_position",
 *   "position": <degrees> (if command == go_to_position)
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

    // Build our new motor_command_t
    motor_command_t cmd = {
        .motor_id = motor_id,
        .cmd_type = MOTOR_CMD_EXIT_MODE, // default, will override
        .position = 0.0f};

    if (strcmp(cmd_str, "enter_motor_mode") == 0)
    {
        cmd.cmd_type = MOTOR_CMD_ENTER_MODE;
    }
    else if (strcmp(cmd_str, "exit_motor_mode") == 0)
    {
        cmd.cmd_type = MOTOR_CMD_EXIT_MODE;
    }
    else if (strcmp(cmd_str, "zero_position_sensor") == 0)
    {
        cmd.cmd_type = MOTOR_CMD_ZERO_POS_SENSOR;
    }
    else if (strcmp(cmd_str, "go_to_position") == 0)
    {
        cmd.cmd_type = MOTOR_CMD_MOVE;
        cJSON *pos_item = cJSON_GetObjectItem(root, "position");
        if (!cJSON_IsNumber(pos_item))
        {
            cJSON_Delete(root);
            httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Missing 'position' in degrees\"}");
            return ESP_FAIL;
        }
        float deg = (float)pos_item->valuedouble;
        // will probably change this later, not all motors have the same limits
        if (deg < 0.0f || deg > 360.0f)
        {
            cJSON_Delete(root);
            httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Position must be between 0 and 360 deg\"}");
            return ESP_FAIL;
        }
        // convert to radians
        cmd.position = deg * (M_PI / 180.0f);
    }
    else
    {
        cJSON_Delete(root);
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Unknown command\"}");
        return ESP_FAIL;
    }

    cJSON_Delete(root);

    esp_err_t err = motor_control_handle_command(&cmd);

    if (err == ESP_OK)
    {
        httpd_resp_sendstr(req, "{\"status\":\"ok\",\"message\":\"Command executed successfully\"}");
    }
    else if (err == ESP_ERR_INVALID_ARG)
    {
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Invalid motor_id or command type\"}");
    }
    else if (err == ESP_ERR_INVALID_STATE)
    {
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Motor is busy or in invalid state\"}");
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
