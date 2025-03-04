#include "http_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "motor_control_core.h"
#include "motor_command_handlers.h"
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
 * POST /api/command/move
 * JSON body: {
 *   "motor_id": 1|2|3,
 *   "position": <degrees>
 *   "speed": 1..100,
 * }
 */
static esp_err_t api_command_move_handler(httpd_req_t *req)
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
    if (!cJSON_IsNumber(motor_id_item))
    {
        cJSON_Delete(root);
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Missing 'motor_id' or 'command'\"}");
        return ESP_FAIL;
    }
    int motor_id = motor_id_item->valueint;
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
    float pos_deg = (float)pos_item->valuedouble;
    cJSON *speed_item = cJSON_GetObjectItem(root, "speed");
    if (!cJSON_IsNumber(speed_item))
    {
        cJSON_Delete(root);
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Missing or invalid 'speed'\"}");
        return ESP_FAIL;
    }
    float speed_deg = (float)speed_item->valuedouble;
    if (speed_deg <= 0)
    {
        cJSON_Delete(root);
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Speed must be a positive number (deg/s)\"}");
        return ESP_FAIL;
    }

    // Validate that the provided speed does not exceed the motor's maximum allowed speed (in deg/s).
    float max_speed = 0.0f;
    if (motor_id == 1)
        max_speed = MOTOR1_MAX_SPEED_DPS;
    else if (motor_id == 2)
        max_speed = MOTOR2_MAX_SPEED_DPS;
    else if (motor_id == 3)
        max_speed = MOTOR3_MAX_SPEED_DPS;
    else
    {
        cJSON_Delete(root);
        httpd_resp_sendstr(req, "{\"status\":\"error\",\"message\":\"Invalid motor_id\"}");
        return ESP_FAIL;
    }
    if (speed_deg > max_speed)
    {
        cJSON_Delete(root);
        char error_msg[128];
        snprintf(error_msg, sizeof(error_msg),
                 "{\"status\":\"error\",\"message\":\"Speed exceeds maximum allowed (%.2f deg/s)\"}", max_speed);
        httpd_resp_sendstr(req, error_msg);
        return ESP_FAIL;
    }
    cJSON_Delete(root);

    // Convert user-provided degrees into radians.
    motor_command_t cmd = {
        .motor_id = motor_id,
        .cmd_type = MOTOR_CMD_MOVE,
        .position = pos_deg * (M_PI / 180.0f),
        .speed = speed_deg * (M_PI / 180.0f) // convert deg/s to rad/s
    };

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

static esp_err_t api_command_batch_handler(httpd_req_t *req)
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

    // Do not accept a new batch if one is in progress.
    for (int i = 1; i <= NUM_MOTORS; i++)
    {
        motor_state_t *state = motor_control_get_state(i);
        if (state && state->batch_commands != NULL)
        {
            httpd_resp_sendstr(req, "{\"global_status\":\"error\",\"message\":\"Batch already in progress\"}");
            free(buf);
            return ESP_FAIL;
        }
    }

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root)
    {
        httpd_resp_sendstr(req, "{\"global_status\":\"error\",\"message\":\"Invalid JSON\"}");
        return ESP_FAIL;
    }
    cJSON *batch_array = cJSON_GetObjectItem(root, "batch");
    if (!cJSON_IsArray(batch_array))
    {
        cJSON_Delete(root);
        httpd_resp_sendstr(req, "{\"global_status\":\"error\",\"message\":\"Missing 'batch' array\"}");
        return ESP_FAIL;
    }

    // Temporary per-motor command count arrays.
    int counts[NUM_MOTORS] = {0};
    int total_commands = cJSON_GetArraySize(batch_array);

    // First pass: Validate each command and count per motor.
    for (int i = 0; i < total_commands; i++)
    {
        cJSON *cmd_item = cJSON_GetArrayItem(batch_array, i);
        if (!cmd_item)
            continue;
        cJSON *motor_id_item = cJSON_GetObjectItem(cmd_item, "motor_id");
        cJSON *position_item = cJSON_GetObjectItem(cmd_item, "position");
        cJSON *speed_item = cJSON_GetObjectItem(cmd_item, "speed");
        if (!cJSON_IsNumber(motor_id_item) ||
            !cJSON_IsNumber(position_item) || 
            !cJSON_IsNumber(speed_item))
        {
            cJSON_Delete(root);
            httpd_resp_sendstr(req, "{\"global_status\":\"error\",\"message\":\"Invalid command format in batch\"}");
            return ESP_FAIL;
        }
        int motor_id = motor_id_item->valueint;
        if (motor_id < 1 || motor_id > NUM_MOTORS)
        {
            cJSON_Delete(root);
            httpd_resp_sendstr(req, "{\"global_status\":\"error\",\"message\":\"Invalid motor_id in batch\"}");
            return ESP_FAIL;
        }
        counts[motor_id - 1]++;
    }

    // Second pass: Allocate per-motor arrays and load commands.
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        if (counts[i] > 0)
        {
            motor_state_t *state = motor_control_get_state(i + 1);
            state->batch_commands = malloc(sizeof(motor_command_t) * counts[i]);
            if (!state->batch_commands)
            {
                cJSON_Delete(root);
                httpd_resp_sendstr(req, "{\"global_status\":\"error\",\"message\":\"Memory allocation failed for batch\"}");
                return ESP_FAIL;
            }
            state->batch_count = counts[i];
            state->batch_index = 0;
            state->batch_error = false;
            state->batch_error_message[0] = '\0';
        }
    }
    // Third pass: Copy commands into the per-motor arrays.
    int indices[NUM_MOTORS] = {0};
    for (int i = 0; i < total_commands; i++)
    {
        cJSON *cmd_item = cJSON_GetArrayItem(batch_array, i);
        int motor_id = cJSON_GetObjectItem(cmd_item, "motor_id")->valueint;
        // Convert position from degrees to radians.
        float position_deg = (float)cJSON_GetObjectItem(cmd_item, "position")->valuedouble;
        float position_rad = position_deg * (M_PI / 180.0f);
        // Process speed: convert from deg/s to rad/s.
        float speed_deg = (float)cJSON_GetObjectItem(cmd_item, "speed")->valuedouble;
        if (speed_deg <= 0)
        {
            cJSON_Delete(root);
            httpd_resp_sendstr(req, "{\"global_status\":\"error\",\"message\":\"Speed must be positive in batch\"}");
            return ESP_FAIL;
        }
        float max_speed = 0.0f;
        if (motor_id == 1)
            max_speed = MOTOR1_MAX_SPEED_DPS;
        else if (motor_id == 2)
            max_speed = MOTOR2_MAX_SPEED_DPS;
        else if (motor_id == 3)
            max_speed = MOTOR3_MAX_SPEED_DPS;
        else
        {
            cJSON_Delete(root);
            httpd_resp_sendstr(req, "{\"global_status\":\"error\",\"message\":\"Invalid motor_id in batch\"}");
            return ESP_FAIL;
        }
        if (speed_deg > max_speed)
        {
            cJSON_Delete(root);
            char error_msg[128];
            snprintf(error_msg, sizeof(error_msg),
                     "{\"global_status\":\"error\",\"message\":\"Speed in batch exceeds maximum allowed (%.2f deg/s)\"}", max_speed);
            httpd_resp_sendstr(req, error_msg);
            return ESP_FAIL;
        }
        float speed_rad = speed_deg * (M_PI / 180.0f);

        motor_command_t command = {
            .motor_id = motor_id,
            .cmd_type = MOTOR_CMD_MOVE,
            .position = position_rad,
            .speed = speed_rad};
        motor_state_t *state = motor_control_get_state(motor_id);
        state->batch_commands[indices[motor_id - 1]++] = command;
    }
    cJSON_Delete(root);

    // Mark global batch in progress.
    motor_control_set_batch_in_progress(true);

    // Wait (polling) until all motors have finished processing their batches or an error occurs.
    const int max_wait_ticks = 10000; // e.g., 10 seconds timeout
    int waited = 0;
    const int poll_delay_ms = 100;
    bool abort_occurred = false;
    while (waited < max_wait_ticks)
    {
        abort_occurred = false;
        bool all_done = true;
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            motor_state_t *state = motor_control_get_state(i + 1);
            if (state->batch_commands != NULL)
            {
                all_done = false;
            }
            if (state->batch_error)
            {
                abort_occurred = true;
            }
        }
        if (all_done || abort_occurred)
            break;
        vTaskDelay(pdMS_TO_TICKS(poll_delay_ms));
        waited += poll_delay_ms;
    }
    motor_control_set_batch_in_progress(false);

    // Build response JSON
    cJSON *resp = cJSON_CreateObject();
    if (!resp)
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    if (abort_occurred)
    {
        cJSON_AddStringToObject(resp, "global_status", "error");
    }
    else
    {
        cJSON_AddStringToObject(resp, "global_status", "ok");
    }
    cJSON *motors_arr = cJSON_CreateArray();
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        motor_state_t *state = motor_control_get_state(i + 1);
        cJSON *mobj = cJSON_CreateObject();
        cJSON_AddNumberToObject(mobj, "motor_id", state->motor_id);
        if (state->batch_error)
        {
            cJSON_AddStringToObject(mobj, "status", "error");
            cJSON_AddStringToObject(mobj, "error_message", state->batch_error_message);
            cJSON_AddNumberToObject(mobj, "commands_executed", state->batch_index);
        }
        else
        {
            cJSON_AddStringToObject(mobj, "status", "ok");
            cJSON_AddNumberToObject(mobj, "commands_executed", state->batch_index);
        }
        cJSON_AddItemToArray(motors_arr, mobj);
    }
    cJSON_AddItemToObject(resp, "motors", motors_arr);
    char *resp_str = cJSON_Print(resp);
    cJSON_Delete(resp);
    if (resp_str)
    {
        httpd_resp_sendstr(req, resp_str);
        free(resp_str);
    }
    else
    {
        httpd_resp_send_500(req);
    }
    return ESP_OK;
}

static httpd_uri_t uri_command_batch = {
    .uri = "/api/command/batch",
    .method = HTTP_POST,
    .handler = api_command_batch_handler,
    .user_ctx = NULL};

static httpd_uri_t uri_root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler,
    .user_ctx = NULL};

static httpd_uri_t uri_api_command = {
    .uri = "/api/command/move",
    .method = HTTP_POST,
    .handler = api_command_move_handler,
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
        httpd_register_uri_handler(server, &uri_command_batch);
        ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
        return server;
    }
    else
    {
        ESP_LOGE(TAG, "Error starting HTTP server!");
        return NULL;
    }
}
