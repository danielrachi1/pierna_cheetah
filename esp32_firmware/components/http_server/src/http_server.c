#include "http_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "message_parser.h"  // For processing POST commands

#define TAG "HTTP_SERVER"
#define FILEPATH_MAX 520
#define BUF_SIZE 256

/* Handler to serve the index.html file from SPIFFS */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    FILE *f = fopen("/spiffs/index.html", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open index.html");
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/html");

    char buffer[512];
    size_t read_bytes;
    do {
        read_bytes = fread(buffer, 1, sizeof(buffer), f);
        if (read_bytes > 0) {
            if (httpd_resp_send_chunk(req, buffer, read_bytes) != ESP_OK) {
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
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file : %s", filepath);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    const char *type = "text/plain";
    if (strstr(req->uri, ".css")) {
        type = "text/css";
    } else if (strstr(req->uri, ".js")) {
        type = "application/javascript";
    } else if (strstr(req->uri, ".html")) {
        type = "text/html";
    }

    httpd_resp_set_type(req, type);

    char buffer[512];
    size_t read_bytes;
    do {
        read_bytes = fread(buffer, 1, sizeof(buffer), f);
        if (read_bytes > 0) {
            if (httpd_resp_send_chunk(req, buffer, read_bytes) != ESP_OK) {
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
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate memory for POST buffer");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to allocate memory");
        return ESP_FAIL;
    }

    while (cur_len < total_len) {
        received = httpd_req_recv(req, buf + cur_len, total_len - cur_len);
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
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

    if (!parse_json_command(buf, &command, special_command)) {
        ESP_LOGE(TAG, "Failed to parse JSON command");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON command");
        free(buf);
        return ESP_FAIL;
    }

    free(buf);

    /* For now we simply log the received command.
       In a full refactor, you might forward the command to a motor-control module. */
    if (strlen(special_command) > 0) {
        ESP_LOGI(TAG, "Special command: %s for Motor ID: %d", special_command, command.motor_id);
    } else {
        ESP_LOGI(TAG, "Move command for Motor ID: %d", command.motor_id);
        ESP_LOGI(TAG, "  Position: %.4f, Velocity: %.4f, kp: %.4f, kd: %.4f, FF torque: %.4f",
                 command.position, command.velocity, command.kp, command.kd, command.feed_forward_torque);
    }

    httpd_resp_send(req, "Command received", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* Define URI handlers */
static httpd_uri_t uri_root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

static httpd_uri_t uri_send_command = {
    .uri       = "/send_command",
    .method    = HTTP_POST,
    .handler   = send_command_post_handler,
    .user_ctx  = NULL
};

static httpd_uri_t file_uri = {
    .uri       = "/*",
    .method    = HTTP_GET,
    .handler   = file_get_handler,
    .user_ctx  = NULL
};

httpd_handle_t http_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    esp_err_t ret = httpd_start(&server, &config);
    if (ret == ESP_OK) {
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_send_command);
        httpd_register_uri_handler(server, &file_uri);
        ESP_LOGI(TAG, "HTTP server started successfully");
        return server;
    } else {
        ESP_LOGE(TAG, "Error starting HTTP server!");
        return NULL;
    }
}
