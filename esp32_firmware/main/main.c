#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_spiffs.h"

#include "wifi_manager.h"
#include "can_bus.h"
#include "http_server.h"
#include "motor_control.h"

#define LOG_TAG "ESP32_FIRMWARE"
#define CAN_TASK_STACK_SIZE (4096)
#define FILEPATH_MAX 520

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Start Wi‑Fi and set up mDNS via the wifi_manager module
    ESP_ERROR_CHECK(wifi_manager_start());
    ESP_ERROR_CHECK(wifi_manager_setup_mdns());

    // Mount SPIFFS for file serving
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

    // Initialize CAN bus and start the HTTP server
    ESP_ERROR_CHECK(can_bus_init());
    http_server_start();

    // On boot, force all motors to exit motor mode
    twai_message_t response;
    can_bus_send_exit_mode(0x1, &response);
    can_bus_send_exit_mode(0x2, &response);
    can_bus_send_exit_mode(0x3, &response);

    // Initialize motor control (per‑motor state) and launch its task
    motor_control_init();
    xTaskCreate(motor_control_task, "motor_control_task", 4096, NULL, 10, NULL);

    // The previous CAN receive task has been removed in favor of a request–response model.
}
