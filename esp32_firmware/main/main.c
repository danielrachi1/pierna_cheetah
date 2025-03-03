#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_spiffs.h"
#include "wifi_manager.h"
#include "can_bus.h"
#include "http_server.h"
#include "motor_control_core.h"
#include "motor_control_task.h"
#include "driver/gpio.h"
#include "robot_controller.h"

#define LOG_TAG "ESP32_FIRMWARE"
#define TASK_STACK_SIZE 4096

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Mount SPIFFS for file serving
    esp_vfs_spiffs_conf_t spiffs_conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};
    ret = esp_vfs_spiffs_register(&spiffs_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to mount SPIFFS (%s)", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(LOG_TAG, "SPIFFS mounted successfully");

    // Initialize robot controller
    robot_controller_init();

    // Initialize CAN bus
    ESP_ERROR_CHECK(can_bus_init());

    // Initialize motor control
    motor_control_init();
    xTaskCreate(motor_control_task, "motor_control_task", TASK_STACK_SIZE, NULL, 10, NULL);

    // If leftover motors were engaged, force shutdown now
    if (robot_controller_get_motors_engaged_flag())
    {
        ESP_LOGW(LOG_TAG, "Detected leftover engaged motors. Entering recovery mode.");
        robot_controller_set_recovery_needed(true);
    }

    // Start Wi-Fi and mDNS
    ESP_ERROR_CHECK(wifi_manager_start());
    ESP_ERROR_CHECK(wifi_manager_setup_mdns());

    // Start HTTP server
    http_server_start();

    // Configure relay GPIO as OUTPUT but ensure it's off initially
    gpio_reset_pin(GPIO_NUM_4);
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_4, 0);

    ESP_LOGI(LOG_TAG, "Main setup complete.");
}
