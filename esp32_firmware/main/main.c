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
#include "driver/gpio.h"

#define LOG_TAG "ESP32_FIRMWARE"
#define TASK_STACK_SIZE 4096

void app_main(void)
{
    // 1. Initialize NVS (required by Wi‑Fi and other components)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Mount SPIFFS for file serving (UI assets)
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

    // 3. Initialize CAN bus (for motor communication)
    ESP_ERROR_CHECK(can_bus_init());

    // 4. Initialize motor control and start its task
    motor_control_init(); // Not wrapped inside ESP_ERROR_CHECK because it can't fail
    xTaskCreate(motor_control_task, "motor_control_task", TASK_STACK_SIZE, NULL, 10, NULL);

    // 5. Start Wi‑Fi and set up mDNS (for network connectivity and discovery)
    ESP_ERROR_CHECK(wifi_manager_start());
    ESP_ERROR_CHECK(wifi_manager_setup_mdns());

    // 6. Start HTTP server (to serve the UI from SPIFFS and expose the API)
    http_server_start();

    // Set GPIO pin 4 to output and drive it high (3.3V). This enables the relay.
    gpio_reset_pin(GPIO_NUM_4);
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_4, 1);
}
