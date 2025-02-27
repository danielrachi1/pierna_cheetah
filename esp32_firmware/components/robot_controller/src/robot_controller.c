#include "robot_controller.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "can_bus.h"
#include "motor_control.h"
#include "driver/gpio.h" // for relay control

static const char *TAG = "ROBOT_CONTROLLER";

#define NVS_NAMESPACE "storage"
#define NVS_KEY_MOTORS_ENGAGED "motors_engaged"

// A single global (static) boolean indicating if robot is engaged (true) or off (false).
static bool s_robot_engaged = false;

bool robot_controller_is_engaged(void)
{
    return s_robot_engaged;
}

bool robot_controller_get_motors_engaged_flag(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS for reading: %s", esp_err_to_name(err));
        return false;
    }
    uint8_t val = 0;
    err = nvs_get_u8(handle, NVS_KEY_MOTORS_ENGAGED, &val);
    nvs_close(handle);
    if (err == ESP_OK)
    {
        return (val == 1);
    }
    return false;
}

// Helper to write the "motors_engaged" flag in NVS
static esp_err_t set_motors_engaged_flag(bool engaged)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_set_u8(handle, NVS_KEY_MOTORS_ENGAGED, (uint8_t)(engaged ? 1 : 0));
    if (err == ESP_OK)
    {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

// Relay control helper
static esp_err_t relay_set(bool enable)
{
    ESP_LOGI(TAG, "Relay %s", enable ? "ON" : "OFF");
    gpio_set_level(GPIO_NUM_4, enable ? 1 : 0);
    return ESP_OK;
}

void robot_controller_init(void)
{
    // Simply start off as "not engaged"
    s_robot_engaged = false;
}

esp_err_t robot_controller_turn_on(void)
{
    ESP_LOGI(TAG, "Turn On requested...");

    // If already engaged, warn and return
    if (s_robot_engaged)
    {
        ESP_LOGW(TAG, "Robot is already engaged, ignoring");
        return ESP_ERR_INVALID_STATE;
    }

    // 1) Relay ON
    relay_set(true);

    // 2) Zero sensors on all motors
    for (int i = 1; i <= NUM_MOTORS; i++)
    {
        twai_message_t response;
        esp_err_t err = can_bus_send_zero_pos_sensor(i, &response);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Zero sensor failed on motor %d: %s", i, esp_err_to_name(err));
            robot_controller_turn_off(); // Force shutdown if one fails
            return ESP_FAIL;
        }
    }

    // 3) Enter motor mode
    for (int i = 1; i <= NUM_MOTORS; i++)
    {
        motor_command_t cmd = {
            .motor_id = i,
            .cmd_type = MOTOR_CMD_ENTER_MODE,
            .position = 0.0f // not used for enter mode
        };
        esp_err_t err = motor_control_handle_command(&cmd);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Enter mode failed on motor %d: %s", i, esp_err_to_name(err));
            robot_controller_turn_off();
            return ESP_FAIL;
        }
    }

    // 4) Mark engaged in NVS
    esp_err_t err = set_motors_engaged_flag(true);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set NVS motors_engaged: %s", esp_err_to_name(err));
        robot_controller_turn_off();
        return ESP_FAIL;
    }

    // 5) Set our local boolean
    s_robot_engaged = true;
    ESP_LOGI(TAG, "Robot successfully turned on. Engaged = true");
    return ESP_OK;
}

esp_err_t robot_controller_turn_off(void)
{
    ESP_LOGI(TAG, "Turn Off requested...");

    // If already off, do nothing
    if (!s_robot_engaged)
    {
        ESP_LOGW(TAG, "Already off, ignoring.");
        return ESP_OK;
    }

    // 1) Move all motors to home (0.0)
    for (int i = 1; i <= NUM_MOTORS; i++)
    {
        esp_err_t err = motor_control_move_blocking(i, 0.0f, pdMS_TO_TICKS(5000));
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "Motor %d failed to reach home (or timed out). Err: %s", i, esp_err_to_name(err));
        }
    }

    // 2) Exit motor mode on all motors
    for (int i = 1; i <= NUM_MOTORS; i++)
    {
        twai_message_t response;
        esp_err_t err = can_bus_send_exit_mode(i, &response);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "Exit mode failed on motor %d: %s", i, esp_err_to_name(err));
        }
    }

    // 3) Disable relay
    relay_set(false);

    // 4) Clear the engaged flag in NVS
    esp_err_t err_nvs = set_motors_engaged_flag(false);
    if (err_nvs != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to clear NVS flag. err: %s", esp_err_to_name(err_nvs));
    }

    // 5) Clear our local boolean
    s_robot_engaged = false;
    ESP_LOGI(TAG, "Robot is now OFF.");
    return ESP_OK;
}

void robot_controller_handle_motor_error(int motor_id)
{
    ESP_LOGE(TAG, "Motor %d error triggered. Shutting down.", motor_id);
    esp_err_t err_off = robot_controller_turn_off();
    if (err_off != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to shutdown. err: %s", esp_err_to_name(err_off));
    }
}
