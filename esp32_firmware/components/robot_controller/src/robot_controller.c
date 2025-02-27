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

/*
 * Internal state:
 *   s_robot_engaged: false means robot is OFF; true means robot is ON/engaged.
 *   s_recovery_needed: true if a previous run left motors engaged (improper shutdown),
 *                      meaning the user must physically power off motors, home them,
 *                      then call /api/recovery/clear.
 */
static bool s_robot_engaged = false;
static bool s_recovery_needed = false;

bool robot_controller_is_engaged(void)
{
    return s_robot_engaged;
}

bool robot_controller_is_recovery_needed(void)
{
    return s_recovery_needed;
}

void robot_controller_set_recovery_needed(bool needed)
{
    s_recovery_needed = needed;
}

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

esp_err_t robot_controller_clear_recovery(void)
{
    // When the user calls this endpoint, we assume they have physically:
    // 1. Powered off the motors (via switches/emergency stop)
    // 2. Manually moved the motors to the home position
    // 3. Then turned them on again.
    s_recovery_needed = false;
    // Also clear the NVS flag to reflect that motors are no longer engaged.
    set_motors_engaged_flag(false);
    ESP_LOGI(TAG, "Recovery cleared. The user indicates motors are physically safe.");
    return ESP_OK;
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

// Relay control helper
static esp_err_t relay_set(bool enable)
{
    ESP_LOGI(TAG, "Relay %s", enable ? "ON" : "OFF");
    gpio_set_level(GPIO_NUM_4, enable ? 1 : 0);
    return ESP_OK;
}

void robot_controller_init(void)
{
    // On a fresh boot, default to off and no recovery needed.
    s_robot_engaged = false;
    s_recovery_needed = false;
}

esp_err_t robot_controller_turn_on(void)
{
    ESP_LOGI(TAG, "Turn On requested...");

    // Do not allow turning on if recovery is needed.
    if (s_recovery_needed)
    {
        ESP_LOGE(TAG,
                 "Cannot turn on while 'recovery needed' is set. "
                 "User must physically power off each motor, manually home them, then call /api/recovery/clear.");
        return ESP_ERR_INVALID_STATE;
    }

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
            robot_controller_turn_off();
            return ESP_FAIL;
        }
    }

    // 3) Enter motor mode on all motors
    for (int i = 1; i <= NUM_MOTORS; i++)
    {
        motor_command_t cmd = {
            .motor_id = i,
            .cmd_type = MOTOR_CMD_ENTER_MODE,
            .position = 0.0f // Not used for enter mode
        };
        esp_err_t err = motor_control_handle_command(&cmd);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Enter mode failed on motor %d: %s", i, esp_err_to_name(err));
            robot_controller_turn_off();
            return ESP_FAIL;
        }
    }

    // 4) Set NVS flag
    esp_err_t err = set_motors_engaged_flag(true);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set NVS motors_engaged: %s", esp_err_to_name(err));
        robot_controller_turn_off();
        return ESP_FAIL;
    }

    // 5) Mark robot as engaged
    s_robot_engaged = true;
    ESP_LOGI(TAG, "Robot successfully turned on. Engaged = true");
    return ESP_OK;
}

esp_err_t robot_controller_turn_off(void)
{
    ESP_LOGI(TAG, "Turn Off requested...");

    // Even if s_robot_engaged is false, we continue with cleanup.
    if (s_robot_engaged)
    {
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
    }
    else
    {
        ESP_LOGW(TAG, "Robot is physically off, skipping move and exit steps.");
    }

    // 3) Disable relay
    relay_set(false);

    // 4) Clear the engaged flag in NVS
    esp_err_t err_nvs = set_motors_engaged_flag(false);
    if (err_nvs != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to clear NVS flag. err: %s", esp_err_to_name(err_nvs));
    }

    // 5) Mark robot as off
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
