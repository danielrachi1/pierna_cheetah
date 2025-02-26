#include "robot_controller.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "can_bus.h"
#include "motor_control.h"
#include "driver/gpio.h" // for relay control

static const char *TAG = "ROBOT_CONTROLLER";

#define NVS_NAMESPACE "storage"
#define NVS_KEY_MOTORS_ENGAGED "motors_engaged"

// Global (static) state
static robot_state_t s_robot_state = ROBOT_STATE_OFF;
static SemaphoreHandle_t s_state_mutex = NULL;

// Forward declarations
static esp_err_t set_motors_engaged_flag(bool engaged);

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
    if (!s_state_mutex)
    {
        s_state_mutex = xSemaphoreCreateMutex();
    }
    s_robot_state = ROBOT_STATE_OFF;
}

robot_state_t robot_controller_get_state(void)
{
    robot_state_t tmp;
    if (xSemaphoreTake(s_state_mutex, portMAX_DELAY))
    {
        tmp = s_robot_state;
        xSemaphoreGive(s_state_mutex);
    }
    else
    {
        tmp = ROBOT_STATE_ERROR;
    }
    return tmp;
}

void robot_controller_set_state(robot_state_t new_state)
{
    if (xSemaphoreTake(s_state_mutex, portMAX_DELAY))
    {
        s_robot_state = new_state;
        xSemaphoreGive(s_state_mutex);
    }
}

esp_err_t robot_controller_turn_on(void)
{
    ESP_LOGI(TAG, "Turn On requested...");
    if (xSemaphoreTake(s_state_mutex, portMAX_DELAY) == pdFALSE)
    {
        return ESP_FAIL;
    }
    if (s_robot_state != ROBOT_STATE_OFF && s_robot_state != ROBOT_STATE_INITIALIZING)
    {
        ESP_LOGW(TAG, "Turn on called in state %d, ignoring.", s_robot_state);
        xSemaphoreGive(s_state_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    s_robot_state = ROBOT_STATE_INITIALIZING;
    xSemaphoreGive(s_state_mutex);

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
            robot_controller_forced_shutdown();
            return ESP_FAIL;
        }
    }

    // 3) Enter motor mode on all motors
    for (int i = 1; i <= NUM_MOTORS; i++)
    {
        twai_message_t response;
        esp_err_t err = can_bus_send_enter_mode(i, &response);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Enter mode failed on motor %d: %s", i, esp_err_to_name(err));
            robot_controller_forced_shutdown();
            return ESP_FAIL;
        }
    }

    // 4) Set motors_engaged flag in NVS
    esp_err_t err = set_motors_engaged_flag(true);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set NVS motors_engaged: %s", esp_err_to_name(err));
        robot_controller_forced_shutdown();
        return ESP_FAIL;
    }

    // 5) Set state to ENGAGED_READY
    robot_controller_set_state(ROBOT_STATE_ENGAGED_READY);
    ESP_LOGI(TAG, "Robot successfully turned on. State = ENGAGED_READY");
    return ESP_OK;
}

esp_err_t robot_controller_turn_off(void)
{
    ESP_LOGI(TAG, "Turn Off requested...");
    if (xSemaphoreTake(s_state_mutex, portMAX_DELAY) == pdFALSE)
    {
        return ESP_FAIL;
    }
    if (s_robot_state == ROBOT_STATE_OFF || s_robot_state == ROBOT_STATE_SHUTTING_DOWN)
    {
        ESP_LOGW(TAG, "Already OFF or shutting down, ignoring.");
        xSemaphoreGive(s_state_mutex);
        return ESP_OK;
    }
    s_robot_state = ROBOT_STATE_SHUTTING_DOWN;
    xSemaphoreGive(s_state_mutex);

    // 1) Move all motors to home (0.0)
    for (int i = 1; i <= NUM_MOTORS; i++)
    {
        esp_err_t err = motor_control_move_blocking(i, 0.0f, pdMS_TO_TICKS(5000));
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "Motor %d failed to reach home (or timed out). Forcing shutdown anyway.", i);
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

    // 4) Clear motors_engaged flag in NVS
    (void)set_motors_engaged_flag(false);

    // 5) Set state to OFF
    robot_controller_set_state(ROBOT_STATE_OFF);
    ESP_LOGI(TAG, "Robot is now OFF.");
    return ESP_OK;
}

esp_err_t robot_controller_forced_shutdown(void)
{
    ESP_LOGE(TAG, "!!! FORCED SHUTDOWN TRIGGERED !!!");
    robot_controller_set_state(ROBOT_STATE_SHUTTING_DOWN);

    // Attempt to move motors to home (best effort)
    for (int i = 1; i <= NUM_MOTORS; i++)
    {
        (void)motor_control_move_blocking(i, 0.0f, pdMS_TO_TICKS(2000));
    }

    // Exit motor mode
    for (int i = 1; i <= NUM_MOTORS; i++)
    {
        (void)can_bus_send_exit_mode(i, NULL);
    }

    // Relay off
    relay_set(false);

    // Clear motors_engaged flag
    (void)set_motors_engaged_flag(false);

    // Set state to OFF
    robot_controller_set_state(ROBOT_STATE_OFF);
    ESP_LOGI(TAG, "Forced shutdown complete. State = OFF");
    return ESP_OK;
}

void robot_controller_handle_motor_error(int motor_id)
{
    ESP_LOGE(TAG, "Motor %d error triggered. Switching to ERROR state and forcing shutdown.", motor_id);
    robot_controller_set_state(ROBOT_STATE_ERROR);
    (void)robot_controller_forced_shutdown();
}
