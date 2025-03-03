#include "motor_control_core.h"
#include "robot_controller.h" // used for robot_controller_is_engaged()
#include "can_bus.h"
#include "message_parser.h"
#include "motion_profile.h"
#include "motor_command_handlers.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <math.h>

static const char *TAG = "MOTOR_CONTROL_CORE";

static motor_state_t s_motor_states[NUM_MOTORS]; ///< Global array for storing motor states.
static bool s_batch_in_progress = false;         ///< Tracks if a global batch is in progress.

// -------------------------------------------------------------------
// Helpers to invert angles on hardware-inverted motors
// -------------------------------------------------------------------
bool s_inverted[NUM_MOTORS] = {
    true,
    false,
    false};

float to_user_angle(int motor_id, float hardware_angle)
{
    return s_inverted[motor_id - 1] ? -hardware_angle : hardware_angle;
}

float from_user_angle(int motor_id, float user_angle)
{
    return s_inverted[motor_id - 1] ? -user_angle : user_angle;
}

// -------------------------------------------------------------------
// Basic getters/setters
// -------------------------------------------------------------------
void motor_control_init(void)
{
    ESP_LOGI(TAG, "Initializing Motor Control (Core)...");
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        s_motor_states[i].motor_id = i + 1;
        s_motor_states[i].engaged = false;
        s_motor_states[i].trajectory = NULL;
        s_motor_states[i].trajectory_points = 0;
        s_motor_states[i].trajectory_index = 0;
        s_motor_states[i].trajectory_active = false;
        s_motor_states[i].current_position = 0.0f;

        // Batch fields
        s_motor_states[i].batch_commands = NULL;
        s_motor_states[i].batch_count = 0;
        s_motor_states[i].batch_index = 0;
        s_motor_states[i].batch_error = false;
        s_motor_states[i].batch_error_message[0] = '\0';
    }
}

motor_state_t *motor_control_get_state(int motor_id)
{
    if (motor_id < 1 || motor_id > NUM_MOTORS)
    {
        ESP_LOGE(TAG, "motor_control_get_state: invalid motor_id=%d", motor_id);
        return NULL;
    }
    return &s_motor_states[motor_id - 1];
}

void motor_control_set_batch_in_progress(bool in_progress)
{
    s_batch_in_progress = in_progress;
}

bool motor_control_is_batch_in_progress(void)
{
    return s_batch_in_progress;
}

esp_err_t motor_control_move_blocking(int motor_id, float target_position_rad, int timeout_ticks)
{
    motor_state_t *state = motor_control_get_state(motor_id);
    if (!state)
    {
        return ESP_ERR_INVALID_ARG; // invalid ID
    }

    float default_speed_rads = (float)M_PI;

    motor_command_t cmd = {
        .motor_id = motor_id,
        .cmd_type = MOTOR_CMD_MOVE,
        .position = target_position_rad,
        .speed = default_speed_rads};

    esp_err_t err = motor_control_handle_command(&cmd);
    if (err != ESP_OK)
    {
        return err;
    }

    TickType_t start = xTaskGetTickCount();
    while (1)
    {
        // If the robot is not engaged anymore, bail out:
        if (!robot_controller_is_engaged())
        {
            return ESP_ERR_INVALID_STATE;
        }
        // If the trajectory is done, success:
        if (!state->trajectory_active)
        {
            return ESP_OK;
        }
        // Timeout check:
        if ((xTaskGetTickCount() - start) > timeout_ticks)
        {
            ESP_LOGW(TAG, "motor_control_move_blocking: Motor %d timed out waiting for trajectory finish.", motor_id);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
