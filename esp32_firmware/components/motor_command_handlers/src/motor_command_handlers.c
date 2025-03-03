#include "motor_command_handlers.h"
#include "motor_control_core.h"
#include "can_bus.h"
#include "message_parser.h"
#include "motion_profile.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "MOTOR_CMD_HANDLERS";

// Helper function for angle range
static esp_err_t check_angle_range(int motor_id, float angle_deg)
{
    switch (motor_id)
    {
    case 1:
        if (angle_deg < MOTOR1_MIN_ANGLE_DEG || angle_deg > MOTOR1_MAX_ANGLE_DEG)
            goto out_of_range;
        break;
    case 2:
        if (angle_deg < MOTOR2_MIN_ANGLE_DEG || angle_deg > MOTOR2_MAX_ANGLE_DEG)
            goto out_of_range;
        break;
    case 3:
        if (angle_deg < MOTOR3_MIN_ANGLE_DEG || angle_deg > MOTOR3_MAX_ANGLE_DEG)
            goto out_of_range;
        break;
    default:
        ESP_LOGE(TAG, "Unknown motor ID %d in check_angle_range.", motor_id);
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;

out_of_range:
    ESP_LOGE(TAG, "Motor %d: angle %.2f deg out of range", motor_id, angle_deg);
    return ESP_ERR_INVALID_ARG;
}

/**
 * @brief Sync the current motor position from the hardware by sending either an Enter or Exit Mode command.
 *        This is used inside handle_move_command() to get a fresh position reading.
 */
static esp_err_t motor_control_sync_position(motor_state_t *state)
{
    twai_message_t response;
    esp_err_t err;

    if (state->engaged)
    {
        // Enter Mode command to get updated reading
        err = can_bus_send_enter_mode(state->motor_id, &response);
    }
    else
    {
        // Exit Mode command if not engaged
        err = can_bus_send_exit_mode(state->motor_id, &response);
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Motor %d: Sync command failed: %s", state->motor_id, esp_err_to_name(err));
        return err;
    }

    if (response.data_length_code == 6)
    {
        motor_reply_t reply = {0};
        unpack_reply(response.data, &reply);

        float user_angle = to_user_angle(state->motor_id, reply.position);
        state->current_position = user_angle;

        ESP_LOGI(TAG,
                 "Motor %d: Synced => hw=%.4f => user=%.4f, velocity=%.4f, current=%.2f",
                 state->motor_id, reply.position, user_angle, reply.velocity, reply.current);
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG,
                 "Motor %d: Expected 6-byte sensor reply for sync, got %d bytes.",
                 state->motor_id, response.data_length_code);
        return ESP_FAIL;
    }
}

// -------------------------------------------------------------------
// Individual command handlers
// -------------------------------------------------------------------
static esp_err_t handle_enter_mode(motor_state_t *state)
{
    if (state->engaged)
    {
        ESP_LOGI(TAG, "Motor %d already engaged.", state->motor_id);
        return ESP_OK;
    }

    twai_message_t response;
    esp_err_t err = can_bus_send_enter_mode(state->motor_id, &response);
    if (err == ESP_OK)
    {
        state->engaged = true;
        ESP_LOGI(TAG, "Motor %d: Enter Motor Mode successful.", state->motor_id);
    }
    else
    {
        ESP_LOGE(TAG, "Motor %d: Enter Mode failed: %s", state->motor_id, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t handle_exit_mode(motor_state_t *state)
{
    if (!state->engaged)
    {
        ESP_LOGI(TAG, "Motor %d already disengaged.", state->motor_id);
        return ESP_OK;
    }

    twai_message_t response;
    esp_err_t err = can_bus_send_exit_mode(state->motor_id, &response);
    if (err == ESP_OK)
    {
        state->engaged = false;
        ESP_LOGI(TAG, "Motor %d: Exit Motor Mode successful.", state->motor_id);
    }
    else
    {
        ESP_LOGE(TAG, "Motor %d: Exit Mode failed: %s", state->motor_id, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t handle_zero_sensor(motor_state_t *state)
{
    if (state->engaged)
    {
        ESP_LOGE(TAG, "Motor %d: Cannot zero sensor while engaged.", state->motor_id);
        return ESP_ERR_INVALID_STATE;
    }

    twai_message_t response;
    esp_err_t err = can_bus_send_zero_pos_sensor(state->motor_id, &response);
    if (err == ESP_OK)
    {
        state->current_position = 0.0f;
        ESP_LOGI(TAG, "Motor %d: Zero Position successful; user-space=0.0f", state->motor_id);
    }
    else
    {
        ESP_LOGE(TAG, "Motor %d: Zero sensor failed: %s", state->motor_id, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t handle_move_command(motor_state_t *state, float target_position_rad, float speed_rads)
{
    // Must be engaged to move
    if (!state->engaged)
    {
        ESP_LOGE(TAG, "Motor %d is not engaged; must enter mode first.", state->motor_id);
        return ESP_ERR_INVALID_STATE;
    }

    // If a trajectory is already active, reject
    if (state->trajectory_active)
    {
        ESP_LOGE(TAG, "Motor %d is already executing a trajectory.", state->motor_id);
        return ESP_ERR_INVALID_STATE;
    }

    // 1) Sync position
    esp_err_t err = motor_control_sync_position(state);
    if (err != ESP_OK)
    {
        return err;
    }

    // 2) Check angle range
    float target_deg = target_position_rad * (180.0f / M_PI);
    err = check_angle_range(state->motor_id, target_deg);
    if (err != ESP_OK)
    {
        return err;
    }

    // 3) Generate a new trajectory
    if (state->trajectory)
    {
        motion_profile_free_trajectory(state->trajectory);
        state->trajectory = NULL;
    }
    state->trajectory_points = 0;
    state->trajectory_index = 0;
    state->trajectory_active = false;

    float start_pos = state->current_position;
    bool success = motion_profile_generate_s_curve(
        start_pos,
        0.0f,                // start velocity
        target_position_rad, // end position
        0.0f,                // end velocity
        speed_rads,          // max velocity
        (speed_rads * 2.0f), // max acceleration
        MP_DEFAULT_MAX_JERK, // jerk (ignored)
        MP_TIME_STEP,
        &state->trajectory,
        &state->trajectory_points);

    if (!success || (state->trajectory == NULL))
    {
        ESP_LOGE(TAG, "Motor %d: Failed to generate trajectory", state->motor_id);
        return ESP_FAIL;
    }

    state->trajectory_active = true;
    ESP_LOGI(TAG, "Motor %d: Trajectory created with %d points.", state->motor_id, state->trajectory_points);
    return ESP_OK;
}

// -------------------------------------------------------------------
// Public command dispatcher
// -------------------------------------------------------------------
esp_err_t motor_control_handle_command(const motor_command_t *command)
{
    if (!command)
    {
        ESP_LOGE(TAG, "handle_command: null command pointer");
        return ESP_ERR_INVALID_ARG;
    }

    motor_state_t *state = motor_control_get_state(command->motor_id);
    if (!state)
    {
        return ESP_ERR_INVALID_ARG; // invalid motor ID
    }

    switch (command->cmd_type)
    {
    case MOTOR_CMD_ENTER_MODE:
        return handle_enter_mode(state);

    case MOTOR_CMD_EXIT_MODE:
        return handle_exit_mode(state);

    case MOTOR_CMD_ZERO_POS_SENSOR:
        return handle_zero_sensor(state);

    case MOTOR_CMD_MOVE:
        return handle_move_command(state, command->position, command->speed);

    default:
        ESP_LOGE(TAG, "Invalid command type for motor %d", command->motor_id);
        return ESP_ERR_INVALID_ARG;
    }
}
