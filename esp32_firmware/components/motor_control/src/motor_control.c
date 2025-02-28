#include "motor_control.h"
#include "can_bus.h"
#include "message_parser.h"
#include "motion_profile.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "robot_controller.h"

#define LOG_TAG "MOTOR_CONTROL"
#define CAN_CMD_LENGTH 8

// Per-motor state
static motor_state_t motor_states[NUM_MOTORS];

// Global flag indicating a batch is in progress.
static bool s_batch_in_progress = false; // remains static

void motor_control_set_batch_in_progress(bool in_progress)
{
    s_batch_in_progress = in_progress;
}

bool motor_control_is_batch_in_progress(void)
{
    return s_batch_in_progress;
}

// If true, we invert this motor's angles (commands & feedback).
static bool s_inverted[NUM_MOTORS] = {
    true,  // Motor 1 is inverted (example)
    false, // Motor 2 is normal
    false  // Motor 3 is normal
};

// Watchdog counters
static int s_motor_failure_count[NUM_MOTORS] = {0};

// Inline helpers for angle conversions
static inline float to_user_angle(int motor_id, float hardware_angle)
{
    return s_inverted[motor_id - 1] ? -hardware_angle : hardware_angle;
}

static inline float from_user_angle(int motor_id, float user_angle)
{
    return s_inverted[motor_id - 1] ? -user_angle : user_angle;
}

void motor_control_init(void)
{
    ESP_LOGI(LOG_TAG, "Initializing Motor Control Module...");
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        motor_states[i].motor_id = i + 1;
        motor_states[i].engaged = false;
        motor_states[i].trajectory = NULL;
        motor_states[i].trajectory_points = 0;
        motor_states[i].trajectory_index = 0;
        motor_states[i].trajectory_active = false;
        motor_states[i].current_position = 0.0f; // user-space
        // Initialize batch fields
        motor_states[i].batch_commands = NULL;
        motor_states[i].batch_count = 0;
        motor_states[i].batch_index = 0;
        motor_states[i].batch_error = false;
        motor_states[i].batch_error_message[0] = '\0';
        s_motor_failure_count[i] = 0;
        ESP_LOGI(LOG_TAG, "Motor %d initialized.", i + 1);
    }
}

motor_state_t *motor_control_get_state(int motor_id)
{
    if (motor_id < 1 || motor_id > NUM_MOTORS)
    {
        ESP_LOGE(LOG_TAG, "Invalid motor ID %d requested.", motor_id);
        return NULL;
    }
    return &motor_states[motor_id - 1];
}

esp_err_t motor_control_sync_position(int motor_id)
{
    motor_state_t *state = motor_control_get_state(motor_id);
    if (!state)
    {
        ESP_LOGE(LOG_TAG, "Invalid motor id %d for sync.", motor_id);
        return ESP_ERR_INVALID_ARG;
    }

    twai_message_t response;
    esp_err_t err;

    if (state->engaged)
    {
        // Use Enter Mode command to get updated reading
        err = can_bus_send_enter_mode(motor_id, &response);
    }
    else
    {
        // Use Exit Mode command if not engaged
        err = can_bus_send_exit_mode(motor_id, &response);
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Motor %d: Sync command failed: %s", motor_id, esp_err_to_name(err));
        return err;
    }

    if (response.data_length_code == 6)
    {
        motor_reply_t reply = {0};
        unpack_reply(response.data, &reply);

        float user_angle = to_user_angle(motor_id, reply.position);
        state->current_position = user_angle;

        ESP_LOGI(LOG_TAG,
                 "Motor %d: Sync => hardware=%.4f, velocity=%.4f => user=%.4f, current=%.2f",
                 motor_id, reply.position, reply.velocity, user_angle, reply.current);

        return ESP_OK;
    }
    else
    {
        ESP_LOGE(LOG_TAG,
                 "Motor %d: Sync expected 6-byte sensor reply, got %d bytes.",
                 motor_id, response.data_length_code);
        return ESP_FAIL;
    }
}

static float get_motor_max_speed_dps(int motor_id)
{
    switch (motor_id)
    {
    case 1:
        return MOTOR1_MAX_SPEED_DPS;
    case 2:
        return MOTOR2_MAX_SPEED_DPS;
    case 3:
        return MOTOR3_MAX_SPEED_DPS;
    default:
        return 180.0f; // fallback
    }
}

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
        ESP_LOGE(LOG_TAG, "Unknown motor ID %d in check_angle_range.", motor_id);
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
out_of_range:
    ESP_LOGE(LOG_TAG, "Motor %d: angle %.2f deg out of range", motor_id, angle_deg);
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t handle_move_command(motor_state_t *state, float target_position_rad, float speed_percentage)
{
    if (!state->engaged)
    {
        ESP_LOGE(LOG_TAG, "Motor %d is not engaged; must enter mode first.", state->motor_id);
        return ESP_ERR_INVALID_STATE;
    }

    if (state->trajectory_active)
    {
        ESP_LOGE(LOG_TAG, "Motor %d is already executing a trajectory.", state->motor_id);
        return ESP_ERR_INVALID_STATE;
    }

    // 1) Sync current position
    esp_err_t err = motor_control_sync_position(state->motor_id);
    if (err != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Motor %d: Failed to sync position before move.", state->motor_id);
        return err;
    }

    // 2) Check angle range
    float target_deg = target_position_rad * (180.0f / M_PI);
    err = check_angle_range(state->motor_id, target_deg);
    if (err != ESP_OK)
    {
        return err;
    }

    // 3) Compute max velocity in rad/s
    float motor_dps = get_motor_max_speed_dps(state->motor_id);
    float user_dps = (speed_percentage / 100.0f) * motor_dps;
    float user_rads = user_dps * (M_PI / 180.0f);

    // 4) Clear old trajectory if any
    if (state->trajectory)
    {
        motion_profile_free_trajectory(state->trajectory);
        state->trajectory = NULL;
    }
    state->trajectory_points = 0;
    state->trajectory_index = 0;
    state->trajectory_active = false;

    // 5) Generate S-curve
    ESP_LOGI(LOG_TAG, "Motor %d: Generating trajectory from %.4f rad to %.4f rad, max_vel=%.4f rad/s (%.0f%%)",
             state->motor_id, state->current_position, target_position_rad, user_rads, speed_percentage);

    bool success = motion_profile_generate_s_curve(
        state->current_position,
        0.0f, // start vel
        target_position_rad,
        0.0f,      // end vel
        user_rads, // speed limit
        user_rads*2, // acceleration limit, reach speed limit in half a second
        MP_DEFAULT_MAX_JERK,
        MP_TIME_STEP,
        &state->trajectory,
        &state->trajectory_points);
    if (!success)
    {
        ESP_LOGE(LOG_TAG, "Motor %d: Failed to generate trajectory", state->motor_id);
        return ESP_FAIL;
    }

    state->trajectory_active = true;
    state->trajectory_index = 0;
    ESP_LOGI(LOG_TAG, "Motor %d: Trajectory created with %d points.", state->motor_id, state->trajectory_points);

    return ESP_OK;
}

static esp_err_t handle_enter_mode(motor_state_t *state)
{
    if (state->engaged)
    {
        ESP_LOGI(LOG_TAG, "Motor %d already engaged.", state->motor_id);
        return ESP_OK;
    }

    twai_message_t response;
    esp_err_t err = can_bus_send_enter_mode(state->motor_id, &response);
    if (err == ESP_OK)
    {
        state->engaged = true;
        ESP_LOGI(LOG_TAG, "Motor %d: Enter Motor Mode successful.", state->motor_id);
    }
    else
    {
        ESP_LOGE(LOG_TAG, "Motor %d: Enter Mode failed: %s", state->motor_id, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t handle_exit_mode(motor_state_t *state)
{
    if (!state->engaged)
    {
        ESP_LOGI(LOG_TAG, "Motor %d already disengaged.", state->motor_id);
        return ESP_OK;
    }

    twai_message_t response;
    esp_err_t err = can_bus_send_exit_mode(state->motor_id, &response);
    if (err == ESP_OK)
    {
        state->engaged = false;
        ESP_LOGI(LOG_TAG, "Motor %d: Exit Motor Mode successful.", state->motor_id);
    }
    else
    {
        ESP_LOGE(LOG_TAG, "Motor %d: Exit Mode failed: %s", state->motor_id, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t handle_zero_sensor(motor_state_t *state)
{
    if (state->engaged)
    {
        ESP_LOGE(LOG_TAG, "Motor %d: Cannot zero sensor while engaged.", state->motor_id);
        return ESP_ERR_INVALID_STATE;
    }

    twai_message_t response;
    esp_err_t err = can_bus_send_zero_pos_sensor(state->motor_id, &response);
    if (err == ESP_OK)
    {
        state->current_position = 0.0f;
        ESP_LOGI(LOG_TAG, "Motor %d: Zero Position successful; user-space=0.0f", state->motor_id);
    }
    else
    {
        ESP_LOGE(LOG_TAG, "Motor %d: Zero sensor failed: %s", state->motor_id, esp_err_to_name(err));
    }
    return err;
}

esp_err_t motor_control_handle_command(const motor_command_t *command)
{
    if (!command)
    {
        ESP_LOGE(LOG_TAG, "NULL command pointer in handle_command");
        return ESP_ERR_INVALID_ARG;
    }
    motor_state_t *state = motor_control_get_state(command->motor_id);
    if (!state)
    {
        return ESP_ERR_INVALID_ARG;
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
        return handle_move_command(state, command->position, command->speed_percentage);

    default:
        ESP_LOGE(LOG_TAG, "Invalid command type for motor %d", command->motor_id);
        return ESP_ERR_INVALID_ARG;
    }
}

static void global_batch_abort(void)
{
    ESP_LOGE(LOG_TAG, "Global batch abort triggered.");
    // For each motor, clear its batch commands and reset batch state.
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        if (motor_states[i].batch_commands)
        {
            free(motor_states[i].batch_commands);
            motor_states[i].batch_commands = NULL;
        }
        motor_states[i].batch_count = 0;
        motor_states[i].batch_index = 0;
        motor_states[i].batch_error = true; // mark as error occurred
    }
    s_batch_in_progress = false;
}

void motor_control_task(void *arg)
{
    ESP_LOGI(LOG_TAG, "Starting Motor Control Task...");
    const TickType_t delay_ticks = pdMS_TO_TICKS((int)(MP_TIME_STEP * 1000));

    while (1)
    {
        // If the robot is not engaged, skip sending setpoints
        if (!robot_controller_is_engaged())
        {
            vTaskDelay(delay_ticks);
            continue;
        }

        // For each motor
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            motor_state_t *state = &motor_states[i];
            // Process trajectory if active (existing code)
            if (state->trajectory_active && state->trajectory_index < state->trajectory_points)
            {
                motion_profile_point_t *pt = &state->trajectory[state->trajectory_index];

                float kp_current = 0.0f;
                float kd_current = 0.0f;
                switch (state->motor_id)
                {
                case 1:
                    kp_current = KP1;
                    kd_current = KD1;
                    break;
                case 2:
                    kp_current = KP2;
                    kd_current = KD2;
                    break;
                case 3:
                    kp_current = KP3;
                    kd_current = KD3;
                    break;
                default:
                    break;
                }

                float hardware_pos = from_user_angle(state->motor_id, pt->position);
                float hardware_vel = from_user_angle(state->motor_id, pt->velocity);

                uint8_t can_msg_data[CAN_CMD_LENGTH] = {0};
                pack_cmd(hardware_pos, hardware_vel, kp_current, kd_current, 0.0f, can_msg_data);

                ESP_LOGI(LOG_TAG,
                         "Motor %d: setpoint [%d/%d] => user=(pos=%.4f,vel=%.4f) => hw=(pos=%.4f,vel=%.4f)",
                         state->motor_id,
                         state->trajectory_index + 1,
                         state->trajectory_points,
                         pt->position, pt->velocity,
                         hardware_pos, hardware_vel);

                twai_message_t response;
                esp_err_t err = can_bus_request_response(can_msg_data, CAN_CMD_LENGTH, state->motor_id, &response);
                if (err == ESP_OK)
                {
                    s_motor_failure_count[i] = 0;

                    if (response.data_length_code == 6)
                    {
                        motor_reply_t reply = {0};
                        unpack_reply(response.data, &reply);
                        float user_angle = to_user_angle(state->motor_id, reply.position);
                        state->current_position = user_angle;

                        ESP_LOGI(LOG_TAG,
                                 "Motor %d: Feedback => hw_pos=%.4f,hw_vel=%.4f => user_pos=%.4f, current=%.2f",
                                 state->motor_id, reply.position, reply.velocity, user_angle, reply.current);
                    }
                    else
                    {
                        ESP_LOGW(LOG_TAG,
                                 "Motor %d: No 6-byte feedback (got %d bytes).",
                                 state->motor_id, response.data_length_code);
                    }

                    // Advance trajectory index
                    state->trajectory_index++;
                    if (state->trajectory_index >= state->trajectory_points)
                    {
                        state->trajectory_active = false;
                        ESP_LOGI(LOG_TAG, "Motor %d: Trajectory completed.", state->motor_id);
                        // Free old trajectory
                        if (state->trajectory)
                        {
                            free(state->trajectory);
                            state->trajectory = NULL;
                        }
                    }
                }
                else
                {
                    s_motor_failure_count[i]++;
                    ESP_LOGE(LOG_TAG, "Motor %d: CAN transmit/receive failed. Consecutive errors = %d",
                             state->motor_id, s_motor_failure_count[i]);

                    if (s_motor_failure_count[i] >= 3)
                    {
                        // Trigger forced shutdown (global abort)
                        robot_controller_handle_motor_error(state->motor_id);
                        global_batch_abort();
                        break;
                    }
                }
            }
            // --- Batch processing ---
            // If no trajectory is active and a batch is loaded for this motor, process the next command.
            if (!state->trajectory_active && state->batch_commands && state->batch_index < state->batch_count)
            {
                motor_command_t cmd = state->batch_commands[state->batch_index];
                ESP_LOGI(LOG_TAG, "Motor %d: Processing batch command %d/%d",
                         state->motor_id, state->batch_index + 1, state->batch_count);
                esp_err_t err = motor_control_handle_command(&cmd);
                if (err == ESP_OK)
                {
                    state->batch_index++;
                    // Note: motor_control_handle_command will start a new trajectory,
                    // and its progress is handled above.
                }
                else
                {
                    snprintf(state->batch_error_message, sizeof(state->batch_error_message),
                             "Command %d failed: %s", state->batch_index, esp_err_to_name(err));
                    state->batch_error = true;
                    global_batch_abort();
                }
                // If we have finished all commands for this motor, clear the batch.
                if (state->batch_index >= state->batch_count)
                {
                    free(state->batch_commands);
                    state->batch_commands = NULL;
                    state->batch_count = 0;
                    state->batch_index = 0;
                }
            }
        }
        vTaskDelay(delay_ticks);
    }
    vTaskDelete(NULL);
}

esp_err_t motor_control_move_blocking(int motor_id, float target_position_rad, int timeout_ticks)
{
    motor_state_t *state = motor_control_get_state(motor_id);
    if (!state)
    {
        return ESP_ERR_INVALID_ARG;
    }

    motor_command_t cmd = {
        .motor_id = motor_id,
        .cmd_type = MOTOR_CMD_MOVE,
        .position = target_position_rad,
        .speed_percentage = 50.0f // default 50% speed
    };
    esp_err_t err = motor_control_handle_command(&cmd);
    if (err != ESP_OK)
    {
        return err;
    }

    TickType_t start = xTaskGetTickCount();
    while (1)
    {
        // If robot is off mid-move, abort
        if (!robot_controller_is_engaged())
        {
            return ESP_ERR_INVALID_STATE;
        }

        // If trajectory is finished
        if (!state->trajectory_active)
        {
            return ESP_OK;
        }

        if ((xTaskGetTickCount() - start) > timeout_ticks)
        {
            ESP_LOGW(LOG_TAG, "motor_control_move_blocking: Motor %d timed out waiting for trajectory finish.", motor_id);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
