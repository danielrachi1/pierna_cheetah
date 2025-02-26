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

static motor_state_t motor_states[NUM_MOTORS];

// Watchdog counters for each motor. If a motor hits 3 consecutive timeouts, forced shutdown triggers.
static int s_motor_failure_count[NUM_MOTORS] = {0};

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
        motor_states[i].current_position = 0.0f;
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
        state->current_position = reply.position;
        ESP_LOGI(LOG_TAG,
                 "Motor %d: Sync => position=%.4f, velocity=%.4f, current=%.2f",
                 motor_id, reply.position, reply.velocity, reply.current);
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
        return MOTOR1_MAX_SPEED_DPS;
    }
}

/**
 * @brief Helper for handling a "move" command (MOTOR_CMD_MOVE).
 *        speed_percentage is 0..100 => scaled maximum velocity.
 */
static esp_err_t handle_move_command(motor_state_t *state, float target_position_rad, float speed_percentage)
{
    if (!state->engaged)
    {
        ESP_LOGE(LOG_TAG, "Motor %d is not engaged; must enter mode first.", state->motor_id);
        return ESP_ERR_INVALID_STATE;
    }

    if (state->trajectory_active)
    {
        ESP_LOGE(LOG_TAG, "Motor %d is already executing a trajectory; can't accept new move.", state->motor_id);
        return ESP_ERR_INVALID_STATE;
    }

    // Sync current position
    esp_err_t err = motor_control_sync_position(state->motor_id);
    if (err != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Motor %d: Failed to sync position before move.", state->motor_id);
        return err;
    }

    // Validate degrees
    float target_deg = target_position_rad * (180.0f / M_PI);
    float final_position_rad;
    switch (state->motor_id)
    {
    case 1:
        if (target_deg < MOTOR1_MIN_ANGLE_DEG || target_deg > MOTOR1_MAX_ANGLE_DEG)
        {
            ESP_LOGE(LOG_TAG, "Motor 1: Commanded angle %.2f deg out of range (%.2f - %.2f)",
                     target_deg, MOTOR1_MIN_ANGLE_DEG, MOTOR1_MAX_ANGLE_DEG);
            return ESP_ERR_INVALID_ARG;
        }
        // invert angle for motor 1
        final_position_rad = -target_position_rad;
        break;
    case 2:
        if (target_deg < MOTOR2_MIN_ANGLE_DEG || target_deg > MOTOR2_MAX_ANGLE_DEG)
        {
            ESP_LOGE(LOG_TAG, "Motor 2: Commanded angle %.2f deg out of range (%.2f - %.2f)",
                     target_deg, MOTOR2_MIN_ANGLE_DEG, MOTOR2_MAX_ANGLE_DEG);
            return ESP_ERR_INVALID_ARG;
        }
        final_position_rad = target_position_rad;
        break;
    case 3:
        if (target_deg < MOTOR3_MIN_ANGLE_DEG || target_deg > MOTOR3_MAX_ANGLE_DEG)
        {
            ESP_LOGE(LOG_TAG, "Motor 3: Commanded angle %.2f deg out of range (%.2f - %.2f)",
                     target_deg, MOTOR3_MIN_ANGLE_DEG, MOTOR3_MAX_ANGLE_DEG);
            return ESP_ERR_INVALID_ARG;
        }
        final_position_rad = target_position_rad;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    // Compute max velocity from speed percentage
    float motor_dps = get_motor_max_speed_dps(state->motor_id);
    float user_dps = (speed_percentage / 100.0f) * motor_dps; // 0..motor_dps
    float user_rads = user_dps * (M_PI / 180.0f);

    // Clear old trajectory if any
    if (state->trajectory)
    {
        motion_profile_free_trajectory(state->trajectory);
        state->trajectory = NULL;
    }
    state->trajectory_points = 0;
    state->trajectory_index = 0;
    state->trajectory_active = false;

    ESP_LOGI(LOG_TAG, "Motor %d: Generating S‑curve from %.4f to %.4f (rad) with max_vel=%.4f rad/s (%.2f%%)",
             state->motor_id, state->current_position, final_position_rad, user_rads, speed_percentage);

    bool success = motion_profile_generate_s_curve(
        state->current_position,
        0.0f, // start velocity
        final_position_rad,
        0.0f,      // end velocity
        user_rads, // max velocity depends on user's speed %
        MP_DEFAULT_MAX_ACC,
        MP_DEFAULT_MAX_JERK,
        MP_TIME_STEP,
        &state->trajectory,
        &state->trajectory_points);
    if (!success)
    {
        ESP_LOGE(LOG_TAG, "Motor %d: Failed to generate S‑curve.", state->motor_id);
        return ESP_FAIL;
    }

    state->trajectory_active = true;
    state->trajectory_index = 0;
    ESP_LOGI(LOG_TAG, "Motor %d: S‑curve created with %d points.",
             state->motor_id, state->trajectory_points);

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
        ESP_LOGE(LOG_TAG, "Motor %d: Enter Mode failed: %s",
                 state->motor_id, esp_err_to_name(err));
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
        ESP_LOGE(LOG_TAG, "Motor %d: Exit Mode failed: %s",
                 state->motor_id, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t handle_zero_sensor(motor_state_t *state)
{
    // Setting zero sensor while engaged is dangerous
    if (state->engaged)
    {
        ESP_LOGE(LOG_TAG,
                 "Motor %d: Cannot zero sensor while engaged.",
                 state->motor_id);
        return ESP_ERR_INVALID_STATE;
    }

    twai_message_t response;
    esp_err_t err = can_bus_send_zero_pos_sensor(state->motor_id, &response);
    if (err == ESP_OK)
    {
        state->current_position = 0.0f;
        ESP_LOGI(LOG_TAG, "Motor %d: Zero Position successful; position=0.0f", state->motor_id);
    }
    else
    {
        ESP_LOGE(LOG_TAG, "Motor %d: Zero sensor failed: %s",
                 state->motor_id, esp_err_to_name(err));
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

void motor_control_task(void *arg)
{
    ESP_LOGI(LOG_TAG, "Starting Motor Control Task...");
    const TickType_t delay_ticks = pdMS_TO_TICKS((int)(MP_TIME_STEP * 1000));

    while (1)
    {
        robot_state_t rstate = robot_controller_get_state();

        // Skip sending setpoints if we're OFF, SHUTTING_DOWN, or in ERROR
        if (rstate == ROBOT_STATE_SHUTTING_DOWN ||
            rstate == ROBOT_STATE_ERROR ||
            rstate == ROBOT_STATE_OFF)
        {
            vTaskDelay(delay_ticks);
            continue;
        }

        // Normal operation
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            motor_state_t *state = &motor_states[i];
            if (state->trajectory_active && state->trajectory_index < state->trajectory_points)
            {
                // Next point
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

                // Pack data
                uint8_t can_msg_data[CAN_CMD_LENGTH] = {0};
                pack_cmd(pt->position, pt->velocity, kp_current, kd_current, 0.0f, can_msg_data);

                ESP_LOGI(LOG_TAG,
                         "Motor %d: setpoint [%d/%d] => pos=%.4f, vel=%.4f",
                         state->motor_id,
                         state->trajectory_index + 1,
                         state->trajectory_points,
                         pt->position,
                         pt->velocity);

                twai_message_t response;
                esp_err_t err = can_bus_request_response(can_msg_data, CAN_CMD_LENGTH, state->motor_id, &response);

                if (err == ESP_OK)
                {
                    // Reset failure count for this motor
                    s_motor_failure_count[i] = 0;

                    if (response.data_length_code == 6)
                    {
                        motor_reply_t reply = {0};
                        unpack_reply(response.data, &reply);
                        state->current_position = reply.position;

                        ESP_LOGI(LOG_TAG,
                                 "Motor %d: Feedback => pos=%.4f, vel=%.4f, current=%.2f",
                                 state->motor_id, reply.position, reply.velocity, reply.current);
                    }
                    else
                    {
                        ESP_LOGW(LOG_TAG,
                                 "Motor %d: No 6-byte feedback (got %d bytes).",
                                 state->motor_id, response.data_length_code);
                    }

                    // Move to next trajectory point
                    state->trajectory_index++;
                    if (state->trajectory_index >= state->trajectory_points)
                    {
                        state->trajectory_active = false;
                        ESP_LOGI(LOG_TAG, "Motor %d: Trajectory completed.", state->motor_id);
                    }
                }
                else
                {
                    // CAN failure or timeout
                    s_motor_failure_count[i]++;
                    ESP_LOGE(LOG_TAG, "Motor %d: CAN transmit/receive failed. Consecutive errors = %d",
                             state->motor_id, s_motor_failure_count[i]);

                    if (s_motor_failure_count[i] >= 3)
                    {
                        // Trigger forced shutdown
                        robot_controller_handle_motor_error(state->motor_id);
                        // Break out of loop
                        break;
                    }
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

    // We'll default to speed=10% for blocking moves (you can adjust as needed).
    motor_command_t cmd = {
        .motor_id = motor_id,
        .cmd_type = MOTOR_CMD_MOVE,
        .position = target_position_rad,
        .speed_percentage = 10.0f};
    esp_err_t err = motor_control_handle_command(&cmd);
    if (err != ESP_OK)
    {
        return err;
    }

    // Wait until trajectory completes or time out
    TickType_t start = xTaskGetTickCount();
    while (1)
    {
        // If in error or shutting down (or turned off), stop
        robot_state_t rstate = robot_controller_get_state();
        if (rstate == ROBOT_STATE_ERROR || rstate == ROBOT_STATE_SHUTTING_DOWN || rstate == ROBOT_STATE_OFF)
        {
            return ESP_ERR_INVALID_STATE;
        }

        // Check trajectory
        if (!state->trajectory_active)
        {
            // Completed
            return ESP_OK;
        }
        // Check timeout
        TickType_t now = xTaskGetTickCount();
        if ((now - start) > timeout_ticks)
        {
            ESP_LOGW(LOG_TAG, "motor_control_move_blocking: Motor %d timed out waiting for trajectory finish.", motor_id);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    // Unreachable
}
