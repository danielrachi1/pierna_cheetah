#include "motor_control_task.h"
#include "motor_control_core.h"
#include "motor_command_handlers.h"
#include "robot_controller.h"
#include "can_bus.h"
#include "message_parser.h"
#include "motion_profile.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "MOTOR_CONTROL_TASK";

// We keep a separate failure counter for each motor:
static int s_motor_failure_count[NUM_MOTORS] = {0};

/**
 * @brief If we detect a batch error or repeated CAN failures, abort all batch commands system-wide.
 */
static void global_batch_abort(void)
{
    ESP_LOGE(TAG, "Global batch abort triggered due to error.");
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        motor_state_t *st = motor_control_get_state(i + 1);
        if (st->batch_commands)
        {
            free(st->batch_commands);
            st->batch_commands = NULL;
        }
        st->batch_count = 0;
        st->batch_index = 0;
        st->batch_error = true;
    }
    motor_control_set_batch_in_progress(false);
    ESP_LOGI(TAG, "All batch commands aborted.");
}

/**
 * @brief Send the next point of the trajectory (if active) and parse the feedback.
 *        Increments the motor's trajectory index. If done, frees the old trajectory.
 */
static void process_motor_trajectory(motor_state_t *state)
{
    if (!state->trajectory_active || (state->trajectory_index >= state->trajectory_points))
    {
        return; // No active trajectory to process
    }

    // Grab next setpoint
    motion_profile_point_t *pt = &state->trajectory[state->trajectory_index];

    // We retrieve the motor-specific gains
    float kp = 0.0f;
    float kd = 0.0f;
    switch (state->motor_id)
    {
    case 1:
        kp = KP1;
        kd = KD1;
        break;
    case 2:
        kp = KP2;
        kd = KD2;
        break;
    case 3:
        kp = KP3;
        kd = KD3;
        break;
    default:
        break;
    }

    // The hardware expects angles in "hardware space" (account for s_inverted).
    float hw_pos = from_user_angle(state->motor_id, pt->position);
    float hw_vel = from_user_angle(state->motor_id, pt->velocity);

    uint8_t can_msg_data[8];
    memset(can_msg_data, 0, sizeof(can_msg_data));
    pack_cmd(hw_pos, hw_vel, kp, kd, 0.0f, can_msg_data);

    twai_message_t response;
    esp_err_t err = can_bus_request_response(can_msg_data, 8, state->motor_id, &response);
    if (err != ESP_OK)
    {
        // Increment the failure counter
        int idx = state->motor_id - 1;
        s_motor_failure_count[idx]++;
        ESP_LOGE(TAG, "Motor %d: CAN TX/RX failed. Consecutive errors=%d",
                 state->motor_id, s_motor_failure_count[idx]);

        if (s_motor_failure_count[idx] >= 3)
        {
            // Too many consecutive failures, force global shutdown
            robot_controller_handle_motor_error(state->motor_id);
            global_batch_abort();
        }
        return; // skip the rest of the logic on this iteration
    }

    // If we get here, we had a successful exchange:
    int idx = state->motor_id - 1;
    s_motor_failure_count[idx] = 0; // reset on success

    if (response.data_length_code == 6)
    {
        // parse the new position feedback
        motor_reply_t reply;
        memset(&reply, 0, sizeof(reply));
        unpack_reply(response.data, &reply);

        float user_angle = to_user_angle(state->motor_id, reply.position);
        state->current_position = user_angle;

        ESP_LOGI(TAG,
                 "Motor %d: Traj idx=%d/%d => hw=(%.4f pos, %.4f vel) => user=(%.4f), feedbackCur=%.2f",
                 state->motor_id, state->trajectory_index + 1, state->trajectory_points,
                 reply.position, reply.velocity, user_angle, reply.current);
    }

    // Increment trajectory index
    state->trajectory_index++;
    if (state->trajectory_index >= state->trajectory_points)
    {
        // Trajectory completed
        state->trajectory_active = false;
        if (state->trajectory)
        {
            motion_profile_free_trajectory(state->trajectory);
            state->trajectory = NULL;
        }
        ESP_LOGI(TAG, "Motor %d: Trajectory finished.", state->motor_id);
    }
}

/**
 * @brief If the motor is not currently moving, but has batch commands left, handle the next batch command.
 *        If a command fails, mark error and do a global batch abort.
 */
static void process_motor_batch(motor_state_t *state)
{
    if (state->trajectory_active)
    {
        return; // motor is busy executing a trajectory
    }
    if (!state->batch_commands || (state->batch_index >= state->batch_count))
    {
        return; // no more commands
    }

    ESP_LOGI(TAG, "Motor %d: Processing batch command %d/%d",
             state->motor_id, state->batch_index + 1, state->batch_count);

    esp_err_t err = motor_control_handle_command(&state->batch_commands[state->batch_index]);
    if (err == ESP_OK)
    {
        state->batch_index++;
        // If that was a move command, it may have started a trajectory...
    }
    else
    {
        // Mark local error
        state->batch_error = true;
        snprintf(state->batch_error_message, sizeof(state->batch_error_message),
                 "Command %d failed: %s",
                 state->batch_index, esp_err_to_name(err));
        global_batch_abort();
        return;
    }

    // If we finished all commands, release the memory
    if (state->batch_index >= state->batch_count)
    {
        free(state->batch_commands);
        state->batch_commands = NULL;
        state->batch_count = 0;
        state->batch_index = 0;
        ESP_LOGI(TAG, "Motor %d: Finished all batch commands.", state->motor_id);
    }
}

void motor_control_task(void *arg)
{
    ESP_LOGI(TAG, "Starting Motor Control Task...");
    const TickType_t delay_ticks = pdMS_TO_TICKS((int)(MP_TIME_STEP * 1000)); // same as your motion profile step

    while (1)
    {
        // If the robot is off, do nothing but wait
        if (!robot_controller_is_engaged())
        {
            vTaskDelay(delay_ticks);
            continue;
        }

        // Otherwise, for each motor, step through trajectory + batch
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            motor_state_t *st = motor_control_get_state(i + 1);

            // 1) process trajectory (if active)
            process_motor_trajectory(st);

            // 2) process next batch command (if any)
            process_motor_batch(st);
        }

        // sleep for one motion-profile increment
        vTaskDelay(delay_ticks);
    }

    // vTaskDelete(NULL); // Typically unreachable
}
