#include "motor_control.h"
#include "can_bus.h"          // For CAN transmission functions
#include "message_parser.h"   // For pack_cmd() and unpack_reply()
#include "motion_profile.h"   // For trajectory generation/freeing
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define LOG_TAG "MOTOR_CONTROL"
#define CAN_CMD_LENGTH 8

// Controller gains for each motor (can be extended to per-motor parameters)
#define KP1 10
#define KD1 0
#define KP2 0
#define KD2 0
#define KP3 0
#define KD3 0

// Array holding the state for each motor.
static motor_state_t motor_states[NUM_MOTORS];

void motor_control_init(void)
{
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_states[i].motor_id = i + 1;
        motor_states[i].engaged = false;
        motor_states[i].trajectory = NULL;
        motor_states[i].trajectory_points = 0;
        motor_states[i].trajectory_index = 0;
        motor_states[i].trajectory_active = false;
        motor_states[i].current_position = 0.0f;
    }
}

motor_state_t* motor_control_get_state(int motor_id)
{
    if (motor_id < 1 || motor_id > NUM_MOTORS) {
        return NULL;
    }
    return &motor_states[motor_id - 1];
}

void sync_and_engage_motor_control(int motor_id)
{
    motor_state_t *state = motor_control_get_state(motor_id);
    if (!state) {
        ESP_LOGE(LOG_TAG, "Invalid motor id %d", motor_id);
        return;
    }

    uint8_t dummy_cmd[CAN_CMD_LENGTH] = {0};
    pack_cmd(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, dummy_cmd);
    if (can_bus_transmit(dummy_cmd, CAN_CMD_LENGTH, motor_id) != ESP_OK) {
        ESP_LOGE(LOG_TAG, "Motor ID %d: Failed to send dummy command for synchronization", motor_id);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Wait for sensor reply (this is a simplified approach; ideally, sensor data would be updated asynchronously)
    twai_message_t sensor_msg;
    bool sensor_received = false;
    const int max_attempts = 5;
    for (int i = 0; i < max_attempts; i++) {
        if (can_bus_receive(&sensor_msg) == ESP_OK &&
            sensor_msg.data_length_code == 6 &&
            sensor_msg.identifier == motor_id)
        {
            sensor_received = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!sensor_received) {
        ESP_LOGE(LOG_TAG, "Motor ID %d: No sensor reply received during synchronization", motor_id);
        return;
    }

    motor_reply_t reply = {0};
    unpack_reply(sensor_msg.data, &reply);
    ESP_LOGI(LOG_TAG, "Motor ID %d: Sensor reading: position=%.4f, velocity=%.4f, current=%.2f",
             motor_id, reply.position, reply.velocity, reply.current);

    // Update the motor's current position
    state->current_position = reply.position;

    uint8_t hold_cmd[CAN_CMD_LENGTH] = {0};
    pack_cmd(reply.position, 0.0f, 10, 1, 0.0f, hold_cmd);
    if (can_bus_transmit(hold_cmd, CAN_CMD_LENGTH, motor_id) != ESP_OK) {
        ESP_LOGE(LOG_TAG, "Motor ID %d: Failed to send hold command", motor_id);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    if (can_bus_send_enter_mode(motor_id) == ESP_OK) {
        ESP_LOGI(LOG_TAG, "Motor ID %d: Enter motor mode command sent successfully", motor_id);
    } else {
        ESP_LOGE(LOG_TAG, "Motor ID %d: Failed to send enter motor mode command", motor_id);
    }
}

void motor_control_task(void *arg)
{
    const TickType_t delay_ticks = pdMS_TO_TICKS((int)(MP_TIME_STEP * 1000));
    while (1) {
        // Iterate over each motor's state and send trajectory setpoints if active.
        for (int i = 0; i < NUM_MOTORS; i++) {
            motor_state_t *state = &motor_states[i];
            if (state->trajectory_active && state->trajectory_index < state->trajectory_points) {
                motion_profile_point_t *pt = &state->trajectory[state->trajectory_index];
                float kp_current = 0.0f, kd_current = 0.0f;
                switch (state->motor_id) {
                    case 1: kp_current = KP1; kd_current = KD1; break;
                    case 2: kp_current = KP2; kd_current = KD2; break;
                    case 3: kp_current = KP3; kd_current = KD3; break;
                    default: kp_current = 0.0f; kd_current = 0.0f; break;
                }
                uint8_t can_msg_data[CAN_CMD_LENGTH] = {0};
                pack_cmd(pt->position, pt->velocity, kp_current, kd_current, 0.0f, can_msg_data);
                if (can_bus_transmit(can_msg_data, CAN_CMD_LENGTH, state->motor_id) == ESP_OK) {
                    ESP_LOGI(LOG_TAG, "Motor %d: Trajectory point %d/%d | pos=%.3f, vel=%.3f, kp=%.3f, kd=%.3f",
                             state->motor_id, state->trajectory_index, state->trajectory_points,
                             pt->position, pt->velocity, kp_current, kd_current);
                }
                state->trajectory_index++;
                if (state->trajectory_index >= state->trajectory_points) {
                    state->trajectory_active = false;
                    ESP_LOGI(LOG_TAG, "Motor %d: Trajectory completed.", state->motor_id);
                }
            }
        }
        vTaskDelay(delay_ticks);
    }
    vTaskDelete(NULL);
}

void motor_control_handle_command(const motor_command_t *command, const char *special_command)
{
    motor_state_t *state = motor_control_get_state(command->motor_id);
    if (!state) {
        ESP_LOGE(LOG_TAG, "Invalid motor id %d", command->motor_id);
        return;
    }

    if (special_command && strlen(special_command) > 0) {
        if (strcmp(special_command, "ENTER_MODE") == 0) {
            if (!state->engaged) {
                sync_and_engage_motor_control(state->motor_id);
                state->engaged = true;
            } else {
                ESP_LOGI(LOG_TAG, "Motor ID %d already engaged.", state->motor_id);
            }
        } else if (strcmp(special_command, "EXIT_MODE") == 0) {
            if (state->engaged) {
                if (can_bus_send_exit_mode(state->motor_id) == ESP_OK) {
                    ESP_LOGI(LOG_TAG, "Exit Motor Mode command sent successfully for Motor %d", state->motor_id);
                    state->engaged = false;
                } else {
                    ESP_LOGE(LOG_TAG, "Failed to send Exit Motor Mode command for Motor %d", state->motor_id);
                }
            } else {
                ESP_LOGI(LOG_TAG, "Motor ID %d already disengaged.", state->motor_id);
            }
        } else if (strcmp(special_command, "ZERO_POS") == 0) {
            if (!state->engaged) {
                if (can_bus_send_zero_pos_sensor(state->motor_id) == ESP_OK) {
                    ESP_LOGI(LOG_TAG, "Zero Position Sensor command sent successfully for Motor %d", state->motor_id);
                } else {
                    ESP_LOGE(LOG_TAG, "Failed to send Zero Position Sensor command for Motor %d", state->motor_id);
                }
            } else {
                ESP_LOGE(LOG_TAG, "Cannot set sensor zero while Motor %d is engaged! Command rejected.", state->motor_id);
            }
        }
    } else {
        // Handle a move (position) command.
        if (!state->engaged) {
            ESP_LOGE(LOG_TAG, "Motor ID %d is not engaged. Please enter motor mode first.", state->motor_id);
            return;
        }
        bool position_only = (command->velocity == 0.0f &&
                              command->kp == 0.0f &&
                              command->kd == 0.0f &&
                              command->feed_forward_torque == 0.0f);
        if (position_only) {
            if (state->trajectory) {
                motion_profile_free_trajectory(state->trajectory);
                state->trajectory = NULL;
                state->trajectory_points = 0;
                state->trajectory_index = 0;
                state->trajectory_active = false;
            }
            if (motion_profile_generate_s_curve(
                    state->current_position, 0.0f,
                    command->position, 0.0f,
                    MP_DEFAULT_MAX_VEL, MP_DEFAULT_MAX_ACC, MP_DEFAULT_MAX_JERK, MP_TIME_STEP,
                    &state->trajectory, &state->trajectory_points))
            {
                ESP_LOGI(LOG_TAG, "Motor %d: S-curve trajectory generated with %d points", state->motor_id, state->trajectory_points);
                state->trajectory_active = true;
                state->trajectory_index = 0;
            } else {
                ESP_LOGE(LOG_TAG, "Motor %d: Failed to generate S-curve trajectory", state->motor_id);
            }
        }
    }
}
