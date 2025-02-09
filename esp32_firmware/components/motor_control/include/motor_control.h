#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include "motion_profile.h"
#include "message_parser.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_MOTORS 3

/**
 * @brief Structure that holds the state for a single motor.
 */
typedef struct {
    int motor_id;                           ///< Motor identifier (1-indexed)
    bool engaged;                           ///< Whether the motor is in motor mode
    motion_profile_point_t *trajectory;     ///< Pointer to the active trajectory setpoints
    int trajectory_points;                  ///< Total number of trajectory points
    int trajectory_index;                   ///< Current index in the trajectory
    bool trajectory_active;                 ///< Flag indicating if a trajectory is running
    float current_position;                 ///< Latest sensor reading for this motor
} motor_state_t;

/**
 * @brief Initializes the motor control module and per-motor state.
 */
void motor_control_init(void);

/**
 * @brief Retrieves the state structure for a given motor.
 *
 * @param motor_id The motor's identifier (1-indexed).
 * @return Pointer to the corresponding motor_state_t, or NULL if invalid.
 */
motor_state_t* motor_control_get_state(int motor_id);

/**
 * @brief Task that sends trajectory setpoints to each motor as needed.
 *
 * This task iterates over all motors and, for any motor with an active trajectory,
 * sends the current setpoint via CAN.
 *
 * @param arg Unused.
 */
void motor_control_task(void *arg);

/**
 * @brief Synchronizes a motor's target (by forcing a sensor update) and enters motor mode.
 *
 * @param motor_id The identifier of the motor.
 */
esp_err_t sync_and_engage_motor_control(int motor_id);

/**
 * @brief Processes an incoming motor command.
 *
 * Based on the parsed command and any special command string, this function either
 * synchronizes/engages the motor, generates a trajectory, or sends a direct CAN message.
 *
 * @param command Pointer to a motor_command_t structure.
 * @param special_command A string with a special command (if any).
 */
void motor_control_handle_command(const motor_command_t *command, const char *special_command);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROL_H
