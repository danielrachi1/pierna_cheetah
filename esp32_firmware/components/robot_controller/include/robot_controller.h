#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Robot operational states.
 */
typedef enum
{
    ROBOT_STATE_OFF,           ///< Robot is fully off, relay disabled, motors unpowered
    ROBOT_STATE_ENGAGED_READY, ///< Relay on, motors engaged, ready to accept position commands
} robot_state_t;

/**
 * @brief Initializes the robot controller, including creation of state mutex.
 */
void robot_controller_init(void);

/**
 * @brief Retrieves the current robot state.
 *
 * @return The current state of the robot.
 */
robot_state_t robot_controller_get_state(void);

/**
 * @brief Sets the robot state (thread-safe).
 *
 * @param new_state The state to set.
 */
void robot_controller_set_state(robot_state_t new_state);

/**
 * @brief Checks if the motors were engaged on the previous run.
 *
 * This reads from NVS to determine if the "motors_engaged" flag is set.
 *
 * @return true if motors were engaged; false otherwise.
 */
bool robot_controller_get_motors_engaged_flag(void);

/**
 * @brief Attempts to turn on the robot:
 *  1) Relay ON
 *  2) Zero sensors on all motors
 *  3) Enter motor mode on all motors
 *  4) Set NVS flag (motors_engaged = true)
 *  5) Robot goes to ROBOT_STATE_ENGAGED_READY on success
 *
 * @return ESP_OK on success; otherwise calls forced shutdown internally.
 */
esp_err_t robot_controller_turn_on(void);

/**
 * @brief Attempts to turn off the robot:
 *  1) Move all motors to home (0.0)
 *  2) Exit motor mode on all motors
 *  3) Disable relay
 *  4) Clear NVS flag (motors_engaged = false)
 *  5) Robot goes to ROBOT_STATE_OFF
 *
 * @return ESP_OK on success.
 */
esp_err_t robot_controller_turn_off(void);

/**
 * @brief Called when a motor experiences repeated CAN timeouts or other critical fault.
 *
 * Sets the state to ROBOT_STATE_ERROR and invokes forced shutdown.
 *
 * @param motor_id The motor that triggered the error.
 */
void robot_controller_handle_motor_error(int motor_id);

#endif // ROBOT_CONTROLLER_H
