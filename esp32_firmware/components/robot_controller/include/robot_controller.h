#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Initializes the robot controller:
 *        - Sets internal flags to "off"
 */
void robot_controller_init(void);

/**
 * @brief Returns true if the robot is currently "engaged" (on/ready), false if "off".
 */
bool robot_controller_is_engaged(void);

/**
 * @brief Returns true if the firmware is in a "recovery needed" state, meaning
 *        the user must physically power off motors, home them, then call
 *        /api/recovery/clear before proceeding.
 */
bool robot_controller_is_recovery_needed(void);

/**
 * @brief Sets the "recovery needed" flag.
 */
void robot_controller_set_recovery_needed(bool needed);

/**
 * @brief Clears the "recovery needed" flag (i.e., user acknowledges they’ve performed the required steps).
 *
 * Also clears the NVS flag so that the system can safely turn on.
 *
 * @return ESP_OK on success.
 */
esp_err_t robot_controller_clear_recovery(void);

/**
 * @brief Checks if the motors were engaged on the previous run (via NVS).
 * @return true if motors_engaged == 1 in NVS; false otherwise.
 */
bool robot_controller_get_motors_engaged_flag(void);

/**
 * @brief Attempts to turn on the robot:
 *  1) If in recovery state, refuses to proceed.
 *  2) Relay ON
 *  3) Zero sensors on all motors
 *  4) Enter motor mode on all motors
 *  5) Set NVS flag (motors_engaged = true)
 *  6) Mark robot as engaged
 *
 * @return ESP_OK on success; otherwise an error.
 */
esp_err_t robot_controller_turn_on(void);

/**
 * @brief Attempts to turn off the robot:
 *  1) If engaged, move all motors to home (0.0) and exit motor mode.
 *  2) Disable relay
 *  3) Clear NVS flag (motors_engaged = false)
 *  4) Mark robot as off
 *
 * @return ESP_OK on success.
 */
esp_err_t robot_controller_turn_off(void);

/**
 * @brief Called when a motor experiences repeated CAN timeouts or other critical fault.
 *
 * Invokes forced shutdown internally.
 *
 * @param motor_id The motor that triggered the error.
 */
void robot_controller_handle_motor_error(int motor_id);

#endif // ROBOT_CONTROLLER_H
