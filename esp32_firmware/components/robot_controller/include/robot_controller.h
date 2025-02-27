#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Initializes the robot controller (reads NVS, etc.).
 */
void robot_controller_init(void);

/**
 * @brief Returns true if the robot is "engaged" (on/ready), false if "off".
 */
bool robot_controller_is_engaged(void);

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
 *  5) Mark robot as engaged
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
 *  5) Mark robot as off
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
