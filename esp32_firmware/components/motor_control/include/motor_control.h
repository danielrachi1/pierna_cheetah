#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include "motion_profile.h"
#include "esp_err.h"

/**
 * @brief Number of motors the system supports.
 */
#define NUM_MOTORS 3

/**
 * @brief Enumeration of possible motor commands.
 */
typedef enum
{
    MOTOR_CMD_MOVE,           ///< Go to position (S-curve trajectory).
    MOTOR_CMD_ENTER_MODE,     ///< Enter motor mode.
    MOTOR_CMD_EXIT_MODE,      ///< Exit motor mode.
    MOTOR_CMD_ZERO_POS_SENSOR ///< Zero the position sensor.
} motor_command_type_t;

/**
 * @brief Structure that holds one motor command.
 *
 * For MOTOR_CMD_MOVE, the position field (in radians) is used.
 * For other commands, position is ignored.
 */
typedef struct
{
    int motor_id;                  ///< Motor identifier (1-indexed)
    motor_command_type_t cmd_type; ///< Type of command
    float position;                ///< Used only if cmd_type == MOTOR_CMD_MOVE, in radians
} motor_command_t;

/**
 * @brief A single motor's state.
 */
typedef struct
{
    int motor_id;                       ///< Motor identifier (1-indexed).
    bool engaged;                       ///< True if in motor mode.
    motion_profile_point_t *trajectory; ///< Active trajectory setpoints, or NULL if none.
    int trajectory_points;              ///< Total number of points in the trajectory.
    int trajectory_index;               ///< Current trajectory index.
    bool trajectory_active;             ///< True if a trajectory is being executed.
    float current_position;             ///< Last known sensor reading.
} motor_state_t;

/**
 * @brief Initializes the motor control module and motor states.
 */
void motor_control_init(void);

/**
 * @brief Retrieves the state structure for a given motor.
 *
 * @param motor_id 1-based motor ID
 * @return Pointer to motor_state_t or NULL if invalid ID
 */
motor_state_t *motor_control_get_state(int motor_id);

/**
 * @brief Task that sends trajectory setpoints to each motor until the trajectory is done.
 * @param arg Unused
 */
void motor_control_task(void *arg);

/**
 * @brief Syncs the current motor position from the motor sensor.
 *
 * If engaged, uses ENTER_MODE message; if disengaged, uses EXIT_MODE message
 * to trigger a sensor read.
 *
 * @param motor_id The motor ID to sync
 * @return ESP_OK on success, or an error code
 */
esp_err_t motor_control_sync_position(int motor_id);

/**
 * @brief Processes an incoming motor command based on its type.
 *
 * @param command Pointer to the motor_command_t structure.
 * @return esp_err_t ESP_OK on success or an error code on failure.
 */
esp_err_t motor_control_handle_command(const motor_command_t *command);

#endif // MOTOR_CONTROL_H
