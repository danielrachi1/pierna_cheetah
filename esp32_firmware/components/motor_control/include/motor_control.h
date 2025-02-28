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
    MOTOR_CMD_MOVE,           ///< Go to position (S‑curve trajectory).
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
    int motor_id;                  ///< Motor identifier (1-based)
    motor_command_type_t cmd_type; ///< Type of command
    float position;                ///< Used only if cmd_type == MOTOR_CMD_MOVE, in radians
    float speed_percentage;        ///< Speed setting 0..100 for MOTOR_CMD_MOVE
} motor_command_t;

/**
 * @brief A single motor's state.
 */
typedef struct
{
    int motor_id;                       ///< Motor identifier (1-based).
    bool engaged;                       ///< True if in motor mode.
    motion_profile_point_t *trajectory; ///< Active trajectory setpoints, or NULL if none.
    int trajectory_points;              ///< Total number of points in the trajectory.
    int trajectory_index;               ///< Current trajectory index.
    bool trajectory_active;             ///< True if a trajectory is being executed.
    float current_position;             ///< Last known sensor reading.
    // --- New Batch Fields ---
    motor_command_t *batch_commands; ///< Array of commands loaded from a batch.
    int batch_count;                 ///< Total number of commands in the batch.
    int batch_index;                 ///< Next command index to execute.
    bool batch_error;                ///< True if a command failed in this batch.
    char batch_error_message[64];    ///< Error message if a failure occurred.
} motor_state_t;

/**
 *  @brief Setter for batch in progress
 */
void motor_control_set_batch_in_progress(bool in_progress);

/**
 *  @brief Getter for batch in progress
 */
bool motor_control_is_batch_in_progress(void);

/**
 * @brief Initializes the motor control module and motor states.
 */
void motor_control_init(void);

/**
 * @brief Retrieves the state structure for a given motor.
 *
 * @param motor_id 1‑based motor ID
 * @return Pointer to motor_state_t or NULL if invalid ID
 */
motor_state_t *motor_control_get_state(int motor_id);

/**
 * @brief Task that sends trajectory setpoints to each motor until the trajectory is done.
 *        Also includes watchdog logic for CAN timeouts.
 * @param arg Unused
 */
void motor_control_task(void *arg);

/**
 * @brief Syncs the current motor position from the motor sensor.
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

/**
 * @brief A blocking helper function that moves the motor to the specified radian position
 *        and waits until the trajectory completes or a timeout occurs.
 *
 * @param motor_id Motor ID (1-based)
 * @param target_position_rad Target angle in radians
 * @param timeout_ticks How long to wait before giving up
 * @return ESP_OK if the move completes, ESP_ERR_TIMEOUT if it times out, or an error if something else fails
 */
esp_err_t motor_control_move_blocking(int motor_id, float target_position_rad, int timeout_ticks);

/* Gain definitions */
#define KP1 10
#define KD1 1
#define KP2 0
#define KD2 0
#define KP3 0
#define KD3 0

/* Motion limits for each motor (in degrees) */
#define MOTOR1_MIN_ANGLE_DEG 0.0f
#define MOTOR1_MAX_ANGLE_DEG 180.0f
#define MOTOR2_MIN_ANGLE_DEG -180.0f
#define MOTOR2_MAX_ANGLE_DEG 180.0f
#define MOTOR3_MIN_ANGLE_DEG -135.0f
#define MOTOR3_MAX_ANGLE_DEG 135.0f

/**
 * @brief Maximum speed (degrees/s) for each motor at 100% speed setting.
 * You may customize these values.
 */
#define MOTOR1_MAX_SPEED_DPS 360.0f
#define MOTOR2_MAX_SPEED_DPS 360.0f
#define MOTOR3_MAX_SPEED_DPS 90.0f

#endif // MOTOR_CONTROL_H
