#ifndef MOTOR_CONTROL_CORE_H
#define MOTOR_CONTROL_CORE_H

#include <stdbool.h>
#include "esp_err.h"
#include "motion_profile.h"

/**
 * @brief Number of motors the system supports.
 */
#define NUM_MOTORS 3

/**
 * @brief Gains for each motor
 */
#define KP1 12
#define KD1 1
#define KP2 8
#define KD2 1
#define KP3 25
#define KD3 2.1

/**
 * Motion limits for each motor (in degrees)
 */
#define MOTOR1_MIN_ANGLE_DEG 0.0f
#define MOTOR1_MAX_ANGLE_DEG 180.0f
#define MOTOR2_MIN_ANGLE_DEG -180.0f
#define MOTOR2_MAX_ANGLE_DEG 0.0f
#define MOTOR3_MIN_ANGLE_DEG -135.0f
#define MOTOR3_MAX_ANGLE_DEG 135.0f

/**
 * @brief Maximum speed (degrees/s) for each motor.
 */
#define MOTOR1_MAX_SPEED_DPS 720.0f
#define MOTOR2_MAX_SPEED_DPS 720.0f
#define MOTOR3_MAX_SPEED_DPS 720.0f

/**
 * @brief Some motors are inverted in hardware.
 */
extern bool s_inverted[NUM_MOTORS];

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
 * @brief Holds one motor command.
 *
 * For MOTOR_CMD_MOVE, 'position' and 'speed' are in radians and rad/s, respectively.
 */
typedef struct
{
    int motor_id;                  ///< Motor identifier (1-based).
    motor_command_type_t cmd_type; ///< Type of command.
    float position;                ///< Target position (radians).
    float speed;                   ///< Target speed (rad/s).
} motor_command_t;

/**
 * @brief Single motor's state structure.
 */
typedef struct
{
    int motor_id;                       ///< Motor identifier (1-based).
    bool engaged;                       ///< True if in motor mode.
    motion_profile_point_t *trajectory; ///< Active trajectory setpoints, or NULL if none.
    int trajectory_points;              ///< Total number of points in the trajectory.
    int trajectory_index;               ///< Current trajectory index.
    bool trajectory_active;             ///< True if a trajectory is being executed.
    float current_position;             ///< Last known user-space angle (radians).

    // --- Batch command fields ---
    motor_command_t *batch_commands; ///< Dynamically allocated array of commands from a batch.
    int batch_count;                 ///< Total number of commands in the batch.
    int batch_index;                 ///< Which command index in the batch we are on.
    bool batch_error;                ///< True if a command failed in this batch.
    char batch_error_message[64];    ///< Reason for error if batch_error is true.
} motor_state_t;

/**
 * @brief Initializes motor states, preparing them for use.
 *        This must be called before using any other motor_control_* function.
 */
void motor_control_init(void);

/**
 * @brief
 */
float from_user_angle(int motor_id, float user_angle);

/**
 * @brief
 */
float to_user_angle(int motor_id, float hardware_angle);

/**
 * @brief Retrieves the state structure for a given motor.
 *
 * @param motor_id Motor ID, 1-based.
 * @return Pointer to the motor_state_t, or NULL if invalid ID.
 */
motor_state_t *motor_control_get_state(int motor_id);

/**
 * @brief Sets a global flag to indicate a batch is in progress (used internally).
 */
void motor_control_set_batch_in_progress(bool in_progress);

/**
 * @brief Returns the global flag indicating a batch is in progress.
 */
bool motor_control_is_batch_in_progress(void);

/**
 * @brief A blocking function that moves the motor to the specified radian position
 *        and waits until the trajectory completes or a timeout occurs.
 *
 * @param motor_id Motor ID (1-based).
 * @param target_position_rad Target angle in radians.
 * @param timeout_ticks How long to wait (FreeRTOS ticks) before giving up.
 * @return ESP_OK if the move completes, ESP_ERR_TIMEOUT if it times out,
 *         or another error on failure.
 */
esp_err_t motor_control_move_blocking(int motor_id, float target_position_rad, int timeout_ticks);

#endif // MOTOR_CONTROL_CORE_H
