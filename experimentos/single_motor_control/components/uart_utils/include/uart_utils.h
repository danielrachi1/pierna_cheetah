#ifndef UART_UTILS_H
#define UART_UTILS_H

#include <stdint.h>

/**
 * @brief Structure to hold motor command parameters.
 */
typedef struct {
    float p_des;  /**< Desired position in radians */
    float v_des;  /**< Desired velocity in rad/s */
    float kp;     /**< Proportional gain */
    float kd;     /**< Derivative gain */
    float t_ff;   /**< Feed-forward torque in N-m */
} motor_command_t;

/**
 * @brief Parses a command string and fills the motor_command_t structure.
 *
 * This function takes an input string containing motor control parameters
 * formatted as "P<p_des>_V<v_des>_KP<kp>_KD<kd>_TFF<t_ff>" and parses it to
 * extract the floating-point values for each parameter. The extracted values
 * are stored in the provided motor_command_t structure.
 *
 * @param[in]  input_string    The command string to parse.
 * @param[out] command_struct  Pointer to the structure where parsed values
 *                             will be stored.
 */
void parse_command(const char *input_string, motor_command_t *command_struct);

#endif // UART_UTILS_H
