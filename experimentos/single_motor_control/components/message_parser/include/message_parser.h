#ifndef MESSAGE_PARSER_H
#define MESSAGE_PARSER_H

#include <stdint.h>

/* Define ranges for position, velocity, etc. */
#define P_MIN -12.5f  /**< Minimum position, radians */
#define P_MAX 12.5f   /**< Maximum position, radians */
#define V_MIN -65.0f  /**< Minimum velocity, rad/s */
#define V_MAX 65.0f   /**< Maximum velocity, rad/s */
#define KP_MIN 0.0f   /**< Minimum proportional gain, N-m/rad */
#define KP_MAX 500.0f /**< Maximum proportional gain, N-m/rad */
#define KD_MIN 0.0f   /**< Minimum derivative gain, N-m*s/rad */
#define KD_MAX 5.0f   /**< Maximum derivative gain, N-m*s/rad */
#define T_MIN -18.0f  /**< Minimum torque, N-m */
#define T_MAX 18.0f   /**< Maximum torque, N-m */

#define CAN_CMD_LENGTH 8   /**< Command message length in bytes */
#define CAN_REPLY_LENGTH 5 /**< Reply message length in bytes */

#define LOG_TAG "MESSAGE_PARSER"
#define BUF_SIZE 256

/**
 * @brief Structure to hold the unpacked reply data from the motor.
 */
typedef struct
{
    float position; /**< Position in radians */
    float velocity; /**< Velocity in rad/s */
    float torque;   /**< Torque in N-m */
} motor_reply_t;

/**
 * @brief Structure to hold motor command parameters.
 */
typedef struct
{
    float p_des; /**< Desired position in radians */
    float v_des; /**< Desired velocity in rad/s */
    float kp;    /**< Proportional gain */
    float kd;    /**< Derivative gain */
    float t_ff;  /**< Feed-forward torque in N-m */
} motor_command_t;

/**
 * @brief Maps a float x in [x_min,x_max] to an unsigned int in [0,2^bits - 1].
 *
 * @param x      The float value to be converted.
 * @param x_min  Minimum value of x.
 * @param x_max  Maximum value of x.
 * @param bits   Number of bits of the unsigned int.
 * @return uint16_t The mapped unsigned integer.
 */
uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);

/**
 * @brief Maps an unsigned int x_int in [0,2^bits -1] to a float in [x_min,x_max].
 *
 * @param x_int  The unsigned int value to be converted.
 * @param x_min  Minimum value of x.
 * @param x_max  Maximum value of x.
 * @param bits   Number of bits of the unsigned int.
 * @return float The mapped float value.
 */
float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits);

/**
 * @brief Packs the command data into an 8-byte array for CAN transmission.
 *
 * CAN Command Packet Structure:
 * - 16-bit position command, range: [-12.5, 12.5] radians
 * - 12-bit velocity command, range: [-65.0, 65.0] rad/s
 * - 12-bit kp (proportional gain), range: [0.0, 500.0] N-m/rad
 * - 12-bit kd (derivative gain), range: [0.0, 5.0] N-m*s/rad
 * - 12-bit feed-forward torque, range: [-18.0, 18.0] N-m
 *
 * The CAN packet consists of 8 bytes (8 x 8 bits), formatted as follows:
 * - For each quantity, bit 0 is the LSB (Least Significant Bit).
 *
 * Byte Index and Bit Allocation:
 * - Byte 0: [position[15:8]]
 * - Byte 1: [position[7:0]]
 * - Byte 2: [velocity[11:4]]
 * - Byte 3: [velocity[3:0], kp[11:8]]
 * - Byte 4: [kp[7:0]]
 * - Byte 5: [kd[11:4]]
 * - Byte 6: [kd[3:0], torque[11:8]]
 * - Byte 7: [torque[7:0]]
 *
 * @param p_des     Desired position in radians.
 * @param v_des     Desired velocity in rad/s.
 * @param kp        Proportional gain.
 * @param kd        Derivative gain.
 * @param t_ff      Feed-forward torque in N-m.
 * @param msg_data  Pointer to an 8-byte array to store the packed data.
 */
void pack_cmd(float p_des, float v_des, float kp, float kd, float t_ff, uint8_t *msg_data);

/**
 * @brief Simplified function to pack command with only position (other parameters default).
 *
 * @param p_des     Desired position in radians.
 * @param msg_data  Pointer to an 8-byte array to store the packed data.
 */
void pack_cmd_position(float p_des, uint8_t *msg_data);

/**
 * @brief Unpacks the reply message data from the motor.
 *
 * @param msg_data  Pointer to a 5-byte array containing the reply message.
 * @param reply     Pointer to a ReplyData struct to store the unpacked data.
 */
void unpack_reply(uint8_t *msg_data, motor_reply_t *reply);

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

#endif // MESSAGE_PARSER_H
