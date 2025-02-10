#ifndef MESSAGE_PARSER_H
#define MESSAGE_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include "cJSON.h" // Include cJSON library

/* Define ranges for position, velocity, etc. */
#define P_MIN -95.5f  // Minimum position, radians
#define P_MAX 95.5f   // Maximum position, radians
#define V_MIN -45.0f  // Min velocity, rad/s
#define V_MAX 45.0f   // Max velocity, rad/s
#define KP_MIN 0.0f   // Min proportional gain, N-m/rad
#define KP_MAX 500.0f // Max proportional gain, N-m/rad
#define KD_MIN 0.0f   // Min derivatiive gain, N-m*s/rad
#define KD_MAX 5.0f   // Max derivative gain, N-m*s/rad
#define I_MIN -18.0f  // Min current, A
#define I_MAX 18.0f   // Max current, A
#define T_MIN -18.0f  // Minimum torque, N-m
#define T_MAX 18.0f   // Maximum torque, N-m

#define CAN_CMD_LENGTH 8   /**< Command message length in bytes */
#define CAN_REPLY_LENGTH 5 /**< Reply message length in bytes */

#define BUF_SIZE 256

/**
 * @brief Structure to hold the unpacked reply data from the motor.
 */
typedef struct
{
    uint8_t motor_id;
    float position; /**< Position in radians */
    float velocity; /**< Velocity in rad/s */
    float current;  /**< Torque in N-m */
} motor_reply_t;

/**
 * @brief Structure to hold motor command parameters.
 */
typedef struct
{
    int motor_id;
    float position;
    float velocity;
    float kp;
    float kd;
    float feed_forward_torque;
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
 * @param position     Desired position in radians.
 * @param velocity     Desired velocity in rad/s.
 * @param kp           Proportional gain.
 * @param kd           Derivative gain.
 * @param feed_forward_torque Feed-forward torque in N-m.
 * @param msg_data     Pointer to an 8-byte array to store the packed data.
 */
void pack_cmd(float position, float velocity, float kp, float kd, float feed_forward_torque, uint8_t *msg_data);

/**
 * @brief Unpacks the reply message data from the motor.
 *
 * @param msg_data  Pointer to a 5-byte array containing the reply message.
 * @param reply     Pointer to a ReplyData struct to store the unpacked data.
 */
void unpack_reply(uint8_t *msg_data, motor_reply_t *reply);

/**
 * @brief Parses a JSON command string and fills the motor_command_t structure.
 *
 * @param[in]  json_string      The JSON command string to parse.
 * @param[out] command_struct   Pointer to the structure where parsed values
 *                              will be stored.
 * @param[out] special_command  Buffer to store any special command found.
 * @return     true if parsing was successful, false otherwise.
 */
bool parse_json_command(const char *json_string, motor_command_t *command_struct, char *special_command);

#endif // MESSAGE_PARSER_H
