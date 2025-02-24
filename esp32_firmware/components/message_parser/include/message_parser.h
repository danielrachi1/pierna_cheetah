#ifndef MESSAGE_PARSER_H
#define MESSAGE_PARSER_H

#include <stdint.h>
#include <stdbool.h>

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

#define CAN_CMD_LENGTH 8
#define CAN_REPLY_LENGTH 5
#define BUF_SIZE 256

/**
 * @brief Holds unpacked reply data from the motor.
 */
typedef struct
{
    uint8_t motor_id;
    float position;
    float velocity;
    float current;
} motor_reply_t;

/**
 * @brief Packs the command data into an 8-byte array for CAN transmission.
 *
 * The parameters are clamped to their valid ranges.
 */
void pack_cmd(float position, float velocity, float kp, float kd, float feed_forward_torque, uint8_t *msg_data);

/**
 * @brief Unpacks the reply message data from the motor.
 */
void unpack_reply(uint8_t *msg_data, motor_reply_t *reply);

/**
 * @brief Mapping utility: float -> uint16_t
 */
uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);

/**
 * @brief Mapping utility: uint16_t -> float
 */
float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits);

#endif // MESSAGE_PARSER_H
