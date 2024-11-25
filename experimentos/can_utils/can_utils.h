// can_utils.h

#ifndef CAN_UTILS_H
#define CAN_UTILS_H

#include <stdint.h>

// Define ranges for position, velocity, etc.
#define P_MIN -12.5f     // Minimum position, adjust as per your system
#define P_MAX 12.5f      // Maximum position, adjust as per your system
#define V_MIN -65.0f     // Minimum velocity
#define V_MAX 65.0f      // Maximum velocity
#define KP_MIN 0.0f      // Minimum proportional gain
#define KP_MAX 500.0f    // Maximum proportional gain
#define KD_MIN 0.0f      // Minimum derivative gain
#define KD_MAX 5.0f      // Maximum derivative gain
#define T_MIN -18.0f     // Minimum torque
#define T_MAX 18.0f      // Maximum torque

#define CAN_CMD_LENGTH 8      // Command message length in bytes
#define CAN_REPLY_LENGTH 5    // Reply message length in bytes

// Structure to hold the unpacked reply data
typedef struct {
    float position;  // Position in radians
    float velocity;  // Velocity in rad/s
    float torque;    // Torque in N-m
} ReplyData;

// Function to map a float to unsigned int
uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);

// Function to map unsigned int to float
float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits);

// Function to pack command with all parameters
void pack_cmd(float p_des, float v_des, float kp, float kd, float t_ff, uint8_t *msg_data);

// Simplified function to pack command with only position (other parameters default)
void pack_cmd_position(float p_des, uint8_t *msg_data);

// Function to unpack reply message
void unpack_reply(uint8_t *msg_data, ReplyData *reply);

#endif // CAN_UTILS_H
