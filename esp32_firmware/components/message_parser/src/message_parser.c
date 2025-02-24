#include "message_parser.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "cJSON.h"

#define LOG_TAG "MESSAGE_PARSER"

uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x - x_min;
    uint32_t max_int = (1 << bits) - 1;
    uint16_t result = (uint16_t)((offset * max_int) / span);
    return result;
}

float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    uint32_t max_int = (1 << bits) - 1;
    float result = ((float)x_int) * span / max_int + x_min;
    return result;
}

void pack_cmd(float position, float velocity, float kp, float kd, float feed_forward_torque, uint8_t *msg_data)
{
    // Clamp to valid ranges
    if (position < P_MIN)
        position = P_MIN;
    if (position > P_MAX)
        position = P_MAX;
    if (velocity < V_MIN)
        velocity = V_MIN;
    if (velocity > V_MAX)
        velocity = V_MAX;
    if (kp < KP_MIN)
        kp = KP_MIN;
    if (kp > KP_MAX)
        kp = KP_MAX;
    if (kd < KD_MIN)
        kd = KD_MIN;
    if (kd > KD_MAX)
        kd = KD_MAX;
    if (feed_forward_torque < T_MIN)
        feed_forward_torque = T_MIN;
    if (feed_forward_torque > T_MAX)
        feed_forward_torque = T_MAX;

    uint16_t p_int = float_to_uint(position, P_MIN, P_MAX, 16);
    uint16_t v_int = float_to_uint(velocity, V_MIN, V_MAX, 12);
    uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint16_t t_int = float_to_uint(feed_forward_torque, T_MIN, T_MAX, 12);

    msg_data[0] = (p_int >> 8) & 0xFF;
    msg_data[1] = p_int & 0xFF;
    msg_data[2] = (v_int >> 4) & 0xFF;
    msg_data[3] = ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF);
    msg_data[4] = kp_int & 0xFF;
    msg_data[5] = (kd_int >> 4) & 0xFF;
    msg_data[6] = ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF);
    msg_data[7] = t_int & 0xFF;
}

void unpack_reply(uint8_t *msg_data, motor_reply_t *reply)
{
    reply->motor_id = msg_data[0];

    uint16_t p_int = ((uint16_t)msg_data[1] << 8) | msg_data[2];
    uint16_t v_int = ((uint16_t)msg_data[3] << 4) | (msg_data[4] >> 4);
    uint16_t i_int = (((uint16_t)(msg_data[4] & 0x0F)) << 8) | msg_data[5];

    reply->position = uint_to_float(p_int, P_MIN, P_MAX, 16);
    reply->velocity = uint_to_float(v_int, V_MIN, V_MAX, 12);
    reply->current = uint_to_float(i_int, I_MIN, I_MAX, 12);
}
