#include "can_utils.h"

/**
 * @brief Maps a float x in [x_min,x_max] to an unsigned int in [0,2^bits - 1].
 *
 * @param x      The float value to be converted.
 * @param x_min  Minimum value of x.
 * @param x_max  Maximum value of x.
 * @param bits   Number of bits of the unsigned int.
 * @return uint16_t The mapped unsigned integer.
 */
uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    float offset = x - x_min;
    uint32_t max_int = (1 << bits) - 1;
    uint16_t result = (uint16_t)((offset * max_int) / span);
    return result;
}

/**
 * @brief Maps an unsigned int x_int in [0,2^bits -1] to a float in [x_min,x_max].
 *
 * @param x_int  The unsigned int value to be converted.
 * @param x_min  Minimum value of x.
 * @param x_max  Maximum value of x.
 * @param bits   Number of bits of the unsigned int.
 * @return float The mapped float value.
 */
float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    uint32_t max_int = (1 << bits) - 1;
    float result = ((float)x_int) * span / max_int + x_min;
    return result;
}

void pack_cmd(float p_des, float v_des, float kp, float kd, float t_ff, uint8_t *msg_data) {
    // Limit data to be within bounds
    if (p_des < P_MIN) p_des = P_MIN;
    if (p_des > P_MAX) p_des = P_MAX;
    if (v_des < V_MIN) v_des = V_MIN;
    if (v_des > V_MAX) v_des = V_MAX;
    if (kp < KP_MIN) kp = KP_MIN;
    if (kp > KP_MAX) kp = KP_MAX;
    if (kd < KD_MIN) kd = KD_MIN;
    if (kd > KD_MAX) kd = KD_MAX;
    if (t_ff < T_MIN) t_ff = T_MIN;
    if (t_ff > T_MAX) t_ff = T_MAX;

    // Convert floats to unsigned ints
    uint16_t p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    uint16_t v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint16_t t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    // Pack data into bytes
    msg_data[0] = (p_int >> 8) & 0xFF;
    msg_data[1] = p_int & 0xFF;
    msg_data[2] = (v_int >> 4) & 0xFF;
    msg_data[3] = ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF);
    msg_data[4] = kp_int & 0xFF;
    msg_data[5] = (kd_int >> 4) & 0xFF;
    msg_data[6] = ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF);
    msg_data[7] = t_int & 0xFF;
}

void pack_cmd_position(float p_des, uint8_t *msg_data) {
    // Default values for other parameters
    float v_des = 0.0f;
    float kp = KP_MIN;
    float kd = KD_MIN;
    float t_ff = 0.0f;
    pack_cmd(p_des, v_des, kp, kd, t_ff, msg_data);
}

void unpack_reply(uint8_t *msg_data, ReplyData *reply) {
    // Extract data from bytes
    uint16_t p_int = (msg_data[0] << 8) | msg_data[1];
    uint16_t v_int = (msg_data[2] << 4) | (msg_data[3] >> 4);
    uint16_t t_int = ((msg_data[3] & 0xF) << 8) | msg_data[4];

    // Convert unsigned ints to floats
    reply->position = uint_to_float(p_int, P_MIN, P_MAX, 16);
    reply->velocity = uint_to_float(v_int, V_MIN, V_MAX, 12);
    reply->torque   = uint_to_float(t_int, T_MIN, T_MAX, 12);
}
