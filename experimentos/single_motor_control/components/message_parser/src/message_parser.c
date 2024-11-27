#include "message_parser.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"

/**
 * @brief Maps a float x in [x_min,x_max] to an unsigned int in [0,2^bits - 1].
 *
 * @param x      The float value to be converted.
 * @param x_min  Minimum value of x.
 * @param x_max  Maximum value of x.
 * @param bits   Number of bits of the unsigned int.
 * @return uint16_t The mapped unsigned integer.
 */
uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
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
float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    uint32_t max_int = (1 << bits) - 1;
    float result = ((float)x_int) * span / max_int + x_min;
    return result;
}

void pack_cmd(float p_des, float v_des, float kp, float kd, float t_ff, uint8_t *msg_data)
{
    // Limit data to be within bounds
    if (p_des < P_MIN)
        p_des = P_MIN;
    if (p_des > P_MAX)
        p_des = P_MAX;
    if (v_des < V_MIN)
        v_des = V_MIN;
    if (v_des > V_MAX)
        v_des = V_MAX;
    if (kp < KP_MIN)
        kp = KP_MIN;
    if (kp > KP_MAX)
        kp = KP_MAX;
    if (kd < KD_MIN)
        kd = KD_MIN;
    if (kd > KD_MAX)
        kd = KD_MAX;
    if (t_ff < T_MIN)
        t_ff = T_MIN;
    if (t_ff > T_MAX)
        t_ff = T_MAX;

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

void unpack_reply(uint8_t *msg_data, motor_reply_t *reply)
{
    // Extract Motor ID if needed
    reply->motor_id = msg_data[0];

    // Extract position (16 bits from bytes 1 and 2)
    uint16_t p_int = ((uint16_t)msg_data[1] << 8) | msg_data[2];
    
    // Extract velocity (12 bits: upper 8 bits from byte 3 and upper 4 bits from byte 4)
    uint16_t v_int = ((uint16_t)msg_data[3] << 4) | (msg_data[4] >> 4);
    
    // Extract current (12 bits: lower 4 bits from byte 4 and all 8 bits from byte 5)
    uint16_t i_int = (((uint16_t)(msg_data[4] & 0x0F)) << 8) | msg_data[5];
    
    // Convert raw values to scaled floats
    reply->position = uint_to_float(p_int, P_MIN, P_MAX, 16);
    reply->velocity = uint_to_float(v_int, V_MIN, V_MAX, 12);
    reply->current = uint_to_float(i_int, I_MIN, I_MAX, 12);
}

void parse_command(const char *input_string, motor_command_t *command_struct)
{
    char buffer[BUF_SIZE];
    strncpy(buffer, input_string, BUF_SIZE - 1);
    buffer[BUF_SIZE - 1] = '\0'; // Ensure null-termination

    char *token = strtok(buffer, "_");
    while (token != NULL)
    {
        if (strncmp(token, "P", 1) == 0)
        {
            command_struct->p_des = strtof(token + 1, NULL);
        }
        else if (strncmp(token, "V", 1) == 0)
        {
            command_struct->v_des = strtof(token + 1, NULL);
        }
        else if (strncmp(token, "KP", 2) == 0)
        {
            command_struct->kp = strtof(token + 2, NULL);
        }
        else if (strncmp(token, "KD", 2) == 0)
        {
            command_struct->kd = strtof(token + 2, NULL);
        }
        else if (strncmp(token, "TFF", 3) == 0)
        {
            command_struct->t_ff = strtof(token + 3, NULL);
        }
        else
        {
            ESP_LOGW(LOG_TAG, "Unknown parameter: %s", token);
        }
        token = strtok(NULL, "_");
    }
}
