#include "message_parser.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "cJSON.h" // Include cJSON library

#define LOG_TAG "MESSAGE_PARSER" // If I define it in message_parser.h a warning is raised

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

/**#define LOG_TAG "MOTOR_CONTROLLER"
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

void pack_cmd(float position, float velocity, float kp, float kd, float feed_forward_torque, uint8_t *msg_data)
{
    // Limit data to be within bounds
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

    // Convert floats to unsigned ints
    uint16_t p_int = float_to_uint(position, P_MIN, P_MAX, 16);
    uint16_t v_int = float_to_uint(velocity, V_MIN, V_MAX, 12);
    uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint16_t t_int = float_to_uint(feed_forward_torque, T_MIN, T_MAX, 12);

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

bool parse_json_command(const char *json_string, motor_command_t *command_struct, char *special_command)
{
    cJSON *json = cJSON_Parse(json_string);
    if (json == NULL)
    {
        ESP_LOGE(LOG_TAG, "Failed to parse JSON");
        special_command[0] = '\0'; // Ensure special_command is empty
        return false;
    }

    cJSON *command_item = cJSON_GetObjectItem(json, "command");
    if (cJSON_IsString(command_item) && (command_item->valuestring != NULL))
    {
        strncpy(special_command, command_item->valuestring, BUF_SIZE - 1);
        special_command[BUF_SIZE - 1] = '\0';
    }
    else
    {
        special_command[0] = '\0'; // No special command
    }

    cJSON *p_des_item = cJSON_GetObjectItem(json, "position");
    if (cJSON_IsNumber(p_des_item))
    {
        command_struct->position = p_des_item->valuedouble;
    }

    cJSON *v_des_item = cJSON_GetObjectItem(json, "velocity");
    if (cJSON_IsNumber(v_des_item))
    {
        command_struct->velocity = v_des_item->valuedouble;
    }

    cJSON *kp_item = cJSON_GetObjectItem(json, "kp");
    if (cJSON_IsNumber(kp_item))
    {
        command_struct->kp = kp_item->valuedouble;
    }

    cJSON *kd_item = cJSON_GetObjectItem(json, "kd");
    if (cJSON_IsNumber(kd_item))
    {
        command_struct->kd = kd_item->valuedouble;
    }

    cJSON *t_ff_item = cJSON_GetObjectItem(json, "feed_forward_torque");
    if (cJSON_IsNumber(t_ff_item))
    {
        command_struct->feed_forward_torque = t_ff_item->valuedouble;
    }

    cJSON_Delete(json);
    return true;
}
