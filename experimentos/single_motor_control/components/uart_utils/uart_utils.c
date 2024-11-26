#include "uart_utils.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"

#define TAG "UART_UTILS"
#define BUF_SIZE 256

void parse_command(const char *input_string, motor_command_t *command_struct)
{
    char buffer[BUF_SIZE];
    strncpy(buffer, input_string, BUF_SIZE - 1);
    buffer[BUF_SIZE - 1] = '\0';  // Ensure null-termination

    char *token = strtok(buffer, "_");
    while (token != NULL) {
        if (strncmp(token, "P", 1) == 0) {
            command_struct->p_des = strtof(token + 1, NULL);
        } else if (strncmp(token, "V", 1) == 0) {
            command_struct->v_des = strtof(token + 1, NULL);
        } else if (strncmp(token, "KP", 2) == 0) {
            command_struct->kp = strtof(token + 2, NULL);
        } else if (strncmp(token, "KD", 2) == 0) {
            command_struct->kd = strtof(token + 2, NULL);
        } else if (strncmp(token, "TFF", 3) == 0) {
            command_struct->t_ff = strtof(token + 3, NULL);
        } else {
            ESP_LOGW(TAG, "Unknown parameter: %s", token);
        }
        token = strtok(NULL, "_");
    }
}
