#ifndef MOTOR_COMMAND_HANDLERS_H
#define MOTOR_COMMAND_HANDLERS_H

#include "motor_control_core.h"
#include "esp_err.h"

/**
 * @brief Dispatches a motor command (move, enter mode, exit mode, zero sensor).
 *        Internally calls the appropriate static helper based on cmd_type.
 *
 * @param command The command to be processed.
 * @return ESP_OK on success or an error code on failure.
 */
esp_err_t motor_control_handle_command(const motor_command_t *command);

#endif // MOTOR_COMMAND_HANDLERS_H
