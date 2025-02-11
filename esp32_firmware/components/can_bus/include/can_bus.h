#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <stdint.h>
#include "driver/twai.h"
#include "esp_err.h"

#define DATA_LENGTH 8

/**
 * @brief Initializes the TWAI (CAN) driver.
 *
 * @return esp_err_t ESP_OK on success or an error code.
 */
esp_err_t can_bus_init(void);

/**
 * @brief Flushes any pending messages from the CAN bus.
 *
 * This function runs a short receive loop to clear any messages.
 *
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t can_bus_flush(void);

/**
 * @brief Sends a CAN message and waits for its response.
 *
 * This function flushes the CAN bus, sends the message, and then blocks until
 * a response is received (or a timeout occurs).
 *
 * @param msg_data Pointer to the message data.
 * @param length Length of the message data.
 * @param motor_id The motor identifier (used as the CAN message identifier).
 * @param response Pointer to a twai_message_t structure to store the received response.
 * @return esp_err_t ESP_OK on success or an error code.
 */
esp_err_t can_bus_request_response(uint8_t *msg_data, size_t length, int motor_id, twai_message_t *response);

/**
 * @brief Sends the Enter Motor Mode command and waits for a response.
 *
 * @param motor_id The motor identifier.
 * @param response Pointer to a twai_message_t structure to store the received response.
 * @return esp_err_t ESP_OK on success or an error code.
 */
esp_err_t can_bus_send_enter_mode(int motor_id, twai_message_t *response);

/**
 * @brief Sends the Exit Motor Mode command and waits for a response.
 *
 * @param motor_id The motor identifier.
 * @param response Pointer to a twai_message_t structure to store the received response.
 * @return esp_err_t ESP_OK on success or an error code.
 */
esp_err_t can_bus_send_exit_mode(int motor_id, twai_message_t *response);

/**
 * @brief Sends the Zero Position Sensor command and waits for a response.
 *
 * @param motor_id The motor identifier.
 * @param response Pointer to a twai_message_t structure to store the received response.
 * @return esp_err_t ESP_OK on success or an error code.
 */
esp_err_t can_bus_send_zero_pos_sensor(int motor_id, twai_message_t *response);

#endif // CAN_BUS_H
