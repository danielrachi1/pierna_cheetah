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
 * @brief Transmits a CAN message.
 *
 * @param msg_data Pointer to the message data.
 * @param length Length of the message data.
 * @param motor_id The motor identifier to be used as the CAN message identifier.
 * @return esp_err_t ESP_OK on success or an error code.
 */
esp_err_t can_bus_transmit(uint8_t *msg_data, size_t length, int motor_id);

/**
 * @brief Receives a CAN message.
 *
 * @param message Pointer to a twai_message_t structure to store the received message.
 * @return esp_err_t ESP_OK on success or an error code.
 */
esp_err_t can_bus_receive(twai_message_t *message);

/**
 * @brief Sends the Enter Motor Mode command.
 *
 * @param motor_id The motor identifier.
 * @return esp_err_t ESP_OK on success or an error code.
 */
esp_err_t can_bus_send_enter_mode(int motor_id);

/**
 * @brief Sends the Exit Motor Mode command.
 *
 * @param motor_id The motor identifier.
 * @return esp_err_t ESP_OK on success or an error code.
 */
esp_err_t can_bus_send_exit_mode(int motor_id);

/**
 * @brief Sends the Zero Position Sensor command.
 *
 * @param motor_id The motor identifier.
 * @return esp_err_t ESP_OK on success or an error code.
 */
esp_err_t can_bus_send_zero_pos_sensor(int motor_id);

#endif // CAN_BUS_H
