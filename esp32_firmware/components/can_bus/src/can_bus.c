#include "can_bus.h"
#include "motor_control.h"
#include "message_parser.h"
#include "esp_log.h"
#include <string.h>
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LOG_TAG "CAN_BUS"

// TWAI (CAN) configuration constants
#define TWAI_TX_GPIO_NUM (21)
#define TWAI_RX_GPIO_NUM (22)
#define TWAI_MODE TWAI_MODE_NORMAL
#define TWAI_BIT_RATE TWAI_TIMING_CONFIG_1MBITS()
#define TWAI_FILTER_CONFIG TWAI_FILTER_CONFIG_ACCEPT_ALL()

// Predefined command messages
static const uint8_t ENTER_MOTOR_MODE_CMD[DATA_LENGTH] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
static const uint8_t EXIT_MOTOR_MODE_CMD[DATA_LENGTH] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
static const uint8_t ZERO_POS_SENSOR_CMD[DATA_LENGTH] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

esp_err_t can_bus_init(void)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO_NUM, TWAI_RX_GPIO_NUM, TWAI_MODE);
    twai_timing_config_t t_config = TWAI_BIT_RATE;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG;

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to install TWAI driver: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(LOG_TAG, "TWAI driver installed");

    err = twai_start();
    if (err != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to start TWAI driver: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(LOG_TAG, "TWAI driver started");

    return ESP_OK;
}

esp_err_t can_bus_transmit(uint8_t *msg_data, size_t length, int motor_id)
{
    twai_message_t message = {0};
    message.identifier = motor_id;
    message.data_length_code = length;
    memcpy(message.data, msg_data, length);

    ESP_LOGI(LOG_TAG, "Sending CAN Message to Motor ID %d", motor_id);
    ESP_LOG_BUFFER_HEX(LOG_TAG, message.data, message.data_length_code);

    esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(1000));
    if (err != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "CAN transmit failed: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(LOG_TAG, "CAN transmit successful");
    }
    return err;
}

esp_err_t can_bus_receive(twai_message_t *message)
{
    esp_err_t err = twai_receive(message, pdMS_TO_TICKS(1000));
    if (err == ESP_OK)
    {
        ESP_LOGI(LOG_TAG, "Received CAN Message: ID: 0x%lx, DLC: %d", message->identifier, message->data_length_code);
        ESP_LOG_BUFFER_HEX(LOG_TAG, message->data, message->data_length_code);
    }
    else if (err != ESP_ERR_TIMEOUT)
    {
        ESP_LOGE(LOG_TAG, "CAN receive failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t can_bus_send_enter_mode(int motor_id)
{
    ESP_LOGI(LOG_TAG, "Sending Enter Motor Mode command to Motor ID %d", motor_id);
    return can_bus_transmit((uint8_t *)ENTER_MOTOR_MODE_CMD, DATA_LENGTH, motor_id);
}

esp_err_t can_bus_send_exit_mode(int motor_id)
{
    ESP_LOGI(LOG_TAG, "Sending Exit Motor Mode command to Motor ID %d", motor_id);
    return can_bus_transmit((uint8_t *)EXIT_MOTOR_MODE_CMD, DATA_LENGTH, motor_id);
}

esp_err_t can_bus_send_zero_pos_sensor(int motor_id)
{
    ESP_LOGI(LOG_TAG, "Sending Zero Position Sensor command to Motor ID %d", motor_id);
    return can_bus_transmit((uint8_t *)ZERO_POS_SENSOR_CMD, DATA_LENGTH, motor_id);
}

void can_bus_receive_task(void *arg)
{
    while (1)
    {
        twai_message_t received_msg;
        esp_err_t err = can_bus_receive(&received_msg);
        if (err == ESP_OK)
        {
            if (received_msg.data_length_code >= 5)
            {
                motor_reply_t reply = {0};
                unpack_reply(received_msg.data, &reply);

                // Update the corresponding motor's current position using per-motor state.
                // motor_control_get_state() is defined in the motor_control module.
                motor_state_t *state = motor_control_get_state(reply.motor_id);
                if (state)
                {
                    state->current_position = reply.position;
                }

                ESP_LOGI(LOG_TAG, "Received CAN Message:");
                ESP_LOGI(LOG_TAG, "  Motor ID: %d", reply.motor_id);
                ESP_LOGI(LOG_TAG, "  Position: %.4f radians", reply.position);
                ESP_LOGI(LOG_TAG, "  Velocity: %.4f rad/s", reply.velocity);
                ESP_LOGI(LOG_TAG, "  Current: %.2f A", reply.current);
            }
            else
            {
                ESP_LOGE(LOG_TAG, "Received CAN message does not contain enough data to unpack");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}
