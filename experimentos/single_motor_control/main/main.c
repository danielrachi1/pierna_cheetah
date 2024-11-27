#include <stdio.h>
#include <string.h> // Include for string operations
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "message_parser.h"

#define LOG_TAG "MOTOR_CONTROLLER"

#define UART_PORT_NUM (CONFIG_EXAMPLE_UART_PORT_NUM)
#define UART_BAUD_RATE (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define UART_RXD (CONFIG_EXAMPLE_UART_RXD)
#define UART_TXD (CONFIG_EXAMPLE_UART_TXD)
#define UART_RTS (UART_PIN_NO_CHANGE)
#define UART_CTS (UART_PIN_NO_CHANGE)
#define TASK_STACK_SIZE (CONFIG_EXAMPLE_TASK_STACK_SIZE)
#define BUF_SIZE (1024)  // UART buffer size
#define CAN_CMD_LENGTH 8 // CAN message length

#define TWAI_TX_GPIO_NUM (21)
#define TWAI_RX_GPIO_NUM (22)
#define TWAI_MODE TWAI_MODE_NO_ACK
#define TWAI_BIT_RATE TWAI_TIMING_CONFIG_1MBITS()
#define TWAI_FILTER_CONFIG TWAI_FILTER_CONFIG_ACCEPT_ALL()
#define DATA_LENGTH 8

#define CAN_ID 0x01

// Special Command Byte Arrays
const uint8_t ENTER_MOTOR_MODE_CMD[DATA_LENGTH] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
const uint8_t EXIT_MOTOR_MODE_CMD[DATA_LENGTH] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
const uint8_t ZERO_POS_SENSOR_CMD[DATA_LENGTH] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

/**
 * @brief Initializes the TWAI (CAN) driver in loopback mode.
 */
static void twai_init()
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO_NUM, TWAI_RX_GPIO_NUM, TWAI_MODE);
    twai_timing_config_t t_config = TWAI_BIT_RATE;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG;

    // Install TWAI driver
    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to install TWAI driver: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(LOG_TAG, "TWAI driver installed");

    // Start TWAI driver
    err = twai_start();
    if (err != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "Failed to start TWAI driver: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(LOG_TAG, "TWAI driver started");
}

/**
 * @brief Sends a CAN message via TWAI.
 *
 * @param msg_data Pointer to the CAN message data array.
 * @param length   Length of the CAN message data (up to 8 bytes).
 * @return esp_err_t ESP_OK on success, otherwise an error code.
 */
static esp_err_t send_can_message(uint8_t *msg_data, size_t length)
{
    twai_message_t message;
    memset(&message, 0, sizeof(twai_message_t));

    // Set message fields
    message.identifier = CAN_ID;
    message.data_length_code = length;

    memcpy(message.data, msg_data, length);

    // *** Added Logging: Log the entire TWAI message before sending ***
    ESP_LOGI(LOG_TAG, "Preparing to Send CAN Message:");
    ESP_LOGI(LOG_TAG, "  identifier: 0x%lX", (unsigned long)message.identifier);
    ESP_LOGI(LOG_TAG, "  data_length_code: %u", message.data_length_code);

    // Log each byte of the CAN message data
    ESP_LOGI(LOG_TAG, "  Data:");
    for (int i = 0; i < message.data_length_code; i++)
    {
        ESP_LOGI(LOG_TAG, "    Byte %d: 0x%02X", i, message.data[i]);
    }

    // Transmit the message
    esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(1000)); // Timeout 1 second
    if (err != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "TWAI transmit failed: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(LOG_TAG, "TWAI transmit successful");
    }
    return err;
}

/**
 * @brief Receives a CAN message via TWAI (self-reception in loopback mode).
 *
 * @param message Pointer to the twai_message_t structure to store the received message.
 * @return esp_err_t ESP_OK on success, otherwise an error code.
 */
static esp_err_t receive_can_message(twai_message_t *message)
{
    // Attempt to receive a CAN message with a 1-second timeout
    esp_err_t err = twai_receive(message, pdMS_TO_TICKS(1000)); // Timeout 1 second

    if (err == ESP_OK)
    {
        ESP_LOGI(LOG_TAG, "Received CAN Message:");

        // Log each field with appropriate format specifiers and casting
        ESP_LOGI(LOG_TAG, "  identifier: 0x%lX", (unsigned long)message->identifier);
        ESP_LOGI(LOG_TAG, "  data_length_code: %u", message->data_length_code);

        // Log each byte of the CAN message data
        ESP_LOGI(LOG_TAG, "  Data:");
        for (int i = 0; i < message->data_length_code; i++)
        {
            ESP_LOGI(LOG_TAG, "    Byte %d: 0x%02X", i, message->data[i]);
        }
    }
    else if (err != ESP_ERR_TIMEOUT)
    {
        // Log the error if reception failed, excluding timeout which is normal
        ESP_LOGE(LOG_TAG, "TWAI receive failed: %s", esp_err_to_name(err));
    }

    return err;
}

/**
 * @brief Sends the "Enter Motor Mode" special command.
 *
 * @return esp_err_t ESP_OK on success, otherwise an error code.
 */
static esp_err_t send_enter_motor_mode()
{
    ESP_LOGI(LOG_TAG, "Sending Enter Motor Mode command");
    return send_can_message((uint8_t *)ENTER_MOTOR_MODE_CMD, DATA_LENGTH);
}

/**
 * @brief Sends the "Exit Motor Mode" special command.
 *
 * @return esp_err_t ESP_OK on success, otherwise an error code.
 */
static esp_err_t send_exit_motor_mode()
{
    ESP_LOGI(LOG_TAG, "Sending Exit Motor Mode command");
    return send_can_message((uint8_t *)EXIT_MOTOR_MODE_CMD, DATA_LENGTH);
}

/**
 * @brief Sends the "Zero Position Sensor" special command.
 *
 * @return esp_err_t ESP_OK on success, otherwise an error code.
 */
static esp_err_t send_zero_position_sensor()
{
    ESP_LOGI(LOG_TAG, "Sending Zero Position Sensor command");
    return send_can_message((uint8_t *)ZERO_POS_SENSOR_CMD, DATA_LENGTH);
}

/**
 * @brief Task to handle UART commands.
 *
 * This task reads commands from UART, parses them, and sends corresponding
 * CAN messages. It also handles special motor commands.
 *
 * @param arg Pointer to parameters (unused).
 */
static void uart_command_task(void *arg)
{
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TXD, UART_RXD, UART_RTS, UART_CTS));

    // Allocate buffer for incoming data
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    if (data == NULL)
    {
        ESP_LOGE(LOG_TAG, "Failed to allocate memory for UART data buffer");
        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        // Read data from UART
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0)
        {
            data[len] = '\0'; // Null-terminate the received string
            ESP_LOGI(LOG_TAG, "Received String: %s", (char *)data);

            // Check for special commands first
            if (strncmp((char *)data, "ENTER_MODE", strlen("ENTER_MODE")) == 0)
            {
                if (send_enter_motor_mode() == ESP_OK)
                {
                    ESP_LOGI(LOG_TAG, "Enter Motor Mode command sent successfully");
                }
                else
                {
                    ESP_LOGE(LOG_TAG, "Failed to send Enter Motor Mode command");
                }
            }
            else if (strncmp((char *)data, "EXIT_MODE", strlen("EXIT_MODE")) == 0)
            {
                if (send_exit_motor_mode() == ESP_OK)
                {
                    ESP_LOGI(LOG_TAG, "Exit Motor Mode command sent successfully");
                }
                else
                {
                    ESP_LOGE(LOG_TAG, "Failed to send Exit Motor Mode command");
                }
            }
            else if (strncmp((char *)data, "ZERO_POS", strlen("ZERO_POS")) == 0)
            {
                if (send_zero_position_sensor() == ESP_OK)
                {
                    ESP_LOGI(LOG_TAG, "Zero Position Sensor command sent successfully");
                }
                else
                {
                    ESP_LOGE(LOG_TAG, "Failed to send Zero Position Sensor command");
                }
            }
            else
            {
                // Assume it's a regular motor command. Example: P0.1_V1.0_KP1.0_KD2.0_TFF1.0
                motor_command_t command = {0};
                parse_command((char *)data, &command);

                // Log the parsed command data
                ESP_LOGI(LOG_TAG, "Parsed Command Data:");
                ESP_LOGI(LOG_TAG, "  Position (p_des): %.4f radians", command.p_des);
                ESP_LOGI(LOG_TAG, "  Velocity (v_des): %.4f rad/s", command.v_des);
                ESP_LOGI(LOG_TAG, "  Proportional Gain (kp): %.4f N-m/rad", command.kp);
                ESP_LOGI(LOG_TAG, "  Derivative Gain (kd): %.4f N-m*s/rad", command.kd);
                ESP_LOGI(LOG_TAG, "  Feed-Forward Torque (t_ff): %.4f N-m", command.t_ff);

                // Pack the command data into a CAN frame
                uint8_t can_msg_data[CAN_CMD_LENGTH] = {0}; // 8-byte CAN message buffer
                pack_cmd(command.p_des, command.v_des, command.kp, command.kd, command.t_ff, can_msg_data);

                // Send the CAN message via TWAI
                if (send_can_message(can_msg_data, CAN_CMD_LENGTH) == ESP_OK)
                {
                    ESP_LOGI(LOG_TAG, "CAN message sent successfully");
                }
                else
                {
                    ESP_LOGE(LOG_TAG, "Failed to send CAN message");
                }

                // Optionally, you can handle immediate responses here if needed
                // For now, reception is handled in the separate CAN receive task
            }

            // Optionally, clear the buffer or handle the command as needed
            memset(data, 0, BUF_SIZE); // Clear buffer after processing
        }
        else if (len < 0)
        {
            ESP_LOGE(LOG_TAG, "UART read error");
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // Short delay to prevent task starvation
    }

    // Free allocated memory (unreachable in this example, but good practice)
    free(data);
    vTaskDelete(NULL);
}

/**
 * @brief Task to continuously receive CAN messages.
 *
 * This task runs indefinitely, listening for incoming CAN messages
 * and processing them as they arrive.
 *
 * @param arg Pointer to parameters (unused).
 */
static void can_receive_task(void *arg)
{
    while (1)
    {
        twai_message_t received_msg;
        esp_err_t err = receive_can_message(&received_msg);
        if (err == ESP_OK)
        {
            // Process the received CAN message
            if (received_msg.data_length_code >= 5) // Adjust based on expected data
            {
                motor_reply_t reply = {0}; // Initialize the reply structure

                // Call the unpack_reply function
                unpack_reply(received_msg.data, &reply);

                // Log the unpacked reply data
                ESP_LOGI(LOG_TAG, "Received CAN Reply:");
                ESP_LOGI(LOG_TAG, "  Position: %.4f radians", reply.position);
                ESP_LOGI(LOG_TAG, "  Velocity: %.4f rad/s", reply.velocity);
                ESP_LOGI(LOG_TAG, "  Current: %.2f A", reply.current);
            }
            else
            {
                ESP_LOGE(LOG_TAG, "Received CAN message does not contain enough data to unpack");
            }
        }
        else if (err != ESP_ERR_TIMEOUT)
        {
            // Log the error if reception failed, excluding timeout which is normal
            ESP_LOGE(LOG_TAG, "CAN receive error: %s", esp_err_to_name(err));
        }

        // Short delay to prevent task hogging the CPU
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Should never reach here, but good practice to delete the task if it does
    vTaskDelete(NULL);
}

void app_main(void)
{
    // Initialize TWAI (CAN) driver in loopback mode
    twai_init();

    // Create UART command task
    xTaskCreate(uart_command_task, "uart_command_task", TASK_STACK_SIZE, NULL, 10, NULL);

    // Create CAN receive task
    xTaskCreate(can_receive_task, "can_receive_task", TASK_STACK_SIZE, NULL, 10, NULL);
}
