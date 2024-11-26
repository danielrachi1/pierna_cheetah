#include <stdio.h>
#include <string.h> // Include for string operations
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "can_utils.h"
#include "uart_utils.h"

#include "driver/twai.h" // Include for TWAI (CAN) functionality

// UART Configuration
#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM (CONFIG_EXAMPLE_UART_PORT_NUM)
#define ECHO_UART_BAUD_RATE (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE (CONFIG_EXAMPLE_TASK_STACK_SIZE)

// TWAI (CAN) Configuration
#define TWAI_TX_GPIO_NUM (21) // TX pin
#define TWAI_RX_GPIO_NUM (22) // RX pin

#define MOTOR_CAN_ID 0x00

static const char *TAG = "SINGLE MOTOR CONTROL";

#define BUF_SIZE (1024)

/**
 * @brief Initializes the TWAI (CAN) driver in loopback mode.
 */
static void twai_init()
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO_NUM, TWAI_RX_GPIO_NUM, TWAI_MODE_NO_ACK);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "TWAI driver installed");

    // Start TWAI driver
    err = twai_start();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "TWAI driver started in loopback mode");
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
    if (length > 8)
    {
        ESP_LOGE(TAG, "CAN message length exceeds 8 bytes");
        return ESP_ERR_INVALID_ARG;
    }

    // Initialize the message structure
    twai_message_t message;
    memset(&message, 0, sizeof(twai_message_t)); // Clear all fields

    // Set message fields
    message.identifier = MOTOR_CAN_ID; // Set your CAN ID
    message.data_length_code = length; // Set data length code
    message.self = 1;

    memcpy(message.data, msg_data, length); // Copy data

    // *** Added Logging: Log the entire TWAI message before sending ***
    ESP_LOGI(TAG, "Preparing to Send CAN Message:");
    ESP_LOGI(TAG, "  extd: %lu", (unsigned long)message.extd);
    ESP_LOGI(TAG, "  rtr: %lu", (unsigned long)message.rtr);
    ESP_LOGI(TAG, "  ss: %lu", (unsigned long)message.ss);
    ESP_LOGI(TAG, "  self: %lu", (unsigned long)message.self);
    ESP_LOGI(TAG, "  dlc_non_comp: %lu", (unsigned long)message.dlc_non_comp);
    ESP_LOGI(TAG, "  reserved: %lu", (unsigned long)message.reserved);
    ESP_LOGI(TAG, "  flags: %lu (deprecated)", (unsigned long)message.flags);
    ESP_LOGI(TAG, "  identifier: 0x%lX", (unsigned long)message.identifier);
    ESP_LOGI(TAG, "  data_length_code: %u", message.data_length_code);

    // Ensure that data_length_code does not exceed the data array size
    if (message.data_length_code > sizeof(message.data))
    {
        ESP_LOGE(TAG, "Data length code (%d) exceeds buffer size (%zu)", message.data_length_code, sizeof(message.data));
        return ESP_ERR_INVALID_ARG;
    }

    // Log each byte of the CAN message data
    ESP_LOGI(TAG, "  Data:");
    for (int i = 0; i < message.data_length_code; i++)
    {
        ESP_LOGI(TAG, "    Byte %d: 0x%02X", i, message.data[i]);
    }

    // Transmit the message
    esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(1000)); // Timeout 1 second
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "TWAI transmit failed: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "TWAI transmit successful");
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
        ESP_LOGI(TAG, "Received CAN Message:");

        // Log each field with appropriate format specifiers and casting
        ESP_LOGI(TAG, "  extd: %lu", (unsigned long)message->extd);
        ESP_LOGI(TAG, "  rtr: %lu", (unsigned long)message->rtr);
        ESP_LOGI(TAG, "  ss: %lu", (unsigned long)message->ss);
        ESP_LOGI(TAG, "  self: %lu", (unsigned long)message->self);
        ESP_LOGI(TAG, "  dlc_non_comp: %lu", (unsigned long)message->dlc_non_comp);
        ESP_LOGI(TAG, "  reserved: %lu", (unsigned long)message->reserved);
        ESP_LOGI(TAG, "  flags: %lu (deprecated)", (unsigned long)message->flags);
        ESP_LOGI(TAG, "  identifier: 0x%lX", (unsigned long)message->identifier);
        ESP_LOGI(TAG, "  data_length_code: %u", message->data_length_code);

        // Ensure that data_length_code does not exceed the data array size
        if (message->data_length_code > sizeof(message->data))
        {
            ESP_LOGE(TAG, "Data length code (%d) exceeds buffer size (%zu)", message->data_length_code, sizeof(message->data));
            return ESP_ERR_INVALID_ARG;
        }

        // Log each byte of the CAN message data
        ESP_LOGI(TAG, "  Data:");
        for (int i = 0; i < message->data_length_code; i++)
        {
            ESP_LOGI(TAG, "    Byte %d: 0x%02X", i, message->data[i]);
        }
    }
    else
    {
        // Log the error if reception failed
        ESP_LOGE(TAG, "TWAI receive failed: %s", esp_err_to_name(err));
    }

    return err;
}

static void uart_command_task(void *arg)
{
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
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
    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Allocate buffer for incoming data
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    if (data == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for UART buffer");
        vTaskDelete(NULL); // Delete task if memory allocation fails
    }

    while (1)
    {
        // Read data from UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0)
        {
            data[len] = '\0'; // Null-terminate the received string
            ESP_LOGI(TAG, "Received String: %s", (char *)data);

            // Parse the command string
            motor_command_t command = {0};
            parse_command((char *)data, &command);

            // Log the parsed command data
            ESP_LOGI(TAG, "Parsed Command Data:");
            ESP_LOGI(TAG, "  Position (p_des): %.4f radians", command.p_des);
            ESP_LOGI(TAG, "  Velocity (v_des): %.4f rad/s", command.v_des);
            ESP_LOGI(TAG, "  Proportional Gain (kp): %.4f N-m/rad", command.kp);
            ESP_LOGI(TAG, "  Derivative Gain (kd): %.4f N-m*s/rad", command.kd);
            ESP_LOGI(TAG, "  Feed-Forward Torque (t_ff): %.4f N-m", command.t_ff);

            // Pack the command data into a CAN frame
            uint8_t can_msg_data[CAN_CMD_LENGTH] = {0}; // 8-byte CAN message buffer
            pack_cmd(command.p_des, command.v_des, command.kp, command.kd, command.t_ff, can_msg_data);

            // Log the packed CAN message in hexadecimal format for verification
            ESP_LOGI(TAG, "Packed CAN Data:");
            for (int i = 0; i < CAN_CMD_LENGTH; i++)
            {
                ESP_LOGI(TAG, "  Byte %d: 0x%02X", i, can_msg_data[i]);
            }

            // Send the CAN message via TWAI
            if (send_can_message(can_msg_data, CAN_CMD_LENGTH) == ESP_OK)
            {
                ESP_LOGI(TAG, "CAN message sent successfully");
            }
            else
            {
                ESP_LOGE(TAG, "Failed to send CAN message");
            }

            // Receive the CAN message via TWAI (self-reception in loopback mode)
            twai_message_t received_msg;
            if (receive_can_message(&received_msg) == ESP_OK)
            {
                ESP_LOGI(TAG, "CAN message received successfully");
            }
            else
            {
                ESP_LOGE(TAG, "Failed to receive CAN message");
            }

            // Optionally, clear the buffer or handle the command as needed
            memset(data, 0, BUF_SIZE); // Clear buffer after processing
        }
        else if (len < 0)
        {
            ESP_LOGE(TAG, "UART read error");
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // Short delay to prevent task starvation
    }

    // Free allocated memory (unreachable in this example, but good practice)
    free(data);
    vTaskDelete(NULL);
}

void app_main(void)
{
    // Initialize TWAI (CAN) driver in loopback mode
    twai_init();

    // Create UART command task
    xTaskCreate(uart_command_task, "uart_command_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}