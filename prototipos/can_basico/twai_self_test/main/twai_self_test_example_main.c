/*
 * SPDX-FileCopyrightText: 2010-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"

/* --------------------- Definitions and static variables ------------------ */

//Example Configurations
#define RX_GPIO_NUM             CONFIG_EXAMPLE_RX_GPIO_NUM
#define TX_GPIO_NUM             CONFIG_EXAMPLE_TX_GPIO_NUM
#define RX_TASK_PRIO            9       //Receiving task priority
#define MSG_ID_STANDARD         0x12    // Standard ID used by Arduino sender
#define MSG_ID_EXTENDED         0xABCDEF // Extended ID used by Arduino sender
#define EXAMPLE_TAG             "TWAI Receiver"

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = {
    .acceptance_code = 0,
    .acceptance_mask = 0xFFFFFFFF,  // Accept all messages
    .single_filter = true
};
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

static SemaphoreHandle_t rx_sem;
static SemaphoreHandle_t done_sem;

/* --------------------------- Tasks and Functions -------------------------- */

static void twai_receive_task(void *arg)
{
    twai_message_t rx_message;

    while (true) {
        xSemaphoreTake(rx_sem, portMAX_DELAY); // Wait for permission to receive messages
        // Receive message and print message data
        if (twai_receive(&rx_message, portMAX_DELAY) == ESP_OK) {
            if (rx_message.extd) {
                ESP_LOGI(EXAMPLE_TAG, "Extended Msg ID 0x%lx: Data = ", rx_message.identifier);
            } else {
                ESP_LOGI(EXAMPLE_TAG, "Standard Msg ID 0x%lx: Data = ", rx_message.identifier);
            }

            for (int i = 0; i < rx_message.data_length_code; i++) {
                printf("%c", rx_message.data[i]);
            }
            printf("\n");
        } else {
            ESP_LOGE(EXAMPLE_TAG, "Failed to receive message");
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    // Create tasks and synchronization primitives
    rx_sem = xSemaphoreCreateBinary();
    done_sem = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);

    // Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");

    // Start TWAI driver
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(EXAMPLE_TAG, "Driver started");

    // Start receiving messages
    xSemaphoreGive(rx_sem);

    // Wait indefinitely (or could use some condition to stop receiving)
    xSemaphoreTake(done_sem, portMAX_DELAY);

    // Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

    // Cleanup
    vSemaphoreDelete(rx_sem);
    vSemaphoreDelete(done_sem);
}
