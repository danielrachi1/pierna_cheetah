#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"

/**
 * @brief Initializes the Wi‑Fi interface and connects to the configured SSID.
 *
 * This function sets up the Wi‑Fi station mode, registers event handlers, and
 * blocks until either a connection is made or the connection fails.
 *
 * @return esp_err_t ESP_OK if connected successfully, or an error code otherwise.
 */
esp_err_t wifi_manager_start(void);

/**
 * @brief Initializes mDNS so that the device can be discovered on the network.
 *
 * This function sets the mDNS hostname, instance name, and registers the HTTP service.
 *
 * @return esp_err_t ESP_OK on success or an error code otherwise.
 */
esp_err_t wifi_manager_setup_mdns(void);

#endif // WIFI_MANAGER_H
