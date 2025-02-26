#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

#include "esp_http_server.h"

/**
 * @brief Starts the HTTP server.
 *
 * This function creates and starts the HTTP server, registers URI handlers for:
 * - Root (serving index.html)
 * - POST /api/command (for all motor commands as JSON)
 * - POST /api/robot/on
 * - POST /api/robot/off
 * - Wildcard file handler (for static files in SPIFFS)
 *
 * @return httpd_handle_t The handle to the HTTP server, or NULL on error.
 */
httpd_handle_t http_server_start(void);

#endif // HTTP_SERVER_H
