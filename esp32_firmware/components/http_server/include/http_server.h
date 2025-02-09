#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Starts the HTTP server.
 *
 * This function creates and starts the HTTP server, registers URI handlers for the
 * root (serving index.html), a POST endpoint for commands, and a wildcard handler for files.
 *
 * @return httpd_handle_t The handle to the HTTP server, or NULL on error.
 */
httpd_handle_t http_server_start(void);

#ifdef __cplusplus
}
#endif

#endif // HTTP_SERVER_H
