/*
 * FreeRTOS STM32 Reference Integration (MXCHIP -> ST67W6X migration)
 *
 * This header is intentionally API-compatible with mx_netconn.h.
 */

#ifndef ST67_NETCONN_H
#define ST67_NETCONN_H

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Networking task entry point (same signature/name as mx_netconn).
 */
void net_main( void * pvParameters );

/**
 * @brief Asks the networking task to reconnect.
 *
 * The default implementation triggers an internal notification bit. The actual
 * reconnect action is delegated to a weak hook (see st67_netconn.c).
 */
BaseType_t net_request_reconnect( void );

#ifdef __cplusplus
}
#endif

#endif /* ST67_NETCONN_H */
