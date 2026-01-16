/*
 * FreeRTOS STM32 Reference Integration (MXCHIP -> ST67W6X migration)
 *
 * This header is intentionally patterned after mx_lwip.h.
 */

#ifndef ST67_LWIP_MXCOMPAT_H
#define ST67_LWIP_MXCOMPAT_H

#include <stdint.h>

#include "lwip/etharp.h"
#include "lwip/opt.h"
#include "lwip/pbuf.h"
#include "lwip/netifapi.h"
#include "lwip/prot/dhcp.h"

#include "FreeRTOS.h"
#include "task.h"

/* -----------------------------
 * "Generic" aliases (same idea as mx_lwip.h)
 * ----------------------------- */
typedef struct netif      NetInterface_t;
typedef struct pbuf       PacketBuffer_t;
typedef struct eth_addr   MacAddress_t;

/* -----------------------------
 * Network task notification bits
 *
 * These are internal to the networking task implemented in st67_netconn.c
 * and mirrored here because the lwIP status callback needs them.
 * ----------------------------- */
#ifndef NET_EVT_IDX
#define NET_EVT_IDX                    ( 0 )
#endif

#ifndef NET_LWIP_READY_BIT
#define NET_LWIP_READY_BIT             ( 1UL << 0 )
#endif
#ifndef NET_LWIP_IP_CHANGE_BIT
#define NET_LWIP_IP_CHANGE_BIT         ( 1UL << 1 )
#endif
#ifndef NET_LWIP_IFUP_BIT
#define NET_LWIP_IFUP_BIT              ( 1UL << 2 )
#endif
#ifndef NET_LWIP_IFDOWN_BIT
#define NET_LWIP_IFDOWN_BIT            ( 1UL << 3 )
#endif
#ifndef NET_LWIP_LINK_UP_BIT
#define NET_LWIP_LINK_UP_BIT           ( 1UL << 4 )
#endif
#ifndef NET_LWIP_LINK_DOWN_BIT
#define NET_LWIP_LINK_DOWN_BIT         ( 1UL << 5 )
#endif
#ifndef ASYNC_REQUEST_RECONNECT_BIT
#define ASYNC_REQUEST_RECONNECT_BIT    ( 1UL << 6 )
#endif

/* -----------------------------
 * Minimal context shared between st67_netconn and the lwIP callbacks.
 * ----------------------------- */
typedef struct
{
    NetInterface_t xNetif;
    TaskHandle_t xNetTaskHandle;
    MacAddress_t xMacAddress;
} St67NetConnectCtx_t;

/* -----------------------------
 * pbuf helpers
 * ----------------------------- */
#ifndef ST67_RX_BUFF_SZ
/* Conservative default; adjust to your driver RX buffer sizing if needed. */
#define ST67_RX_BUFF_SZ    ( 1600 )
#endif

#define PBUF_VALID( pbuf )          \
    ( ( ( pbuf ) != NULL ) &&       \
      ( ( pbuf )->next == NULL ) && \
      ( ( pbuf )->len > 0 ) &&      \
      ( ( pbuf )->len <= ST67_RX_BUFF_SZ ) )

#define PBUF_LEN( buf )         ( ( buf )->len )
#define PBUF_ALLOC_RX( len )    pbuf_alloc( PBUF_RAW, (u16_t) ( len ), PBUF_POOL )
#define PBUF_ALLOC_TX( len )    pbuf_alloc( PBUF_RAW, (u16_t) ( len ), PBUF_RAM )
#define PBUF_FREE( pbuf )       pbuf_free( pbuf )

/* -----------------------------
 * Helper utilities (logging symbols expected to be provided by the project).
 * ----------------------------- */
static inline void vLogAddress( const char * pucLabel,
                                ip_addr_t xAddress )
{
    uint8_t * pucAddrOctets = ( uint8_t * ) &( xAddress.addr );

    ( void ) pucAddrOctets;

    /* LogSys is used by the original reference integration; keep the same call-site. */
    LogSys( "%-12s%d.%d.%d.%d", pucLabel,
            pucAddrOctets[ 0 ], pucAddrOctets[ 1 ], pucAddrOctets[ 2 ], pucAddrOctets[ 3 ] );
}

static inline void vClearAddress( NetInterface_t * pxNetif )
{
    if( ( pxNetif->ip_addr.addr != 0 ) ||
        ( pxNetif->gw.addr != 0 ) ||
        ( pxNetif->netmask.addr != 0 ) )
    {
        err_t xLwipError;
        struct ip4_addr xEmptyAddr;
        xEmptyAddr.addr = 0;
        xLwipError = netifapi_netif_set_addr( pxNetif,
                                              &xEmptyAddr,
                                              &xEmptyAddr,
                                              &xEmptyAddr );

        if( xLwipError != ERR_OK )
        {
            LogError( "Failed to clear ip address rc: %d", xLwipError );
        }
    }
}

static inline void vStartDhcp( NetInterface_t * pxNetif )
{
    struct dhcp * pxDHCP = netif_dhcp_data( pxNetif );

    if( ( pxDHCP == NULL ) ||
        ( ( pxDHCP != NULL ) && ( pxDHCP->state == DHCP_STATE_OFF ) ) )
    {
        LogInfo( "Starting DHCP." );
        err_t xLwipError = netifapi_dhcp_start( pxNetif );

        if( xLwipError != ERR_OK )
        {
            LogError( "Failed to start DHCP on link rc: %d", xLwipError );
        }
    }
}

static inline void vStopDhcp( NetInterface_t * pxNetif )
{
    struct dhcp * pxDHCP = netif_dhcp_data( pxNetif );

    if( ( pxDHCP != NULL ) && ( pxDHCP->state != DHCP_STATE_OFF ) )
    {
        LogInfo( "Stopping DHCP." );
        /* NOTE: lwIP uses dhcp_stop() for stopping; netifapi_dhcp_stop exists on many ports.
         * If your lwIP port doesn't have netifapi_dhcp_stop, replace with tcpip_callback.
         */
#ifdef netifapi_dhcp_stop
        err_t xLwipError = netifapi_dhcp_stop( pxNetif );
#else
        err_t xLwipError = netifapi_dhcp_release( pxNetif );
#endif
        if( xLwipError != ERR_OK )
        {
            LogError( "Failed to stop DHCP on link rc: %d", xLwipError );
        }
    }
}

static inline void vSetAdminUp( NetInterface_t * pxNetif )
{
    if( ( pxNetif->flags & NETIF_FLAG_UP ) == 0 )
    {
        LogInfo( "Setting interface administrative state to UP." );

        err_t xLwipError = netifapi_netif_set_up( pxNetif );

        if( xLwipError != ERR_OK )
        {
            LogError( "Failed to set interface administrative state to UP rc: %d", xLwipError );
        }
    }
}

static inline void vSetAdminDown( NetInterface_t * pxNetif )
{
    if( ( pxNetif->flags & NETIF_FLAG_UP ) != 0 )
    {
        LogInfo( "Setting interface administrative state to DOWN." );

        err_t xLwipError = netifapi_netif_set_down( pxNetif );

        if( xLwipError != ERR_OK )
        {
            LogError( "Failed to set interface administrative state to DOWN rc: %d", xLwipError );
        }
    }
}

static inline void vSetLinkUp( NetInterface_t * pxNetif )
{
    err_t xLwipError = netifapi_netif_set_link_up( pxNetif );

    if( xLwipError != ERR_OK )
    {
        LogError( "Failed to set link state to UP rc: %d", xLwipError );
    }
}

static inline void vSetLinkDown( NetInterface_t * pxNetif )
{
    err_t xLwipError = netifapi_netif_set_link_down( pxNetif );

    if( xLwipError != ERR_OK )
    {
        LogError( "Failed to set link state to DOWN rc: %d", xLwipError );
    }
}

/* -----------------------------
 * lwIP netif hooks
 * ----------------------------- */
err_t prvxLinkOutput( NetInterface_t * pxNetif, PacketBuffer_t * pxPbuf );
err_t prvInitNetInterface( NetInterface_t * pxNetif );

#endif /* ST67_LWIP_MXCOMPAT_H */
