/*
 * FreeRTOS STM32 Reference Integration (MXCHIP -> ST67W6X migration)
 *
 * Patterned after mx_lwip.c, adapted for ST67W6X.
 */

#include "logging_levels.h"
#define LOG_LEVEL    LOG_ERROR
#include "logging.h"

#include "../W6X_ARCH_T02/st67_lwip_mxcompat.h"
#include "../W6X_ARCH_T02/st67_lwip_netif.h"

#include "lwip/etharp.h"

/* lwIP status callback: translate lwIP netif state changes into task notifications */
static void vLwipStatusCallback( struct netif * pxNetif )
{
    static ip_addr_t xLastAddr = { 0 };
    static uint8_t xLastFlags = 0;

    St67NetConnectCtx_t * pxCtx = ( St67NetConnectCtx_t * ) pxNetif->state;

    uint32_t ulNotifyValue = 0;

    /* Check for change in flags */
    if( ( pxNetif->flags ^ xLastFlags ) & NETIF_FLAG_UP )
    {
        ulNotifyValue |= ( ( pxNetif->flags & NETIF_FLAG_UP ) != 0 ) ? NET_LWIP_IFUP_BIT : NET_LWIP_IFDOWN_BIT;
    }
    else if( ( pxNetif->flags ^ xLastFlags ) & NETIF_FLAG_LINK_UP )
    {
        ulNotifyValue |= ( ( pxNetif->flags & NETIF_FLAG_LINK_UP ) != 0 ) ? NET_LWIP_LINK_UP_BIT : NET_LWIP_LINK_DOWN_BIT;
    }

    if( pxNetif->ip_addr.addr != xLastAddr.addr )
    {
        ulNotifyValue |= NET_LWIP_IP_CHANGE_BIT;
    }

    if( ( ulNotifyValue != 0 ) && ( pxCtx != NULL ) && ( pxCtx->xNetTaskHandle != NULL ) )
    {
        ( void ) xTaskNotifyIndexed( pxCtx->xNetTaskHandle,
                                     NET_EVT_IDX,
                                     ulNotifyValue,
                                     eSetBits );
    }

    xLastAddr = pxNetif->ip_addr;
    xLastFlags = pxNetif->flags;
}

/* Network output function for lwIP */
err_t prvxLinkOutput( NetInterface_t * pxNetif,
                      PacketBuffer_t * pxPbuf )
{
    err_t xError = ERR_OK;
    struct pbuf * pxPbufToSend = pxPbuf;

    if( ( pxPbuf == NULL ) || ( pxNetif == NULL ) )
    {
        return ERR_VAL;
    }

    /* Handle chained pbufs by cloning to a contiguous buffer (same pattern as mx_lwip.c) */
    if( ( pxPbuf->len != pxPbuf->tot_len ) || ( pxPbuf->next != NULL ) )
    {
        pxPbufToSend = pbuf_clone( PBUF_RAW, PBUF_RAM, pxPbuf );
        if( pxPbufToSend == NULL )
        {
            return ERR_MEM;
        }
    }
    else
    {
        /* Ensure the pbuf stays valid for the duration of this function */
        pbuf_ref( pxPbufToSend );
    }

    /* Forward to the ST67W6X netif adapter */
    xError = net_if_output( (struct netif *) pxNetif, pxPbufToSend );

    /* Drop our reference / free clone */
    ( void ) pbuf_free( pxPbufToSend );

    return xError;
}

/* Initialize lwIP netif struct */
err_t prvInitNetInterface( NetInterface_t * pxNetif )
{
    configASSERT( pxNetif != NULL );

    St67NetConnectCtx_t * pxCtx = ( St67NetConnectCtx_t * ) pxNetif->state;

    pxNetif->output = &etharp_output;
    pxNetif->linkoutput = &prvxLinkOutput;

    pxNetif->name[ 0 ] = 's';
    pxNetif->name[ 1 ] = '7';

    pxNetif->num = 0;
    pxNetif->mtu = (u16_t) 1500;

    pxNetif->hwaddr_len = ETHARP_HWADDR_LEN;

    pxNetif->flags = ( NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET );

    netif_set_status_callback( (struct netif *) pxNetif, vLwipStatusCallback );
    netif_set_link_callback( (struct netif *) pxNetif, vLwipStatusCallback );

    /* If the caller provided a MAC, use it; otherwise keep lwIP default zeros. */
    if( pxCtx != NULL )
    {
        ( void ) memcpy( &( pxNetif->hwaddr ), &( pxCtx->xMacAddress ), ETHARP_HWADDR_LEN );
    }

    return ERR_OK;
}
