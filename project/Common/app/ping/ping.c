/*
 * ping_fixed2.c
 *
 * Socket-based ICMP ping for lwIP (IPv4).
 * Based on lwIP contrib ping/ping.c ideas:
 * - use getaddrinfo for DNS
 * - set SO_RCVTIMEO using struct timeval
 * - initialize fromlen
 * - check sendto/recvfrom error codes
 * - loop on recv until timeout expires, ignoring unrelated ICMP packets
 *
 * Notes:
 * - Requires LWIP_SOCKET && LWIP_RAW && LWIP_ICMP && LWIP_DNS (for hostname)
 * - Raw sockets may be disabled on some vendor ports; if this still times out,
 *   consider using lwIP RAW API (raw_pcb) instead of sockets.
 */

#if (defined(ETHERNET) || defined(MXCHIP) || defined(ST67W6X_RCP))
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/icmp.h"
#include "lwip/ip4.h"
#include "lwip/ip_addr.h"
#include "lwip/netdb.h"
#include "lwip/errno.h"
#include "lwip/inet_chksum.h"
#include "lwip/sys.h"
#include "lwip/timeouts.h"
#endif

#if defined(ST67W6X_NCP)
#define u16_t uint16_t
#include "w6x_lwip_port.h"
#endif

#include "logging.h"
#include "sys_evt.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "kvstore.h"

#define PING_INTERVAL_MS   1000
#define PING_PAYLOAD_SIZE  32
#define PING_TIMEOUT_MS    1000
#define PING_COUNT         4

#define PING_ID            0xAFAF

static u16_t s_ping_seq;
static uint32_t s_ping_time_sent_ms;

/* ---------- DNS ---------- */

static BaseType_t resolve_hostname_v4(const char *hostname, ip_addr_t *out_addr, struct in_addr *out_inaddr)
{
    if (!hostname || !out_addr || !out_inaddr || hostname[0] == '\0')
    {
        LogError("resolve_hostname_v4: invalid args");
        return pdFALSE;
    }

    struct addrinfo hints;
    struct addrinfo *res = NULL;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET; /* IPv4 */

    int gai = lwip_getaddrinfo(hostname, NULL, &hints, &res);
    if (gai != 0 || res == NULL || res->ai_addr == NULL)
    {
        LogError("Failed to resolve hostname: %s (getaddrinfo=%d)", hostname, gai);
        return pdFALSE;
    }

    struct sockaddr_in *sa = (struct sockaddr_in *)res->ai_addr;
    *out_inaddr = sa->sin_addr;

#if LWIP_IPV4 && LWIP_IPV6
    ip_addr_set_ip4_u32(out_addr, (u32_t)sa->sin_addr.s_addr);
#else
    out_addr->addr = (u32_t)sa->sin_addr.s_addr;
#endif

    LogInfo("Resolved hostname %s to IP: %s", hostname, inet_ntoa(sa->sin_addr));
    lwip_freeaddrinfo(res);
    return pdTRUE;
}

/* ---------- ICMP ---------- */

static void ping_prepare_echo(struct icmp_echo_hdr *iecho, u16_t len)
{
    size_t i;
    const size_t hdr_len = sizeof(struct icmp_echo_hdr);
    const size_t data_len = (len > hdr_len) ? (len - hdr_len) : 0;

    ICMPH_TYPE_SET(iecho, ICMP_ECHO);
    ICMPH_CODE_SET(iecho, 0);
    iecho->chksum = 0;
    iecho->id = PING_ID;
    iecho->seqno = htons(++s_ping_seq);

    for (i = 0; i < data_len; i++)
    {
        ((uint8_t *)iecho)[hdr_len + i] = (uint8_t)i;
    }

    iecho->chksum = inet_chksum(iecho, len);
}

static BaseType_t ping_send(int sock, const struct in_addr *dst)
{
    const size_t ping_len = sizeof(struct icmp_echo_hdr) + PING_PAYLOAD_SIZE;

    struct icmp_echo_hdr *iecho = (struct icmp_echo_hdr *)mem_malloc((mem_size_t)ping_len);
    if (!iecho)
    {
        LogError("Failed to allocate ping buffer (%u bytes)", (unsigned)ping_len);
        return pdFALSE;
    }

    ping_prepare_echo(iecho, (u16_t)ping_len);

    struct sockaddr_in to;
    memset(&to, 0, sizeof(to));
    to.sin_len = sizeof(to);
    to.sin_family = AF_INET;
    to.sin_port = 0;
    to.sin_addr = *dst;

    s_ping_time_sent_ms = sys_now();

    int sent = lwip_sendto(sock, iecho, ping_len, 0, (struct sockaddr *)&to, sizeof(to));
    mem_free(iecho);

    if (sent < 0)
    {
        LogError("sendto() failed (errno=%d)", errno);
        return pdFALSE;
    }

    LogDebug("Sent ICMP Echo: dst=%s id=0x%04x seq=%u bytes=%d",
             inet_ntoa(to.sin_addr), (unsigned)PING_ID, (unsigned)s_ping_seq, sent);
    return pdTRUE;
}

static BaseType_t ping_recv_wait(int sock, uint32_t timeout_ms, const struct in_addr *expected_from)
{
    /* Large enough for IP header + ICMP header + payload */
    uint8_t buf[128];

    const uint32_t start_ms = sys_now();
    while ((sys_now() - start_ms) < timeout_ms)
    {
        struct sockaddr_in from;
        socklen_t fromlen = sizeof(from);
        int len = lwip_recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &fromlen);

        if (len < 0)
        {
            if (errno == EWOULDBLOCK || errno == EAGAIN)
            {
                /* timeout / no data */
                return pdFALSE;
            }
            LogError("recvfrom() failed (errno=%d)", errno);
            return pdFALSE;
        }

        /* Optional: ensure reply is from the expected host */
        if (expected_from && (from.sin_addr.s_addr != expected_from->s_addr))
        {
            LogDebug("Ignoring packet from %s (expected %s)",
                     inet_ntoa(from.sin_addr), inet_ntoa(*expected_from));
            continue;
        }

        /* lwIP raw sockets usually deliver IP header + payload */
        if (len < (int)(sizeof(struct ip_hdr) + sizeof(struct icmp_echo_hdr)))
        {
            LogDebug("Ignoring short packet: %d bytes", len);
            continue;
        }

        struct ip_hdr *iphdr = (struct ip_hdr *)buf;
        const uint16_t iphdr_len = (uint16_t)(IPH_HL(iphdr) * 4U);
        if (iphdr_len < sizeof(struct ip_hdr) || iphdr_len > (uint16_t)len)
        {
            LogDebug("Ignoring invalid IP header length: %u (len=%d)", (unsigned)iphdr_len, len);
            continue;
        }

        struct icmp_echo_hdr *iecho = (struct icmp_echo_hdr *)(buf + iphdr_len);

        if ((ICMPH_TYPE(iecho) == ICMP_ER) &&
            (iecho->id == PING_ID) &&
            (iecho->seqno == htons(s_ping_seq)))
        {
            const uint32_t rtt = sys_now() - s_ping_time_sent_ms;
            const uint8_t ttl = iphdr->_ttl;

            LogInfo("Reply from %s: bytes=%d time=%ums TTL=%u",
                    inet_ntoa(from.sin_addr), len, (unsigned)rtt, (unsigned)ttl);
            return pdTRUE;
        }

        /* Ignore unrelated ICMP (e.g., other app traffic) */
        LogDebug("Ignoring ICMP: type=%u code=%u id=0x%04x seq=%u",
                 (unsigned)ICMPH_TYPE(iecho),
                 (unsigned)ICMPH_CODE(iecho),
                 (unsigned)iecho->id,
                 (unsigned)lwip_ntohs(iecho->seqno));
    }

    return pdFALSE;
}

/* ---------- Task ---------- */

void ping_task(void *pvParameters)
{
    (void)pvParameters;

    LogInfo("%s started", __func__);

    LogInfo("Waiting for network connection...");
    (void)xEventGroupWaitBits(xSystemEvents, EVT_MASK_NET_CONNECTED, pdFALSE, pdTRUE, portMAX_DELAY);
    LogInfo("Network connected");

    size_t host_len = 0;
    char *host = KVStore_getStringHeap(CS_CORE_MQTT_ENDPOINT, &host_len);
    if (!host)
    {
        LogError("KVStore_getStringHeap(%s) failed", CS_CORE_MQTT_ENDPOINT);
        vTaskDelete(NULL);
    }

    ip_addr_t target_ip;
    struct in_addr target_inaddr;
    if (resolve_hostname_v4(host, &target_ip, &target_inaddr) != pdTRUE)
    {
        vPortFree(host);
        vTaskDelete(NULL);
    }
    vPortFree(host);

    int s = lwip_socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
    if (s < 0)
    {
        LogError("Failed to create raw ICMP socket (errno=%d)", errno);
        vTaskDelete(NULL);
    }

    /* IMPORTANT: lwIP expects SO_RCVTIMEO as struct timeval on many ports. */
    struct timeval tv;
    tv.tv_sec = PING_TIMEOUT_MS / 1000;
    tv.tv_usec = (PING_TIMEOUT_MS % 1000) * 1000;
    if (lwip_setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) != 0)
    {
        LogError("setsockopt(SO_RCVTIMEO) failed (errno=%d)", errno);
    }

    for (int i = 0; i < PING_COUNT; i++)
    {
        LogDebug("Sending ping %d/%d...", i + 1, PING_COUNT);

        if (ping_send(s, &target_inaddr) == pdTRUE)
        {
            BaseType_t ok = ping_recv_wait(s, PING_TIMEOUT_MS, &target_inaddr);
            if (!ok)
            {
                LogError("Ping reply reception failed or timed out (errno=%d)", errno);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(PING_INTERVAL_MS));
    }

    (void)lwip_close(s);
    LogInfo("Ping process completed. Terminating task.");
    vTaskDelete(NULL);
}
