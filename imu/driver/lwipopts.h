#pragma once

/* ===== Build style: NO OS, Pico W background lwIP ===== */
#define NO_SYS                         1    
#define SYS_LIGHTWEIGHT_PROT           0

/* ===== Core features ===== */
#define LWIP_IPV4                      1
#define LWIP_IPV6                      0
#define LWIP_ARP                       1
#define LWIP_ICMP                      1
#define LWIP_RAW                       1            /* needed by MQTT (raw API) */
#define LWIP_UDP                       1
#define LWIP_TCP                       1
#define TCP_QUEUE_OOSEQ                0

/* We are NOT using netconn/sockets APIs, only raw API */
#define LWIP_NETCONN                   0
#define LWIP_SOCKET                    0

/* DHCP/DNS (keep enabled unless you hardcode static IPs and raw IP broker) */
#define LWIP_DHCP                      1
#define LWIP_DNS                       1
#define LWIP_NETIF_HOSTNAME            1

/* ===== Memory / pools ===== */
#define MEM_ALIGNMENT                  4
#define MEM_SIZE                       10000

#define MEMP_NUM_PBUF                  16
#define PBUF_POOL_SIZE                 16
#define PBUF_POOL_BUFSIZE              512

#define MEMP_NUM_UDP_PCB               4
#define MEMP_NUM_TCP_PCB               6
/* >= TCP_SND_QUEUELEN (see below) */
#define MEMP_NUM_TCP_SEG               32
#define MEMP_NUM_SYS_TIMEOUT           10

/* ===== TCP tuning (balanced for small pool) ===== */
#define TCP_MSS                        1460
/* Fits into pool: 4 * 1460 = 5840 (<= ~16 * (512 - headers)) */
#define TCP_WND                        (4 * TCP_MSS)
/* Keep send buffer modest and consistent with window */
#define TCP_SND_BUF                    (4 * TCP_MSS)
/* Queue length derived from SND_BUF -> about 20; set explicitly */
#define TCP_SND_QUEUELEN               20

#define LWIP_TCP_KEEPALIVE             1

/* Optional: if you still hit sanity checks while experimenting,
   you can disable them — but only if you know the implications. */
/* #define LWIP_DISABLE_TCP_SANITY_CHECKS 1 */


/* ===== Misc ===== */
#define LWIP_RANDOMIZE_INITIAL_LOCAL_PORTS 1
#define LWIP_STATS                     0
#define LWIP_PROVIDE_ERRNO             1

/* ===== MQTT app (non-TLS) ===== */
#define LWIP_ALTCP                     0
/* Some SDKs use this flag; harmless if ignored: */
#define LWIP_MQTT                      1
