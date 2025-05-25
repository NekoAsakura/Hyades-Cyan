#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/dhcpv4_server.h>
#include <zephyr/net/net_if.h>
#include <zephyr/posix/sys/socket.h>
#include <zephyr/posix/arpa/inet.h>
#include <zephyr/posix/unistd.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include "tjpgd.h"
#include <string.h>

LOG_MODULE_REGISTER(main_ap);

#define WIFI_AP_SSID    "M5CORE2"
#define WIFI_AP_PSK     ""
#define WIFI_AP_IP      "192.168.5.1"
#define WIFI_AP_NETMASK "255.255.255.0"
#define DHCP_POOL_INIT  14
#define TCP_PORT        8000

#define FRAME_BUF_SIZE  (8 * 1024)
#define FRAME_WIDTH     320
#define MJPEG_STACK_SIZE 8192

static struct net_if *ap_iface;
static struct net_mgmt_event_callback wifi_cb;
static int server_sock = -1;

__attribute__((section(".ext_ram.bss")))
static uint8_t frame_buf[FRAME_BUF_SIZE];
static uint16_t line_buf[FRAME_WIDTH];

static const uint8_t *jpg_ptr;
static size_t jpg_size;
static size_t jpg_pos;

static size_t jpg_infunc(JDEC *jd, uint8_t *buf, size_t n)
{
    size_t remain = jpg_size - jpg_pos;
    size_t cnt    = remain < n ? remain : n;
    if (buf) {
        memcpy(buf, jpg_ptr + jpg_pos, cnt);
    }
    jpg_pos += cnt;
    return cnt;
}

static int jpg_output(JDEC *jd, void *bitmap, JRECT *rect)
{
    const struct device *disp = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(disp)) {
        return 0;
    }
    static bool inited;
    if (!inited) {
        display_blanking_off(disp);
        inited = true;
    }
    struct display_buffer_descriptor desc = {
        .buf_size = (rect->right  - rect->left + 1) *
                    (rect->bottom - rect->top  + 1) * 2,
        .width    = rect->right  - rect->left + 1,
        .height   = rect->bottom - rect->top  + 1,
        .pitch    = FRAME_WIDTH,
    };
    display_write(disp,
                  rect->left,
                  rect->top,
                  &desc,
                  bitmap);
    return 1;
}

static void setup_ip_and_dhcp(void)
{
    struct in_addr ip, nm, pool;
    net_addr_pton(AF_INET, WIFI_AP_IP,      &ip);
    net_addr_pton(AF_INET, WIFI_AP_NETMASK, &nm);
    pool = ip;
    pool.s4_addr[3] = DHCP_POOL_INIT;
    net_if_ipv4_addr_add(ap_iface, &ip,   NET_ADDR_MANUAL, 0);
    net_if_ipv4_set_netmask_by_addr(ap_iface, &ip, &nm);
    net_dhcpv4_server_start(ap_iface, &pool);
}

static void start_tcp_server(void)
{
    struct sockaddr_in addr;
    if (server_sock >= 0) {
        return;
    }
    server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_sock < 0) {
        return;
    }
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(TCP_PORT);
    net_addr_pton(AF_INET, WIFI_AP_IP, &addr.sin_addr);
    if (bind(server_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(server_sock);
        server_sock = -1;
        return;
    }
    if (listen(server_sock, 1) < 0) {
        close(server_sock);
        server_sock = -1;
    }
}

static void wifi_event_handler(struct net_mgmt_event_callback *cb,
                               uint32_t mgmt_event,
                               struct net_if *iface)
{
    if (mgmt_event == NET_EVENT_WIFI_AP_ENABLE_RESULT) {
        ap_iface = iface;
        setup_ip_and_dhcp();
    } else if (mgmt_event == NET_EVENT_WIFI_AP_STA_CONNECTED) {
        start_tcp_server();
    }
}

static void mjpeg_thread(void *p1, void *p2, void *p3)
{
    int client, rd;
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    size_t buf_pos    = 0;
    JDEC jd;

    while (1) {
        if (server_sock < 0) {
            k_sleep(K_SECONDS(1));
            continue;
        }
        client = accept(server_sock,
                        (struct sockaddr *)&client_addr,
                        &addr_len);
        buf_pos = 0;
        while ((rd = recv(client,
                          frame_buf + buf_pos,
                          FRAME_BUF_SIZE - buf_pos,
                          0)) > 0) {
            buf_pos += rd;
            for (size_t i = 0; i + 1 < buf_pos; i++) {
                if (frame_buf[i] == 0xFF &&
                    frame_buf[i + 1] == 0xD8) {
                    size_t start = i;
                    for (size_t j = start + 2;
                         j + 1 < buf_pos; j++) {
                        if (frame_buf[j] == 0xFF &&
                            frame_buf[j + 1] == 0xD9) {
                            size_t len = j + 2 - start;
                            jpg_ptr  = frame_buf + start;
                            jpg_size = len;
                            jpg_pos  = 0;
                            if (jd_prepare(&jd,
                                           jpg_infunc,
                                           line_buf,
                                           sizeof(line_buf),
                                           NULL) == JDR_OK) {
                                jd_decomp(&jd,
                                          jpg_output,
                                          0);
                            }
                            if (j + 2 < buf_pos) {
                                memmove(frame_buf,
                                        frame_buf + j + 2,
                                        buf_pos - (j + 2));
                                buf_pos -= (j + 2);
                            } else {
                                buf_pos = 0;
                            }
                            i = 0;
                            break;
                        }
                    }
                }
            }
        }
        close(client);
    }
}

Z_KERNEL_STACK_DEFINE_IN(mjpeg_stack, MJPEG_STACK_SIZE,
                         __attribute__((section(".ext_ram.bss"))));
static struct k_thread mjpeg_thread_data;

int main(void)
{
    struct wifi_connect_req_params ap = {
        .ssid         = (const uint8_t *)WIFI_AP_SSID,
        .ssid_length  = sizeof(WIFI_AP_SSID) - 1,
        .psk          = (const uint8_t *)WIFI_AP_PSK,
        .psk_length   = sizeof(WIFI_AP_PSK) - 1,
        .channel      = WIFI_CHANNEL_ANY,
        .band         = WIFI_FREQ_BAND_2_4_GHZ,
        .security     = (sizeof(WIFI_AP_PSK) == 1)
                        ? WIFI_SECURITY_TYPE_NONE
                        : WIFI_SECURITY_TYPE_PSK,
    };

    k_sleep(K_SECONDS(2));

    net_mgmt_init_event_callback(&wifi_cb,
                                 wifi_event_handler,
                                 NET_EVENT_WIFI_AP_ENABLE_RESULT |
                                 NET_EVENT_WIFI_AP_STA_CONNECTED);
    net_mgmt_add_event_callback(&wifi_cb);

    net_mgmt(NET_REQUEST_WIFI_AP_ENABLE,
             net_if_get_wifi_sap(),
             &ap, sizeof(ap));

    k_thread_create(&mjpeg_thread_data,
                    mjpeg_stack,
                    MJPEG_STACK_SIZE,
                    mjpeg_thread,
                    NULL, NULL, NULL,
                    5, 0, K_NO_WAIT);

    while (1) {
        k_sleep(K_SECONDS(60));
    }

    return 0;
}