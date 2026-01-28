// main/app_main.c
// ESP32-C3 Super Mini
// - BLE Wi-Fi Provisioning (Espressif "ESP BLE Provisioning" app)
// - After Wi-Fi connected: SPI SLAVE receives [10B hdr + payload] and forwards via UDP
// - No JPEG decode, no frame reassembly on ESP32.
//
// SPI protocol: Header = 10 bytes: <I H B B H  (little-endian)
//   frame_id(u32), chunk_id(u16), flags(u8), rsv(u8), payload_len(u16)
// Then payload_len bytes follow.
//
// Pins (per your table):
//   SCLK=GPIO4, MISO=GPIO5, MOSI=GPIO6, CS=GPIO7, RDY=GPIO10(output to K210)

#include <string.h>
#include <inttypes.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_mac.h"

#include "nvs_flash.h"

#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "driver/gpio.h"
#include "driver/spi_slave.h"

#include "network_provisioning/manager.h"
#include "network_provisioning/scheme_ble.h"

#include "credential.h" // provides UDP_HOST_IP / UDP_HOST_PORT

static const char *TAG = "app_main.c";

// ===== Pins (ESP32-C3 Super Mini) =====
#define PIN_SCLK 4
#define PIN_MISO 5
#define PIN_MOSI 6
#define PIN_CS 7
#define PIN_RDY 10 // output to K210 RDY input

#define SPI_HOST SPI2_HOST
#define DMA_CHAN SPI_DMA_CH_AUTO

// Header = 10 bytes: <I H B B H
#define HDR_LEN 10
#define PAYLOAD_MAX 2048

// Wi-Fi connected event bit
#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t s_wifi_event_group;

// UDP
static int udp_sock = -1;
static struct sockaddr_in udp_dst;

static inline void set_rdy(int v) { gpio_set_level(PIN_RDY, v); }

// -----------------------------
// Wi-Fi event handler
// -----------------------------
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT)
    {
        if (event_id == WIFI_EVENT_STA_START)
        {
            ESP_LOGI(TAG, "WIFI_EVENT_STA_START -> esp_wifi_connect()");
            esp_wifi_connect();
        }
        else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
        {
            ESP_LOGW(TAG, "WIFI_EVENT_STA_DISCONNECTED -> reconnect");
            esp_wifi_connect();
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ESP_LOGI(TAG, "IP_EVENT_STA_GOT_IP");
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// -----------------------------
// Provisioning event handler (only logs)
// -----------------------------
static void prov_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base != NETWORK_PROV_EVENT)
    {
        return;
    }

    switch (event_id)
    {
    case NETWORK_PROV_START:
        ESP_LOGI(TAG, "NETWORK_PROV_START");
        break;
    case NETWORK_PROV_WIFI_CRED_RECV:
        ESP_LOGI(TAG, "NETWORK_PROV_WIFI_CRED_RECV");
        break;
    case NETWORK_PROV_WIFI_CRED_SUCCESS:
        ESP_LOGI(TAG, "NETWORK_PROV_WIFI_CRED_SUCCESS");
        break;
    case NETWORK_PROV_WIFI_CRED_FAIL:
        ESP_LOGW(TAG, "NETWORK_PROV_WIFI_CRED_FAIL");
        break;
    case NETWORK_PROV_END:
        ESP_LOGI(TAG, "NETWORK_PROV_END");
        break;
    default:
        ESP_LOGI(TAG, "NETWORK_PROV_EVENT id=%" PRId32, event_id);
        break;
    }
}

// -----------------------------
// Wi-Fi base init (STA mode, no hardcoded credentials)
// -----------------------------
static void wifi_init_base(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// -----------------------------
// BLE provisioning if not provisioned yet
// SECURITY: WIFI_PROV_SECURITY_0 (lab-friendly)
// -----------------------------
static void ble_provisioning_if_needed(void)
{
    bool provisioned = false;
    ESP_ERROR_CHECK(network_prov_mgr_is_wifi_provisioned(&provisioned));

    if (provisioned)
    {
        ESP_LOGI(TAG, "Already provisioned.");
        return;
    }

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));
    char service_name[32];
    snprintf(service_name, sizeof(service_name), "LV-JPEG-%02X%02X%02X", mac[3], mac[4], mac[5]);

    ESP_LOGI(TAG, "Not provisioned. Start BLE provisioning: %s", service_name);

    // SECURITY_0: easiest (no PoP). If your component version doesn't have SECURITY_0,
    // change this to NETWORK_PROV_SECURITY_1 or _2 (see note below).
    network_prov_security_t security = NETWORK_PROV_SECURITY_2;
    const char *pop = "abcd1234";

    ESP_ERROR_CHECK(network_prov_mgr_start_provisioning(security, pop, service_name, NULL));
    ESP_LOGI(TAG, "Open 'ESP BLE Provisioning' app and provision device: %s", service_name);

    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    ESP_LOGI(TAG, "Provisioned & connected. Deinit provisioning manager.");
    network_prov_mgr_deinit();
}

// -----------------------------
// UDP init
// -----------------------------
static void udp_init(void)
{
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_sock < 0)
    {
        ESP_LOGE(TAG, "socket() failed");
        abort();
    }

    udp_dst.sin_family = AF_INET;
    udp_dst.sin_port = htons(UDP_HOST_PORT);
    udp_dst.sin_addr.s_addr = inet_addr(UDP_HOST_IP);

    ESP_LOGI(TAG, "UDP target %s:%d", UDP_HOST_IP, UDP_HOST_PORT);
}

// -----------------------------
// SPI slave init
// -----------------------------
static void spi_slave_init_bus(void)
{
    gpio_config_t io = {0};
    io.intr_type = GPIO_INTR_DISABLE;
    io.mode = GPIO_MODE_OUTPUT;
    io.pin_bit_mask = 1ULL << PIN_RDY;
    ESP_ERROR_CHECK(gpio_config(&io));
    set_rdy(1);

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = (PAYLOAD_MAX > HDR_LEN ? PAYLOAD_MAX : HDR_LEN),
    };

    spi_slave_interface_config_t slvcfg = {
        .spics_io_num = PIN_CS,
        .queue_size = 8,
        .mode = 0,
        .flags = 0,
    };

    ESP_ERROR_CHECK(spi_slave_initialize(SPI_HOST, &buscfg, &slvcfg, DMA_CHAN));
    ESP_LOGI(TAG, "SPI slave ready (SCLK=%d MOSI=%d MISO=%d CS=%d RDY=%d)",
             PIN_SCLK, PIN_MOSI, PIN_MISO, PIN_CS, PIN_RDY);
}

// -----------------------------
// SPI -> UDP forwarding loop (no reassembly)
// -----------------------------
static void spi_udp_forward_loop(void)
{
    uint8_t hdr[HDR_LEN];
    uint8_t payload[PAYLOAD_MAX];
    spi_slave_transaction_t t = {0};

    while (1)
    {
        set_rdy(1);

        // Receive header (10 bytes)
        memset(&t, 0, sizeof(t));
        memset(hdr, 0, sizeof(hdr));
        t.length = HDR_LEN * 8;
        t.rx_buffer = hdr;
        ESP_ERROR_CHECK(spi_slave_transmit(SPI_HOST, &t, portMAX_DELAY));

        // payload_len is last 2 bytes (little-endian)
        uint16_t payload_len = (uint16_t)(hdr[8] | ((uint16_t)hdr[9] << 8));
        if (payload_len > PAYLOAD_MAX)
            payload_len = PAYLOAD_MAX;

        set_rdy(1);

        // Receive payload
        memset(&t, 0, sizeof(t));
        memset(payload, 0, sizeof(payload));
        t.length = payload_len * 8;
        t.rx_buffer = payload;
        ESP_ERROR_CHECK(spi_slave_transmit(SPI_HOST, &t, portMAX_DELAY));

        // Forward via UDP: [hdr + payload]
        uint8_t out[HDR_LEN + PAYLOAD_MAX];
        memcpy(out, hdr, HDR_LEN);
        memcpy(out + HDR_LEN, payload, payload_len);

        sendto(udp_sock, out, HDR_LEN + payload_len, 0,
               (struct sockaddr *)&udp_dst, sizeof(udp_dst));
    }
}

// -----------------------------
// app_main
// -----------------------------
void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Create event loop + netif first
    wifi_init_base(); // this calls esp_netif_init() + esp_event_loop_create_default()

    // Now init provisioning manager (needs event loop)
    network_prov_mgr_config_t prov_cfg = {
        .scheme = network_prov_scheme_ble,
        .scheme_event_handler = NETWORK_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM,
    };
    ESP_ERROR_CHECK(network_prov_mgr_init(prov_cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(NETWORK_PROV_EVENT, ESP_EVENT_ANY_ID, &prov_event_handler, NULL));

    // Provision if needed
    ble_provisioning_if_needed();

    // For already-provisioned case, still wait for IP
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Wi-Fi connected. Start UDP + SPI forwarding.");

    // Safe deinit (if not already deinit'ed inside provisioning path)
    network_prov_mgr_deinit();

    udp_init();
    spi_slave_init_bus();
    spi_udp_forward_loop();
}
