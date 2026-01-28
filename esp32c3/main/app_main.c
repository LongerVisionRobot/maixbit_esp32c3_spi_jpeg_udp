// main/app_main.c
// ESP32-C3 Super Mini: SPI SLAVE receives [10B hdr + payload] and forwards via UDP.
// No JPEG decode, no frame reassembly on ESP32.

#include <string.h>
#include <inttypes.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_slave.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"

#include "credential.h"

static const char *TAG = "app_main.c";

// ===== Your chosen pins (ESP32-C3 Super Mini) =====
// From your table: SCLK=4, MISO=5, MOSI=6, CS=7, RDY=10
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

static int udp_sock = -1;
static struct sockaddr_in udp_dst;

static inline void set_rdy(int v) { gpio_set_level(PIN_RDY, v); }

static void wifi_init_sta_simple(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi connecting to SSID=%s ...", WIFI_SSID);
    ESP_ERROR_CHECK(esp_wifi_connect());
}

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

static void spi_slave_init_bus(void)
{
    // RDY pin output
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
        .mode = 0, // SPI mode 0
        .flags = 0,
    };

    ESP_ERROR_CHECK(spi_slave_initialize(SPI_HOST, &buscfg, &slvcfg, DMA_CHAN));
    ESP_LOGI(TAG, "SPI slave ready (SCLK=%d MOSI=%d MISO=%d CS=%d RDY=%d)",
             PIN_SCLK, PIN_MOSI, PIN_MISO, PIN_CS, PIN_RDY);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_init_sta_simple();
    // crude DHCP wait (good enough for LAN testing)
    vTaskDelay(pdMS_TO_TICKS(3000));
    udp_init();

    spi_slave_init_bus();

    uint8_t hdr[HDR_LEN];
    uint8_t payload[PAYLOAD_MAX];

    spi_slave_transaction_t t = {0};

    while (1)
    {
        // Tell K210 it's OK to send header
        set_rdy(1);

        // ---- Receive header (10 bytes) ----
        memset(&t, 0, sizeof(t));
        memset(hdr, 0, sizeof(hdr));
        t.length = HDR_LEN * 8;
        t.rx_buffer = hdr;
        ESP_ERROR_CHECK(spi_slave_transmit(SPI_HOST, &t, portMAX_DELAY));

        // Parse payload_len from header last 2 bytes (little-endian)
        uint16_t payload_len = (uint16_t)(hdr[8] | ((uint16_t)hdr[9] << 8));
        if (payload_len > PAYLOAD_MAX)
            payload_len = PAYLOAD_MAX;

        // Tell K210 it's OK to send payload
        set_rdy(1);

        // ---- Receive payload ----
        memset(&t, 0, sizeof(t));
        memset(payload, 0, sizeof(payload));
        t.length = payload_len * 8;
        t.rx_buffer = payload;
        ESP_ERROR_CHECK(spi_slave_transmit(SPI_HOST, &t, portMAX_DELAY));

        // ---- Forward via UDP: [hdr(10) + payload(payload_len)] ----
        uint8_t out[HDR_LEN + PAYLOAD_MAX];
        memcpy(out, hdr, HDR_LEN);
        memcpy(out + HDR_LEN, payload, payload_len);

        sendto(udp_sock, out, HDR_LEN + payload_len, 0,
               (struct sockaddr *)&udp_dst, sizeof(udp_dst));
    }
}