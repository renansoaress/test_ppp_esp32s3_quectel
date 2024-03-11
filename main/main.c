/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* PPPoS Client Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <regex.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_modem_api.h"
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "esp_sntp.h"
#include "driver/gpio.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "mbedtls/md.h"
#include "mbedtls/base64.h"
#include "lwip/dns.h"

// Quectel
#define EG915_UART_AT_RX    (GPIO_NUM_35)   /* UART RX - EG915 */
#define EG915_UART_AT_TX    (GPIO_NUM_34)   /* UART TX - EG915 */
#define EG915_RST           (GPIO_NUM_39)   /* Digital output - EG915 reset signal */
#define EG915_PWRKEY        (GPIO_NUM_40)   /* Digital output - EG915 power-key signal */
#define EG915_MAIN_DTR      (GPIO_NUM_33)

#define GPIO_OUTPUT_PIN_SEL ((1ULL << EG915_RST) | \
    (1ULL << EG915_PWRKEY) | (1ULL <<  EG915_MAIN_DTR) \
    )

static const char *TAG = "pppos_example";
static EventGroupHandle_t event_group = NULL;
static const int CONNECT_BIT = BIT0;
static const int DISCONNECT_BIT = BIT2;
#define MAX_APN_LEN 50
#define MAX_CONN_STR_LEN 200
static char APN[MAX_APN_LEN] = "lf.br\0";
// static char APN[MAX_APN_LEN] = "m2m.pc.br\0";

static esp_modem_dce_t *dce = NULL;
static bool flag_connection = false;

static void on_ppp_changed(void *arg, esp_event_base_t event_base,
                           int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "PPP state changed event");
    if (event_id == NETIF_PPP_ERRORUSER) {
        /* User interrupted event from esp-netif */
        esp_netif_t *netif = event_data;
        ESP_LOGI(TAG, "User interrupted event from netif:%p", netif);
    }
}


static void on_ip_event(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    // ESP_LOGI(TAG, "IP event! %d", event_id);
    if (event_id == IP_EVENT_PPP_GOT_IP) {
        esp_netif_dns_info_t dns_info;

        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        esp_netif_t *netif = event->esp_netif;

        ESP_LOGI(TAG, "Modem Connect to PPP Server");
        ESP_LOGI(TAG, "~~~~~~~~~~~~~~");
        ESP_LOGI(TAG, "IP          : " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Netmask     : " IPSTR, IP2STR(&event->ip_info.netmask));
        ESP_LOGI(TAG, "Gateway     : " IPSTR, IP2STR(&event->ip_info.gw));
        esp_netif_get_dns_info(netif, 0, &dns_info);
        ESP_LOGI(TAG, "Name Server1: " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
        esp_netif_get_dns_info(netif, 1, &dns_info);
        ESP_LOGI(TAG, "Name Server2: " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
        ESP_LOGI(TAG, "~~~~~~~~~~~~~~");
        xEventGroupSetBits(event_group, CONNECT_BIT);

        ESP_LOGI(TAG, "GOT ip event!!!");
    } else if (event_id == IP_EVENT_PPP_LOST_IP) {
        ESP_LOGI(TAG, "Modem Disconnect from PPP Server");
        xEventGroupSetBits(event_group, DISCONNECT_BIT);
    } else if (event_id == IP_EVENT_GOT_IP6) {
        ESP_LOGI(TAG, "GOT IPv6 event!");

        ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
        ESP_LOGI(TAG, "Got IPv6 address " IPV6STR, IPV62STR(event->ip6_info.ip));
    }
}

esp_err_t set_EG_RST_level(uint32_t level) {
    return gpio_set_level(EG915_RST, level);
}

esp_err_t set_EG_PWRKEY_level(uint32_t level) {
    return gpio_set_level(EG915_PWRKEY, level);
}

void reset_modem(void)
{
    ESP_LOGI(TAG, "Restart the modem");
    set_EG_RST_level(1);
    vTaskDelay(pdMS_TO_TICKS(1350));
    set_EG_RST_level(0);
    vTaskDelay(pdMS_TO_TICKS(12000));
}

void wakeup_modem(void)
{
    ESP_LOGI(TAG, "Power on the modem");
    set_EG_PWRKEY_level(1);
    vTaskDelay(pdMS_TO_TICKS(750));
    set_EG_PWRKEY_level(0);
    vTaskDelay(pdMS_TO_TICKS(12000));
}

void check_signal(void)
{
    while(true)
    {
        if(flag_connection)
        {
            int rssi, ber;
            esp_err_t err = esp_modem_get_signal_quality(dce, &rssi, &ber);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "esp_modem_get_signal_quality failed with %d %s", err, esp_err_to_name(err));
            }
            ESP_LOGI(TAG, "Signal quality: rssi=%d, ber=%d", rssi, ber);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    reset_modem();

    /* Init and register system/core components */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &on_ip_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, &on_ppp_changed, NULL));

    /* Configure the PPP netif */
    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(APN);
    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();

    esp_netif_t *esp_netif = esp_netif_new(&netif_ppp_config);
    assert(esp_netif);
    esp_netif_set_default_netif(esp_netif);

    event_group = xEventGroupCreate();

    /* Configure the DTE */
    /* setup UART specific configuration based on kconfig options */
    dte_config.uart_config.port_num = UART_NUM_1;
    dte_config.uart_config.tx_io_num = EG915_UART_AT_TX;
    dte_config.uart_config.rx_io_num = EG915_UART_AT_RX;
    dte_config.uart_config.baud_rate = 115200;
    dte_config.uart_config.rx_buffer_size = 1024 * 2;
    dte_config.uart_config.tx_buffer_size = 1024 * 2;
    dte_config.uart_config.event_queue_size = 30;
    dte_config.task_stack_size = 4096 * 2;
    dte_config.task_priority = 20;
    dte_config.dte_buffer_size = 1024;

    ESP_LOGI(TAG, "Initializing Network module...");
    
    dce = esp_modem_new_dev(ESP_MODEM_DCE_EG91, &dte_config, &dce_config, esp_netif);
    assert(dce);

    xEventGroupClearBits(event_group, CONNECT_BIT | DISCONNECT_BIT);

    esp_err_t err;
    for (uint8_t i = 0; i < 5; i++)
    {
        err = esp_modem_sync(dce);
        if (err == ESP_OK)
        {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    if (err != ESP_OK)
    {
        wakeup_modem();
    }

    bool pin_status = false;
    err = esp_modem_read_pin(dce, &pin_status);
    while(err != ESP_OK)
    {
        err = esp_modem_read_pin(dce, &pin_status);
        ESP_LOGE(TAG, "CHIP SIM ERROR - esp_modem_read_pin failed with %d", err);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    int rssi, ber;
    err = esp_modem_get_signal_quality(dce, &rssi, &ber);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_modem_get_signal_quality failed with %d %s", err, esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Signal quality: rssi=%d, ber=%d", rssi, ber);

    err = esp_modem_set_mode(dce, ESP_MODEM_MODE_CMUX);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_modem_set_mode(ESP_MODEM_MODE_DATA) failed with %d", err);
        return;
    }
    /* Wait for IP address */
    ESP_LOGI(TAG, "Waiting for IP address");
    
    xTaskCreate(check_signal, "check_signal", 2048*2, NULL, 4, NULL);

    xEventGroupWaitBits(event_group, CONNECT_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    flag_connection = true;

    xEventGroupWaitBits(event_group, DISCONNECT_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    flag_connection = false;
    vTaskDelay(pdMS_TO_TICKS(5000));

    char imsi[32];
    err = esp_modem_get_imsi(dce, imsi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_modem_get_imsi failed with %d", err);
        return;
    }
    ESP_LOGI(TAG, "IMSI=%s", imsi);


    // UART DTE clean-up
    esp_modem_destroy(dce);
    esp_netif_destroy(esp_netif);
}
