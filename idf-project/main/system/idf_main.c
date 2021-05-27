/* Wi-Fi Provisioning Manager Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_event_loop.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"

#include "idf_main.h"
#include "button.h"
#include "softap_otaserver.h"


static const char *TAG = "app";

static const char *MY_SSID = EXAMPLE_ESP_WIFI_SSID;
static uint8_t my_mac[6];

static void wifi_init_softap();
static void init_deepsleep();
static void check_button(void * pvArgs);
static void event_handler(void* arg, esp_event_base_t event_base,
                          int event_id, void* event_data);


/* Event handler for catching system events */
static void event_handler(void* arg, esp_event_base_t event_base, int event_id, void* event_data){
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",MAC2STR(event->mac), event->aid);        
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",MAC2STR(event->mac), event->aid);

    }
}

static void wifi_init_softap()
{

    char ssid_buf[32];
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_efuse_mac_get_default(my_mac));
    ESP_LOGI(TAG,"MAC ADDR:%x:%x:%x:%x:%x:%x",my_mac[0],my_mac[1],my_mac[2],my_mac[3],my_mac[4],my_mac[5]);

    sprintf(ssid_buf,"%s%x:%x:%x:%x:%x:%x",MY_SSID,my_mac[0],my_mac[1],my_mac[2],my_mac[3],my_mac[4],my_mac[5]);
    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = strlen(ssid_buf),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    strcpy((char *)wifi_config.sta.ssid,(char *)ssid_buf);
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             ssid_buf, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}


void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition was truncated
         * and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());

        /* Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    /* Initialize TCP/IP */
    tcpip_adapter_init();
    /* Initialize Wi-Fi */
    wifi_init_softap();
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    start_ota_http();
    xTaskCreate(&check_button,"BTN_TASK",configMINIMAL_STACK_SIZE*3,NULL,9,NULL);
}



// interrupt service routine, called when the button is pressed and distribute event to system
static void check_button(void * pvArgs){
    QueueHandle_t btn_event = button_init(PIN_BIT(GPIO_BTN));
    rtc_gpio_deinit(GPIO_BTN);
    button_event_t ev;
    bool held = false;
    while(true){
        /*Do blocking check to se if button has been pressed and released */
        if (xQueueReceive(btn_event, &ev, portMAX_DELAY)){
            if (ev.pin == GPIO_BTN){
                switch (ev.event)
                {
                case BUTTON_DOWN:                    
		break;
                case BUTTON_HELD:
                    held = true;
                    break;
                case BUTTON_UP:
                    if(held){
                        held = false;
                        /* Deinitialize everything and put to sleep */
                        init_deepsleep();
                    }
                    else{
                       start_ota_http();
                    }
                    break;
                default:
                    break;
                }
            }
        }
    }
}

static void init_deepsleep(){
    esp_wifi_stop();
    esp_wifi_deinit();
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(GPIO_BTN, 0));
    esp_deep_sleep_start();

}

