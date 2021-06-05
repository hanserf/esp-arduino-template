#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <encoder.h>
#include <esp_idf_lib_helpers.h>
#include <esp_log.h>
#include <my_encoder.h>

// Connect common encoder pin to ground
#if HELPER_TARGET_IS_ESP8266
#define RE_A_GPIO   14
#define RE_B_GPIO   12
#define RE_BTN_GPIO 13

#elif HELPER_TARGET_IS_ESP32
#define RE_A_GPIO   16
#define RE_B_GPIO   17
#define RE_BTN_GPIO 5

#else
#error Unknown platform
#endif

#define EV_QUEUE_LEN 5

static const char *TAG = "encoder_example";

static struct {
    my_encoder_callback_t event_callback;
    QueueHandle_t event_queue;
    rotary_encoder_t re;

}MY_ENC;

/*This task only runs in response to an encoder interrupt*/
void encoder_task(void *arg)
{
    // Create event queue for rotary encoders
    event_queue = xQueueCreate(EV_QUEUE_LEN, sizeof(rotary_encoder_event_t));

    // Setup rotary encoder library
    ESP_ERROR_CHECK(rotary_encoder_init(event_queue));

    // Add one encoder
    memset(&re, 0, sizeof(rotary_encoder_t));
    re.pin_a = RE_A_GPIO;
    re.pin_b = RE_B_GPIO;
    re.pin_btn = RE_BTN_GPIO;
    ESP_ERROR_CHECK(rotary_encoder_add(&re));

    rotary_encoder_event_t e;
    int32_t val = 0;

    ESP_LOGI(TAG, "Initial value: %d", val);
    while (1)
    {
        xQueueReceive(event_queue, &e, portMAX_DELAY);

        switch (e.type)
        {
            case RE_ET_BTN_PRESSED:
                ESP_LOGI(TAG, "Button pressed");
                break;
            case RE_ET_BTN_RELEASED:
                ESP_LOGI(TAG, "Button released");
                break;
            case RE_ET_BTN_CLICKED:
                ESP_LOGI(TAG, "Button clicked");
                break;
            case RE_ET_BTN_LONG_PRESSED:
                ESP_LOGI(TAG, "Looooong pressed button");
                break;
            case RE_ET_CHANGED:
                val += e.diff;
                ESP_LOGI(TAG, "Value = %d", val);
                break;
            default:
                break;
        }
    }
}

void init_encoder(int pina,int pinb, int pin_btn,my_encoder_callback_t encoder_event_callback)
{
    xTaskCreate(test, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}
