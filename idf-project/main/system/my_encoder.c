#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <encoder.h>
#include <esp_idf_lib_helpers.h>
#include <esp_log.h>
#include <my_encoder.h>

// Connect common encoder pin to ground

#define EV_QUEUE_LEN 5

static const char *TAG = "my_encoder";

static struct {
    my_encoder_callback_t event_callback;
    QueueHandle_t event_queue;
    rotary_encoder_t re;
    TaskHandle_t task;

}MY_ENC;

static inline void __set_bit(int32_t *x, int bitNum) {
    *x |= (1L << bitNum);
}
static inline void __clear_bit(int32_t *x, int bitNum) {
    *x &= ~(1 << (bitNum));
}


/*This task only runs in response to an encoder interrupt*/
void encoder_task(void *arg)
{
    
    rotary_encoder_event_t e;
    int32_t val = 0;
    int32_t ctl = 0;
    
    bool valid; 
    ESP_LOGI(TAG, "Initial value: %d", val);
    while (1)
    {
        ESP_ERROR_CHECK((xQueueReceive(MY_ENC.event_queue, &e, portMAX_DELAY) == pdPASS)?ESP_OK:ESP_FAIL);
        valid = true;
        switch (e.type)
        {
            case RE_ET_BTN_PRESSED:
                __set_bit(&val,ROTINC_MSB);
                ESP_LOGI(TAG, "Button pressed");
                break;
            case RE_ET_BTN_RELEASED:
                __clear_bit(&val,ROTINC_LONG);
                __clear_bit(&val,ROTINC_MSB);
                ESP_LOGI(TAG, "Button released");
                break;
            case RE_ET_BTN_CLICKED:
                __set_bit(&val,ROTINC_MSB);
                ESP_LOGI(TAG, "Button clicked");
                break;
            case RE_ET_BTN_LONG_PRESSED:
                __set_bit(&val,ROTINC_LONG);
                ESP_LOGI(TAG, "Looooong pressed button");
                break;
            case RE_ET_CHANGED:
                ctl += e.diff;
                val |= ctl;
                ESP_LOGI(TAG, "Value = %d", val);
                break;
            default:
                valid = false;
                break;
        }
        if(valid){
            if(MY_ENC.event_callback != NULL){
                MY_ENC.event_callback(e.type,val);
            }
        }
    }
}

void init_encoder(int pina,int pinb, int pin_btn,my_encoder_callback_t encoder_event_callback)
{
    static StaticTask_t tcb;
    static StackType_t stack[ configMINIMAL_STACK_SIZE * 8 ];
    const uint32_t stack_size = ( sizeof( stack ) / sizeof( stack[ 0 ] ) );
    // Create event queue for rotary encoders
    if(MY_ENC.event_queue == NULL){
        MY_ENC.event_queue = xQueueCreate(EV_QUEUE_LEN, sizeof(rotary_encoder_event_t));
        ESP_ERROR_CHECK(rotary_encoder_init(MY_ENC.event_queue));
    }
    // Setup rotary encoder library
    // Add one encoder
    memset(&MY_ENC.re, 0, sizeof(rotary_encoder_t));
    MY_ENC.re.pin_a = pina;
    MY_ENC.re.pin_b = pinb;
    MY_ENC.re.pin_btn = pin_btn;
    ESP_ERROR_CHECK(rotary_encoder_add(&MY_ENC.re));
    /*Camera task is dedicated to run on core #1, it has high priority 7 */
    MY_ENC.task = xTaskCreateStaticPinnedToCore(& encoder_task, (const char *) TAG, stack_size, NULL, 5, stack, &tcb,0);
    ESP_ERROR_CHECK((MY_ENC.task != NULL) ? ESP_OK : ESP_FAIL);
}

void destroy_encoder(){
    vTaskDelete(MY_ENC.task);
    ESP_ERROR_CHECK(rotary_encoder_remove(&MY_ENC.re));
}
