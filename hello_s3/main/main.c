#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "hello_s3";

void app_main(void)
{
    int counter = 0;

    ESP_LOGI(TAG, "bring-up test started");

    while (1) {
        ESP_LOGI(TAG, "hello from ESP32-S3, counter=%d", counter++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
