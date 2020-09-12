#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

extern "C" [[noreturn]] void app_main();

[[noreturn]] void app_main() {
    vTaskDelay(pdMS_TO_TICKS(1000));

    for (;;) {
        ESP_LOGI("Tag", "Hello World!!!");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
