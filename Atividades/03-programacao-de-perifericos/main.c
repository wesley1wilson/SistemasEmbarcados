#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

typedef struct {
    gpio_num_t gpio;
    TickType_t delay;
} led_t;

void blink(void *arg) {
    led_t *led = (led_t *)arg;
    gpio_reset_pin(led->gpio);
    gpio_set_direction(led->gpio, GPIO_MODE_OUTPUT);
    while (1) {
        gpio_set_level(led->gpio, 1);
        vTaskDelay(led->delay);
        gpio_set_level(led->gpio, 0);
        vTaskDelay(led->delay);
    }
}

void app_main(void) {
    static led_t leds[] = {
        {GPIO_NUM_5, pdMS_TO_TICKS(1000)},
        {GPIO_NUM_4, pdMS_TO_TICKS(200)}
    };
    for (int i = 0; i < sizeof(leds)/sizeof(leds[0]); i++) {
        xTaskCreate(blink, "", 1024, &leds[i], 5, NULL);
    }
}
