#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define BTN_INC    GPIO_NUM_14   // incrementar
#define BTN_DEC    GPIO_NUM_19   // decrementar

#define NUM_LEDS 4
const gpio_num_t LED_PINS[NUM_LEDS] = {
    GPIO_NUM_1,   // LED1 
    GPIO_NUM_2,   // LED2
    GPIO_NUM_42,  // LED3
    GPIO_NUM_41   // LED4 
};

static int count = 0;

static void gpio_init(void)
{
    gpio_config_t btn_conf = {
        .pin_bit_mask   = (1ULL<<BTN_INC) | (1ULL<<BTN_DEC),
        .mode           = GPIO_MODE_INPUT,
        .pull_up_en     = GPIO_PULLUP_ENABLE,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_conf);

    gpio_config_t led_conf = {
        .pin_bit_mask   = 0,
        .mode           = GPIO_MODE_OUTPUT,
        .pull_up_en     = GPIO_PULLUP_DISABLE,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_DISABLE
    };
    for (int i = 0; i < NUM_LEDS; i++) {
        led_conf.pin_bit_mask |= (1ULL<<LED_PINS[i]);
    }
    gpio_config(&led_conf);
}

static void update_leds(int value)
{
    for (int i = 0; i < NUM_LEDS; i++) {
        gpio_set_level(LED_PINS[i], (value >> i) & 0x1);
    }
}

void app_main(void)
{
    gpio_init();
    update_leds(count);

    while (1) {
        if (gpio_get_level(BTN_INC) == 0) {
            count = (count + 1) & 0x0F;   // de 0 a 15 em módulo 16
            update_leds(count);
            vTaskDelay(pdMS_TO_TICKS(200));  
        }
        if (gpio_get_level(BTN_DEC) == 0) {
            count = (count - 1) & 0x0F;
            update_leds(count);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        vTaskDelay(pdMS_TO_TICKS(10));  
    }
}
