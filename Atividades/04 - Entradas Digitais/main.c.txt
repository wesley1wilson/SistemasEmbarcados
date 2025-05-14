#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "BIN_CNTR";

#define LED_PINS_NUM 4
static const gpio_num_t led_pins[LED_PINS_NUM] = {
    GPIO_NUM_41, 
    GPIO_NUM_42, 
    GPIO_NUM_2,  
    GPIO_NUM_1   
};

#define BUTTON_A_GPIO GPIO_NUM_14  // incrementa (+1 ou +2)
#define BUTTON_B_GPIO GPIO_NUM_19  // alterna passo

#define DEBOUNCE_MS    50          // ms para debounce

static volatile uint8_t counter = 0;
static volatile uint8_t step    = 1;

static TimerHandle_t  debounce_timer_a;
static TimerHandle_t  debounce_timer_b;
static volatile bool  flag_inc    = false;
static volatile bool  flag_toggle = false;

static void IRAM_ATTR button_isr_handler(void* arg);
static void debounce_timer_cb(TimerHandle_t t);

static void gpio_init(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 0,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    for (int i = 0; i < LED_PINS_NUM; i++) {
        io_conf.pin_bit_mask |= (1ULL << led_pins[i]);
    }
    gpio_config(&io_conf);

    // Botões como entrada com pull‑up interno, interrupção em borda de descida
    io_conf.pin_bit_mask = (1ULL<<BUTTON_A_GPIO) | (1ULL<<BUTTON_B_GPIO);
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_NEGEDGE;
    gpio_config(&io_conf);

    // Instala handler de ISR
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_A_GPIO, button_isr_handler, (void*) BUTTON_A_GPIO);
    gpio_isr_handler_add(BUTTON_B_GPIO, button_isr_handler, (void*) BUTTON_B_GPIO);
}

// --- ISR dos botões: só dispara o timer de debounce ---
static void IRAM_ATTR button_isr_handler(void* arg)
{
    gpio_num_t pin = (gpio_num_t)(uint32_t)arg;
    if (pin == BUTTON_A_GPIO) {
        if (!xTimerIsTimerActive(debounce_timer_a)) {
            xTimerStartFromISR(debounce_timer_a, NULL);
        }
    } else {
        if (!xTimerIsTimerActive(debounce_timer_b)) {
            xTimerStartFromISR(debounce_timer_b, NULL);
        }
    }
}

// --- callback de debounce: faz leitura filtrada e seta flags ---
static void debounce_timer_cb(TimerHandle_t t)
{
    if (t == debounce_timer_a) {
        if (gpio_get_level(BUTTON_A_GPIO) == 0) {
            flag_inc = true;
        }
        xTimerStop(debounce_timer_a, 0);
    } else {
        if (gpio_get_level(BUTTON_B_GPIO) == 0) {
            flag_toggle = true;
        }
        xTimerStop(debounce_timer_b, 0);
    }
}

// --- atualiza LEDs conforme valor de counter ---
static void update_leds(uint8_t val)
{
    for (int i = 0; i < LED_PINS_NUM; i++) {
        gpio_set_level(led_pins[i], (val >> i) & 0x1);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Inicializando GPIOs e timers...");
    gpio_init();

    // cria timers de debounce (one-shot)
    debounce_timer_a = xTimerCreate("dbA",
                          pdMS_TO_TICKS(DEBOUNCE_MS),
                          pdFALSE, NULL,
                          debounce_timer_cb);
    debounce_timer_b = xTimerCreate("dbB",
                          pdMS_TO_TICKS(DEBOUNCE_MS),
                          pdFALSE, NULL,
                          debounce_timer_cb);

    ESP_LOGI(TAG, "Entrando no loop principal...");
    update_leds(counter);

    while (1) {
        // Alterna passo entre 1 e 2
        if (flag_toggle) {
            flag_toggle = false;
            step = (step == 1) ? 2 : 1;
            ESP_LOGI(TAG, "Passo alterado para +%d", step);
        }

        // Incrementa contador com máscara de 4 bits
        if (flag_inc) {
            flag_inc = false;
            counter = (counter + step) & 0x0F;
            ESP_LOGI(TAG, "Contador = 0x%X (%d)", counter, counter);
            update_leds(counter);
        }

        // alivia CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
