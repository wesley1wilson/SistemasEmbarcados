#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h" 

static const char *TAG = "BIN_CNTR_OPT";

#define LED_PINS_NUM 4
static const gpio_num_t led_pins[LED_PINS_NUM] = { GPIO_NUM_41, GPIO_NUM_42, GPIO_NUM_2, GPIO_NUM_1 };
#define BUTTON_A_GPIO GPIO_NUM_14
#define BUTTON_B_GPIO GPIO_NUM_19
#define DEBOUNCE_MS   50 

#define BUTTON_A_PRESSED_NOTIF (1UL << 0)
#define BUTTON_B_PRESSED_NOTIF (1UL << 1)

static volatile uint8_t counter = 0;
static volatile uint8_t step    = 1;

static TimerHandle_t  debounce_timer_a = NULL;
static TimerHandle_t  debounce_timer_b = NULL;
static TaskHandle_t   app_main_task_handle = NULL;

static void gpio_init(void);
static void IRAM_ATTR button_isr_handler(void* arg);
static void debounce_timer_cb(TimerHandle_t xTimer);
static void update_leds(uint8_t val);

static void gpio_init(void)
{
    gpio_config_t io_conf = {0}; 

    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 0;
    for (int i = 0; i < LED_PINS_NUM; i++) {
        io_conf.pin_bit_mask |= (1ULL << led_pins[i]);
    }
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    io_conf.pin_bit_mask = (1ULL << BUTTON_A_GPIO) | (1ULL << BUTTON_B_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE; // Habilita pull-up interno
    io_conf.intr_type = GPIO_INTR_NEGEDGE;  // Interrupção na borda de descida
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Instala serviço de ISR e adiciona handlers para os botões
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_A_GPIO, button_isr_handler, (void*)(uintptr_t)BUTTON_A_GPIO));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_B_GPIO, button_isr_handler, (void*)(uintptr_t)BUTTON_B_GPIO));
}

static void IRAM_ATTR button_isr_handler(void* arg)
{
    gpio_num_t pin_num = (gpio_num_t)(uintptr_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (pin_num == BUTTON_A_GPIO) {
        if (debounce_timer_a != NULL && !xTimerIsTimerActive(debounce_timer_a)) { // Inicia timer A se não estiver ativo
            xTimerStartFromISR(debounce_timer_a, &xHigherPriorityTaskWoken);
        }
    } else if (pin_num == BUTTON_B_GPIO) {
        if (debounce_timer_b != NULL && !xTimerIsTimerActive(debounce_timer_b)) { // Inicia timer B se não estiver ativo
            xTimerStartFromISR(debounce_timer_b, &xHigherPriorityTaskWoken);
        }
    }

    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR(); // Cede CPU se uma tarefa de maior prioridade foi desbloqueada
    }
}

static void debounce_timer_cb(TimerHandle_t xTimer)
{
    gpio_num_t button_gpio_to_check;
    uint32_t notification_to_send;

    if (xTimer == debounce_timer_a) {
        button_gpio_to_check = BUTTON_A_GPIO;
        notification_to_send = BUTTON_A_PRESSED_NOTIF;
    } else if (xTimer == debounce_timer_b) {
        button_gpio_to_check = BUTTON_B_GPIO;
        notification_to_send = BUTTON_B_PRESSED_NOTIF;
    } else {
        return; 
    }

    if (gpio_get_level(button_gpio_to_check) == 0) { // Se o botão ainda estiver pressionado
        if (app_main_task_handle != NULL) {
            xTaskNotify(app_main_task_handle, notification_to_send, eSetBits); // Notifica tarefa principal
        }
    }
}

static void update_leds(uint8_t val)
{
    for (int i = 0; i < LED_PINS_NUM; i++) {
        gpio_set_level(led_pins[i], (val >> i) & 0x01); 
    }
}

void app_main(void)
{
    app_main_task_handle = xTaskGetCurrentTaskHandle();

    gpio_init(); 

    debounce_timer_a = xTimerCreate("TimerA", pdMS_TO_TICKS(DEBOUNCE_MS), pdFALSE, (void*)0, debounce_timer_cb);
    debounce_timer_b = xTimerCreate("TimerB", pdMS_TO_TICKS(DEBOUNCE_MS), pdFALSE, (void*)1, debounce_timer_cb);

    if (debounce_timer_a == NULL || debounce_timer_b == NULL) {
        ESP_LOGE(TAG, "Falha ao criar timers de debounce!");
        return; // Encerra se timers não puderam ser criados
    }

    update_leds(counter); 
    ESP_LOGI(TAG, "Setup completo. Aguardando eventos dos botões...");

    uint32_t notified_value;
    while (1) {
        // Aguarda notificação de um botão pressionado (após debounce)
        if (xTaskNotifyWait(0x00, ULONG_MAX, &notified_value, portMAX_DELAY) == pdPASS) {
            if (notified_value & BUTTON_A_PRESSED_NOTIF) {
                counter = (counter + step) & 0x0F; // Incrementa contador (máscara de 4 bits)
                ESP_LOGD(TAG, "Botão A: Contador = 0x%02X (%d)", counter, counter); 
                update_leds(counter);
            }

            if (notified_value & BUTTON_B_PRESSED_NOTIF) {
                step = (step == 1) ? 2 : 1; // Alterna passo
                ESP_LOGD(TAG, "Botão B: Passo = +%d", step); 
            }
        }
    }
}
