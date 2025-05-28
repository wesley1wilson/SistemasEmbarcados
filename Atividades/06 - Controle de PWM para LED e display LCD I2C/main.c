#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"

#include "esp_log.h"
#include "esp_err.h"

#include "int_i2c.h" 

// LEDs Contador Binário
#define LED1_GPIO GPIO_NUM_1
#define LED2_GPIO GPIO_NUM_2
#define LED3_GPIO GPIO_NUM_42
#define LED4_GPIO GPIO_NUM_41
const gpio_num_t binary_led_gpios[] = {LED1_GPIO, LED2_GPIO, LED3_GPIO, LED4_GPIO};

// LED PWM
#define PWM_LED_GPIO GPIO_NUM_39
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY          (5000)

// I2C LCD
#define I2C_MASTER_SCL_IO    GPIO_NUM_12
#define I2C_MASTER_SDA_IO    GPIO_NUM_13
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000
#define LCD_ADDRESS          0x27
#define LCD_DISPLAY_TYPE     DISPLAY_16X02

// Botões
#define BUTTON_INC_GPIO      GPIO_NUM_14
#define BUTTON_DEC_GPIO      GPIO_NUM_19

// --- Variáveis Globais ---
lcd_i2c_handle_t lcd_handle;
static const char *TAG = "MAIN_APP";

static bool last_button_inc_state = true;
static bool last_button_dec_state = true;
static TickType_t last_inc_press_time = 0;
static TickType_t last_dec_press_time = 0;
const TickType_t debounce_delay = pdMS_TO_TICKS(50);


// Funções de Inicialização e Controle
static void binary_leds_init(void) {
    for (int i = 0; i < sizeof(binary_led_gpios)/sizeof(gpio_num_t); i++) {
        gpio_reset_pin(binary_led_gpios[i]);
        gpio_set_direction(binary_led_gpios[i], GPIO_MODE_OUTPUT);
        gpio_set_level(binary_led_gpios[i], 0);
    }
    ESP_LOGI(TAG, "LEDs do contador binário inicializados.");
}

void set_binary_leds(uint8_t value) {
    for (int i = 0; i < sizeof(binary_led_gpios)/sizeof(gpio_num_t); i++) {
        if ((value >> i) & 0x01) {
            gpio_set_level(binary_led_gpios[i], 1);
        } else {
            gpio_set_level(binary_led_gpios[i], 0);
        }
    }
}

static void pwm_led_init(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWM_LED_GPIO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_LOGI(TAG, "LED PWM Inicializado no GPIO %d", PWM_LED_GPIO);
}

void set_pwm_led_brightness(uint8_t percentage) {
    if (percentage > 100) percentage = 100;
    uint32_t duty = (percentage * ((1 << LEDC_DUTY_RES) - 1)) / 100;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

static void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
    ESP_LOGI(TAG, "Mestre I2C inicializado. SDA: %d, SCL: %d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
}

static void buttons_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    io_conf.pin_bit_mask = (1ULL << BUTTON_INC_GPIO);
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << BUTTON_DEC_GPIO);
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "Botões inicializados. INC: GPIO %d, DEC: GPIO %d", BUTTON_INC_GPIO, BUTTON_DEC_GPIO);
}


// --- Função Principal app_main ---
void app_main(void) {
    binary_leds_init();
    pwm_led_init();
    buttons_init();
    i2c_master_init();

    lcd_handle.address = LCD_ADDRESS;
    lcd_handle.num = I2C_MASTER_NUM;
    lcd_handle.backlight = 1;
    lcd_handle.size = LCD_DISPLAY_TYPE;
    lcd_i2c_init(&lcd_handle);
    ESP_LOGI(TAG, "LCD Inicializado no endereço 0x%X", lcd_handle.address);

    lcd_i2c_print(&lcd_handle, "ESP32-S3 Ativ.6");
    lcd_i2c_cursor_set(&lcd_handle, 0, 1);
    lcd_i2c_print(&lcd_handle, "Contador & PWM");
    vTaskDelay(pdMS_TO_TICKS(1500));

    uint8_t counter = 0;
    uint8_t pwm_brightness = 0;
    uint8_t prev_counter = 0xFF;
    uint8_t prev_pwm_brightness = 0xFF;

    char lcd_buffer[17];

    last_button_inc_state = gpio_get_level(BUTTON_INC_GPIO);
    last_button_dec_state = gpio_get_level(BUTTON_DEC_GPIO);

    pwm_brightness = (counter * 100) / 15;
    set_pwm_led_brightness(pwm_brightness);


    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        bool counter_changed = false;
        bool pwm_value_for_lcd_changed = false;

        // Lógica dos Botões com Contador Circular 
        bool current_button_inc_state_val = gpio_get_level(BUTTON_INC_GPIO);
        if (!current_button_inc_state_val && last_button_inc_state) {
            if ((current_time - last_inc_press_time) > debounce_delay) {
                counter++;
                if (counter > 15) { // Se ultrapassar 15, volta para 0
                    counter = 0;
                }
                counter_changed = true;
                ESP_LOGD(TAG, "Botão INC. Contador: %d", counter);
                last_inc_press_time = current_time;
            }
        }
        last_button_inc_state = current_button_inc_state_val;

        bool current_button_dec_state_val = gpio_get_level(BUTTON_DEC_GPIO);
        if (!current_button_dec_state_val && last_button_dec_state) {
            if ((current_time - last_dec_press_time) > debounce_delay) {
                if (counter == 0) { // Se estiver em 0 e decrementar, vai para 15
                    counter = 15;
                } else {
                    counter--;
                }
                counter_changed = true;
                ESP_LOGD(TAG, "Botão DEC. Contador: %d", counter);
                last_dec_press_time = current_time;
            }
        }
        last_button_dec_state = current_button_dec_state_val;

        // Atualizações se o contador mudou 
        if (counter_changed) {
            set_binary_leds(counter);

            uint8_t new_pwm_brightness = (counter * 100) / 15;
            if (pwm_brightness != new_pwm_brightness) {
                pwm_brightness = new_pwm_brightness;
                pwm_value_for_lcd_changed = true;
            }
            set_pwm_led_brightness(pwm_brightness);
        }

        // Atualizações Condicionais do LCD
        if (counter_changed || prev_counter == 0xFF) {
            lcd_i2c_cursor_set(&lcd_handle, 0, 0);
            snprintf(lcd_buffer, sizeof(lcd_buffer), "Cont: %02u (0-15)", counter);
            lcd_i2c_print(&lcd_handle, lcd_buffer);
            prev_counter = counter;
        }

        if (pwm_value_for_lcd_changed || prev_pwm_brightness == 0xFF) {
            lcd_i2c_cursor_set(&lcd_handle, 0, 1);
            snprintf(lcd_buffer, sizeof(lcd_buffer), "PWM: %3u%%      ", pwm_brightness);
            lcd_i2c_print(&lcd_handle, lcd_buffer);
            prev_pwm_brightness = pwm_brightness;
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}