#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"

// Inclusões para SD Card
#include "driver/spi_master.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "int_i2c.h"

// --- Definições de Pinos e Parâmetros ---
#define NTC_ADC_CHANNEL         ADC_CHANNEL_3
#define NTC_NOMINAL_TEMPERATURE 25.0f
#define NTC_BETA_COEFFICIENT    3950.0f
#define LED1_GPIO               GPIO_NUM_39
#define LED2_GPIO               GPIO_NUM_40
#define LED3_GPIO               GPIO_NUM_41
#define LED4_GPIO               GPIO_NUM_42
#define BUZZER_GPIO             GPIO_NUM_38
#define BTN_A_GPIO              GPIO_NUM_14
#define BTN_B_GPIO              GPIO_NUM_19
#define I2C_MASTER_SDA_IO       GPIO_NUM_8
#define I2C_MASTER_SCL_IO       GPIO_NUM_9
#define LCD_ADDR                0x27
#define SPI_MOSI_PIN            GPIO_NUM_36
#define SPI_MISO_PIN            GPIO_NUM_45
#define SPI_SCLK_PIN            GPIO_NUM_35
#define SD_CS_PIN               GPIO_NUM_37
#define MOUNT_POINT             "/sdcard"
#define LEDC_TIMER_PWM          LEDC_TIMER_0
#define LEDC_MODE_PWM           LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_PWM        LEDC_CHANNEL_0
#define LEDC_DUTY_RES_PWM       LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY_PWM      2000
#define BUZZER_DUTY_ON          (511)
#define BUZZER_DUTY_OFF         (0)
#define DEBOUNCE_TIME_MS        150
#define I2C_MASTER_NUM_USER     I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000
#define BLINK_INTERVAL_MS       250

// --- Estados da Máquina ---
typedef enum {
    STATE_INIT,
    STATE_READ_TEMP,
    STATE_LOG_SD,
    STATE_CHECK_ALARM,
    STATE_UPDATE_LEDS,
    STATE_UPDATE_LCD,
    STATE_WAIT,
} system_state_t;

// --- Variáveis Globais ---
static const char *TAG = "DAQ_ATIV9";
volatile float current_ntc_temp = 0.0f;
volatile int alarm_temperature = 25;
volatile bool alarm_active = false;
volatile bool leds_blinking = false;
static bool sd_card_mounted = false;
static bool blink_state = false;
static TickType_t last_blink_toggle_time = 0;
static QueueHandle_t button_evt_queue = NULL;
static uint64_t last_interrupt_time_btnA = 0;
static uint64_t last_interrupt_time_btnB = 0;
static adc_oneshot_unit_handle_t adc_unit_handle;
static lcd_i2c_handle_t lcd_display_handle;
static system_state_t current_state = STATE_INIT;

// --- Prototipação de Funções ---
static void i2c_bus_init(void);
static void app_lcd_init(void);
static void adc_init(void);
static void pwm_buzzer_init(void);
static void gpios_init(void);
static void sd_card_init(void);
float read_ntc_temperature(void);
void set_buzzer_state(bool on);
static void button_isr_generic_handler(void* arg);
static void button_task(void* arg);
void update_leds_proximity_state(void);
void handle_led_blinking_alarm(void);
void log_temperature_to_sd(float temp);
static void state_machine_tick(void);

// --- Implementação das Funções ---
static void i2c_bus_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM_USER, &conf);
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM_USER, conf.mode, 0, 0, 0));
}

static void app_lcd_init(void) {
    lcd_display_handle.address = LCD_ADDR;
    lcd_display_handle.i2c_port_num = I2C_MASTER_NUM_USER;
    lcd_display_handle.backlight_on = 1;
    lcd_display_handle.display_size = DISPLAY_16X02;
    lcd_i2c_init(&lcd_display_handle);
    ESP_LOGI(TAG, "Display LCD I2C inicializado.");
}

static void adc_init(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_unit_handle));
    adc_oneshot_chan_cfg_t chan_cfg = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_unit_handle, NTC_ADC_CHANNEL, &chan_cfg));
}

static void pwm_buzzer_init(void) {
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_MODE_PWM,
        .timer_num = LEDC_TIMER_PWM,
        .duty_resolution = LEDC_DUTY_RES_PWM,
        .freq_hz = LEDC_FREQUENCY_PWM,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));
    ledc_channel_config_t channel_cfg = {
        .speed_mode = LEDC_MODE_PWM,
        .channel = LEDC_CHANNEL_PWM,
        .timer_sel = LEDC_TIMER_PWM,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BUZZER_GPIO,
        .duty = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
}

static void gpios_init(void) {
    gpio_config_t io_conf = {};
    // LEDs
    io_conf.pin_bit_mask = (1ULL<<LED1_GPIO)|(1ULL<<LED2_GPIO)|(1ULL<<LED3_GPIO)|(1ULL<<LED4_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);
    // Botões
    io_conf.pin_bit_mask = (1ULL<<BTN_A_GPIO)|(1ULL<<BTN_B_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    ESP_LOGI(TAG, "GPIOs inicializados.");
}

static void sd_card_init(void) {
    esp_err_t ret;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) return;
    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = SD_CS_PIN;
    slot_cfg.host_id = host.slot;
    esp_vfs_fat_sdmmc_mount_config_t mnt_cfg = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t* card;
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_cfg, &mnt_cfg, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao montar SD card (%s)", esp_err_to_name(ret));
        sd_card_mounted = false;
    } else {
        sd_card_mounted = true;
        ESP_LOGI(TAG, "SD card montado em %s", MOUNT_POINT);
        sdmmc_card_print_info(stdout, card);
    }
    if (ret == ESP_OK) {
        sd_card_mounted = true;
        ESP_LOGI(TAG, "SD card montado.");
    }
}

float read_ntc_temperature(void) {
    int raw;
    if (adc_oneshot_read(adc_unit_handle, NTC_ADC_CHANNEL, &raw) != ESP_OK) return -273.15f;
    const float max = 4095.0f;
    float r_div = (float)raw / (max - (float)raw);
    float log_r = logf(r_div);
    float t0 = NTC_NOMINAL_TEMPERATURE + 273.15f;
    float tk = 1.0f / ((1.0f/t0) + (1.0f/NTC_BETA_COEFFICIENT)*log_r);
    return tk - 273.15f;
}

void set_buzzer_state(bool on) {
    ledc_set_duty(LEDC_MODE_PWM, LEDC_CHANNEL_PWM, on ? BUZZER_DUTY_ON : BUZZER_DUTY_OFF);
    ledc_update_duty(LEDC_MODE_PWM, LEDC_CHANNEL_PWM);
}

static void IRAM_ATTR button_isr_generic_handler(void* arg) {
    uint32_t gpio = (uint32_t)arg;
    uint64_t now = esp_timer_get_time();
    uint64_t* last = (gpio==BTN_A_GPIO)?&last_interrupt_time_btnA:&last_interrupt_time_btnB;
    if (now - *last > DEBOUNCE_TIME_MS*1000) {
        *last = now;
        xQueueSendFromISR(button_evt_queue, &gpio, NULL);
    }
}

static void button_task(void* arg) {
    uint32_t io;
    while (1) {
        if (xQueueReceive(button_evt_queue, &io, portMAX_DELAY)) {
            if (io == BTN_A_GPIO) {
                alarm_temperature += 5;
            } else {
                alarm_temperature = (alarm_temperature >= 5) ? alarm_temperature - 5 : 0;
            }
        }
    }
}

void update_leds_proximity_state(void) {
    float diff = alarm_temperature - current_ntc_temp;
    bool l1=0,l2=0,l3=0,l4=0;
    if      (diff <= 2.0f && diff > 0)      { l1=1; l2=1; l3=1; l4=1; }
    else if (diff <= 10.0f && diff > 0)     { l1=1; l2=1; l3=1; }
    else if (diff <= 15.0f && diff > 0)     { l1=1; l2=1; }
    else if (diff <= 20.0f && diff > 0)     { l1=1; }
    gpio_set_level(LED1_GPIO, l1);
    gpio_set_level(LED2_GPIO, l2);
    gpio_set_level(LED3_GPIO, l3);
    gpio_set_level(LED4_GPIO, l4);
}

void handle_led_blinking_alarm(void) {
    if ((xTaskGetTickCount() - last_blink_toggle_time)*portTICK_PERIOD_MS >= BLINK_INTERVAL_MS) {
        blink_state = !blink_state;
        gpio_set_level(LED1_GPIO, blink_state);
        gpio_set_level(LED2_GPIO, blink_state);
        gpio_set_level(LED3_GPIO, blink_state);
        gpio_set_level(LED4_GPIO, blink_state);
        last_blink_toggle_time = xTaskGetTickCount();
    }
}

void log_temperature_to_sd(float temp) {
    if (!sd_card_mounted) return;
    FILE* f = fopen(MOUNT_POINT "/datalog.txt", "a");
    if (!f) return;
    fprintf(f, "%.2f\n", temp);
    fclose(f);
}

static void state_machine_tick(void) {
    static TickType_t last_lcd = 0;
    static float last_t = -1000;
    static int   last_a = -1000;
    switch (current_state) {
        case STATE_INIT:
            gpios_init();
            adc_init();
            i2c_bus_init();
            app_lcd_init();
            pwm_buzzer_init();
            sd_card_init();
            button_evt_queue = xQueueCreate(2, sizeof(uint32_t));
            gpio_isr_handler_add(BTN_A_GPIO, button_isr_generic_handler, (void*)BTN_A_GPIO);
            gpio_isr_handler_add(BTN_B_GPIO, button_isr_generic_handler, (void*)BTN_B_GPIO);
            xTaskCreate(button_task, "button_task", 3072, NULL, 10, NULL);
            current_state = STATE_READ_TEMP;
            break;
        case STATE_READ_TEMP:
            current_ntc_temp = read_ntc_temperature();
            current_state = STATE_LOG_SD;
            break;
        case STATE_LOG_SD:
            log_temperature_to_sd(current_ntc_temp);
            current_state = STATE_CHECK_ALARM;
            break;
        case STATE_CHECK_ALARM: {
            bool new_a = (current_ntc_temp > alarm_temperature);
            if (new_a != alarm_active) {
                alarm_active = new_a;
                set_buzzer_state(alarm_active);
                ESP_LOGI(TAG, "Alarme sonoro %s.", alarm_active?"ATIVADO":"DESATIVADO");
            }
            current_state = STATE_UPDATE_LEDS;
            break;
        }
        case STATE_UPDATE_LEDS:
            if (alarm_active) handle_led_blinking_alarm();
            else update_leds_proximity_state();
            current_state = STATE_UPDATE_LCD;
            break;
        case STATE_UPDATE_LCD: {
            TickType_t now = xTaskGetTickCount();
            if (fabsf(current_ntc_temp - last_t) > 0.05f
             || alarm_temperature != last_a
             || (now - last_lcd)*portTICK_PERIOD_MS > 1000) {
                char buf[17];
                snprintf(buf, sizeof(buf), "NTC: %.1fC", current_ntc_temp);
                lcd_i2c_cursor_set(&lcd_display_handle, 0, 0);
                lcd_i2c_print_str(&lcd_display_handle, buf);
                snprintf(buf, sizeof(buf), "Alarme: %dC", alarm_temperature);
                lcd_i2c_cursor_set(&lcd_display_handle, 0, 1);
                lcd_i2c_print_str(&lcd_display_handle, buf);
                last_t = current_ntc_temp;
                last_a = alarm_temperature;
                last_lcd = now;
            }
            current_state = STATE_WAIT;
            break;
        }
        case STATE_WAIT:
            vTaskDelay(pdMS_TO_TICKS(100));
            current_state = STATE_READ_TEMP;
            break;
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Atividade 09: Máquina de Estados iniciada");
    while (1) {
        state_machine_tick();
    }
}
