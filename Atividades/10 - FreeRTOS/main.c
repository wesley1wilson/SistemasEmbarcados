#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "int_i2c.h"

#define NTC_ADC_CHANNEL         ADC_CHANNEL_3 
#define NTC_ADC_PIN             GPIO_NUM_4
#define NTC_NOMINAL_TEMPERATURE 25.0f
#define NTC_BETA_COEFFICIENT    3950.0f

#define SEG_A_GPIO              GPIO_NUM_2
#define SEG_B_GPIO              GPIO_NUM_42
#define SEG_C_GPIO              GPIO_NUM_41
#define SEG_D_GPIO              GPIO_NUM_40
#define SEG_E_GPIO              GPIO_NUM_39
#define SEG_F_GPIO              GPIO_NUM_1
#define SEG_G_GPIO              GPIO_NUM_47

#define BUZZER_GPIO             GPIO_NUM_38
#define BTN_A_GPIO              GPIO_NUM_14
#define BTN_B_GPIO              GPIO_NUM_19
#define I2C_MASTER_SDA_IO       GPIO_NUM_8
#define I2C_MASTER_SCL_IO       GPIO_NUM_9
#define LCD_ADDR                0x27
#define I2C_MASTER_NUM_USER     I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000

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
#define BLINK_INTERVAL_MS       250

static const char *TAG = "FreeRTOS_DAQ_ATIV10";

static float current_ntc_temp = 0.0f;
static int alarm_temperature = 25;
static bool alarm_active = false;
static bool sd_card_mounted = false;

static QueueHandle_t button_evt_queue = NULL;
static SemaphoreHandle_t xDataMutex = NULL; 

static adc_oneshot_unit_handle_t adc_unit_handle;
static lcd_i2c_handle_t lcd_display_handle;

static void i2c_bus_init(void);
static void app_lcd_init(void);
static void adc_init(void);
static void pwm_buzzer_init(void);
static void gpios_init(void);
static void sd_card_init(void);
float read_ntc_temperature_task(void);
void set_buzzer_state(bool on);
void display_7seg(char digit);
void log_temperature_to_sd(float temp);
static void button_task(void* arg);
static void sensor_task(void* arg);
static void alarm_logic_task(void* arg);
static void lcd_task(void* arg);
static void sd_card_task(void* arg);
static void display_7seg_task(void* arg);
static void button_isr_handler(void* arg);


static void i2c_bus_init(void) {
    i2c_config_t conf = { .mode = I2C_MODE_MASTER, .sda_io_num = I2C_MASTER_SDA_IO, .scl_io_num = I2C_MASTER_SCL_IO, .sda_pullup_en = GPIO_PULLUP_ENABLE, .scl_pullup_en = GPIO_PULLUP_ENABLE, .master.clk_speed = I2C_MASTER_FREQ_HZ };
    i2c_param_config(I2C_MASTER_NUM_USER, &conf);
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM_USER, conf.mode, 0, 0, 0));
}

static void app_lcd_init(void) {
    lcd_display_handle.address = LCD_ADDR;
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
    ledc_timer_config_t timer_cfg = { .speed_mode = LEDC_MODE_PWM, .timer_num = LEDC_TIMER_PWM, .duty_resolution = LEDC_DUTY_RES_PWM, .freq_hz = LEDC_FREQUENCY_PWM, .clk_cfg = LEDC_AUTO_CLK };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));
    ledc_channel_config_t channel_cfg = { .speed_mode = LEDC_MODE_PWM, .channel = LEDC_CHANNEL_PWM, .timer_sel = LEDC_TIMER_PWM, .intr_type = LEDC_INTR_DISABLE, .gpio_num = BUZZER_GPIO, .duty = 0 };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
}

static void gpios_init(void) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL<<SEG_A_GPIO)|(1ULL<<SEG_B_GPIO)|(1ULL<<SEG_C_GPIO)|(1ULL<<SEG_D_GPIO)|(1ULL<<SEG_E_GPIO)|(1ULL<<SEG_F_GPIO)|(1ULL<<SEG_G_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL<<BTN_A_GPIO)|(1ULL<<BTN_B_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_A_GPIO, button_isr_handler, (void*)BTN_A_GPIO);
    gpio_isr_handler_add(BTN_B_GPIO, button_isr_handler, (void*)BTN_B_GPIO);
    ESP_LOGI(TAG, "GPIOs inicializados.");
}

static void sd_card_init(void) {
    esp_vfs_fat_sdmmc_mount_config_t mnt_cfg = { .format_if_mount_failed = false, .max_files = 5, .allocation_unit_size = 16 * 1024 };
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = { .mosi_io_num = SPI_MOSI_PIN, .miso_io_num = SPI_MISO_PIN, .sclk_io_num = SPI_SCLK_PIN, .quadwp_io_num = -1, .quadhd_io_num = -1 };
    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Falha ao inicializar SPI bus"); return; }
    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = SD_CS_PIN;
    slot_cfg.host_id = host.slot;
    sdmmc_card_t* card;
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_cfg, &mnt_cfg, &card);
    sd_card_mounted = (ret == ESP_OK);
    if (!sd_card_mounted) { ESP_LOGE(TAG, "Falha ao montar SD card (%s)", esp_err_to_name(ret)); }
    else { ESP_LOGI(TAG, "SD card montado com sucesso."); }
}

float read_ntc_temperature_task(void) {
    int raw;
    if (adc_oneshot_read(adc_unit_handle, NTC_ADC_CHANNEL, &raw) != ESP_OK) return -273.15f;
    float r_div = (float)raw / (4095.0f - (float)raw);
    float log_r = logf(r_div);
    float t0_k = NTC_NOMINAL_TEMPERATURE + 273.15f;
    float tk = 1.0f / ((1.0f / t0_k) + (1.0f / NTC_BETA_COEFFICIENT) * log_r);
    return tk - 273.15f;
}

void set_buzzer_state(bool on) {
    ledc_set_duty(LEDC_MODE_PWM, LEDC_CHANNEL_PWM, on ? BUZZER_DUTY_ON : BUZZER_DUTY_OFF);
    ledc_update_duty(LEDC_MODE_PWM, LEDC_CHANNEL_PWM);
}

void display_7seg(char digit) {
    uint8_t segments = 0;
    switch(digit) {
        case '0': segments = 0b0111111; break; 
        case '3': segments = 0b01001111; break;
        case '7': segments = 0b0000111; break; 
        case 'D': segments = 0b01011110; break;
        case 'F': segments = 0b01110001; break; 
        default:  segments = 0b00000000; break;
    }
    gpio_set_level(SEG_A_GPIO, (segments >> 0) & 1);
    gpio_set_level(SEG_B_GPIO, (segments >> 1) & 1);
    gpio_set_level(SEG_C_GPIO, (segments >> 2) & 1);
    gpio_set_level(SEG_D_GPIO, (segments >> 3) & 1);
    gpio_set_level(SEG_E_GPIO, (segments >> 4) & 1);
    gpio_set_level(SEG_F_GPIO, (segments >> 5) & 1);
    gpio_set_level(SEG_G_GPIO, (segments >> 6) & 1);
}

static void IRAM_ATTR button_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    static uint64_t last_interrupt_time = 0;
    uint64_t interrupt_time = esp_timer_get_time();
    if (interrupt_time - last_interrupt_time > DEBOUNCE_TIME_MS * 1000) {
        last_interrupt_time = interrupt_time;
        xQueueSendFromISR(button_evt_queue, &gpio_num, NULL);
    }
}

// FreeRTOS
static void button_task(void* arg) {
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(button_evt_queue, &io_num, portMAX_DELAY)) {
            if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
                if (io_num == BTN_A_GPIO) {
                    alarm_temperature += 5;
                } else if (io_num == BTN_B_GPIO) {
                    alarm_temperature -= 5;
                }
                ESP_LOGI(TAG, "Nova temperatura de alarme: %dC", alarm_temperature);
                xSemaphoreGive(xDataMutex);
            }
        }
    }
}

static void sensor_task(void* arg) {
    for(;;) {
        float temp = read_ntc_temperature_task();
        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            current_ntc_temp = temp;
            xSemaphoreGive(xDataMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void alarm_logic_task(void* arg) {
    for(;;) {
        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            bool previous_alarm_state = alarm_active;
            if (!alarm_active && current_ntc_temp > alarm_temperature) {
                alarm_active = true;
            } else if (alarm_active && current_ntc_temp < alarm_temperature) {
                alarm_active = false;
            }
            if (alarm_active != previous_alarm_state) {
                 set_buzzer_state(alarm_active);
            }
            xSemaphoreGive(xDataMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

static void lcd_task(void* arg) {
    char buf1[17], buf2[17];
    float last_temp_disp = -1000;
    int last_alarm_disp = -1000;
    for(;;) {
        float temp_local = 0.0f;
        int alarm_local = 0;

        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            temp_local = current_ntc_temp;
            alarm_local = alarm_temperature;
            xSemaphoreGive(xDataMutex);
        }

        if (fabsf(temp_local - last_temp_disp) > 0.1f || alarm_local != last_alarm_disp) {
            snprintf(buf1, sizeof(buf1), "NTC: %.1fC", temp_local);
            snprintf(buf2, sizeof(buf2), "Alarme: %dC", alarm_local);
            
            lcd_i2c_cursor_set(&lcd_display_handle, 0, 0);
            lcd_i2c_printf(&lcd_display_handle, buf1); 
            lcd_i2c_cursor_set(&lcd_display_handle, 0, 1);
            lcd_i2c_printf(&lcd_display_handle, buf2);
            
            last_temp_disp = temp_local;
            last_alarm_disp = alarm_local;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void sd_card_task(void* arg) {
    if (!sd_card_mounted) {
        ESP_LOGE(TAG, "SD Card não montado, encerrando task de log.");
        vTaskDelete(NULL);
    }

    FILE* f = fopen(MOUNT_POINT "/datalog.txt", "a");
    if (!f) {
        ESP_LOGE(TAG, "Falha ao abrir datalog.txt, encerrando task.");
        vTaskDelete(NULL);
    }
    
    for(;;) {
        float temp_local = 0.0f;

        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            temp_local = current_ntc_temp;
            xSemaphoreGive(xDataMutex);
        }
        
        fprintf(f, "%.2f\n", temp_local);
        fflush(f);
        
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
    fclose(f);
}

static void display_7seg_task(void* arg) {
    char digit_to_show = ' ';
    bool is_blinking = false;
    bool blink_state = false;
    TickType_t last_blink_time = 0;

    for(;;) {
        float temp_local = 0.0f;
        int alarm_local = 0;
        bool alarm_on = false;

        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            temp_local = current_ntc_temp;
            alarm_local = alarm_temperature;
            alarm_on = alarm_active;
            xSemaphoreGive(xDataMutex);
        }

        float diff = (float)alarm_local - temp_local;
        
        if (alarm_on) {
            digit_to_show = 'F';
            is_blinking = true;
        } else {
            is_blinking = false;
            if (diff > 0 && diff <= 2.0f) digit_to_show = 'D';
            else if (diff > 2.0f && diff <= 10.0f) digit_to_show = '7';
            else if (diff > 10.0f && diff <= 15.0f) digit_to_show = '3';
            else if (diff > 15.0f && diff <= 20.0f) digit_to_show = '0';
            else digit_to_show = ' ';
        }
        
        if (is_blinking) {
            if ((xTaskGetTickCount() - last_blink_time) * portTICK_PERIOD_MS >= BLINK_INTERVAL_MS) {
                blink_state = !blink_state;
                display_7seg(blink_state ? digit_to_show : ' ');
                last_blink_time = xTaskGetTickCount();
            }
        } else {
            display_7seg(digit_to_show);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Atividade 10: FreeRTOS - Iniciando sistema.");
    
    xDataMutex = xSemaphoreCreateMutex();
    if (xDataMutex == NULL) {
        ESP_LOGE(TAG, "Falha ao criar Mutex!");
        return;
    }

    gpios_init();
    adc_init();
    i2c_bus_init();
    app_lcd_init();
    pwm_buzzer_init();
    sd_card_init();

    button_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
    xTaskCreate(sensor_task, "sensor_task", 2048, NULL, 5, NULL);
    xTaskCreate(alarm_logic_task, "alarm_logic_task", 2048, NULL, 6, NULL);
    xTaskCreate(lcd_task, "lcd_task", 4096, NULL, 4, NULL);
    xTaskCreate(sd_card_task, "sd_card_task", 4096, NULL, 3, NULL);
    xTaskCreate(display_7seg_task, "display_7seg_task", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "Todas as tarefas foram iniciadas.");
}