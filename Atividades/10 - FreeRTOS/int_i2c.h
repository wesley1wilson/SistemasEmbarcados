#ifndef INT_I2C_H
#define INT_I2C_H

#include "driver/i2c.h"
#include "rom/ets_sys.h" // For ets_delay_us
#include <stdarg.h>     // For va_list, va_start, etc.
#include <stdio.h>      // For vsnprintf
#include "freertos/FreeRTOS.h" // For vTaskDelay
#include "freertos/task.h"

// Biblioteca fornecida por luizfilipemr (com ajustes)

// PINOS DO PCF8574 (mapeamento comum)
// P0=RS, P1=RW, P2=EN, P3=BL
// P4=D4, P5=D5, P6=D6, P7=D7
#define RS_PIN_PCF8574 0 // Bit 0 do PCF8574 para RS
#define RW_PIN_PCF8574 1 // Bit 1 do PCF8574 para RW (geralmente não usado, conectado ao GND no módulo LCD)
#define EN_PIN_PCF8574 2 // Bit 2 do PCF8574 para EN
#define BL_PIN_PCF8574 3 // Bit 3 do PCF8574 para Backlight

// MEDIDAS DOS DISPLAYS
#define DISPLAY_16X02 0
#define DISPLAY_20X04 1

// INSTRUÇÕES DO DISPLAY LCD (HD44780)
#define LCD_CMD_CLEAR_DISPLAY               0x01
#define LCD_CMD_RETURN_HOME                 0x02 // Usado na inicialização, cuidado com o tempo de execução

#define LCD_CMD_ENTRY_MODE_SET              0x04
#define LCD_ENTRY_INCREMENT                 0x02 // I/D: Incrementa cursor
#define LCD_ENTRY_DECREMENT                 0x00 // I/D: Decrementa cursor
#define LCD_ENTRY_DISPLAY_SHIFT_ON          0x01 // S:   Habilita shift do display
#define LCD_ENTRY_DISPLAY_SHIFT_OFF         0x00 // S:   Desabilita shift do display

#define LCD_CMD_DISPLAY_CONTROL             0x08
#define LCD_DISPLAY_ON                      0x04 // D: Display ON
#define LCD_DISPLAY_OFF                     0x00 // D: Display OFF
#define LCD_CURSOR_ON                       0x02 // C: Cursor ON
#define LCD_CURSOR_OFF                      0x00 // C: Cursor OFF
#define LCD_BLINK_ON                        0x01 // B: Blink ON
#define LCD_BLINK_OFF                       0x00 // B: Blink OFF

#define LCD_CMD_CURSOR_DISPLAY_SHIFT        0x10
#define LCD_CMD_FUNCTION_SET                0x20
#define LCD_FUNCTION_8BIT_MODE              0x10 // DL: 8-bit
#define LCD_FUNCTION_4BIT_MODE              0x00 // DL: 4-bit
#define LCD_FUNCTION_2LINE                  0x08 // N:  2 linhas
#define LCD_FUNCTION_1LINE                  0x00 // N:  1 linha
#define LCD_FUNCTION_5x10DOTS               0x04 // F:  5x10 pontos
#define LCD_FUNCTION_5x8DOTS                0x00 // F:  5x8 pontos

#define LCD_CMD_SET_CGRAM_ADDR              0x40
#define LCD_CMD_SET_DDRAM_ADDR              0x80 // Comando base para endereço DDRAM

typedef struct {
  uint8_t address;    // Endereço I2C do PCF8574
  uint8_t i2c_port_num; // Número da porta I2C do ESP32 (ex: I2C_NUM_0)
  uint8_t backlight_on; // 1 para backlight ON, 0 para OFF
  uint8_t display_size; // DISPLAY_16X02 ou DISPLAY_20X04
  // Interno, não precisa ser configurado pelo usuário
  uint8_t current_control_byte; // Mantém estado de RS, RW, EN, BL
} lcd_i2c_handle_t;

// --- Funções Internas ---
static void lcd_i2c_send_pcf8574_byte(lcd_i2c_handle_t *lcd, uint8_t pcf_data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (lcd->address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, pcf_data, true);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(lcd->i2c_port_num, cmd, pdMS_TO_TICKS(50));
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) {
      // Adicionar log de erro se desejar
  }
}

static void lcd_i2c_pulse_enable(lcd_i2c_handle_t *lcd, uint8_t data_with_control) {
  lcd_i2c_send_pcf8574_byte(lcd, data_with_control | (1 << EN_PIN_PCF8574)); // EN high
  ets_delay_us(2); // EN pulse high time (tPW) >= 450ns, 2us é seguro
  lcd_i2c_send_pcf8574_byte(lcd, data_with_control & ~(1 << EN_PIN_PCF8574)); // EN low
  ets_delay_us(40); // Command execution time (tCYC) > 37us, 40-50us é seguro
}

static void lcd_i2c_send_nibble(lcd_i2c_handle_t *lcd, uint8_t nibble, bool is_data) {
  uint8_t pcf_data_bits = (nibble << 4) & 0xF0; // Mapeia o nibble para os bits D4-D7 (P4-P7 do PCF8574)
  uint8_t control_bits = (is_data ? (1 << RS_PIN_PCF8574) : 0) | (lcd->backlight_on ? (1 << BL_PIN_PCF8574) : 0);
  // RW_PIN_PCF8574 é sempre 0 (write)
  
  lcd_i2c_pulse_enable(lcd, pcf_data_bits | control_bits);
}

// --- Funções da API ---
void lcd_i2c_send_command(lcd_i2c_handle_t *lcd, uint8_t command) {
  lcd_i2c_send_nibble(lcd, (command >> 4) & 0x0F, false); // Envia high nibble
  lcd_i2c_send_nibble(lcd, command & 0x0F, false);    // Envia low nibble

  if (command == LCD_CMD_CLEAR_DISPLAY || command == LCD_CMD_RETURN_HOME) {
    vTaskDelay(pdMS_TO_TICKS(2)); // Esses comandos demoram mais (1.52ms - 1.64ms)
  }
}

void lcd_i2c_send_data_byte(lcd_i2c_handle_t *lcd, uint8_t data_byte) {
  lcd_i2c_send_nibble(lcd, (data_byte >> 4) & 0x0F, true); // Envia high nibble
  lcd_i2c_send_nibble(lcd, data_byte & 0x0F, true);    // Envia low nibble
}

void lcd_i2c_init(lcd_i2c_handle_t *lcd) {
  // Inicializar campos internos
  lcd->current_control_byte = (lcd->backlight_on ? (1 << BL_PIN_PCF8574) : 0);
  // Envia byte inicial para PCF8574 para garantir estado conhecido (ex: backlight)
  lcd_i2c_send_pcf8574_byte(lcd, lcd->current_control_byte);

  vTaskDelay(pdMS_TO_TICKS(50)); // Aguarda LCD ligar (datasheet: >40ms after VCC rises to 2.7V)

  // Sequência de inicialização para modo 4-bits (Hitachi HD44780 datasheet)
  // 1. Enviar comando 0x3 (para Function Set, 8-bit) - 3 vezes
  lcd_i2c_send_nibble(lcd, 0x03, false); // Envia nibble 0011 (para comando 0x30)
  vTaskDelay(pdMS_TO_TICKS(5));          // Aguarda > 4.1ms

  lcd_i2c_send_nibble(lcd, 0x03, false); // Envia nibble 0011 novamente
  vTaskDelay(pdMS_TO_TICKS(1));          // Aguarda > 100µs (usando 1ms por segurança)

  lcd_i2c_send_nibble(lcd, 0x03, false); // Envia nibble 0011 mais uma vez
  vTaskDelay(pdMS_TO_TICKS(1));

  // 2. Enviar comando 0x2 (para Function Set, mudar para 4-bit)
  lcd_i2c_send_nibble(lcd, 0x02, false); // Envia nibble 0010 (para comando 0x20)
  vTaskDelay(pdMS_TO_TICKS(1));          // Aguarda

  // Agora o LCD está em modo 4-bits. Comandos subsequentes são enviados como dois nibbles.
  // 3. Function Set: 4-bit, número de linhas, fonte
  lcd_i2c_send_command(lcd, LCD_CMD_FUNCTION_SET | LCD_FUNCTION_4BIT_MODE | LCD_FUNCTION_2LINE | LCD_FUNCTION_5x8DOTS); // 0x28

  // 4. Display Control: Display ON, Cursor OFF, Blink OFF
  lcd_i2c_send_command(lcd, LCD_CMD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF); // 0x0C

  // 5. Clear Display
  lcd_i2c_send_command(lcd, LCD_CMD_CLEAR_DISPLAY); // 0x01 (demora, delay já em send_command)

  // 6. Entry Mode Set: Incrementa cursor, sem shift do display
  lcd_i2c_send_command(lcd, LCD_CMD_ENTRY_MODE_SET | LCD_ENTRY_INCREMENT | LCD_ENTRY_DISPLAY_SHIFT_OFF); // 0x06
  
  vTaskDelay(pdMS_TO_TICKS(2)); // Delay adicional para estabilizar
}

void lcd_i2c_cursor_set(lcd_i2c_handle_t *lcd, uint8_t column, uint8_t row) {
  uint8_t row_offsets_1602[] = { 0x00, 0x40 };
  uint8_t row_offsets_2004[] = { 0x00, 0x40, 0x14, 0x54 }; // Endereços base das linhas
  uint8_t target_addr;

  if (lcd->display_size == DISPLAY_16X02) {
    if (row >= 2) row = 1; // Limita à linha 1 para 16x02
    target_addr = LCD_CMD_SET_DDRAM_ADDR | (row_offsets_1602[row] + column);
  } else { // Assume DISPLAY_20X04
    if (row >= 4) row = 3; // Limita à linha 3 para 20x04
    target_addr = LCD_CMD_SET_DDRAM_ADDR | (row_offsets_2004[row] + column);
  }
  lcd_i2c_send_command(lcd, target_addr);
}

void lcd_i2c_print_str(lcd_i2c_handle_t *lcd, const char *str) {
  while (*str) {
    lcd_i2c_send_data_byte(lcd, (uint8_t)(*str));
    str++;
  }
}

void lcd_i2c_printf(lcd_i2c_handle_t *lcd, const char *format_string, ...) {
  char buffer_string[128]; // Cuidado com o tamanho do buffer

  va_list arguments;
  va_start(arguments, format_string);
  vsnprintf(buffer_string, sizeof(buffer_string), format_string, arguments);
  va_end(arguments);

  lcd_i2c_print_str(lcd, buffer_string);
}

// Função para limpar o display (wrapper)
void lcd_i2c_clear(lcd_i2c_handle_t *lcd) {
    lcd_i2c_send_command(lcd, LCD_CMD_CLEAR_DISPLAY);
}

// Para compatibilidade com a API original da sua biblioteca (se precisar de custom char)
// A implementação original de lcd_i2c_write foi substituída por send_command/send_data_byte
// Se precisar de lcd_i2c_write(lcd, rs_flag, data_byte), pode ser reimplementada assim:
/*
void lcd_i2c_write(lcd_i2c_handle_t * lcd, char rs_flag, char data_byte) {
    if (rs_flag) { // Data
        lcd_i2c_send_data_byte(lcd, data_byte);
    } else { // Command
        lcd_i2c_send_command(lcd, data_byte);
    }
}
*/

#endif // INT_I2C_H