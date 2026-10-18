#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "ROBOT_SERIAL";

// --- Настройки I2C для PCA9685 ---
#define PIN_SDA             GPIO_NUM_21
#define PIN_SCL             GPIO_NUM_22
#define PCA9685_ADDR        0x40

// --- Настройки UART (USB Serial) ---
#define UART_PORT_NUM       UART_NUM_0
#define UART_BUF_SIZE       512

// Тайминги сервоприводов для 50 Гц (~500 .. 2400 мкс)
#define TICKS_MIN           102   // 0°
#define TICKS_MAX           492   // 180°

static i2c_master_dev_handle_t pca_dev;

static esp_err_t pca_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(pca_dev, buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t set_servo_angle(uint8_t ch, float deg)
{
    if (ch >= 16) return ESP_ERR_INVALID_ARG;
    if (deg < 0.0f) deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;

    uint16_t off = TICKS_MIN + (uint16_t)((deg / 180.0f) * (TICKS_MAX - TICKS_MIN));
    uint8_t reg = 0x06 + (4 * ch);

    uint8_t pkt[5] = {reg, 0x00, 0x00, (uint8_t)(off & 0xFF), (uint8_t)(off >> 8)};
    return i2c_master_transmit(pca_dev, pkt, 5, pdMS_TO_TICKS(100));
}

static void release_all_servos(void)
{
    for (uint8_t ch = 0; ch < 4; ch++) {
        uint8_t reg = 0x06 + (4 * ch);
        uint8_t pkt[5] = {reg, 0x00, 0x00, 0x00, 0x10};
        i2c_master_transmit(pca_dev, pkt, 5, pdMS_TO_TICKS(100));
    }
}

static void pca9685_init_safe(void)
{
    pca_write(0x00, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));
    pca_write(0x00, 0x11); // SLEEP = 1
    vTaskDelay(pdMS_TO_TICKS(5));
    pca_write(0xFE, 121);  // 50 Гц
    pca_write(0x00, 0x21); // SLEEP = 0, AI = 1
    vTaskDelay(pdMS_TO_TICKS(10));
    pca_write(0x01, 0x04); // MODE2: OUTDRV = 1
}

static void process_command(char *cmd)
{
    cmd[strcspn(cmd, "\r\n")] = '\0';
    if (strlen(cmd) == 0) return;

    if (strncmp(cmd, "G ", 2) == 0) {
        float a, b, c, d;
        if (sscanf(cmd + 2, "%f %f %f %f", &a, &b, &c, &d) == 4) {
            set_servo_angle(0, a);
            set_servo_angle(1, b);
            set_servo_angle(2, c);
            set_servo_angle(3, d);
            printf("OK\n");
        } else {
            printf("ERR INVALID_ARGS\n");
        }
    } else if (strncmp(cmd, "J ", 2) == 0) {
        int ch;
        float deg;
        if (sscanf(cmd + 2, "%d %f", &ch, &deg) == 2 && ch >= 0 && ch <= 3) {
            set_servo_angle((uint8_t)ch, deg);
            printf("OK\n");
        } else {
            printf("ERR INVALID_JOINT\n");
        }
    } else if (strcmp(cmd, "STOP") == 0) {
        release_all_servos();
        printf("OK\n");
    } else {
        printf("ERR UNKNOWN_CMD\n");
    }
    fflush(stdout);
}

static void uart_rx_task(void *arg)
{
    char line_buf[128];
    int line_len = 0;
    uint8_t data[64];

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data), pdMS_TO_TICKS(20));
        for (int i = 0; i < len; i++) {
            char c = (char)data[i];
            if (c == '\n' || c == '\r') {
                if (line_len > 0) {
                    line_buf[line_len] = '\0';
                    process_command(line_buf);
                    line_len = 0;
                }
            } else if (line_len < sizeof(line_buf) - 1) {
                line_buf[line_len++] = c;
            }
        }
    }
}

void app_main(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = PIN_SCL,
        .sda_io_num = PIN_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCA9685_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &pca_dev));

    pca9685_init_safe();
    ESP_LOGI(TAG, "Готов к приему команд по Serial (115200)");

    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 5, NULL);
}	
