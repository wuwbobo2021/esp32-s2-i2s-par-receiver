#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "rom/ets_sys.h" //ets_delay_us()

#include "soc/soc.h"
#include "soc/gpio_periph.h"
#include "esp_rom_gpio.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "i2s_par_rec.h"

#define TX_DATA_SIZE 5120
static uint8_t tx_data[TX_DATA_SIZE] = {0};

static void input_sim(const uint8_t *data, int len);

__attribute__((always_inline))
static inline void i2s_par_sim_input_byte(uint8_t byte);

// portTICK_PERIOD_MS can be 10ms, which is the minimum (unit) delay time
#define os_delay_ms(ms) vTaskDelay(ms / portTICK_PERIOD_MS)

static void i2s_par_sim_init()
{
    for (int i = 1; i < TX_DATA_SIZE; i++) {
        tx_data[i] = tx_data[i - 1] + 1;
    }

    // it's said that I2S can be intialized only once,
    // but i2s_par_init() might be called in a loop
    ESP_ERROR_CHECK(i2s_par_init());
    for (int i = 0; i < sizeof(PinTab); i++)
        gpio_set_direction(PinTab[i], GPIO_MODE_INPUT_OUTPUT);
}

static void i2s_par_sim_rec(int sz)
{
    ESP_ERROR_CHECK(i2s_par_start(sz));

    ESP_LOGI("main", "doing input simulation...");
    input_sim(tx_data, sz); // spends the most time
    tx_data[0] += 1; // increase the HEAD byte for the next frame

    ESP_LOGI("main", "start waiting...");
    if (! i2s_wait_data(3000 / portTICK_PERIOD_MS)) {
        ESP_LOGI("main", "timeout, stop waiting.");
        i2s_par_log_status();
        return;
    }

    ESP_LOGI("main", "received, HEAD: 0x%02x.", i2s_par_rx_data[0]);
    i2s_par_log_status();

    // checks data integrity
    
    if (i2s_par_rx_data[1] != 1)
        ESP_LOGI("main", "ERROR detected: data[1]:0x%02x", i2s_par_rx_data[1]);
    
    uint8_t counter = i2s_par_rx_data[1] + 1;
    for (int i = 2; i < sz; i++) {
        if (i2s_par_rx_data[i] != counter) {
            ESP_LOGI("main", "ERROR detected: i:%u  data[i-1]:0x%02x  data[i]:0x%02x",
                i, i2s_par_rx_data[i - 1], i2s_par_rx_data[i]);
        }
        counter = i2s_par_rx_data[i] + 1;
    }
}

static void input_sim(const uint8_t *data, int len)
{
    gpio_set_level(PCLK, 0);
    gpio_set_level(VSYNC, 0);
    os_delay_ms(10);

    gpio_set_level(VSYNC, 1);
    os_delay_ms(10);
    // the reference said: camera should provide at least 8 PCLK rising-edge clock signals
    // when I2S0I_V_SYNC is held high. But it seems working without PCLK pulses.
    gpio_set_level(VSYNC, 0);
    os_delay_ms(10);

    for (int i = 0; i < len; i++) {
        gpio_set_level(PCLK, 0); // PCLK falling edge
        i2s_par_sim_input_byte(data[i]);
        ets_delay_us(200);
        gpio_set_level(PCLK, 1); // PCLK rising edge, I2S Rx sample?
        ets_delay_us(200);
    }

    gpio_set_level(PCLK, 0); // last PCLK falling edge, no operation
    os_delay_ms(10);
}

__attribute__((always_inline))
static inline bool get_pin_input(gpio_num_t pin)
{
    // pin is input - read the GPIO_IN_REG register
    if (pin < 32)
        return (REG_READ(GPIO_IN_REG) >> pin) & 1U;
    else
        return (REG_READ(GPIO_IN1_REG) >> (pin - 31)) & 1U;
}

__attribute__((always_inline))
static inline void i2s_par_sim_input_byte(uint8_t byte)
{
failed:
    for (uint8_t i = 0; i < 8; i++)
        gpio_set_level(PinTab[i], (byte >> i) & 1);
    ets_delay_us(500);
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t bit_read = get_pin_input(PinTab[i]);
        if (bit_read != ((byte >> i) & 1)) {
            ESP_LOGI("i2s_par_pins", "failed to set bit %u. expected: %u current %u",
                i, ((byte >> i) & 1), bit_read);
            goto failed;
        }
    }
}

void app_main(void)
{
    i2s_par_sim_init();

    int sz = 96;
    while (true) {
        i2s_par_sim_rec(sz);
        sz += 256; if (sz > TX_DATA_SIZE) sz = 96;
    }
}
