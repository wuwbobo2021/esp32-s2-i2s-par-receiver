# esp32-s2-i2s-par-receiver

This program generates VSYNC, PCLK and 8-bit data signals on GPIOs, reads by itself with the I2S camera mode (which exists on ESP32 and ESP32-S2), and checks the integrity. Please check `i2s_par_pins.h` before running this program.

The idea of input simulation was inspired by <https://github.com/xenpac/ESP32-I2S-Simulator>, which seems to have a few problems with ESP32-S2:

- for ESP32-S2, the LSB can be `I2S0I_DATA_IN8_IDX` (not `I2S0I_DATA_IN0_IDX`) in 8-bit mode;
- `I2S0.fifo_conf.rx_fifo_mod` makes no sense at least in 8-bit mode, and it doesn't exist in ESP32-S2 technical reference;
- the pull-up, pull-down switching method of output simulation from `ESP32-I2S-Simulator` is proved unreliable, use `GPIO_MODE_INPUT_OUTPUT` instead;
- in `ESP32-I2S-Simulator`, `vTaskDelay(1/portTICK_PERIOD_MS)` is useless when `portTICK_PERIOD_MS` is larger than 1.

Known problem: the first frame received after I2S initialization is broken.

References:

- <https://github.com/espressif/esp32-camera/blob/master/target/esp32s2/ll_cam.c>
- <https://esp32.com/viewtopic.php?t=24980>
- <https://github.com/esp-rs/esp-hal/pull/1646>
