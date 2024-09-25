#ifndef I2S_PAR_PINS_H
#define I2S_PAR_PINS_H

#include <stdint.h>

// PIN definitions for the parallel interface. Make sure they are all free.
// use 0x3C (ESP32S2!) for constant LOW, or 0x38 for constant HIGH Levels, as PIN-number.
#define VSYNC   3 
#define PCLK    4  // read data on rising edge (modify i2s_par_rec.c to config for falling edge)
#define D7      8
#define D6      7
#define D5      13
#define D4      12
#define D3      11
#define D2      10
#define D1      17
#define D0      5

// don't modify
static const uint8_t PinTab[10] = {D0, D1, D2, D3, D4, D5, D6, D7, VSYNC, PCLK};

#endif
