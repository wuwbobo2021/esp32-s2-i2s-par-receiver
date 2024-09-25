#ifndef I2S_PAR_REC_H
#define I2S_PAR_REC_H

#include "i2s_par_pins.h"

// one DMA descriptor's transfer size is limited by 12 bits (2^12)
#define DMA_BUFS_CNT_MAX 8
#define DMA_UNIT_SIZE 3600
#define RX_DATA_SIZE_MAX (DMA_BUFS_CNT_MAX * DMA_UNIT_SIZE)

// pointer of static buffer (RX_DATA_SIZE_MAX) defined in i2s_par_rec.c
extern volatile uint8_t *i2s_par_rx_data;

// initializes I2S for 8-bit parallel receving mode
esp_err_t i2s_par_init();

// starts receiving data, maximum rx_size is RX_DATA_SIZE_MAX
esp_err_t i2s_par_start(int rx_size);

// prints I2S 1st DMA desc status and INT_RAW_REG
void i2s_par_log_status();

// waits until rx_size of data is received, returns rx_size
// returns 0 if I2S_IN_SUC_EOF is not received within max_delay
// Never call it before i2s_par_start()
int i2s_wait_data(TickType_t max_delay);

// stop and disable I2S peripheral
void i2s_par_deinit();

#endif
