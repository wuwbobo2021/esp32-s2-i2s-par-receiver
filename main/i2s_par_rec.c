// by wuwbobo2021 <wuwbobo@outlook.com>
// inspired by https://github.com/xenpac/ESP32-I2S-Simulator

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "time.h"
#include "sys/time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "soc/soc.h"
#include "soc/periph_defs.h"
#include "soc/gpio_sig_map.h"
#include "soc/system_reg.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_periph.h"

#include "rom/lldesc.h"
#include "rom/gpio.h"
#include "esp_rom_gpio.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"

#include "esp_private/periph_ctrl.h"
#include "esp_intr_alloc.h"
#include "esp_system.h"
#include "esp_log.h"

#include "i2s_par_rec.h"

static SemaphoreHandle_t bin_semaphore = NULL;

bool i2s_par_inited = false;

int rx_data_size = 0;
DMA_ATTR lldesc_t dma_descs[DMA_BUFS_CNT_MAX]; //dma descriptors
DMA_ATTR uint8_t dma_bufs[DMA_BUFS_CNT_MAX][DMA_UNIT_SIZE];
volatile uint8_t *i2s_par_rx_data = (volatile uint8_t *)dma_bufs;

intr_handle_t i2s_int_hdl = NULL;

static void i2s_conf_reset(); //just for ensuring
void i2s_interrupt_handler(void *arg);
void vsync_handler(void *arg);

esp_err_t i2s_par_init()
{
    i2s_par_deinit();

    // initialize semaphore
    if (! bin_semaphore)
        bin_semaphore = xSemaphoreCreateBinary();
    else
        xSemaphoreTake(bin_semaphore, 0);
    if (! bin_semaphore) return ESP_ERR_NO_MEM;

    // ------------------------ Config GPIOs ------------------------
    gpio_config_t gpio_conf = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    for (int i = 0; i < sizeof(PinTab); i++) {
        gpio_conf.pin_bit_mask = 1ULL << PinTab[i];
        gpio_config(& gpio_conf);
    }

    // Route Pins to I2S peripheral using GPIO matrix, last parameter is invert
    // the received LSB is I2S0I_DATA_IN8_IDX in 8-bit mode
    gpio_matrix_in(D0,    I2S0I_DATA_IN8_IDX, 0);
    gpio_matrix_in(D1,    I2S0I_DATA_IN9_IDX, 0);
    gpio_matrix_in(D2,    I2S0I_DATA_IN10_IDX, 0);
    gpio_matrix_in(D3,    I2S0I_DATA_IN11_IDX, 0);
    gpio_matrix_in(D4,    I2S0I_DATA_IN12_IDX, 0);
    gpio_matrix_in(D5,    I2S0I_DATA_IN13_IDX, 0);
    gpio_matrix_in(D6,    I2S0I_DATA_IN14_IDX, 0);
    gpio_matrix_in(D7,    I2S0I_DATA_IN15_IDX, 0);
	
    // unused bits
    gpio_matrix_in(0x3C,  I2S0I_DATA_IN0_IDX, 0);
    gpio_matrix_in(0x3C,  I2S0I_DATA_IN1_IDX, 0);
    gpio_matrix_in(0x3C,  I2S0I_DATA_IN2_IDX, 0);
    gpio_matrix_in(0x3C,  I2S0I_DATA_IN3_IDX, 0);
    gpio_matrix_in(0x3C,  I2S0I_DATA_IN4_IDX, 0);
    gpio_matrix_in(0x3C,  I2S0I_DATA_IN5_IDX, 0);
    gpio_matrix_in(0x3C,  I2S0I_DATA_IN6_IDX, 0);
    gpio_matrix_in(0x3C,  I2S0I_DATA_IN7_IDX, 0);

    gpio_matrix_in(VSYNC, I2S0I_V_SYNC_IDX, 0);   // VSYNC
    gpio_matrix_in(0x38,  I2S0I_H_SYNC_IDX, 0);   // HIGH
    gpio_matrix_in(0x38,  I2S0I_H_ENABLE_IDX, 0); // HIGH
    gpio_matrix_in(PCLK,  I2S0I_WS_IN_IDX, 0);    // PCLK not inverted, sample on rising edge?

	gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM);
	gpio_isr_handler_add(VSYNC, &vsync_handler, NULL);
	gpio_set_intr_type(VSYNC, GPIO_INTR_DISABLE); // disable

    // ------------------------ Configure I2S peripheral ------------------------
    periph_module_enable(PERIPH_I2S0_MODULE);
    i2s_conf_reset();

    I2S0.clkm_conf.clkm_div_num = 2; // 160MHz / 2 = 80MHz
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_a = 0;
    I2S0.clkm_conf.clk_sel = 2;
    I2S0.clkm_conf.clk_en = 1;

    I2S0.conf.val = 0;
    I2S0.fifo_conf.val = 0;
    I2S0.fifo_conf.dscr_en = 1;

    I2S0.lc_conf.ahbm_fifo_rst = 1;
    I2S0.lc_conf.ahbm_fifo_rst = 0;
    I2S0.lc_conf.ahbm_rst = 1;
    I2S0.lc_conf.ahbm_rst = 0;
    I2S0.lc_conf.check_owner = 0;
    //I2S0.lc_conf.indscr_burst_en = 1;
    // DMA access external memory block size. 0: 16 bytes, 1: 32 bytes, 2:64 bytes, 3:reserved
    //I2S0.lc_conf.ext_mem_bk_size = 0; 

    I2S0.timing.val = 0;

    I2S0.int_ena.val = 0;
    I2S0.int_clr.val = ~0;

    I2S0.conf2.lcd_en = 1;
    I2S0.conf2.camera_en = 1;

    // Configuration data format
    I2S0.conf.rx_slave_mod = 1;
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = 0; //cam->swap_data;
    I2S0.conf.rx_short_sync = 0;
    I2S0.conf.rx_mono = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_dma_equal = 1;

    // Configure sampling rate
    I2S0.sample_rate_conf.rx_bck_div_num = 1; // manual says it cannot be 1
    I2S0.sample_rate_conf.rx_bits_mod = 8; // CHECK for different bit width

    I2S0.conf2.i_v_sync_filter_en = 1;
    I2S0.conf2.i_v_sync_filter_thres = 4;
    I2S0.conf2.cam_sync_fifo_reset = 1;
    I2S0.conf2.cam_sync_fifo_reset = 0;

    I2S0.conf_chan.rx_chan_mod = 1;

    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    I2S0.fifo_conf.rx_data_num = 32;
    I2S0.fifo_conf.rx_fifo_mod = 1; // SingleChannel16? doesn't make sense for S2 chip?

    I2S0.int_clr.val = 0x3ffff; // clear all intflags

    esp_err_t result = esp_intr_alloc(
        ETS_I2S0_INTR_SOURCE,
        /*ESP_INTR_FLAG_INTRDISABLED |*/ ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM, //IRAM!
        &i2s_interrupt_handler, NULL,
        &i2s_int_hdl
    );
    if (result != ESP_OK) return result;
    i2s_par_inited = true;
    
    return ESP_OK;
}

esp_err_t i2s_par_start(int rx_size)
{
    if (rx_size == 0
    ||  rx_size % 4 != 0
    ||  rx_size > RX_DATA_SIZE_MAX)
        return ESP_ERR_INVALID_SIZE;
    
    // make sure it's stopped
    I2S0.conf.rx_start = 0;
    I2S0.in_link.stop = 1;
    vTaskDelay(10);

    // create DMA descriptors for I2S Rx
    memset(dma_descs, 0, sizeof(dma_descs));
	int dma_full_size_cnt = rx_size / DMA_UNIT_SIZE;
	for (int i = 0; i < dma_full_size_cnt; i++) {
		dma_descs[i].owner = 1;             // DMA can use it
		dma_descs[i].size = DMA_UNIT_SIZE;  // destination buffersize
		dma_descs[i].length = 0;            // total bytes transfered, set by hardware at EOT
		dma_descs[i].buf = (uint8_t*)&(dma_bufs[i]);
		dma_descs[i].qe.stqe_next = &(dma_descs[i + 1]);
	}
	// set the last or only one
    dma_descs[dma_full_size_cnt].owner = 1; 
    dma_descs[dma_full_size_cnt].size = rx_size % DMA_UNIT_SIZE;  
    dma_descs[dma_full_size_cnt].length = 0; 
    dma_descs[dma_full_size_cnt].buf = (uint8_t*)&(dma_bufs[dma_full_size_cnt]);
    dma_descs[dma_full_size_cnt].eof = 1;
    dma_descs[dma_full_size_cnt].qe.stqe_next = NULL; //&(dma_descs[0]);
    
    // remember the total length that should be received
    rx_data_size = rx_size;

    // configure I2S with Rx size and DMA descriptors
    i2s_conf_reset();
    I2S0.rx_eof_num = rx_size; // CHECK for different bit width
    I2S0.in_link.addr = (uint32_t)&(dma_descs[0]); // set address of first descriptor
    I2S0.in_link.start = 1; // start DMA to process inlink descriptors

    I2S0.int_clr.val = I2S0.int_raw.val; // clear all int flags
    I2S0.int_ena.val = 0; // disable all ints
    I2S0.int_ena.in_suc_eof = 1; // enable int I2S_IN_SUC_EOF, triggered when rx_data_size is reached
    
    gpio_set_intr_type(VSYNC, GPIO_INTR_NEGEDGE); // VSYNC triggers int on neg edge
    esp_err_t result = esp_intr_enable(i2s_int_hdl);
    if (result == ESP_OK)
        I2S0.conf.rx_start = 1; // start receiving data. this bit starts/stops the receiver!

    if (bin_semaphore) xSemaphoreTake(bin_semaphore, 0);

    return result;
}

int i2s_wait_data(TickType_t max_delay)
{
    if (xSemaphoreTake(bin_semaphore, max_delay) == pdTRUE)
        return rx_data_size;
    else
        return 0;
}

// this triggers when requested number of bytes have been received
void IRAM_ATTR i2s_interrupt_handler(void *arg) // arg is just NULL
{
    I2S0.int_clr.val = I2S0.int_raw.val; // clear all int flags
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(bin_semaphore, &xHigherPriorityTaskWoken);
}

void i2s_par_deinit()
{
    if (! i2s_par_inited) return;
    
    I2S0.in_link.stop = 1;
    
    gpio_isr_handler_remove(VSYNC);
    if (i2s_int_hdl) {
        esp_intr_free(i2s_int_hdl);
        i2s_int_hdl = NULL;
    }

    i2s_conf_reset();
    periph_module_disable(PERIPH_I2S0_MODULE);
    i2s_par_inited = false;

    memset(dma_descs, 0, sizeof(dma_descs));
}

static void i2s_conf_reset()
{
    // reset DMA controller, AHB interface and AHB cmdFifo
    const uint32_t lc_conf_reset_flags = I2S_IN_RST_M | I2S_OUT_RST_M | I2S_AHBM_RST_M | I2S_AHBM_FIFO_RST_M;
    I2S0.lc_conf.val |= lc_conf_reset_flags; // set reset bits
    I2S0.lc_conf.val &= ~lc_conf_reset_flags; // clear reset bits

    // perform fifo and receiver reset, also transmitter
    const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
    I2S0.conf.val |= conf_reset_flags;
    I2S0.conf.val &= ~conf_reset_flags;
}

void i2s_par_log_status()
{
    printf("i2s_par  I2S0.int_raw.val: 0x%08lx \n", I2S0.int_raw.val);

    int dma_bufs_cnt = rx_data_size / DMA_UNIT_SIZE;
    if (rx_data_size % DMA_UNIT_SIZE) dma_bufs_cnt += 1;

    for (int i = 0; i < dma_bufs_cnt; i++)
	    printf(
            "i2s_par  dma_descs[%u]  size:%u  length:%u  eof:%u  owner:%u \n", i,
            dma_descs[i].size,   // length of data to be received for this desc (set by prog)
            dma_descs[i].length, // received length of this desc (set by DMA)
            dma_descs[i].eof,    // 1 for the last desc, 0 for others (set by prog)
            dma_descs[i].owner   // 1: DMA is using it 0: free from DMA
        );
}

void IRAM_ATTR vsync_handler(void *arg)
{
    // TODO
}
