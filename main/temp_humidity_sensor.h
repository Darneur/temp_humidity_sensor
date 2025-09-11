#ifndef TEMP_HUMIDITY_SENSOR_H
#include <stdint.h>
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"

const int DHT_IO = GPIO_NUM_32;
const int DHT_BOUNDARY = 50; // boundary between high and low states

typedef struct DHTData {
  uint8_t rh_i;
  uint8_t rh_d;
  uint8_t temp_i;
  uint8_t temp_d;
  uint8_t checksum;
} DHTData;

// ---- TRANSMIT CONFIG ----

const rmt_tx_channel_config_t tx_chan_config = {
    .gpio_num = DHT_IO,
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = 1000000,
    .mem_block_symbols = 64,
    .flags.invert_out = false,
    .flags.with_dma = false,
    .trans_queue_depth = 4,
};

const rmt_copy_encoder_config_t copy_encoder_config = {};
const rmt_transmit_config_t tx_config = {
    .loop_count = 0,
    .flags.eot_level = 1,
};

// ---- RECEIVE CONFIG ----

const rmt_rx_channel_config_t rx_chan_config = {
    .gpio_num = DHT_IO,
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = 1000000,
    .mem_block_symbols = 64,
    .flags.invert_in = false,
    .flags.with_dma = false,
};

const rmt_receive_config_t rx_config = {
    .signal_range_min_ns = 3000,   // 3us, would go higher if hardware allowed
    .signal_range_max_ns = 120000, // 800us
};

#endif // !TEMP_HUMIDITY_SENSOR_H
#define TEMP_HUMIDITY_SENSOR_H
