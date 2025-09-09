#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "hal/rmt_types.h"
#include "portmacro.h"
#include <string.h>
static const char *tag = "Main";
#include "freertos/FreeRTOS.h"
#include <stdio.h>

#define DHT_IO GPIO_NUM_32
#define DHT_BOUNDARY 50 // boundary between 0 and 1
typedef struct DHTData {
  uint8_t rh_i;
  uint8_t rh_d;
  uint8_t temp_i;
  uint8_t temp_d;
  uint8_t checksum;
} DHTData;

static TaskHandle_t dht_sensor_handle;
static DHTData dht_data;

static uint64_t _last_sensor_read_time = 0;

// ---- TRANSMIT CONFIG ----

rmt_tx_channel_config_t tx_chan_config = {
    .gpio_num = DHT_IO,
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = 1000000,
    .mem_block_symbols = 64,
    .flags.invert_out = false,
    .flags.with_dma = false,
    .trans_queue_depth = 4,
};

rmt_encoder_handle_t dht_encoder = NULL;
rmt_copy_encoder_config_t copy_encoder_config = {};
const rmt_transmit_config_t tx_config = {
    .loop_count = 0,
    .flags.eot_level = 1,
};

// ---- RECEIVE CONFIG ----

rmt_rx_channel_config_t rx_chan_config = {
    .gpio_num = DHT_IO,
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = 1000000,
    .mem_block_symbols = 64,
    .flags.invert_in = false,
    .flags.with_dma = false,
};

rmt_receive_config_t rx_config = {
    .signal_range_min_ns = 3000,   // 3us, would go higher if hardware allowed
    .signal_range_max_ns = 120000, // 800us
};

static SemaphoreHandle_t rmt_done_semaphore;

bool IRAM_ATTR on_revc_done_callback(rmt_channel_handle_t rx_chan,
                                     const rmt_rx_done_event_data_t *edata,
                                     void *user_ctx) {
  BaseType_t high_task_woken = pdFALSE;
  SemaphoreHandle_t sem = (SemaphoreHandle_t)user_ctx;
  xSemaphoreGiveFromISR(sem, &high_task_woken);
  return high_task_woken == pdTRUE;
}

rmt_rx_event_callbacks_t cbs = {
    .on_recv_done = on_revc_done_callback,
};

void dht_sensor_task(void *parameter) {
  // If < 1 seconds since startup, wait
  if (esp_timer_get_time() < 2000000) {
    vTaskDelay(pdMS_TO_TICKS(2000));  }
  // Set up receive channel and callback
  rmt_channel_handle_t rx_handle = NULL;
  rmt_channel_handle_t tx_handle = NULL;

  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_handle));
  ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_config, &dht_encoder));

  ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_handle));

  rmt_done_semaphore = xSemaphoreCreateBinary();
  ESP_ERROR_CHECK(
      rmt_rx_register_event_callbacks(rx_handle, &cbs, rmt_done_semaphore));

  rmt_symbol_word_t start_pulse[] = {
      {.duration0 = 20000, .level0 = 0, .duration1 = 30, .level1 = 1},
  };

  rmt_symbol_word_t buffer[50] = {0};
	//DHTData dht_data;

  ESP_ERROR_CHECK(rmt_enable(tx_handle));
  ESP_ERROR_CHECK(rmt_enable(rx_handle));

  while (1) {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
      memset(buffer, 0, sizeof(buffer));

      // Send intial 18ms pulse
      // gpio_set_direction(DHT_IO, GPIO_MODE_OUTPUT);
      // ggpio_set_level(DHT_IO, 0);
      // gvTaskDelay(pdMS_TO_TICKS(30));
      // ggpio_set_level(DHT_IO, 1);
      // ggpio_set_direction(DHT_IO, GPIO_MODE_INPUT);

      // Transmit start pulse
      ESP_ERROR_CHECK(rmt_transmit(tx_handle, dht_encoder, start_pulse,
                                   sizeof(start_pulse), &tx_config));
      // ESP_ERROR_CHECK(rmt_tx_wait_all_done(tx_handle, 100));

      // Start receive after registering event callbacks
      ESP_ERROR_CHECK(
          rmt_receive(rx_handle, buffer, sizeof(buffer), &rx_config));

      // Wait for task
      if (xSemaphoreTake(rmt_done_semaphore, pdMS_TO_TICKS(1000))) {
        ESP_LOGI(tag, "Reception finished!");
        uint8_t bytes[5] = {0};
        for (size_t i = 2; i < 50; i++) {
          ESP_LOGI(tag, "Symbol: %d: L0=%d, D0=%d ticks, L1=%d, D1=%d ticks", i,
                   buffer[i].level0, buffer[i].duration0, buffer[i].level1,
                   buffer[i].duration1);

          // Assemble bits
          int byte_index = (i - 2) / 8;
          int bit_index = 7 - ((i - 2) % 8);

          if (buffer[i].duration0 > DHT_BOUNDARY) {
            bytes[byte_index] |= (1 << bit_index);
          }
        }
        uint8_t checksum = bytes[0] + bytes[1] + bytes[2] + bytes[3];
        if (checksum != bytes[4]) {
          ESP_LOGE(tag, "Checksum mismatch! Calculated: %d, Received: %d",
                   checksum, bytes[4]);
        }
        // Checksum is valid, populate the struct
        dht_data.rh_i = bytes[0];
        dht_data.rh_d = bytes[1];
        dht_data.temp_i = bytes[2];
        dht_data.temp_d = bytes[3];
        dht_data.checksum = bytes[4];

        ESP_LOGI(tag, "Relative Humidity: %d.%d%%", dht_data.rh_i,
                 dht_data.rh_d);
        ESP_LOGI(tag, "Temperature: %d.%dC", dht_data.temp_i, dht_data.temp_d);

      } else {
        ESP_LOGE(tag, "Reception timed out!");
      }
    }
  }
}

// Safely calls dht_sensor_task
// TODO: return struct with data
void dht_get_data() {
	// wait at least 2 seconds between calls
	if (esp_timer_get_time() - _last_sensor_read_time < 2000000) {
		ESP_LOGI(tag, "dht_get_data called recently. Returning");
		return;
	}
	_last_sensor_read_time = esp_timer_get_time();
	xTaskNotifyGive(dht_sensor_handle);
}

void app_main(void) {

  // Launch Tasks
  xTaskCreate(dht_sensor_task, "dht_sensor_task", 4096, NULL, 10,
              &dht_sensor_handle);
	vTaskDelay(pdMS_TO_TICKS(4000));
  dht_get_data();

	//xTaskNotifyGive(dht_sensor_handle);


}
