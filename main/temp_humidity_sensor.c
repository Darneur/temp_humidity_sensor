#include "temp_humidity_sensor.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "hal/gpio_types.h"
#include "portmacro.h"
#include <string.h>
static const char *tag = "Main";
#include "freertos/FreeRTOS.h"
#include <stdio.h>

#define TOUCH_SENSOR_GPIO GPIO_NUM_23

static TaskHandle_t dht_sensor_handle;
static DHTData dht_data;

static uint64_t _last_sensor_read_time = -2000001;
static SemaphoreHandle_t rmt_done_semaphore;     // For dht_sensor_task to signal recv done
static SemaphoreHandle_t data_ready_semaphore;   // For function dht_get_data()
static SemaphoreHandle_t touch_sensor_semaphore; // Give when touch sensor activates

rmt_encoder_handle_t dht_encoder = NULL;

// Handler for GPIO interrupt
void IRAM_ATTR touch_sensor_isr(void *parameter) {
  SemaphoreHandle_t sem = (SemaphoreHandle_t)parameter;
  xSemaphoreGiveFromISR(sem, NULL);
}

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
  const char *tag = "dht_sensor_task";
  // Set up receive channel and callback
  rmt_channel_handle_t rx_handle = NULL;
  rmt_channel_handle_t tx_handle = NULL;

  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_handle));
  ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_config, &dht_encoder));

  ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_handle));

  data_ready_semaphore = xSemaphoreCreateBinary();
  rmt_done_semaphore = xSemaphoreCreateBinary();
  ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_handle, &cbs, rmt_done_semaphore));

  rmt_symbol_word_t start_pulse[] = {
      {.duration0 = 20000, .level0 = 0, .duration1 = 30, .level1 = 1},
  };

  rmt_symbol_word_t buffer[50] = {0};
  // DHTData dht_data;

  ESP_ERROR_CHECK(rmt_enable(tx_handle));
  ESP_ERROR_CHECK(rmt_enable(rx_handle));

  // If < 2 seconds since startup, wait
  if (esp_timer_get_time() < 2000000) {
    vTaskDelay(pdMS_TO_TICKS(2000));
  }

  while (1) {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
      memset(buffer, 0, sizeof(buffer));

      // Transmit start pulse
      ESP_ERROR_CHECK(rmt_transmit(tx_handle, dht_encoder, start_pulse, sizeof(start_pulse), &tx_config));
      // ESP_ERROR_CHECK(rmt_tx_wait_all_done(tx_handle, 100));

      // Start receive after registering event callbacks
      ESP_ERROR_CHECK(rmt_receive(rx_handle, buffer, sizeof(buffer), &rx_config));

      // Wait for task
      if (xSemaphoreTake(rmt_done_semaphore, pdMS_TO_TICKS(1000))) {
        ESP_LOGI(tag, "Reception finished!");
        uint8_t bytes[5] = {0};
        for (size_t i = 2; i < 50; i++) {
          //ESP_LOGI(tag, "Symbol: %d: L0=%d, D0=%d ticks, L1=%d, D1=%d ticks", i, buffer[i].level0, buffer[i].duration0, buffer[i].level1, buffer[i].duration1);

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

        // ESP_LOGI(tag, "Relative Humidity: %d.%d%%", dht_data.rh_i,
        //          dht_data.rh_d);
        // ESP_LOGI(tag, "Temperature: %d.%dC", dht_data.temp_i, dht_data.temp_d);

      } else {
        ESP_LOGE(tag, "Reception timed out!");
      }
      xSemaphoreGive(data_ready_semaphore); // Signal to dht_get_data
    }
  }
}

// Safely calls dht_sensor_task
DHTData *dht_get_data() {
  const char *tag = "dht_get_data()";
  // If < 2 seconds since startup, wait
  if (esp_timer_get_time() < 2000000) {
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  // wait at least 2 seconds between calls
  if (esp_timer_get_time() - _last_sensor_read_time < 2000000) {
    ESP_LOGI(tag, "dht_get_data called recently. Returning cached data");
    return &dht_data;
  }
  ESP_LOGI(tag, "Getting new data");
  _last_sensor_read_time = esp_timer_get_time();
  xTaskNotifyGive(dht_sensor_handle);
  if (xSemaphoreTake(data_ready_semaphore, pdMS_TO_TICKS(2000))) {
    ESP_LOGI(tag, "Data request fulfilled");
    return &dht_data;
  } else {
    ESP_LOGE(tag, "Failed to get new data - request timed out!");
    return &dht_data;
  }
}

void app_main(void) {

  // Launch DHT11 sensor task
  xTaskCreate(dht_sensor_task, "dht_sensor_task", 4096, NULL, 10, &dht_sensor_handle);
	dht_get_data(); // Initial read to start sensor

  touch_sensor_semaphore = xSemaphoreCreateBinary();
  // Setup GPIO pin as interrupt for touch sensor
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  gpio_isr_handler_add(TOUCH_SENSOR_GPIO, touch_sensor_isr, touch_sensor_semaphore);
  gpio_intr_enable(TOUCH_SENSOR_GPIO);
  gpio_set_intr_type(TOUCH_SENSOR_GPIO, GPIO_INTR_POSEDGE);

  // Busy wait for touch sensor
  while (1) {
    if (xSemaphoreTake(touch_sensor_semaphore, portMAX_DELAY)) {
      dht_get_data();
      ESP_LOGI(tag, "Relative Humidity: %d.%d%%", dht_data.rh_i, dht_data.rh_d);
      ESP_LOGI(tag, "Temperature: %d.%dC", dht_data.temp_i, dht_data.temp_d);
    }
  }
}
