#ifdef USE_ESP_IDF

#include "dmx512esp32idf.h"
#include "esphome/core/log.h"
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <hal/uart_hal.h>
#include <hal/uart_ll.h>

namespace esphome {
namespace dmx512 {

static const char *TAG = "dmx512";

void DMX512ESP32IDF::send_break() {
  uart_hal_context_t hal;
  hal.dev = UART_LL_GET_HW(this->uart_idx_);

  uint32_t baud = 0;
  if (uart_get_baudrate(static_cast<uart_port_t>(this->uart_idx_), &baud) != ESP_OK || baud == 0) {
    baud = 250000;  // fall back to typical DMX baud if the driver hasn't been configured yet
  }

  const uint64_t break_cycles = ((uint64_t) baud * this->break_len_) / 1'000'000ULL;
  uint32_t break_symbols = static_cast<uint32_t>(break_cycles);
  if (break_symbols == 0) {
    break_symbols = 1;
  }

  uart_hal_tx_break(&hal, break_symbols);
  if (uart_wait_tx_done(static_cast<uart_port_t>(this->uart_idx_), pdMS_TO_TICKS(5)) != ESP_OK) {
    this->report_rmt_underrun();
  }
  delayMicroseconds(this->mab_len_);
}

}  // namespace dmx512
}  // namespace esphome
#endif  // USE_ESP_IDF
