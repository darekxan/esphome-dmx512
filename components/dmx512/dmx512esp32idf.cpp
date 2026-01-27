#ifdef USE_ESP_IDF

#include "dmx512esp32idf.h"
#include "esphome/core/log.h"
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>

namespace esphome {
namespace dmx512 {

static const char *TAG = "dmx512";

void DMX512ESP32IDF::send_break() {
  uart_set_break((uart_port_t) this->uart_idx_, this->break_len_);

  // `uart_set_break` is asynchronous; wait until the break has fully completed before
  // applying the Mark-After-Break guard band so the TX pin stays attached to the UART
  // matrix instead of floating via GPIO toggles.
  uart_wait_tx_done((uart_port_t) this->uart_idx_, pdMS_TO_TICKS(5));

  delayMicroseconds(this->mab_len_);
}

}  // namespace dmx512
}  // namespace esphome
#endif  // USE_ESP_IDF
