#ifdef USE_ESP32_FRAMEWORK_ARDUINO

#include "dmx512esp32.h"
#include "esphome/core/log.h"

namespace esphome {
namespace dmx512 {

static const char *TAG = "dmx512";

void DMX512ESP32::send_break() {
  const uint8_t pin = this->tx_pin_->get_pin();
  pinMatrixOutDetach(pin, false, false);
  gpio_set_direction(static_cast<gpio_num_t>(pin), GPIO_MODE_OUTPUT);
  gpio_set_level(static_cast<gpio_num_t>(pin), 0);
  delayMicroseconds(this->break_len_);
  gpio_set_level(static_cast<gpio_num_t>(pin), 1);
  delayMicroseconds(this->mab_len_);
  pinMatrixOutAttach(pin, this->uart_idx_, false, false);
}

}  // namespace dmx512
}  // namespace esphome
#endif  // USE_ESP32_FRAMEWORK_ARDUINO
