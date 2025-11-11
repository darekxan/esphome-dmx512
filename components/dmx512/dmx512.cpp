#include "dmx512.h"
#include "esphome/core/log.h"

namespace esphome {
namespace dmx512 {

static const char *TAG = "dmx512";

void DMX512::loop() {
  // Cache millis() to avoid multiple system calls
  const uint32_t now = millis();
  const uint32_t elapsed = now - this->last_update_;
  
  // Check if update needed: either data changed OR periodic update is due
  if(this->update_ || (elapsed > this->update_interval_ && this->periodic_update_)) {
    // Enforce minimum DMX interval (23ms per spec)
    if(elapsed > DMX_MIN_INTERVAL_MS) {
      this->send_break();
      this->device_values_[0] = 0;
      this->uart_->write_array(this->device_values_, this->max_chan_ + 1);
      this->update_ = false;
      this->last_update_ = now;
    }
  }
}

void DMX512::dump_config() {
  ESP_LOGCONFIG(TAG, "Setting up DMX512...");
}

void DMX512::setup() {
  memset(this->device_values_, 0, DMX_MSG_SIZE);
  if(this->pin_enable_) {
    ESP_LOGD(TAG, "Enabling RS485 module");
    this->pin_enable_->setup();
    this->pin_enable_->digital_write(true);
  }
}

void DMX512::set_channel_used(uint16_t channel) {
  if(channel > this->max_chan_)
    this->max_chan_ = channel;
  if(this->force_full_frames_)
    this->max_chan_ = DMX_MAX_CHANNEL;
}

void DMX512::write_channel(uint16_t channel, uint8_t value) {
  this->device_values_[channel] = value;
  this->update_ = true;
}

void DMX512Output::set_channel(uint16_t channel) {
  this->channel_ = channel;
  if(this->universe_) {
    this->universe_->set_channel_used(channel);
  }
}

void DMX512Output::write_state(float state) {
  this->state = state;
  // Fast conversion: add 0.5f before cast for proper rounding
  const uint8_t value = static_cast<uint8_t>(state * 255.0f + 0.5f);
  if(this->universe_)
    this->universe_->write_channel(this->channel_, value);
}

}  // namespace dmx512
}  // namespace esphome
