#include "dmx512.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <cstring>
#ifdef USE_ESP_IDF
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#endif

namespace esphome {
namespace dmx512 {

static const char *TAG = "dmx512";

void DMX512::loop() {
  const uint32_t now = millis();
  const uint32_t elapsed = now - this->last_update_;

  const bool needs_update = this->update_ || (elapsed > this->update_interval_ && this->periodic_update_);
  if (!needs_update)
    return;

  const uint16_t active_channels = this->force_full_frames_ ? DMX_MAX_CHANNEL : this->max_chan_;
  const uint32_t min_interval = ((active_channels + 1) * 44) / 1000 + 2;
  if (elapsed <= min_interval)
    return;

  size_t frame_len = active_channels + 1;
  if (frame_len > DMX_MSG_SIZE)
    frame_len = DMX_MSG_SIZE;

  {
    InterruptLock lock;
    this->device_values_[0] = 0;
    memcpy(this->tx_buffer_, this->device_values_, frame_len);
    this->update_ = false;
  }

  if (this->pin_enable_) {
    this->pin_enable_->digital_write(true);
  }

  this->send_break();
  this->uart_->write_array(this->tx_buffer_, frame_len);
#ifdef USE_ESP_IDF
  const uint32_t frame_time_us = frame_len * 44 + this->break_len_ + this->mab_len_;
  const TickType_t wait_ticks = pdMS_TO_TICKS((frame_time_us / 1000) + 2);
  uart_wait_tx_done(static_cast<uart_port_t>(this->uart_idx_), wait_ticks);
#else
  this->uart_->flush();
#endif

  if (this->pin_enable_) {
    this->pin_enable_->digital_write(false);
  }

  this->last_update_ = now;
}

void DMX512::dump_config() {
  ESP_LOGCONFIG(TAG, "Setting up DMX512...");
}

void DMX512::setup() {
  memset(this->device_values_, 0, DMX_MSG_SIZE);
  if(this->pin_enable_) {
    ESP_LOGD(TAG, "Configuring RS485 driver enable pin");
    this->pin_enable_->setup();
    this->pin_enable_->digital_write(false);
  }
}

void DMX512::set_channel_used(uint16_t channel, bool used) {
  if (channel == 0 || channel > DMX_MAX_CHANNEL)
    return;

  this->channel_active_.set(channel, used);

  if (used) {
    if (channel > this->max_chan_)
      this->max_chan_ = channel;
    return;
  }

  if (channel == this->max_chan_) {
    this->recompute_max_channel_from_mask_();
  }
}

void DMX512::set_force_full_frames(bool force) {
  this->force_full_frames_ = force;
  if (force) {
    this->max_chan_ = DMX_MAX_CHANNEL;
    return;
  }

  this->recompute_max_channel_from_mask_();
}

void DMX512::write_channel(uint16_t channel, uint8_t value) {
  if (channel > DMX_MAX_CHANNEL)
    return;

  this->device_values_[channel] = value;
  this->update_ = true;
  this->set_channel_used(channel, value != 0);
}

void DMX512::recompute_max_channel_from_mask_() {
  if (this->force_full_frames_) {
    this->max_chan_ = DMX_MAX_CHANNEL;
    return;
  }

  for (int ch = DMX_MAX_CHANNEL; ch >= 1; --ch) {
    if (this->channel_active_.test(ch)) {
      this->max_chan_ = static_cast<uint16_t>(ch);
      return;
    }
  }

  this->max_chan_ = 0;
}

void DMX512Output::set_channel(uint16_t channel) {
  if (this->channel_ == channel)
    return;

  if (this->universe_ && this->channel_ != 0) {
    this->universe_->set_channel_used(this->channel_, false);
  }

  this->channel_ = channel;
  if(this->universe_) {
    this->universe_->set_channel_used(channel, true);
  }
}

void DMX512Output::write_state(float state) {
  this->state = state;
  // Fast conversion: add 0.5f before cast for proper rounding
  const uint8_t value = static_cast<uint8_t>(state * 255.0f + 0.5f);
  if(this->universe_)
    this->universe_->write_channel(this->channel_, value);
}

void DMX512Output::set_universe(DMX512 *universe) {
  if (universe == nullptr) {
    if (this->universe_ && this->channel_ != 0) {
      this->universe_->set_channel_used(this->channel_, false);
    }
    this->universe_ = nullptr;
    return;
  }

  if (this->universe_ == universe)
    return;

  if (this->universe_ && this->channel_ != 0) {
    this->universe_->set_channel_used(this->channel_, false);
  }

  this->universe_ = universe;

  if (this->universe_ && this->channel_ != 0) {
    this->universe_->set_channel_used(this->channel_, true);
  }
}

}  // namespace dmx512
}  // namespace esphome
