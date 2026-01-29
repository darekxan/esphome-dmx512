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
  if (!needs_update) {
    return;
  }

  const uint16_t active_channels = this->force_full_frames_ ? DMX_MAX_CHANNEL : this->max_chan_;
  const uint32_t min_interval = ((active_channels + 1) * 44) / 1000 + 2;
  if (elapsed < min_interval) {
    const bool coalesced = this->update_;
    if (coalesced) {
      this->coalesced_updates_++;
    } else {
      this->skipped_updates_++;
    }
    const uint32_t skip_log_elapsed = now - this->last_skip_log_;
    if (skip_log_elapsed >= this->diag_log_interval_ms_ && now > DMX_DIAG_LOG_START_DELAY_MS) {
      this->last_skip_log_ = now;
      ESP_LOGD(TAG,
               "DMX skip: elapsed=%ums min_interval=%ums update=%d periodic=%d coalesced=%d max_chan=%u "
               "active=%u force_full=%d update_interval=%d writes_since_send=%u",
               static_cast<unsigned>(elapsed),
               static_cast<unsigned>(min_interval),
               static_cast<int>(this->update_),
               static_cast<int>(this->periodic_update_),
               static_cast<int>(coalesced),
               static_cast<unsigned>(this->max_chan_),
               static_cast<unsigned>(active_channels),
               static_cast<int>(this->force_full_frames_),
               static_cast<int>(this->update_interval_),
               static_cast<unsigned>(this->writes_since_send_));
    }
    return;
  }

  size_t frame_len = this->max_chan_ + 1;
  if (frame_len > DMX_MSG_SIZE)
    frame_len = DMX_MSG_SIZE;

  {
    InterruptLock lock;
    this->device_values_[0] = 0;
    memcpy(this->tx_buffer_, this->device_values_, frame_len);
    this->update_ = false;
  }

  this->send_break();
  this->uart_->write_array(this->tx_buffer_, frame_len);
#ifdef USE_ESP_IDF
  const uint32_t frame_time_us = frame_len * 44 + this->break_len_ + this->mab_len_;
  const TickType_t wait_ticks = pdMS_TO_TICKS((frame_time_us / 1000) + 2);
  if (uart_wait_tx_done(static_cast<uart_port_t>(this->uart_idx_), wait_ticks) != ESP_OK) {
    this->missed_tx_waits_++;
  }
#else
  this->uart_->flush();
#endif
  this->last_update_ = now;
  this->writes_since_send_ = 0;

  const uint32_t diag_elapsed = now - this->last_diag_log_;
  const bool diag_time_reached = diag_elapsed >= this->diag_log_interval_ms_;
  const bool diag_after_start_delay = now > DMX_DIAG_LOG_START_DELAY_MS;
  if (diag_time_reached && diag_after_start_delay) {
    this->last_diag_log_ = now;
    ESP_LOGD(TAG,
             "DMX diagnostics: missed waits=%u skipped frames=%u coalesced=%u rmt underruns=%u "
             "frame_len=%u max_chan=%u active=%u",
             static_cast<unsigned>(this->missed_tx_waits_),
             static_cast<unsigned>(this->skipped_updates_),
             static_cast<unsigned>(this->coalesced_updates_),
             static_cast<unsigned>(this->rmt_underruns_),
             static_cast<unsigned>(frame_len),
             static_cast<unsigned>(this->max_chan_),
             static_cast<unsigned>(active_channels));
#ifdef USE_ESP_IDF
    uint32_t baud = 0;
    if (uart_get_baudrate(static_cast<uart_port_t>(this->uart_idx_), &baud) == ESP_OK) {
      ESP_LOGD(TAG, "DMX UART: baud=%u uart_idx=%d", static_cast<unsigned>(baud), this->uart_idx_);
    } else {
      ESP_LOGD(TAG, "DMX UART: baud=unknown uart_idx=%d", this->uart_idx_);
    }
#endif
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
  this->last_diag_log_ = millis();
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
  this->writes_since_send_++;
}

void DMX512::report_rmt_underrun() {
  this->rmt_underruns_++;
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
