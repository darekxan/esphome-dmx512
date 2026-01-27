#pragma once

#include <algorithm>
#include <bitset>

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/output/float_output.h"

static const uint16_t UPDATE_INTERVAL_MS = 500;
static const uint16_t DMX_MAX_CHANNEL = 512;
static const uint16_t DMX_MSG_SIZE = DMX_MAX_CHANNEL + 1;
static const int DMX_BREAK_LEN = 92;
static const int DMX_MAB_LEN = 12;
static const int DMX_MIN_INTERVAL_MS = 23;
static const uint32_t DMX_DIAG_LOG_INTERVAL_MS = 60 * 1000;
static const uint32_t DMX_DIAG_LOG_START_DELAY_MS = 5 * 1000;

namespace esphome {
namespace dmx512 {

class DMX512Output;

class DMX512 : public Component {
 public:
  DMX512() = default;
  void set_uart_parent(esphome::uart::UARTComponent *parent) { this->uart_ = parent; }

  void setup() override;

  void loop() override;

  void dump_config() override;
  
  virtual void send_break() = 0;

  void set_enable_pin(GPIOPin *pin_enable) { pin_enable_ = pin_enable; }
  
  void set_uart_tx_pin(InternalGPIOPin *tx_pin) { tx_pin_ = tx_pin; }
  
  void set_channel_used(uint16_t channel, bool used = true);

  void set_force_full_frames(bool force);

  void set_periodic_update(bool update) { periodic_update_ = update; }

  void set_mab_len(int len) { mab_len_ = len; }

  void set_break_len(int len) { break_len_ = len; }

  void set_update_interval(int intvl) {
    this->update_interval_ = std::max(intvl, DMX_MIN_INTERVAL_MS);
  }

  int get_update_interval() const { return this->update_interval_; }

  virtual void set_uart_num(int num) = 0;

  float get_setup_priority() const override { return setup_priority::BUS; }

  void write_channel(uint16_t channel, uint8_t value);

 protected:

  esphome::uart::UARTComponent *uart_{nullptr};
  uint8_t device_values_[DMX_MSG_SIZE];
  int uart_idx_{0};
  InternalGPIOPin *tx_pin_{nullptr};
  int update_interval_{UPDATE_INTERVAL_MS};
  int mab_len_{DMX_MAB_LEN};
  int break_len_{DMX_BREAK_LEN};
  volatile uint16_t max_chan_{0};
  volatile bool update_{true};
  uint8_t tx_buffer_[DMX_MSG_SIZE];
  bool periodic_update_{true};
  bool force_full_frames_{false};
  uint32_t last_update_{0};
  GPIOPin *pin_enable_{nullptr};
  std::bitset<DMX_MSG_SIZE> channel_active_{};
  volatile uint32_t missed_tx_waits_{0};
  volatile uint32_t skipped_updates_{0};
  volatile uint32_t rmt_underruns_{0};
  uint32_t last_diag_log_{0};
  uint32_t diag_log_interval_ms_{DMX_DIAG_LOG_INTERVAL_MS};

  void recompute_max_channel_from_mask_();
  void report_rmt_underrun();
};

class DMX512Output : public output::FloatOutput, public Component {
public:
  void set_universe(DMX512 *universe);
  void set_channel(uint16_t channel);
  void write_state(float state) override;
  float state;

protected:
  uint16_t channel_{0};
  DMX512 *universe_{nullptr};
};

}  // namespace dmx512
}  // namespace esphome
