## DMX Blink Mitigation Plan (ESP-IDF)

### 1. Stabilize DMX frame emission (highest likelihood)
- Add a dedicated TX buffer inside [`dmx512::DMX512::loop()`](components/dmx512/dmx512.cpp:9)
- Copy `device_values_` into the TX buffer while holding a short critical section
- Mark shared flags (`update_`, `max_chan_`) as volatile (or use atomics) to ensure visibility across cores
- After `uart_write_bytes`, call `uart_wait_tx_done(uart_idx_, timeout)` before triggering the next break

**Status:** Implemented via `tx_buffer_` double-buffering, guarded copy, volatile flags, and platform-aware transmit draining (`uart_wait_tx_done`/`flush`).

### 2. Harden break/MAB generation (high likelihood)
- Update [`dmx512::DMX512ESP32IDF::send_break()`](components/dmx512/dmx512esp32idf.cpp:12) to use ESP-IDF’s UART break support (`uart_set_break`) or an RMT sequence, rather than toggling GPIO
- Keep the TX pin permanently attached to the UART matrix to avoid floating intervals
- Optionally, migrate break+MAB timing to RMT for sub-microsecond jitter

**Status:** Implemented by switching [`DMX512ESP32IDF::send_break()`](components/dmx512/dmx512esp32idf.cpp:11) over to the ESP-IDF HAL path (`uart_hal_tx_break`) with automatic baud-derived break symbol counts plus a TX-drain wait, keeping the UART pin muxed throughout break+MAB.

### 3. Reduce frame length jitter (medium likelihood)
- Track the highest active channel dynamically in [`dmx512::DMX512::set_channel_used()`](components/dmx512/dmx512.cpp:46)
- Recompute `max_chan_` when channels are removed or disabled, unless `force_full_frames_` is enabled
- Use the trimmed channel count when calculating `min_interval` to enlarge idle margin

### 4. Pulse RS-485 driver enable per frame (medium-to-low likelihood)
- Modify [`dmx512::DMX512::setup()`](components/dmx512/dmx512.cpp:37) and the transmit path so `pin_enable_` goes high only during break+data
- Lower `pin_enable_` immediately after `uart_wait_tx_done()` completes to let the bus idle high and reduce EMI sensitivity

### 5. Optional safeguards (lowest likelihood)
- Add counters for missed `uart_wait_tx_done` waits, RMT underruns, or skipped updates and expose via diagnostics
- Provide a configuration knob for `update_interval_` so integrators can reduce refresh pressure when scenes are static
