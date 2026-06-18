#pragma once

#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"
#include "../bus_t4.h"

namespace esphome::bus_t4 {

// Exposes a single control-unit configuration flag (auto-close, close-after-photo,
// etc.) as a Home Assistant switch. Each instance owns one config parameter byte:
//   - setup() reads the current value from the controller synchronously
//   - write_state() sends a DMP SET to change it
//   - on_packet() reacts to GET/SET replies on the bus and publishes the real value
class BusT4Switch : public switch_::Switch, public BusT4Device, public Component {
 public:
  void setup() override;
  void dump_config() override;

  // Receives every bus packet; we react to GET and SET replies for our parameter.
  void on_packet(const T4Packet &packet) override;

  void set_param(uint8_t param) { this->param_ = param; }

 protected:
  void write_state(bool state) override;

  uint8_t param_{0};
  bool pending_state_{false};
  bool has_pending_{false};
};

} // namespace esphome::bus_t4
