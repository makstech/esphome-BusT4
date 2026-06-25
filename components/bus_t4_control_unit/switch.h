#pragma once

#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"
#include "esphome/components/bus_t4/bus_t4.h"

namespace esphome::bus_t4_control_unit {

// bus_t4 vocabulary this entity uses:
using bus_t4::BusT4Device;
using bus_t4::T4Packet;
using enum bus_t4::T4Protocol;
using enum bus_t4::T4Target;
using enum bus_t4::T4RequestType;
using enum bus_t4::T4ResponseType;
using enum bus_t4::T4Error;

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

} // namespace esphome::bus_t4_control_unit
