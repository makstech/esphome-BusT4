#pragma once

#include <vector>
#include "esphome/components/select/select.h"
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

// Exposes an enumerated control-unit parameter (e.g. step-by-step mode) as a
// Home Assistant select. The controller speaks raw byte values; option_values_
// maps each option (in declared order) to its raw value. Mirrors BusT4Number:
// setup() reads the current value, control() sends a DMP SET, on_packet() tracks
// GET/SET replies on the bus.
class BusT4Select : public select::Select, public BusT4Device, public Component {
 public:
  void setup() override;
  void dump_config() override;
  void on_packet(const T4Packet &packet) override;

  void set_param(uint8_t param) { this->param_ = param; }
  void add_option_value(uint8_t raw) { this->option_values_.push_back(raw); }

 protected:
  void control(const std::string &value) override;
  // Publish the option mapped to a raw controller value; warns if unmapped.
  void publish_raw_(uint8_t raw);

  uint8_t param_{0};
  std::vector<uint8_t> option_values_;  // parallel to traits().get_options()
  uint8_t pending_value_{0};
  bool has_pending_{false};
};

} // namespace esphome::bus_t4_control_unit
