#pragma once

#include "esphome/components/number/number.h"
#include "esphome/core/component.h"
#include "esphome/components/bus_t4/bus_t4.h"

namespace esphome::bus_t4_control_unit {

// bus_t4 vocabulary this entity uses:
using bus_t4::BusT4Device;
using bus_t4::T4Packet;
using bus_t4::t4_read_be;
using enum bus_t4::T4Protocol;
using enum bus_t4::T4Target;
using enum bus_t4::T4RequestType;
using enum bus_t4::T4ResponseType;
using enum bus_t4::T4Error;

// Exposes a numeric control-unit parameter (e.g. pause time, in seconds) as a
// Home Assistant number. Mirrors BusT4Switch: setup() reads the current value,
// control() sends a DMP SET, on_packet() tracks GET/SET replies on the bus.
class BusT4Number : public number::Number, public BusT4Device, public Component {
 public:
  void setup() override;
  void dump_config() override;
  void on_packet(const T4Packet &packet) override;

  void set_param(uint8_t param) { this->param_ = param; }
  void set_width(uint8_t width) { this->width_ = width; }
  // Displayed value = raw controller value * scale (e.g. 0.1 when raw is tenths).
  void set_scale(float scale) { this->scale_ = scale; }

 protected:
  void control(float value) override;

  uint8_t param_{0};
  uint8_t width_{1};
  float scale_{1.0f};
  uint32_t pending_value_{0};  // raw controller value
  bool has_pending_{false};
};

} // namespace esphome::bus_t4_control_unit
