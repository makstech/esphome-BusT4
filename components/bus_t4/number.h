#pragma once

#include "esphome/components/number/number.h"
#include "esphome/core/component.h"
#include "bus_t4.h"

namespace esphome::bus_t4 {

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

 protected:
  void control(float value) override;

  uint8_t param_{0};
  uint8_t width_{1};
  uint32_t pending_value_{0};
  bool has_pending_{false};
};

} // namespace esphome::bus_t4
