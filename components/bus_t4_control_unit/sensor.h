#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/components/bus_t4/bus_t4.h"

namespace esphome::bus_t4_control_unit {

// bus_t4 vocabulary this entity uses:
using bus_t4::BusT4Device;
using bus_t4::T4Packet;
using bus_t4::T4InfoCommand;
using bus_t4::t4_read_be;
using enum bus_t4::T4Protocol;
using enum bus_t4::T4Target;
using enum bus_t4::T4RequestType;
using enum bus_t4::T4ResponseType;
using enum bus_t4::T4Error;

// Read-only numeric control-unit value (e.g. maintenance counters). setup() reads
// the value once; update() polls a GET on the configured interval, with the reply
// published via on_packet.
class BusT4Sensor : public sensor::Sensor, public BusT4Device, public PollingComponent {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  void on_packet(const T4Packet &packet) override;

  void set_param(uint8_t param) { this->param_ = param; }
  void set_width(uint8_t width) { this->width_ = width; }

 protected:
  uint8_t param_{0};
  uint8_t width_{1};
};

} // namespace esphome::bus_t4_control_unit
