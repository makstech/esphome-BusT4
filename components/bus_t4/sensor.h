#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "bus_t4.h"

namespace esphome::bus_t4 {

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

} // namespace esphome::bus_t4
