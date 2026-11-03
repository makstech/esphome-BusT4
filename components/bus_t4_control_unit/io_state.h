#pragma once

#include <string>
#include <vector>
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/bus_t4/bus_t4.h"
#include "esphome/core/component.h"

namespace esphome::bus_t4_control_unit {

using bus_t4::BusT4Device;
using bus_t4::T4Packet;

// Polls the control unit's I/O diagnostics register (INF_IO, 0xD1) and publishes
// selected bits as binary sensors. The bit layout follows gashtaan's
// nice-bidiwifi-firmware diagnostics page.
class BusT4IoState : public PollingComponent, public BusT4Device {
 public:
  void update() override;
  void dump_config() override;
  void on_packet(const T4Packet &packet) override;

  void add_bit(binary_sensor::BinarySensor *sensor, uint8_t byte, uint8_t mask) {
    this->bits_.push_back({sensor, byte, mask});
  }
  void set_raw_sensor(text_sensor::TextSensor *s) { this->raw_sensor_ = s; }

 protected:
  struct Bit {
    binary_sensor::BinarySensor *sensor;
    uint8_t byte;
    uint8_t mask;
  };
  std::vector<Bit> bits_;
  text_sensor::TextSensor *raw_sensor_{nullptr};
  bool unsupported_logged_{false};
};

}  // namespace esphome::bus_t4_control_unit
