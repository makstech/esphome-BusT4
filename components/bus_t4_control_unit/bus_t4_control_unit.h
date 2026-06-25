#pragma once

#include <string>
#include "esphome/core/component.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/bus_t4/bus_t4.h"

namespace esphome::bus_t4_control_unit {

using bus_t4::BusT4Device;
using bus_t4::T4Source;

// The Nice control unit as a device on the bus: reads its identity at startup
// and owns the identity sensors.
class BusT4ControlUnit : public Component, public BusT4Device {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }  // after the bus (HARDWARE)

  void set_product_sensor(text_sensor::TextSensor *s) { product_sensor_ = s; }
  void set_firmware_sensor(text_sensor::TextSensor *s) { firmware_sensor_ = s; }
  void set_hardware_sensor(text_sensor::TextSensor *s) { hardware_sensor_ = s; }
  void set_description_sensor(text_sensor::TextSensor *s) { description_sensor_ = s; }

  const std::string &product() const { return product_; }
  bool identity_ready() const { return ready_; }

 protected:
  text_sensor::TextSensor *product_sensor_{nullptr};
  text_sensor::TextSensor *firmware_sensor_{nullptr};
  text_sensor::TextSensor *hardware_sensor_{nullptr};
  text_sensor::TextSensor *description_sensor_{nullptr};
  std::string manufacturer_;
  std::string product_;
  std::string firmware_;
  std::string hardware_;
  std::string description_;
  bool ready_{false};
};

}  // namespace esphome::bus_t4_control_unit
