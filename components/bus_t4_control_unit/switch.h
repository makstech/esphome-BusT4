#pragma once

#include "config_param.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"

namespace esphome::bus_t4_control_unit {

// A control-unit on/off flag (auto-close, pre-flash, …) as a Home Assistant switch.
class BusT4Switch : public switch_::Switch, public Component, public ConfigParam<BusT4Switch> {
 public:
  void setup() override { this->initial_get_(); }
  void dump_config() override;
  void publish_value_(uint32_t raw) { this->publish_state(raw != 0); }

 protected:
  void write_state(bool state) override;
};

}  // namespace esphome::bus_t4_control_unit
