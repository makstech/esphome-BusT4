#pragma once

#include "config_param.h"
#include "esphome/components/number/number.h"
#include "esphome/core/component.h"

namespace esphome::bus_t4_control_unit {

// A numeric control-unit parameter (e.g. pause time, in seconds) as a HA number.
class BusT4Number : public number::Number, public Component, public ConfigParam<BusT4Number> {
 public:
  void setup() override { this->initial_get_(); }
  void dump_config() override;
  // Displayed value = raw controller value * scale (e.g. 0.1 when raw is tenths).
  void set_scale(float scale) { this->scale_ = scale; }
  void publish_value_(uint32_t raw) { this->publish_state(raw * this->scale_); }

 protected:
  void control(float value) override;
  float scale_{1.0f};
};

}  // namespace esphome::bus_t4_control_unit
