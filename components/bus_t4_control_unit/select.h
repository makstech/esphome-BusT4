#pragma once

#include <vector>
#include "config_param.h"
#include "esphome/components/select/select.h"
#include "esphome/core/component.h"

namespace esphome::bus_t4_control_unit {

// An enumerated control-unit parameter (e.g. step-by-step mode) as a HA select.
// option_values_ maps each option (in declared order) to its raw controller value.
class BusT4Select : public select::Select, public Component, public ConfigParam<BusT4Select> {
 public:
  void setup() override { this->initial_get_(); }
  void dump_config() override;
  void add_option_value(uint8_t raw) { this->option_values_.push_back(raw); }
  // Publish the option mapped to a raw controller value; warns if unmapped.
  void publish_value_(uint32_t raw);

 protected:
  void control(const std::string &value) override;
  std::vector<uint8_t> option_values_;  // parallel to traits().get_options()
};

}  // namespace esphome::bus_t4_control_unit
