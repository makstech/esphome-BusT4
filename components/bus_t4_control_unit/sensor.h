#pragma once

#include "config_param.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome::bus_t4_control_unit {

using bus_t4::T4InfoCommand;

// Read-only numeric control-unit value (e.g. maintenance counters). update() polls
// a GET on the configured interval; the reply is published via the base on_packet.
class BusT4Sensor : public sensor::Sensor, public PollingComponent, public ConfigParam<BusT4Sensor> {
 public:
  void setup() override { this->initial_get_(); }
  void update() override { this->send_info_request(FOR_CU, static_cast<T4InfoCommand>(this->param_)); }
  void dump_config() override;
  void publish_value_(uint32_t raw) { this->publish_state(raw); }
};

}  // namespace esphome::bus_t4_control_unit
