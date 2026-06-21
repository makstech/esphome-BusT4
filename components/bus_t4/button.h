#pragma once

#include "esphome/components/button/button.h"
#include "esphome/core/component.h"
#include "bus_t4.h"

namespace esphome::bus_t4 {

// Writes a fixed value to a control-unit parameter on press (e.g. reset the
// maintenance counter). One-shot; no state.
class BusT4Button : public button::Button, public BusT4Device, public Component {
 public:
  void dump_config() override;

  void set_param(uint8_t param) { this->param_ = param; }
  void set_value(uint8_t value) { this->value_ = value; }

 protected:
  void press_action() override;

  uint8_t param_{0};
  uint8_t value_{1};
};

} // namespace esphome::bus_t4
