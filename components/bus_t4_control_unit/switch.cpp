#include "switch.h"
#include "esphome/core/log.h"

namespace esphome::bus_t4_control_unit {

static const char *const TAG = "bus_t4.switch";

void BusT4Switch::write_state(bool state) {
  ESP_LOGD(TAG, "Setting config 0x%02X to %s", this->param_, state ? "ON" : "OFF");
  this->set_pending_(state ? 0x01 : 0x00);
  this->send_config_set(this->param_, state ? 0x01 : 0x00);
}

void BusT4Switch::dump_config() {
  LOG_SWITCH("", "Bus T4 Switch", this);
  ESP_LOGCONFIG(TAG, "  Config param: 0x%02X", this->param_);
}

}  // namespace esphome::bus_t4_control_unit
