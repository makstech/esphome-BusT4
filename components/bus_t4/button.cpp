#include "button.h"
#include "esphome/core/log.h"
#include "t4_packet.h"

namespace esphome::bus_t4 {

static const char *const TAG = "bus_t4.button";

void BusT4Button::press_action() {
  ESP_LOGD(TAG, "Writing 0x%02X = %d", this->param_, this->value_);
  this->send_config_set(this->param_, this->value_);
}

void BusT4Button::dump_config() {
  LOG_BUTTON("", "Bus T4 Button", this);
  ESP_LOGCONFIG(TAG, "  Config param: 0x%02X", this->param_);
}

} // namespace esphome::bus_t4
