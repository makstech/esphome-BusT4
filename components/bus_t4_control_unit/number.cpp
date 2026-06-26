#include "number.h"
#include "esphome/core/log.h"

#include <cmath>

namespace esphome::bus_t4_control_unit {

static const char *const TAG = "bus_t4.number";

void BusT4Number::control(float value) {
  uint32_t raw = static_cast<uint32_t>(lroundf(value / this->scale_));
  ESP_LOGD(TAG, "Setting config 0x%02X to %u", this->param_, raw);
  this->set_pending_(raw);
  this->send_config_set(this->param_, raw, this->width_);
}

void BusT4Number::dump_config() {
  LOG_NUMBER("", "Bus T4 Number", this);
  ESP_LOGCONFIG(TAG, "  Config param: 0x%02X", this->param_);
}

}  // namespace esphome::bus_t4_control_unit
