#include "select.h"
#include "esphome/core/log.h"

namespace esphome::bus_t4_control_unit {

static const char *const TAG = "bus_t4.select";

void BusT4Select::control(const std::string &value) {
  auto idx = this->index_of(value);
  if (!idx.has_value() || *idx >= this->option_values_.size()) {
    ESP_LOGW(TAG, "Unknown option '%s'", value.c_str());
    return;
  }
  uint8_t raw = this->option_values_[*idx];
  ESP_LOGD(TAG, "Setting config 0x%02X to %d (%s)", this->param_, raw, value.c_str());
  this->set_pending_(raw);
  this->send_config_set(this->param_, raw);
}

void BusT4Select::publish_value_(uint32_t raw) {
  for (size_t i = 0; i < this->option_values_.size(); i++) {
    if (this->option_values_[i] == raw) {
      this->publish_state(i);
      return;
    }
  }
  ESP_LOGW(TAG, "Controller reported unmapped value %u for 0x%02X", raw, this->param_);
}

void BusT4Select::dump_config() {
  LOG_SELECT("", "Bus T4 Select", this);
  ESP_LOGCONFIG(TAG, "  Config param: 0x%02X", this->param_);
}

}  // namespace esphome::bus_t4_control_unit
