#include "sensor.h"
#include "esphome/core/log.h"

namespace esphome::bus_t4_control_unit {

static const char *const TAG = "bus_t4.sensor";

void BusT4Sensor::dump_config() {
  LOG_SENSOR("", "Bus T4 Sensor", this);
  ESP_LOGCONFIG(TAG, "  Config param: 0x%02X", this->param_);
}

}  // namespace esphome::bus_t4_control_unit
