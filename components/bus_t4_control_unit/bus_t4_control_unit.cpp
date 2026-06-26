#include "bus_t4_control_unit.h"
#include "esphome/core/log.h"

namespace esphome::bus_t4_control_unit {

using enum bus_t4::T4InfoCommand;  // INF_MAN, INF_PRD, INF_FRM, INF_HWR, INF_DSC

static const char *const TAG = "bus_t4.control_unit";

void BusT4ControlUnit::setup() {
  if (!parent_->discovery_ready())
    return;  // bus WHO didn't resolve the controller
  T4Source a = parent_->get_controller_address();
  manufacturer_ = parent_->fetch_string(a, INF_MAN);
  product_ = parent_->fetch_string(a, INF_PRD);
  firmware_ = parent_->fetch_string(a, INF_FRM);
  hardware_ = parent_->fetch_string(a, INF_HWR);
  description_ = parent_->fetch_string(a, INF_DSC);
  ESP_LOGI(TAG, "Identity: %s %s hw=%s fw=%s (%s)", manufacturer_.c_str(), product_.c_str(),
           hardware_.c_str(), firmware_.c_str(), description_.c_str());
  if (product_sensor_ != nullptr && !product_.empty())
    product_sensor_->publish_state(product_);
  if (firmware_sensor_ != nullptr && !firmware_.empty())
    firmware_sensor_->publish_state(firmware_);
  if (hardware_sensor_ != nullptr && !hardware_.empty())
    hardware_sensor_->publish_state(hardware_);
  if (description_sensor_ != nullptr && !description_.empty())
    description_sensor_->publish_state(description_);
  ready_ = true;
}

void BusT4ControlUnit::dump_config() {
  ESP_LOGCONFIG(TAG, "Bus-T4 control unit:");
  if (ready_) {
    T4Source a = parent_->get_controller_address();
    ESP_LOGCONFIG(TAG, "  Address: 0x%02X.%02X %s %s", a.address, a.endpoint, product_.c_str(),
                  firmware_.c_str());
  } else {
    ESP_LOGCONFIG(TAG, "  Not discovered");
  }
}

}  // namespace esphome::bus_t4_control_unit
