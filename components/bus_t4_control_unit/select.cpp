#include "select.h"
#include "esphome/core/log.h"
#include "esphome/components/bus_t4/t4_packet.h"

namespace esphome::bus_t4_control_unit {

static const char *const TAG = "bus_t4.select";

// DMP payload starts after the header (7 bytes) + message header (5 bytes).
static constexpr uint8_t DATA_OFFSET = 12;

void BusT4Select::setup() {
  uint8_t message[5] = {FOR_CU, this->param_, REQ_GET, 0x00, 0x00};
  T4Packet rsp;

  if (this->parent_->dmp_request(this->target_address_, message, sizeof(message), &rsp, 500)) {
    if (rsp.message.dmp.status == ERR_NONE && rsp.size >= DATA_OFFSET + 2) {
      this->publish_raw_(rsp.data[DATA_OFFSET]);
    } else if (rsp.message.dmp.status == ERR_UNSUPPORTED) {
      this->mark_failed(LOG_STR("parameter not supported by controller"));
    }
  } else {
    this->mark_failed(LOG_STR("no response from controller"));
  }
}

void BusT4Select::control(const std::string &value) {
  auto idx = this->index_of(value);
  if (!idx.has_value() || *idx >= this->option_values_.size()) {
    ESP_LOGW(TAG, "Unknown option '%s'", value.c_str());
    return;
  }
  uint8_t raw = this->option_values_[*idx];
  ESP_LOGD(TAG, "Setting config 0x%02X to %d (%s)", this->param_, raw, value.c_str());
  this->pending_value_ = raw;
  this->has_pending_ = true;
  this->send_config_set(this->param_, raw);
}

void BusT4Select::on_packet(const T4Packet &packet) {
  if (packet.header.protocol != DMP)
    return;
  if (packet.message.command != this->param_)
    return;

  uint8_t flags = packet.message.dmp.flags;

  // Another client's SET request — stash the value, wait for confirmation.
  if (flags == REQ_SET) {
    if (packet.size < DATA_OFFSET + 2)
      return;
    this->pending_value_ = packet.data[DATA_OFFSET];
    this->has_pending_ = true;
    return;
  }

  if (packet.message.dmp.status != ERR_NONE)
    return;

  // SET confirmed — publish the pending value (ours or another client's).
  if (flags == RSP_SET_COMPLETE && this->has_pending_) {
    this->has_pending_ = false;
    this->publish_raw_(this->pending_value_);
    return;
  }

  // GET response — parse the value directly.
  if (flags == RSP_GET_COMPLETE) {
    if (packet.size < DATA_OFFSET + 2)
      return;
    this->publish_raw_(packet.data[DATA_OFFSET]);
  }
}

void BusT4Select::publish_raw_(uint8_t raw) {
  for (size_t i = 0; i < this->option_values_.size(); i++) {
    if (this->option_values_[i] == raw) {
      this->publish_state(i);
      return;
    }
  }
  ESP_LOGW(TAG, "Controller reported unmapped value %d for 0x%02X", raw, this->param_);
}

void BusT4Select::dump_config() {
  LOG_SELECT("", "Bus T4 Select", this);
  ESP_LOGCONFIG(TAG, "  Config param: 0x%02X", this->param_);
}

} // namespace esphome::bus_t4_control_unit
