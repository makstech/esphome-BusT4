#include "switch.h"
#include "esphome/core/log.h"
#include "esphome/components/bus_t4/t4_packet.h"

namespace esphome::bus_t4_control_unit {

static const char *const TAG = "bus_t4.switch";

// DMP payload starts after the header (7 bytes) + message header (5 bytes).
static constexpr uint8_t DATA_OFFSET = 12;

void BusT4Switch::setup() {
  // Send a GET request and block until the controller responds.
  uint8_t message[5] = {FOR_CU, this->param_, REQ_GET, 0x00, 0x00};
  T4Packet rsp;

  if (this->parent_->dmp_request(this->target_address_, message, sizeof(message), &rsp, 500)) {
    if (rsp.message.dmp.status == ERR_NONE && rsp.size >= DATA_OFFSET + 2) {
      bool state = rsp.data[DATA_OFFSET] != 0x00;
      this->publish_state(state);
    } else if (rsp.message.dmp.status == ERR_UNSUPPORTED) {
      this->mark_failed(LOG_STR("parameter not supported by controller"));
    }
  } else {
    this->mark_failed(LOG_STR("no response from controller"));
  }
}

void BusT4Switch::write_state(bool state) {
  ESP_LOGD(TAG, "Setting config 0x%02X to %s", this->param_, state ? "ON" : "OFF");
  this->pending_state_ = state;
  this->has_pending_ = true;
  this->send_config_set(this->param_, state ? 0x01 : 0x00);
}

void BusT4Switch::on_packet(const T4Packet &packet) {
  if (packet.header.protocol != DMP)
    return;
  if (packet.message.command != this->param_)
    return;

  uint8_t flags = packet.message.dmp.flags;

  // Another client's SET request — stash the value, wait for confirmation.
  if (flags == REQ_SET) {
    if (packet.size < DATA_OFFSET + 2)
      return;
    this->pending_state_ = packet.data[DATA_OFFSET] != 0x00;
    this->has_pending_ = true;
    return;
  }

  if (packet.message.dmp.status != ERR_NONE)
    return;

  // SET confirmed — publish the pending value (ours or another client's).
  if (flags == RSP_SET_COMPLETE && this->has_pending_) {
    this->has_pending_ = false;
    ESP_LOGD(TAG, "Config 0x%02X confirmed %s", this->param_, this->pending_state_ ? "ON" : "OFF");
    this->publish_state(this->pending_state_);
    return;
  }

  // GET response — parse the value directly.
  if (flags == RSP_GET_COMPLETE) {
    if (packet.size < DATA_OFFSET + 2)
      return;
    bool state = packet.data[DATA_OFFSET] != 0x00;
    ESP_LOGD(TAG, "Config 0x%02X reported %s", this->param_, state ? "ON" : "OFF");
    this->publish_state(state);
  }
}

void BusT4Switch::dump_config() {
  LOG_SWITCH("", "Bus T4 Switch", this);
  ESP_LOGCONFIG(TAG, "  Config param: 0x%02X", this->param_);
}

} // namespace esphome::bus_t4_control_unit
