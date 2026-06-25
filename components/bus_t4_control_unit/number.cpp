#include "number.h"
#include "esphome/core/log.h"
#include "esphome/components/bus_t4/t4_packet.h"

#include <cmath>

namespace esphome::bus_t4_control_unit {

static const char *const TAG = "bus_t4.number";

// DMP payload starts after the header (7 bytes) + message header (5 bytes).
static constexpr uint8_t DATA_OFFSET = 12;

void BusT4Number::setup() {
  uint8_t message[5] = {FOR_CU, this->param_, REQ_GET, 0x00, 0x00};
  T4Packet rsp;

  if (this->parent_->dmp_request(this->target_address_, message, sizeof(message), &rsp, 500)) {
    if (rsp.message.dmp.status == ERR_NONE && rsp.size >= DATA_OFFSET + this->width_ + 1) {
      this->publish_state(t4_read_be(rsp, DATA_OFFSET, this->width_) * this->scale_);
    } else if (rsp.message.dmp.status == ERR_UNSUPPORTED) {
      this->mark_failed(LOG_STR("parameter not supported by controller"));
    }
  } else {
    this->mark_failed(LOG_STR("no response from controller"));
  }
}

void BusT4Number::control(float value) {
  uint32_t v = static_cast<uint32_t>(lroundf(value / this->scale_));
  ESP_LOGD(TAG, "Setting config 0x%02X to %u", this->param_, v);
  this->pending_value_ = v;
  this->has_pending_ = true;
  this->send_config_set(this->param_, v, this->width_);
}

void BusT4Number::on_packet(const T4Packet &packet) {
  if (packet.header.protocol != DMP)
    return;
  if (packet.message.command != this->param_)
    return;

  uint8_t flags = packet.message.dmp.flags;

  // Another client's SET request — stash the value, wait for confirmation.
  if (flags == REQ_SET) {
    if (packet.size < DATA_OFFSET + this->width_ + 1)
      return;
    this->pending_value_ = t4_read_be(packet, DATA_OFFSET, this->width_);
    this->has_pending_ = true;
    return;
  }

  if (packet.message.dmp.status != ERR_NONE)
    return;

  // SET confirmed — publish the pending value (ours or another client's).
  if (flags == RSP_SET_COMPLETE && this->has_pending_) {
    this->has_pending_ = false;
    this->publish_state(this->pending_value_ * this->scale_);
    return;
  }

  // GET response — parse the value directly.
  if (flags == RSP_GET_COMPLETE) {
    if (packet.size < DATA_OFFSET + this->width_ + 1)
      return;
    this->publish_state(t4_read_be(packet, DATA_OFFSET, this->width_) * this->scale_);
  }
}

void BusT4Number::dump_config() {
  LOG_NUMBER("", "Bus T4 Number", this);
  ESP_LOGCONFIG(TAG, "  Config param: 0x%02X", this->param_);
}

} // namespace esphome::bus_t4_control_unit
