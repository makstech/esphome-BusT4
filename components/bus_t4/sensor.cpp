#include "sensor.h"
#include "esphome/core/log.h"
#include "t4_packet.h"

namespace esphome::bus_t4 {

static const char *const TAG = "bus_t4.sensor";

// DMP payload starts after the header (7 bytes) + message header (5 bytes).
static constexpr uint8_t DATA_OFFSET = 12;

void BusT4Sensor::setup() {
  uint8_t message[5] = {FOR_CU, this->param_, REQ_GET, 0x00, 0x00};
  T4Packet rsp;

  if (this->parent_->dmp_request(this->target_address_, message, sizeof(message), &rsp, 500)) {
    if (rsp.message.dmp.status == ERR_NONE && rsp.size >= DATA_OFFSET + this->width_ + 1) {
      this->publish_state(t4_read_be(rsp, DATA_OFFSET, this->width_));
    } else if (rsp.message.dmp.status == ERR_UNSUPPORTED) {
      this->mark_failed(LOG_STR("parameter not supported by controller"));
    }
  }
}

void BusT4Sensor::update() {
  this->send_info_request(FOR_CU, static_cast<T4InfoCommand>(this->param_));
}

void BusT4Sensor::on_packet(const T4Packet &packet) {
  if (packet.header.protocol != DMP || packet.message.command != this->param_)
    return;
  if (packet.message.dmp.flags == RSP_GET_COMPLETE && packet.message.dmp.status == ERR_NONE &&
      packet.size >= DATA_OFFSET + this->width_ + 1) {
    this->publish_state(t4_read_be(packet, DATA_OFFSET, this->width_));
  }
}

void BusT4Sensor::dump_config() {
  LOG_SENSOR("", "Bus T4 Sensor", this);
  ESP_LOGCONFIG(TAG, "  Config param: 0x%02X", this->param_);
}

} // namespace esphome::bus_t4
