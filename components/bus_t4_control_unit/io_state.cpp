#include "io_state.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome::bus_t4_control_unit {

using enum bus_t4::T4Target;
using enum bus_t4::T4Protocol;
using enum bus_t4::T4ResponseType;
using enum bus_t4::T4Error;
using enum bus_t4::T4InfoCommand;

static const char *const TAG = "bus_t4.io";

// DMP payload starts after the header (7 bytes) + message header (5 bytes).
static constexpr uint8_t IO_DATA_OFFSET = 12;

void BusT4IoState::update() { this->send_info_request(FOR_CU, INF_IO); }

void BusT4IoState::on_packet(const T4Packet &packet) {
  if (packet.header.protocol != DMP || packet.message.command != INF_IO ||
      packet.message.dmp.flags != RSP_GET_COMPLETE)
    return;
  if (packet.message.dmp.status != ERR_NONE) {
    // Logged once: the poll repeats for as long as the controller rejects it
    if (!this->unsupported_logged_) {
      ESP_LOGW(TAG, "Controller rejected the I/O diagnostics request (0x%02X)", packet.message.dmp.status);
      this->unsupported_logged_ = true;
    }
    return;
  }
  // The frame ends with a one-byte checksum after the payload
  if (packet.size <= IO_DATA_OFFSET + 1)
    return;
  const uint8_t len = packet.size - IO_DATA_OFFSET - 1;
  const uint8_t *data = packet.data + IO_DATA_OFFSET;

  for (auto &bit : this->bits_) {
    if (bit.byte < len)
      bit.sensor->publish_state((data[bit.byte] & bit.mask) != 0);
  }
  if (this->raw_sensor_ != nullptr) {
    std::string raw = format_hex_pretty(data, len);
    if (!this->raw_sensor_->has_state() || this->raw_sensor_->state != raw)
      this->raw_sensor_->publish_state(raw);
  }
}

void BusT4IoState::dump_config() {
  ESP_LOGCONFIG(TAG, "Bus T4 I/O state:");
  LOG_UPDATE_INTERVAL(this);
  for (auto &bit : this->bits_)
    LOG_BINARY_SENSOR("  ", "Bit", bit.sensor);
  LOG_TEXT_SENSOR("  ", "Raw", this->raw_sensor_);
}

}  // namespace esphome::bus_t4_control_unit
