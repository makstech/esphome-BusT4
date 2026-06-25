#include "bus_t4_oxi.h"
#include "esphome/core/log.h"

namespace esphome::bus_t4_oxi {

using enum bus_t4::T4OxiCommand;   // OXI_REMOTE_LIST, OXI_BUTTON_READ
using enum bus_t4::T4Error;        // ERR_NONE
using enum bus_t4::T4InfoCommand;  // INF_PRD, INF_HWR, INF_FRM

static const char *const TAG = "bus_t4.oxi";

void BusT4Oxi::setup() {
  if (!parent_->has_oxi()) {
    ESP_LOGI(TAG, "No OXI receiver detected on the bus");
    return;
  }
  T4Source addr = parent_->get_oxi_address();
  product_ = parent_->fetch_string(addr, INF_PRD);
  hardware_ = parent_->fetch_string(addr, INF_HWR);
  firmware_ = parent_->fetch_string(addr, INF_FRM);
  ESP_LOGI(TAG, "OXI identity: %s hw=%s fw=%s", product_.c_str(), hardware_.c_str(), firmware_.c_str());
  if (product_sensor_ != nullptr && !product_.empty())
    product_sensor_->publish_state(product_);
  if (hardware_sensor_ != nullptr && !hardware_.empty())
    hardware_sensor_->publish_state(hardware_);
  if (firmware_sensor_ != nullptr && !firmware_.empty())
    firmware_sensor_->publish_state(firmware_);
}

void BusT4Oxi::dump_config() {
  ESP_LOGCONFIG(TAG, "Bus-T4 OXI:");
  if (parent_->has_oxi()) {
    T4Source a = parent_->get_oxi_address();
    ESP_LOGCONFIG(TAG, "  Address: 0x%02X.%02X %s %s", a.address, a.endpoint, product_.c_str(),
                  firmware_.c_str());
  } else {
    ESP_LOGCONFIG(TAG, "  Not detected");
  }
}

void BusT4Oxi::on_packet(const T4Packet &packet) {
  // The bus only routes FOR_OXI packets here.
  parse_oxi_packet(packet);
}

void BusT4Oxi::parse_oxi_packet(const T4Packet &packet) {
  // OXI packet: data[10]=command, [11]=subcommand, [12]=sequence, [13]=status, [14+]=payload.
  if (packet.size < 15)
    return;

  uint8_t command = packet.data[10];
  uint8_t subcommand = packet.data[11];
  uint8_t sequence = packet.data[12];
  uint8_t status = packet.data[13];
  if (status != ERR_NONE)
    return;

  const uint8_t *payload = &packet.data[14];

  // Remote control list (0x25): info about a remote that triggered an action.
  if (command == OXI_REMOTE_LIST && subcommand == 0x01 && sequence == 0x0A) {
    uint32_t remote_serial = (payload[5] << 24) | (payload[4] << 16) | (payload[3] << 8) | payload[2];
    uint8_t remote_command = payload[8] >> 4;
    uint8_t remote_button = payload[5] >> 4;
    uint8_t remote_mode = payload[7] + 1;
    uint8_t press_count = payload[6];
    ESP_LOGI(TAG, "Remote: serial=%08X, cmd=%d, btn=%d, mode=%d, presses=%d", remote_serial,
             remote_command, remote_button, remote_mode, press_count);
  }

  // Button read (0x26, 0x41): direct button press detection.
  if (command == OXI_BUTTON_READ && subcommand == 0x41 && sequence == 0x08) {
    uint8_t button = payload[0] >> 4;
    uint32_t remote_serial = (payload[0] & 0x0F) | (payload[1] << 4) | (payload[2] << 12) | (payload[3] << 20);
    ESP_LOGI(TAG, "Button press: btn=%d, remote=%05X", button, remote_serial);
  }
}

}  // namespace esphome::bus_t4_oxi
