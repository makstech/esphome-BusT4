#include "device_list.h"
#include "esphome/core/log.h"
#include <cstdio>

namespace esphome::bus_t4 {

static const char *const TAG = "bus_t4.device_list";

// Sentinel for "type not known yet" (no INF_WHO response seen for this address).
static constexpr uint8_t TYPE_UNKNOWN = 0xFF;

const char *BusT4DeviceList::device_type_name(uint8_t type) {
  switch (type) {
    case STANDARD: return "standard";
    case OVIEW: return "oview";
    case IT4WIFI: return "it4wifi";
    case CONTROLLER: return "controller";
    case SCREEN: return "screen";
    case RADIO: return "radio";
    case TYPE_UNKNOWN: return "unknown";
    default: return "other";
  }
}

bool BusT4DeviceList::record_device(uint16_t address, uint8_t type) {
  for (auto &e : devices_) {
    if (e.address == address) {
      // Upgrade a previously-unknown type once a known one is observed.
      if (type != TYPE_UNKNOWN && e.type != type) {
        e.type = type;
        return true;
      }
      return false;
    }
  }
  devices_.push_back({address, type});
  return true;
}

void BusT4DeviceList::on_packet(const T4Packet &packet) {
  uint16_t address = (static_cast<uint16_t>(packet.header.from.address) << 8) | packet.header.from.endpoint;
  if (address == 0xFFFF)
    return;  // broadcast, not a real source

  // Most packets only reveal the source address. An INF_WHO GET response also carries the
  // responder's device type in its first payload byte (cover.cpp uses the same field).
  uint8_t type = TYPE_UNKNOWN;
  if (packet.is_dmp() && packet.command() == INF_WHO && packet.dmp_status() == ERR_NONE &&
      (packet.dmp_flags() == RSP_GET_COMPLETE || packet.dmp_flags() == RSP_GET_INCOMPLETE) &&
      packet.dmp_payload_len() >= 1) {
    type = packet.dmp_payload()[0];
  }

  if (record_device(address, type))
    publish_list_();
}

void BusT4DeviceList::publish_list_() {
  std::string list;
  char buf[32];
  for (const auto &e : devices_) {
    if (!list.empty())
      list += ", ";
    std::snprintf(buf, sizeof(buf), "0x%02X.%02X %s", e.address >> 8, e.address & 0xFF, device_type_name(e.type));
    list += buf;
  }
  ESP_LOGD(TAG, "Bus devices: %s", list.c_str());
  this->publish_state(list);
}

void BusT4DeviceList::dump_config() { LOG_TEXT_SENSOR("", "Bus T4 Device List", this); }

}  // namespace esphome::bus_t4
