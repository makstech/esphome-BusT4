#include "photocell.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"  // format_hex_pretty

namespace esphome::bus_t4 {

static const char *const TAG = "bus_t4.photocell";

// How often we ask the controller for its BlueBus diagnostics (0xD0). Photocell beam state
// is not pushed while the gate is idle, so it must be polled. Kept gentle to avoid loading
// the bus (the cover already polls status independently).
static constexpr uint32_t POLL_INTERVAL_MS = 2000;

std::optional<bool> BusT4Photocell::decode_blocked(const uint8_t *payload, int len, uint8_t address) {
  // ================== GUESS — NOT yet verified against a capture ==================
  // The 0xD0 "BlueBus diagnostics" payload is undocumented and model-specific. The working
  // assumption is that it carries one status byte per recognised BlueBus device, in order,
  // and that photocell N occupies index (N-1) with the beam-blocked flag in the low bit.
  //
  // This is deliberately the ONLY place the assumption lives. When a capture is available
  // (block vs clear the beam, diff the logged "0xD0 payload" lines), correct the index/mask
  // here and add a unit test in tests/ with the real bytes. See .agent/PROTOCOL.md §3c.
  // ================================================================================
  int index = address - 1;
  if (payload == nullptr || index < 0 || index >= len)
    return std::nullopt;
  return (payload[index] & 0x01) != 0;
}

void BusT4Photocell::setup() {
  ESP_LOGW(TAG, "Photocell '%s' (address %u) is EXPERIMENTAL: the 0xD0 BlueBus diagnostics "
                "layout is unverified. Confirm against a capture before trusting the state.",
           this->get_name().c_str(), address_);
}

void BusT4Photocell::loop() {
  if (parent_ == nullptr)
    return;
  uint32_t now = millis();
  if (now - last_poll_ < POLL_INTERVAL_MS)
    return;
  last_poll_ = now;

  // Poll the controller (whatever address discovery found) for BlueBus diagnostics.
  this->set_target_address(parent_->get_controller_address());
  this->send_info_request(FOR_CU, INF_DIAG_BB);
}

void BusT4Photocell::on_packet(const T4Packet &packet) {
  // Only the controller's INF_DIAG_BB (0xD0) GET response is relevant.
  if (!packet.is_dmp() || packet.command() != INF_DIAG_BB)
    return;
  if (packet.dmp_status() != ERR_NONE)
    return;
  uint8_t flags = packet.dmp_flags();
  if (flags != RSP_GET_COMPLETE && flags != RSP_GET_INCOMPLETE)
    return;

  const uint8_t *payload = packet.dmp_payload();
  int len = packet.dmp_payload_len();
  if (len < 0)
    len = 0;

  // Always log the raw payload so the layout can be mapped from real hardware.
  ESP_LOGD(TAG, "0xD0 payload (addr %u): %s", address_,
           format_hex_pretty(payload, static_cast<size_t>(len)).c_str());

  auto blocked = decode_blocked(payload, len, address_);
  if (!blocked.has_value()) {
    ESP_LOGD(TAG, "0xD0 payload too short for photocell %u (len=%d)", address_, len);
    return;
  }
  this->publish_state(*blocked);
}

void BusT4Photocell::dump_config() {
  LOG_BINARY_SENSOR("", "Bus T4 Photocell", this);
  ESP_LOGCONFIG(TAG, "  Address (photocell index): %u", address_);
  ESP_LOGCONFIG(TAG, "  State source: BlueBus diagnostics 0xD0 (EXPERIMENTAL/unverified)");
}

}  // namespace esphome::bus_t4
