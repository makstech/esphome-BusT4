#pragma once

#include <optional>
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/component.h"
#include "../bus_t4.h"

namespace esphome::bus_t4 {

// EXPERIMENTAL — see .agent/PROTOCOL.md §3c.
//
// Exposes a BlueBus photocell beam as a binary_sensor (ON = beam blocked / obstacle).
// State is read from the controller's "BlueBus diagnostics" response (INF_DIAG_BB, 0xD0),
// which this device polls periodically. The 0xD0 payload byte layout is NOT documented for
// any controller and is known to differ between models, so the decode in photocell.cpp is a
// clearly-isolated best guess: the raw payload is always logged so it can be verified against
// a real capture (block/clear the beam) and the one decode constant corrected. Until then the
// published state should be treated as unverified.
class BusT4Photocell : public binary_sensor::BinarySensor, public BusT4Device, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_packet(const T4Packet &packet) override;

  void set_address(uint8_t address) { address_ = address; }

  // Decode whether the photocell at `address` (1..3) reports its beam blocked, from the
  // 0xD0 payload. Returns nullopt if the payload is too short / the entry isn't present.
  // Pure/static so it can be unit-tested on the host. GUESS layout — see photocell.cpp.
  static std::optional<bool> decode_blocked(const uint8_t *payload, int len, uint8_t address);

 protected:
  uint8_t address_{1};
  uint32_t last_poll_{0};
};

}  // namespace esphome::bus_t4
