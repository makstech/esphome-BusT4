#pragma once

#include "bus_t4_component.h"

#include <freertos/queue.h>
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome::bus_t4 {

class BusT4Device {
 public:
  BusT4Device() = default;
  explicit BusT4Device(BusT4Component *parent) : parent_(parent) {}

  bool read(T4Packet *packet, TickType_t xTicksToWait) {
    return parent_->read(packet, xTicksToWait);
  }

  bool write(T4Packet *packet, TickType_t xTicksToWait) {
    return parent_->write(packet, xTicksToWait);
  }

  void set_parent(BusT4Component *parent) {
    this->parent_ = parent;
    parent->register_device(this);
  }

  // Send a DEP control command as OVIEW device (default)
  void send_cmd(T4Command cmd);

  // Send a DEP control command as a specific device type
  // Use IT4WIFI for security commands (block/release)
  void send_cmd(T4Command cmd, T4Device device);

  // Send a DMP info request (get status, position, etc)
  void send_info_request(T4Target target, T4InfoCommand command);

  // Send a DMP config set command
  // Accepts uint8_t to allow raw parameter values beyond the T4InfoCommand enum
  void send_config_set(uint8_t param, uint8_t value);

  // Called by BusT4Component when a packet is received
  virtual void on_packet(const T4Packet &packet) {}

  // Get the address of the motor controller we're talking to
  T4Source get_target_address() const { return target_address_; }
  void set_target_address(T4Source addr) { target_address_ = addr; }
  // Pin the target from a configured 0xAABB address; locks it against discovery.
  void set_target_address(uint16_t addr) {
    target_address_ = {static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr & 0xFF)};
    target_locked_ = true;
  }

  // Called by the component when WHO resolves the controller; ignored if pinned.
  void on_controller_resolved(T4Source addr) {
    if (!target_locked_)
      target_address_ = addr;
  }

 protected:
  BusT4Component *parent_{nullptr};
  T4Source target_address_{0x00, 0x03};  // Default motor controller address
  bool target_locked_{false};            // true when the address was configured (skip discovery override)
};

} // namespace esphome::bus_t4

// Entity classes live in subdirectories; the hub creates them directly, so pull
// their headers into this auto-included header (after BusT4Device is defined, so
// the entities' back-include of this file resolves cleanly).
#include "cover.h"
#include "switch.h"
