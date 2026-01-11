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
  
  // Send a DEP control command (open/close/stop etc)
  void send_cmd(T4Command cmd);
  
  // Send a DMP info request (get status, position, etc)
  void send_info_request(T4Target target, T4InfoCommand command);
  
  // Called by BusT4Component when a packet is received
  virtual void on_packet(const T4Packet &packet) {}
  
  // Get the address of the motor controller we're talking to
  T4Source get_target_address() const { return target_address_; }
  void set_target_address(T4Source addr) { target_address_ = addr; }
  
 protected:
  BusT4Component *parent_{nullptr};
  T4Source target_address_{0x00, 0x03};  // Default motor controller address
};

} // namespace esphome::bus_t4
