#pragma once

#include "esphome/components/bus_t4/bus_t4.h"
#include "esphome/components/bus_t4/t4_packet.h"
#include "esphome/core/log.h"

namespace esphome::bus_t4_control_unit {

using bus_t4::BusT4Device;
using bus_t4::T4Packet;
using bus_t4::t4_read_be;
using enum bus_t4::T4Target;
using enum bus_t4::T4Protocol;
using enum bus_t4::T4RequestType;
using enum bus_t4::T4ResponseType;
using enum bus_t4::T4Error;

// DMP payload starts after the header (7 bytes) + message header (5 bytes).
static constexpr uint8_t DATA_OFFSET = 12;

// CRTP base for entities wrapping one control-unit parameter byte: a blocking GET
// at boot, then GET/SET reply tracking (including SETs from other bus clients, so
// HA mirrors the controller). The value is read big-endian over width_ bytes;
// Derived turns it into entity state via publish_value_(raw).
template<typename Derived> class ConfigParam : public BusT4Device {
 public:
  void set_param(uint8_t param) { this->param_ = param; }
  void set_width(uint8_t width) { this->width_ = width; }

  void on_packet(const T4Packet &packet) override {
    if (packet.header.protocol != DMP || packet.message.command != this->param_)
      return;
    uint8_t flags = packet.message.dmp.flags;
    if (flags == REQ_SET) {  // another client's SET — stash, publish on confirm
      if (this->has_payload_(packet))
        this->set_pending_(this->read_raw_(packet));
      return;
    }
    if (packet.message.dmp.status != ERR_NONE)
      return;
    if (flags == RSP_SET_COMPLETE && this->has_pending_) {
      this->has_pending_ = false;
      this->derived_()->publish_value_(this->pending_raw_);
    } else if (flags == RSP_GET_COMPLETE && this->has_payload_(packet)) {
      this->derived_()->publish_value_(this->read_raw_(packet));
    }
  }

 protected:
  Derived *derived_() { return static_cast<Derived *>(this); }

  void initial_get_() {
    uint8_t message[5] = {FOR_CU, this->param_, REQ_GET, 0x00, 0x00};
    T4Packet rsp;
    if (!this->parent_->dmp_request(this->target_address_, message, sizeof(message), &rsp, 500))
      this->derived_()->mark_failed(LOG_STR("no response from controller"));
    else if (rsp.message.dmp.status == ERR_UNSUPPORTED)
      this->derived_()->mark_failed(LOG_STR("parameter not supported by controller"));
    else if (rsp.message.dmp.status == ERR_NONE && this->has_payload_(rsp))
      this->derived_()->publish_value_(this->read_raw_(rsp));
  }

  uint32_t read_raw_(const T4Packet &p) const { return t4_read_be(p, DATA_OFFSET, this->width_); }
  bool has_payload_(const T4Packet &p) const { return p.size >= DATA_OFFSET + this->width_ + 1; }
  void set_pending_(uint32_t raw) {
    this->pending_raw_ = raw;
    this->has_pending_ = true;
  }

  uint8_t param_{0};
  uint8_t width_{1};
  uint32_t pending_raw_{0};
  bool has_pending_{false};
};

}  // namespace esphome::bus_t4_control_unit
