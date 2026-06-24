#pragma once

#include "esphome/components/event/event.h"
#include "esphome/core/component.h"
#include "../bus_t4.h"

namespace esphome::bus_t4 {

// Fires an ESPHome event entity every time the control unit reports an executed command
// on Bus-T4 (step-by-step / stop / open / close / partial 1-3). The command is carried by
// the RUN packet's command echo; see .agent/PROTOCOL.md §2.
//
// Source attribution is not possible: a wired keypad, an OXI remote, a hardwired input and
// our own ESP all produce the same packet. The event reports *which* command, not *who*.
class BusT4KeypadEvent : public event::Event, public BusT4Device, public Component {
 public:
  void dump_config() override;
  void on_packet(const T4Packet &packet) override;

  // Map a Bus-T4 command value to its event-type string, or nullptr if it is not one of
  // the commands this entity reports. Pure/static so it can be unit-tested on the host.
  static const char *command_to_event_type(uint8_t command);
};

}  // namespace esphome::bus_t4
