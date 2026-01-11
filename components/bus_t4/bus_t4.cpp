#include "bus_t4.h"

namespace esphome::bus_t4 {

void BusT4Device::send_cmd(T4Command cmd) {
  // DEP packet structure: [device] [command] [cmd_value] [offset]
  uint8_t message[4] = { OVIEW, RUN, cmd, 0x64 };
  T4Packet packet(target_address_, parent_->get_address(), DEP, message, sizeof(message));
  write(&packet, 0);
}

void BusT4Device::send_info_request(T4Target target, T4InfoCommand command) {
  // DMP GET packet structure: [target] [command] [request_type] [offset] [length]
  uint8_t message[5] = { target, command, REQ_GET, 0x00, 0x00 };
  T4Packet packet(target_address_, parent_->get_address(), DMP, message, sizeof(message));
  write(&packet, 0);
}

} // namespace esphome::bus_t4
