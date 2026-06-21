#include "bus_t4.h"

namespace esphome::bus_t4 {

void BusT4Device::send_cmd(T4Command cmd) {
  send_cmd(cmd, OVIEW);
}

void BusT4Device::send_cmd(T4Command cmd, T4Device device) {
  // DEP packet structure: [device] [command] [cmd_value] [offset]
  uint8_t message[4] = { device, RUN, cmd, 0x64 };
  parent_->dep_send(target_address_, message, sizeof(message));
}

void BusT4Device::send_info_request(T4Target target, T4InfoCommand command) {
  // DMP GET packet structure: [target] [command] [request_type] [offset] [length]
  uint8_t message[5] = { target, command, REQ_GET, 0x00, 0x00 };
  parent_->dmp_send(target_address_, message, sizeof(message));
}

void BusT4Device::send_config_set(uint8_t param, uint8_t value) {
  // DMP SET packet structure: [target] [param] [request_type] [offset] [length] [value]
  uint8_t message[6] = { FOR_CU, param, REQ_SET, 0x00, 0x01, value };
  parent_->dmp_send(target_address_, message, sizeof(message));
}

} // namespace esphome::bus_t4
