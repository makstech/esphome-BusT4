#include "keypad_event.h"
#include "esphome/core/log.h"

namespace esphome::bus_t4 {

static const char *const TAG = "bus_t4.event";

const char *BusT4KeypadEvent::command_to_event_type(uint8_t command) {
  switch (command) {
    case CMD_STEP: return "step_by_step";
    case CMD_STOP: return "stop";
    case CMD_OPEN: return "open";
    case CMD_CLOSE: return "close";
    case CMD_OPEN_PARTIAL_1: return "partial_1";
    case CMD_OPEN_PARTIAL_2: return "partial_2";
    case CMD_OPEN_PARTIAL_3: return "partial_3";
    default: return nullptr;
  }
}

void BusT4KeypadEvent::dump_config() { LOG_EVENT("", "Bus T4 Keypad Event", this); }

void BusT4KeypadEvent::on_packet(const T4Packet &packet) {
  // Only RUN packets carrying a command echo (data[9] >= 0x80) are commands; everything
  // else (movement statuses, info responses) is ignored. See .agent/PROTOCOL.md §2.
  if (!packet.is_run_packet())
    return;

  uint8_t command = packet.run_command_echo();
  if (command == 0)
    return;  // movement status, not a command

  const char *event_type = command_to_event_type(command);
  if (event_type == nullptr)
    return;  // a command we don't surface (e.g. a security block/release)

  ESP_LOGD(TAG, "Keypad/command event: %s (cmd=0x%02X)", event_type, command);
  this->trigger(event_type);
}

}  // namespace esphome::bus_t4
