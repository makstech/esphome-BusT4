#pragma once

#include <vector>
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "../bus_t4.h"

namespace esphome::bus_t4 {

// Diagnostic text_sensor: a comma-separated list of the Bus-T4 devices seen on the bus
// (e.g. "0x00.03 controller, 0x0A.6D radio"), built passively from observed traffic. It
// confirms the component is actually talking to the controller / receiver.
//
// Scope note: this lists Bus-T4 endpoints. Individual BlueBus devices (photocells, keypads)
// live behind the controller and are only enumerable via the 0xD0 diagnostics, whose layout
// is not yet mapped (see .agent/PROTOCOL.md §3c); they are not listed here yet.
class BusT4DeviceList : public text_sensor::TextSensor, public BusT4Device, public Component {
 public:
  void dump_config() override;
  void on_packet(const T4Packet &packet) override;

  // Human-readable name for a Bus-T4 device-type byte (T4Device). Static for unit testing.
  static const char *device_type_name(uint8_t type);

 protected:
  // Record a (address, type) pair; returns true if the known set changed.
  bool record_device(uint16_t address, uint8_t type);
  void publish_list_();

  struct Entry {
    uint16_t address;
    uint8_t type;
  };
  std::vector<Entry> devices_;
};

}  // namespace esphome::bus_t4
