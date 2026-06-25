#pragma once

#include <string>
#include "esphome/core/component.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/bus_t4/bus_t4.h"

namespace esphome::bus_t4_oxi {

using bus_t4::BusT4Device;
using bus_t4::T4Packet;
using bus_t4::T4Source;

// OXI radio receiver on the Bus-T4. Reads its identity from the bus at startup
// and decodes the receiver's remote/button packets (target FOR_OXI).
class BusT4Oxi : public Component, public BusT4Device {
 public:
  BusT4Oxi() { this->role_ = bus_t4::FOR_OXI; }  // receive FOR_OXI packets, not FOR_CU

  void setup() override;
  void dump_config() override;
  void on_packet(const T4Packet &packet) override;
  float get_setup_priority() const override { return setup_priority::DATA; }  // after the bus (HARDWARE)

  void set_product_sensor(text_sensor::TextSensor *s) { product_sensor_ = s; }
  void set_hardware_sensor(text_sensor::TextSensor *s) { hardware_sensor_ = s; }
  void set_firmware_sensor(text_sensor::TextSensor *s) { firmware_sensor_ = s; }

 protected:
  void parse_oxi_packet(const T4Packet &packet);

  text_sensor::TextSensor *product_sensor_{nullptr};
  text_sensor::TextSensor *hardware_sensor_{nullptr};
  text_sensor::TextSensor *firmware_sensor_{nullptr};
  std::string product_;
  std::string hardware_;
  std::string firmware_;
};

}  // namespace esphome::bus_t4_oxi
