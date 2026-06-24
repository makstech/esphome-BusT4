#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <functional>
#include <vector>
#include "esphome/components/uart/uart.h"
#include "t4_packet.h"

#include <driver/uart.h>
#include "esphome/components/uart/uart_component_esp_idf.h"

namespace esphome::bus_t4 {

// Forward declaration
class BusT4Device;

class BusT4Component final : public Component, public uart::UARTDevice {
 public:
  BusT4Component() = default;

  void setup() override;
  void loop() override;
  void dump_config() override;

  bool read(T4Packet *packet, TickType_t xTicksToWait) {
    if (rxQueue_ == nullptr)
      return false;
    return xQueueReceive(rxQueue_, packet, xTicksToWait);
  }

  bool write(T4Packet *packet, TickType_t xTicksToWait) {
    if (txQueue_ == nullptr)
      return false;
    return xQueueSend(txQueue_, packet, xTicksToWait);
  }

  // Send raw bytes directly to UART (for debugging/testing)
  void write_raw(const uint8_t *data, size_t len);

  void set_address(const uint16_t address) {
    address_.address = static_cast<uint8_t>(address >> 8);
    address_.endpoint = static_cast<uint8_t>(address & 0xFF);
  }

  T4Source get_address() const { return address_; }

  // Address of the motor controller, shared across devices. Defaults to the common Nice
  // endpoint {0x00, 0x03}; the cover updates it once discovery (INF_WHO) finds the unit,
  // so other devices (e.g. photocell polling) don't each have to re-run discovery.
  void set_controller_address(T4Source addr) { controller_address_ = addr; }
  T4Source get_controller_address() const { return controller_address_; }

  // Register a device to receive packet callbacks
  void register_device(BusT4Device *device) { devices_.push_back(device); }

  // When enabled, every received packet is logged with a classification guess so that
  // unmapped BlueBus events (photocells, keypads) can be captured. See .agent/PROTOCOL.md.
  void set_debug_unknown_packets(bool enable) { debug_unknown_packets_ = enable; }

 private:
  void rxTask();
  // Log a received packet with a human-readable type guess (debug_unknown_packets).
  void log_packet_classification(const T4Packet &packet);
  void txTask();
  static void rxTaskThunk(void *self) { static_cast<BusT4Component *>(self)->rxTask(); }
  static void txTaskThunk(void *self) { static_cast<BusT4Component *>(self)->txTask(); }

  // Send a BusT4 break signal (~1ms low pulse) before each packet.
  // Temporarily lowers UART baud rate to produce the correct break duration.
  void send_break();

  T4Source address_;
  T4Source controller_address_{0x00, 0x03};

  TaskHandle_t rxTask_ = nullptr;
  TaskHandle_t txTask_ = nullptr;

  QueueHandle_t rxQueue_ = nullptr;
  QueueHandle_t txQueue_ = nullptr;

  EventGroupHandle_t requestEvent_ = nullptr;

  std::vector<BusT4Device *> devices_;

  bool debug_unknown_packets_ = false;

  // Cached UART port for direct baud rate register writes during break signal.
  uart_port_t uart_num_ = UART_NUM_MAX;
};

enum { EB_REQUEST_FREE = 1, EB_REQUEST_PENDING = 2, EB_REQUEST_COMPLETE = 4 };

} // namespace esphome::bus_t4
