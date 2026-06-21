#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <functional>
#include <string>
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
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

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

  // --- Send primitives (used by devices via parent_, and by the debug helpers) ---
  // Verb says whether a reply is expected: `send` fires and returns; `request` waits.
  void dep_send(T4Source to, const uint8_t *msg, size_t len);  // DEP (execute-only, no reply)
  void dmp_send(T4Source to, const uint8_t *msg, size_t len);  // DMP, async (reply via on_packet)
  // DMP request: frame `msg` to `to`, send, and block up to timeout_ms for the
  // matching reply (same command byte); non-matching packets are dispatched to
  // devices normally. Returns true and fills `reply`. Don't call from a hot loop —
  // it blocks; the cover's periodic polling uses async dmp_send instead.
  bool dmp_request(T4Source to, const uint8_t *msg, size_t len, T4Packet *reply, uint32_t timeout_ms = 500);

  // --- Interactive debugging (hex wrappers, callable from YAML lambdas / API actions) ---
  // DMP request to the control unit from message bytes; returns the reply as a hex
  // string (e.g. "00.81.00.03..."), or "no reply" on timeout / "" on bad input.
  // e.g. debug_request("04 D1 99 00 00") reads IO state.
  std::string debug_request(const std::string &message_hex, uint32_t timeout_ms = 500);
  // Send byte-exact frame bytes (no framing/CRCs added — you supply the whole frame).
  std::string debug_request_raw(const std::string &frame_hex, uint32_t timeout_ms = 500);

  void set_address(const uint16_t address) {
    address_.address = static_cast<uint8_t>(address >> 8);
    address_.endpoint = static_cast<uint8_t>(address & 0xFF);
  }

  T4Source get_address() const { return address_; }

  // Register a device to receive packet callbacks
  void register_device(BusT4Device *device) { devices_.push_back(device); }

 private:
  void rxTask();
  void txTask();
  static void rxTaskThunk(void *self) { static_cast<BusT4Component *>(self)->rxTask(); }
  static void txTaskThunk(void *self) { static_cast<BusT4Component *>(self)->txTask(); }

  // Dispatch a received packet to all registered devices (skips TX echo).
  void dispatch_packet_(const T4Packet &packet);

  // Send a BusT4 break signal (~1ms low pulse) before each packet.
  // Temporarily lowers UART baud rate to produce the correct break duration.
  void send_break();

  // Parse a hex string ("04 D1 99" or "04D199") into bytes. Returns empty on
  // an odd digit count or no hex digits (callers treat empty as invalid input).
  static std::vector<uint8_t> parse_hex_(const std::string &s);

  T4Source address_;

  TaskHandle_t rxTask_ = nullptr;
  TaskHandle_t txTask_ = nullptr;

  QueueHandle_t rxQueue_ = nullptr;
  QueueHandle_t txQueue_ = nullptr;

  EventGroupHandle_t requestEvent_ = nullptr;

  std::vector<BusT4Device *> devices_;

  // Cached UART port for direct baud rate register writes during break signal.
  uart_port_t uart_num_ = UART_NUM_MAX;
};

enum { EB_REQUEST_FREE = 1, EB_REQUEST_PENDING = 2, EB_REQUEST_COMPLETE = 4 };

} // namespace esphome::bus_t4
