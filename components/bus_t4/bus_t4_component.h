#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include "esphome/components/uart/uart.h"
#include "t4_packet.h"

namespace esphome::bus_t4 {

class BusT4Component final : public Component, public uart::UARTDevice {
 public:
  BusT4Component() = default;

  void setup() override;
  void dump_config() override;

  bool read(T4Packet *packet, TickType_t xTicksToWait) { return xQueueReceive(rxQueue_, packet, xTicksToWait); }

  bool write(T4Packet *packet, TickType_t xTicksToWait) { return xQueueSend(txQueue_, packet, xTicksToWait); }

  void set_address(const uint16_t address) {
    address_.address = static_cast<uint8_t>(address >> 8);
    address_.endpoint = static_cast<uint8_t>(address & 0xFF);
  }

  T4Source get_address() const { return address_; }

 private:
  void rxTask();
  void txTask();
  static void rxTaskThunk(void *self) { static_cast<BusT4Component *>(self)->rxTask(); }
  static void txTaskThunk(void *self) { static_cast<BusT4Component *>(self)->txTask(); }

  T4Source address_;

  TaskHandle_t rxTask_ = nullptr;
  TaskHandle_t txTask_ = nullptr;

  QueueHandle_t rxQueue_ = nullptr;
  QueueHandle_t txQueue_ = nullptr;

  EventGroupHandle_t requestEvent_ = nullptr;
};

enum { EB_REQUEST_FREE = 1, EB_REQUEST_PENDING = 2, EB_REQUEST_COMPLETE = 4 };

} // namespace esphome::bus_t4
