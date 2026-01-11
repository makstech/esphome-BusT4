#include "bus_t4_component.h"
#include "bus_t4.h"

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include "t4_packet.h"

namespace esphome::bus_t4 {

static const char *TAG = "bus_t4";

void BusT4Component::setup() {
  rxQueue_ = xQueueCreate(32, sizeof(T4Packet));
  txQueue_ = xQueueCreate(32, sizeof(T4Packet));

  requestEvent_ = xEventGroupCreate();
  xEventGroupSetBits(requestEvent_, EB_REQUEST_FREE);

  xTaskCreate(rxTaskThunk, "bus_t4_rx", 8192, this, 10, &rxTask_);
  xTaskCreate(txTaskThunk, "bus_t4_tx", 8192, this, 10, &txTask_);
}

void BusT4Component::loop() {
  // Process received packets and dispatch to registered devices
  T4Packet packet;
  while (xQueueReceive(rxQueue_, &packet, 0)) {
    // Dispatch to all registered devices
    for (auto *device : devices_) {
      device->on_packet(packet);
    }
  }
}

void BusT4Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BusT4:");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X%02X", address_.address, address_.endpoint);
}

void BusT4Component::rxTask() {
  T4Packet packet;
  uint8_t checksum = 0;
  uint8_t expected_size = 0;
  enum { WAIT = 0, SYNC, SIZE, DATA, CHECKSUM, RESET } rx_state = WAIT;

  for (;;) {
    uint8_t byte;
    if (parent_->available() && parent_->read_byte(&byte) == true) {
      switch (rx_state) {
        case WAIT:
          rx_state = (byte == T4_BREAK) ? SYNC : RESET;
          break;

        case SYNC:
          rx_state = (byte == T4_SYNC) ? SIZE : RESET;
          break;

        case SIZE:
          if (byte <= 60) {
            expected_size = byte;
            packet.size = 0;
            checksum = 0;
            rx_state = DATA;
          } else {
            rx_state = RESET;
          }
          break;

        case DATA:
          packet.data[packet.size++] = byte;
          checksum ^= byte;
          if (packet.size == expected_size)
            rx_state = CHECKSUM;
          break;

        case CHECKSUM:
          if (byte == checksum) {
            ESP_LOGD(TAG, "Received packet: %s (%d bytes)", 
                     format_hex_pretty(packet.data, packet.size).c_str(), packet.size);
            xQueueSend(rxQueue_, &packet, portMAX_DELAY);
          } else {
            ESP_LOGW(TAG, "Checksum mismatch: expected 0x%02X, got 0x%02X", checksum, byte);
          }
          rx_state = RESET;
          break;

        case RESET:
          packet.size = 0;
          expected_size = 0;
          checksum = 0;
          rx_state = WAIT;
          break;
      }
    } else {
      // Timeout - reset state if we were in the middle of receiving
      if (rx_state != WAIT) {
        rx_state = RESET;
      }
    }
    vTaskDelay(2);
  }

  rxTask_ = nullptr;
  vTaskDelete(nullptr);
}

void BusT4Component::txTask() {
  for (;;) {
    T4Packet packet;

    if (xQueueReceive(txQueue_, &packet, 0)) {
      ESP_LOGD(TAG, "Sending packet: %s", format_hex_pretty(packet.data, packet.size).c_str());
      parent_->write_byte(T4_BREAK);
      parent_->write_byte(T4_SYNC);
      parent_->write_byte(packet.size);
      parent_->write_array(packet.data, packet.size);
      parent_->write_byte(packet.size);
      parent_->flush();
    }
    vTaskDelay(2);
  }

  txTask_ = nullptr;
  vTaskDelete(nullptr);
}

} // namespace esphome::bus_t4
