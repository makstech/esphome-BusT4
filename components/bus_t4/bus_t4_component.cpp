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
    // Ignore packets FROM ourselves (TX echo on half-duplex bus)
    if (packet.header.from == address_) {
      ESP_LOGV(TAG, "Ignoring TX echo");
      continue;
    }

    // Ignore packets addressed TO ourselves that we sent (broadcast responses come TO us)
    // But accept packets where TO matches our address (responses to our requests)

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
  uint8_t expected_size = 0;
  // Protocol format: [BREAK] SYNC SIZE DATA[N] SIZE
  // The size byte is sent twice (start and end), no separate checksum byte.
  // Internal checksums are within the DATA portion.
  enum { WAIT_SYNC = 0, SIZE, DATA, TRAILING_SIZE } rx_state = WAIT_SYNC;

  for (;;) {
    uint8_t byte;
    if (parent_->available() && parent_->read_byte(&byte) == true) {
      switch (rx_state) {
        case WAIT_SYNC:
          // Wait for SYNC byte (0x55), ignore break bytes (0x00) and others
          if (byte == T4_SYNC) {
            rx_state = SIZE;
          }
          break;

        case SIZE:
          if (byte > 0 && byte <= 60) {
            expected_size = byte;
            packet.size = 0;
            rx_state = DATA;
          } else {
            // Invalid size, go back to waiting for sync
            rx_state = WAIT_SYNC;
          }
          break;

        case DATA:
          packet.data[packet.size++] = byte;
          if (packet.size == expected_size)
            rx_state = TRAILING_SIZE;
          break;

        case TRAILING_SIZE:
          // Verify trailing size matches
          if (byte == expected_size) {
            // Verify header checksum (CRC1): XOR of bytes 0-5 should equal byte 6
            uint8_t header_check = packet.checksum(0, 6);
            if (header_check != packet.data[6]) {
              ESP_LOGW(TAG, "Header checksum mismatch: expected 0x%02X, got 0x%02X",
                       header_check, packet.data[6]);
              rx_state = WAIT_SYNC;
              break;
            }

            // Verify payload checksum (CRC2): XOR of bytes 7 to size-2 should equal byte size-1
            if (packet.size >= 9) {  // Minimum: 7 header + 1 payload + 1 CRC2
              uint8_t payload_check = packet.data[7];
              for (size_t i = 8; i < packet.size - 1; i++) {
                payload_check ^= packet.data[i];
              }
              if (payload_check != packet.data[packet.size - 1]) {
                ESP_LOGW(TAG, "Payload checksum mismatch: expected 0x%02X, got 0x%02X",
                         payload_check, packet.data[packet.size - 1]);
                rx_state = WAIT_SYNC;
                break;
              }
            }

            // Both checksums valid - accept packet
            ESP_LOGD(TAG, "Received packet: %s (%d bytes)",
                     format_hex_pretty(packet.data, packet.size).c_str(), packet.size);
            xQueueSend(rxQueue_, &packet, portMAX_DELAY);
          } else {
            ESP_LOGW(TAG, "Trailing size mismatch: expected 0x%02X, got 0x%02X", expected_size, byte);
          }
          rx_state = WAIT_SYNC;
          break;
      }
    }
    vTaskDelay(2);
  }

  rxTask_ = nullptr;
  vTaskDelete(nullptr);
}

void BusT4Component::txTask() {
  TickType_t last_tx_time = 0;
  const TickType_t TX_MIN_INTERVAL = pdMS_TO_TICKS(100);  // Minimum 100ms between transmissions

  for (;;) {
    T4Packet packet;

    // Wait for packet with timeout (allows checking for queue items periodically)
    if (xQueueReceive(txQueue_, &packet, pdMS_TO_TICKS(10))) {
      // Ensure minimum interval between transmissions
      TickType_t now = xTaskGetTickCount();
      TickType_t elapsed = now - last_tx_time;
      if (elapsed < TX_MIN_INTERVAL) {
        vTaskDelay(TX_MIN_INTERVAL - elapsed);
      }

      ESP_LOGD(TAG, "Sending packet: %s", format_hex_pretty(packet.data, packet.size).c_str());
      parent_->write_byte(T4_BREAK);
      parent_->write_byte(T4_SYNC);
      parent_->write_byte(packet.size);
      parent_->write_array(packet.data, packet.size);
      parent_->write_byte(packet.size);
      parent_->flush();

      last_tx_time = xTaskGetTickCount();
    }
  }

  txTask_ = nullptr;
  vTaskDelete(nullptr);
}

void BusT4Component::write_raw(const uint8_t *data, size_t len) {
  // Send raw bytes directly to UART with break prefix
  // Used for debugging/testing with user-provided hex commands
  parent_->write_byte(T4_BREAK);
  parent_->write_array(data, len);
  parent_->flush();
}

} // namespace esphome::bus_t4
