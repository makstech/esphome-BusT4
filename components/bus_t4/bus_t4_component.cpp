#include "bus_t4_component.h"
#include "bus_t4.h"

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include "t4_packet.h"

#include <esp_rom_sys.h>

namespace esphome::bus_t4 {

static const char *TAG = "bus_t4";

// Bumped when tagging a release; checked against the tag by CI
static const char *VERSION = "1.2.2";

// Break signal baud rate: sending 0x00 at 9200 baud produces a ~1ms low pulse
// on a 19200 baud bus. Reference: pruwait/Nice_BusT4 nice-bust4.h
static constexpr uint32_t T4_BAUD_BREAK = 9200;

// Observed request-to-reply latency on a shared bus reaches ~55ms
static constexpr uint32_t BUS_IDLE_MS = 100;

// Bounded so a busy bus cannot starve the transmit path
static constexpr uint8_t RX_DRAIN_LIMIT = 64;

static constexpr TickType_t TX_MIN_INTERVAL = pdMS_TO_TICKS(100);

void BusT4Component::setup() {
  rxQueue_ = xQueueCreate(32, sizeof(T4Packet));
  if (rxQueue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create RX queue");
    this->mark_failed();
    return;
  }

  txQueue_ = xQueueCreate(32, sizeof(T4Frame));
  if (txQueue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create TX queue");
    vQueueDelete(rxQueue_);
    rxQueue_ = nullptr;
    this->mark_failed();
    return;
  }

  if (xTaskCreate(busTaskThunk, "bus_t4", 8192, this, 10, &busTask_) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create bus task");
    this->mark_failed();
    return;
  }

  // Cache UART port number for direct baud rate changes during break signal.
  auto *idf_uart = static_cast<uart::IDFUARTComponent *>(parent_);
  uart_num_ = static_cast<uart_port_t>(idf_uart->get_hw_serial_number());
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

    // Log foreign destinations without dropping them yet
    if (!t4_addressed_to(packet.header.to, address_)) {
      ESP_LOGD(TAG, "Packet addressed to 0x%02X.%02X, not us: %s",
               packet.header.to.address, packet.header.to.endpoint,
               format_hex_pretty(packet.data, packet.size).c_str());
    }

    // Dispatch to all registered devices
    for (auto *device : devices_) {
      device->on_packet(packet);
    }
  }
}

void BusT4Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BusT4:");
  ESP_LOGCONFIG(TAG, "  Version: %s", VERSION);
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X%02X", address_.address, address_.endpoint);
}

void BusT4Component::busTask() {
  T4Packet packet;
  uint8_t expected_size = 0;
  // Protocol format: [BREAK] SYNC SIZE DATA[N] SIZE
  // The size byte is sent twice (start and end), no separate checksum byte.
  // Internal checksums are within the DATA portion.
  enum { WAIT_SYNC = 0, SIZE, DATA, TRAILING_SIZE } rx_state = WAIT_SYNC;
  uint32_t last_rx_ms = 0;
  TickType_t last_tx_time = 0;

  for (;;) {
    uint8_t byte;
    uint8_t drained = 0;
    while (drained < RX_DRAIN_LIMIT && parent_->available() && parent_->read_byte(&byte) == true) {
      drained++;
      last_rx_ms = millis();
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
              ESP_LOGW(TAG, "Header checksum mismatch: expected 0x%02X, got 0x%02X: %s",
                       header_check, packet.data[6],
                       format_hex_pretty(packet.data, packet.size).c_str());
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
                ESP_LOGW(TAG, "Payload checksum mismatch: expected 0x%02X, got 0x%02X: %s",
                         payload_check, packet.data[packet.size - 1],
                         format_hex_pretty(packet.data, packet.size).c_str());
                rx_state = WAIT_SYNC;
                break;
              }
            }

            // Both checksums valid - accept packet
            ESP_LOGD(TAG, "Received packet: %s (%d bytes)",
                     format_hex_pretty(packet.data, packet.size).c_str(), packet.size);
            if (!xQueueSend(rxQueue_, &packet, pdMS_TO_TICKS(100))) {
              ESP_LOGW(TAG, "RX queue full, dropping packet");
            }
          } else {
            ESP_LOGW(TAG, "Trailing size mismatch: expected 0x%02X, got 0x%02X", expected_size, byte);
          }
          rx_state = WAIT_SYNC;
          break;
      }
    }

    // Transmit only between frames, and only once the bus has gone quiet
    if (rx_state == WAIT_SYNC && millis() - last_rx_ms >= BUS_IDLE_MS) {
      T4Frame frame;
      if (xQueueReceive(txQueue_, &frame, 0)) {
        TickType_t elapsed = xTaskGetTickCount() - last_tx_time;
        if (elapsed < TX_MIN_INTERVAL) {
          vTaskDelay(TX_MIN_INTERVAL - elapsed);
        }

        if (frame.raw) {
          ESP_LOGD(TAG, "Sending raw: %s", format_hex_pretty(frame.data, frame.size).c_str());
        } else {
          ESP_LOGD(TAG, "Sending packet: %s",
                   format_hex_pretty(frame.data + 2, frame.size - 3).c_str());
        }
        send_break();
        parent_->write_array(frame.data, frame.size);
        parent_->flush();

        last_tx_time = xTaskGetTickCount();
      }
    }

    vTaskDelay(1);
  }

  busTask_ = nullptr;
  vTaskDelete(nullptr);
}

bool BusT4Component::queue_frame(const T4Frame &frame, TickType_t xTicksToWait) {
  if (txQueue_ == nullptr)
    return false;
  return xQueueSend(txQueue_, &frame, xTicksToWait);
}

bool BusT4Component::write(T4Packet *packet, TickType_t xTicksToWait) {
  if (packet == nullptr || packet->size == 0 || packet->size > sizeof(T4Packet::data))
    return false;

  T4Frame frame;
  frame.data[0] = T4_SYNC;
  frame.data[1] = packet->size;
  std::copy_n(packet->data, packet->size, frame.data + 2);
  frame.data[packet->size + 2] = packet->size;
  frame.size = packet->size + 3;
  return queue_frame(frame, xTicksToWait);
}

void BusT4Component::write_raw(const uint8_t *data, size_t len) {
  // User-provided bytes are already framed, so they go out verbatim
  if (len == 0 || len > sizeof(T4Frame::data)) {
    ESP_LOGW(TAG, "Raw frame of %u bytes rejected", static_cast<unsigned>(len));
    return;
  }

  T4Frame frame;
  frame.raw = true;
  frame.size = static_cast<uint8_t>(len);
  std::copy_n(data, len, frame.data);
  queue_frame(frame, 0);
}

void BusT4Component::send_break() {
  // BusT4 protocol requires a break signal (~1ms low pulse) before each packet.
  // Achieved by sending 0x00 at a lower baud rate (9200 instead of 19200).
  // Reference: pruwait/Nice_BusT4 send_array_cmd() implementation.
  if (uart_num_ < UART_NUM_MAX) {
    uint32_t work_baud = parent_->get_baud_rate();
    uart_set_baudrate(uart_num_, T4_BAUD_BREAK);
    parent_->write_byte(T4_BREAK);
    parent_->flush();
    esp_rom_delay_us(100);
    uart_set_baudrate(uart_num_, work_baud);
  } else {
    // UART port not cached — fall back to simple break byte
    parent_->write_byte(T4_BREAK);
    parent_->flush();
  }
}

} // namespace esphome::bus_t4
