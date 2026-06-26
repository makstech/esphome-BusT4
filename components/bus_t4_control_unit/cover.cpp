#include "cover.h"
#include "bus_t4_control_unit.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <cmath>
#include <string>
#include <algorithm>

namespace esphome::bus_t4_control_unit {

static const char *TAG = "bus_t4.cover";

cover::CoverTraits BusT4Cover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_is_assumed_state(false);  // We track actual state now
  traits.set_supports_position(true);
  traits.set_supports_tilt(false);
  traits.set_supports_toggle(false);
  traits.set_supports_stop(true);
  return traits;
}

void BusT4Cover::setup() {
  // Initialize with unknown position
  this->position = cover::COVER_OPEN;
  this->current_operation = cover::COVER_OPERATION_IDLE;

  // Load learned durations from flash
  timing_.load();
}

void BusT4Cover::loop() {
  uint32_t now = millis();

  // Initialization state machine with exponential backoff
  // Step 0 (discovery) uses longer intervals to avoid flooding the bus
  // Other steps use shorter intervals since we're talking to a known device
  if (!init_ok_) {
    uint32_t retry_interval = (init_step_ == 0) ? get_discovery_interval() : 500;
    if (now - last_init_attempt_ > retry_interval) {
      last_init_attempt_ = now;
      init_device();
    }
  }

  // Periodic status refresh every 15 seconds
  // This helps recover from missed packets and keeps state in sync
  if (init_ok_ && (now - last_status_refresh_ > 15000)) {
    last_status_refresh_ = now;

    // If idle, request current status
    if (current_operation == cover::COVER_OPERATION_IDLE) {
      ESP_LOGV(TAG, "Periodic status refresh");
      request_status();
      // Also request position if we have encoder support
      if (has_encoder_) {
        request_position();
      }
    }
  }

  // Time-based position tracking during movement
  if (timing_.active() && current_operation != cover::COVER_OPERATION_IDLE) {
    Direction dir = (current_operation == cover::COVER_OPERATION_OPENING) ? Direction::OPEN : Direction::CLOSE;
    bool encoder_fresh = !force_estimated_position_ && has_encoder_ && (now - last_encoder_update_ < 2000);
    if (!encoder_fresh) {
      this->position = timing_.estimate(now, dir);
    }

    // Rate-limit position publishing
    if (now - last_position_publish_ >= position_report_interval_) {
      last_position_publish_ = now;
      publish_state_if_changed();
    }

    // Poll encoder position during movement, only if the unit has an encoder.
    if (has_encoder_ && init_ok_ && !force_estimated_position_) {
      if (now - last_position_update_ >= position_report_interval_) {
        last_position_update_ = now;
        request_position();
      }
    }

    // Check if we've reached target position (always check, don't rate-limit)
    if (target_position_ >= 0.0f) {
      bool target_reached = false;
      if (current_operation == cover::COVER_OPERATION_OPENING && this->position >= target_position_) {
        target_reached = true;
      } else if (current_operation == cover::COVER_OPERATION_CLOSING && this->position <= target_position_) {
        target_reached = true;
      }

      if (target_reached) {
        ESP_LOGI(TAG, "Target position %.0f%% reached, stopping", target_position_ * 100);
        send_cmd(CMD_STOP);
        target_position_ = -1.0f;
      }
    }
  }
}

void BusT4Cover::dump_config() {
  LOG_COVER("", "Bus T4 Cover", this);
  ESP_LOGCONFIG(TAG, "  Initialized: %s", init_ok_ ? "Yes" : "No");
  ESP_LOGCONFIG(TAG, "  Auto-learn timing: %s", timing_.auto_learn() ? "Yes" : "No");
  ESP_LOGCONFIG(TAG, "  Open duration: %.1fs", timing_.open_duration() / 1000.0f);
  ESP_LOGCONFIG(TAG, "  Close duration: %.1fs", timing_.close_duration() / 1000.0f);
  if (init_ok_) {
    const char *type_str = "Unknown";
    switch (motor_type_) {
      case MOTOR_SLIDING: type_str = "Sliding"; break;
      case MOTOR_SECTIONAL: type_str = "Sectional"; break;
      case MOTOR_SWING: type_str = "Swing"; break;
      case MOTOR_BARRIER: type_str = "Barrier"; break;
      case MOTOR_UPANDOVER: type_str = "Up-and-over"; break;
    }
    ESP_LOGCONFIG(TAG, "  Motor type: %s", type_str);
    ESP_LOGCONFIG(TAG, "  Position range: %d - %d", pos_min_, pos_max_);

    // Device-specific modes
    if (is_walky_) {
      ESP_LOGCONFIG(TAG, "  Mode: Walky (1-byte position)");
    }

    // Position tracking mode
    if (force_estimated_position_) {
      ESP_LOGCONFIG(TAG, "  Position source: Time-based estimation (forced)");
    } else if (has_encoder_) {
      ESP_LOGCONFIG(TAG, "  Position source: Encoder (primary)");
    } else {
      ESP_LOGCONFIG(TAG, "  Position source: Time-based estimation");
    }

  }
}

void BusT4Cover::control(const cover::CoverCall &call) {
  if (call.get_stop()) {
    ESP_LOGD(TAG, "Stopping cover");
    send_cmd(CMD_STOP);
    return;
  }

  if (call.get_position().has_value()) {
    float pos = *call.get_position();

    if (pos == cover::COVER_OPEN) {
      if (current_operation != cover::COVER_OPERATION_OPENING) {
        ESP_LOGD(TAG, "Opening cover");
        send_cmd(CMD_OPEN);
      }
    } else if (pos == cover::COVER_CLOSED) {
      if (current_operation != cover::COVER_OPERATION_CLOSING) {
        ESP_LOGD(TAG, "Closing cover");
        send_cmd(CMD_CLOSE);
      }
    } else {
      // Partial position requested - calculate target encoder position
      // For now, we just open/close based on direction
      target_position_ = pos;
      if (pos > this->position) {
        if (current_operation != cover::COVER_OPERATION_OPENING) {
          ESP_LOGD(TAG, "Opening cover to %.0f%%", pos * 100);
          send_cmd(CMD_OPEN);
        }
      } else {
        if (current_operation != cover::COVER_OPERATION_CLOSING) {
          ESP_LOGD(TAG, "Closing cover to %.0f%%", pos * 100);
          send_cmd(CMD_CLOSE);
        }
      }
    }
  }
}

void BusT4Cover::on_packet(const T4Packet &packet) {
  ESP_LOGV(TAG, "Received packet from 0x%02X.%02X, protocol=%d",
           packet.header.from.address, packet.header.from.endpoint, packet.header.protocol);

  if (packet.header.protocol == DEP) {
    parse_dep_packet(packet);
  } else if (packet.header.protocol == DMP) {
    parse_dmp_packet(packet);
  }
}

void BusT4Cover::parse_dep_packet(const T4Packet &packet) {
  // DEP packets contain command responses and status updates
  // Structure after header: [device] [command] [status/data...]
  // Data layout:
  //   data[7] = device
  //   data[8] = command
  //   data[9+] = payload

  if (packet.size < 10) return;

  uint8_t device = packet.message.device;
  uint8_t command = packet.message.command;

  ESP_LOGD(TAG, "DEP packet: device=0x%02X, command=0x%02X, size=%d", device, command, packet.size);
  ESP_LOGV(TAG, "DEP raw: %02X %02X %02X %02X %02X %02X",
           packet.data[7], packet.data[8], packet.data[9], packet.data[10],
           packet.data[11], packet.data[12]);

  // DEP payload starts at data[9]
  const uint8_t DEP_DATA_OFFSET = 9;

  // Check for RUN command responses (command byte may have 0x80 subtracted)
  if (command == RUN || command == (RUN - 0x80) || command == OP_STOP || command == OP_OPEN || command == OP_CLOSE) {
    uint8_t status = packet.data[DEP_DATA_OFFSET];

    ESP_LOGD(TAG, "RUN/Command response: status=0x%02X", status);

    // Parse operation status
    switch (status) {
      case STA_OPENING:
      case STA_OPENING_ALT:  // Alternate code used by Road 400 and others
        ESP_LOGI(TAG, "Gate opening");
        if (current_operation != cover::COVER_OPERATION_OPENING) {
          timing_.begin(Direction::OPEN, this->position);
        }
        current_operation = cover::COVER_OPERATION_OPENING;
        break;

      case STA_CLOSING:
      case STA_CLOSING_ALT:  // Alternate code used by Road 400 and others
        ESP_LOGI(TAG, "Gate closing");
        if (current_operation != cover::COVER_OPERATION_CLOSING) {
          timing_.begin(Direction::CLOSE, this->position);
        }
        current_operation = cover::COVER_OPERATION_CLOSING;
        break;

      case STA_OPENED:
        ESP_LOGI(TAG, "Gate fully open");
        timing_.finish(Direction::OPEN);
        current_operation = cover::COVER_OPERATION_IDLE;
        this->position = cover::COVER_OPEN;
        target_position_ = -1.0f;
        break;

      case STA_CLOSED:
        ESP_LOGI(TAG, "Gate fully closed");
        timing_.finish(Direction::CLOSE);
        current_operation = cover::COVER_OPERATION_IDLE;
        this->position = cover::COVER_CLOSED;
        target_position_ = -1.0f;
        break;

      case STA_STOPPED:
      case OP_STOPPED:
        ESP_LOGI(TAG, "Gate stopped (mid-movement)");
        timing_.cancel();  // Stopped mid-movement, can't learn
        last_operation_ = current_operation;
        current_operation = cover::COVER_OPERATION_IDLE;
        target_position_ = -1.0f;
        // Don't trust time-based position at endpoints - it may have "completed" falsely
        // Clamp to mid-range until confirmation
        if (this->position <= 0.02f) {
          this->position = 0.05f;  // Show as slightly open until confirmed
          ESP_LOGD(TAG, "Position clamped from 0%% to 5%% pending confirmation");
        } else if (this->position >= 0.98f) {
          this->position = 0.95f;  // Show as slightly closed until confirmed
          ESP_LOGD(TAG, "Position clamped from 100%% to 95%% pending confirmation");
        }
        // Request actual status to confirm position
        request_status_confirmation();
        break;  // Publish now with clamped position, update when confirmed

      case STA_ENDTIME:
        ESP_LOGI(TAG, "Gate operation ended (timeout)");
        timing_.cancel();  // Timeout, can't learn
        last_operation_ = current_operation;
        current_operation = cover::COVER_OPERATION_IDLE;
        target_position_ = -1.0f;
        // Don't trust time-based position at endpoints
        if (this->position <= 0.02f) {
          this->position = 0.05f;
        } else if (this->position >= 0.98f) {
          this->position = 0.95f;
        }
        // Request actual status to confirm position
        request_status_confirmation();
        break;  // Publish now with clamped position, update when confirmed

      case STA_PART_OPENED:
        ESP_LOGI(TAG, "Gate partially open");
        timing_.cancel();  // Partial movement, can't learn
        current_operation = cover::COVER_OPERATION_IDLE;
        target_position_ = -1.0f;
        break;
    }

    publish_state_if_changed();
  }

  // Check for STA (status during movement) packets
  if (command == STA) {
    uint8_t status = packet.data[DEP_DATA_OFFSET];
    uint16_t pos = 0;

    // Position data handling varies by device type
    if (is_walky_) {
      // Walky uses 1-byte position
      if (packet.size >= 11) {
        pos = packet.data[DEP_DATA_OFFSET + 1];
      }
    } else {
      // Standard 2-byte big-endian position
      if (packet.size >= 12) {
        pos = (packet.data[DEP_DATA_OFFSET + 1] << 8) | packet.data[DEP_DATA_OFFSET + 2];
      }
    }

    ESP_LOGD(TAG, "STA packet: status=0x%02X, pos=%d", status, pos);

    switch (status) {
      case STA_OPENING:
      case STA_OPENING_ALT:  // Alternate code used by Road 400 and others
        current_operation = cover::COVER_OPERATION_OPENING;
        break;
      case STA_CLOSING:
      case STA_CLOSING_ALT:  // Alternate code used by Road 400 and others
        current_operation = cover::COVER_OPERATION_CLOSING;
        break;
      case STA_OPENED:
        current_operation = cover::COVER_OPERATION_IDLE;
        this->position = cover::COVER_OPEN;
        break;
      case STA_CLOSED:
        current_operation = cover::COVER_OPERATION_IDLE;
        this->position = cover::COVER_CLOSED;
        break;
      case STA_STOPPED:
        current_operation = cover::COVER_OPERATION_IDLE;
        break;
    }

    if (pos > 0) {
      update_position(pos);
    }
    publish_state_if_changed();
  }
}

void BusT4Cover::parse_dmp_packet(const T4Packet &packet) {
  // DMP packets contain info responses
  // Structure: [device] [command] [flags] [sequence] [status] [data...]

  if (packet.size < 14) return;

  uint8_t device = packet.message.device;
  uint8_t command = packet.message.command;
  uint8_t flags = packet.message.dmp.flags;
  uint8_t sequence = packet.message.dmp.sequence;
  uint8_t status = packet.message.dmp.status;

  ESP_LOGD(TAG, "DMP packet: dev=0x%02X, cmd=0x%02X, flags=0x%02X, seq=%d, status=0x%02X",
           device, command, flags, sequence, status);

  // Log raw data for debugging
  ESP_LOGV(TAG, "DMP raw data: %02X %02X %02X %02X %02X %02X %02X",
           packet.data[8], packet.data[9], packet.data[10], packet.data[11],
           packet.data[12], packet.data[13], packet.data[14]);

  // Check for errors
  if (status != ERR_NONE) {
    ESP_LOGW(TAG, "DMP error: 0x%02X", status);
    return;
  }

  // Only process GET responses (0x19 = complete, 0x18 = incomplete)
  if (flags != RSP_GET_COMPLETE && flags != RSP_GET_INCOMPLETE) {
    ESP_LOGV(TAG, "Ignoring non-GET response: flags=0x%02X", flags);
    return;
  }

  // Handle incomplete responses - request continuation with new offset
  // The sequence byte contains the next offset to request
  if (flags == RSP_GET_INCOMPLETE) {
    ESP_LOGD(TAG, "Incomplete response for cmd=0x%02X, requesting continuation at offset %d",
             command, sequence);
    // Build continuation request with the next offset
    uint8_t cont_msg[5] = { device, command, REQ_GET, sequence, 0x00 };
    T4Packet cont_packet(target_address_, parent_->get_address(), DMP, cont_msg, sizeof(cont_msg));
    write(&cont_packet, 0);
    // Still process the partial data we received
  }

  // Process based on command type
  // DMP payload data starts at data[12] (after header + message header)
  const uint8_t DATA_OFFSET = 12;

  switch (command) {
    case INF_TYPE: {
      // Motor type response
      uint8_t mtype = packet.data[DATA_OFFSET];
      motor_type_ = static_cast<T4MotorType>(mtype);
      ESP_LOGI(TAG, "Motor type: 0x%02X", mtype);
      break;
    }


    case INF_STATUS: {
      // Gate status response
      uint8_t gate_status = packet.data[DATA_OFFSET];
      ESP_LOGI(TAG, "Gate status: 0x%02X%s", gate_status,
               awaiting_confirmation_ ? " (confirmation)" : "");

      switch (gate_status) {
        case STA_OPENED:
          current_operation = cover::COVER_OPERATION_IDLE;
          this->position = cover::COVER_OPEN;
          if (awaiting_confirmation_) {
            ESP_LOGI(TAG, "Confirmed: gate is fully open");
          }
          break;
        case STA_CLOSED:
          current_operation = cover::COVER_OPERATION_IDLE;
          this->position = cover::COVER_CLOSED;
          if (awaiting_confirmation_) {
            ESP_LOGI(TAG, "Confirmed: gate is fully closed");
          }
          break;
        case STA_OPENING:
          current_operation = cover::COVER_OPERATION_OPENING;
          break;
        case STA_CLOSING:
          current_operation = cover::COVER_OPERATION_CLOSING;
          break;
        case STA_STOPPED:
        case STA_UNKNOWN:
        case STA_PART_OPENED:
          current_operation = cover::COVER_OPERATION_IDLE;
          if (awaiting_confirmation_) {
            // Gate stopped mid-movement - position is uncertain
            // Keep the time-based estimate but log it
            ESP_LOGW(TAG, "Gate stopped at unknown position (estimated %.0f%%)", this->position * 100);
          }
          break;
      }
      awaiting_confirmation_ = false;
      publish_state_if_changed();
      break;
    }

    case INF_CUR_POS: {
      // Current position response
      // Walky uses 1-byte position, others use 2 bytes big-endian
      uint16_t pos;
      if (is_walky_) {
        pos = packet.data[DATA_OFFSET];
        ESP_LOGD(TAG, "Current position (Walky 1-byte): %d", pos);
      } else {
        pos = (packet.data[DATA_OFFSET] << 8) | packet.data[DATA_OFFSET + 1];
        ESP_LOGD(TAG, "Current position: %d", pos);
      }
      update_position(pos);
      break;
    }

    case INF_POS_MAX: {
      // Open position (2 bytes, big endian). 0xFFFF = not calibrated.
      uint16_t pos = (packet.data[DATA_OFFSET] << 8) | packet.data[DATA_OFFSET + 1];
      if (pos > 0 && pos != 0xFFFF) {
        pos_max_ = pos;
        ESP_LOGI(TAG, "Open position: %d", pos_max_);
      }
      break;
    }

    case INF_POS_MIN: {
      // Close position (2 bytes, big endian). 0xFFFF = not calibrated; keep default 0.
      uint16_t pos = (packet.data[DATA_OFFSET] << 8) | packet.data[DATA_OFFSET + 1];
      if (pos != 0xFFFF) {
        pos_min_ = pos;
        ESP_LOGI(TAG, "Close position: %d", pos_min_);
      }
      break;
    }

    case INF_MAX_OPN: {
      // Maximum encoder position
      // Walky uses 1-byte, others use 2 bytes big-endian
      uint16_t pos;
      if (is_walky_) {
        pos = packet.data[DATA_OFFSET];
      } else {
        pos = (packet.data[DATA_OFFSET] << 8) | packet.data[DATA_OFFSET + 1];
      }
      ESP_LOGI(TAG, "Max encoder position: %d", pos);
      if (pos > 0 && pos != 0xFFFF) {
        pos_max_ = pos;
        has_encoder_ = true;  // a valid extent means the unit reports encoder position
      }
      break;
    }

  }
}

void BusT4Cover::init_device() {
  // State machine for gradual initialization
  // Each call advances one step to avoid flooding the bus

  switch (init_step_) {
    case 0:
      if (cu_ == nullptr || !cu_->identity_ready())
        break;
      if (cu_->product().find(PRODUCT_WALKY) == 0) {
        is_walky_ = true;
        ESP_LOGI(TAG, "Detected Walky device - using 1-byte position mode");
      }
      init_step_ = 1;
      break;

    case 1:
      send_info_request(FOR_CU, INF_TYPE);
      init_step_ = 2;
      break;

    case 2:
      send_info_request(FOR_CU, INF_POS_MAX);
      send_info_request(FOR_CU, INF_POS_MIN);
      init_step_ = 3;
      break;

    case 3:
      send_info_request(FOR_CU, INF_MAX_OPN);
      init_step_ = 4;
      break;

    case 4:
      send_info_request(FOR_CU, INF_STATUS);
      init_step_ = 5;
      break;

    case 5:
      ESP_LOGI(TAG, "Device initialization complete");
      init_ok_ = true;
      init_step_ = 6;
      publish_state_if_changed();
      break;

    default:
      break;
  }
}

void BusT4Cover::request_position() {
  if (parent_ == nullptr) return;
  send_info_request(FOR_CU, INF_CUR_POS);
}

void BusT4Cover::request_status() {
  if (parent_ == nullptr) return;
  send_info_request(FOR_CU, INF_STATUS);
}

void BusT4Cover::request_status_confirmation() {
  if (parent_ == nullptr) return;
  ESP_LOGD(TAG, "Requesting status for confirmation");
  awaiting_confirmation_ = true;
  send_info_request(FOR_CU, INF_STATUS);
}

void BusT4Cover::update_position(uint16_t encoder_pos) {
  pos_current_ = encoder_pos;
  last_encoder_update_ = millis();

  // Convert encoder position to percentage
  if (pos_max_ > pos_min_) {
    float pos = static_cast<float>(encoder_pos - pos_min_) / static_cast<float>(pos_max_ - pos_min_);
    pos = std::max(0.0f, std::min(1.0f, pos));  // Clamp to 0-1

    // Consider very small values as fully closed
    if (pos < CLOSED_POSITION_THRESHOLD) {
      pos = cover::COVER_CLOSED;
    }

    this->position = pos;
    ESP_LOGD(TAG, "Encoder position: %d -> %.1f%%", encoder_pos, pos * 100);
  }

  // Check if we've reached target position
  if (target_position_ >= 0.0f) {
    bool reached = false;
    if (current_operation == cover::COVER_OPERATION_OPENING && this->position >= target_position_) {
      reached = true;
    } else if (current_operation == cover::COVER_OPERATION_CLOSING && this->position <= target_position_) {
      reached = true;
    }

    if (reached) {
      ESP_LOGI(TAG, "Reached target position, stopping");
      send_cmd(CMD_STOP);
      target_position_ = -1.0f;
    }
  }

  publish_state_if_changed();
}

void BusT4Cover::publish_state_if_changed() {
  // Reset target position when idle
  if (current_operation == cover::COVER_OPERATION_IDLE) {
    target_position_ = -1.0f;
  }

  // Only publish if something changed
  if (last_published_op_ != current_operation || last_published_pos_ != this->position) {
    this->publish_state();
    last_published_op_ = current_operation;
    last_published_pos_ = this->position;
  }
}

uint32_t BusT4Cover::get_discovery_interval() const {
  // Exponential backoff: 1s, 2s, 4s, 8s, then cap at 10s
  // This avoids flooding the bus while still retrying periodically
  static const uint32_t intervals[] = {1000, 2000, 4000, 8000, 10000};
  uint8_t idx = std::min(discovery_attempts_, (uint8_t)4);
  return intervals[idx];
}

} // namespace esphome::bus_t4_control_unit
