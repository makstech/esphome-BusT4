#include "cover.h"
#include "esphome/core/log.h"

namespace esphome::bus_t4 {

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
}

void BusT4Cover::loop() {
  uint32_t now = millis();
  
  // Try to initialize device if not yet done
  if (!init_ok_ && (now - last_init_attempt_ > 10000)) {
    last_init_attempt_ = now;
    init_device();
  }
  
  // Request position periodically while moving
  if (init_ok_ && current_operation != cover::COVER_OPERATION_IDLE) {
    if (now - last_position_request_ > POSITION_UPDATE_INTERVAL) {
      last_position_request_ = now;
      request_position();
    }
  }
}

void BusT4Cover::dump_config() {
  LOG_COVER("", "Bus T4 Cover", this);
  ESP_LOGCONFIG(TAG, "  Initialized: %s", init_ok_ ? "Yes" : "No");
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
  // During initialization, accept packets from any source (for device discovery)
  // After init, only process packets from our target controller
  if (init_ok_ && packet.header.from != target_address_) {
    return;
  }
  
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
      case 0x83:  // Some controllers use this for opening
        ESP_LOGI(TAG, "Gate opening");
        current_operation = cover::COVER_OPERATION_OPENING;
        break;
        
      case STA_CLOSING:
      case 0x84:  // Some controllers use this for closing
        ESP_LOGI(TAG, "Gate closing");
        current_operation = cover::COVER_OPERATION_CLOSING;
        break;
        
      case STA_OPENED:
        ESP_LOGI(TAG, "Gate fully open");
        current_operation = cover::COVER_OPERATION_IDLE;
        this->position = cover::COVER_OPEN;
        target_position_ = -1.0f;
        break;
        
      case STA_CLOSED:
        ESP_LOGI(TAG, "Gate fully closed");
        current_operation = cover::COVER_OPERATION_IDLE;
        this->position = cover::COVER_CLOSED;
        target_position_ = -1.0f;
        break;
        
      case STA_STOPPED:
      case OP_STOPPED:
        ESP_LOGI(TAG, "Gate stopped");
        current_operation = cover::COVER_OPERATION_IDLE;
        target_position_ = -1.0f;
        request_position();  // Get final position
        break;
        
      case STA_ENDTIME:
        ESP_LOGI(TAG, "Gate operation ended (timeout)");
        current_operation = cover::COVER_OPERATION_IDLE;
        target_position_ = -1.0f;
        request_position();
        break;
        
      case STA_PART_OPENED:
        ESP_LOGI(TAG, "Gate partially open");
        current_operation = cover::COVER_OPERATION_IDLE;
        target_position_ = -1.0f;
        request_position();
        break;
    }
    
    publish_state_if_changed();
  }
  
  // Check for STA (status during movement) packets
  if (command == STA) {
    uint8_t status = packet.data[DEP_DATA_OFFSET];
    uint16_t pos = 0;
    if (packet.size >= 12) {
      pos = (packet.data[DEP_DATA_OFFSET + 1] << 8) | packet.data[DEP_DATA_OFFSET + 2];
    }
    
    ESP_LOGD(TAG, "STA packet: status=0x%02X, pos=%d", status, pos);
    
    switch (status) {
      case STA_OPENING:
        current_operation = cover::COVER_OPERATION_OPENING;
        break;
      case STA_CLOSING:
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
    
    case INF_WHO: {
      // Device discovery response
      // First byte of payload is the device type that responded
      uint8_t responder_type = packet.data[DATA_OFFSET];
      ESP_LOGI(TAG, "INF_WHO response: device_type=0x%02X from 0x%02X.%02X", 
               responder_type, packet.header.from.address, packet.header.from.endpoint);
      
      // Accept if it's a controller (0x04) or if from address has endpoint 0x03 (common for CU)
      if (responder_type == CONTROLLER || packet.header.from.endpoint == 0x03) {
        ESP_LOGI(TAG, "Found motor controller at 0x%02X.%02X", 
                 packet.header.from.address, packet.header.from.endpoint);
        target_address_ = packet.header.from;
        init_ok_ = true;
        
        // Now request additional info
        send_info_request(FOR_CU, INF_TYPE);
        send_info_request(FOR_CU, INF_POS_MAX);
        send_info_request(FOR_CU, INF_POS_MIN);
        send_info_request(FOR_CU, INF_STATUS);
      }
      break;
    }
    
    case INF_STATUS: {
      // Gate status response
      uint8_t gate_status = packet.data[DATA_OFFSET];
      ESP_LOGI(TAG, "Gate status: 0x%02X", gate_status);
      
      switch (gate_status) {
        case STA_OPENED:
          current_operation = cover::COVER_OPERATION_IDLE;
          this->position = cover::COVER_OPEN;
          break;
        case STA_CLOSED:
          current_operation = cover::COVER_OPERATION_IDLE;
          this->position = cover::COVER_CLOSED;
          break;
        case STA_OPENING:
          current_operation = cover::COVER_OPERATION_OPENING;
          break;
        case STA_CLOSING:
          current_operation = cover::COVER_OPERATION_CLOSING;
          break;
        case STA_STOPPED:
        case STA_UNKNOWN:
          current_operation = cover::COVER_OPERATION_IDLE;
          request_position();
          break;
      }
      publish_state_if_changed();
      break;
    }
    
    case INF_CUR_POS: {
      // Current position response (2 bytes, big endian)
      uint16_t pos = (packet.data[DATA_OFFSET] << 8) | packet.data[DATA_OFFSET + 1];
      ESP_LOGD(TAG, "Current position: %d", pos);
      update_position(pos);
      break;
    }
    
    case INF_POS_MAX: {
      // Open position (2 bytes, big endian)
      uint16_t pos = (packet.data[DATA_OFFSET] << 8) | packet.data[DATA_OFFSET + 1];
      if (pos > 0) {
        pos_max_ = pos;
        ESP_LOGI(TAG, "Open position: %d", pos_max_);
      }
      break;
    }
    
    case INF_POS_MIN: {
      // Close position (2 bytes, big endian)
      uint16_t pos = (packet.data[DATA_OFFSET] << 8) | packet.data[DATA_OFFSET + 1];
      pos_min_ = pos;
      ESP_LOGI(TAG, "Close position: %d", pos_min_);
      break;
    }
    
    case INF_MAX_OPN: {
      // Maximum encoder position (2 bytes, big endian)
      uint16_t pos = (packet.data[DATA_OFFSET] << 8) | packet.data[DATA_OFFSET + 1];
      ESP_LOGI(TAG, "Max encoder position: %d", pos);
      if (pos > 0) {
        pos_max_ = pos;
      }
      break;
    }
    
    case INF_IO: {
      // Input/Output state - includes limit switches
      // I/O data may have additional offset
      uint8_t io_state = packet.data[DATA_OFFSET + 3];
      ESP_LOGD(TAG, "I/O state: 0x%02X", io_state);
      
      switch (io_state) {
        case 0x01:  // Close limit switch
          this->position = cover::COVER_CLOSED;
          break;
        case 0x02:  // Open limit switch
          this->position = cover::COVER_OPEN;
          break;
      }
      publish_state_if_changed();
      break;
    }
  }
}

void BusT4Cover::init_device() {
  ESP_LOGI(TAG, "Initializing device...");
  
  // Broadcast: Who is on the bus?
  T4Source broadcast{0xFF, 0xFF};
  uint8_t who_msg[5] = { FOR_ALL, INF_WHO, REQ_GET, 0x00, 0x00 };
  T4Packet who_packet(broadcast, parent_->get_address(), DMP, who_msg, sizeof(who_msg));
  write(&who_packet, 0);
  
  // If we already have a target, request info
  if (init_ok_) {
    // Request motor type
    send_info_request(FOR_CU, INF_TYPE);
    
    // Request position limits
    send_info_request(FOR_CU, INF_POS_MAX);
    send_info_request(FOR_CU, INF_POS_MIN);
    send_info_request(FOR_CU, INF_MAX_OPN);
    
    // Request current status and position
    request_status();
    request_position();
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

void BusT4Cover::update_position(uint16_t encoder_pos) {
  pos_current_ = encoder_pos;
  
  // Convert encoder position to percentage
  if (pos_max_ > pos_min_) {
    float pos = static_cast<float>(encoder_pos - pos_min_) / static_cast<float>(pos_max_ - pos_min_);
    pos = std::max(0.0f, std::min(1.0f, pos));  // Clamp to 0-1
    
    // Consider very small values as fully closed
    if (pos < CLOSED_POSITION_THRESHOLD) {
      pos = cover::COVER_CLOSED;
    }
    
    this->position = pos;
    ESP_LOGD(TAG, "Position: %d -> %.1f%%", encoder_pos, pos * 100);
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

} // namespace esphome::bus_t4
