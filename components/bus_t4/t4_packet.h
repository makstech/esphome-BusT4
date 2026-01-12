/*
   https://github.com/gashtaan/nice-bidiwifi-firmware

   Copyright (C) 2024, Michal Kovacik
   Copyright (C) 2025, Andrei Nistor

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 3, as
   published by the Free Software Foundation.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <algorithm>
#include <cstdint>

static constexpr uint8_t T4_BREAK = 0x00;
static constexpr uint8_t T4_SYNC = 0x55;

struct T4Source {
  uint8_t address;
  uint8_t endpoint;

  bool operator==(const T4Source& other) const { return address == other.address && endpoint == other.endpoint; }
  bool operator!=(const T4Source& other) const { return !(*this == other); }
};

// Message type (byte 6 of packet)
enum T4Protocol : uint8_t {
  DEP = 0x01,  // Direct Execute Protocol - command execution
  DMP = 0x08,  // Device Management Protocol - info requests/responses
};

// Device type identifiers
enum T4Device : uint8_t {
  STANDARD = 0x00,
  OVIEW = 0x01,
  CONTROLLER = 0x04,  // Motor control unit
  SCREEN = 0x06,
  RADIO = 0x0A,       // OXI receiver
};

// Command/info target (byte 9)
enum T4Target : uint8_t {
  FOR_ALL = 0x00,     // Broadcast to all devices
  FOR_CU = 0x04,      // For control unit (motor)
  FOR_OXI = 0x0A,     // For OXI receiver
};

// OXI receiver commands (byte 10 when target is FOR_OXI)
enum T4OxiCommand : uint8_t {
  OXI_REMOTE_LIST = 0x25,   // Remote control list/info
  OXI_BUTTON_READ = 0x26,   // Button press detected
};

// Info commands (byte 10 for DMP packets)
enum T4InfoCommand : uint8_t {
  INF_TYPE = 0x00,       // Motor type
  INF_STATUS = 0x01,     // Gate status (open/closed/stopped)
  INF_WHO = 0x04,        // Who is on the bus?
  INF_MAC = 0x07,        // MAC address
  INF_MAN = 0x08,        // Manufacturer
  INF_PRD = 0x09,        // Product name
  INF_HWR = 0x0A,        // Hardware version
  INF_FRM = 0x0B,        // Firmware version
  INF_DSC = 0x0C,        // Description
  INF_CUR_POS = 0x11,    // Current position
  INF_MAX_OPN = 0x12,    // Max encoder position
  INF_POS_MAX = 0x18,    // Open position
  INF_POS_MIN = 0x19,    // Close position
  INF_IO = 0xD1,         // Input/output state (limit switches)

  // Configuration parameters (settable)
  CFG_AUTOCLS = 0x80,    // Auto-close (L1) - 0x00=off, 0x01=on
  CFG_PH_CLS = 0x81,     // Close after photo (L2) - 0x00=off, 0x01=on
  CFG_ALW_CLS = 0x82,    // Always close (L3) - 0x00=off, 0x01=on
  CFG_STANDBY = 0x83,    // Standby mode - 0x00=off, 0x01=on
  CFG_PEAK = 0x84,       // Peak mode - 0x00=off, 0x01=on
  CFG_PRE_FLASH = 0x85,  // Pre-flash warning - 0x00=off, 0x01=on
  CFG_CLOSE_SPEED = 0x90, // Close speed (0-100)
  CFG_OPEN_SPEED = 0x91,  // Open speed (0-100)
};

// Request types (byte 11)
enum T4RequestType : uint8_t {
  REQ_SET = 0xA9,        // Set parameter
  REQ_GET = 0x99,        // Get parameter
  REQ_GET_SUPP = 0x89,   // Get supported commands
};

// Response flags (subtract 0x80 from request type)
enum T4ResponseType : uint8_t {
  RSP_GET_COMPLETE = 0x19,      // GET response complete (0x99 - 0x80)
  RSP_GET_INCOMPLETE = 0x18,    // GET response incomplete, more data available
  RSP_SET_COMPLETE = 0x29,      // SET response complete (0xA9 - 0x80)
};

// Error codes (byte 13)
enum T4Error : uint8_t {
  ERR_NONE = 0x00,       // No error
  ERR_UNSUPPORTED = 0xFD, // Command not supported
};

// Gate/motor types
enum T4MotorType : uint8_t {
  MOTOR_SLIDING = 0x01,
  MOTOR_SECTIONAL = 0x02,
  MOTOR_SWING = 0x03,
  MOTOR_BARRIER = 0x04,
  MOTOR_UPANDOVER = 0x05,
};

// Gate status values (from INF_STATUS response, byte 14)
enum T4GateStatus : uint8_t {
  STA_UNKNOWN = 0x00,
  STA_STOPPED = 0x01,
  STA_OPENING = 0x02,
  STA_CLOSING = 0x03,
  STA_OPENED = 0x04,
  STA_CLOSED = 0x05,
  STA_ENDTIME = 0x06,       // Maneuver ended by timeout
  STA_PART_OPENED = 0x10,   // Partially opened
  // Alternate status codes used by some controllers (e.g., Road 400)
  STA_OPENING_ALT = 0x83,   // Opening (alternate)
  STA_CLOSING_ALT = 0x84,   // Closing (alternate)
};

// Operation status in RSP packets (byte 11)
enum T4OperationStatus : uint8_t {
  OP_SBS = 0x01,         // Step by step
  OP_STOP = 0x02,        // Stop command
  OP_OPEN = 0x03,        // Open command
  OP_CLOSE = 0x04,       // Close command
  OP_PARTIAL_1 = 0x05,   // Partial open 1
  OP_PARTIAL_2 = 0x06,   // Partial open 2
  OP_PARTIAL_3 = 0x07,   // Partial open 3
  OP_STOPPED = 0x08,     // Movement stopped
};

struct T4Packet {
  uint8_t size = 0;
  union {
    uint8_t data[63];
    struct {
      struct {
        T4Source to;
        T4Source from;
        T4Protocol protocol;
        uint8_t messageSize;
        uint8_t checksum;
      } header;
      struct {
        uint8_t device;
        uint8_t command;
        union {
          struct {
            uint8_t flags;
            uint8_t sequence;
            uint8_t status;
            uint8_t data[0];
          } dmp;
          struct {
            uint8_t data[0];
          } dep;
        };
        uint8_t checksum;
      } message;
    };
  };

  uint8_t checksum(uint8_t i, uint8_t c) const {
    uint8_t h = 0;
    while (c-- > 0)
      h ^= data[i++];
    return h;
  }

  T4Packet() = default;
  T4Packet(const T4Source to, const T4Source from, const T4Protocol protocol, uint8_t *messageData,
           const uint8_t messageSize) {
    size = sizeof(header) + messageSize + 1;

    header.to = to;
    header.from = from;
    header.protocol = protocol;
    header.messageSize = messageSize + 1;
    header.checksum = checksum(0, sizeof(header) - 1);

    std::copy_n(messageData, messageSize, data + sizeof(header));

    data[size - 1] = checksum(sizeof(header), messageSize);
  }
};

// DEP command packet structure
enum T4CommandPacket : uint8_t {
  RUN = 0x82,  // Execute command
  STA = 0xC0,  // Status during movement
};

// Control commands (for RUN)
enum T4Command : uint8_t {
  CMD_STEP = 0x01,
  CMD_STOP = 0x02,
  CMD_OPEN = 0x03,
  CMD_CLOSE = 0x04,
  CMD_OPEN_PARTIAL_1 = 0x05,
  CMD_OPEN_PARTIAL_2 = 0x06,
  CMD_OPEN_PARTIAL_3 = 0x07,
};
