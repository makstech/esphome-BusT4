#pragma once

#include "esphome/components/cover/cover.h"
#include "esphome/components/bus_t4/bus_t4.h"
#include "time_based_position_estimator.h"
#include <string>

namespace esphome::bus_t4_control_unit {

// bus_t4 vocabulary this entity uses:
using bus_t4::BusT4Device;
using bus_t4::T4Packet;
using bus_t4::T4MotorType;
using enum bus_t4::T4MotorType;
using enum bus_t4::T4Protocol;
using enum bus_t4::T4Target;
using enum bus_t4::T4RequestType;
using enum bus_t4::T4ResponseType;
using enum bus_t4::T4Error;
using enum bus_t4::T4Command;
using enum bus_t4::T4CommandPacket;
using enum bus_t4::T4InfoCommand;
using enum bus_t4::T4GateStatus;
using enum bus_t4::T4OperationStatus;

class BusT4ControlUnit;

// Position update interval during movement (ms)
static constexpr uint32_t POSITION_UPDATE_INTERVAL = 500;

// Position threshold below which cover is considered fully closed
static constexpr float CLOSED_POSITION_THRESHOLD = 0.007f;

// Products starting with WLA = Walky (uses 1-byte position values)
static const std::string PRODUCT_WALKY = "WLA";

class BusT4Cover : public cover::Cover, public BusT4Device, public Component {
 public:
  BusT4Cover() = default;

  cover::CoverTraits get_traits() override;

  void setup() override;
  void loop() override;
  void dump_config() override;

  // Called when a packet is received from the bus
  void on_packet(const T4Packet &packet) override;

  // Configuration setters (from YAML)
  void set_open_duration(uint32_t duration) { timing_.set_open_duration(duration); }
  void set_close_duration(uint32_t duration) { timing_.set_close_duration(duration); }
  void set_auto_learn_timing(bool enable) { timing_.set_auto_learn(enable); }
  void set_position_report_interval(uint32_t interval) { position_report_interval_ = interval; }
  void set_force_estimated_position(bool force) { force_estimated_position_ = force; }
  void set_control_unit(BusT4ControlUnit *cu) { cu_ = cu; }

 protected:
  void control(const cover::CoverCall &call) override;

 private:
  // Parse different packet types
  void parse_dep_packet(const T4Packet &packet);
  void parse_dmp_packet(const T4Packet &packet);

  // Request current position from controller
  void request_position();

  // Request status from controller
  void request_status();

  // Update position from encoder value
  void update_position(uint16_t encoder_pos);

  // Publish state only if changed
  void publish_state_if_changed();

  // Device initialization
  void init_device();
  bool init_ok_{false};
  uint8_t init_step_{0};  // Initialization state machine step
  uint8_t discovery_attempts_{0};  // Discovery retry counter for exponential backoff
  uint32_t get_discovery_interval() const;  // Get current discovery retry interval

  BusT4ControlUnit *cu_{nullptr};

  // Walky gates report 1-byte position values (derived from the product string).
  bool is_walky_{false};

  // Position tracking
  uint16_t pos_max_{2048};    // Encoder position for fully open
  uint16_t pos_min_{0};       // Encoder position for fully closed
  uint16_t pos_current_{0};   // Current encoder position

  // Motor type
  T4MotorType motor_type_{MOTOR_SLIDING};

  // Target position for time-based positioning
  float target_position_{-1.0f};

  // Time-based position estimation + auto-learning (gates without encoder feedback)
  TimeBasedPositionEstimator timing_;
  bool force_estimated_position_{false};  // Ignore the encoder and always use the estimate (testing)

  // Timing
  uint32_t last_position_update_{0};
  uint32_t last_position_publish_{0};  // Rate-limit position publishing
  uint32_t position_report_interval_{1000};  // Position report rate (ms)
  uint32_t last_init_attempt_{0};
  uint32_t last_status_refresh_{0};  // Periodic status refresh

  // Position source tracking
  bool has_encoder_{false};          // True if device reports encoder positions
  uint32_t last_encoder_update_{0};  // Last time we got encoder data

  // State tracking for change detection
  cover::CoverOperation last_published_op_{cover::COVER_OPERATION_IDLE};
  float last_published_pos_{-1.0f};

  // Request status confirmation after unexpected stop
  void request_status_confirmation();
  bool awaiting_confirmation_{false};
  cover::CoverOperation last_operation_{cover::COVER_OPERATION_IDLE};
};

} // namespace esphome::bus_t4_control_unit
