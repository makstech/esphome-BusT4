#pragma once

#include "esphome/components/cover/cover.h"
#include "esphome/core/preferences.h"
#include "../bus_t4.h"
#include <string>

namespace esphome::bus_t4 {

// Position update interval during movement (ms)
static constexpr uint32_t POSITION_UPDATE_INTERVAL = 500;

// Position threshold below which cover is considered fully closed
static constexpr float CLOSED_POSITION_THRESHOLD = 0.007f;

// Minimum valid learned duration (ms) - ignore if shorter
static constexpr uint32_t MIN_LEARNED_DURATION = 3000;  // 3 seconds

// Maximum valid learned duration (ms) - ignore if longer
static constexpr uint32_t MAX_LEARNED_DURATION = 300000;  // 5 minutes

// Deviation threshold for updating learned values (10%)
static constexpr float LEARNING_DEVIATION_THRESHOLD = 0.10f;

// Structure for persisting learned durations
struct LearnedDurations {
  uint32_t open_duration;
  uint32_t close_duration;
  bool valid;  // Set to true after first successful learning
};

// Known product identifiers for device-specific handling
// Products starting with WLA = Walky (uses 1-byte position)
static const std::string PRODUCT_WALKY = "WLA";
// Products starting with ROB = Robus family (no position query during movement)
static const std::string PRODUCT_ROBUS = "ROB";

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
  void set_open_duration(uint32_t duration) { open_duration_ = duration; }
  void set_close_duration(uint32_t duration) { close_duration_ = duration; }
  void set_auto_learn_timing(bool enable) { auto_learn_timing_ = enable; }
  void set_position_report_interval(uint32_t interval) { position_report_interval_ = interval; }

  // Motor controller configuration (runtime)
  // These send commands to the motor controller to change its settings
  void set_auto_close(bool enable);      // L1 - Auto-close after opening
  void set_photo_close(bool enable);     // L2 - Close after photo sensor clears
  void set_always_close(bool enable);    // L3 - Always close (ignore hold-open)
  void set_standby(bool enable);         // Standby mode (power saving)
  void set_peak_mode(bool enable);       // Peak mode (faster operation)
  void set_pre_flash(bool enable);       // Pre-flash warning light

 protected:
  void control(const cover::CoverCall &call) override;

 private:
  // Parse different packet types
  void parse_dep_packet(const T4Packet &packet);
  void parse_dmp_packet(const T4Packet &packet);
  void parse_oxi_packet(const T4Packet &packet);  // OXI receiver (remote control)

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

  // Device identification - for device-specific handling
  std::string product_name_;           // Product name (e.g., "WLA1", "ROBUSHSR10")
  std::string manufacturer_;           // Manufacturer name
  std::string firmware_version_;       // Firmware version string
  bool is_walky_{false};               // Walky gates: 1-byte position values
  bool is_robus_{false};               // Robus gates: no position query during movement

  // Position tracking
  uint16_t pos_max_{2048};    // Encoder position for fully open
  uint16_t pos_min_{0};       // Encoder position for fully closed
  uint16_t pos_current_{0};   // Current encoder position

  // Motor type
  T4MotorType motor_type_{MOTOR_SLIDING};

  // Target position for time-based positioning
  float target_position_{-1.0f};

  // Time-based position tracking (for gates without encoder feedback)
  // Configure open_duration and close_duration in seconds
  uint32_t open_duration_{20000};   // Time to fully open (ms) - default 20s
  uint32_t close_duration_{20000};  // Time to fully close (ms) - default 20s
  uint32_t movement_start_time_{0}; // When current movement started
  float position_at_start_{0.0f};   // Position when movement started

  // Auto-learning for open/close durations
  bool auto_learn_timing_{true};         // Whether to auto-learn timing
  bool learning_open_{false};            // Currently learning open duration
  bool learning_close_{false};           // Currently learning close duration
  uint32_t learning_start_time_{0};      // When learning movement started
  ESPPreferenceObject pref_;             // Preference storage for learned values

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

  // Helper methods for learning
  void start_learning_open();
  void start_learning_close();
  void finish_learning_open();
  void finish_learning_close();
  void cancel_learning();
  void save_learned_durations();
  void load_learned_durations();

  // Request status confirmation after unexpected stop
  void request_status_confirmation();
  bool awaiting_confirmation_{false};
  cover::CoverOperation last_operation_{cover::COVER_OPERATION_IDLE};
};

} // namespace esphome::bus_t4
