#pragma once

#include "esphome/components/cover/cover.h"
#include "../bus_t4.h"

namespace esphome::bus_t4 {

// Position update interval during movement (ms)
static constexpr uint32_t POSITION_UPDATE_INTERVAL = 500;

// Position threshold below which cover is considered fully closed
static constexpr float CLOSED_POSITION_THRESHOLD = 0.007f;

class BusT4Cover : public cover::Cover, public BusT4Device, public Component {
 public:
  BusT4Cover() = default;
  
  cover::CoverTraits get_traits() override;
  
  void setup() override;
  void loop() override;
  void dump_config() override;

  // Called when a packet is received from the bus
  void on_packet(const T4Packet &packet) override;

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
  
  // Position tracking
  uint16_t pos_max_{2048};    // Encoder position for fully open
  uint16_t pos_min_{0};       // Encoder position for fully closed
  uint16_t pos_current_{0};   // Current encoder position
  
  // Motor type
  T4MotorType motor_type_{MOTOR_SLIDING};
  
  // Target position for positioning (if supported)
  float target_position_{-1.0f};
  
  // Timing
  uint32_t last_position_request_{0};
  uint32_t last_init_attempt_{0};
  
  // State tracking for change detection
  cover::CoverOperation last_published_op_{cover::COVER_OPERATION_IDLE};
  float last_published_pos_{-1.0f};
};

} // namespace esphome::bus_t4
