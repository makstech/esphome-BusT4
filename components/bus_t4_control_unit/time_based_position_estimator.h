#pragma once

#include "esphome/core/preferences.h"

namespace esphome::bus_t4_control_unit {

enum class Direction { OPEN, CLOSE };

// Estimates cover position from elapsed time for gates without encoder feedback,
// and auto-learns the open/close durations from full maneuvers (persisted to flash).
class TimeBasedPositionEstimator {
 public:
  void load();  // restore learned durations from flash

  void set_open_duration(uint32_t ms) { open_duration_ = ms; }
  void set_close_duration(uint32_t ms) { close_duration_ = ms; }
  void set_auto_learn(bool enable) { auto_learn_ = enable; }

  uint32_t open_duration() const { return open_duration_; }
  uint32_t close_duration() const { return close_duration_; }
  bool auto_learn() const { return auto_learn_; }

  // Start tracking a maneuver from from_pos (0..1); begins learning on a full
  // open-from-closed or close-from-open.
  void begin(Direction dir, float from_pos);

  // Estimated position now for the in-progress maneuver.
  float estimate(uint32_t now, Direction dir) const;

  // Maneuver reached its endpoint: finish learning for that direction, then reset.
  void finish(Direction dir);

  // Maneuver interrupted (stop/timeout/partial): reset without learning.
  void cancel();

  bool active() const { return start_time_ != 0; }

 protected:
  void save_();

  uint32_t open_duration_{20000};   // ms to fully open, default 20s
  uint32_t close_duration_{20000};  // ms to fully close, default 20s
  bool auto_learn_{true};

  uint32_t start_time_{0};  // millis() when the current maneuver started
  float start_pos_{0.0f};   // position when it started

  bool learning_open_{false};
  bool learning_close_{false};
  uint32_t learning_start_{0};

  ESPPreferenceObject pref_;
};

}  // namespace esphome::bus_t4_control_unit
