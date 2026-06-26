#include "time_based_position_estimator.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include <cmath>

namespace esphome::bus_t4_control_unit {

static const char *const TAG = "bus_t4.cover.timing";

// Ignore learned durations outside this range (sensor glitch / partial run).
static constexpr uint32_t MIN_LEARNED_DURATION = 3000;     // 3s
static constexpr uint32_t MAX_LEARNED_DURATION = 300000;   // 5min
// Only adopt a learned value if it deviates this much from the current one.
static constexpr float LEARNING_DEVIATION_THRESHOLD = 0.10f;
// Position at/below (and 1 - this above) which a maneuver counts as full-travel.
static constexpr float CLOSED_POSITION_THRESHOLD = 0.007f;

// Persisted across reboots; key must stay stable to keep existing learned values.
struct LearnedDurations {
  uint32_t open_duration;
  uint32_t close_duration;
  bool valid;
};

void TimeBasedPositionEstimator::begin(Direction dir, float from_pos) {
  start_time_ = millis();
  start_pos_ = from_pos;
  if (!auto_learn_)
    return;
  if (dir == Direction::OPEN && from_pos <= CLOSED_POSITION_THRESHOLD) {
    ESP_LOGI(TAG, "Learning open duration (opening from closed)");
    learning_open_ = true;
    learning_close_ = false;
    learning_start_ = millis();
  } else if (dir == Direction::CLOSE && from_pos >= (1.0f - CLOSED_POSITION_THRESHOLD)) {
    ESP_LOGI(TAG, "Learning close duration (closing from open)");
    learning_close_ = true;
    learning_open_ = false;
    learning_start_ = millis();
  }
}

float TimeBasedPositionEstimator::estimate(uint32_t now, Direction dir) const {
  uint32_t elapsed = now - start_time_;
  float pos = start_pos_;
  if (dir == Direction::OPEN) {
    pos = start_pos_ + (float) elapsed / (float) open_duration_;
    if (pos > 1.0f)
      pos = 1.0f;
  } else {
    pos = start_pos_ - (float) elapsed / (float) close_duration_;
    if (pos < 0.0f)
      pos = 0.0f;
  }
  return pos;
}

void TimeBasedPositionEstimator::finish(Direction dir) {
  bool opening = (dir == Direction::OPEN);
  bool learning = opening ? learning_open_ : learning_close_;
  if (learning && learning_start_ != 0) {
    uint32_t learned = millis() - learning_start_;
    uint32_t &target = opening ? open_duration_ : close_duration_;
    const char *dir = opening ? "open" : "close";
    if (learned < MIN_LEARNED_DURATION) {
      ESP_LOGW(TAG, "Learned %s duration too short (%.1fs), ignoring", dir, learned / 1000.0f);
    } else if (learned > MAX_LEARNED_DURATION) {
      ESP_LOGW(TAG, "Learned %s duration too long (%.1fs), ignoring", dir, learned / 1000.0f);
    } else {
      float deviation = std::abs((float) learned - (float) target) / (float) target;
      if (deviation > LEARNING_DEVIATION_THRESHOLD || target == 20000) {
        ESP_LOGI(TAG, "Learned %s duration: %.1fs (was %.1fs)", dir, learned / 1000.0f, target / 1000.0f);
        target = learned;
        save_();
      }
    }
  }
  cancel();
}

void TimeBasedPositionEstimator::cancel() {
  learning_open_ = false;
  learning_close_ = false;
  learning_start_ = 0;
  start_time_ = 0;
}

void TimeBasedPositionEstimator::save_() {
  LearnedDurations data{open_duration_, close_duration_, true};
  if (pref_.save(&data)) {
    ESP_LOGI(TAG, "Saved learned durations (open %.1fs, close %.1fs)", open_duration_ / 1000.0f,
             close_duration_ / 1000.0f);
  } else {
    ESP_LOGW(TAG, "Failed to save learned durations");
  }
}

void TimeBasedPositionEstimator::load() {
  pref_ = global_preferences->make_preference<LearnedDurations>(fnv1_hash("bus_t4_cover_timing"));
  LearnedDurations data;
  if (pref_.load(&data) && data.valid) {
    if (data.open_duration >= MIN_LEARNED_DURATION && data.open_duration <= MAX_LEARNED_DURATION) {
      open_duration_ = data.open_duration;
      ESP_LOGI(TAG, "Loaded open duration: %.1fs", open_duration_ / 1000.0f);
    }
    if (data.close_duration >= MIN_LEARNED_DURATION && data.close_duration <= MAX_LEARNED_DURATION) {
      close_duration_ = data.close_duration;
      ESP_LOGI(TAG, "Loaded close duration: %.1fs", close_duration_ / 1000.0f);
    }
  }
}

}  // namespace esphome::bus_t4_control_unit
