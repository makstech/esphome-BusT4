// Host unit test for TimeBasedPositionEstimator. See the CI unit-test job (or the
// README) for the g++ build command.
#include "time_based_position_estimator.h"

#include <cmath>
#include <cstdio>
#include <string>

// Test doubles for the esphome symbols the estimator links against.
namespace esphome {
uint32_t g_now = 0;
uint32_t millis() { return g_now; }
uint32_t fnv1_hash(const std::string &) { return 0xB0571234; }  // fixed key for the test
static ESPPreferences prefs;
ESPPreferences *global_preferences = &prefs;
}  // namespace esphome

using esphome::g_now;
using esphome::bus_t4_control_unit::Direction;
using esphome::bus_t4_control_unit::TimeBasedPositionEstimator;

static int failures = 0;

#define CHECK(cond) \
  do { \
    if (!(cond)) { \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
      failures++; \
    } \
  } while (0)

#define CHECK_NEAR(a, b) \
  do { \
    if (std::fabs((double) (a) - (double) (b)) > 1e-4) { \
      std::printf("FAIL %s:%d  %f != %f\n", __FILE__, __LINE__, (double) (a), (double) (b)); \
      failures++; \
    } \
  } while (0)

int main() {
  // estimate(): opening ramps linearly from start and clamps at 1.0.
  {
    TimeBasedPositionEstimator t;  // defaults: 20s open/close
    g_now = 1000;
    t.begin(Direction::OPEN, 0.0f);
    CHECK(t.active());
    CHECK_NEAR(t.estimate(11000, Direction::OPEN), 0.5f);
    CHECK_NEAR(t.estimate(21000, Direction::OPEN), 1.0f);
    CHECK_NEAR(t.estimate(99000, Direction::OPEN), 1.0f);  // clamp
  }

  // estimate(): closing ramps down and clamps at 0.0.
  {
    TimeBasedPositionEstimator t;
    g_now = 0;
    t.begin(Direction::CLOSE, 1.0f);
    CHECK_NEAR(t.estimate(10000, Direction::CLOSE), 0.5f);
    CHECK_NEAR(t.estimate(25000, Direction::CLOSE), 0.0f);  // clamp
  }

  // Full open-from-closed learns the open duration; finish() clears active().
  {
    TimeBasedPositionEstimator t;
    g_now = 1000;
    t.begin(Direction::OPEN, 0.0f);
    g_now = 26000;  // 25s travel
    t.finish(Direction::OPEN);
    CHECK(!t.active());
    CHECK(t.open_duration() == 25000);
    CHECK(t.close_duration() == 20000);  // untouched
  }

  // A maneuver that doesn't start at an endpoint is not learned.
  {
    TimeBasedPositionEstimator t;
    g_now = 0;
    t.begin(Direction::OPEN, 0.5f);  // mid-travel
    g_now = 25000;
    t.finish(Direction::OPEN);
    CHECK(t.open_duration() == 20000);
  }

  // Durations below the minimum (3s) are rejected.
  {
    TimeBasedPositionEstimator t;
    g_now = 0;
    t.begin(Direction::OPEN, 0.0f);
    g_now = 1000;  // 1s < min
    t.finish(Direction::OPEN);
    CHECK(t.open_duration() == 20000);
  }

  // auto_learn off => never learns.
  {
    TimeBasedPositionEstimator t;
    t.set_auto_learn(false);
    g_now = 0;
    t.begin(Direction::CLOSE, 1.0f);
    g_now = 30000;
    t.finish(Direction::CLOSE);
    CHECK(t.close_duration() == 20000);
  }

  // cancel() resets without learning.
  {
    TimeBasedPositionEstimator t;
    g_now = 5000;
    t.begin(Direction::OPEN, 0.0f);
    CHECK(t.active());
    g_now = 30000;
    t.cancel();
    CHECK(!t.active());
    CHECK(t.open_duration() == 20000);
  }

  // Learned durations persist into a fresh instance via load().
  {
    esphome::test_pref_store().clear();
    TimeBasedPositionEstimator a;
    a.load();
    g_now = 1000;
    a.begin(Direction::OPEN, 0.0f);
    g_now = 31000;  // 30s
    a.finish(Direction::OPEN);
    CHECK(a.open_duration() == 30000);

    TimeBasedPositionEstimator b;
    b.load();
    CHECK(b.open_duration() == 30000);  // restored from "flash"
  }

  if (failures == 0) {
    std::printf("All estimator tests passed.\n");
    return 0;
  }
  std::printf("%d estimator test(s) FAILED.\n", failures);
  return 1;
}
