#pragma once
// Host-test stub for esphome/core/preferences.h. Backs preferences with an
// in-process map keyed by hash, so a save() is visible to a later load() —
// enough to exercise the estimator's persist/restore path.
#include <cstdint>
#include <cstring>
#include <map>
#include <vector>

namespace esphome {

inline std::map<uint32_t, std::vector<uint8_t>> &test_pref_store() {
  static std::map<uint32_t, std::vector<uint8_t>> store;
  return store;
}

class ESPPreferenceObject {
 public:
  ESPPreferenceObject() = default;
  explicit ESPPreferenceObject(uint32_t key) : key_(key), valid_(true) {}

  template<typename T> bool save(const T *src) {
    if (!valid_)
      return false;
    auto &buf = test_pref_store()[key_];
    buf.resize(sizeof(T));
    std::memcpy(buf.data(), src, sizeof(T));
    return true;
  }

  template<typename T> bool load(T *dest) {
    if (!valid_)
      return false;
    auto it = test_pref_store().find(key_);
    if (it == test_pref_store().end() || it->second.size() != sizeof(T))
      return false;
    std::memcpy(dest, it->second.data(), sizeof(T));
    return true;
  }

 private:
  uint32_t key_{0};
  bool valid_{false};
};

class ESPPreferences {
 public:
  template<typename T> ESPPreferenceObject make_preference(uint32_t key) { return ESPPreferenceObject(key); }
};

extern ESPPreferences *global_preferences;

}  // namespace esphome
