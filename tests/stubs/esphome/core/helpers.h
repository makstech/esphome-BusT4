#pragma once
// Host-test stub for esphome/core/helpers.h — just the hash the estimator keys on.
#include <cstdint>
#include <string>
namespace esphome {
uint32_t fnv1_hash(const std::string &str);
}
