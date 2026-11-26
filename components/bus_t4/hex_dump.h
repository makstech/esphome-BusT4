#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace esphome::bus_t4 {

// Hex dump for log messages, e.g. "55.0C.00.FF.0A (5)" - the same format as the
// format_hex_pretty() this component used before. ESPHome 2026.11 no longer exposes
// that function through esphome/core/helpers.h (it moved to alloc_helpers.h in
// 2026.5), and its stack-based replacement format_hex_pretty_to() only exists since
// 2026.1. Keeping our own formatter builds on every ESPHome version and does not
// touch the heap. The result lives on the caller's stack, so
// hex_dump(...).c_str() is valid until the end of the full expression (the log call).
struct HexDump {
  // Longest frame on the bus is T4Frame (66 bytes); longer input, e.g. a raw
  // command typed by the user, is cut off and marked with "..."
  static constexpr size_t MAX_BYTES = 72;
  char buf[MAX_BYTES * 3 + 16];  // "XX." per byte, "... (NNNNNNNNNN)", NUL

  const char *c_str() const { return buf; }
};

inline HexDump hex_dump(const uint8_t *data, size_t length) {
  static const char HEX_DIGITS[] = "0123456789ABCDEF";
  HexDump out;
  char *p = out.buf;
  const size_t shown = length < HexDump::MAX_BYTES ? length : HexDump::MAX_BYTES;
  for (size_t i = 0; data != nullptr && i < shown; i++) {
    if (i > 0)
      *p++ = '.';
    *p++ = HEX_DIGITS[data[i] >> 4];
    *p++ = HEX_DIGITS[data[i] & 0x0F];
  }
  *p = '\0';
  if (data != nullptr && length > 4) {
    snprintf(p, sizeof(out.buf) - (p - out.buf), "%s (%u)", shown < length ? "..." : "", static_cast<unsigned>(length));
  }
  return out;
}

inline HexDump hex_dump(const std::vector<uint8_t> &data) { return hex_dump(data.data(), data.size()); }

} // namespace esphome::bus_t4
