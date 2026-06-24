// Host-compilable unit tests for the Bus-T4 packet parser (t4_packet.h is standalone:
// it only needs <cstdint>/<algorithm>, so no ESPHome toolchain is required).
//
//   g++ -std=c++17 -Wall -Wextra tests/test_t4_packet.cpp -o /tmp/t4test && /tmp/t4test
//
// The byte arrays double as documentation of the verified wire layout (.agent/PROTOCOL.md).

#include "../components/bus_t4/t4_packet.h"
#include <cstdio>
#include <cstring>
#include <vector>

static int g_failures = 0;

#define CHECK(cond) \
  do { \
    if (!(cond)) { \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures; \
    } \
  } while (0)

// Build a T4Packet from raw body bytes (data[] starts at header.to, per PROTOCOL.md §1).
static T4Packet make_packet(const std::vector<uint8_t> &bytes) {
  T4Packet p;
  p.size = static_cast<uint8_t>(bytes.size());
  std::memcpy(p.data, bytes.data(), bytes.size());
  return p;
}

int main() {
  // --- A wired-keypad / remote / app "Open" command, reported by the controller ---
  // DEP, from controller 0x00.03 to us 0x50.90, device=FOR_CU, cmd=RUN-0x80, echo=OPEN+0x80.
  {
    T4Packet p = make_packet({0x50, 0x90, 0x00, 0x03, DEP, 0x05, 0x00,
                              FOR_CU, RUN & 0x7F, 0x83 /*OPEN|0x80*/, 0x64, 0x00});
    CHECK(p.is_dep());
    CHECK(!p.is_dmp());
    CHECK(p.is_run_packet());
    CHECK(p.run_command_echo() == CMD_OPEN);
    CHECK(p.device() == FOR_CU);
  }

  // Close / Step / Partial-1 echoes map to the right command values.
  {
    T4Packet p = make_packet({0x50, 0x90, 0x00, 0x03, DEP, 0x05, 0x00, FOR_CU, RUN & 0x7F, 0x84, 0x64, 0x00});
    CHECK(p.run_command_echo() == CMD_CLOSE);
  }
  {
    T4Packet p = make_packet({0x50, 0x90, 0x00, 0x03, DEP, 0x05, 0x00, FOR_CU, RUN & 0x7F, 0x81, 0x64, 0x00});
    CHECK(p.run_command_echo() == CMD_STEP);
  }
  {
    T4Packet p = make_packet({0x50, 0x90, 0x00, 0x03, DEP, 0x05, 0x00, FOR_CU, RUN & 0x7F, 0x85, 0x64, 0x00});
    CHECK(p.run_command_echo() == CMD_OPEN_PARTIAL_1);
  }

  // --- A RUN *status* report (data[9] < 0x80) is NOT a command echo ---
  {
    T4Packet p = make_packet({0x50, 0x90, 0x00, 0x03, DEP, 0x05, 0x00,
                              FOR_CU, RUN & 0x7F, STA_OPENING /*0x02*/, 0x64, 0x00});
    CHECK(p.is_run_packet());
    CHECK(p.run_command_echo() == 0);  // status, no command issued
  }

  // --- A DMP info response (gate status) is not a command and not an EVT push ---
  {
    T4Packet p = make_packet({0x50, 0x90, 0x00, 0x03, DMP, 0x07, 0x00,
                              FOR_CU, INF_STATUS, RSP_GET_COMPLETE, 0x00, ERR_NONE, STA_OPENED, 0x00});
    CHECK(p.is_dmp());
    CHECK(!p.is_dep());
    CHECK(!p.is_run_packet());
    CHECK(p.command() == INF_STATUS);
    CHECK(p.dmp_flags() == RSP_GET_COMPLETE);
    CHECK(!p.is_event_push());
    CHECK(p.dmp_payload()[0] == STA_OPENED);
  }

  // --- A spontaneous EVT-flagged DMP packet is detected ---
  {
    T4Packet p = make_packet({0x50, 0x90, 0x00, 0x03, DMP, 0x07, 0x00,
                              FOR_CU, INF_STATUS, T4_FLAG_EVT | T4_FLAG_FIN, 0x00, ERR_NONE, 0x00, 0x00});
    CHECK(p.is_event_push());
  }

  // --- Flags byte composition matches the documented bit decode ---
  CHECK(REQ_GET == (T4_FLAG_REQ | T4_FLAG_GET | T4_FLAG_ACK | T4_FLAG_FIN));
  CHECK(REQ_SET == (T4_FLAG_REQ | T4_FLAG_SET | T4_FLAG_ACK | T4_FLAG_FIN));
  CHECK(RSP_GET_COMPLETE == (T4_FLAG_GET | T4_FLAG_ACK | T4_FLAG_FIN));

  // --- Too-short frames never read past the header (guards return false) ---
  {
    T4Packet p = make_packet({0x50, 0x90, 0x00, 0x03, DEP, 0x00, 0x00});  // size 7
    CHECK(!p.is_dep());
    CHECK(!p.is_dmp());
    CHECK(!p.is_run_packet());
  }

  if (g_failures == 0) {
    std::printf("OK  all t4_packet tests passed\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", g_failures);
  return 1;
}
