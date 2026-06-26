# Nice Bus-T4 ESPHome Component

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![ESPHome](https://img.shields.io/badge/ESPHome-2024.6+-green.svg)](https://esphome.io/)
[![ESP32](https://img.shields.io/badge/ESP32-supported-brightgreen.svg)](https://www.espressif.com/en/products/socs/esp32)

ESPHome component for integrating **Nice gate and garage door automation** into Home Assistant via the Bus-T4 protocol.

> **Perfect for Nice BiDi-WiFi module owners** who want local control without cloud dependency!

## Features

- 🏠 **Home Assistant** - Native ESPHome integration
- 🔒 **Local control** - No cloud, no internet required
- 🚪 **Full gate control** - Open, close, stop, and partial opening commands
- 📊 **Real-time status** - Opening, closing, stopped, fully open/closed states
- 📏 **Position tracking** - Time-based estimation with encoder priority when available
- 🧠 **Auto-learning** - Automatically learns and re-learns your gate's open/close timing
- ⚙️ **Motor configuration** - Control auto-close, standby, surge mode, and more via SET commands
- 🔧 **Wide compatibility** - Device-specific handling for Walky, Robus, Road 400, and more
- 📡 **OXI receiver logging** - Remote control button presses are logged for debugging

## Supported Hardware

### Nice BiDi-WiFi Module (Recommended)

The [Nice BiDi-WiFi](https://www.niceforyou.com/uk/nicepost/bidi-wifi-new-pocket-programming-interface) module contains an ESP32-WROOM and connects directly to your Nice gate controller. By flashing ESPHome firmware, you get local Home Assistant control.

### Compatible Nice Controllers

Tested with:
- Nice Robus (RBS400, RBS600, RBS800, etc.)

Should work with:
- Nice Walky (WLA1)
- Nice Road 400
- Nice Spin (SPn21)

Should work with any Nice controller that has a Bus-T4 port.

**Device-specific features:**
- **Walky gates**: Uses 1-byte position values (auto-detected)
- **Road 400**: Alternate status codes supported (0x83/0x84)

## Quick Start

### Flash Nice BiDi-WiFi Module

#### Step 1: Backup Original Firmware

**⚠️ Important:** Before flashing, backup the original firmware so you can restore it if needed.

```bash
esptool.py --port /dev/ttyUSB0 read_flash 0x0 0x400000 bidiwifi_backup.bin
```

#### Step 2: Connect for Flashing

Use the test points on the BiDi-WiFi board:

![BiDi-WiFi Pinout](img/bidiwifi-pinout.jpg)

| Test Point | Connect To |
|------------|------------|
| Tx         | USB-TTL RX |
| Rx         | USB-TTL TX |
| IO0        | GND (hold during reset to enter flash mode) |
| EN         | 3.3V |
| +3V3       | 3.3V |
| GND        | Ground |

#### Step 3: Create Configuration

Create a `gate.yaml` file:

```yaml
esphome:
  name: gate
  friendly_name: Gate

esp32:
  board: esp32dev
  framework:
    type: esp-idf
    advanced:
      # BiDi-WiFi uses ESP32 rev3.1 - reduces binary size by excluding legacy silicon workarounds.
      # Safe to use with OTA: incompatible chips will reject the firmware and roll back automatically.
      # Remove or adjust for custom ESP32 hardware with a different chip revision.
      minimum_chip_revision: "3.1"

logger:
  baud_rate: 0  # Disable serial logging (UART used for Bus-T4)

api:
  encryption:
    key: !secret api_encryption_key

ota:
  - platform: esphome
    password: !secret ota_password

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  ap:
    ssid: "Gate Fallback"
    password: !secret fallback_password

external_components:
  - source:
      type: git
      url: https://github.com/makstech/esphome-BusT4
    components: [bus_t4, bus_t4_control_unit, bus_t4_oxi]

uart:
  tx_pin: GPIO21
  rx_pin: GPIO18
  baud_rate: 19200

# Layered model: uart -> bus_t4 (transport) -> device(s) on the bus -> entities
bus_t4:
  id: bus

bus_t4_control_unit:
  bus_t4_id: bus
  cover:
    id: gate    # name defaults to "" -> inherits the device/node name (here, "Gate")
```

> **Note**: The `minimum_chip_revision: "3.1"` setting is specific to the Nice BiDi-WiFi module (ESP32 rev3.1). It reduces binary size by excluding legacy workaround code. Safe to use with OTA updates — if the chip revision doesn't match, ESPHome will reject the firmware and automatically roll back. Remove or adjust for custom ESP32 hardware. See [ESPHome ESP32 advanced configuration](https://esphome.io/components/esp32/#advanced-configuration) for details.

#### Step 4: Flash and Connect

```bash
esphome run gate.yaml
```

## Configuration

### Full Example

```yaml
esphome:
  name: gate
  friendly_name: "Nice BusT4"   # the bridge/node device
  # Optional: group entities into Home Assistant sub-devices. An entity with an
  # empty name inherits its device's name, so the cover shows as just "Gate".
  devices:
    - id: cu_dev
      name: "Gate"
    - id: oxi_dev
      name: "OXI receiver"

external_components:
  - source:
      type: git
      url: https://github.com/makstech/esphome-BusT4
    components: [bus_t4, bus_t4_control_unit, bus_t4_oxi]

uart:
  tx_pin: GPIO21
  rx_pin: GPIO18
  baud_rate: 19200

# The bus is the transport. Devices on the bus declare entities.
bus_t4:
  id: bus
  address: 0x5090   # Optional: this ESP module's own bus address
  diagnostics: true # Optional: bus link-health sensors (errors / timeouts)

bus_t4_control_unit:
  bus_t4_id: bus
  device: cu_dev    # group this block's entities under the "Gate" device
  cover:
    id: gate                      # name defaults to "" -> inherits the device name ("Gate")
    auto_learn_timing: true       # Auto-learn open/close duration
    open_duration: 20s            # Initial/fallback open time
    close_duration: 20s           # Initial/fallback close time
    position_report_interval: 1s  # Position update rate
    force_estimated_position: false # Debug: ignore the encoder, always use the time estimate
  # Config flags: bare type, or a map with name/icon overrides
  flags:
    - auto_close
    - photo_close
    - type: surge
      name: "Starting torque"
      icon: "mdi:flash"
  numbers: [pause_time, speed_opening, speed_closing]
  diagnostics: true   # controller identity: firmware / product / hardware / description

# Optional: the OXI plug-in radio receiver as its own device
bus_t4_oxi:
  bus_t4_id: bus
  device: oxi_dev

# Optional: additional control buttons (grouped under the gate device).
# Protocol enums live in the bus_t4 namespace, so qualify them in lambdas.
button:
  - platform: template
    name: "Partial Open"
    device_id: cu_dev
    icon: "mdi:gate-arrow-right"
    on_press:
      - lambda: id(gate).send_cmd(bus_t4::CMD_OPEN_PARTIAL_1);

  - platform: template
    name: "Step-by-Step"
    device_id: cu_dev
    icon: "mdi:gate"
    on_press:
      - lambda: id(gate).send_cmd(bus_t4::CMD_STEP);
```

### Configuration Variables

The component is layered like `uart → modbus → modbus_controller`: a `bus_t4`
transport, with one or more devices declared on it.

#### bus_t4 (transport)

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `id` | id | — | Referenced by the device components via `bus_t4_id` |
| `address` | hex | `0x5090` | This ESP module's own bus address (the `from` field) |
| `diagnostics` | bool/map | — | `true` adds bus link-health sensors (`Errors`, `Timeouts`); a map renames them |

#### bus_t4_control_unit (device)

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `bus_t4_id` | id | *auto* | The `bus_t4` to attach to |
| `device` | id | — | A device declared in `esphome: devices:` — all of this block's entities are grouped under it as one HA device (set once, no per-entity `device_id`) |
| `address` | hex | *auto* | Controller address override; auto-detected via `INF_WHO` when omitted |
| `cover` | block | — | The gate cover (options below) |
| `flags` | list | — | Config-flag `switch`es (types below) |
| `numbers` | list | — | Numeric param `number`s (types below) |
| `selects` | list | — | Enumerated param `select`s (types below) |
| `sensors` | list | — | Read-only `sensor`s polled from the controller (types below) |
| `buttons` | list | — | Action `button`s that write a fixed value (types below) |
| `diagnostics` | bool/map | — | `true` adds the controller's firmware/product/hardware/description text sensors (or a map to rename them) |

Every `flags`/`numbers`/`selects`/`sensors`/`buttons` entry is a bare type name **or** a map with per-entity overrides (`name`, `icon`, `id`, …). Settable entities send a SET on change and also track GET/SET replies on the bus, so changes made elsewhere (Oview, another client) are reflected immediately. A param the controller doesn't support shows as unavailable.

`cover` options: `name` (defaults to `""`, which makes the cover inherit its device name), `auto_learn_timing` (`true`), `open_duration` (`20s`), `close_duration` (`20s`), `position_report_interval` (`1s`), `force_estimated_position` (`false`; debug — ignore the encoder and always report the time-based estimate, at `position_report_interval` cadence).

#### bus_t4_oxi (device)

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `bus_t4_id` | id | *auto* | The `bus_t4` to attach to |
| `device` | id | — | Group the OXI sensors under this `esphome: devices:` device |
| `product` / `hardware` / `firmware` | string/map | *default names* | OXI radio-receiver identity text sensors |

```yaml
bus_t4:
  id: bus
bus_t4_control_unit:
  bus_t4_id: bus
  flags: [photo_close, auto_close]
  numbers: [pause_time, standby_time]
  selects: [step_by_step, standby_mode]
  sensors: [total_maneuvers]
  buttons: [reset_maintenance]
```

##### `flags` types (`switch`)

<!-- BEGIN SWITCH_TYPES -->
| `type` | Description |
|---|---|
| `auto_close` | Auto-close |
| `photo_close` | Close after photo |
| `always_close` | Always-close |
| `standby` | Stand-by |
| `surge` | Surge |
| `pre_flash` | Pre-flashing |
| `disable_internal_radio` | Disable internal radio |
| `slave` | Slave mode |
| `automation_lock` | Automation lock |
| `keylock` | Keylock |
| `decelerations` | Decelerations |
<!-- END SWITCH_TYPES -->

##### `numbers` types (`number`)

<!-- BEGIN NUMBER_TYPES -->
| `type` | Description | Range |
|---|---|---|
| `pause_time` | Auto-close pause time | 0–240 s |
| `speed_opening` | Speed (opening) | 25–100 % |
| `speed_closing` | Speed (closing) | 25–100 % |
| `force_opening` | Force (opening) | 0–100 % |
| `force_closing` | Force (closing) | 0–100 % |
| `decel_speed_opening` | Deceleration speed (opening) | 25–50 % |
| `decel_speed_closing` | Deceleration speed (closing) | 25–50 % |
| `decel_force_opening` | Deceleration force (opening) | 0–100 % |
| `decel_force_closing` | Deceleration force (closing) | 0–100 % |
| `decel_sensitivity_opening` | Deceleration sensitivity (opening) | 0–100 % |
| `decel_sensitivity_closing` | Deceleration sensitivity (closing) | 0–100 % |
| `maintenance_threshold` | Maintenance threshold | 100–20000 |
| `photo_close_time` | Close after photo time | 0–250 s |
| `always_close_time` | Always-close time | 0–20 s |
| `standby_time` | Stand-by time | 5–250 s |
| `surge_time` | Surge time | 1–10 s |
| `pre_flash_open_time` | Pre-flash time (opening) | 1–10 s |
| `pre_flash_close_time` | Pre-flash time (closing) | 1–10 s |
| `max_work_time` | Maximum work time | 10–250 s |
| `courtesy_light_time` | Courtesy light time | 0–240 s |
| `electric_lock_time` | Electric lock time | 0.1–10 s |
| `suction_cup_time` | Suction cup time | 0.1–10 s |
| `brief_reversal` | Brief reversal | 0.5–5 s |
<!-- END NUMBER_TYPES -->

##### `selects` types (`select`)

<!-- BEGIN SELECT_TYPES -->
| `type` | Description |
|---|---|
| `step_by_step` | Step-by-Step mode |
| `output_1` | Output 1 function |
| `output_2` | Output 2 function |
| `output_3` | Output 3 function |
| `output_4` | Output 4 function |
| `output_5` | Output 5 function |
| `output_6` | Output 6 function |
| `maintenance_management` | Maintenance management |
| `photo_close_mode` | Close after photo mode |
| `always_close_mode` | Always-close mode |
| `standby_mode` | Stand-by mode |
<!-- END SELECT_TYPES -->

##### `sensors` types (`sensor`, read-only)

<!-- BEGIN SENSOR_TYPES -->
| `type` | Description |
|---|---|
| `maintenance_count` | Maintenance counter |
| `total_maneuvers` | Total maneuvers |
<!-- END SENSOR_TYPES -->

##### `buttons` types (`button`, momentary)

<!-- BEGIN BUTTON_TYPES -->
| `type` | Description |
|---|---|
| `reset_maintenance` | Reset maintenance counter |
<!-- END BUTTON_TYPES -->

### Available Commands

Use in lambdas with `id(gate).send_cmd(bus_t4::COMMAND)` (the protocol enums live in the `bus_t4` namespace):

| Command | Description |
|---------|-------------|
| `CMD_OPEN` | Open gate |
| `CMD_CLOSE` | Close gate |
| `CMD_STOP` | Stop movement |
| `CMD_STEP` | Step-by-step (toggle) |
| `CMD_OPEN_PARTIAL_1` | Partial open position 1 |
| `CMD_OPEN_PARTIAL_2` | Partial open position 2 |
| `CMD_OPEN_PARTIAL_3` | Partial open position 3 |

### Security Commands (Lock/Block)

Security commands require the IT4WIFI device identity. Use `send_cmd(bus_t4::COMMAND, bus_t4::IT4WIFI)`:

| Command | Description |
|---------|-------------|
| `CMD_BLOCK` | Lock the motor |
| `CMD_RELEASE` | Unlock the motor |
| `CMD_OPEN_AND_BLOCK` | Open gate, then lock |
| `CMD_CLOSE_AND_BLOCK` | Close gate, then lock |
| `CMD_RELEASE_AND_OPEN` | Unlock, then open |
| `CMD_RELEASE_AND_CLOSE` | Unlock, then close |

Example lock entity:

```yaml
lock:
  - platform: template
    name: "Gate Lock"
    optimistic: true
    on_lock:
      - lambda: 'id(gate).send_cmd(bus_t4::CMD_BLOCK, bus_t4::IT4WIFI);'
    on_unlock:
      - lambda: 'id(gate).send_cmd(bus_t4::CMD_RELEASE, bus_t4::IT4WIFI);'
```

### Raw Configuration Parameters

`send_config_set()` writes any controller parameter by its raw hex address — it's
the same call the `number` and `switch` platforms make under the hood. A good way
to get a feel for it is to drive a parameter you already have as a platform type,
e.g. opening/closing speed (`0x42`/`0x43`, the params the `speed_opening` /
`speed_closing` numbers wrap), and watch the built-in number track your changes:

```yaml
number:
  - platform: template
    name: "Opening speed (raw)"   # 0x42 — same parameter as the speed_opening number
    min_value: 25
    max_value: 100
    set_action:
      - lambda: 'id(gate).send_config_set(0x42, (uint8_t)x);'

  - platform: template
    name: "Closing speed (raw)"   # 0x43
    min_value: 25
    max_value: 100
    set_action:
      - lambda: 'id(gate).send_config_set(0x43, (uint8_t)x);'
```

Multi-byte parameters take a width: `send_config_set(param, (uint16_t)x, 2)`. Find
addresses with the `debug_request` action (below) or gashtaan's parameter table.

### Sending Commands for Debugging

Two component methods let you push commands onto the bus at runtime (no reflash). Each sends, waits for the reply, and **returns it as a hex string** (or `"no reply"` on timeout, `""` on bad input); the reply is also logged (`[debug] RX: ...`):

- `id(bus)->debug_request(message)` — a DMP request to the control unit; supply only the message bytes and the header, `55`/size framing and both checksums are added for you. Returns as soon as the reply with the matching command byte arrives.
- `id(bus)->debug_request_raw(frame)` — sends a byte-exact frame (you supply the whole thing including framing and checksums) and returns on the first non-echo reply.

These are thin hex wrappers over the component's send primitives: `dep_send()` / `dmp_send()` (fire-and-forget) and `dmp_request()` (waits for the reply) — the same primitives `send_cmd()` and the switch platform use.

Both accept hex in any format (`"04 D1 99"`, `"04.D1.99"` or `"04D199"` — non-hex characters are stripped). They block the main loop only until the reply lands (typically ~150 ms), or until `timeout_ms` (default 500) on no reply.

The cleanest way to call them at runtime is via Home Assistant actions. Wrap a send in `api.respond` to return the reply to the caller:

```yaml
api:
  actions:
    # Each returns {"reply": "<hex>"} to the caller.
    - action: debug_request
      variables:
        message: string
      then:
        - api.respond:
            data: !lambda 'root["reply"] = id(bus)->debug_request(message);'
    - action: debug_request_raw
      variables:
        frame: string
      then:
        - api.respond:
            data: !lambda 'root["reply"] = id(bus)->debug_request_raw(frame);'
```

Call it from Developer Tools → Actions with "return response":

```yaml
action: esphome.gate_debug_request_raw
data:
  frame: "55.0D.00.03.00.81.08.06.8C.04.08.89.00.00.85.0D"
# response -> {"reply": "00.81.00.03..."}
```

> **Note**: `api.respond` needs ESPHome 2025.12+; `supports_response` is auto-detected (`optional`) when the action returns data.

## How Position Tracking Works

This component uses **multiple strategies** for accurate position tracking:

### 1. Encoder Position (Primary when available)

For devices that support it, the component polls encoder position during movement every 500ms. This provides the most accurate position tracking.

- Encoder data is prioritized when available (updated within last 2 seconds)
- Automatically detected - no configuration needed

> **Note**: Robus devices don't support position queries during movement

### 2. Time-Based Estimation (Fallback)

1. **Auto-Learning**: When the gate performs a complete movement (fully closed → fully open or vice versa), the duration is measured and saved
2. **Position Calculation**: During movement, position is calculated based on elapsed time
3. **Persistence**: Learned durations are stored in flash and survive reboots
4. **Adaptive**: If timing deviates >10% from stored value, it's automatically updated
5. **Smart Fallback**: Only used when encoder data is unavailable or stale

### 3. Limit Switch Confirmation

When the gate stops, the component queries the controller's I/O state (INF_IO) to confirm if a limit switch is active. This ensures accurate detection of fully open/closed states even if the time-based estimate is slightly off.

### 4. Periodic Status Refresh

Every 15 seconds, the component requests a status update from the controller. This helps:
- Recover from missed packets
- Keep state synchronized
- Detect external changes (e.g., remote control operation)

### Learning Requirements

- Only learns from **complete** movements (end-to-end)
- Duration must be between 3 seconds and 5 minutes
- Interrupted movements don't update learned values

## Device Detection

During initialization, the component queries the controller for product information and automatically enables device-specific handling:

| Product | Mode | Special Handling |
|---------|------|------------------|
| WLA1 (Walky) | `is_walky` | Uses 1-byte position values |
| Road 400 | Standard | Alternate status codes (0x83/0x84) |

The control unit's identity (manufacturer, product, hardware, firmware, description) is read at startup, logged, and exposed via the `diagnostics` text sensors.

## Troubleshooting

### Position not updating

1. Wait for a complete open/close cycle for auto-learning
2. Check logs for "Learned new open/close duration" messages
3. Manually set `open_duration` and `close_duration` if auto-learning fails

### BiDi-WiFi won't enter flash mode

1. Hold IO0 to GND
2. Briefly disconnect EN from +3V3 (reset)
3. Release IO0
4. Start flashing immediately

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Submit a pull request

## Credits

- Original Bus-T4 work by [@pruwait](https://github.com/pruwait/Nice_BusT4)
- BiDi-WiFi firmware by [@gashtaan](https://github.com/gashtaan/nice-bidiwifi-firmware)
- Initial ESPHome ESP32 PoC by [@andrein](https://github.com/andrein/esphome-BusT4)

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.

## Related Resources

- [Home Assistant Community Discussion](https://community.home-assistant.io/t/nice-app-with-bidi-wifi-gate-automation/606241)
- [Nice BiDi-WiFi Product Page](https://www.niceforyou.com/uk/nicepost/bidi-wifi-new-pocket-programming-interface)
- [ESPHome Documentation](https://esphome.io/)
  
### Schematics

- [BiDi-WiFi schematic](https://github.com/gashtaan/nice-bidiwifi-firmware/blob/e6bc474c782ba9cd9ef3ada83a24e4970b281ae0/schematics/bidiwifi.pdf)
- [Pinout for 10-pin Bus4T](https://github.com/xdanik/Nice_BusT4/blob/9faa86262692da99e75979662b4f4ac555746ebf/img/connector.jpg)

### Nice Resources

- [TTPCI](https://www.niceforyou.com/sites/default/files/upload/manuals/IS0326A00MM.pdf)
- [DMBM](https://www.niceforyou.com/sites/default/files/upload/manuals/nice_dmbm_integration_protocol.pdf)
