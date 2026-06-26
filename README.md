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
- ⚙️ **Motor configuration** - Control auto-close, standby, peak mode, and more via SET commands
- 🔧 **Wide compatibility** - Device-specific handling for Walky, Robus, Road 400, and more
- 📡 **OXI receiver logging** - Remote control button presses are logged for debugging
- ⌨️ **Keypad / command events** - `event` entity fires on every command on the bus (open, close, step, partial, stop)
- 🚦 **Photocell state** *(experimental)* - per-photocell `binary_sensor` for beam blocked/clear
- 🔍 **Packet capture logger** - `debug_unknown_packets` logs/classifies bus traffic to map new events

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
- **Robus gates**: Position queries disabled during movement (auto-detected)
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
    components: [bus_t4]

uart:
  tx_pin: GPIO21
  rx_pin: GPIO18
  baud_rate: 19200

bus_t4:
  id: bus

cover:
  - platform: bus_t4
    name: "Gate"
    id: gate
```

> **Note**: The `minimum_chip_revision: "3.1"` setting is specific to the Nice BiDi-WiFi module (ESP32 rev3.1). It reduces binary size by excluding legacy workaround code. Safe to use with OTA updates — if the chip revision doesn't match, ESPHome will reject the firmware and automatically roll back. Remove or adjust for custom ESP32 hardware. See [ESPHome ESP32 advanced configuration](https://esphome.io/components/esp32/#advanced-configuration) for details.

#### Step 4: Flash and Connect

```bash
esphome run gate.yaml
```

## Configuration

### Full Example

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/makstech/esphome-BusT4
    components: [bus_t4]

uart:
  tx_pin: GPIO21
  rx_pin: GPIO18
  baud_rate: 19200

bus_t4:
  id: bus
  address: 0x5090  # Optional: custom device address

cover:
  - platform: bus_t4
    name: "Gate"
    id: gate
    auto_learn_timing: true       # Auto-learn open/close duration
    open_duration: 20s            # Initial/fallback open time
    close_duration: 20s           # Initial/fallback close time
    position_report_interval: 1s  # Position update rate

# Optional: Additional control buttons
button:
  - platform: template
    name: "Partial Open"
    icon: "mdi:gate-arrow-right"
    on_press:
      - lambda: id(gate).send_cmd(CMD_OPEN_PARTIAL_1);

  - platform: template
    name: "Step-by-Step"
    icon: "mdi:gate"
    on_press:
      - lambda: id(gate).send_cmd(CMD_STEP);
```

### Configuration Variables

#### bus_t4 Component

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `address` | hex | `0x5090` | Device address on the bus |

#### Cover Platform

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `name` | string | *Required* | Name for Home Assistant |
| `auto_learn_timing` | boolean | `true` | Auto-learn open/close duration |
| `open_duration` | time | `20s` | Initial/fallback time to fully open |
| `close_duration` | time | `20s` | Initial/fallback time to fully close |
| `position_report_interval` | time | `1s` | How often to update position during movement |

### Available Commands

Use in lambdas with `id(gate).send_cmd(COMMAND)`:

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

Security commands require the IT4WIFI device identity. Use `send_cmd(COMMAND, IT4WIFI)`:

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
      - lambda: 'id(gate).send_cmd(CMD_BLOCK, IT4WIFI);'
    on_unlock:
      - lambda: 'id(gate).send_cmd(CMD_RELEASE, IT4WIFI);'
```

### Motor Controller Configuration

You can change motor controller settings via lambdas. These send SET commands to the controller:

| Method | Description |
|--------|-------------|
| `set_auto_close(bool)` | L1 - Enable/disable auto-close after opening |
| `set_photo_close(bool)` | L2 - Close after photo sensor clears |
| `set_always_close(bool)` | L3 - Always close (ignore hold-open) |
| `set_standby(bool)` | Enable/disable standby mode (power saving) |
| `set_peak_mode(bool)` | Enable/disable peak mode (faster operation) |
| `set_pre_flash(bool)` | Enable/disable pre-flash warning light |

Example usage with buttons:

```yaml
button:
  - platform: template
    name: "Enable Auto-Close"
    on_press:
      - lambda: id(gate).set_auto_close(true);

  - platform: template
    name: "Disable Auto-Close"
    on_press:
      - lambda: id(gate).set_auto_close(false);
```

Example usage with switches:

```yaml
switch:
  - platform: template
    name: "Auto-Close"
    icon: "mdi:timer"
    optimistic: true
    turn_on_action:
      - lambda: id(gate).set_auto_close(true);
    turn_off_action:
      - lambda: id(gate).set_auto_close(false);

  - platform: template
    name: "Pre-Flash Warning"
    icon: "mdi:alarm-light"
    optimistic: true
    turn_on_action:
      - lambda: id(gate).set_pre_flash(true);
    turn_off_action:
      - lambda: id(gate).set_pre_flash(false);
```

### Raw Configuration Parameters

In addition to the named methods (`set_auto_close`, `set_standby`, etc.), you can set any controller parameter by its raw hex address using `send_config_set()`:

```yaml
number:
  - platform: template
    name: "Motor Force"
    min_value: 0
    max_value: 100
    step: 5
    set_action:
      - lambda: 'id(gate).send_config_set(0x92, (uint8_t)x);'

  - platform: template
    name: "Pause Duration"
    min_value: 0
    max_value: 250
    unit_of_measurement: "s"
    set_action:
      - lambda: 'id(gate).send_config_set(0x88, (uint8_t)x);'
```

### Raw Command for Debugging

For debugging and testing, you can send raw hex commands directly to the bus using `send_raw_cmd()`. This accepts hex strings in formats like `"55.0C.00.FF..."` or `"550C00FF..."` (dots/spaces are automatically stripped).

The easiest way to use this is with an ESPHome Text component:

```yaml
text:
  - platform: template
    name: "Raw Command"
    id: raw_command
    optimistic: true
    mode: text
    on_value:
      then:
        - lambda: |-
            if (!x.empty()) {
              id(gate).send_raw_cmd(x);
            }
```

This creates a text input in Home Assistant where you can paste hex commands. The command is sent immediately when you submit the text.

> **Note**: The text component approach is recommended because ESPHome's API service string variables have [known issues](https://github.com/esphome/issues/issues/3564) with the ESP-IDF framework.

## BlueBus events

In addition to gate control, the component can surface events that the Nice controller
forwards onto the Bus-T4 wire from its BlueBus accessories (photocells and wired keypads).
These are **additive** — they don't affect the `cover` or any existing entity.

> **What this can and cannot see.** The ESP32 is a passive consumer of Bus-T4 traffic. It
> never touches the BlueBus 2‑wire bus directly (a second master there would break the gate).
> It only parses what the controller already broadcasts / answers on Bus-T4. As a result the
> **source** of a command (which keypad, which remote) is *not* available — the protocol does
> not carry it — and some BlueBus details are controller-specific (see the photocell caveat).

### Keypad / command events (`event`)

Fires an [event entity](https://www.home-assistant.io/integrations/event/) every time the
control unit reports an executed command on the bus:

```yaml
event:
  - platform: bus_t4
    name: "Gate keypad"
    device_class: button   # optional
```

Event types emitted: `open`, `close`, `step_by_step`, `partial_1`, `partial_2`,
`partial_3`, `stop`.

Caveats:
- The payload is **which command** was issued, not **who** issued it. A wired keypad, an OXI
  remote, a hardwired input and a command sent from Home Assistant all look identical.
- For the same reason it may also fire for commands the ESP itself sends (e.g. opening the
  cover from HA). If that turns out to be noisy on your controller, capture it (below) and
  open an issue — echo suppression can be added once confirmed.

### Photocells (`binary_sensor`) — EXPERIMENTAL

Exposes a BlueBus photocell beam as a binary sensor (ON = beam blocked / obstacle):

```yaml
binary_sensor:
  - platform: bus_t4
    address: 1            # photocell index: 1 = PHOTO, 2 = PHOTO2, 3 = PHOTO3
    name: "Photocell gate"
    device_class: safety  # optional
```

> ⚠️ **Experimental.** Photocell state is read from the controller's "BlueBus diagnostics"
> response (`0xD0`), which the component polls. That payload's byte layout is **undocumented
> and differs between controller models**, so the decode is a best guess. The raw payload is
> logged at `DEBUG` so it can be verified against your hardware. Treat the state as unverified
> until you've confirmed it with a capture (see below). Everything else (the polling, entity,
> address selection) works; only the one decode constant needs your bytes to be pinned down.

### Bus device list (`text_sensor`, diagnostic)

A comma-separated list of the Bus-T4 endpoints seen on the bus (e.g.
`0x00.03 controller, 0x0A.6D radio`). Handy to confirm the component is actually talking to
your controller and receiver:

```yaml
text_sensor:
  - platform: bus_t4
    name: "Bus devices"
```

(This lists Bus-T4 endpoints. Individual BlueBus accessories live behind the controller and
are only enumerable via the `0xD0` diagnostics, which isn't fully mapped yet.)

### Capturing packets (help map the rest)

The photocell layout and a few other details aren't documented for every controller. You can
help map them with the built-in logger:

1. Flash [`debug_yaml`](debug_yaml) (or add `debug_unknown_packets: true` under `bus_t4:`).
2. `esphome logs debug_yaml` — the UART carries the bus, so logs arrive over WiFi.
3. Do **one** thing at a time and note the time: block/clear a photocell beam; press each
   keypad button.
4. Each received packet is logged as `[pkt] <type guess> ... <hex>`. Copy the lines around
   each action, say what you physically did, and share them (e.g. a GitHub issue).

The byte layouts and confidence levels are tracked in
[`.agent/PROTOCOL.md`](.agent/PROTOCOL.md).

### Robus note

On Robus controllers the cover already disables position queries during movement. The BlueBus
event features above don't change that and add only lightweight, read-only polling.

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
| ROBUSHSR10 | `is_robus` | No position queries during movement |
| Road 400 | Standard | Alternate status codes (0x83/0x84) |

Device information (manufacturer, product, firmware) is logged at startup.

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
