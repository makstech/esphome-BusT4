Nice Bus-T4 ESPHome component

ESPHome component for controlling Nice gate motors via the Bus-T4 protocol using a CAN transceiver.

## Features

- Control Nice gates (open/close/stop) via Home Assistant
- Time-based position tracking for gates without encoder feedback
- **Auto-learning of open/close timing** - no manual measurement needed
- Learned timing values persist across reboots (stored in flash)
- Automatic device discovery on the bus
- Compatible with ESPHome 2025.12.5+

## Hardware Setup

This component requires a CAN transceiver (e.g., SN65HVD230) connected to the Nice gate controller's Bus-T4 port.

## Configuration

```yaml
logger:
  baud_rate: 0

external_components:
  - source:
      type: git
      url: https://github.com/makstech/esphome-BusT4
    components: [ bus_t4 ]
    refresh: 0s

uart:
  - id: t4_uart
    tx_pin: GPIO21
    rx_pin: GPIO18
    baud_rate: 19200
    debug:

bus_t4:
  id: bus

cover:
  - platform: bus_t4
    name: "Gate"
    id: gate
    # Auto-learn timing (default: true)
    # Learns open/close duration by observing complete cycles
    auto_learn_timing: true
    # Initial/fallback values (will be overwritten when auto-learning completes)
    open_duration: 20s
    close_duration: 20s
```

### Configuration Variables

- **name** (*Required*, string): The name for this cover.
- **auto_learn_timing** (*Optional*, boolean): Automatically learn open/close durations by observing complete movement cycles. Learned values are saved to flash. Default: `true`
- **open_duration** (*Optional*, time): Initial/fallback time for gate to fully open. Will be overwritten by auto-learning. Default: 20s
- **close_duration** (*Optional*, time): Initial/fallback time for gate to fully close. Will be overwritten by auto-learning. Default: 20s

## Position Tracking

Since many Nice controllers don't report encoder positions over the bus, this component uses time-based position estimation.

### Auto-Learning (Recommended)

With `auto_learn_timing: true` (default), the component automatically learns your gate's timing:

1. **Learning trigger**: When the gate performs a complete movement (fully closed → fully open, or vice versa), the duration is measured
2. **Persistence**: Learned values are stored in flash and survive reboots
3. **Adaptive**: If the timing deviates by more than 10% from the stored value, the new timing is saved
4. **Validation**: Only durations between 3 seconds and 5 minutes are accepted

Simply operate your gate normally and the timing will be learned automatically!

### Manual Configuration

If you prefer to set timing manually, disable auto-learning:

```yaml
cover:
  - platform: bus_t4
    name: "Gate"
    auto_learn_timing: false
    open_duration: 18s   # Measure with a stopwatch
    close_duration: 17s
```

## Nice Resources

* [TTPCI Protocol](https://www.niceforyou.com/sites/default/files/upload/manuals/IS0326A00MM.pdf)
* [DMBM Integration Protocol](https://www.niceforyou.com/sites/default/files/upload/manuals/nice_dmbm_integration_protocol.pdf)

## Related Projects

* https://github.com/gashtaan/nice-bidiwifi-firmware
* https://github.com/pruwait/Nice_BusT4
